"""
Mock de comunicação serial com um braço robótico com servos.

Simula, em memória, um controlador de braço (tipo firmware Arduino/GRBL)
falando um protocolo de texto por linha (comandos terminados em '\\n').
A classe expõe a mesma cara de uso de um serial.Serial de verdade
(write / readline / in_waiting / close), então o dashboard (ou qualquer
outro código) que usa MockArmSerial pode ser trocado depois por uma
pyserial.Serial real sem precisar mudar a lógica de cima.

Protocolo:
    SET <id> <angulo>   -> "OK SET <id> <angulo>"
    GET <id>            -> "POS <id> <angulo>"
    GET ALL             -> "POS ALL 0:<a0> 1:<a1> ..."
    HOME                -> manda todos os servos para 90 graus
    <comando inválido>  -> "ERR ..."

Se um `porta_arduino` for passada, cada comando SET/HOME processado aqui
também é encaminhado por essa porta serial de verdade para o Arduino Nano
(firmware em firmware/braco_firmware/), que fala o mesmo protocolo e move
os servos de verdade via PCA9685. O estado mostrado no dashboard continua
sendo a simulação local (não há sensor de posição real no braço), mas o
comando físico sai de verdade pela serial.

Rode este arquivo diretamente para abrir um terminal interativo e
testar os comandos manualmente:

    python mock_arm_serial.py
"""

import random
import threading
import time
from collections import deque
from dataclasses import dataclass

import serial
from serial.tools import list_ports

SERVOS = {
    0: "base",
    1: "ombro",
    2: "cotovelo",
    3: "garra",
}


def detectar_porta_arduino():
    """Procura uma porta serial USB plugada na Raspberry Pi (Arduino Nano)."""
    for info in list_ports.comports():
        if "ttyUSB" in info.device or "ttyACM" in info.device:
            return info.device
    return None

ANGULO_MIN = 0
ANGULO_MAX = 180
VELOCIDADE_GRAUS_S = 90.0  # velocidade angular simulada do servo
RUIDO_LEITURA = 0.4        # ruído simulando a leitura do potenciômetro/encoder


@dataclass
class EstadoServo:
    atual: float = 90.0
    alvo: float = 90.0

    @property
    def em_movimento(self) -> bool:
        return abs(self.alvo - self.atual) > 0.5


class MockArmSerial:
    def __init__(self, port="MOCK_ARM", baudrate=115200, servos=None, porta_arduino=None):
        self.baudrate = baudrate
        self.is_open = True

        self._serial_real = None
        if porta_arduino:
            try:
                self._serial_real = serial.Serial(porta_arduino, baudrate, timeout=0.2)
                time.sleep(2)  # Nano reinicia ao abrir a serial; espera o bootloader/sketch subir
                # O firmware imprime "READY braco_firmware" ao subir. Se ficar
                # na fila, e ele que a primeira pergunta le como resposta.
                self._serial_real.reset_input_buffer()
            except Exception:
                self._serial_real = None

        self.braco_real = self._serial_real is not None
        self.port = porta_arduino if self.braco_real else port

        ids = servos if servos is not None else SERVOS
        self._servos = {i: EstadoServo() for i in ids}
        self._lock = threading.Lock()
        self._rx = deque()
        self._historico = deque(maxlen=200)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop_fisica, daemon=True)
        self._thread.start()

    # ---- API no estilo pyserial ---------------------------------------

    def write(self, data: bytes) -> int:
        linha = data.decode("utf-8", errors="ignore").strip()
        if not linha:
            return len(data)

        # A simulação roda sempre: com Arduino real ela não é a verdade, mas
        # serve de estado de exibição caso a porta caia no meio da operação.
        respostas = self._processa_comando(linha)

        if self._serial_real is not None:
            try:
                self._serial_real.write((linha + "\n").encode())
            except Exception:
                pass
            # Não enfileira a resposta simulada: com hardware presente, quem
            # responde é o firmware. Enfileirar as duas faria a fila crescer
            # sem fim e mascararia a resposta real.
        else:
            for resposta in respostas:
                self._registra("RX", resposta)
                self._rx.append((resposta + "\n").encode())

        return len(data)

    @property
    def in_waiting(self) -> int:
        if self._serial_real is not None:
            try:
                return self._serial_real.in_waiting
            except Exception:
                return 0
        return len(self._rx)

    def readline(self) -> bytes:
        """
        Uma linha de resposta.

        Com Arduino conectado devolve a resposta DELE, não a da simulação.
        Antes este método devolvia sempre a simulada mesmo com hardware
        presente, então um `ERR ...` do firmware nunca chegava a aparecer em
        lugar nenhum — o erro acontecia e ninguém ficava sabendo.
        """
        if self._serial_real is not None:
            try:
                bruto = self._serial_real.readline()
            except Exception:
                return b""
            if bruto:
                self._registra("RX", bruto.decode("utf-8", errors="ignore").strip())
            return bruto

        if self._rx:
            return self._rx.popleft()
        return b""

    # ---- diálogo com o firmware -----------------------------------------

    def _pergunta(self, comando: str, prefixo: str, timeout_s: float = 1.0):
        """
        Manda um comando e espera a resposta que começa com `prefixo`.

        Linhas que vierem antes (respostas atrasadas de SETs anteriores) são
        registradas no histórico e descartadas — é assim que um `ERR` do
        firmware fica visível no log em vez de ser engolido silenciosamente.
        """
        if self._serial_real is None:
            return None
        try:
            self._serial_real.write((comando + "\n").encode())
        except Exception:
            return None
        self._registra("TX", comando)

        limite = time.monotonic() + timeout_s
        while time.monotonic() < limite:
            try:
                linha = self._serial_real.readline().decode("utf-8", errors="ignore").strip()
            except Exception:
                return None
            if not linha:
                continue
            self._registra("RX", linha)
            if linha.upper().startswith(prefixo.upper()):
                return linha
        return None

    def aguardar_parada(self, timeout_s: float = 15.0, intervalo_s: float = 0.05) -> bool:
        """
        Bloqueia até o braço parar de se mexer. True se parou, False no timeout.

        Existe porque o firmware suaviza os movimentos, e a duração passou a
        depender da distância: um SET de 100° leva quase o triplo de um de 30°.
        Qualquer espera de tempo fixo erra nos dois sentidos — curta demais
        manda a garra fechar antes de o braço ter descido, longa demais só
        desperdiça ciclo.
        """
        limite = time.monotonic() + timeout_s

        while time.monotonic() < limite:
            if self._serial_real is not None:
                resposta = self._pergunta("BUSY", "BUSY")
                if resposta is None:
                    return False              # firmware mudo: não fica preso
                if resposta.split()[-1] == "0":
                    return True
            else:
                with self._lock:
                    parado = not any(e.em_movimento for e in self._servos.values())
                if parado:
                    return True
            time.sleep(intervalo_s)

        return False

    def enviar_limites(self, calib) -> int:
        """
        Empurra os limites de junta do braco.json para o firmware, via LIM.

        Sem isto os limites existem em dois lugares — o JSON e os arrays do
        firmware — e dessincronizam em silêncio: alguém corrige o JSON depois
        de recalibrar, esquece de recompilar o sketch, e o firmware continua
        deixando a junta ir a um ângulo que já se sabe que bate.

        Com esta chamada no início da operação, o JSON passa a ser a fonte
        única e os valores gravados no sketch viram só um padrão de segurança
        para quando ninguém empurrou nada.

        Devolve quantas juntas foram aceitas pelo firmware.
        """
        if self._serial_real is None:
            return 0

        aceitas = 0
        for servo in (calib.base, calib.ombro, calib.cotovelo, calib.garra):
            resposta = self._pergunta(
                f"LIM {servo.canal} {servo.minimo:.1f} {servo.maximo:.1f}", "OK LIM"
            )
            if resposta is not None:
                aceitas += 1
        return aceitas

    def read_all(self) -> bytes:
        linhas = list(self._rx)
        self._rx.clear()
        return b"".join(linhas)

    def close(self):
        self._stop.set()
        self.is_open = False
        if self._serial_real is not None:
            self._serial_real.close()

    # ---- estado consultável (usado pelo dashboard) ----------------------

    def snapshot(self):
        with self._lock:
            return {
                sid: {
                    "nome": SERVOS.get(sid, str(sid)),
                    "atual": round(estado.atual, 1),
                    "alvo": estado.alvo,
                    "em_movimento": estado.em_movimento,
                }
                for sid, estado in sorted(self._servos.items())
            }

    def historico(self):
        return list(self._historico)

    # ---- núcleo -----------------------------------------------------------

    def _registra(self, direcao: str, linha: str):
        self._historico.append((time.strftime("%H:%M:%S"), direcao, linha))

    def _processa_comando(self, linha: str):
        self._registra("TX", linha)
        partes = linha.upper().split()
        respostas = []

        if not partes:
            return respostas

        cmd = partes[0]
        try:
            if cmd == "SET" and len(partes) == 3:
                sid, angulo = int(partes[1]), float(partes[2])
                angulo_aplicado = self._set_alvo(sid, angulo)
                respostas.append(f"OK SET {sid} {angulo_aplicado:.1f}")

            elif cmd == "HOME":
                for sid in self._servos:
                    self._set_alvo(sid, 90)
                respostas.append("OK HOME")

            elif cmd == "GET" and len(partes) == 2 and partes[1] == "ALL":
                with self._lock:
                    valores = " ".join(
                        f"{sid}:{estado.atual:.1f}"
                        for sid, estado in sorted(self._servos.items())
                    )
                respostas.append(f"POS ALL {valores}")

            elif cmd == "GET" and len(partes) == 2:
                sid = int(partes[1])
                with self._lock:
                    estado = self._servos[sid]
                    respostas.append(f"POS {sid} {estado.atual:.1f}")

            else:
                respostas.append(f"ERR UNKNOWN {linha}")

        except (KeyError, ValueError, IndexError):
            respostas.append(f"ERR BAD_ARGS {linha}")

        # O RX e registrado por quem ENTREGA a resposta (write, quando nao ha
        # Arduino, ou readline/_pergunta, quando ha). Registrar aqui faria a
        # resposta simulada aparecer no log mesmo com hardware conectado.
        return respostas

    def _set_alvo(self, sid: int, angulo: float) -> float:
        if sid not in self._servos:
            raise KeyError(sid)
        angulo = max(ANGULO_MIN, min(ANGULO_MAX, angulo))
        with self._lock:
            self._servos[sid].alvo = angulo
        return angulo

    def _loop_fisica(self):
        passo_s = 0.05
        while not self._stop.is_set():
            with self._lock:
                for estado in self._servos.values():
                    diferenca = estado.alvo - estado.atual
                    passo_max = VELOCIDADE_GRAUS_S * passo_s
                    if abs(diferenca) <= passo_max:
                        estado.atual = estado.alvo
                    else:
                        estado.atual += passo_max if diferenca > 0 else -passo_max
                    estado.atual += random.uniform(-RUIDO_LEITURA, RUIDO_LEITURA)
                    estado.atual = max(ANGULO_MIN, min(ANGULO_MAX, estado.atual))
            time.sleep(passo_s)


if __name__ == "__main__":
    arm = MockArmSerial()
    print(f"Braço mock conectado em {arm.port} @ {arm.baudrate} baud")
    print("Servos:", ", ".join(f"{i}={nome}" for i, nome in SERVOS.items()))
    print("Comandos: SET <id> <angulo> | GET <id> | GET ALL | HOME | sair\n")

    try:
        while True:
            linha = input(">> ").strip()
            if linha.lower() in ("sair", "exit", "quit"):
                break
            if not linha:
                continue

            arm.write((linha + "\n").encode())
            time.sleep(0.05)
            while arm.in_waiting:
                print("<<", arm.readline().decode().strip())
    except KeyboardInterrupt:
        pass
    finally:
        arm.close()
        print("\nConexão encerrada.")
