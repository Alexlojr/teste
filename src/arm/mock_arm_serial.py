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
        if linha:
            if self._serial_real is not None:
                try:
                    self._serial_real.write((linha + "\n").encode())
                except Exception:
                    pass
            for resposta in self._processa_comando(linha):
                self._rx.append((resposta + "\n").encode())
        return len(data)

    @property
    def in_waiting(self) -> int:
        return len(self._rx)

    def readline(self) -> bytes:
        if self._rx:
            return self._rx.popleft()
        return b""

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

        for resposta in respostas:
            self._registra("RX", resposta)
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
