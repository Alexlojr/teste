"""
Teste de integração da corrente visão -> braço.

Cobre o acoplamento que os testes unitários não pegam: uma bounding box em
pixels entra, e comandos SET saem pela serial, passando por homografia,
portão de estabilidade, cinemática inversa e sequência de coleta.

Não usa câmera nem YOLO — esses são os dois elos que precisam de hardware e
de peso de modelo. Tudo abaixo deles é exercitado de verdade, com uma
homografia sintética de escala e translação conhecidas.

    python tests/test_pipeline.py
"""

import sys
from pathlib import Path

import numpy as np

BASE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(BASE))

from src.arm.coleta import ControladorColeta, ParametrosColeta  # noqa: E402
from src.arm.kinematics import PoseInalcancavel, carregar_config  # noqa: E402
from src.vision.estabilidade import (  # noqa: E402
    ParametrosEstabilidade,
    PortaoEstabilidade,
)
from src.vision.homografia import Homografia  # noqa: E402

falhas = []


def checa(condicao, descricao):
    if not condicao:
        falhas.append(descricao)


class SerialEspiã:
    """
    Registra os comandos e modela o tempo que o firmware levaria para
    executá-los, para que o teste consiga detectar etapa cortada pela metade.

    `aguardar_parada` é o que o ControladorColeta chama entre etapas; aqui ela
    só adianta o relógio simulado, sem dormir de verdade. Se o controlador
    deixar de chamá-la, `parado` fica False no fim da etapa e o teste acusa.
    """

    VELOCIDADE = 60.0        # °/s, o padrão do firmware
    DURACAO_MIN = 0.08

    def __init__(self):
        self.linhas = []
        self.pos = {0: 90.0, 1: 90.0, 2: 90.0, 3: 90.0}
        self.t = 0.0                  # relógio simulado, em segundos
        self.t_fim_movimento = 0.0    # quando o movimento em curso termina

    @property
    def parado(self) -> bool:
        return self.t >= self.t_fim_movimento - 1e-9

    @property
    def pendente_s(self) -> float:
        return max(0.0, self.t_fim_movimento - self.t)

    def write(self, dados: bytes) -> int:
        linha = dados.decode().strip()
        if not linha:
            return len(dados)
        self.linhas.append(linha)

        if linha.startswith("SET "):
            _, canal, angulo = linha.split()
            canal, angulo = int(canal), float(angulo)
            dur = max(abs(angulo - self.pos[canal]) / self.VELOCIDADE, self.DURACAO_MIN)
            self.pos[canal] = angulo
            # Comandos do mesmo lote compartilham a duração: fica valendo o fim
            # mais distante, como o lote do firmware faz.
            self.t_fim_movimento = max(self.t_fim_movimento, self.t + dur)
        elif linha == "HOME":
            dur = max(
                (max(abs(90.0 - a) for a in self.pos.values()) / self.VELOCIDADE),
                self.DURACAO_MIN,
            )
            self.pos = {c: 90.0 for c in self.pos}
            self.t_fim_movimento = max(self.t_fim_movimento, self.t + dur)

        return len(dados)

    def aguardar_parada(self, timeout_s: float = 15.0) -> bool:
        self.t = max(self.t, self.t_fim_movimento)
        return True


class RelogioFalso:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def avanca(self, s):
        self.t += s


# --- homografia sintética: 4 pontos de uma mesa vista de cima -------------
# Escolhidos para dar uma correspondência simples de conferir: a área de
# 640x480 pixels vira uma janela de 200x150 mm à frente da base do braço.
PIXEIS = [(0, 0), (640, 0), (640, 480), (0, 480)]
MILIMETROS = [(-100, 150), (100, 150), (100, 0), (-100, 0)]

homografia = Homografia.de_pontos(PIXEIS, MILIMETROS)
residual = homografia.meta["erro_residual_mm"]
checa(residual < 0.01, f"homografia sintetica deveria ser exata, residual {residual}")


# --- a conversão de caixa usa a base, não o centro ------------------------
# Uma caixa alta e uma baixa com a MESMA base têm que dar a mesma coordenada.
alta = homografia.caixa_para_mm(300, 100, 340, 400)
baixa = homografia.caixa_para_mm(300, 380, 340, 400)
checa(abs(alta[0] - baixa[0]) < 0.01 and abs(alta[1] - baixa[1]) < 0.01,
      f"caixas com a mesma base deveriam dar a mesma coordenada: {alta} vs {baixa}")

# E uma caixa mais embaixo na imagem tem que estar mais perto da base do braço.
perto = homografia.caixa_para_mm(300, 400, 340, 460)
longe = homografia.caixa_para_mm(300, 40, 340, 100)
checa(perto[1] < longe[1],
      f"caixa mais baixa na imagem deveria ter y menor: {perto[1]} vs {longe[1]}")


# --- corrente completa ----------------------------------------------------
braco, calib, params_dict, _calibrado = carregar_config(BASE / "config" / "braco.json")
serial = SerialEspiã()
controlador = ControladorColeta(
    serial, braco=braco, calib=calib, params=ParametrosColeta(**params_dict)
)

relogio = RelogioFalso()
portao = PortaoEstabilidade(
    homografia,
    ParametrosEstabilidade(janela=5, tolerancia_mm=3.0),
    relogio=relogio,
)

# O alvo é procurado, não fixado: o braco.json muda quando vocês calibrarem,
# e um alvo escrito na mão hoje viraria falha falsa amanhã. Procura o ponto
# alcançável mais central dentro do campo que a homografia cobre.
def procura_alvo():
    bons = [
        (float(x), float(y))
        for x in range(-100, 101, 5)
        for y in range(0, 151, 5)
        if controlador.alcancavel(float(x), float(y))[0]
    ]
    if not bons:
        return None
    cx = sum(b[0] for b in bons) / len(bons)
    cy = sum(b[1] for b in bons) / len(bons)
    return min(bons, key=lambda b: (b[0] - cx) ** 2 + (b[1] - cy) ** 2)


alvo_mm = procura_alvo()

if alvo_mm is None:
    # Não é um bug de código: quer dizer que a área que a câmera enxerga e o
    # envelope do braço não se sobrepõem em ponto nenhum. Mas o sistema
    # inteiro seria inútil assim, então vale gritar.
    falhas.append(
        "NENHUM ponto do campo da camera e alcancavel — a area de coleta e o "
        "alcance do braco nao se sobrepoem. Confira elos_mm e os limites das "
        "juntas com 'python mapa_alcance.py'."
    )
    for f in falhas:
        print(f"  FALHA: {f}")
    raise SystemExit(1)

# mm -> pixel pela inversa da homografia, para o teste falar em mm.
inversa = np.linalg.inv(homografia.matriz)
ponto = inversa @ np.array([alvo_mm[0], alvo_mm[1], 1.0])
u, v = ponto[0] / ponto[2], ponto[1] / ponto[2]
checa(0 <= u <= 640 and 0 <= v <= 480,
      f"o alvo {alvo_mm} caiu fora da imagem: pixel ({u:.0f}, {v:.0f})")

# Cinco detecções com tremor de sub-milímetro: tem que estabilizar.
leitura = None
for i in range(5):
    tremor = [0.0, 0.4, -0.3, 0.2, -0.1][i]
    leitura = portao.observar(u - 20 + tremor, v - 40, u + 20 + tremor, v)
    relogio.avanca(0.16)

checa(leitura.estavel, f"deveria ter estabilizado: {leitura.motivo}")
checa(abs(leitura.x_mm - alvo_mm[0]) < 1.0 and abs(leitura.y_mm - alvo_mm[1]) < 1.0,
      f"coordenada deveria bater com o alvo {alvo_mm}, veio {leitura.coordenada}")

# A coordenada estabilizada vira sequência de comandos.
etapas = []
cortadas = []
for etapa in controlador.executar(leitura.x_mm, leitura.y_mm):
    etapas.append(etapa)
    # No ponto em que uma etapa termina, o braço tem que estar parado. Se não
    # estiver, a etapa seguinte comeca por cima do movimento em curso.
    if not serial.parado:
        cortadas.append((etapa.descricao, serial.pendente_s))

checa(len(etapas) == 8, f"a coleta deveria ter 8 etapas, tem {len(etapas)}")
checa(len(serial.linhas) == 16, f"deveriam sair 16 comandos, sairam {len(serial.linhas)}")
checa(all(l.startswith("SET ") or l == "HOME" for l in serial.linhas),
      "todo comando deveria ser SET ou HOME")
checa(serial.linhas[-1] == "HOME", "a sequencia deveria terminar em HOME")

# Nenhuma etapa pode comecar com a anterior ainda em movimento.
#
# Este e o teste que teria pego o bug que a suavizacao do firmware criou: a
# pausa fixa de 0.6s foi calibrada quando o SET era instantaneo, e virou curta
# demais quando o movimento passou a durar de 0.08s a 1.7s conforme a
# distancia. Cinco das oito etapas comecavam pela metade, e "fechar a garra"
# era interrompida aos 36% — o braco subia com a garra quase aberta.
if cortadas:
    pior = max(cortadas, key=lambda x: x[1])
    checa(False,
          f"{len(cortadas)} etapa(s) terminaram com o braco ainda andando; "
          f"pior: '{pior[0]}' faltando {pior[1]:.2f}s")

# Todo ângulo enviado tem que respeitar os limites da junta do braco.json.
for linha in serial.linhas:
    if not linha.startswith("SET "):
        continue
    _, canal, angulo = linha.split()
    canal, angulo = int(canal), float(angulo)
    servo = next(s for s in (calib.base, calib.ombro, calib.cotovelo, calib.garra)
                 if s.canal == canal)
    checa(servo.minimo <= angulo <= servo.maximo,
          f"'{linha}' viola o limite de {servo.nome} "
          f"[{servo.minimo:.0f}, {servo.maximo:.0f}]")


# --- objeto se movendo não dispara nada ----------------------------------
serial2 = SerialEspiã()
controlador2 = ControladorColeta(
    serial2, braco=braco, calib=calib, params=ParametrosColeta(**params_dict)
)
relogio2 = RelogioFalso()
portao2 = PortaoEstabilidade(
    homografia, ParametrosEstabilidade(janela=5, tolerancia_mm=3.0), relogio=relogio2
)

for i in range(15):
    desloca = i * 12          # ~12 px por deteccao, bem acima da tolerancia
    leit = portao2.observar(u - 20 + desloca, v - 40, u + 20 + desloca, v)
    if leit.estavel:
        controlador2.executar(leit.x_mm, leit.y_mm)
    relogio2.avanca(0.16)

checa(not serial2.linhas,
      f"objeto em movimento nao deveria gerar comando, gerou {len(serial2.linhas)}")


# --- angulos da garra: explicitos mandam, e sao validados ----------------
from src.arm.coleta import ParametrosColeta as PC  # noqa: E402

base_params = dict(params_dict)

# sem configurar, cai no minimo/maximo da junta (comportamento antigo)
s_pad = SerialEspiã()
c_pad = ControladorColeta(s_pad, braco=braco, calib=calib, params=PC(**base_params))
cmds = [e.comandos[0] for e in c_pad.planejar(*alvo_mm) if "garra" in e.descricao.lower()]
checa(cmds[0] == f"SET {calib.garra.canal} {calib.garra.minimo}",
      f"garra aberta padrao deveria ser o minimo da junta, veio {cmds[0]}")
checa(cmds[1] == f"SET {calib.garra.canal} {calib.garra.maximo}",
      f"garra fechada padrao deveria ser o maximo da junta, veio {cmds[1]}")

# configurados, os valores explicitos prevalecem
p_expl = dict(base_params, garra_aberta=40.0, garra_fechada=110.0)
c_expl = ControladorColeta(SerialEspiã(), braco=braco, calib=calib, params=PC(**p_expl))
cmds = [e.comandos[0] for e in c_expl.planejar(*alvo_mm) if "garra" in e.descricao.lower()]
checa(cmds[0] == f"SET {calib.garra.canal} 40.0", f"garra aberta explicita ignorada: {cmds[0]}")
checa(cmds[1] == f"SET {calib.garra.canal} 110.0", f"garra fechada explicita ignorada: {cmds[1]}")

# invertidos (garra montada ao contrario) tambem funcionam: nada assume ordem
p_inv = dict(base_params, garra_aberta=120.0, garra_fechada=35.0)
c_inv = ControladorColeta(SerialEspiã(), braco=braco, calib=calib, params=PC(**p_inv))
cmds = [e.comandos[0] for e in c_inv.planejar(*alvo_mm) if "garra" in e.descricao.lower()]
checa(cmds[0] == f"SET {calib.garra.canal} 120.0" and cmds[1] == f"SET {calib.garra.canal} 35.0",
      f"garra montada ao contrario deveria funcionar, veio {cmds}")

# fora dos limites da junta: recusa antes de mover, e diz onde consertar
p_ruim = dict(base_params, garra_fechada=175.0)   # acima do maximo (130)
c_ruim = ControladorColeta(SerialEspiã(), braco=braco, calib=calib, params=PC(**p_ruim))
try:
    c_ruim.planejar(*alvo_mm)
    checa(False, "garra fora do limite deveria levantar PoseInalcancavel")
except PoseInalcancavel as e:
    checa("config" in str(e).lower(),
          f"a mensagem deveria apontar para a config, veio: {e}")


for f in falhas:
    print(f"  FALHA: {f}")
print("OK — a corrente visao -> braco esta ligada" if not falhas
      else f"\n{len(falhas)} falha(s)")
raise SystemExit(1 if falhas else 0)
