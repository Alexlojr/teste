"""
Testes do portão de estabilidade.

Usa uma homografia falsa (1 pixel = 1 mm) e um relógio controlado, para que os
testes rodem instantaneamente e sem câmera.

    python tests/test_estabilidade.py
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from src.vision.estabilidade import (  # noqa: E402
    ParametrosEstabilidade,
    PortaoEstabilidade,
)


class HomografiaFalsa:
    """1 pixel = 1 mm, base da caixa. Deixa os testes falarem em mm direto."""

    def caixa_para_mm(self, x1, y1, x2, y2):
        return ((x1 + x2) / 2.0, y2)


class RelogioFalso:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def avanca(self, s):
        self.t += s


def caixa_em(x_mm, y_mm, largura=20.0):
    """Caixa cujo centro horizontal e base caem no ponto pedido."""
    return (x_mm - largura / 2, y_mm - largura, x_mm + largura / 2, y_mm)


falhas = []


def checa(condicao, descricao):
    if not condicao:
        falhas.append(descricao)


def novo(janela=5, tolerancia=3.0, timeout=8.0, validade=1.0):
    relogio = RelogioFalso()
    portao = PortaoEstabilidade(
        HomografiaFalsa(),
        ParametrosEstabilidade(janela=janela, tolerancia_mm=tolerancia,
                               timeout_s=timeout, validade_amostra_s=validade),
        relogio=relogio,
    )
    return portao, relogio


# --- objeto parado estabiliza e devolve a posição ------------------------
portao, relogio = novo()
leitura = None
for i in range(5):
    leitura = portao.observar(*caixa_em(100.0, 80.0))
    if i < 4:
        checa(not leitura.estavel, f"nao deveria estabilizar com {i+1} amostras")
    relogio.avanca(0.16)

checa(leitura.estavel, "objeto parado deveria estabilizar na 5a amostra")
checa(abs(leitura.x_mm - 100.0) < 0.01 and abs(leitura.y_mm - 80.0) < 0.01,
      f"coordenada errada: {leitura.coordenada}")
checa(leitura.dispersao_mm < 0.01, f"dispersao deveria ser 0, veio {leitura.dispersao_mm}")


# --- objeto em movimento nunca estabiliza --------------------------------
portao, relogio = novo()
estavel_alguma = False
y = 50.0
for _ in range(12):
    leitura = portao.observar(*caixa_em(100.0, y))
    estavel_alguma |= leitura.estavel
    y += 4.0                       # 4 mm por deteccao, acima da tolerancia
    relogio.avanca(0.16)
checa(not estavel_alguma, "objeto se movendo 4 mm/deteccao nao deveria estabilizar")


# --- tremor menor que a tolerância passa ---------------------------------
portao, relogio = novo(tolerancia=3.0)
tremor = [0.0, 0.8, -0.6, 0.5, -0.4]
for d in tremor:
    leitura = portao.observar(*caixa_em(100.0 + d, 80.0 + d))
    relogio.avanca(0.16)
checa(leitura.estavel, f"tremor de +-1 mm deveria passar, motivo: {leitura.motivo}")


# --- a mediana ignora uma detecção muito errada --------------------------
portao, relogio = novo(tolerancia=100.0)     # tolerancia alta de proposito
for d in [0.0, 0.0, 40.0, 0.0, 0.0]:         # uma amostra despencada
    leitura = portao.observar(*caixa_em(100.0 + d, 80.0))
    relogio.avanca(0.16)
checa(leitura.estavel, "deveria estabilizar com tolerancia alta")
checa(abs(leitura.x_mm - 100.0) < 0.01,
      f"a mediana deveria ignorar o outlier, veio x={leitura.x_mm}")


# --- amostras velhas caem sozinhas ---------------------------------------
portao, relogio = novo(validade=1.0)
for _ in range(4):
    portao.observar(*caixa_em(100.0, 80.0))
    relogio.avanca(0.16)
relogio.avanca(5.0)                          # objeto sumiu por 5 s
leitura = portao.observar(*caixa_em(100.0, 80.0))
checa(not leitura.estavel, "amostras de antes da pausa nao deveriam contar")
checa(leitura.amostras == 1, f"deveria ter sobrado 1 amostra, tem {leitura.amostras}")


# --- timeout desiste em vez de travar ------------------------------------
portao, relogio = novo(timeout=2.0)
desistiu = False
y = 50.0
for _ in range(40):
    leitura = portao.observar(*caixa_em(100.0, y))
    if "desistiu" in leitura.motivo:
        desistiu = True
        break
    y += 4.0
    relogio.avanca(0.16)
checa(desistiu, "deveria ter desistido apos o timeout")


# --- rearmar zera de verdade ---------------------------------------------
portao, relogio = novo()
for _ in range(5):
    portao.observar(*caixa_em(100.0, 80.0))
    relogio.avanca(0.16)
portao.rearmar()
leitura = portao.observar(*caixa_em(100.0, 80.0))
checa(not leitura.estavel and leitura.amostras == 1,
      f"apos rearmar deveria comecar do zero, veio {leitura.amostras} amostras")


# --- objeto trocado no meio nao mistura coordenadas ----------------------
portao, relogio = novo()
for _ in range(3):
    portao.observar(*caixa_em(50.0, 50.0))
    relogio.avanca(0.16)
portao.rearmar()                              # objeto saiu, outro entrou
for _ in range(5):
    leitura = portao.observar(*caixa_em(150.0, 90.0))
    relogio.avanca(0.16)
checa(leitura.estavel and abs(leitura.x_mm - 150.0) < 0.01,
      f"deveria usar so o objeto novo, veio {leitura.coordenada}")


for f in falhas:
    print(f"  FALHA: {f}")
print("OK — todos os testes de estabilidade passaram" if not falhas
      else f"\n{len(falhas)} falha(s)")
raise SystemExit(1 if falhas else 0)
