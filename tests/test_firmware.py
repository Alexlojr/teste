"""
Testes do firmware sem Arduino na mesa.

O firmware é C e roda no Nano, então não dá para importá-lo aqui. O que este
arquivo faz é reimplementar em Python as duas partes que mais quebram em
silêncio, e exercitá-las:

1. a cadeia de if/else que despacha os comandos, junto com a tokenização e as
   conversões atoi/atof que ela usa — é aí que um comando engole o outro
   (SET/SPEED/SWEEP/STOP começam todos com S, e GET colide com GET ALL);

2. a curva de suavização do movimento, para confirmar que a velocidade
   realmente nasce e morre em zero. Se ela não fizer isso, o servo dá tranco,
   que é o defeito que a curva existe para resolver.

Isto NÃO substitui compilar e carregar. Erro de sintaxe, biblioteca faltando e
problema de I2C não aparecem aqui — e, mais importante, DIFERENÇAS DA
BIBLIOTECA DO ALVO também não. A primeira versão deste arquivo modelava o
sscanf de C padrão e passava com 100%, enquanto o firmware respondia
ERR UNKNOWN no hardware, porque o avr-libc linka um scanf sem ponto flutuante.
É teste de lógica, não de integração, e a lógica pode estar certa contra a
plataforma errada.

    python tests/test_firmware.py
"""

import re

# Precisam bater com os #define do firmware.
NUM_SERVOS = 4
INTERVALO_PASSO_MS = 20
DURACAO_MIN_MS = 80
JANELA_LOTE_MS = 50
SPEED_MAX = 150


# ---------------------------------------------------------------------------
# 1. Dispatch de comandos
# ---------------------------------------------------------------------------

def divide_em_tokens(texto, maximo=4):
    """
    Espelha divideEmTokens() do firmware: separa por espaço ou tabulação,
    no máximo `maximo` tokens.
    """
    tok, i, n = [], 0, len(texto)
    while i < n and len(tok) < maximo:
        while i < n and texto[i] in " 	":
            i += 1
        if i >= n:
            break
        j = i
        while j < n and texto[j] not in " 	":
            j += 1
        tok.append(texto[i:j])
        i = j
    return tok


def atoi(t):
    """atoi() do C: lê o inteiro do começo, 0 se não houver nenhum."""
    m = re.match(r"\s*[+-]?\d+", t)
    return int(m.group(0)) if m else 0


def atof(t):
    """atof() do C: lê o float do começo, 0.0 se não houver nenhum."""
    m = re.match(r"\s*[+-]?(\d+\.?\d*|\.\d+)([eE][+-]?\d+)?", t)
    return float(m.group(0)) if m else 0.0


def dispatch(texto):
    """
    Espelha processaComando(), devolvendo o nome do ramo que casou.

    NOTA HISTÓRICA: a primeira versão do firmware usava sscanf com "%d %f", e
    esta função a reproduzia com a semântica do sscanf de C padrão. No AVR isso
    não vale: a biblioteca do Arduino linka um scanf sem suporte a ponto
    flutuante, então TODOS os comandos com argumento decimal falhavam em
    silêncio e respondiam ERR UNKNOWN. O teste passava e o hardware não
    funcionava. O parser foi reescrito com tokenização mais atoi/atof, e este
    modelo acompanha.
    """
    tok = divide_em_tokens(texto.upper())
    n = len(tok)
    if n == 0:
        return None

    cmd = tok[0]

    def id_ok(i):
        return 0 <= i < NUM_SERVOS

    if cmd == "SET" and n == 3:
        return "SET" if id_ok(atoi(tok[1])) else "ERR BAD_ARGS"
    elif cmd == "GET" and n == 2 and tok[1] == "ALL":
        return "GET ALL"
    elif cmd == "GET" and n == 2:
        return "GET" if id_ok(atoi(tok[1])) else "ERR BAD_ARGS"
    elif cmd == "HOME" and n == 1:
        return "HOME"
    elif cmd == "STOP" and n == 1:
        return "STOP"
    elif cmd == "BUSY" and n == 1:
        return "BUSY"
    elif cmd == "SWEEP" and n == 4:
        return "SWEEP" if id_ok(atoi(tok[1])) else "ERR BAD_ARGS"
    elif cmd == "LIM" and n == 4:
        i, a, b = atoi(tok[1]), atof(tok[2]), atof(tok[3])
        return "ERR BAD_ARGS" if not id_ok(i) or a > b else "LIM set"
    elif cmd == "LIM" and n == 1:
        return "LIM show"
    elif cmd == "SPEED" and n == 2:
        a = atof(tok[1])
        return "ERR BAD_ARGS" if a <= 0 or a > SPEED_MAX else "SPEED"
    elif cmd == "DETACH" and n == 2 and tok[1] == "ALL":
        return "DETACH ALL"
    elif cmd == "DETACH" and n == 2:
        return "DETACH" if id_ok(atoi(tok[1])) else "ERR BAD_ARGS"
    elif cmd == "ATTACH" and n == 2 and tok[1] == "ALL":
        return "ATTACH ALL"
    elif cmd == "ATTACH" and n == 2:
        return "ATTACH" if id_ok(atoi(tok[1])) else "ERR BAD_ARGS"
    elif cmd == "RAW" and n == 3:
        i, pulso = atoi(tok[1]), atoi(tok[2])
        return "ERR BAD_ARGS" if not id_ok(i) or not (0 <= pulso <= 4095) else "RAW"
    elif cmd == "PULSO" and n == 4:
        i, pi, pj = atoi(tok[1]), atoi(tok[2]), atoi(tok[3])
        return "ERR BAD_ARGS" if not id_ok(i) or pi < 0 or pj > 4095 or pi >= pj else "PULSO"
    elif cmd == "HELP" and n == 1:
        return "HELP"
    else:
        return "ERR UNKNOWN"


CASOS_DISPATCH = [
    # operação
    ("SET 1 45", "SET"), ("SET 0 90.5", "SET"), ("set 2 100", "SET"),
    # Os quatro abaixo usam argumento DECIMAL. Eram exatamente os que o parser
    # com sscanf quebrava no AVR, respondendo ERR UNKNOWN.
    ("SET 1 45.7", "SET"), ("SWEEP 1 80.5 100.5", "SWEEP"),
    ("LIM 1 20.5 160.5", "LIM set"), ("SPEED 12.5", "SPEED"),
    ("SET 3 -10", "SET"),                       # negativo é limitado, não recusado
    ("GET 0", "GET"), ("GET ALL", "GET ALL"),
    ("HOME", "HOME"), ("STOP", "STOP"), ("BUSY", "BUSY"),

    # bancada
    ("SWEEP 1 80 100", "SWEEP"),
    ("LIM", "LIM show"), ("LIM 1 20 160", "LIM set"),
    ("SPEED 30", "SPEED"), ("SPEED 45.5", "SPEED"), ("SPEED 150", "SPEED"),
    ("DETACH ALL", "DETACH ALL"), ("DETACH 2", "DETACH"),
    ("ATTACH ALL", "ATTACH ALL"), ("ATTACH 2", "ATTACH"),
    ("RAW 0 300", "RAW"), ("PULSO 0 140 610", "PULSO"), ("HELP", "HELP"),

    # prefixos que poderiam se engolir: SET/SPEED/SWEEP/STOP, GET/GET ALL
    ("SPEED 60", "SPEED"), ("SWEEP 0 10 30", "SWEEP"),
    ("GET 3", "GET"), ("LIM 0 0 180", "LIM set"),
    ("DETACH 0", "DETACH"), ("ATTACH 0", "ATTACH"), ("PULSO 1 150 600", "PULSO"),

    # argumentos inválidos
    ("SET 9 45", "ERR BAD_ARGS"), ("SET 4 45", "ERR BAD_ARGS"),
    ("GET 9", "ERR BAD_ARGS"), ("SWEEP 7 10 20", "ERR BAD_ARGS"),
    ("LIM 0 170 10", "ERR BAD_ARGS"),           # min > max
    ("SPEED 0", "ERR BAD_ARGS"), ("SPEED 200", "ERR BAD_ARGS"),
    ("RAW 0 9999", "ERR BAD_ARGS"), ("PULSO 0 600 150", "ERR BAD_ARGS"),

    # lixo e argumentos faltando
    ("SET", "ERR UNKNOWN"), ("SET 1", "ERR UNKNOWN"), ("XYZ", "ERR UNKNOWN"),
    ("SETUP 1 2", "ERR UNKNOWN"), ("GETALL", "ERR UNKNOWN"),
    ("", None), ("   ", None),          # linha vazia: o firmware nao responde

    # o firmware come espaço das pontas antes de comparar
    ("  HOME", "HOME"), ("SET 1 45  ", "SET"), ("\tGET ALL ", "GET ALL"),
]


# ---------------------------------------------------------------------------
# 2. Suavização do movimento
# ---------------------------------------------------------------------------

def suaviza(p):
    """smootherstep: 6p^5 - 15p^4 + 10p^3. Espelha suaviza() do firmware."""
    if p <= 0.0:
        return 0.0
    if p >= 1.0:
        return 1.0
    return p * p * p * (p * (p * 6.0 - 15.0) + 10.0)


class BracoSimulado:
    """Modelo de defineAlvo() + atualizaMovimento()."""

    def __init__(self, velocidade=60.0):
        self.atual = [90.0] * NUM_SERVOS
        self.alvo = [90.0] * NUM_SERVOS
        self.origem = [90.0] * NUM_SERVOS
        self.t_inicio = [0] * NUM_SERVOS
        self.duracao = [0] * NUM_SERVOS
        self.velocidade = velocidade
        self.t = 0                       # ms
        self.t_lote = -10 ** 9
        self.duracao_lote = 0
        self.no_lote = [False] * NUM_SERVOS

    def define_alvo(self, i, angulo):
        if self.t - self.t_lote > JANELA_LOTE_MS:
            self.t_lote = self.t
            self.duracao_lote = 0
            self.no_lote = [False] * NUM_SERVOS

        self.origem[i] = self.atual[i]
        self.alvo[i] = angulo
        self.no_lote[i] = True

        natural = int(abs(angulo - self.origem[i]) / self.velocidade * 1000)
        natural = max(natural, DURACAO_MIN_MS)
        self.duracao_lote = max(self.duracao_lote, natural)

        for k in range(NUM_SERVOS):
            if self.no_lote[k]:
                self.t_inicio[k] = self.t_lote
                self.duracao[k] = self.duracao_lote

    def passo(self):
        self.t += INTERVALO_PASSO_MS
        for i in range(NUM_SERVOS):
            if self.duracao[i] == 0:
                self.atual[i] = self.alvo[i]
                continue
            decorrido = self.t - self.t_inicio[i]
            if decorrido >= self.duracao[i]:
                self.atual[i] = self.alvo[i]
                self.duracao[i] = 0
            else:
                p = decorrido / self.duracao[i]
                self.atual[i] = self.origem[i] + (self.alvo[i] - self.origem[i]) * suaviza(p)

    def perfil(self, i):
        """Velocidades instantâneas (°/s) até a junta i parar."""
        vels, anterior = [], self.atual[i]
        while self.duracao[i] != 0:
            self.passo()
            vels.append((self.atual[i] - anterior) / (INTERVALO_PASSO_MS / 1000.0))
            anterior = self.atual[i]
        return vels


def testa_movimento():
    """Devolve a lista de falhas encontradas."""
    falhas = []

    def checa(condicao, descricao):
        if not condicao:
            falhas.append(descricao)

    # --- a velocidade nasce e morre em zero -----------------------------
    b = BracoSimulado(60.0)
    b.define_alvo(1, 180.0)
    vels = b.perfil(1)

    checa(abs(vels[0]) < 1.0, f"velocidade inicial deveria ser ~0, veio {vels[0]:.2f}")
    checa(abs(vels[-1]) < 1.0, f"velocidade final deveria ser ~0, veio {vels[-1]:.2f}")

    media = sum(vels) / len(vels)
    checa(abs(media - 60.0) < 1.0, f"velocidade media deveria ser ~60, veio {media:.2f}")

    # A smootherstep tem pico exatamente 1,875x a média. Se essa razão mudar,
    # a curva foi trocada e o teto do SPEED precisa ser revisto junto.
    razao = max(vels) / media
    checa(abs(razao - 1.875) < 0.05, f"pico/media deveria ser ~1.875, veio {razao:.3f}")

    duracao_s = len(vels) * INTERVALO_PASSO_MS / 1000.0
    checa(abs(duracao_s - 1.5) < 0.05, f"90 graus a 60/s deveria durar 1.5 s, deu {duracao_s:.2f}")

    # --- movimento minúsculo ainda é suavizado --------------------------
    b = BracoSimulado(60.0)
    b.define_alvo(0, 91.0)          # 1 grau: natural daria 16 ms, o piso e 80
    checa(b.duracao[0] == DURACAO_MIN_MS,
          f"movimento de 1 grau deveria durar {DURACAO_MIN_MS} ms, deu {b.duracao[0]}")

    # --- juntas disparadas juntas chegam juntas -------------------------
    b = BracoSimulado(60.0)
    b.define_alvo(0, 120.0)         # 30 graus
    b.define_alvo(1, 150.0)         # 60 graus
    b.define_alvo(2, 95.0)          #  5 graus
    checa(b.duracao[0] == b.duracao[1] == b.duracao[2],
          f"lote deveria ter duracao unica, veio {b.duracao[:3]}")

    chegada = [None] * 3
    while any(b.duracao[i] != 0 for i in range(3)):
        b.passo()
        for i in range(3):
            if b.duracao[i] == 0 and chegada[i] is None:
                chegada[i] = b.t
    checa(chegada[0] == chegada[1] == chegada[2],
          f"juntas do mesmo lote deveriam chegar juntas, chegaram em {chegada}")

    # --- SET no meio do movimento não salta de posição ------------------
    b = BracoSimulado(60.0)
    b.define_alvo(1, 180.0)
    for _ in range(20):             # 400 ms de movimento
        b.passo()
    meio = b.atual[1]
    b.define_alvo(1, 100.0)         # muda de ideia no meio do caminho
    checa(abs(b.origem[1] - meio) < 0.01,
          "um SET novo deveria partir da posicao atual, nao do alvo antigo")
    b.passo()
    checa(abs(b.atual[1] - meio) < 2.0,
          f"a posicao saltou {abs(b.atual[1]-meio):.1f} graus ao mudar de alvo")

    # --- sweep inverte com o servo parado -------------------------------
    b = BracoSimulado(30.0)
    b.define_alvo(1, 80.0)
    de, ate = 80.0, 100.0
    vels_inversao = []
    for _ in range(int(8000 / INTERVALO_PASSO_MS)):
        anterior = b.atual[1]
        b.passo()
        if b.duracao[1] == 0:
            vels_inversao.append(abs((b.atual[1] - anterior) / (INTERVALO_PASSO_MS / 1000.0)))
            b.define_alvo(1, ate if abs(b.alvo[1] - de) < 0.05 else de)

    checa(len(vels_inversao) >= 4, "o sweep deveria ter invertido varias vezes")
    checa(max(vels_inversao) < 2.0,
          f"o sweep deveria inverter parado, veio {max(vels_inversao):.2f} graus/s")

    return falhas


# ---------------------------------------------------------------------------

def main():
    falhas = 0

    for entrada, esperado in CASOS_DISPATCH:
        obtido = dispatch(entrada)
        if obtido != esperado:
            falhas += 1
            print(f"  FALHA dispatch {entrada!r:20} esperado={esperado!r:16} obtido={obtido!r}")
    print(f"{len(CASOS_DISPATCH) - falhas}/{len(CASOS_DISPATCH)} casos de dispatch")

    falhas_mov = testa_movimento()
    for f in falhas_mov:
        print(f"  FALHA movimento: {f}")
    print(f"{'todos os' if not falhas_mov else ''} testes de movimento "
          f"{'passaram' if not falhas_mov else f'-> {len(falhas_mov)} falha(s)'}")

    total = falhas + len(falhas_mov)
    print("\nOK" if total == 0 else f"\n{total} falha(s)")
    return 1 if total else 0


if __name__ == "__main__":
    raise SystemExit(main())
