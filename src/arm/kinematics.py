"""
Cinemática do braço: converte uma posição (x, y, z) em milímetros
nos ângulos de servo que o firmware entende.

São duas camadas separadas de propósito:

1. `resolver_ik()` — matemática pura. Trabalha na convenção matemática
   (ângulos absolutos, 0° = horizontal, cotovelo relativo ao ombro).
   Não sabe nada sobre servos.

2. `CalibracaoServo` — traduz ângulo matemático -> ângulo de servo (0-180).
   É aqui que entram offset, inversão de sentido e limites físicos, que
   variam conforme a montagem mecânica do braço.

Essa separação importa: se o braço for remontado ou trocado, só a camada 2
muda. A matemática da camada 1 continua válida.

Convenção da camada 1 (mantenha igual no firmware, se algum dia migrar pra lá):
    theta_base     -> rotação no plano XY, 0° = eixo +X
    theta_ombro    -> ângulo do elo L2 em relação à horizontal
    theta_cotovelo -> ângulo do elo L3 RELATIVO ao L2 (0° = braço esticado)
"""

from dataclasses import dataclass, field
import math


# ---------------------------------------------------------------------------
# Camada 1: matemática pura
# ---------------------------------------------------------------------------

@dataclass
class Braco:
    """Dimensões físicas do braço, em milímetros. MEÇA no braço real."""
    # Altura VERTICAL do topo da placa até o eixo do ombro. A placa do braço
    # tem a mesma espessura das placas da área de coleta, então esse topo é o
    # mesmo plano onde o objeto se apoia e onde a homografia é calibrada:
    # z = 0 vale para os dois, sem offset a corrigir.
    l1: float = 50.0
    l2: float = 80.0   # eixo do ombro -> eixo do cotovelo
    l3: float = 80.0   # eixo do cotovelo -> ponto de preensão da garra

    # Deslocamento horizontal entre o eixo da base e o eixo do ombro.
    # Em vários braços o ombro não fica exatamente acima do centro de giro
    # da base; ignorar isso produz um erro radial constante, do mesmo
    # tamanho do deslocamento, em todos os alvos.
    offset_radial: float = 0.0

    @property
    def alcance_max(self) -> float:
        return self.l2 + self.l3

    @property
    def alcance_min(self) -> float:
        return abs(self.l2 - self.l3)

    def resolver_ik(self, x: float, y: float, z: float):
        """
        Entrada: alvo (x, y, z) em mm, relativo à base do braço.
        Saída: dict com ângulos em GRAUS (convenção matemática), ou
               None se o alvo estiver fora do envelope alcançável.
        """
        theta_base = math.degrees(math.atan2(y, x))

        # Distância horizontal, descontando o deslocamento do ombro: o
        # ombro já "adianta" essa distância na direção do alvo.
        r = math.hypot(x, y) - self.offset_radial
        h = z - self.l1           # altura relativa ao ombro
        dist2 = r * r + h * h
        dist = math.sqrt(dist2)

        if dist > self.alcance_max or dist < self.alcance_min:
            return None

        # Lei dos cossenos -> ângulo relativo do cotovelo (solução "elbow-up")
        cos_cot = (dist2 - self.l2 ** 2 - self.l3 ** 2) / (2 * self.l2 * self.l3)
        cos_cot = max(-1.0, min(1.0, cos_cot))   # margem contra erro de float
        theta_cotovelo = math.acos(cos_cot)

        theta_ombro = math.atan2(h, r) - math.atan2(
            self.l3 * math.sin(theta_cotovelo),
            self.l2 + self.l3 * math.cos(theta_cotovelo),
        )

        return {
            "base": theta_base,
            "ombro": math.degrees(theta_ombro),
            "cotovelo": math.degrees(theta_cotovelo),
        }

    def cinematica_direta(self, base: float, ombro: float, cotovelo: float):
        """Inverso da IK. Serve para validar que os ângulos reproduzem o alvo."""
        tb, to, tc = map(math.radians, (base, ombro, cotovelo))
        r = self.l2 * math.cos(to) + self.l3 * math.cos(to + tc) + self.offset_radial
        h = self.l2 * math.sin(to) + self.l3 * math.sin(to + tc)
        return r * math.cos(tb), r * math.sin(tb), h + self.l1

    def erro_reconstrucao(self, x, y, z) -> float | None:
        """
        Resolve a IK e reconstrói via cinemática direta.
        Retorna a distância entre o alvo pedido e o reconstruído (mm),
        ou None se o alvo for inalcançável. Use isso como teste de sanidade.
        """
        ang = self.resolver_ik(x, y, z)
        if ang is None:
            return None
        xr, yr, zr = self.cinematica_direta(ang["base"], ang["ombro"], ang["cotovelo"])
        return math.dist((x, y, z), (xr, yr, zr))


# ---------------------------------------------------------------------------
# Camada 2: tradução para ângulos de servo
# ---------------------------------------------------------------------------

@dataclass
class CalibracaoServo:
    """
    Traduz um ângulo matemático em ângulo de servo (0-180).

        angulo_servo = offset + sentido * angulo_matematico

    - `offset`: quanto vale, no servo, o 0° matemático da junta.
    - `sentido`: +1 se o servo cresce no mesmo sentido da convenção
      matemática, -1 se cresce ao contrário (descobre-se testando).
    - `minimo`/`maximo`: limites FÍSICOS reais da junta, achados com o
      sketch de teste individual. Nunca deixe 0/180 sem ter verificado.
    """
    canal: int
    nome: str
    offset: float = 90.0
    sentido: int = 1
    minimo: float = 10.0
    maximo: float = 170.0

    def para_servo(self, angulo_matematico: float) -> float:
        return self.offset + self.sentido * angulo_matematico

    def dentro_dos_limites(self, angulo_servo: float) -> bool:
        return self.minimo <= angulo_servo <= self.maximo

    def clamp(self, angulo_servo: float) -> float:
        return max(self.minimo, min(self.maximo, angulo_servo))


@dataclass
class CalibracaoBraco:
    """
    Conjunto de calibrações, uma por junta, na ordem dos canais do PCA9685.

    ATENÇÃO: os valores abaixo são PLACEHOLDERS auto-consistentes, escolhidos
    só para o pipeline rodar ponta a ponta antes da calibração física. Eles
    NÃO correspondem ao braço real. Antes de mover o braço de verdade:

      1. rode o sketch de teste individual e ache o mín/máx real de cada junta;
      2. ache o `offset` mandando a junta para o 0° matemático e vendo que
         ângulo de servo deixa o elo na posição de referência;
      3. ache o `sentido` mandando um ângulo maior e vendo se o elo se move
         no sentido esperado (se for ao contrário, use -1).
    """
    base: CalibracaoServo = field(
        default_factory=lambda: CalibracaoServo(0, "base", offset=0, sentido=1, minimo=10, maximo=170)
    )
    ombro: CalibracaoServo = field(
        default_factory=lambda: CalibracaoServo(1, "ombro", offset=90, sentido=1, minimo=20, maximo=160)
    )
    cotovelo: CalibracaoServo = field(
        default_factory=lambda: CalibracaoServo(2, "cotovelo", offset=0, sentido=1, minimo=10, maximo=170)
    )
    garra: CalibracaoServo = field(
        default_factory=lambda: CalibracaoServo(3, "garra", offset=0, sentido=1, minimo=30, maximo=130)
    )

    def juntas_ik(self):
        """Só as três juntas que participam da cinemática (garra fica de fora)."""
        return {"base": self.base, "ombro": self.ombro, "cotovelo": self.cotovelo}


class PoseInalcancavel(Exception):
    """
    Alvo fora do envelope físico, ou ângulo fora dos limites da junta.

    O `motivo` diz qual dos dois casos ocorreu, e as duas coisas se resolvem
    de maneiras opostas: ENVELOPE quer dizer que o braço é curto (ou longo)
    demais para o alvo, e só se resolve mudando a geometria ou aproximando a
    área de coleta; LIMITE_JUNTA quer dizer que o braço alcançaria, mas a
    calibração proíbe o ângulo — normalmente é limite apertado demais no
    `braco.json`, não impossibilidade física.

    Leia `e.motivo`, nunca o texto da mensagem: o texto é para humanos e pode
    mudar sem aviso.
    """

    ENVELOPE = "envelope"
    LIMITE_JUNTA = "limite_junta"

    def __init__(self, mensagem: str, motivo: str):
        super().__init__(mensagem)
        self.motivo = motivo


def pose_para_servos(
    braco: Braco,
    calib: CalibracaoBraco,
    x: float,
    y: float,
    z: float,
    permitir_clamp: bool = False,
) -> dict[int, float]:
    """
    Caminho completo: alvo em mm -> {canal_do_servo: ângulo}.

    Levanta PoseInalcancavel se o alvo estiver fora do envelope, ou se
    algum ângulo cair fora dos limites da junta (a menos que
    `permitir_clamp=True`, que satura em vez de recusar).

    Recusar por padrão é intencional: um alvo ligeiramente fora do alcance
    que vira um clamp silencioso faz o braço ir para uma pose errada sem
    ninguém perceber. Melhor falhar alto e tratar em cima.
    """
    angulos = braco.resolver_ik(x, y, z)
    if angulos is None:
        raise PoseInalcancavel(
            f"Alvo ({x:.0f}, {y:.0f}, {z:.0f}) fora do envelope "
            f"[{braco.alcance_min:.0f}..{braco.alcance_max:.0f}] mm",
            motivo=PoseInalcancavel.ENVELOPE,
        )

    comandos: dict[int, float] = {}
    for nome, servo in calib.juntas_ik().items():
        bruto = servo.para_servo(angulos[nome])
        if not servo.dentro_dos_limites(bruto):
            if not permitir_clamp:
                raise PoseInalcancavel(
                    f"Junta '{nome}' exigiria {bruto:.1f}°, "
                    f"fora do limite [{servo.minimo:.0f}, {servo.maximo:.0f}]",
                    motivo=PoseInalcancavel.LIMITE_JUNTA,
                )
            bruto = servo.clamp(bruto)
        comandos[servo.canal] = round(bruto, 1)

    return comandos


def classificar_alvo(braco: Braco, calib: CalibracaoBraco, x: float, y: float, z: float) -> str:
    """
    Diz se o braço pega em (x, y, z), sem levantar exceção.

    Devolve "ok", PoseInalcancavel.ENVELOPE ou PoseInalcancavel.LIMITE_JUNTA.
    Serve para desenhar mapas de alcance, onde é preciso testar milhares de
    pontos e um try/except por ponto polui quem chama.
    """
    try:
        pose_para_servos(braco, calib, x, y, z)
        return "ok"
    except PoseInalcancavel as e:
        return e.motivo


# ---------------------------------------------------------------------------
# Carregamento a partir de config/braco.json
# ---------------------------------------------------------------------------

def carregar_config(caminho="config/braco.json"):
    """
    Lê o JSON de configuração e devolve (braco, calib, params_dict, calibrado).

    `params_dict` sai como dicionário e não como ParametrosColeta para
    evitar import circular — quem chama monta o objeto.

    `calibrado` é a flag do arquivo: enquanto for False, os números ainda
    são placeholders e o braço não deve ser movido de verdade.
    """
    import json
    from pathlib import Path

    caminho = Path(caminho)
    if not caminho.exists():
        raise FileNotFoundError(
            f"config não encontrada em {caminho}. "
            "Copie config/braco.json do repositório e preencha com as medidas reais."
        )

    dados = json.loads(caminho.read_text(encoding="utf-8"))

    elos = dados["elos_mm"]
    braco = Braco(
        l1=float(elos["l1"]),
        l2=float(elos["l2"]),
        l3=float(elos["l3"]),
        offset_radial=float(dados.get("offset_radial_mm", 0.0)),
    )

    def _servo(nome: str) -> CalibracaoServo:
        j = dados["juntas"][nome]
        return CalibracaoServo(
            canal=int(j["canal"]),
            nome=nome,
            offset=float(j["offset"]),
            sentido=int(j["sentido"]),
            minimo=float(j["minimo"]),
            maximo=float(j["maximo"]),
        )

    calib = CalibracaoBraco(
        base=_servo("base"),
        ombro=_servo("ombro"),
        cotovelo=_servo("cotovelo"),
        garra=_servo("garra"),
    )

    coleta = dados.get("coleta", {})
    params = {
        "z_mesa": float(coleta.get("z_mesa", 0.0)),
        "z_pega": float(coleta.get("z_pega", 15.0)),
        "z_aproximacao": float(coleta.get("z_aproximacao", 70.0)),
        "z_transporte": float(coleta.get("z_transporte", 80.0)),
        "deposito": tuple(coleta.get("deposito_xy", (-80.0, 100.0))),
        "pausa_s": float(coleta.get("pausa_s", 0.25)),
        # None deixa a coleta cair no mínimo/máximo da junta. Explícito é
        # melhor: qual extremo fecha a garra depende da montagem mecânica.
        "garra_aberta": (
            float(coleta["garra_aberta_graus"])
            if coleta.get("garra_aberta_graus") is not None else None
        ),
        "garra_fechada": (
            float(coleta["garra_fechada_graus"])
            if coleta.get("garra_fechada_graus") is not None else None
        ),
    }

    return braco, calib, params, bool(dados.get("calibrado", False))
