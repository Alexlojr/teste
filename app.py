import time
from collections import defaultdict
from pathlib import Path

import altair as alt
import cv2
import pandas as pd
import streamlit as st
from picamera2 import Picamera2
from ultralytics import YOLO

from src.arm.mock_arm_serial import SERVOS, MockArmSerial, detectar_porta_arduino

BASE_DIR = Path(__file__).resolve().parent
CAMINHO_MODELO = BASE_DIR / "models" / "yolov8n.pt"

st.set_page_config(page_title="Dashboard", layout="wide")
st.title("Dashboard")

aba_yolo, aba_braco, aba_coleta = st.tabs(
    ["Detecção de Objetos (YOLO)", "Braço Robótico (mock serial)", "Simulação de Coleta (clique)"]
)

# ---------------------------------------------------------------------------
# Aba 1: detecção de objetos com YOLO + Picamera2
# ---------------------------------------------------------------------------

CLASSES_PERMITIDAS = [0, 67]
INTERVALO_DETECCAO = 3
CONFIANCA_MINIMA = 0.5


@st.cache_resource
def carregar_modelo():
    return YOLO(str(CAMINHO_MODELO))


@st.cache_resource
def iniciar_camera():
    picam2 = Picamera2()
    picam2.configure(
        picam2.create_preview_configuration(
            main={"format": "RGB888", "size": (640, 480)},
            controls={"AwbEnable": True, "AwbMode": 0}  # Auto white balance
        )
    )
    picam2.start()
    return picam2


with aba_yolo:
    model = carregar_modelo()
    picam2 = iniciar_camera()

    if "rodando" not in st.session_state:
        st.session_state.rodando = False

    if "contagem" not in st.session_state:
        st.session_state.contagem = defaultdict(int)

    if "ultimo_objeto" not in st.session_state:
        st.session_state.ultimo_objeto = None

    if st.sidebar.button("Iniciar câmera"):
        st.session_state.rodando = True

    if st.sidebar.button("Parar"):
        st.session_state.rodando = False

    col1, col2 = st.columns([2, 1])

    camera_placeholder = col1.empty()
    status_placeholder = col2.empty()
    tabela_placeholder = col2.empty()

    frame_count = 0
    ultima_detec = None

    while st.session_state.rodando:
        frame = picam2.capture_array()
        frame_count += 1

        if frame_count % INTERVALO_DETECCAO == 0:
            results = model(
                frame,
                classes=CLASSES_PERMITIDAS,
                verbose=False
            )

            melhor_objeto = None
            maior_confianca = 0

            for box in results[0].boxes:
                confianca = float(box.conf[0])

                if confianca > maior_confianca:
                    maior_confianca = confianca
                    melhor_objeto = box

            if melhor_objeto is not None and maior_confianca > CONFIANCA_MINIMA:
                x1, y1, x2, y2 = map(int, melhor_objeto.xyxy[0])
                classe = int(melhor_objeto.cls[0])
                nome = model.names[classe]

                ultima_detec = (x1, y1, x2, y2, nome, maior_confianca)

                if st.session_state.ultimo_objeto != nome:
                    st.session_state.contagem[nome] += 1
                    st.session_state.ultimo_objeto = nome
            else:
                ultima_detec = None
                st.session_state.ultimo_objeto = None

        if ultima_detec is not None:
            x1, y1, x2, y2, nome, conf = ultima_detec

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

            cv2.putText(
                frame,
                f"{nome} {conf:.2f}",
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )

            status_placeholder.success(f"Detectando: {nome} ({conf:.2f})")
        else:
            status_placeholder.info("Nenhum objeto detectado")

        # O formato "RGB888" do Picamera2 na verdade entrega os bytes em
        # ordem BGR — sem essa conversão, o st.image() (que espera RGB)
        # exibe a imagem com tom azulado.
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        camera_placeholder.image(frame_rgb)

        tabela_placeholder.subheader("Contagem de objetos")

        if len(st.session_state.contagem) > 0:
            tabela_placeholder.table(dict(st.session_state.contagem))
        else:
            tabela_placeholder.write("Nenhum objeto contado ainda.")

        time.sleep(0.01)

    if not st.session_state.rodando:
        st.info("Clique em **Iniciar câmera** na barra lateral.")

# ---------------------------------------------------------------------------
# Aba 2: braço robótico com servos (mock de comunicação serial)
# ---------------------------------------------------------------------------

CORES_SERVO = {
    0: ("#2a78d6", "#3987e5"),  # base    -> azul
    1: ("#008300", "#008300"),  # ombro   -> verde
    2: ("#e87ba4", "#d55181"),  # cotovelo-> magenta
    3: ("#1baf7a", "#199e70"),  # garra   -> aqua
}
COR_BOA = "#0ca30c"
COR_ALERTA = "#fab219"

if "arm" not in st.session_state:
    st.session_state.arm = MockArmSerial(porta_arduino=detectar_porta_arduino())
if "arm_rodando" not in st.session_state:
    st.session_state.arm_rodando = False

arm: MockArmSerial = st.session_state.arm

_ESTILO_GAUGE = """
<style>
.arm-gauge { margin-bottom: 18px; }
.arm-gauge .cabecalho {
    display: flex; justify-content: space-between; align-items: baseline;
    font-family: system-ui, -apple-system, "Segoe UI", sans-serif;
}
.arm-gauge .nome { font-weight: 600; color: #0b0b0b; }
.arm-gauge .valor { font-variant-numeric: tabular-nums; color: #0b0b0b; }
.arm-gauge .trilho {
    position: relative; height: 12px; border-radius: 6px;
    background: #e1e0d9; margin-top: 6px; overflow: visible;
}
.arm-gauge .preenchido {
    position: absolute; top: 0; left: 0; height: 100%;
    border-radius: 6px; transition: width 0.15s linear;
}
.arm-gauge .alvo-marcador {
    position: absolute; top: -3px; width: 2px; height: 18px;
    background: #52514e;
}
.arm-gauge .status {
    font-family: system-ui, -apple-system, "Segoe UI", sans-serif;
    font-size: 0.8rem; margin-top: 4px;
}
""" + "\n".join(
    f".arm-gauge .servo-{sid} {{ background: {cor_clara}; }}"
    for sid, (cor_clara, _cor_escura) in CORES_SERVO.items()
) + """
@media (prefers-color-scheme: dark) {
    .arm-gauge .nome, .arm-gauge .valor { color: #ffffff; }
    .arm-gauge .trilho { background: #2c2c2a; }
    .arm-gauge .alvo-marcador { background: #c3c2b7; }
""" + "\n".join(
    f".arm-gauge .servo-{sid} {{ background: {cor_escura}; }}"
    for sid, (_cor_clara, cor_escura) in CORES_SERVO.items()
) + """
}
</style>
"""


def render_gauge(sid, estado):
    pct_atual = max(0.0, min(100.0, estado["atual"] / 180 * 100))
    pct_alvo = max(0.0, min(100.0, estado["alvo"] / 180 * 100))

    if estado["em_movimento"]:
        status_html = f'<span style="color:{COR_ALERTA};">● movendo</span>'
    else:
        status_html = f'<span style="color:{COR_BOA};">✓ parado</span>'

    st.markdown(
        f"""
        <div class="arm-gauge">
          <div class="cabecalho">
            <span class="nome">{estado['nome']}</span>
            <span class="valor">{estado['atual']:.1f}°</span>
          </div>
          <div class="trilho">
            <div class="preenchido servo-{sid}" style="width:{pct_atual}%;"></div>
            <div class="alvo-marcador" style="left:{pct_alvo}%;" title="alvo: {estado['alvo']:.0f}°"></div>
          </div>
          <div class="status">{status_html} · alvo {estado['alvo']:.0f}°</div>
        </div>
        """,
        unsafe_allow_html=True,
    )


with aba_braco:
    st.markdown(_ESTILO_GAUGE, unsafe_allow_html=True)

    with st.sidebar:
        st.divider()
        st.subheader("Braço robótico — conexão serial")
        status_conexao = "🔌 Arduino real conectado" if arm.braco_real else "🖥️ Só simulação (Arduino não encontrado)"
        st.caption(f"{status_conexao} · Porta: `{arm.port}` · {arm.baudrate} baud")

        st.subheader("Enviar comando manual")
        comando = st.text_input("Comando", placeholder="SET 1 45", label_visibility="collapsed")
        col_enviar, col_home = st.columns(2)
        if col_enviar.button("Enviar", use_container_width=True) and comando:
            arm.write((comando + "\n").encode())
        if col_home.button("HOME", use_container_width=True):
            arm.write(b"HOME\n")

        st.session_state.arm_rodando = st.checkbox(
            "Atualização contínua do braço", value=st.session_state.arm_rodando
        )

        with st.expander("Log serial do braço (TX/RX)", expanded=True):
            log_placeholder = st.empty()

    st.subheader("Controle dos servos")
    colunas = st.columns(len(SERVOS))

    for col, (sid, nome) in zip(colunas, sorted(SERVOS.items())):
        chave_slider = f"slider_{sid}"
        chave_ultimo = f"ultimo_alvo_{sid}"
        angulo = col.slider(nome.capitalize(), 0, 180, st.session_state.get(chave_slider, 90), key=chave_slider)

        if st.session_state.get(chave_ultimo) != angulo:
            arm.write(f"SET {sid} {angulo}\n".encode())
            st.session_state[chave_ultimo] = angulo

    st.divider()
    st.subheader("Leitura dos servos (resposta do braço)")
    gauges_placeholder = st.empty()

    def desenha_estado_braco():
        snap = arm.snapshot()
        with gauges_placeholder.container():
            colunas_gauge = st.columns(len(snap))
            for col, (sid, estado) in zip(colunas_gauge, snap.items()):
                with col:
                    render_gauge(sid, estado)

        with log_placeholder.container():
            linhas = arm.historico()[-12:]
            if linhas:
                texto = "\n".join(
                    f"{hora}  {'→' if direcao == 'TX' else '←'}  {linha}"
                    for hora, direcao, linha in linhas
                )
                st.code(texto, language=None)
            else:
                st.caption("Nenhum comando enviado ainda.")

    desenha_estado_braco()

    if st.session_state.arm_rodando:
        while st.session_state.arm_rodando:
            desenha_estado_braco()
            time.sleep(0.1)

# ---------------------------------------------------------------------------
# Aba 3: simulação de coleta por clique (maquete ainda não tem câmera)
# ---------------------------------------------------------------------------
# Enquanto a parte de visão da maquete não fica pronta, esta aba deixa
# clicar num ponto de uma área "de coleta" e o braço mock finge que enxergou
# um item ali: aproxima, desce, fecha a garra, sobe, leva até o depósito e
# solta. Servos e faixas de ângulo abaixo são só um chute razoável pra testar
# a cinemática — ajuste quando a maquete física estiver montada.

AREAS_COLETA = {
    "frente": {
        "titulo": "Área frontal",
        "base": (60, 120),
        "ombro_hover": (100, 70),
        "cotovelo_hover": (60, 100),
    },
    "direita": {
        "titulo": "Área à direita",
        "base": (0, 55),
        "ombro_hover": (100, 70),
        "cotovelo_hover": (60, 100),
    },
}

GARRA_ABERTA = 30
GARRA_FECHADA = 130
DESCIDA_OMBRO_COTOVELO = 25
POS_DEPOSITO = {0: 90, 1: 80, 2: 110}

N_X_GRADE, N_Y_GRADE = 24, 14
_GRADE_COLETA = pd.DataFrame(
    [
        {"x": xi * 100 / (N_X_GRADE - 1), "y": yi * 100 / (N_Y_GRADE - 1)}
        for xi in range(N_X_GRADE)
        for yi in range(N_Y_GRADE)
    ]
)


def _interp(valor_0_100, faixa):
    ini, fim = faixa
    return ini + (fim - ini) * (valor_0_100 / 100)


def calcula_pose_coleta(area_id, x, y):
    cfg = AREAS_COLETA[area_id]
    base = _interp(x, cfg["base"])
    ombro_hover = _interp(y, cfg["ombro_hover"])
    cotovelo_hover = _interp(y, cfg["cotovelo_hover"])
    return base, ombro_hover, cotovelo_hover


def montar_grafico_coleta(alvo):
    fundo = (
        alt.Chart(pd.DataFrame([{"x0": 0, "x1": 100, "y0": 0, "y1": 100}]))
        .mark_rect(fill="#e8e4d8", stroke="#9c9a8f")
        .encode(x="x0:Q", x2="x1:Q", y="y0:Q", y2="y1:Q")
    )
    selecao = alt.selection_point(name="sel", fields=["x", "y"], nearest=True, on="click", empty=False)
    pontos = (
        alt.Chart(_GRADE_COLETA)
        .mark_circle(size=60, opacity=0.15, color="#52514e")
        .encode(
            x=alt.X("x:Q", scale=alt.Scale(domain=[0, 100]), axis=alt.Axis(title="lateral")),
            y=alt.Y("y:Q", scale=alt.Scale(domain=[0, 100]), axis=alt.Axis(title="distância")),
        )
        .add_params(selecao)
    )
    camadas = [fundo, pontos]
    if alvo is not None:
        marcador = (
            alt.Chart(pd.DataFrame([alvo]))
            .mark_point(size=260, color="#d62728", shape="cross", strokeWidth=3)
            .encode(x="x:Q", y="y:Q")
        )
        camadas.append(marcador)
    return alt.layer(*camadas).properties(height=280)


with aba_coleta:
    st.subheader("Simulação de coleta (clique na área)")
    st.caption(
        "Sem câmera ainda: clique num ponto de uma das áreas abaixo pra simular "
        "o braço enxergando um item ali e indo buscar. É só pra testar a "
        "cinemática enquanto a parte de visão da maquete não fica pronta."
    )

    if "coleta_alvo" not in st.session_state:
        st.session_state.coleta_alvo = {"frente": None, "direita": None}
    if "coleta_processado" not in st.session_state:
        st.session_state.coleta_processado = {"frente": None, "direita": None}
    if "coleta_pendente" not in st.session_state:
        st.session_state.coleta_pendente = None

    col_frente, col_direita = st.columns(2)
    for area_id, coluna in (("frente", col_frente), ("direita", col_direita)):
        with coluna:
            st.markdown(f"**{AREAS_COLETA[area_id]['titulo']}**")
            grafico = montar_grafico_coleta(st.session_state.coleta_alvo[area_id])
            evento = st.altair_chart(
                grafico, on_select="rerun", width="stretch", key=f"grafico_coleta_{area_id}"
            )
            pontos_sel = evento.selection.get("sel") if evento and evento.selection else None
            if pontos_sel:
                novo_alvo = {"x": pontos_sel[0]["x"], "y": pontos_sel[0]["y"]}
                if novo_alvo != st.session_state.coleta_processado[area_id]:
                    st.session_state.coleta_alvo[area_id] = novo_alvo
                    st.session_state.coleta_processado[area_id] = novo_alvo
                    st.session_state.coleta_pendente = (area_id, novo_alvo["x"], novo_alvo["y"])

    st.divider()
    status_coleta = st.empty()
    gauges_coleta = st.empty()

    def atualiza_gauges_coleta():
        snap = arm.snapshot()
        with gauges_coleta.container():
            colunas_gauge = st.columns(len(snap))
            for col, (sid, estado) in zip(colunas_gauge, snap.items()):
                with col:
                    render_gauge(sid, estado)

    def aguarda_braco_parar():
        while any(estado["em_movimento"] for estado in arm.snapshot().values()):
            atualiza_gauges_coleta()
            time.sleep(0.05)
        atualiza_gauges_coleta()

    def executa_coleta(area_id, x, y):
        titulo = AREAS_COLETA[area_id]["titulo"]
        base, ombro_hover, cotovelo_hover = calcula_pose_coleta(area_id, x, y)
        ombro_baixo = min(180, ombro_hover + DESCIDA_OMBRO_COTOVELO)
        cotovelo_baixo = min(180, cotovelo_hover + DESCIDA_OMBRO_COTOVELO)

        etapas = [
            ("Aproximando do item...", {0: base, 1: ombro_hover, 2: cotovelo_hover, 3: GARRA_ABERTA}),
            ("Descendo até o item...", {1: ombro_baixo, 2: cotovelo_baixo}),
            ("Fechando a garra...", {3: GARRA_FECHADA}),
            ("Subindo com o item...", {1: ombro_hover, 2: cotovelo_hover}),
            ("Levando até o depósito...", dict(POS_DEPOSITO)),
            ("Soltando o item...", {3: GARRA_ABERTA}),
        ]

        for texto, alvos in etapas:
            status_coleta.info(texto)
            for sid, angulo in alvos.items():
                arm.write(f"SET {sid} {angulo}\n".encode())
            aguarda_braco_parar()

        arm.write(b"HOME\n")
        aguarda_braco_parar()
        status_coleta.success(
            f"Item coletado em ({x:.0f}, {y:.0f}) na {titulo.lower()} e entregue no depósito!"
        )

    if st.session_state.coleta_pendente:
        area_pendente, x_pendente, y_pendente = st.session_state.coleta_pendente
        st.session_state.coleta_pendente = None
        executa_coleta(area_pendente, x_pendente, y_pendente)
    else:
        status_coleta.info("Clique em um ponto de uma das áreas acima para simular a coleta.")
        atualiza_gauges_coleta()
