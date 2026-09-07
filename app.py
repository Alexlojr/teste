"""
Dashboard de detecção — Raspberry Pi 5 + Camera Module 3 + YOLOv8n.

Mostra a câmera ao vivo com as detecções desenhadas e conta os objetos que
passam. É a camada de visão do braço separador: quando a homografia estiver
calibrada, é daqui que sai a bounding box que vira coordenada em mm.

O braço não é acionado por esta dashboard. Para movimentar servos durante a
montagem, use:

    python src/arm/mock_arm_serial.py    terminal interativo (SET/GET/HOME)
    python mapa_alcance.py               o que o braço alcança, sem hardware
"""

import time
from collections import defaultdict
from pathlib import Path

import streamlit as st
from picamera2 import Picamera2
from ultralytics import YOLO

from src.vision.deteccao import (
    CLASSES_PERMITIDAS,
    CONFIANCA_MINIMA,
    INTERVALO_DETECCAO,
    desenhar,
    melhor_deteccao,
    para_rgb,
)

BASE_DIR = Path(__file__).resolve().parent
CAMINHO_MODELO = BASE_DIR / "models" / "yolov8n.pt"

st.set_page_config(page_title="Detecção de Objetos", layout="wide")
st.title("Detecção de Objetos (YOLO)")


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
        resultado = model(frame, classes=CLASSES_PERMITIDAS, verbose=False)[0]
        ultima_detec = melhor_deteccao(resultado, model.names, CONFIANCA_MINIMA)

        if ultima_detec is not None:
            if st.session_state.ultimo_objeto != ultima_detec.nome:
                st.session_state.contagem[ultima_detec.nome] += 1
                st.session_state.ultimo_objeto = ultima_detec.nome
        else:
            st.session_state.ultimo_objeto = None

    if ultima_detec is not None:
        desenhar(frame, ultima_detec)
        status_placeholder.success(f"Detectando: {ultima_detec.rotulo}")
    else:
        status_placeholder.info("Nenhum objeto detectado")

    camera_placeholder.image(para_rgb(frame))

    tabela_placeholder.subheader("Contagem de objetos")

    if len(st.session_state.contagem) > 0:
        tabela_placeholder.table(dict(st.session_state.contagem))
    else:
        tabela_placeholder.write("Nenhum objeto contado ainda.")

    time.sleep(0.01)

if not st.session_state.rodando:
    st.info("Clique em **Iniciar câmera** na barra lateral.")
