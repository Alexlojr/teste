#!/usr/bin/env python3
"""
Pipeline de coleta: câmera -> YOLO -> espera assentar -> mm -> serial -> braço.

É a ligação de ponta a ponta que os módulos já permitiam mas ninguém chamava.
O laço é:

    captura frame
      -> YOLO detecta o objeto de maior confiança
      -> portão de estabilidade acumula detecções até o objeto parar
      -> homografia converte a caixa em milímetros na mesa
      -> cinemática inversa converte em ângulos
      -> sequência de coleta manda os SET pela serial
      -> rearma e espera o próximo objeto

Roda sem interface: numa produção o braço não deveria depender de um navegador
aberto. A dashboard (app.py) continua sendo só visualização.

USO

    python coletar.py --dry-run      mostra os comandos, não abre serial
    python coletar.py                coleta de verdade

O `--dry-run` é o passo 5 do roteiro de montagem. Rode ele antes de energizar
os servos, conferindo se as coordenadas e os ângulos fazem sentido.

PRÉ-REQUISITOS

    config/braco.json       com "calibrado": true (passos 1 e 2)
    config/homografia.json  gerado por calibrar_camera.py (passo 4)

Sem os dois, o script recusa a rodar — apontar o braço com números de
placeholder é como ele quebra engrenagem.
"""

import argparse
import sys
import time
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(BASE_DIR))

from src.arm.coleta import ControladorColeta, ParametrosColeta  # noqa: E402
from src.arm.kinematics import PoseInalcancavel, carregar_config  # noqa: E402
from src.vision.estabilidade import (  # noqa: E402
    ParametrosEstabilidade,
    PortaoEstabilidade,
)
from src.vision.deteccao import (  # noqa: E402
    CLASSES_PERMITIDAS,
    CONFIANCA_MINIMA,
    melhor_deteccao,
)
from src.vision.homografia import CalibracaoInvalida, Homografia  # noqa: E402

CAMINHO_CONFIG = BASE_DIR / "config" / "braco.json"
CAMINHO_HOMOGRAFIA = BASE_DIR / "config" / "homografia.json"
CAMINHO_MODELO = BASE_DIR / "models" / "yolov8n.pt"

# Depois de coletar, ignora detecções por este tempo. Sem isso o braço volta
# do depósito, vê o objeto seguinte (ou a sombra do que acabou de sair) e sai
# de novo antes de a cena assentar.
PAUSA_APOS_COLETA_S = 2.0


class SerialDeMentira:
    """Registra os comandos em vez de enviar. É o que o --dry-run usa."""

    def __init__(self):
        self.linhas = []
        self.braco_real = False
        self.port = "DRY-RUN"

    def write(self, dados: bytes) -> int:
        linha = dados.decode("utf-8", errors="ignore").strip()
        if linha:
            self.linhas.append(linha)
            print(f"    -> {linha}")
        return len(dados)


def abrir_camera(largura=640, altura=480):
    """Picamera2 no Pi; cai para webcam USB quando ela não existe."""
    try:
        from picamera2 import Picamera2

        cam = Picamera2()
        cam.configure(
            cam.create_preview_configuration(
                main={"format": "RGB888", "size": (largura, altura)},
                controls={"AwbEnable": True, "AwbMode": 0},
            )
        )
        cam.start()
        time.sleep(2)  # deixa o auto-exposure assentar antes do primeiro frame
        return lambda: cam.capture_array(), cam.stop

    except ImportError:
        import cv2

        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            raise RuntimeError("nenhuma câmera disponível (nem Picamera2, nem webcam)")

        def captura():
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError("falha ao capturar frame")
            return frame

        return captura, cap.release


def main() -> int:
    ap = argparse.ArgumentParser(description="Coleta automática ponta a ponta")
    ap.add_argument("--dry-run", action="store_true",
                    help="mostra os comandos sem abrir a serial")
    ap.add_argument("--forcar", action="store_true",
                    help="roda mesmo com a config marcada como não calibrada")
    ap.add_argument("--confianca", type=float, default=CONFIANCA_MINIMA)
    args = ap.parse_args()

    # ---- configuração ------------------------------------------------
    braco, calib, params_dict, calibrado = carregar_config(CAMINHO_CONFIG)

    if not calibrado and not args.forcar:
        print("ERRO: config/braco.json está com \"calibrado\": false.")
        print("      Os elos e limites ainda são placeholders — o braço apontaria")
        print("      para o lugar errado. Faça os passos 1 e 2 do README, ou use")
        print("      --forcar se souber o que está fazendo.")
        return 1

    try:
        homografia = Homografia.carregar(CAMINHO_HOMOGRAFIA)
    except CalibracaoInvalida as e:
        print(f"ERRO: {e}")
        return 1

    residual = homografia.meta.get("erro_residual_mm")
    if residual is not None and residual > 5.0:
        print(f"AVISO: a calibração da câmera tem erro residual de {residual:.1f} mm.")
        print("       Acima de ~5 mm costuma ser ponto mal medido. Considere refazer.")

    params_est = ParametrosEstabilidade.de_json(CAMINHO_CONFIG)

    # ---- serial ------------------------------------------------------
    if args.dry_run:
        serial = SerialDeMentira()
        print("MODO DRY-RUN: nada será enviado pela serial.\n")
    else:
        from src.arm.mock_arm_serial import MockArmSerial, detectar_porta_arduino

        porta = detectar_porta_arduino()
        serial = MockArmSerial(porta_arduino=porta)
        if serial.braco_real:
            print(f"Arduino encontrado em {serial.port}.")

            # Empurra os limites do JSON para o firmware. Sem isto eles vivem
            # em dois lugares e dessincronizam calados: alguém recalibra e
            # corrige o JSON, esquece de recompilar o sketch, e o firmware
            # segue permitindo um ângulo que já se sabe que bate.
            aceitas = serial.enviar_limites(calib)
            if aceitas == 4:
                print("Limites de junta enviados ao firmware.\n")
            else:
                print(f"AVISO: o firmware aceitou {aceitas}/4 limites. Ele pode ser")
                print("       uma versão antiga, sem o comando LIM. Os limites do")
                print("       sketch é que valem — confira se batem com o JSON.\n")
        else:
            print("AVISO: Arduino não encontrado. Rodando só na simulação.\n")

    controlador = ControladorColeta(
        serial, braco=braco, calib=calib, params=ParametrosColeta(**params_dict)
    )
    portao = PortaoEstabilidade(homografia, params_est)

    # ---- modelo e câmera ---------------------------------------------
    from ultralytics import YOLO

    modelo = YOLO(str(CAMINHO_MODELO))
    capturar, fechar_camera = abrir_camera()

    print(f"Portão: {params_est.janela} detecções dentro de "
          f"{params_est.tolerancia_mm:.1f} mm. Ctrl-C para sair.\n")

    ultima_coleta = 0.0
    coletados = 0

    try:
        while True:
            frame = capturar()

            if time.monotonic() - ultima_coleta < PAUSA_APOS_COLETA_S:
                continue

            resultado = modelo(frame, classes=CLASSES_PERMITIDAS, verbose=False)[0]
            deteccao = melhor_deteccao(resultado, modelo.names, args.confianca)

            if deteccao is None:
                portao.perdeu_deteccao()
                continue

            leitura = portao.observar(*deteccao.caixa)

            if not leitura.estavel:
                print(f"  {deteccao.rotulo} — {leitura.motivo}")
                continue

            x_mm, y_mm = leitura.x_mm, leitura.y_mm
            print(f"\n{modelo.names[classe]} assentado em "
                  f"({x_mm:.0f}, {y_mm:.0f}) mm — {leitura.motivo}")

            pode, motivo = controlador.alcancavel(x_mm, y_mm)
            if not pode:
                # Recusar e seguir observando: o objeto pode ser empurrado para
                # dentro do alcance, e travar o laço aqui não ajudaria ninguém.
                print(f"  fora de alcance: {motivo}\n")
                portao.rearmar()
                ultima_coleta = time.monotonic()
                continue

            try:
                for etapa in controlador.executar(x_mm, y_mm):
                    print(f"  {etapa.descricao}")
            except PoseInalcancavel as e:
                print(f"  coleta abortada: {e}")

            coletados += 1
            print(f"  concluída ({coletados} no total)\n")
            portao.rearmar()
            ultima_coleta = time.monotonic()

    except KeyboardInterrupt:
        print(f"\nEncerrando. {coletados} coleta(s) nesta sessão.")

    finally:
        fechar_camera()
        if hasattr(serial, "close"):
            serial.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
