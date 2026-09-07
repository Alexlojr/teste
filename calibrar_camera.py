#!/usr/bin/env python3
"""
Calibra a homografia câmera -> mesa.

Faça isso UMA VEZ depois de fixar câmera, mesa e base do braço — e refaça
sempre que qualquer um dos três se mexer.

Preparação física:
    1. marque 4 pontos na mesa, bem espalhados pela área de coleta
       (os cantos de uma folha A4 servem, se ela estiver toda visível);
    2. meça cada ponto com régua, em mm, relativo à BASE do braço:
         eixo +X = para a direita da base
         eixo +Y = para a frente da base
         a base é a origem (0, 0)
    3. anote os 4 pares (x, y).

Uso:
    python calibrar_camera.py            # clica nos pontos na imagem
    python calibrar_camera.py --manual   # digita os pixels na mão

O arquivo gerado (config/homografia.json) é lido pela aba de coleta.
"""

import argparse
import sys
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(BASE_DIR))

from src.vision.homografia import Homografia  # noqa: E402

SAIDA = BASE_DIR / "config" / "homografia.json"


def capturar_frame():
    """Um frame da Picamera2; cai para webcam USB se a Picamera2 não existir."""
    try:
        from picamera2 import Picamera2

        cam = Picamera2()
        cam.configure(
            cam.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)})
        )
        cam.start()
        import time

        time.sleep(2)  # deixa o auto-exposure estabilizar antes de capturar
        frame = cam.capture_array()
        cam.stop()
        return frame
    except ImportError:
        import cv2

        cap = cv2.VideoCapture(0)
        ok, frame = cap.read()
        cap.release()
        if not ok:
            raise RuntimeError("não consegui capturar frame de nenhuma câmera")
        return frame


def pedir_mm(n=4):
    print(f"\nDigite as coordenadas REAIS dos {n} pontos, em mm, relativas à base.")
    print("Formato: x,y   (ex: -100,40)\n")
    pontos = []
    for i in range(n):
        while True:
            try:
                bruto = input(f"  ponto {i + 1} (x,y em mm): ").strip()
                x, y = (float(v) for v in bruto.split(","))
                pontos.append((x, y))
                break
            except (ValueError, TypeError):
                print("    formato inválido, use x,y — ex: -100,40")
    return pontos


def coletar_por_clique(frame):
    import cv2

    cliques = []
    janela = "Clique nos 4 pontos de referencia (ESC cancela)"

    def ao_clicar(evento, x, y, flags, param):
        if evento == cv2.EVENT_LBUTTONDOWN and len(cliques) < 4:
            cliques.append((x, y))
            print(f"  ponto {len(cliques)}: pixel ({x}, {y})")

    cv2.namedWindow(janela)
    cv2.setMouseCallback(janela, ao_clicar)
    print("\nClique nos 4 pontos de referência, na ordem em que vai medi-los.")

    while True:
        vis = frame.copy()
        for i, (cx, cy) in enumerate(cliques, 1):
            cv2.circle(vis, (cx, cy), 6, (0, 0, 255), -1)
            cv2.putText(vis, str(i), (cx + 10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.imshow(janela, vis)

        tecla = cv2.waitKey(20) & 0xFF
        if tecla == 27:
            cv2.destroyAllWindows()
            return None
        if len(cliques) == 4:
            cv2.imshow(janela, vis)
            cv2.waitKey(400)
            break

    cv2.destroyAllWindows()
    return cliques


def coletar_manual():
    print("\nDigite os pixels dos 4 pontos, formato u,v")
    pontos = []
    for i in range(4):
        while True:
            try:
                bruto = input(f"  ponto {i + 1} (u,v em pixels): ").strip()
                u, v = (float(x) for x in bruto.split(","))
                pontos.append((u, v))
                break
            except (ValueError, TypeError):
                print("    formato inválido, use u,v — ex: 320,240")
    return pontos


def main():
    ap = argparse.ArgumentParser(description="Calibra a homografia câmera->mesa")
    ap.add_argument("--manual", action="store_true", help="digitar pixels em vez de clicar")
    ap.add_argument("--saida", default=str(SAIDA), help="onde salvar o JSON")
    args = ap.parse_args()

    print("=" * 62)
    print("CALIBRAÇÃO DA HOMOGRAFIA CÂMERA -> MESA")
    print("=" * 62)

    if args.manual:
        pixels = coletar_manual()
    else:
        print("\nCapturando frame da câmera...")
        try:
            frame = capturar_frame()
        except Exception as e:
            print(f"\nNão consegui usar a câmera ({e}).")
            print("Rode com --manual para digitar os pixels na mão.")
            return 1
        pixels = coletar_por_clique(frame)
        if pixels is None:
            print("\nCancelado.")
            return 1

    milimetros = pedir_mm(len(pixels))

    try:
        h = Homografia.de_pontos(pixels, milimetros, meta={"pixels": pixels, "mm": milimetros})
    except Exception as e:
        print(f"\nFalha ao calcular a homografia: {e}")
        return 1

    erro = h.meta["erro_residual_mm"]
    print(f"\nErro residual: {erro} mm")
    if erro > 5:
        print("  ATENÇÃO: erro alto. Provavelmente algum ponto foi clicado ou")
        print("  medido errado. Vale refazer antes de usar para valer.")
    else:
        print("  Erro dentro do esperado.")

    h.salvar(args.saida)
    print(f"\nSalvo em: {args.saida}")

    print("\nConferência — reprojetando os pontos de calibração:")
    for (u, v), (xm, ym) in zip(pixels, milimetros):
        x, y = h.pixel_para_mm(u, v)
        print(f"  pixel({u:.0f},{v:.0f}) -> ({x:7.1f},{y:7.1f}) mm | medido ({xm},{ym})")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
