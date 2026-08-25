from pathlib import Path

import cv2
from picamera2 import Picamera2
from ultralytics import YOLO

CAMINHO_MODELO = Path(__file__).resolve().parent.parent / "models" / "yolov8n.pt"
model = YOLO(str(CAMINHO_MODELO))

CLASSES_PERMITIDAS = [0, 67]  # 0 = pessoa | 67 = celular
INTERVALO_DETECCAO = 3        # roda YOLO a cada 3 frames

picam2 = Picamera2()
picam2.configure(
    picam2.create_preview_configuration(
        main={"format": "RGB888", "size": (640, 480)}
    )
)
picam2.start()

frame_count = 0
ultima_detec = None

while True:
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

        if melhor_objeto is not None and maior_confianca > 0.5:
            x1, y1, x2, y2 = map(int, melhor_objeto.xyxy[0])
            classe = int(melhor_objeto.cls[0])
            nome = model.names[classe]

            ultima_detec = (x1, y1, x2, y2, nome, maior_confianca)
        else:
            ultima_detec = None

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

    cv2.imshow("Camera", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

picam2.stop()
cv2.destroyAllWindows()