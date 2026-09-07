"""
Camada fina sobre o YOLO: escolher a detecção que interessa e desenhá-la.

Existe porque essa lógica estava copiada em três lugares — a dashboard, o
visualizador standalone e o pipeline de coleta. Três cópias da mesma regra
significam que ajustar o critério de confiança num deles e esquecer dos
outros faz o mesmo objeto ser aceito por um caminho e recusado por outro,
com o pipeline inteiro parecendo intermitente.

O critério é sempre o mesmo: uma detecção por frame, a de maior confiança
acima do limiar. O braço só pega um objeto de cada vez, então rastrear vários
não acrescentaria nada — e escolher "o mais confiante" é mais previsível do
que "o primeiro da lista", que depende da ordem interna do modelo.
"""

from __future__ import annotations

from dataclasses import dataclass

# Classes COCO, provisórias até o modelo ser treinado nos objetos do projeto.
# 0 = pessoa, 67 = celular. Servem para exercitar o pipeline inteiro antes de
# existir dataset próprio.
CLASSES_PERMITIDAS = [0, 67]
CONFIANCA_MINIMA = 0.5

# Roda o YOLO a cada N frames. Entre inferências o desenho reaproveita a
# última caixa, o que mantém a imagem fluida sem custar inferência.
INTERVALO_DETECCAO = 3


@dataclass
class Deteccao:
    x1: float
    y1: float
    x2: float
    y2: float
    classe: int
    confianca: float
    nome: str

    @property
    def caixa(self) -> tuple[float, float, float, float]:
        """No formato que a homografia espera."""
        return (self.x1, self.y1, self.x2, self.y2)

    @property
    def rotulo(self) -> str:
        return f"{self.nome} {self.confianca:.2f}"


def melhor_deteccao(resultado, nomes, confianca_minima: float = CONFIANCA_MINIMA):
    """
    A detecção de maior confiança acima do limiar, ou None.

    `resultado` é um item do que o Ultralytics devolve (results[0]); `nomes`
    é o `model.names`.
    """
    melhor = None
    maior = 0.0

    for caixa in resultado.boxes:
        conf = float(caixa.conf[0])
        if conf > maior:
            maior = conf
            melhor = caixa

    if melhor is None or maior < confianca_minima:
        return None

    x1, y1, x2, y2 = (float(v) for v in melhor.xyxy[0])
    classe = int(melhor.cls[0])
    return Deteccao(x1, y1, x2, y2, classe, maior, nomes[classe])


def desenhar(frame, deteccao: Deteccao, cor=(0, 255, 0)):
    """Desenha a caixa e o rótulo no frame, no lugar (modifica o array)."""
    import cv2

    x1, y1, x2, y2 = (int(v) for v in deteccao.caixa)
    cv2.rectangle(frame, (x1, y1), (x2, y2), cor, 2)
    cv2.putText(
        frame,
        deteccao.rotulo,
        (x1, y1 - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        cor,
        2,
    )
    return frame


def para_rgb(frame):
    """
    Converte o frame da Picamera2 para RGB.

    O formato "RGB888" da Picamera2 na verdade entrega os bytes em ordem BGR.
    Sem esta conversão, st.image() (que espera RGB) mostra tudo azulado.
    """
    import cv2

    return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
