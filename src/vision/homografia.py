"""
Homografia: converte um pixel da imagem em coordenada real (mm) na mesa.

A câmera fica fixa, olhando de cima para a área de coleta. Como todos os
objetos estão sobre um mesmo plano (a mesa), a relação entre pixel e
milímetro é uma homografia — uma matriz 3x3 que se calcula a partir de
4 pontos de referência conhecidos.

Fluxo de uso:

    1. cole 4 marcadores na mesa em posições que você consiga MEDIR com
       régua em relação à base do braço;
    2. capture um frame e anote o pixel (u, v) de cada marcador;
    3. `Homografia.de_pontos(...)` e depois `.salvar()`;
    4. em produção, `Homografia.carregar()` e usar `.pixel_para_mm()`.

A calibração vale enquanto câmera, mesa e base do braço não se moverem.
Se qualquer um dos três for mexido, refaça — leva poucos minutos.
"""

from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np


class CalibracaoInvalida(Exception):
    pass


class Homografia:
    def __init__(self, matriz: np.ndarray, meta: dict | None = None):
        if matriz.shape != (3, 3):
            raise CalibracaoInvalida(f"matriz deve ser 3x3, veio {matriz.shape}")
        self.matriz = matriz.astype(np.float64)
        self.meta = meta or {}

    # ---- construção ----------------------------------------------------

    @classmethod
    def de_pontos(cls, pontos_pixel, pontos_mm, meta: dict | None = None) -> "Homografia":
        """
        pontos_pixel: 4+ pares (u, v) na imagem
        pontos_mm:    os MESMOS pontos, em mm, relativos à base do braço

        Com exatamente 4 pontos usa solução exata; com mais de 4 usa
        RANSAC, que tolera um ponto mal clicado.
        """
        px = np.asarray(pontos_pixel, dtype=np.float32)
        mm = np.asarray(pontos_mm, dtype=np.float32)

        if px.shape != mm.shape:
            raise CalibracaoInvalida("listas de pontos com tamanhos diferentes")
        if len(px) < 4:
            raise CalibracaoInvalida(f"precisa de pelo menos 4 pontos, veio {len(px)}")

        if len(px) == 4:
            matriz = cv2.getPerspectiveTransform(px, mm)
        else:
            matriz, _ = cv2.findHomography(px, mm, cv2.RANSAC, 5.0)
            if matriz is None:
                raise CalibracaoInvalida("não foi possível estimar a homografia")

        h = cls(matriz, meta)
        h.meta["erro_residual_mm"] = h._erro_residual(px, mm)
        return h

    def _erro_residual(self, pontos_pixel, pontos_mm) -> float:
        """Erro médio ao reprojetar os próprios pontos de calibração (mm)."""
        erros = [
            float(np.hypot(*(np.array(self.pixel_para_mm(u, v)) - np.array([xm, ym]))))
            for (u, v), (xm, ym) in zip(pontos_pixel, pontos_mm)
        ]
        return round(sum(erros) / len(erros), 3)

    # ---- uso ------------------------------------------------------------

    def pixel_para_mm(self, u: float, v: float) -> tuple[float, float]:
        """Pixel (u, v) -> (x, y) em mm, relativo à base do braço."""
        ponto = np.array([[[float(u), float(v)]]], dtype=np.float32)
        destino = cv2.perspectiveTransform(ponto, self.matriz)
        x, y = destino[0][0]
        return float(x), float(y)

    def caixa_para_mm(self, x1, y1, x2, y2) -> tuple[float, float]:
        """
        Converte uma bounding box do YOLO na posição de pega, em mm.

        Usa o centro horizontal e a BASE da caixa (y2), não o centro
        vertical: a base é onde o objeto encosta na mesa, que é o plano
        da homografia. Usar o centro superestima a distância em objetos
        altos, e o erro cresce com a altura do objeto.
        """
        return self.pixel_para_mm((x1 + x2) / 2.0, y2)

    # ---- persistência ----------------------------------------------------

    def salvar(self, caminho: str | Path) -> None:
        caminho = Path(caminho)
        caminho.parent.mkdir(parents=True, exist_ok=True)
        caminho.write_text(
            json.dumps({"matriz": self.matriz.tolist(), "meta": self.meta}, indent=2),
            encoding="utf-8",
        )

    @classmethod
    def carregar(cls, caminho: str | Path) -> "Homografia":
        caminho = Path(caminho)
        if not caminho.exists():
            raise CalibracaoInvalida(
                f"calibração não encontrada em {caminho}. "
                "Rode a rotina de calibração antes de usar a coleta automática."
            )
        dados = json.loads(caminho.read_text(encoding="utf-8"))
        return cls(np.array(dados["matriz"]), dados.get("meta", {}))
