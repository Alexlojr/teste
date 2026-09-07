"""
Portão de estabilidade: decide QUANDO a coordenada de um objeto pode ser usada.

O YOLO devolve uma bounding box a cada inferência, mas nem toda detecção
serve para mandar o braço buscar. Duas coisas atrapalham:

**O objeto pode ainda estar se movendo.** Com o movimento suavizado, o braço
leva ~2 s para chegar ao alvo. Se as coordenadas forem tiradas enquanto o
objeto ainda escorrega, a garra fecha onde ele estava, não onde ele está.

**A caixa treme mesmo com o objeto parado.** Detecção é estatística: a mesma
cena, no frame seguinte, produz uma caixa alguns pixels diferente. Uma
detecção isolada carrega esse ruído inteiro para dentro da coordenada.

A saída resolve os dois de uma vez: guardar as últimas N posições e só
liberar quando todas couberem dentro de uma tolerância. Se couberem, o objeto
está parado E a mediana delas é uma coordenada melhor que qualquer uma
sozinha.

A tolerância é medida em MILÍMETROS, não em pixels, porque a grandeza que
importa é física — "o objeto se mexeu menos do que a folga da garra". O mesmo
número de pixels significa distâncias diferentes conforme a posição na mesa,
ainda mais com a câmera em perspectiva.

Uso:

    portao = PortaoEstabilidade(homografia, ParametrosEstabilidade())

    # a cada detecção:
    leitura = portao.observar(x1, y1, x2, y2)
    if leitura.estavel:
        controlador.executar(leitura.x_mm, leitura.y_mm)
        portao.rearmar()      # não recoletar o mesmo objeto
"""

from __future__ import annotations

import statistics
import time
from collections import deque
from dataclasses import dataclass


@dataclass
class ParametrosEstabilidade:
    """
    Ajuste com o objeto real na mesa: olhe `Leitura.dispersao_mm` com o objeto
    parado e escolha uma tolerância com folga sobre o ruído que aparecer.
    """

    # Quantas detecções seguidas precisam concordar. Poucas demais e um
    # objeto lento passa por parado; muitas demais e a espera fica longa.
    # Com o YOLO a ~6 FPS no Pi 5, 5 detecções são pouco menos de 1 segundo.
    janela: int = 5

    # Espalhamento máximo tolerado dentro da janela, em mm. Precisa ser maior
    # que o tremor natural da detecção e menor que a folga da garra.
    tolerancia_mm: float = 3.0

    # Tempo máximo esperando estabilizar antes de desistir e recomeçar. Sem
    # isto, um objeto que treme perto do limite prende o portão para sempre.
    timeout_s: float = 8.0

    # Uma detecção velha não conta para a janela: se o objeto sumiu por um
    # tempo e voltou, as posições de antes não dizem nada sobre agora.
    validade_amostra_s: float = 1.0

    @classmethod
    def de_json(cls, caminho="config/braco.json") -> "ParametrosEstabilidade":
        """
        Lê a seção "estabilidade" do braco.json. Ausente, usa os padrões —
        o portão funciona sem configuração, só não afinado para o objeto real.
        """
        import json
        from pathlib import Path

        dados = json.loads(Path(caminho).read_text(encoding="utf-8"))
        secao = dados.get("estabilidade", {})
        return cls(
            janela=int(secao.get("janela", cls.janela)),
            tolerancia_mm=float(secao.get("tolerancia_mm", cls.tolerancia_mm)),
            timeout_s=float(secao.get("timeout_s", cls.timeout_s)),
            validade_amostra_s=float(secao.get("validade_amostra_s", cls.validade_amostra_s)),
        )


@dataclass
class Leitura:
    """Resultado de uma observação."""

    estavel: bool
    x_mm: float | None          # mediana da janela, quando estável
    y_mm: float | None
    amostras: int               # quantas detecções válidas há na janela
    dispersao_mm: float | None  # maior distância entre amostras da janela
    motivo: str                 # legível, para mostrar na interface

    @property
    def coordenada(self) -> tuple[float, float] | None:
        return None if not self.estavel else (self.x_mm, self.y_mm)


class PortaoEstabilidade:
    def __init__(self, homografia, params: ParametrosEstabilidade | None = None,
                 relogio=time.monotonic):
        """
        `homografia` converte pixel em mm. `relogio` é injetável para os testes
        poderem controlar o tempo sem dormir de verdade.
        """
        self.homografia = homografia
        self.params = params or ParametrosEstabilidade()
        self._relogio = relogio
        self._amostras: deque = deque(maxlen=self.params.janela)
        self._t_primeira: float | None = None

    # ---- ciclo de vida --------------------------------------------------

    def rearmar(self) -> None:
        """
        Esquece tudo. Chame depois de coletar, ou quando o objeto sair de cena
        — senão a janela mistura o objeto que saiu com o que entrou, e a
        mediana dos dois cai num ponto onde não há objeto nenhum.
        """
        self._amostras.clear()
        self._t_primeira = None

    def perdeu_deteccao(self) -> None:
        """
        Um frame sem detecção. Não zera a janela na hora: o YOLO pisca, e
        perder uma inferência no meio de um objeto parado é comum. As amostras
        velhas caem sozinhas por `validade_amostra_s`.
        """
        self._descartar_velhas()

    # ---- observação -----------------------------------------------------

    def observar(self, x1: float, y1: float, x2: float, y2: float) -> Leitura:
        """
        Registra uma detecção e diz se já dá para usar a coordenada.

        A caixa entra em pixels, como o YOLO devolve. A conversão para mm usa
        a base da caixa (y2), que é onde o objeto encosta na mesa — o plano da
        homografia.
        """
        agora = self._relogio()
        x_mm, y_mm = self.homografia.caixa_para_mm(x1, y1, x2, y2)

        self._descartar_velhas()
        self._amostras.append((agora, x_mm, y_mm))

        if self._t_primeira is None:
            self._t_primeira = agora

        if len(self._amostras) < self.params.janela:
            return Leitura(
                False, None, None, len(self._amostras), None,
                f"juntando amostras ({len(self._amostras)}/{self.params.janela})",
            )

        dispersao = self._dispersao()

        if dispersao > self.params.tolerancia_mm:
            if agora - self._t_primeira > self.params.timeout_s:
                # Desistir e recomeçar em vez de esperar para sempre. Se o
                # objeto nunca assenta, insistir na mesma janela não melhora.
                self.rearmar()
                return Leitura(
                    False, None, None, 0, dispersao,
                    f"desistiu após {self.params.timeout_s:.0f}s sem estabilizar",
                )
            return Leitura(
                False, None, None, len(self._amostras), dispersao,
                f"ainda se movendo ({dispersao:.1f} mm > {self.params.tolerancia_mm:.1f} mm)",
            )

        # Mediana, não média: se uma das detecções saiu muito errada, a média
        # é puxada por ela e a mediana não.
        x = statistics.median(a[1] for a in self._amostras)
        y = statistics.median(a[2] for a in self._amostras)
        return Leitura(
            True, x, y, len(self._amostras), dispersao,
            f"estável ({dispersao:.1f} mm de dispersão)",
        )

    # ---- interno --------------------------------------------------------

    def _descartar_velhas(self) -> None:
        limite = self._relogio() - self.params.validade_amostra_s
        while self._amostras and self._amostras[0][0] < limite:
            self._amostras.popleft()
        if not self._amostras:
            self._t_primeira = None

    def _dispersao(self) -> float:
        """
        Maior distância entre dois pontos da janela.

        Usa o pior par, não o desvio padrão: o que interessa é a garantia de
        que NENHUMA amostra está longe, e o desvio padrão esconde um outlier
        único no meio de amostras boas.
        """
        pior = 0.0
        pontos = [(a[1], a[2]) for a in self._amostras]
        for i in range(len(pontos)):
            for j in range(i + 1, len(pontos)):
                dx = pontos[i][0] - pontos[j][0]
                dy = pontos[i][1] - pontos[j][1]
                d = (dx * dx + dy * dy) ** 0.5
                if d > pior:
                    pior = d
        return pior
