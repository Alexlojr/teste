"""
Orquestra a coleta: detecção do YOLO -> mm na mesa -> ângulos -> serial.

Este módulo é a cola entre visão e braço. Ele não abre câmera nem porta
serial: recebe as duas coisas prontas por injeção, o que deixa tudo
testável sem hardware (basta passar um mock com `.write()`).

A sequência de pega é a clássica de "top-down grasp":

    aproxima por cima -> desce -> fecha garra -> sobe -> leva -> solta

Descer em linha reta por cima é o que evita derrubar o objeto: se o braço
fosse direto para a pose final, o elo varreria lateralmente a mesa e
empurraria o alvo antes de fechar a garra.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable, Iterator

from .kinematics import Braco, CalibracaoBraco, PoseInalcancavel, pose_para_servos


@dataclass
class ParametrosColeta:
    """Alturas e tempos da sequência. Ajuste quando a maquete existir."""
    z_mesa: float = 0.0          # altura da superfície, em mm
    z_pega: float = 15.0         # altura da garra ao fechar (meia altura do objeto)
    z_aproximacao: float = 70.0  # altura do sobrevoo antes de descer
    z_transporte: float = 80.0   # altura ao carregar o objeto
    # (x, y) do depósito, em mm. Precisa cair dentro dos limites da base:
    # um ponto "atrás" do braço (y negativo com x≈0) exige base ≈ -90°, que
    # nenhum servo de 0-180° alcança. Mantenha o depósito à frente ou ao lado.
    deposito: tuple[float, float] = (-80.0, 100.0)
    # Pausa DEPOIS de o braço chegar, para a estrutura assentar. Não é mais
    # a espera do movimento em si: quem espera é o `aguardar_parada` da
    # serial, que pergunta ao firmware quando ele de fato parou.
    pausa_s: float = 0.25

    # Ângulos de servo da garra aberta e fechada. `None` usa o mínimo e o
    # máximo da junta, respectivamente.
    #
    # Estes valores são explícitos, e não deduzidos dos limites, por dois
    # motivos:
    #
    # 1. Qual extremo fecha a garra depende de como ela foi montada. Assumir
    #    "máximo fecha" é chute: montada ao contrário, a garra ABRE na hora de
    #    agarrar e o objeto nunca sai do lugar.
    # 2. Os objetos de teste são impressos com região de preensão dimensionada
    #    para um curso específico. Abrir sempre até o limite mecânico é mais
    #    do que o necessário, e fechar até o limite força o servo contra a
    #    peça depois de ela já estar retida pelo encaixe.
    garra_aberta: float | None = None
    garra_fechada: float | None = None


@dataclass
class Etapa:
    descricao: str
    comandos: list[str]


class ControladorColeta:
    def __init__(
        self,
        serial,
        braco: Braco | None = None,
        calib: CalibracaoBraco | None = None,
        params: ParametrosColeta | None = None,
    ):
        self.serial = serial
        self.braco = braco or Braco()
        self.calib = calib or CalibracaoBraco()
        self.params = params or ParametrosColeta()

    # ---- montagem da sequência (sem efeito colateral) --------------------

    def _cmds_pose(self, x: float, y: float, z: float) -> list[str]:
        canais = pose_para_servos(self.braco, self.calib, x, y, z)
        return [f"SET {canal} {ang}" for canal, ang in sorted(canais.items())]

    def _cmd_garra(self, fechada: bool) -> str:
        g = self.calib.garra
        p = self.params

        if fechada:
            angulo = p.garra_fechada if p.garra_fechada is not None else g.maximo
        else:
            angulo = p.garra_aberta if p.garra_aberta is not None else g.minimo

        # Um ângulo de garra fora dos limites da junta é erro de configuração,
        # não pose inalcançável — mas falha do mesmo jeito, e antes de mover.
        if not g.dentro_dos_limites(angulo):
            raise PoseInalcancavel(
                f"Garra {'fechada' if fechada else 'aberta'} em {angulo:.1f}°, "
                f"fora do limite [{g.minimo:.0f}, {g.maximo:.0f}]. "
                f"Confira 'garra_aberta_graus'/'garra_fechada_graus' na config.",
                motivo=PoseInalcancavel.LIMITE_JUNTA,
            )

        return f"SET {g.canal} {round(angulo, 1)}"

    def planejar(self, x: float, y: float) -> list[Etapa]:
        """
        Monta a sequência completa para pegar em (x, y) e levar ao depósito.

        Levanta PoseInalcancavel ANTES de mover qualquer servo se alguma
        pose da sequência for impossível — validar o plano inteiro antes de
        executar evita o braço parar no meio do movimento com o objeto preso
        na garra, que é bem pior de recuperar do que simplesmente recusar.
        """
        p = self.params
        dx, dy = p.deposito

        return [
            Etapa("Abrindo a garra", [self._cmd_garra(False)]),
            Etapa("Sobrevoando o alvo", self._cmds_pose(x, y, p.z_aproximacao)),
            Etapa("Descendo até o objeto", self._cmds_pose(x, y, p.z_pega)),
            Etapa("Fechando a garra", [self._cmd_garra(True)]),
            Etapa("Subindo com o objeto", self._cmds_pose(x, y, p.z_transporte)),
            Etapa("Levando ao depósito", self._cmds_pose(dx, dy, p.z_transporte)),
            Etapa("Soltando o objeto", [self._cmd_garra(False)]),
            Etapa("Voltando para HOME", ["HOME"]),
        ]

    def alcancavel(self, x: float, y: float) -> tuple[bool, str]:
        """Testa se dá para coletar em (x, y) sem mover nada."""
        try:
            self.planejar(x, y)
            return True, "alcançável"
        except PoseInalcancavel as e:
            return False, str(e)

    # ---- execução --------------------------------------------------------

    def executar(
        self,
        x: float,
        y: float,
        ao_progredir: Callable[[int, int, str], None] | None = None,
    ) -> Iterator[Etapa]:
        """
        Executa a coleta, rendendo cada etapa conforme acontece.

        É um gerador para que a interface possa mostrar progresso em tempo
        real sem que este módulo precise saber que o Streamlit existe.
        """
        etapas = self.planejar(x, y)   # valida tudo antes de mover
        total = len(etapas)

        for i, etapa in enumerate(etapas, start=1):
            if ao_progredir:
                ao_progredir(i, total, etapa.descricao)
            for cmd in etapa.comandos:
                self.serial.write((cmd + "\n").encode())

            # Espera o movimento TERMINAR, não um tempo fixo.
            #
            # O firmware suaviza cada movimento, então a duração depende da
            # distância percorrida: fechar a garra (100°) leva quase o triplo
            # de descer até o objeto (33°). Uma pausa fixa erra nos dois
            # sentidos, e o erro perigoso é o curto: a etapa seguinte começa
            # com a anterior pela metade, e a garra sobe antes de ter fechado.
            aguardar = getattr(self.serial, "aguardar_parada", None)
            if aguardar is not None:
                aguardar()

            # Depois de chegar, uma pausa curta para o braço assentar: o servo
            # para no ângulo, mas a estrutura de acrílico ainda oscila um
            # pouco. Vale principalmente antes de fechar a garra.
            time.sleep(self.params.pausa_s)
            yield etapa

    # ---- entrada vinda da visão -----------------------------------------

    def coletar_deteccao(self, homografia, caixa, ao_progredir=None) -> Iterator[Etapa]:
        """
        Ponta a ponta: bounding box do YOLO -> coleta executada.

        `caixa` é (x1, y1, x2, y2) em pixels, como o YOLO devolve.
        """
        x_mm, y_mm = homografia.caixa_para_mm(*caixa)
        yield from self.executar(x_mm, y_mm, ao_progredir=ao_progredir)
