#!/usr/bin/env python3
"""
Desenha o mapa de alcance do braço — que pontos da mesa ele consegue pegar.

Não precisa de hardware nenhum: usa só as medidas do config/braco.json.
Rode logo depois de medir os elos, ANTES de montar a área de coleta em
definitivo. Ele responde duas perguntas que custam caro se descobertas
tarde:

    - a área de coleta está posicionada onde o braço realmente alcança?
    - o depósito está dentro do alcance?

Uso:
    python mapa_alcance.py
    python mapa_alcance.py --z 15 --passo 10
"""

import argparse
import sys
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(BASE_DIR))

from src.arm.kinematics import (  # noqa: E402
    PoseInalcancavel,
    carregar_config,
    classificar_alvo,
)

# Símbolos do mapa
OK = "#"        # alcançável
FORA_ENV = "."  # fora do envelope físico (braço curto/longo demais)
FORA_LIM = "x"  # dentro do envelope, mas alguma junta estoura o limite
BASE = "B"
DEPOSITO = "D"


# Símbolo do mapa para cada resultado de classificar_alvo()
SIMBOLO = {
    "ok": OK,
    PoseInalcancavel.ENVELOPE: FORA_ENV,
    PoseInalcancavel.LIMITE_JUNTA: FORA_LIM,
}


def classificar(braco, calib, x, y, z):
    return SIMBOLO[classificar_alvo(braco, calib, x, y, z)]


def main():
    ap = argparse.ArgumentParser(description="Mapa de alcance do braço")
    ap.add_argument("--z", type=float, default=None,
                    help="altura do plano a testar, em mm (padrão: z_pega do config)")
    ap.add_argument("--passo", type=float, default=20.0, help="resolução da grade, em mm")
    ap.add_argument("--config", default=str(BASE_DIR / "config" / "braco.json"))
    args = ap.parse_args()

    braco, calib, params, calibrado = carregar_config(args.config)
    z = args.z if args.z is not None else params["z_pega"]

    print("=" * 66)
    print("MAPA DE ALCANCE")
    print("=" * 66)
    print(f"elos: L1={braco.l1:.0f}  L2={braco.l2:.0f}  L3={braco.l3:.0f} mm"
          f"   offset radial: {braco.offset_radial:.0f} mm")
    print(f"plano testado: z = {z:.0f} mm   |   grade de {args.passo:.0f} mm")
    if not calibrado:
        print('\nAVISO: config marcada como "calibrado": false.')
        print("O mapa abaixo reflete os placeholders, não o braço real.")

    # Área de varredura: um pouco além do alcance máximo, dos dois lados
    limite = braco.alcance_max + braco.offset_radial + args.passo
    passo = args.passo

    ys, xs = [], []
    v = 0.0
    while v <= limite:
        ys.append(v)
        v += passo
    ys.reverse()  # y maior no topo, como se olhasse a mesa de cima

    v = -limite
    while v <= limite:
        xs.append(v)
        v += passo

    dep_x, dep_y = params["deposito"]

    print(f"\n  {BASE} = base do braço   {DEPOSITO} = depósito")
    print(f"  {OK} alcançável   {FORA_LIM} limite de junta   {FORA_ENV} fora do envelope\n")

    total = alcancaveis = 0
    linhas = []
    for y in ys:
        celulas = []
        for x in xs:
            if abs(x) < passo / 2 and abs(y) < passo / 2:
                celulas.append(BASE)
                continue
            if abs(x - dep_x) < passo / 2 and abs(y - dep_y) < passo / 2:
                celulas.append(DEPOSITO)
                continue
            c = classificar(braco, calib, x, y, z)
            total += 1
            if c == OK:
                alcancaveis += 1
            celulas.append(c)
        linhas.append(f"{y:6.0f} |" + " ".join(celulas))

    print("\n".join(linhas))
    print("       +" + "-" * (len(xs) * 2 - 1))

    # Régua do eixo X: um rótulo a cada 5 colunas, alinhado sob a coluna.
    # (marcar todas as colunas não cabe — os números se sobrepõem)
    marcas = [" "] * (len(xs) * 2 - 1)
    for i, x in enumerate(xs):
        if i % 5:
            continue
        rotulo = f"{x:.0f}"
        pos = i * 2 - len(rotulo) // 2
        pos = max(0, min(pos, len(marcas) - len(rotulo)))
        for k, ch in enumerate(rotulo):
            marcas[pos + k] = ch
    print("        " + "".join(marcas))
    print(f"        x em mm (base no 0), y de 0 a {ys[0]:.0f} mm")

    pct = 100.0 * alcancaveis / total if total else 0.0
    print(f"\nAlcançáveis: {alcancaveis}/{total} pontos da grade ({pct:.0f}%)")

    ok_dep = classificar(braco, calib, dep_x, dep_y, params["z_transporte"])
    if ok_dep == OK:
        print(f"Depósito ({dep_x:.0f}, {dep_y:.0f}) na altura de transporte: alcançável")
    else:
        print(f"PROBLEMA: depósito ({dep_x:.0f}, {dep_y:.0f}) NÃO é alcançável.")
        print("Ajuste 'deposito_xy' no config antes de tentar coletar.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
