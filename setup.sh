#!/usr/bin/env bash
# Setup completo do projeto no Raspberry Pi em modo headless.
# Roda uma vez (ou sempre que quiser): cria o venv, instala as
# dependências e já sobe o dashboard Streamlit na rede local.
#
# Uso:
#   chmod +x setup.sh   (só na primeira vez)
#   ./setup.sh

set -euo pipefail

DIR_PROJETO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$DIR_PROJETO"

echo "==> Projeto em: $DIR_PROJETO"

# ---------------------------------------------------------------------------
# 1) Dependências de sistema (apt) — picamera2/libcamera NÃO existem no pip,
#    têm que vir do sistema operacional.
# ---------------------------------------------------------------------------
PACOTES_APT="python3-picamera2 rpicam-apps"
FALTANDO=""
for pkg in $PACOTES_APT; do
    dpkg -s "$pkg" >/dev/null 2>&1 || FALTANDO="$FALTANDO $pkg"
done

if [ -n "$FALTANDO" ]; then
    echo "==> Instalando pacotes de sistema (apt):$FALTANDO"
    sudo apt-get update
    sudo apt-get install -y $FALTANDO
fi

# ---------------------------------------------------------------------------
# 2) Overlay da câmera no /boot/firmware/config.txt (Arducam Camera Module 3
#    / sensor imx708). Sem isso o libcamera não enxerga a câmera, mesmo com
#    todos os pacotes instalados.
# ---------------------------------------------------------------------------
CONFIG_TXT="/boot/firmware/config.txt"
if [ -f "$CONFIG_TXT" ]; then
    PRECISA_REBOOT=0
    if ! grep -q "^camera_auto_detect=0" "$CONFIG_TXT"; then
        echo "==> Ajustando camera_auto_detect em $CONFIG_TXT (backup em config.txt.bak)..."
        sudo cp "$CONFIG_TXT" "$CONFIG_TXT.bak"
        if grep -q "^camera_auto_detect=" "$CONFIG_TXT"; then
            sudo sed -i 's/^camera_auto_detect=.*/camera_auto_detect=0/' "$CONFIG_TXT"
        else
            echo "camera_auto_detect=0" | sudo tee -a "$CONFIG_TXT" >/dev/null
        fi
        PRECISA_REBOOT=1
    fi
    if ! grep -q "^dtoverlay=imx708,cam0" "$CONFIG_TXT"; then
        echo "==> Adicionando dtoverlay=imx708,cam0 em $CONFIG_TXT..."
        sudo cp -n "$CONFIG_TXT" "$CONFIG_TXT.bak" 2>/dev/null || true
        {
            echo ""
            echo "# Arducam Camera Module 3 12MP"
            echo "dtoverlay=imx708,cam0"
        } | sudo tee -a "$CONFIG_TXT" >/dev/null
        PRECISA_REBOOT=1
    fi
    if [ "$PRECISA_REBOOT" -eq 1 ]; then
        echo "AVISO: config.txt foi alterado. É preciso REINICIAR (sudo reboot) antes da câmera funcionar."
    fi
fi

# ---------------------------------------------------------------------------
# 3) Grupos do usuário (acesso a /dev/video* e à porta serial do braço).
# ---------------------------------------------------------------------------
GRUPOS_NECESSARIOS="video gpio dialout"
GRUPOS_ADICIONADOS=""
for grupo in $GRUPOS_NECESSARIOS; do
    if ! id -nG "$USER" | grep -qw "$grupo"; then
        sudo usermod -aG "$grupo" "$USER"
        GRUPOS_ADICIONADOS="$GRUPOS_ADICIONADOS $grupo"
    fi
done
if [ -n "$GRUPOS_ADICIONADOS" ]; then
    echo "AVISO: usuário adicionado aos grupos:$GRUPOS_ADICIONADOS. Faça logout/login (ou reboot) para valer."
fi

# ---------------------------------------------------------------------------
# 4) Ambiente virtual Python + dependências do requirements.txt
# ---------------------------------------------------------------------------
if [ ! -d "venv" ]; then
    echo "==> Criando venv (com acesso aos pacotes do sistema, para enxergar picamera2)..."
    python3 -m venv --system-site-packages venv
fi

source venv/bin/activate

echo "==> Atualizando pip..."
pip install --upgrade pip --quiet

echo "==> Instalando dependências do requirements.txt..."
pip install -r requirements.txt --quiet

if ! python3 -c "import picamera2" 2>/dev/null; then
    echo "AVISO: picamera2 ainda não importável no venv. Confira se o apt install acima rodou sem erro."
fi

# ---------------------------------------------------------------------------
# 5) Configuração do braço — avisa se ainda está com os valores de placeholder
# ---------------------------------------------------------------------------
mkdir -p config

if [ ! -f "config/braco.json" ]; then
    echo "AVISO: config/braco.json não existe. A coleta automática não vai funcionar."
elif grep -q '"calibrado": false' config/braco.json 2>/dev/null; then
    echo ""
    echo "AVISO: config/braco.json ainda está com valores de PLACEHOLDER."
    echo "       Meça os elos e ache os limites das juntas antes de mover o braço."
    echo "       Confira o alcance com: python mapa_alcance.py"
    echo ""
fi

if [ ! -f "config/homografia.json" ]; then
    echo "AVISO: câmera não calibrada (config/homografia.json não existe)."
    echo "       Rode 'python calibrar_camera.py' depois de fixar a câmera."
fi

IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
PORTA=8501

echo "==> Subindo o dashboard Streamlit (modo headless)..."
if [ -n "$IP" ]; then
    echo "==> Acesse de outro dispositivo na mesma rede: http://${IP}:${PORTA}"
fi

exec streamlit run app.py \
    --server.headless true \
    --server.address 0.0.0.0 \
    --server.port "$PORTA"
