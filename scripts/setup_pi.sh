#!/usr/bin/env bash
# One-time Pi setup: the CAN bus at boot, then darm in its own virtualenv.
# Assumes the CAN HAT overlay is already in /boot/firmware/config.txt (see README).
set -euo pipefail

repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if ! ip link show can0 >/dev/null 2>&1; then
    echo "can0 does not exist yet: add the MCP2515 overlay to /boot/firmware/config.txt and reboot first." >&2
    exit 1
fi

echo "Installing the can0 service..."
sudo install -m 644 "${repo}/scripts/can0.service" /etc/systemd/system/can0.service
sudo systemctl daemon-reload
sudo systemctl enable --now can0.service

echo "Installing darm into ${repo}/.venv..."
python3 -m venv "${repo}/.venv"
"${repo}/.venv/bin/pip" install --quiet --upgrade pip
"${repo}/.venv/bin/pip" install --quiet -e "${repo}[gamepad]"

echo
echo "Done. Activate with:  source ${repo}/.venv/bin/activate"
echo "Then try:             darm monitor"
