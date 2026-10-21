#!/usr/bin/env bash
# Kick the CAN bus and Bluetooth back into shape without a reboot.
# Usage: scripts/reset_interfaces.sh [interface] [bitrate]
set -euo pipefail

interface="${1:-can0}"
bitrate="${2:-1000000}"

echo "Resetting ${interface} at ${bitrate} bit/s..."
sudo ip link set "${interface}" down || true
sudo ip link set "${interface}" up type can bitrate "${bitrate}" restart-ms 100
sudo ip link set "${interface}" txqueuelen 256

echo "Restarting Bluetooth..."
sudo systemctl restart bluetooth
sudo udevadm trigger

echo
echo "CAN status (watch rx_over_errors: nonzero means the HAT is dropping frames):"
ip -details -statistics link show "${interface}"
