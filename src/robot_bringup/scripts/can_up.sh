#!/usr/bin/env bash
# Bring up the SocketCAN interface. Needs sudo, so it lives outside the launch files.
# Usage:  ./can_up.sh [interface] [bitrate]
set -e
IFACE="${1:-can0}"
BITRATE="${2:-500000}"

# Down first: bitrate cannot be changed while the interface is up, and the
# combined "set up type can bitrate" form fails on some drivers (e.g. mttcan).
sudo ip link set "$IFACE" down 2>/dev/null || true
sudo ip link set "$IFACE" type can bitrate "$BITRATE"
sudo ip link set "$IFACE" up

echo "--- $IFACE ---"
ip -details link show "$IFACE"
