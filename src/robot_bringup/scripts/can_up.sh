#!/usr/bin/env bash
# Bring up the SocketCAN interface. Needs sudo, so it lives outside the launch files.
# Usage:  ./can_up.sh [interface] [bitrate]
set -e
IFACE="${1:-can0}"
BITRATE="${2:-500000}"

# Single combined form: on this robot's hardware the down/type/up split
# below stopped working, so we do it in one call. Kept for reference in case
# a driver switch someday needs the split form again -- bitrate cannot be
# changed while the interface is already up, so bring it down first, then:
#   sudo ip link set "$IFACE" type can bitrate "$BITRATE"
#   sudo ip link set "$IFACE" up
sudo ip link set "$IFACE" up type can bitrate "$BITRATE"

echo "--- $IFACE ---"
ip -details link show "$IFACE"
