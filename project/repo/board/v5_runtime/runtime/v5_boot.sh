#!/bin/sh
# Explicit, versioned startup; diagnostics do not reconstruct the real scene.
set -eu
ROOT=/home/root/ps_v5_runtime/r1
BOOT=$(cat /proc/sys/kernel/random/boot_id)
/usr/bin/python3.7 -B "$ROOT/v5_runtime.py" --expect-boot "$BOOT" prepare
echo 'V5_STAGE waiting for monitor framebuffer and camera Ethernet link (up to 90 seconds)'
COUNT=0
while [ ! -e /dev/fb0 ] || [ "$(cat /sys/class/net/eth0/carrier 2>/dev/null)" != 1 ]; do
 COUNT=$((COUNT + 1))
 if [ "$COUNT" -ge 90 ]; then
  echo 'DISPLAY_OR_LINK_NOT_READY: check monitor power/input/cable and camera Ethernet; use start-preview after reconnecting' >&2
  exit 1
 fi
 sleep 1
done
/usr/bin/python3.7 -B "$ROOT/v5_runtime.py" --expect-boot "$BOOT" start-preview --fps 8
