#!/bin/sh
# Preserve PID 1 and the recovery console even when startup diagnostics fail.
mount -t proc proc /proc
mount -t sysfs sysfs /sys
mount -t devtmpfs devtmpfs /dev 2>/dev/null || true
mkdir -p /mnt/boot
mount -t vfat -o ro /dev/mmcblk1p1 /mnt/boot
modprobe uio_pdrv_genirq of_id=generic-uio
ip link set lo up
ip link set eth0 up 2>/dev/null || true
/bin/sh /home/root/ps_v5_runtime/r1/v5_boot.sh > /home/root/ps_v5_runtime/state/boot.log 2>&1 &
echo "PenumbraSight V5 recovery console; runtime preparation in background"
while :; do
 /bin/sh </dev/ttyPS0 >/dev/ttyPS0 2>&1
 echo "Console restarted; PID 1 preserved"
 sleep 1
done
