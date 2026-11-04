#!/bin/sh
# PID 1 remains alive even when a child diagnostic shell exits.
mount -t proc proc /proc
mount -t sysfs sysfs /sys
mount -t devtmpfs devtmpfs /dev 2>/dev/null || true
mkdir -p /mnt/boot
mount /dev/mmcblk1p1 /mnt/boot
modprobe uio_pdrv_genirq of_id=generic-uio
ip link set lo up
ip link set eth0 up 2>/dev/null || true
/bin/sh /home/root/ps_board/autostart_linux.sh > /home/root/ps_board/autostart.log 2>&1 &
echo "PenumbraSight recovery console"
while :; do
 /bin/sh </dev/ttyPS0 >/dev/ttyPS0 2>&1
 echo "Console restarted; PID 1 preserved"
 sleep 1
done
