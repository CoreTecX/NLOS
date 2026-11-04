#!/bin/sh
set -eu
python3 /home/root/ps_board/pl_setup.py
echo 0 > /sys/class/fpga_manager/fpga0/flags
echo algorithm_fan_safe.bit > /sys/class/fpga_manager/fpga0/firmware
test "$(cat /sys/class/fpga_manager/fpga0/state)" = operating
python3 /home/root/ps_board/run_board.py --frame /mnt/boot/ps_data/frames/image_test_colbar_f000.psf --output /home/root/ps_board/boot_result
