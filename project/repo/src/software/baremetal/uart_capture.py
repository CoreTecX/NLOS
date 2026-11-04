#!/usr/bin/env python3
# ============================================================================
# PenumbraSight · 串口抓取助手（供 xsct_baremetal_run.tcl 调用）
# src/software/baremetal/uart_capture.py
#
# PS UART0 经板载 USB-串口出来，XSCT 无法直接抓（readjtaguart 只对 JTAG-UART 有效），
# 故用本脚本把串口按行写入文件，供主流程从 KEY=VALUE 行里取判据。
#
# 依赖：pyserial（pip install pyserial）
#
# 用法：
#   python uart_capture.py --port COM3 --baud 115200 --out uart.log --seconds 40
# 参数：
#   --port    串口号（Windows 形如 COM3；Linux 形如 /dev/ttyUSB0）
#   --baud    波特率，默认 115200
#   --out     输出去向文件
#   --seconds 最长抓取秒数（到时自停）
#   --sentinel 看到以它开头的行后退出（默认 DONE=）
#
# 未上板声明：本脚本未在真板上运行过。
# ============================================================================
import argparse
import sys
import time


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--out", required=True)
    ap.add_argument("--seconds", type=float, default=40.0)
    ap.add_argument("--sentinel", default="DONE=")
    args = ap.parse_args()

    try:
        import serial  # pyserial
    except ImportError:
        sys.stderr.write("缺少 pyserial：请先 `pip install pyserial`\n")
        return 2

    deadline = time.time() + args.seconds
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.5)
    except Exception as exc:  # noqa: BLE001
        sys.stderr.write(f"打开串口 {args.port} 失败：{exc}\n")
        return 3

    with open(args.out, "w", encoding="utf-8", errors="replace") as fout:
        sys.stderr.write(f"串口抓取中：{args.port}@{args.baud} -> {args.out}\n")
        while time.time() < deadline:
            try:
                raw = ser.readline()
            except Exception as exc:  # noqa: BLE001
                sys.stderr.write(f"读取中断：{exc}\n")
                break
            if not raw:
                continue
            line = raw.decode("ascii", errors="replace").rstrip("\r\n")
            fout.write(line + "\n")
            fout.flush()
            if line.startswith(args.sentinel):
                sys.stderr.write("看到结束标记，提前退出。\n")
                break
    try:
        ser.close()
    except Exception:  # noqa: BLE001
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
