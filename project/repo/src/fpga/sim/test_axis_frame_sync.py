# -*- coding: utf-8 -*-
"""src/fpga/sim/test_axis_frame_sync.py —— axis_frame_sync RTL 模块的 xsim 门禁。

被验对象：`src/fpga/rtl/axis_frame_sync.v`（**纯 Verilog，故 ext="v"**），
测试平台 `src/fpga/rtl/tb/axis_frame_sync_tb.sv`。
验什么（TB 内的逐值断言，对应任务一的 G2 修复）：
  差分通道（ROW_BEATS=0）：一帧 36 拍里 tlast 握手**恰为 1 次**，且上游按
    vertical_diff 的真实行为每块行（12 拍）置一次 tlast —— 证明它不会漏到输出；
  raw 通道（ROW_BEATS=4）：每帧 16 拍里 tlast 恰为 4 次（每行一次）；
  两个通道都带周期性反压，且每一拍都断言「tvalid=0 时 tlast/tuser 必须为 0」。
判据：TB 末尾 `=== axis_frame_sync_tb : PASS ===` 且比对点 **145**、无 `[ERR` 行。

比对点期望值 145 的出处：本机 2026-09-30 实跑（Vivado xsim 2025.2 与 Icarus Verilog
14.0 两套，取值一致），见 sim/results/g2_g1_g4_fix_20260930.md。
145 = 108 个差分握手拍 + 32 个 raw 握手拍 + 5 个帧级检查。
「无效拍不误置」是逐拍不变量，**不计入**该数（否则会随仿真器在激励总拍数上的差异漂）。
**这个数被钉死**：TB 断言集缩小即 FAIL。

用法：
    python src/fpga/sim/test_axis_frame_sync.py
退出码：通过 0，失败非零。
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rtl_xsim_driver as drv   # noqa: E402

MODULE = "axis_frame_sync"
EXPECTED_POINTS = 145

if __name__ == "__main__":
    sys.exit(drv.cli([(MODULE, EXPECTED_POINTS)], ext="v"))
