# -*- coding: utf-8 -*-
"""src/fpga/sim/test_sat_ratio_div.py —— sat_ratio_div RTL 模块的 xsim 门禁。

被验对象：`src/fpga/rtl/sat_ratio_div.v`（**纯 Verilog，故 ext="v"**），
测试平台 `src/fpga/rtl/tb/sat_ratio_div_tb.sv`。

验什么（TB 内的逐值断言，对应 G5 的「饱和比例」那一项的除法器本体）：
  total==0 时不发布（保留上次值）；
  ratio_q016 = floor(sat * 2^16 / total)，含 vjustified/floor/饱和三类边界；
  0.1% 边界（66/65536 -> 66、67/65536 -> 67，CE 的阈值是 66）；
  sat == total -> 商恰为 2^16，超出 16 bit，饱和到 0xFFFF；
  B 档量级（1e6 / 14,745,600 -> 4444）；
  计数被清零后**保持**上一次发布值（raw_preproc 在 ST_DONE 清零的直接后果）；
  除法跑到一半改输入 -> 本次作废，只发布新输入对的值；
  连续变化（模拟 ST_ACC）期间不产生中间值；复位清零。

另有一条逐拍不变量（输入须稳定约 48 拍才允许发布）由 TB 内的监视器检查，
**不计入比对点**（理由见 TB 文件头：计入会让点数随激励拍数漂）。

判据：TB 末尾 `=== sat_ratio_div_tb : PASS ===` 且比对点 **21**、无 `[ERR` 行。
比对点期望值 21 的出处：本机 2026-09-30 Icarus Verilog 14.0 实跑（与 xsim 同结论），
见 sim/results/g5_g3_raw_fix_20261001.md。**这个数被钉死**：断言集缩小即 FAIL。

用法：
    python src/fpga/sim/test_sat_ratio_div.py
退出码：通过 0，失败非零。
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rtl_xsim_driver as drv   # noqa: E402

MODULE = "sat_ratio_div"
EXPECTED_POINTS = 21

if __name__ == "__main__":
    sys.exit(drv.cli([(MODULE, EXPECTED_POINTS)], ext="v"))
