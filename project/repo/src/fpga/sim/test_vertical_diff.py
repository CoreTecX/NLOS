# -*- coding: utf-8 -*-
"""src/fpga/sim/test_vertical_diff.py —— vertical_diff RTL 模块的 xsim 门禁。

被验对象：`src/fpga/rtl/vertical_diff.sv`，测试平台 `src/fpga/rtl/tb/vertical_diff_tb.sv`。
验什么（TB 内的逐值断言）：差分逐值（下行减上行）、输出坐标与块行末 tlast、
周期反压下拍数恰为 36（不丢拍不重拍）、旁路模式 48 拍逐值等于输入、
tch 串位故障注入使 proto_err 粘滞置起。
判据：TB 末尾 `=== vertical_diff_tb : PASS ===` 且比对点 **84**、无 `[ERR` 行。

比对点期望值 84 的出处：sim/results/xsim_first_run_20260929.md §1 与 §4.2
（T1 36 拍 + T2 48 拍）。**这个数被钉死**：TB 断言集缩小即 FAIL。

用法：
    python src/fpga/sim/test_vertical_diff.py
退出码：通过 0，失败非零。
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rtl_xsim_driver as drv   # noqa: E402

MODULE = "vertical_diff"
EXPECTED_POINTS = 84

if __name__ == "__main__":
    sys.exit(drv.cli([(MODULE, EXPECTED_POINTS)]))
