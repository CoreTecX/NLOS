# -*- coding: utf-8 -*-
"""src/fpga/sim/test_raw_preproc.py —— raw_preproc RTL 模块的 xsim 门禁。

被验对象：`src/fpga/rtl/raw_preproc.sv`，测试平台 `src/fpga/rtl/tb/raw_preproc_tb.sv`。
验什么（TB 内的逐值断言，本脚本负责调用与严格判读）：
  彩色路径（4 帧 16x16 累积后逐值比对 m_meas_tdata / tch / tlast / tuser）、
  暗场扣除（负值饱和到 0）、黑白单通道路径、跨行分块负例自检。
判据：TB 末尾 `=== raw_preproc_tb : PASS ===` 且比对点 **112**、无 `[ERR` 行。

比对点期望值 112 的出处：sim/results/xsim_first_run_20260929.md §1 与 §4.1
（T1 48 拍 + T2 48 拍 + T3 16 拍）。**这个数被钉死**：TB 断言集缩小即 FAIL。

用法：
    python src/fpga/sim/test_raw_preproc.py
退出码：通过 0，失败非零。
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rtl_xsim_driver as drv   # noqa: E402

MODULE = "raw_preproc"
EXPECTED_POINTS = 112

if __name__ == "__main__":
    sys.exit(drv.cli([(MODULE, EXPECTED_POINTS)]))
