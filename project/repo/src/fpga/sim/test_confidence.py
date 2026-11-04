# -*- coding: utf-8 -*-
"""src/fpga/sim/test_confidence.py —— confidence_engine + compositor 的 xsim 门禁。

为什么这一个文件覆盖**两个**模块
--------------------------------
方案 §12 给 `src/fpga/sim/` 排的四个文件名是 raw_preproc / vertical_diff /
fast_alert / confidence（见本目录 README）。五个 RTL 模块里剩 `compositor`
没有同名脚本，而 compositor 的强制灰屏路径正是由 confidence_engine 的
`force_gray` / `gray_reason` 驱动（方案 §6.6 那一条「强制灰屏时 compositor
必须整屏变灰并提示原因」把两者绑在一条链上）。故把 compositor 并入本文件，
在不新增第五个脚本名的前提下让四个脚本覆盖五个模块。

被验对象：
  * `src/fpga/rtl/confidence_engine.sv` + `tb/confidence_engine_tb.sv`
    三种融合方式（最小/连乘/等权平均）逐值、五项强制灰屏条件逐条命中对应位、
    gray_reason 是位图不是优先级编码、432 像素可信度图逐值、low_conf。
  * `src/fpga/rtl/compositor.sv` + `tb/compositor_tb.sv`
    整帧时序计数（de=2,073,600 / tlast=1080 / tuser=1 / ftick=1 / hs_low=49,500 /
    vs_low=11,000）、**整帧逐像素**金模型比对、灰屏覆盖（含边框优先）、退出灰屏恢复。

判据：各自的 TB 打印 PASS，且比对点数分别等于 **64** 与 **2,204,166**，无 `[ERR` 行。
两个模块**严格串行**（compositor 跑整帧，放最后），避免同机内存压力叠加。

比对点期望值出处：sim/results/xsim_first_run_20260929.md §1 / §4.4 / §4.5；
confidence_engine 由 58 改 64 的出处：T5 数据完整性硬闸门新增 6 个比对点
（A5，2026-10-02，见 sim/results/p0_algo_rtl_fix_20261002.md）。

用法：
    python src/fpga/sim/test_confidence.py
退出码：通过 0，失败非零。
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rtl_xsim_driver as drv   # noqa: E402

CASES = [
    ("confidence_engine", 64),
    ("compositor", 2204166),
]

if __name__ == "__main__":
    sys.exit(drv.cli(CASES))
