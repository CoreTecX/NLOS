# -*- coding: utf-8 -*-
"""src/fpga/sim/test_fast_alert.py —— fast_alert RTL 模块的 xsim 门禁。

被验对象：`src/fpga/rtl/fast_alert.sv`，测试平台 `src/fpga/rtl/tb/fast_alert_tb.sv`。
验什么（TB 内的逐值断言）：E = Σ(c-μ)²/(v+ε) 整数 floor 除法**逐值精确**（E=64681）、
连续两帧判据（第 1 帧 0 / 第 2 帧 1 / 低 E 回落 0）、质心精确（1621 Q0.8）与方向 +1、
低能量时质心冻结不假报 0；另补 E 累加溢出（e_ovf，粘滞 + 饱和值逐值）、
v_k=0 的除零防护（E 通路 zero_den 保持 0）、proto_err 注入与粘滞、cfg_eps=0 -> cfg_err、
cfg_persist_en=0 的单帧判据。
判据：TB 末尾 `=== fast_alert_tb : PASS ===` 且比对点 **15**、无 `[ERR` 行。

比对点期望值 15 的出处：sim/results/xsim_first_run_20260929.md §1 与 §4.3 记首轮为 6 个；
2026-09-30 按上板前盘点（sim/results/preboard_port_status_20260930.md §一 1.1）补齐
T5 E 累加溢出 / T6 除零防护（v=0）/ T7 proto_err 注入 / T8 cfg_eps=0 被拒 /
T9 cfg_persist_en=0 单帧判据，共 +9 个比对点（详见 TB 文首覆盖说明）。**这个数被钉死**：
数值回归时点数不变但逐值断言会失败；若断言集被缩小，点数会掉。

用法：
    python src/fpga/sim/test_fast_alert.py
退出码：通过 0，失败非零。
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rtl_xsim_driver as drv   # noqa: E402

MODULE = "fast_alert"
EXPECTED_POINTS = 15

if __name__ == "__main__":
    sys.exit(drv.cli([(MODULE, EXPECTED_POINTS)]))
