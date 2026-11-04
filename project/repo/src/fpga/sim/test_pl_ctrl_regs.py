# -*- coding: utf-8 -*-
"""src/fpga/sim/test_pl_ctrl_regs.py —— pl_ctrl_regs RTL 模块的 xsim 门禁。

被验对象：`src/fpga/rtl/pl_ctrl_regs.v`（**纯 Verilog，故 ext="v"**），
测试平台 `src/fpga/rtl/tb/pl_ctrl_regs_tb.sv`。
验什么（TB 内的逐值断言，对应任务二/三的 G1 + G4 + G5 落地）：
  复位默认值 = 上一版 xlconstant 的钉扎值（PS 不写寄存器时行为不变）；
  cfg 寄存器写-读回环且**标量输出脚真的跟着变**；
  写 CMP_WALL_DATA / CMP_REC_DATA / CMP_OSD_DATA 各产生**恰好一拍**本地写使能
  （G4 桥），只写地址寄存器不产生脉冲；
  写 FA_COEF_PUSH 产生恰好一拍 coef_tvalid（c_k 流来源，G5）；
  μ/v 背景表按 FA_BG_ADDR 落表、按 bg_raddr 同步 1 拍读出（G5）；
  饱和比例的 PL 内来源（CE_SAT_DIV / CE_CFG bit3 二选一 / SATSTAT 比较位，G5 收口）；
  掩膜表 432x8 RAM（写 MASK_ADDR/MASK_DATA 后按 ce_mask_raddr 同步读出，G5 收口）；
  状态寄存器逐位回读、帧计数、CTRL 重载默认值、WSTRB 字节选通。
判据：TB 末尾 `=== pl_ctrl_regs_tb : PASS ===` 且比对点 **90**、无 `[ERR` 行。

比对点期望值 90 的出处：本机 Icarus Verilog 14.0 与 Vivado xsim 各实跑一次
（两套一致），见 sim/results/g5_g3_raw_fix_20261001.md。
**这个数被钉死**：TB 断言集缩小即 FAIL。
（64 是 G2/G1/G4 那一轮的旧值；G5 收口加 T10/T10b/T11 到 83；
2026-10-01 再加 T12/T13 共 7 个比对点 -> 90：margin/rho/snr 标量写寄存器后
CE 输入脚跟随，以及 geom_hash_ok 由 CE_CFG bit2 控制。这两组都只到
**寄存器层**，不是 PL 内自动来源，见报告。）

额外源文件：pl_ctrl_regs 内部实例化 `sat_ratio_div`，故必须把
sat_ratio_div.v 一起交给 xvlog —— 见 rtl_xsim_driver.run_module 的 extra 参数。

用法：
    python src/fpga/sim/test_pl_ctrl_regs.py
退出码：通过 0，失败非零。
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rtl_xsim_driver as drv   # noqa: E402

MODULE = "pl_ctrl_regs"
EXPECTED_POINTS = 90
EXTRA = ["sat_ratio_div.v"]

if __name__ == "__main__":
    sys.exit(drv.cli([(MODULE, EXPECTED_POINTS)], ext="v", extra=EXTRA))
