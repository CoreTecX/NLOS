#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""工程化一致性静态检查（无 Vivado / 无 Vitis 时能做的那部分）。

本脚本把 2026-10-02 那批「工程化与文档缺陷」的修复变成可机判的门禁，覆盖四项：

  E2 时钟单一来源：src/fpga/scripts/clock_plan.tcl 是周期的唯一定义处；
     build_hls.tcl / penumbrasight_bd.tcl / synth_rtl_probe.tcl 都 source 它、
     不再写 10.0 字面量；axu9eg.xdc 不再重复该字面量；
     configs/fixed_point_b.yaml 的 target_clock_mhz 与之一致（100）。
  E8 ps_axi_top.cpp 的 m_axi depth 不小于 B 档真实尺寸（与 ps_fixed_top.cpp 同源）。
  E5/E6 块设计里 PL->PS 中断通路存在（三个 DMA introut 经 xlconcat(8) 进 pl_ps_irq0）。
  E3  build_pl.tcl 无条件 reset_run（不再「PROGRESS!=100% 才 reset」）。
  E7  已无调用者的 ps_const / ps_tie 过程已删除。

只做文本层判据，够用即可：纯静态、可复现、不依赖任何工具链。
退出码：0 = 全部满足；1 = 有项不满足。
"""

import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))          # src/fpga/scripts
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))

CLOCK_PLAN = os.path.join(REPO, "src", "fpga", "scripts", "clock_plan.tcl")
BUILD_HLS = os.path.join(REPO, "src", "fpga", "hls", "scripts", "build_hls.tcl")
BD_TCL = os.path.join(REPO, "src", "fpga", "bd", "penumbrasight_bd.tcl")
XDC = os.path.join(REPO, "src", "fpga", "constraints", "axu9eg.xdc")
PROBE = os.path.join(REPO, "src", "fpga", "scripts", "synth_rtl_probe.tcl")
YAML = os.path.join(REPO, "configs", "fixed_point_b.yaml")
AXI_CPP = os.path.join(REPO, "src", "fpga", "hls", "ps_axi_top.cpp")
BUILD_PL = os.path.join(REPO, "src", "fpga", "scripts", "build_pl.tcl")

# B 档真实尺寸（元素数），出处 psm_format.h:49-58 / configs/geometry_b.yaml：
# 用 ps_fixed_top.cpp 已更正的 depth 口径。
B_MEAS = 9120          # 95 x 96
B_N = 432              # 18 x 24
B_P = 243              # 9 x 9 x 3
B_KM = 583680          # K(64) x M
B_KN = 27648           # K(64) x N
B_3M = 3 * B_MEAS      # 27360
# 整个 model.psm 体（按字节）：64 头 + 243 条 x 1,236,992 字节
B_MODEL_BODY = 64 + B_P * 1236992     # = 300,589,120


def read(path):
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        return fh.read()


class Checker:
    def __init__(self):
        self.fails = []

    def ok(self, cond, msg):
        print("  [%s] %s" % ("ok  " if cond else "FAIL", msg))
        if not cond:
            self.fails.append(msg)

    def run(self):
        print("==== 工程化一致性静态检查（无 Vivado / Vitis）====")
        self._clock()
        self._axi_depth()
        self._interrupt()
        self._build_pl_reset()
        self._dead_procs()
        self._script_defaults()
        print("  全部满足。" if not self.fails else "  有 %d 项不满足，需修。" % len(self.fails))
        return 0 if not self.fails else 1

    # ---------------------------------------------------------------- E2
    def _clock(self):
        print("\n-- E2 时钟单一来源 --")
        cp = read(CLOCK_PLAN)
        m = re.search(r"set\s+PS_PL_CLK_PERIOD_NS\s+([0-9.]+)", cp)
        self.ok(m is not None, "clock_plan.tcl 定义了 PS_PL_CLK_PERIOD_NS")
        period = float(m.group(1)) if m else 0.0
        self.ok(abs(period - 10.0) < 1e-9,
                "clock_plan.tcl 的默认周期 = 10.0 ns（100 MHz），实得 %s" % period)

        hls = read(BUILD_HLS)
        self.ok("clock_plan.tcl" in hls, "build_hls.tcl source 了 clock_plan.tcl")
        self.ok("$PS_PL_CLK_PERIOD_NS" in hls, "build_hls.tcl 的 CLK_PERIOD 取自单一来源")
        self.ok(re.search(r"set\s+CLK_PERIOD\s+10", hls) is None,
                "build_hls.tcl 不再写 CLK_PERIOD 10 字面量")

        bd = read(BD_TCL)
        self.ok("clock_plan.tcl" in bd, "penumbrasight_bd.tcl source 了 clock_plan.tcl")
        self.ok("$PS_PL_CLK_PERIOD_NS" in bd, "BD 的 CLK_PERIOD_NS 取自单一来源")
        self.ok(re.search(r"set\s+CLK_PERIOD_NS\s+10", bd) is None,
                "BD 不再写 CLK_PERIOD_NS 10 字面量")

        xdc = read(XDC)
        self.ok(re.search(r"set\s+PL_CLK_PERIOD_NS\s+[0-9]", xdc) is None,
                "axu9eg.xdc 不再重复 PL_CLK_PERIOD_NS 字面量")

        probe = read(PROBE)
        self.ok("clock_plan.tcl" in probe, "synth_rtl_probe.tcl source 了 clock_plan.tcl")
        self.ok(re.search(r"set\s+CLK_PERIOD\s+10", probe) is None,
                "synth_rtl_probe.tcl 不再写 CLK_PERIOD 10 字面量")

        yml = read(YAML)
        m = re.search(r"target_clock_mhz:\s*(\d+)", yml)
        self.ok(m is not None and int(m.group(1)) == 100,
                "fixed_point_b.yaml 的 target_clock_mhz = 100（与单一来源一致）")

    # ---------------------------------------------------------------- E8
    def _axi_depth(self):
        print("\n-- E8 ps_axi_top.cpp 的 m_axi depth >= B 档真实尺寸 --")
        src = read(AXI_CPP)
        depths = {}
        for m in re.finditer(r"port=(\w+)\s+offset=slave\s+bundle=\w+\s+depth=(\d+)", src):
            depths.setdefault(m.group(1), []).append(int(m.group(2)))

        def all_ge(port, need):
            vals = depths.get(port, [])
            good = len(vals) > 0 and all(v >= need for v in vals)
            self.ok(good, "port=%-10s depth %s 全部 >= %d" % (port, vals, need))

        all_ge("U", B_KM)
        all_ge("V", B_KN)
        all_ge("meas3", B_3M)
        all_ge("d", B_3M)
        all_ge("model_base", B_MODEL_BODY)
        all_ge("h", 64)
        self.ok(depths.get("S") == [64, 64, 64],
                "port=S depth 恒为 64（本层 K 上界 PS_RANK_MAX=64）")

    # ---------------------------------------------------------------- E5/E6
    def _interrupt(self):
        print("\n-- E5/E6 PL->PS 中断通路 --")
        bd = read(BD_TCL)
        for token in ("irq_concat", "irq_gnd", "pl_ps_irq0",
                      "dma_pipe/mm2s_introut", "dma_pipe/s2mm_introut",
                      "dma_disp/s2mm_introut", "CONFIG.NUM_PORTS {8}"):
            self.ok(token in bd, "BD 含中断通路要素：%s" % token)

    # ---------------------------------------------------------------- E3
    def _build_pl_reset(self):
        print("\n-- E3 build_pl.tcl 无条件 reset --")
        t = read(BUILD_PL)
        self.ok("reset_run $r" in t, "build_pl.tcl 调用了 reset_run $r")
        self.ok(re.search(r'PROGRESS\s+\[get_runs\s+\$r\]\]\s+ne\s+"100%"', t) is None,
                "build_pl.tcl 不再以 PROGRESS!=100% 作为 reset 条件")

    # ---------------------------------------------------------------- E7
    def _dead_procs(self):
        print("\n-- E7 无调用者的 ps_const / ps_tie 已删 --")
        bd = read(BD_TCL)
        self.ok("proc ps_const" not in bd, "BD 已无 proc ps_const")
        self.ok("proc ps_tie" not in bd, "BD 已无 proc ps_tie")

    # ---------------------------------------------------------------- E4
    def _script_defaults(self):
        print("\n-- E4 脚本默认值：环境变量优先、默认值保留 --")
        drv = read(os.path.join(REPO, "src", "fpga", "sim", "rtl_xsim_driver.py"))
        self.ok('os.environ.get("PS_VIVADO_BIN"' in drv,
                "rtl_xsim_driver.py 从 PS_VIVADO_BIN 取 Vivado 路径")
        self.ok(r"D:\vivado\2025.2\Vivado\bin" in drv,
                "rtl_xsim_driver.py 仍保留本机默认值（默认可跑）")

        bit = read(os.path.join(REPO, "src", "fpga", "sim",
                                "test_raw_preproc_bitexact.py"))
        self.ok('os.environ.get("PS_IVERILOG_BIN"' in bit,
                "test_raw_preproc_bitexact.py 从 PS_IVERILOG_BIN 取 iverilog 路径")
        self.ok(r"D:\iverilog\bin" in bit,
                "test_raw_preproc_bitexact.py 仍保留本机默认值")

        rx = read(os.path.join(REPO, "src", "fpga", "sim", "run_xsim.ps1"))
        self.ok("$env:PS_VIVADO_BIN" in rx and "$env:XILINXD_license_FILE" in rx,
                "run_xsim.ps1 从环境变量取 Vivado/License 路径")
        self.ok(r"D:\xilinx_lic" in rx,
                "run_xsim.ps1 仍保留本机默认值（默认可跑）")

        rb = read(os.path.join(REPO, "src", "fpga", "scripts", "run_bitstream.ps1"))
        self.ok("$env:PS_BUILD_REPO" in rb and "$env:PS_XILINX_ROOT" in rb,
                "run_bitstream.ps1 从环境变量取构建区/Xilinx 路径")
        self.ok(r"D:\psight\build_t2" in rb,
                "run_bitstream.ps1 仍保留本机默认值（默认可跑）")


if __name__ == "__main__":
    sys.exit(Checker().run())
