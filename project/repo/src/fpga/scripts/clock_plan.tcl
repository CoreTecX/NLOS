# ============================================================================
# PenumbraSight PL 时钟计划 · src/fpga/scripts/clock_plan.tcl
#
# 用途：PL 时钟的**单一来源**。凡是要用 PL 时钟周期/频率的脚本都 source 本文件，
#       不再各自写一个字面量。
#
# 为什么需要它（2026-10-02）
# ----------------------------------------------------------------------------
# 此前同一个 10.0 ns 写在三处，靠注释「三处一起改」维持一致：
#     src/fpga/hls/scripts/build_hls.tcl        CLK_PERIOD = 10.0
#     src/fpga/bd/penumbrasight_bd.tcl          CLK_PERIOD_NS = 10.0
#     src/fpga/constraints/axu9eg.xdc           PL_CLK_PERIOD_NS = 10.000
# 「靠注释约定」不是机制：漏改一处不会有任何报错，只会让 OOC 综合与
# 块设计/实现各按各的周期约束，结果对不上。现改为都从这里取。
#
# 取值依据（X19 实测，2026-09-28；同时记在 build_hls.tcl:43-47 与
# penumbrasight_bd.tcl:325-329）：
#   200 MHz  ps_projection_fix Slack -1.90、ps_pdhg_fix -1.39   -> 违例
#   150 MHz  ps_pdhg_fix 仍 -0.51
#   100 MHz  四个定点核 Slack 全 0.00                            -> 全部收敛
# 即 100 MHz 是实测选定的档位（不是板上限）。
#
# 用法
# ----------------------------------------------------------------------------
#   source [file join $REPO_ROOT src fpga scripts clock_plan.tcl]
#   # 之后可用 $PS_PL_CLK_PERIOD_NS（纳秒）与 $PS_PL_CLK_MHZ（整数 MHz）
#
# 覆盖（可选，便于扫周期做对照，不必改文件）：
#   set PS_PL_CLK_PERIOD_NS=6.667 && vitis-run ...
#   旧名 PS_HLS_CLK_PERIOD 仍接受，保持既有文档 / makefile 用法不失效。
#
# 边界说明
# ----------------------------------------------------------------------------
#   本文件是纯 Tcl 变量赋值，不含任何 Vivado 工程/约束命令，可被
#   build_hls.tcl（Vitis）与 penumbrasight_bd.tcl（Vivado）共用。
#   axu9eg.xdc 是约束文件，Vivado **不允许**在 xdc 里 source 任意 Tcl
#   （见该文件里 current_project / if 被拒的实测记录），故 xdc 侧不再重复
#   这个字面量，只留指向本文件的说明。
# ============================================================================

# 默认 PL 时钟周期（ns）。10.0 ns = 100 MHz，依据见文首。
set PS_PL_CLK_PERIOD_NS 10.0

# 环境变量覆盖：新名优先，兼容旧名 PS_HLS_CLK_PERIOD。
if {[info exists ::env(PS_PL_CLK_PERIOD_NS)] && $::env(PS_PL_CLK_PERIOD_NS) ne ""} {
    set PS_PL_CLK_PERIOD_NS $::env(PS_PL_CLK_PERIOD_NS)
} elseif {[info exists ::env(PS_HLS_CLK_PERIOD)] && $::env(PS_HLS_CLK_PERIOD) ne ""} {
    set PS_PL_CLK_PERIOD_NS $::env(PS_HLS_CLK_PERIOD)
}

# 整数 MHz（用于 PS 的 PL0_REF_CTRL__FREQMHZ，取整到 MHz）。
set PS_PL_CLK_MHZ [expr {round(1000.0 / $PS_PL_CLK_PERIOD_NS)}]
