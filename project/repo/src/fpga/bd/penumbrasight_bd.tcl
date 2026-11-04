# ============================================================================
# PenumbraSight 块设计 · src/fpga/bd/penumbrasight_bd.tcl
#
# 用途：重建 PL 侧块设计 —— Zynq MPSoC PS + **定点层四个 HLS IP** 的 AXI 连接、
#       地址映射、时钟与复位。跑完出 wrapper，是综合/实现/上板的入口。
#
# 用法（在**纯 ASCII 构建区**的仓库根执行）
# ----------------------------------------------------------------------------
#     vivado -mode batch -nojournal -nolog -source src/fpga/bd/penumbrasight_bd.tcl
#
#   ⚠ 为什么是 ASCII 构建区（D:\psight\penumbrasight）而不是中文路径的工作区：
#     本脚本会写 `build/bd_project`；放 ASCII 区可避免工具链里其他组件
#     （vitis-run / xsct / 命令行 IP 打包）再踩一次中文路径编码。
#     构建区约定见 docs/environment_ascii_build.md。
#
# 前置
# ----------------------------------------------------------------------------
#   1. 跑过 `make hls-fixed`，四个定点核各出了 IP（`<proj>/solution1/impl/ip`）。
#   2. `src/fpga/hls/ip_repo/` 是**生成物**（已 gitignore），不进版本库；
#      本脚本 §2 会按需把 IP 填充进去。
#   3. 接口版本须与 ps_fixed_top.hpp 的 PS_FIX_IF_VERSION 一致 —— §0 强校验。
#
# ----------------------------------------------------------------------------
# 本版相对上一版的差别（2026-09-28 重写，全部是实测结论，不是推测）
# ----------------------------------------------------------------------------
# 上一版**从未被 Vivado 跑过**（当时本机无 Vivado），因此带着三处实质错误：
#
#   a) IP 用的是双精度层 `ps_*_axi`。双精度层在 XCZU9EG 的 DSP48E2（27x18 乘法器）
#      上做不了浮点，**本来就上不了板**。现改为定点层四个顶层：
#        ps_projection_fix / ps_lr_forward_fix / ps_lr_adjoint_fix / ps_pdhg_fix
#
#   b) `S_AXI_GP2` **不是本 PS 的端口名**。实测（Vivado 2025.2 + ALINX 预设）
#      PS 实际暴露的是：
#          M_AXI_HPM0_FPD   (Master)  控制通路：PS 读写字寄存器
#          S_AXI_HP0_FPD    (Slave)   数据通路：核访问 DDR
#          S_AXI_HP1_FPD    (Slave)
#      旧脚本里 `CONFIG.PSU__USE__S_AXI_GP2 {1}` 不产生任何端口 —— 是句空设。
#
#   c) **从没连过 PS 的 AXI 时钟脚**：`maxihpm0_fpd_aclk`、`saxihp0_fpd_aclk`。
#      旧脚本只把 `pl_clk0` 连到各核与互连，PS 侧这两个时钟输入是悬空的。
#
# 另有一处旧脚本**漏做**：每个核最多有三组 m_axi bundle
#   gmem0_model（模型：U/S/V/codes，只读）/ gmem1_data（数据，读写）/
#   gmem2_scratch（迭代暂存）
# 旧脚本只接了 gmem0_model 一组，另两组悬空 —— 板上会表现为
# 「读得到模型、写不出结果」。本版改为按核**动态枚举** `m_axi_*` 接口，
# 逐个接入，不再硬编码 bundle 名（避免与 pragma 不同步）。
#
#
# ----------------------------------------------------------------------------
# 本版相对上一版（2026-09-30：把 RTL 五模块与搬运通路接进来）
# ----------------------------------------------------------------------------
# 上一版把「PS + 四个定点核」接完就停了，PL 里**没有任何 RTL 模块**，
# 老师那条「SD 卡喂数据 → 板端 DP 输出」的通路在块设计里并不存在
# （见 sim/results/preboard_port_status_20260930.md §1.3 与 §二 b-1~b-3）。
#
# 本版补的内容与依据（完整论证见 sim/results/bd_full_path_20260930.md）：
#
#   1) 先定显示通路归属：板载 mini-DP 在 **PS 侧**。
#      依据《AXU9EGB开发板用户手册》35/56 页正文（数据通道用 BANK505 的
#      PS MGT，AUX 走 PS 的 MIO）与底板原理图第 2 页网名
#      （GT0/GT1_DP_TX = 505_TX3/TX2、DP_AUX_*_MIO27/28/29/30）。
#      因此**本设计不需要任何 PL 视频时序/输出 IP**，也不新增视频引脚约束；
#      compositor 的 1080p RGB 交付物改为写进 DDR 帧缓冲，由 PS 的 DP 控制器扫描输出。
#
#   2) 五个 RTL 模块进块设计：raw_preproc / vertical_diff / fast_alert /
#      confidence_engine / compositor。彼此以 AXIS 直连，口径取自 RTL 源码端口
#      与 src/fpga/rtl/readme.md §2 的数据流，不自行发明。
#      ⚠ Vivado 的 module reference **不接受 SystemVerilog 顶层文件**（实测
#        [filemgmt 56-195]，报告 §4.1），故 src/fpga/rtl/pl_wrap/ 下每个核配一个
#        .v 结构壳（纯直通、不设参数）。壳只换实例化机制，不改任何行为。
#
#   3) 「等效搬运」用 **AXI DMA**（xilinx.com:ip:axi_dma），不是裸 axi_datamover：
#      实测裸 DataMover 只暴露 cmd/sts 的 AXIS（无 AXI-Lite），PS 无法直接下命令；
#      AXI DMA = DataMover + 寄存器组，S_AXI_LITE 可由 PS 直接写。
#
#   4) 位宽适配：AXI DMA 的流位宽只允许 8/16/32/64/...（实测），而差分流与显示流
#      都是 24 bit，故各加一个 axis_dwidth_converter（3 -> 4 字节）。
#      差分流还要一分二（fast_alert 与搬运器各取一份），用 axis_broadcaster。
#
#   5) 边界：RTL 的配置/阈值全部由 xlconstant 钉住（取值逐条标注出处：configs/*.yaml
#      或各 TB 的实测取值）。**RTL 的 AXI-Lite 配置/回读寄存器未接**（仓库里没有这层桥，
#      报告 §7 缺口 G1），这是本版的明确边界，不声称已完成。
#
# ----------------------------------------------------------------------------
# 本版相对上一版（2026-09-30 第二版：修 G2 / G1 / G4）
# ----------------------------------------------------------------------------
# 上一版把 RTL 接进块设计，但链上有三处「结构上了、语义没通」的地方。本版逐个处理：
#
#   A) G2 帧级 tlast：AXI DMA 的 S2MM 以 TLAST 结束一次传输，而
#      `vertical_diff.sv:286` 的 tlast 是**块行末**（每帧 95 次）。新增模块引用
#      `frm_diff`（source: src/fpga/rtl/axis_frame_sync.v）插在 bc_diff/M01 与
#      dwc_meas 之间，一帧只置一次 tlast。**只改搬运支路**：`fast_alert.sv:263/:542`
#      依赖块行 tlast，那一支刻意不动。
#      raw 侧同类错配一并处理：`frm_raw` 插在 MM2S 与 raw_preproc 之间，补
#      「帧首 tuser + 每条 raw 行末 tlast」。MM2S 的 M_AXIS_MM2S 是 TUSER_WIDTH 0，
#      它**不产生 tuser**，而 raw_preproc 在 ST_IDLE 等 `tvalid && tuser`
#      （`raw_preproc.sv:540`）—— 不补的话这条链一帧都进不去。
#
#   B) G1 配置/回读寄存器：新增模块引用 `ps_regs`（source: src/fpga/rtl/pl_ctrl_regs.v），
#      一个 AXI4-Lite 从口 + 4 KiB 寄存器页，挂到 sctl_ctrl 的第 11 个 MI，
#      地址 0xA00A0000。五个 RTL 模块的全部 `cfg_*` / 阈值改由它驱动；原先的
#      xlconstant 钉扎**逐条删除**（留着会与寄存器输出构成多驱动网）。寄存器表见
#      `board/ps_pl_regs.h`。复位默认值与上一版钉扎值相同 -> 默认行为不变。
#
#   C) G4 compositor 三个本地写口：同一块 ps_regs 里给出 wall/rec/osd 的
#      「地址寄存器 + 数据寄存器」，写数据寄存器的**下一拍**产生一拍本地写使能。
#      选这条路（而不是改 compositor 从 DDR 读图）的理由：共用同一条 AXI-Lite
#      基础设施、不动 compositor 的读通路、改动半径最小（见报告 §任务三）。
#
# 这三个新增单元都写成**纯 Verilog `.v`**：块设计的 module reference 不接受
# SystemVerilog 顶层（[filemgmt 56-195]，报告 §2.4），写成 .v 就直接可引用，
# 不需要再配 pl_wrap/ 结构壳（也就没有「壳与核端口不一致」这类错）。
#
# ----------------------------------------------------------------------------
# 本版相对上一版（2026-09-30 第三版：G5 剩余接线）
# ----------------------------------------------------------------------------
# 改动只有两处接线 + 一个源文件注册，**不新增 BD cell**：
#
#   D) 432 项掩膜表：删掉最后一条 xlconstant 钉扎
#      （原 `ps_tie 8 255 rtl_confidence_engine/mask_rdata`），改接 ps_regs 内部的
#      432x8 RAM（写口 = MASK_ADDR/MASK_DATA，读口 = CE 的 mask_raddr）。
#      RAM 上电内容 0xFF，与旧常量逐位相同。
#
#   E) 饱和比例的 PL 内来源：`sat_ratio_div.v` 注册进工程（pl_ctrl_regs 的子模块），
#      pl_ctrl_regs 用一个 MUX 在「PL 算的」与「PS 写的 CE_SAT」之间二选一
#      （CE_CFG bit3，默认 1）。这一条**没有新增连到 BD 的网**，只是 ps_regs
#      内部多了一个子模块。
#
#   margin / ρ 这两项**本轮没动**：它们分别落在 core_proj 的 s_axilite 寄存器与
#   core_pdhg 的 DDR 缓冲里，PL 内要自动取回都得新增 AXI 主口 —— 属块设计级改动，
#   本轮只在报告里给出路径与代价（sim/results/g5_g3_raw_fix_20261001.md）。
#
# ----------------------------------------------------------------------------
# 本版相对上一版（2026-10-02：中断通路 + 时钟单一来源 + 死代码清理）
# ----------------------------------------------------------------------------
#   F) 中断通路：新增 xlconcat(8) + 一根 xlconstant 0，把三个 AXI DMA 的
#      mm2s_introut / s2mm_introut 汇总到 zynq_ps/pl_ps_irq0。此前两个 DMA 的
#      introut **全部悬空**，PL→PS 无中断通路。宽度按 PG201（pl_ps_irq0[7:0]）
#      取 8，未用输入显式接 0。PS 侧驱动**仍按 ap_done 轮询**，中断是可选通路
#      （详见 §6.3b 的注释）。新增 BD cell：`irq_concat`、`irq_gnd`。
#
#   G) 时钟单一来源：周期改由 src/fpga/scripts/clock_plan.tcl 单一定义（本脚本
#      source 它），不再写 10.0 字面量；与 build_hls.tcl、synth_rtl_probe.tcl 同源。
#
#   H) 死代码：删除已无调用者的 `ps_const` / `ps_tie` 两个过程（见 §6.4 说明）。
#
# 交付边界
# ----------------------------------------------------------------------------
# 本脚本交付「能通过 validate_bd_design 的块设计 + wrapper」。
# **实现（place/route）、时序收敛、资源占用、bitstream 都要另跑**，
# 本脚本不声称这些已完成。综合由 src/fpga/scripts/build_pl.tcl 跑。
# ============================================================================

set BD_NAME        "penumbrasight"
set PART           "xczu9eg-ffvb1156-2-i"
set BOARD_PART     ""
# PL 时钟周期/频率不再在此写字面量：单一来源见 src/fpga/scripts/clock_plan.tcl
# （2026-10-02 起）。下面 REPO_ROOT 算出后 source 它，得到
# PS_PL_CLK_PERIOD_NS / PS_PL_CLK_MHZ，再落到本脚本的 CLK_PERIOD_NS / PL_CLK_MHZ。

# 必须与 src/fpga/hls/ps_fixed_top.hpp 的 PS_FIX_IF_VERSION 相同
set EXPECT_IF_VER  1

# 控制寄存器基址：每核 64 KiB。定点层的 s_axilite 标量比双精度层多
# （投影核最多 ~27 个），最高偏移仍远小于 64 KiB。
# 数据段：核作为 master 访问 PS DDR。**地址不在这里硬编码** —— 实测 PS 的 DDR
# 段是按从端口分组的（`/zynq_ps/SAXIGP2/HP0_DDR_LOW|HIGH`），脚本按名匹配后
# 用段自身的原生范围映射（见 §7）。规模的来由：最大的一块是码本 `u_codes`，
# depth = 17,729,280 个字 x 4 B ≈ 70.9 MiB（见 ps_fixed_top.cpp 的 pragma）。
set CTRL_BASE      0x00A0000000
set CTRL_RANGE     64K

set REPO_ROOT   [file normalize [pwd]]
# PL 时钟单一来源（周期由环境变量 PS_PL_CLK_PERIOD_NS 覆盖，见 clock_plan.tcl 文首）。
source [file join $REPO_ROOT src fpga scripts clock_plan.tcl]
set CLK_PERIOD_NS $PS_PL_CLK_PERIOD_NS
set PL_CLK_MHZ    $PS_PL_CLK_MHZ
set HLS_ROOT    [file normalize [file join $REPO_ROOT src fpga hls]]
set IP_REPO     [file normalize [file join $HLS_ROOT ip_repo]]
set FIX_HDR     [file normalize [file join $HLS_ROOT ps_fixed_top.hpp]]
set BD_PROJ_NAME   "bd_project"
set BD_PROJ        [file normalize [file join $REPO_ROOT build $BD_PROJ_NAME]]
set PRESET_TCL  [file normalize [file join $REPO_ROOT src fpga bd ps_preset_axu9eg_alinx.tcl]]

# 工程目录名 -> 顶层函数名（工程名由 build_hls.tcl 的 fixed 模式决定）
set CORE_MAP [list \
    proj_fix   ps_projection_fix    core_proj \
    lr_fwd_fix ps_lr_forward_fix    core_lr_fwd \
    lr_adj_fix ps_lr_adjoint_fix    core_lr_adj \
    pdhg_fix   ps_pdhg_fix          core_pdhg \
]

puts "==== PenumbraSight 块设计 ===="
puts "  REPO_ROOT  = $REPO_ROOT"
puts "  PART       = $PART"
puts "  PL 时钟    = ${PL_CLK_MHZ} MHz (${CLK_PERIOD_NS} ns)"
puts "  IP 仓      = $IP_REPO"
puts "  工程目录   = $BD_PROJ"

# ---------------------------------------------------------------- 0 前置检查
if {![file exists $FIX_HDR]} {
    puts "ERROR: 找不到 $FIX_HDR"
    puts "       本脚本要求从仓库根执行，且定点层顶层 ps_fixed_top.hpp 已落地。"
    exit 1
}
set fh [open $FIX_HDR r]
set hdr [read $fh]
close $fh
if {![regexp {#define\s+PS_FIX_IF_VERSION\s+(\d+)} $hdr -> hdr_ver]} {
    puts "ERROR: 在 ps_fixed_top.hpp 里找不到 PS_FIX_IF_VERSION"
    exit 1
}
if {[expr {int($hdr_ver)}] != $EXPECT_IF_VER} {
    puts "ERROR: 接口版本不匹配 —— ps_fixed_top.hpp 是 $hdr_ver，本脚本期望 $EXPECT_IF_VER。"
    puts "       改了 bundle 或增删了 s_axilite 标量就必须同时 +1 两处，"
    puts "       否则地址会静默错位（板上最难查的一类）。"
    exit 1
}
puts "  接口版本   = $hdr_ver  (与 ps_fixed_top.hpp 一致 ✓)"

foreach {proj top cell} $CORE_MAP {
    set ipdir [file join $REPO_ROOT $proj solution1 impl ip]
    if {![file exists [file join $ipdir component.xml]]} {
        puts "ERROR: 缺 $proj 的 IP：$ipdir\\component.xml 不存在。"
        puts "       先跑 `make hls-fixed`（在构建区）。"
        exit 1
    }
}

# ---------------------------------------------------------------- 1 工程
if {![file exists [file join $BD_PROJ ${BD_PROJ_NAME}.xpr]]} {
    file mkdir $BD_PROJ
    create_project -force $BD_PROJ_NAME $BD_PROJ -part $PART
} else {
    open_project [file join $BD_PROJ ${BD_PROJ_NAME}.xpr]
}
set_property part $PART [current_project]
if {$BOARD_PART ne "" && [llength [get_board_parts -quiet $BOARD_PART]] > 0} {
    set_property board_part $BOARD_PART [current_project]
    puts "  板级文件   = 找到（用其 PS 预设）"
} else {
    puts "  板级文件   = 无（**预期**：ALINX 不提供 board_files，走 set_part + 手工 PS 预设）"
}

# ---------------------------------------------------------------- 2 IP 仓
# ⚠ 顺序是实测结论：**必须先把 ip_repo_paths 注册进工程，`-add_ip` 才不会报**
#   `[IP_Flow 19-925] ... the specified repository is not an existing IP repository in the project`
file mkdir $IP_REPO
set_property ip_repo_paths $IP_REPO [current_project]
update_ip_catalog -rebuild -quiet

set need_add {}
foreach {proj top cell} $CORE_MAP {
    if {[llength [get_ipdefs -quiet -all xilinx.com:hls:${top}:1.0]] == 0} {
        lappend need_add $proj
    }
}
foreach proj $need_add {
    set ipdir [file join $REPO_ROOT $proj solution1 impl ip]
    puts "  添加 IP    = $proj"
    # ⚠ `-add_ip` 要的是 **component.xml（或 zip 文件）的路径**，不是 IP 目录 ——
    #   传目录会报 `[IP_Flow 19-1666] ... the specified IP file does not exist`。
    update_ip_catalog -add_ip [file join $ipdir component.xml] -repo_path $IP_REPO
}
if {[llength $need_add]} { update_ip_catalog -rebuild -quiet }

set missing {}
foreach {proj top cell} $CORE_MAP {
    if {[llength [get_ipdefs -quiet -all xilinx.com:hls:${top}:1.0]] == 0} { lappend missing $top }
}
if {[llength $missing]} {
    puts "ERROR: 以下 IP 仍不在 catalog 里：$missing"
    exit 1
}
puts "  四个定点 IP = 全部在 catalog 里 ✓"

# ---------------------------------------------------------------- 2.5 RTL 源（供 BD 模块引用）
# ⚠ Vivado 的 module reference 顶层**不支持 SystemVerilog**（实测 [filemgmt 56-195]，
#   报告 §4.1），所以成对加入：SV 核体 + pl_wrap/ 下的 .v 结构壳；BD 引用的是壳。
#   壳只做端口直通、不设参数，默认参数即 B 档冻结值（WALL_N=96 / BLOCK=10 / CH=3 ...）。
set RTL_SRC {}
foreach f {raw_preproc.sv vertical_diff.sv fast_alert.sv confidence_engine.sv compositor.sv} {
    lappend RTL_SRC [file normalize [file join $REPO_ROOT src fpga rtl $f]]
}
foreach f {bd_raw_preproc.v bd_vertical_diff.v bd_fast_alert.v bd_confidence_engine.v bd_compositor.v} {
    lappend RTL_SRC [file normalize [file join $REPO_ROOT src fpga rtl pl_wrap $f]]
}
# 2026-09-30 新增：G2 帧边界整形器 + G1/G4/G5 寄存器叶。
# 这两个刻意写成**纯 Verilog .v**（不是 .sv）：块设计的 module reference 不接受
# SystemVerilog 顶层（[filemgmt 56-195]，见报告 §2.4），写成 .v 就直接可引用，
# 省掉 pl_wrap/ 那一层结构壳，也就没有「壳与核端口不一致」这类错。
#   2026-09-30 收口 G5 再追加 sat_ratio_div.v：pl_ctrl_regs 内部实例化它
#   （饱和比例除法器），**它必须一起进工程**，否则 module reference 展开时
#   找不到子模块。它不进 BD 的 cell 列表，只是 pl_ctrl_regs 的子模块。
foreach f {axis_frame_sync.v pl_ctrl_regs.v sat_ratio_div.v} {
    lappend RTL_SRC [file normalize [file join $REPO_ROOT src fpga rtl $f]]
}
set rtl_added 0
foreach f $RTL_SRC {
    if {![file exists $f]} {
        puts "ERROR: 缺 RTL 源 $f —— 本版块设计需要它（pl_wrap/ 下的 .v 壳由本仓库提供）。"
        exit 1
    }
    if {[llength [get_files -quiet [file tail $f]]] == 0} {
        add_files -norecurse $f
        incr rtl_added
    }
}
if {$rtl_added} { update_compile_order -fileset sources_1 }
puts "  RTL 源     = 5 个 SV 核 + 5 个 .v 结构壳 + 3 个 .v 新核（本次新增 $rtl_added 个文件）"

# ---------------------------------------------------------------- 3 建 BD 与 PS
# ⚠ open_project 之后 BD **不会自动打开**，此时 get_bd_designs 是空的，
#   直接 delete_bd_objs 会报 `[BD 5-104] A block design must be open`（2026-09-30 实测）。
#   所以：先打开再删；删不掉就把文件移出工程。
set old_bd [get_files -quiet ${BD_NAME}.bd]
if {[llength $old_bd]} {
    puts "  已存在同名 BD，删除重建"
    catch { open_bd_design [lindex $old_bd 0] }
    if {[llength [get_bd_designs -quiet $BD_NAME]]} {
        catch { delete_bd_objs [get_bd_designs -quiet $BD_NAME] }
    }
    set still [get_files -quiet ${BD_NAME}.bd]
    if {[llength $still]} { catch { remove_files $still } }
}
create_bd_design $BD_NAME

create_bd_cell -type ip -vlnv xilinx.com:ip:zynq_ultra_ps_e zynq_ps
# ⚠ 先设 $ps_cell 再 source 预设：预设文件末尾的 set_property 引用 $ps_cell
#   （它由调用方提供单元格变量名）。顺序反了会报 "can't read ps_cell"。
set ps_cell [get_bd_cells zynq_ps]
source $PRESET_TCL

# 本设计对厂级预设的覆盖（左=项，中=ALINX 值，右=本设计值）
#   USE__M_AXI_GP0        0 -> 1    控制通路：PS 作 master 访问 PL 寄存器
#   USE__M_AXI_GP1/GP2    0 -> 0    显式写出，便于与预设比对
#   MAXIGP0__DATA_WIDTH 128 -> 32   控制通路 32 bit 足够
#   SAXIGP2__DATA_WIDTH 128 -> 128  与 ALINX 同（数据通路走 HP0）
#   FPGA_PL0_ENABLE       1 -> 1
#   PL0_REF_CTRL__FREQMHZ 200 -> 100（见下）
#   ⚠ **不再设 `USE__S_AXI_GP2`** —— 实测它在本 PS 上不产生任何端口（见文首 b）。
#     数据通路走预设已启用的 S_AXI_HP0_FPD。
#   时钟取 100 MHz 是**实测选定的时序余量**（X19，2026-09-28）：
#     200 MHz 下 ps_projection_fix Slack -1.90、ps_pdhg_fix -1.39（违例）
#     150 MHz 下 ps_pdhg_fix 仍 -0.51
#     100 MHz 下四个定点核 Slack 全 0.00
#   周期由 src/fpga/scripts/clock_plan.tcl **单一定义**（2026-10-02 起）：
#   本脚本与 build_hls.tcl 都 source 它，xdc 侧只留说明 —— 改档只改那一个文件。
#   （此前是「三处字面量 + 注释约定一起改」，漏改一处不报错，已改掉。）
set_property -dict [list \
    CONFIG.PSU__USE__M_AXI_GP0        {1} \
    CONFIG.PSU__USE__M_AXI_GP1        {0} \
    CONFIG.PSU__USE__M_AXI_GP2        {0} \
    CONFIG.PSU__MAXIGP0__DATA_WIDTH   {32} \
    CONFIG.PSU__FPGA_PL0_ENABLE       {1} \
    CONFIG.PSU__CRL_APB__PL0_REF_CTRL__FREQMHZ $PL_CLK_MHZ \
] $ps_cell

# --- 实测回显：把 PS 真实的 AXI 端口与时钟脚打出来 -------------------------
# 这一段的用途是**让脚本自己证明端口名是对的**，而不是靠注释声称。
set ps_master ""
set ps_slaves {}
foreach p [get_bd_intf_pins -quiet zynq_ps/*] {
    set n [get_property NAME $p]
    if {![string match -nocase *axi* $n]} { continue }
    switch -- [get_property MODE $p] {
        Master { if {[string match *HPM0* $n]} { set ps_master $n } }
        Slave  { lappend ps_slaves $n }
    }
}
puts "  PS master  = $ps_master"
puts "  PS slaves  = $ps_slaves"
if {$ps_master eq ""} {
    puts "ERROR: 没找到 PS 的 M_AXI_HPM0_FPD。检查 PSU__USE__M_AXI_GP0。"
    exit 1
}
if {[llength $ps_slaves] == 0} {
    puts "ERROR: PS 没有任何 Slave AXI 端口。检查预设里的 S_AXI_HP*_FPD。"
    exit 1
}
set ps_slave [lindex $ps_slaves 0]
puts "  数据通路用  = $ps_slave（其余 slave 端口只连时钟，保持悬空但不报错）"

# ---------------------------------------------------------------- 4 时钟与复位
create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset rst_pl
connect_bd_net [get_bd_pins zynq_ps/pl_clk0]         [get_bd_pins rst_pl/slowest_sync_clk]
connect_bd_net [get_bd_pins zynq_ps/pl_resetn0]      [get_bd_pins rst_pl/ext_reset_in]
# ⚠ 旧脚本漏掉的一整类：PS 侧每个 AXI 端口都有自己的时钟输入脚，
#   名字是**小写缩写**（`maxihpm0_fpd_aclk` / `saxihp0_fpd_aclk` / `saxihp1_fpd_aclk`），
#   与端口名 `M_AXI_HPM0_FPD` **不是**简单拼接 —— 拼名会得到空结果，
#   再传给 connect_bd_net 就报 `[BD 41-701] requires at least two pins`。
#   所以这里按后缀枚举，不拼名。
foreach c [get_bd_pins -quiet zynq_ps/*aclk*] {
    catch { connect_bd_net [get_bd_pins zynq_ps/pl_clk0] $c }
}

# ---------------------------------------------------------------- 5 四个定点核
set core_names {}
foreach {proj top cell} $CORE_MAP {
    create_bd_cell -type ip -vlnv xilinx.com:hls:${top}:1.0 $cell
    lappend core_names $cell
    connect_bd_net [get_bd_pins zynq_ps/pl_clk0]               [get_bd_pins $cell/ap_clk]
    connect_bd_net [get_bd_pins rst_pl/peripheral_aresetn]     [get_bd_pins $cell/ap_rst_n]
    # 回显每个核的 m_axi 组数：这是本版相对旧版的关键差异，必须看得见
    # （直接打印引脚对象本身即可得到名字；对存进变量的结果再调 get_property
    #   会报 [Common 17-161]，因为那时它已经是个路径字符串而不是活对象）
    set mx [get_bd_intf_pins -quiet $cell/m_axi_*]
    puts [format "  %-12s %-20s m_axi %d 组：" $cell $top [llength $mx]]
    foreach p $mx { puts "       - $p" }
}

# ---------------------------------------------------------------- 6 互连
set n_mx 0
foreach cell $core_names { incr n_mx [llength [get_bd_intf_pins -quiet $cell/m_axi_*]] }

# 本版新增：搬运器（AXI DMA）要占的控制从端口数与数据主端口数。计数在创建时再核一遍。
set N_DMA_LITE 2   ;# dma_pipe/S_AXI_LITE + dma_disp/S_AXI_LITE（PS 写源/目的/长度、读状态）
set N_DMA_MAST 3   ;# dma_pipe/M_AXI_MM2S + dma_pipe/M_AXI_S2MM + dma_disp/M_AXI_S2MM
# 2026-09-30 新增：RTL 侧的配置/回读寄存器叶（G1）要占的控制从端口数。
set N_REG_LITE 1   ;# ps_regs/s_axi（PL 控制页 0xA00A0000 + 0x10000）

# 控制通路：PS M_AXI_GP0 -> sctl_ctrl -> 各核的**所有** AXI-Lite 从接口
#
# ⚠ 每个定点核有**两个** AXI-Lite 从接口，不是一个：
#     s_axi_control    标量：P/K/M/nx/ny/nz/margin_thresh_q16，以及回读的 best_id 等
#     s_axi_control_r  **m_axi 缓冲区的基址寄存器**：meas / u_codes / gain_q16 / score_out / ...
#   2026-09-28 首次实现时只接了前者，而 `validate_bd_design` **照样通过、连一条
#   「未连接」的告警都没报**（已核日志）。后果：核拿不到缓冲区基址，只会按地址 0
#   读写 —— 板上必错，且现场极难查。
#   依据：IP 的 component.xml 有两个 memoryMap；`meas`/`u_codes`/`gain_q16`/`score_out`
#   都落在 `s_axi_control_r` 的 addressBlock 内（`s_axi_control` 的块止于 2661 行，
#   `meas` 在 2683 行，`P`/`K`/`M` 在 1575~1746 行）。见
#   sim/results/bd_wiring_and_address_map.md §3.1。
set CTRL_IFACES {s_axi_control s_axi_control_r}
set n_ctrl [expr {[llength $CTRL_IFACES] * [llength $core_names]}]
set n_ctrl_tot [expr {$n_ctrl + $N_DMA_LITE + $N_REG_LITE}]

create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect sctl_ctrl
set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI $n_ctrl_tot] \
    [get_bd_cells sctl_ctrl]
connect_bd_net [get_bd_pins zynq_ps/pl_clk0]           [get_bd_pins sctl_ctrl/aclk]
connect_bd_net [get_bd_pins rst_pl/peripheral_aresetn] [get_bd_pins sctl_ctrl/aresetn]
connect_bd_intf_net [get_bd_intf_pins zynq_ps/${ps_master}] [get_bd_intf_pins sctl_ctrl/S00_AXI]

# ---------------------------------------------------------------- 6.2 RTL 五模块（module reference）
# 引用的是 pl_wrap/ 下的 .v 结构壳（理由见 §2.5）；壳端口与 SV 核一一对应，
# 默认参数即 B 档冻结值，故这里不设任何参数。
set RTL_CELLS [list \
    bd_raw_preproc       rtl_raw_preproc \
    bd_vertical_diff     rtl_vertical_diff \
    bd_fast_alert        rtl_fast_alert \
    bd_confidence_engine rtl_confidence_engine \
    bd_compositor        rtl_compositor \
]
foreach {mod cell} $RTL_CELLS {
    if {[catch {create_bd_cell -type module -reference $mod $cell} err]} {
        puts "ERROR: 无法把 $mod 作为模块引用放进 BD：$err"
        puts "       检查 src/fpga/rtl/pl_wrap/$mod.v 是否已进工程源（见 §2.5）。"
        exit 1
    }
    # 单时钟域：全部跑 pl_clk0；复位沿用同一路异步复位同步释放
    connect_bd_net [get_bd_pins zynq_ps/pl_clk0]           [get_bd_pins $cell/clk]
    connect_bd_net [get_bd_pins rst_pl/peripheral_aresetn] [get_bd_pins $cell/rst_n]
    # 回显：接口名不是我们起的，是工具按端口名推断出来的（axis_rtl），写下来便于核对
    set il {}
    foreach ip [get_bd_intf_pins -quiet $cell/*] { lappend il [get_property NAME $ip] }
    puts [format "  %-22s 推断接口 = %s" $cell $il]
}

# ---------------------------------------------------------------- 6.2b G2 帧边界整形 + G1/G4 寄存器叶
# 这两类单元都是**纯 Verilog .v**（见 §2.5 的说明），故直接引用，不需要 pl_wrap 壳。
#
# (1) G2：差分流末端补「帧末才置 tlast」的整形，并给 raw 输入侧补行级 tlast / 帧首 tuser。
#     为什么只挂在差分**搬运支路**上、而不是挂在 vertical_diff 正下游：
#       `fast_alert.sv:263` 与 `:542` 用 `s_diff_tlast` 判定「块行末 / 一帧结束」，
#       它有**块行 tlast 的硬依赖**。把 tlast 改成帧末会直接改掉 fast_alert 的判决语义。
#       所以整形器只插在 M01 支路（-> dwc_meas -> dma_pipe/S_AXIS_S2MM），
#       fast_alert 那一支保持块行 tlast 不变 —— 这是改动半径最小的做法。
#     片参数：差分帧长 = (WALL_N-1)*WALL_N*CH = 95*96*3 = 27360（非旁路口径）。
# (2) G1/G4/G5：一个 AXI-Lite 从口 + 4 KiB 寄存器页。地址见 §7（排在两个搬运器之后）。
create_bd_cell -type module -reference axis_frame_sync frm_diff
# 差分支路：只用帧末 tlast（ROW_BEATS=0），默认参数即 DW=24 / FRAME_BEATS=27360
set il {}
foreach ip [get_bd_intf_pins -quiet frm_diff/*] { lappend il [get_property NAME $ip] }
puts [format "  %-22s 推断接口 = %s" frm_diff $il]

create_bd_cell -type module -reference axis_frame_sync frm_raw
# raw 支路：每条 raw 行末置 tlast、每帧第一拍置 tuser
#   ROW_BEATS   = RAW_SIDE = 2*BLOCK*WALL_N = 1920
#   FRAME_BEATS = RAW_SIDE*RAW_SIDE        = 3,686,400
# 注意：MM2S 的 M_AXIS_MM2S 是 TUSER_WIDTH 0（见
#   build/bd_project/.../penumbrasight_dma_pipe_0_stub.v），它**不产生 tuser**；
# 而 raw_preproc.sv:540 在 ST_IDLE 里等 `tvalid && tuser` —— 不整形的话这条链
# 连一帧都进不去。这正是任务一要求一并检查的 raw 侧同类错配。
set_property -dict [list CONFIG.DW {16} CONFIG.ROW_BEATS {1920} CONFIG.FRAME_BEATS {3686400}] \
    [get_bd_cells frm_raw]
set il {}
foreach ip [get_bd_intf_pins -quiet frm_raw/*] { lappend il [get_property NAME $ip] }
puts [format "  %-22s 推断接口 = %s" frm_raw $il]

create_bd_cell -type module -reference axis_frame_sync frm_disp
# 显示支路（2026-10-01，G3 + 显示侧 tlast）：compositor 的 vid_* 已是 AXIS 形状
#   （tvalid=de / tlast=**行末** / tuser=帧首），但 S2MM 以 TLAST 结束一次传输 ——
#   行末 tlast 会让一帧被切成 1080 次传输（与 G2 差分流同类）。这里把 tlast 换成
#   **帧末一次**：ROW_BEATS=0、FRAME_BEATS = H_ACTIVE*V_ACTIVE = 1920*1080 = 2,073,600。
#   DW = 24（compositor 的 PIX_W）。
set_property -dict [list CONFIG.DW {24} CONFIG.ROW_BEATS {0} CONFIG.FRAME_BEATS {2073600}] \
    [get_bd_cells frm_disp]
set il {}
foreach ip [get_bd_intf_pins -quiet frm_disp/*] { lappend il [get_property NAME $ip] }
puts [format "  %-22s 推断接口 = %s" frm_disp $il]

create_bd_cell -type module -reference pl_ctrl_regs ps_regs
set il {}
foreach ip [get_bd_intf_pins -quiet ps_regs/*] { lappend il [get_property NAME $ip] }
puts [format "  %-22s 推断接口 = %s" ps_regs $il]

foreach cell {frm_diff frm_raw frm_disp ps_regs} {
    connect_bd_net [get_bd_pins zynq_ps/pl_clk0]           [get_bd_pins $cell/clk]
    connect_bd_net [get_bd_pins rst_pl/peripheral_aresetn] [get_bd_pins $cell/rst_n]
}

# ---------------------------------------------------------------- 6.3 搬运与位宽适配
# 「等效搬运」：AXI DMA（DataMover + 寄存器组）。裸 axi_datamover 只暴露 cmd/sts 的
# AXIS、没有 AXI-Lite，PS 下不了命令，故改用它的寄存器化封装（报告 §4.2）。
# dma_pipe：MM2S 把 DDR 里的 RAW 帧搬成 AXIS 给 raw_preproc；
#           S2MM 把 vertical_diff 的差分流搬回 DDR（mes 段，供四个定点核读取）。
create_bd_cell -type ip -vlnv xilinx.com:ip:axi_dma dma_pipe
set_property -dict [list \
    CONFIG.c_include_sg              {0} \
    CONFIG.c_sg_length_width         {26} \
    CONFIG.c_include_mm2s            {1} \
    CONFIG.c_include_s2mm            {1} \
    CONFIG.c_m_axi_mm2s_data_width   {32} \
    CONFIG.c_m_axi_s2mm_data_width   {32} \
    CONFIG.c_m_axis_mm2s_tdata_width {16} \
    CONFIG.c_s_axis_s2mm_tdata_width {32} \
] [get_bd_cells dma_pipe]

# dma_disp：只留 S2MM —— 把 compositor 的 1080p RGB 写进 DDR 帧缓冲，
#           由 PS 的 DP 控制器扫描输出（DP 在 PS 侧，见文首第 1 条）。
create_bd_cell -type ip -vlnv xilinx.com:ip:axi_dma dma_disp
set_property -dict [list \
    CONFIG.c_include_sg              {0} \
    CONFIG.c_sg_length_width         {26} \
    CONFIG.c_include_mm2s            {0} \
    CONFIG.c_include_s2mm            {1} \
    CONFIG.c_m_axi_s2mm_data_width   {32} \
    CONFIG.c_s_axis_s2mm_tdata_width {32} \
] [get_bd_cells dma_disp]

foreach c {dma_pipe dma_disp} {
    connect_bd_net [get_bd_pins zynq_ps/pl_clk0]           [get_bd_pins $c/s_axi_lite_aclk]
    connect_bd_net [get_bd_pins rst_pl/peripheral_aresetn] [get_bd_pins $c/axi_resetn]
}
connect_bd_net [get_bd_pins zynq_ps/pl_clk0] [get_bd_pins dma_pipe/m_axi_mm2s_aclk]
connect_bd_net [get_bd_pins zynq_ps/pl_clk0] [get_bd_pins dma_pipe/m_axi_s2mm_aclk]
connect_bd_net [get_bd_pins zynq_ps/pl_clk0] [get_bd_pins dma_disp/m_axi_s2mm_aclk]

# ---------------------------------------------------------------- 6.3b 中断通路（PL -> PS）
# 2026-10-02 新增。此前两个 AXI DMA 的 mm2s_introut / s2mm_introut **全部悬空**，
# PL→PS 完全没有中断通路，PS 只能轮询 CTRL.ap_done（board/fix_core.c 的
# ps_fix_start_and_wait）。本段把三个 DMA 中断经 xlconcat 汇总到
# zynq_ps/pl_ps_irq0。
#
# 为什么只有 3 个中断：
#   dma_pipe/mm2s_introut   —— RAW 帧搬运完成（MM2S）
#   dma_pipe/s2mm_introut   —— meas 差分流回写完成（S2MM）
#   dma_disp/s2mm_introut   —— 1080p 帧缓冲写回完成（S2MM）
# 四个 HLS 核**没有**中断输出（完成标志是 s_axilite 的 ap_done 位），故核侧仍靠
# 轮询；这里接的是「PL 侧能提供的中断」的完整集合。
#
# 宽度依据（不是猜的）：Zynq UltraScale+ PS 的 PL→PS 共享中断是
# pl_ps_irq0[7:0]（PG201 Table 2-1：Interrupt ID 121..128 = pl_ps_irq0[0..7]），
# 即 **8 位**。故 xlconcat 的 NUM_PORTS 取 8，未用的 In3..In7 必须**显式接 0**
# （用一根 1 位 xlconstant 同时驱动 5 个输入脚）。若只建 3 路 concat 接到 8 位
# 端口，Vivado 会报 [BD 41-2383] width mismatch 且高 5 位悬空 —— 那是 CRITICAL
# WARNING 级别的缺陷，不能用。（做法与 Xilinx「GMSL + Vitis AI」参考工程一致：
# 8 输入 concat + 未用输入接地。）
#
# PS 侧：用例 IRQ0 需 PSU__USE__IRQ0=1 —— ALINX 预设已置 1
# （ps_preset_axu9eg_alinx.tcl:1512），本脚本不重复设置，只在下面回显校验。
#
# 对 PS 驱动的影响（重要）：**这是可选通路，默认不改变既有轮询行为**。
#   · board_main / fix_core 仍按 ap_done 轮询，不读中断也能工作；
#   · 要走中断，需设备树把这条 IRQ 连到 GIC/UIO 并另写唤醒循环 —— 独立改动，
#     见 sim/results/uio_entry_20261001.md §2.4（该后端刻意不阻塞 read）。
#   即：接上中断不改变现有行为，只是把「以后可用」从设想补成事实。
create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat irq_concat
set_property -dict [list CONFIG.NUM_PORTS {8}] [get_bd_cells irq_concat]
create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant irq_gnd
set_property -dict [list CONFIG.CONST_WIDTH {1} CONFIG.CONST_VAL {0}] [get_bd_cells irq_gnd]
connect_bd_net [get_bd_pins dma_pipe/mm2s_introut] [get_bd_pins irq_concat/In0]
connect_bd_net [get_bd_pins dma_pipe/s2mm_introut] [get_bd_pins irq_concat/In1]
connect_bd_net [get_bd_pins dma_disp/s2mm_introut] [get_bd_pins irq_concat/In2]
connect_bd_net [get_bd_pins irq_gnd/dout] \
    [get_bd_pins irq_concat/In3] [get_bd_pins irq_concat/In4] [get_bd_pins irq_concat/In5] \
    [get_bd_pins irq_concat/In6] [get_bd_pins irq_concat/In7]
connect_bd_net [get_bd_pins irq_concat/dout] [get_bd_pins zynq_ps/pl_ps_irq0]
# 回显：确认 pl_ps_irq0 真的存在（USE__IRQ0=1）。不存在时明确失败，
# 免得中断线静默没接上、又回到「只能轮询」而无人察觉。
if {[llength [get_bd_pins -quiet zynq_ps/pl_ps_irq0]] == 0} {
    puts "ERROR: 没找到 zynq_ps/pl_ps_irq0 —— 检查预设的 PSU__USE__IRQ0（应为 1）。"
    exit 1
}
puts "  中断通路   = 3 个 DMA 中断 -> xlconcat(8) -> zynq_ps/pl_ps_irq0（未用 In3..In7 接 0）"

# 位宽适配与一分二：
#   bc_diff  差分流一分二（fast_alert 算质心 + 搬运器回写 DDR）
#   dwc_*    24 bit -> 32 bit（AXI DMA 的流位宽只允许 8/16/32/64/...）
create_bd_cell -type ip -vlnv xilinx.com:ip:axis_broadcaster bc_diff
set_property -dict [list CONFIG.NUM_MI {2} CONFIG.S_TDATA_NUM_BYTES {3} CONFIG.M_TDATA_NUM_BYTES {3}] \
    [get_bd_cells bc_diff]
foreach cell {dwc_meas dwc_disp} {
    create_bd_cell -type ip -vlnv xilinx.com:ip:axis_dwidth_converter $cell
    # ⚠ 只设实测存在的参数：HAS_TID / HAS_TDEST / HAS_TUSER 在 2025.2 的
    #   axis_dwidth_converter 上**不存在**，设了会得到
    #   `CRITICAL WARNING: [BD 41-1276] Cannot set the parameter ... Parameter does not exist`
    #   （2026-09-30 实测）。所以这里只留 HAS_TKEEP / HAS_TSTRB。
    set_property -dict [list \
        CONFIG.S_TDATA_NUM_BYTES {3} CONFIG.M_TDATA_NUM_BYTES {4} \
        CONFIG.HAS_TKEEP {1} CONFIG.HAS_TSTRB {0} \
    ] [get_bd_cells $cell]
}
foreach c {bc_diff dwc_meas dwc_disp} {
    connect_bd_net [get_bd_pins zynq_ps/pl_clk0]           [get_bd_pins $c/aclk]
    connect_bd_net [get_bd_pins rst_pl/peripheral_aresetn] [get_bd_pins $c/aresetn]
}

# ---------------------------------------------------------------- 6.4 RTL 配置/阈值
# 2026-10-02：原先这里的 `ps_const` / `ps_tie` 两个过程（用 xlconstant 把
# cfg_*/阈值钉进硬件）自 2026-09-30 换成 ps_regs 驱动后**已无任何调用者**，
# 本次删除（死代码）。历史取值出处由下面两段说明保留。
#
# --- 2026-09-30：原「用 xlconstant 钉住 cfg_*/阈值」已换成 ps_regs 驱动 ---
# 依据：报告 §7 G1 —— RTL 文件头把 cfg_*/阈值写成「AXI-Lite 影子配置（由 fpga_control 写）」
# （如 raw_preproc.sv:190-194），但仓库里没有这层桥。现在由 ps_regs 的寄存器提供，
# **不再有 xlconstant 钉扎**（同一根脚两处驱动会成为多驱动网，必须删干净）。
# 复位默认值已在 pl_ctrl_regs.v 里写成与上一版钉扎值逐位相同，所以「PS 不写任何寄存器」
# 时链上行为不变。
#
# --- 2026-09-30（G5 收口）：最后一条 xlconstant 钉扎也删掉了 ---
#   原先这里是 `ps_tie 8 255 rtl_confidence_engine/mask_rdata`（432 项掩膜表没有来源，
#   只能钉成全 1 —— port_completeness §3.2 的 mask_frac=1.000）。本轮把掩膜表落成
#   ps_regs 内部的一块 432x8 RAM（见 pl_ctrl_regs.v [R6]）：
#     * 写口 = PS 写 MASK_ADDR(0x0F8) / MASK_DATA(0x0FC)，**同一条 AXI-Lite**；
#     * 读口 = CE 的 mask_raddr 直连（同步读 1 拍，与 CE 的 ST_MRD->ST_MWR 两级对齐）；
#     * 上电内容 0xFF，与旧常量逐位相同 —— PS 不写这张表时行为不变。
#   ⇒ 本脚本的**配置/阈值**已无任何 xlconstant 钉扎（ps_const/ps_tie 两个过程
#     因无调用者已于 2026-10-02 删除）。块设计里剩下的唯一一个 xlconstant 是
#     6.3b 的 `irq_gnd`（中断汇总未用输入接 0），与配置通路无关。
connect_bd_net [get_bd_pins rtl_confidence_engine/mask_raddr] [get_bd_pins ps_regs/ce_mask_raddr]
connect_bd_net [get_bd_pins ps_regs/ce_mask_rdata]           [get_bd_pins rtl_confidence_engine/mask_rdata]

# --- ps_regs：配置输出 -> 各 RTL 单元 ---
# raw_preproc
connect_bd_net [get_bd_pins ps_regs/cfg_dark_level]   [get_bd_pins rtl_raw_preproc/cfg_dark_level]
connect_bd_net [get_bd_pins ps_regs/cfg_frames]       [get_bd_pins rtl_raw_preproc/cfg_frames]
connect_bd_net [get_bd_pins ps_regs/cfg_crop_side]    [get_bd_pins rtl_raw_preproc/cfg_crop_side]
connect_bd_net [get_bd_pins ps_regs/cfg_bayer_phase]  [get_bd_pins rtl_raw_preproc/cfg_bayer_phase]
# vertical_diff
connect_bd_net [get_bd_pins ps_regs/cfg_bypass]       [get_bd_pins rtl_vertical_diff/cfg_bypass]
connect_bd_net [get_bd_pins ps_regs/cfg_wall_n]       [get_bd_pins rtl_vertical_diff/cfg_wall_n]
# fast_alert：阈值 + c_k 系数流 + 背景 μ/v 表
connect_bd_net [get_bd_pins ps_regs/cfg_e_thresh]     [get_bd_pins rtl_fast_alert/cfg_e_thresh]
connect_bd_net [get_bd_pins ps_regs/cfg_k]            [get_bd_pins rtl_fast_alert/cfg_k]
connect_bd_net [get_bd_pins ps_regs/cfg_eps]          [get_bd_pins rtl_fast_alert/cfg_eps]
connect_bd_net [get_bd_pins ps_regs/cfg_persist_en]   [get_bd_pins rtl_fast_alert/cfg_persist_en]
connect_bd_net [get_bd_pins ps_regs/cfg_dir_deadband] [get_bd_pins rtl_fast_alert/cfg_dir_deadband]
connect_bd_net [get_bd_pins ps_regs/cfg_min_energy]   [get_bd_pins rtl_fast_alert/cfg_min_energy]
connect_bd_net [get_bd_pins ps_regs/coef_tdata]       [get_bd_pins rtl_fast_alert/s_coef_tdata]
connect_bd_net [get_bd_pins ps_regs/coef_tindex]      [get_bd_pins rtl_fast_alert/s_coef_tindex]
connect_bd_net [get_bd_pins ps_regs/coef_tvalid]      [get_bd_pins rtl_fast_alert/s_coef_tvalid]
connect_bd_net [get_bd_pins ps_regs/coef_tlast]       [get_bd_pins rtl_fast_alert/s_coef_tlast]
connect_bd_net [get_bd_pins ps_regs/bg_mu]            [get_bd_pins rtl_fast_alert/bg_mu]
connect_bd_net [get_bd_pins ps_regs/bg_var]           [get_bd_pins rtl_fast_alert/bg_var]
connect_bd_net [get_bd_pins rtl_fast_alert/bg_raddr]  [get_bd_pins ps_regs/bg_raddr]

# --- confidence_engine：四项实时量 + 阈值全部由 ps_regs 提供 ---
# 2026-09-30（G5 收口）后的四项来源：
#   饱和比例  —— **PL 内自动**：ps_regs 内部 sat_ratio_div 由 raw 的两个计数算出
#                （CE_CFG bit3 默认 1 选它；bit3=0 退回 PS 写的 CE_SAT）。
#   margin    —— 仍在投影核 core_proj 里（s_axilite 输出寄存器），PL 内**无**读数通路：
#                要自动取回需给 PL 加一个 AXI4-Lite 主口读核的控制寄存器。
#   ρ         —— 在 core_pdhg 的 **gmem1_data DDR 缓冲**（out_rho_q16[3]），不在寄存器里：
#                自动取回要么 PS 读 DDR 再写 CE_RHO，要么 PL 加一个 DDR 读主口。
#   SNR       —— 上游没有产这个量的模块，PS 写。
# 因此本脚本在此只把「可写」的寄存器接上；哪些有自动来源、哪些必须外部条件，
# 见 sim/results/g5_g3_raw_fix_20261001.md（本文不声称四项全通）。
connect_bd_net [get_bd_pins ps_regs/ce_snr_q016]       [get_bd_pins rtl_confidence_engine/snr_q016]
connect_bd_net [get_bd_pins ps_regs/ce_margin_q016]    [get_bd_pins rtl_confidence_engine/margin_q016]
connect_bd_net [get_bd_pins ps_regs/ce_rho_q016]       [get_bd_pins rtl_confidence_engine/rho_q016]
connect_bd_net [get_bd_pins ps_regs/ce_sat_ratio_q016] [get_bd_pins rtl_confidence_engine/sat_ratio_q016]
connect_bd_net [get_bd_pins ps_regs/ce_geom_hash_ok]   [get_bd_pins rtl_confidence_engine/geom_hash_ok]
connect_bd_net [get_bd_pins ps_regs/ce_snr_min_q016]   [get_bd_pins rtl_confidence_engine/snr_min_q016]
connect_bd_net [get_bd_pins ps_regs/ce_margin_min_q016] [get_bd_pins rtl_confidence_engine/margin_min_q016]
connect_bd_net [get_bd_pins ps_regs/ce_rho_max_q016]   [get_bd_pins rtl_confidence_engine/rho_max_q016]
connect_bd_net [get_bd_pins ps_regs/ce_sat_max_q016]   [get_bd_pins rtl_confidence_engine/sat_max_q016]
connect_bd_net [get_bd_pins ps_regs/ce_low_conf_q016]  [get_bd_pins rtl_confidence_engine/low_conf_q016]
connect_bd_net [get_bd_pins ps_regs/ce_w_sel]          [get_bd_pins rtl_confidence_engine/w_sel]

# --- compositor：候选/可信度参数 + **三个本地写口**（G4）---
connect_bd_net [get_bd_pins ps_regs/cmp_cand_best_x] [get_bd_pins rtl_compositor/cand_best_x]
connect_bd_net [get_bd_pins ps_regs/cmp_cand_best_y] [get_bd_pins rtl_compositor/cand_best_y]
connect_bd_net [get_bd_pins ps_regs/cmp_cand_best_z] [get_bd_pins rtl_compositor/cand_best_z]
connect_bd_net [get_bd_pins ps_regs/cmp_selected_id] [get_bd_pins rtl_compositor/selected_id]
connect_bd_net [get_bd_pins ps_regs/cmp_conf_q016]   [get_bd_pins rtl_compositor/conf_q016]
connect_bd_net [get_bd_pins ps_regs/cmp_low_conf]    [get_bd_pins rtl_compositor/low_conf]
# 三个本地写口：PS 写 CMP_*_WADDR 定地址、写 CMP_*_WDATA 时由 ps_regs 产生**一拍**写使能。
# 选这条路（而不是让 compositor 从 DDR 读图）的理由：共用同一条 AXI-Lite 基础设施、
# 不动 compositor 的读通路（避免与 1080p 扫描时序对抗），改动半径最小。
connect_bd_net [get_bd_pins ps_regs/wall_we]    [get_bd_pins rtl_compositor/wall_we]
connect_bd_net [get_bd_pins ps_regs/wall_waddr] [get_bd_pins rtl_compositor/wall_waddr]
connect_bd_net [get_bd_pins ps_regs/wall_wdata] [get_bd_pins rtl_compositor/wall_wdata]
connect_bd_net [get_bd_pins ps_regs/rec_we]     [get_bd_pins rtl_compositor/rec_we]
connect_bd_net [get_bd_pins ps_regs/rec_waddr]  [get_bd_pins rtl_compositor/rec_waddr]
connect_bd_net [get_bd_pins ps_regs/rec_wdata]  [get_bd_pins rtl_compositor/rec_wdata]
connect_bd_net [get_bd_pins ps_regs/osd_we]     [get_bd_pins rtl_compositor/osd_we]
connect_bd_net [get_bd_pins ps_regs/osd_waddr]  [get_bd_pins rtl_compositor/osd_waddr]
connect_bd_net [get_bd_pins ps_regs/osd_wdata]  [get_bd_pins rtl_compositor/osd_wdata]

# --- ps_regs 的状态回读输入（可观测性）---
# 注：raw_ovf_sticky / raw_proto_err / diff_ovf / diff_proto_err 这四条自 2026-10-02（A5）
#     起**同时**进 confidence_engine 的硬闸门（见下方 ps_regs/ce_ovf_* -> CE），
#     不再只是回读。回读位与闸门位是同一组状态脚的两个用途。
connect_bd_net [get_bd_pins rtl_raw_preproc/ovf_sticky]    [get_bd_pins ps_regs/raw_ovf_sticky]
connect_bd_net [get_bd_pins rtl_raw_preproc/proto_err]     [get_bd_pins ps_regs/raw_proto_err]
connect_bd_net [get_bd_pins rtl_raw_preproc/cfg_err]       [get_bd_pins ps_regs/raw_cfg_err]
connect_bd_net [get_bd_pins rtl_raw_preproc/busy]          [get_bd_pins ps_regs/raw_busy]
connect_bd_net [get_bd_pins rtl_raw_preproc/sat_pix_cnt]   [get_bd_pins ps_regs/raw_sat_pix_cnt]
connect_bd_net [get_bd_pins rtl_raw_preproc/total_pix_cnt] [get_bd_pins ps_regs/raw_total_pix_cnt]
connect_bd_net [get_bd_pins rtl_raw_preproc/frame_done]    [get_bd_pins ps_regs/raw_frame_done]
connect_bd_net [get_bd_pins rtl_vertical_diff/ovf_diff]    [get_bd_pins ps_regs/diff_ovf]
connect_bd_net [get_bd_pins rtl_vertical_diff/proto_err]   [get_bd_pins ps_regs/diff_proto_err]
connect_bd_net [get_bd_pins rtl_vertical_diff/busy]        [get_bd_pins ps_regs/diff_busy]
connect_bd_net [get_bd_pins rtl_fast_alert/alert_event]      [get_bd_pins ps_regs/fa_alert_event]
connect_bd_net [get_bd_pins rtl_fast_alert/alert_direction]  [get_bd_pins ps_regs/fa_alert_direction]
connect_bd_net [get_bd_pins rtl_fast_alert/alert_score]      [get_bd_pins ps_regs/fa_alert_score]
connect_bd_net [get_bd_pins rtl_fast_alert/alert_centroid_x] [get_bd_pins ps_regs/fa_alert_centroid_x]
connect_bd_net [get_bd_pins rtl_fast_alert/alert_delta_x]    [get_bd_pins ps_regs/fa_alert_delta_x]
connect_bd_net [get_bd_pins rtl_fast_alert/alert_centroid_valid] [get_bd_pins ps_regs/fa_alert_centroid_valid]
connect_bd_net [get_bd_pins rtl_fast_alert/frame_done]       [get_bd_pins ps_regs/fa_frame_done]
connect_bd_net [get_bd_pins rtl_fast_alert/proto_err]        [get_bd_pins ps_regs/fa_proto_err]
connect_bd_net [get_bd_pins rtl_fast_alert/zero_den]         [get_bd_pins ps_regs/fa_zero_den]
connect_bd_net [get_bd_pins rtl_fast_alert/e_ovf]            [get_bd_pins ps_regs/fa_e_ovf]
connect_bd_net [get_bd_pins rtl_fast_alert/cfg_err]          [get_bd_pins ps_regs/fa_cfg_err]
connect_bd_net [get_bd_pins rtl_confidence_engine/global_conf] [get_bd_pins ps_regs/ce_global_conf]
connect_bd_net [get_bd_pins rtl_confidence_engine/low_conf]  [get_bd_pins ps_regs/ce_low_conf]
connect_bd_net [get_bd_pins rtl_confidence_engine/done]      [get_bd_pins ps_regs/ce_done]
connect_bd_net [get_bd_pins rtl_confidence_engine/busy]      [get_bd_pins ps_regs/ce_busy]
connect_bd_net [get_bd_pins rtl_confidence_engine/cfg_err]   [get_bd_pins ps_regs/ce_cfg_err]
connect_bd_net [get_bd_pins rtl_compositor/frame_tick]       [get_bd_pins ps_regs/cmp_frame_tick]
connect_bd_net [get_bd_pins rtl_compositor/busy]             [get_bd_pins ps_regs/cmp_busy]

# --- RTL 之间的功能连线（口径：src/fpga/rtl/readme.md §2 的数据流）---
# confidence_engine 每帧一次触发；RTL 文档未指定触发源，本脚本按「每帧一次」把
# PS 在更新当前帧 margin/rho/mask 后写 CE_START，避免预处理完成时读取旧结果。
# 同一条 frame_done 顺带送进 ps_regs 做帧计数（同一条网、三个端点）。
connect_bd_net [get_bd_pins rtl_vertical_diff/frame_done] \
               [get_bd_pins ps_regs/diff_frame_done]
connect_bd_net [get_bd_pins ps_regs/ce_start] [get_bd_pins rtl_confidence_engine/start]
connect_bd_net [get_bd_pins rtl_confidence_engine/force_gray] \
               [get_bd_pins rtl_compositor/force_gray] \
               [get_bd_pins ps_regs/ce_force_gray]
connect_bd_net [get_bd_pins rtl_confidence_engine/gray_reason] \
               [get_bd_pins rtl_compositor/gray_reason] \
               [get_bd_pins ps_regs/ce_gray_reason]

# --- A5（2026-10-02）：数据完整性标志接入 force_gray 硬闸门 ---
# raw_preproc.ovf_sticky / vertical_diff.ovf_diff 与两者的 proto_err 经 ps_regs 转发进
# confidence_engine。此前它们只进 RO 状态寄存器，对当帧出图无约束。
connect_bd_net [get_bd_pins ps_regs/ce_ovf_raw]   [get_bd_pins rtl_confidence_engine/raw_ovf_sticky]
connect_bd_net [get_bd_pins ps_regs/ce_ovf_diff]  [get_bd_pins rtl_confidence_engine/ovf_diff]
connect_bd_net [get_bd_pins ps_regs/ce_proto_err] [get_bd_pins rtl_confidence_engine/proto_err]

# ---------------------------------------------------------------- 6.5 AXIS 通路
# DDR RAW ring --[dma_pipe MM2S]--> frm_raw --+--> raw_preproc --> vertical_diff
#                                             (补帧首 tuser + 行末 tlast)
#                    vertical_diff --> bc_diff --+--> fast_alert   （保持块行 tlast）
#                                                \--> frm_diff --> [dwc] --> dma_pipe S2MM --> DDR meas
#                                                              (只留帧末 tlast)
# ⚠ G2：MM2S 不产生 TUSER（其 M_AXIS_MM2S 是 TUSER_WIDTH 0）、且只在传输末尾置一次
#   TLAST；而 raw_preproc 要「帧首 tuser + 每行 tlast」。所以 raw 支路的整形器
#   **必须**放在 MM2S 与 raw_preproc 之间。同理 S2MM 以 TLAST 结束传输，差分支路
#   必须先把块行 tlast 换成帧末 tlast。
connect_bd_intf_net [get_bd_intf_pins dma_pipe/M_AXIS_MM2S]  [get_bd_intf_pins frm_raw/s_axis]
connect_bd_intf_net [get_bd_intf_pins frm_raw/m_axis]        [get_bd_intf_pins rtl_raw_preproc/s_axis]
connect_bd_intf_net [get_bd_intf_pins rtl_raw_preproc/m_meas] [get_bd_intf_pins rtl_vertical_diff/s_meas]
connect_bd_net      [get_bd_pins rtl_raw_preproc/m_meas_tch]  [get_bd_pins rtl_vertical_diff/s_meas_tch]
connect_bd_intf_net [get_bd_intf_pins rtl_vertical_diff/m_diff] [get_bd_intf_pins bc_diff/S_AXIS]
connect_bd_intf_net [get_bd_intf_pins bc_diff/M00_AXIS]       [get_bd_intf_pins rtl_fast_alert/s_diff]
connect_bd_net      [get_bd_pins rtl_vertical_diff/m_diff_tcol] [get_bd_pins rtl_fast_alert/s_diff_tcol]
# M01 支路：先整形（块行 tlast -> 帧末 tlast），再位宽转换、再进 S2MM
connect_bd_intf_net [get_bd_intf_pins bc_diff/M01_AXIS]       [get_bd_intf_pins frm_diff/s_axis]
connect_bd_intf_net [get_bd_intf_pins frm_diff/m_axis]        [get_bd_intf_pins dwc_meas/S_AXIS]
connect_bd_intf_net [get_bd_intf_pins dwc_meas/M_AXIS]        [get_bd_intf_pins dma_pipe/S_AXIS_S2MM]
# 显示支路（2026-10-01）：compositor 行末 tlast -> frm_disp 换成帧末 tlast -> dwc -> S2MM。
#   同时把 dwc_disp 的 S_AXIS tready 经 frm_disp 组合透传回 compositor 的 vid_tready
#   （G3：compositor 无 tready 时带宽不足会丢拍且不反压；现在光栅冻结、像素不丢）。
#   DP 在 PS 侧，DDR 帧缓冲由 PS 的 DP 控制器扫描输出。
connect_bd_intf_net [get_bd_intf_pins rtl_compositor/vid]     [get_bd_intf_pins frm_disp/s_axis]
connect_bd_intf_net [get_bd_intf_pins frm_disp/m_axis]        [get_bd_intf_pins dwc_disp/S_AXIS]
connect_bd_intf_net [get_bd_intf_pins dwc_disp/M_AXIS]        [get_bd_intf_pins dma_disp/S_AXIS_S2MM]

# 数据通路：所有核的所有 m_axi + 三个搬运 master -> sctl_mem -> PS S_AXI_HP0_FPD
set data_masters {}
foreach cell $core_names {
    foreach p [get_bd_intf_pins -quiet $cell/m_axi_*] { lappend data_masters $p }
}
lappend data_masters [get_bd_intf_pins dma_pipe/M_AXI_MM2S] \
                     [get_bd_intf_pins dma_pipe/M_AXI_S2MM] \
                     [get_bd_intf_pins dma_disp/M_AXI_S2MM]
set n_si [llength $data_masters]
if {$n_si != [expr {$n_mx + $N_DMA_MAST}]} {
    puts "ERROR: 数据主端口计数对不上（实得 $n_si，期望 [expr {$n_mx + $N_DMA_MAST}]）"
    exit 1
}
create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect sctl_mem
set_property -dict [list CONFIG.NUM_SI $n_si CONFIG.NUM_MI {1}] [get_bd_cells sctl_mem]
connect_bd_net [get_bd_pins zynq_ps/pl_clk0]           [get_bd_pins sctl_mem/aclk]
connect_bd_net [get_bd_pins rst_pl/peripheral_aresetn] [get_bd_pins sctl_mem/aresetn]
connect_bd_intf_net [get_bd_intf_pins sctl_mem/M00_AXI] [get_bd_intf_pins zynq_ps/${ps_slave}]

# 控制：逐核、逐接口接（两个从接口都要接，见上文 ⚠）
set idx 0
foreach cell $core_names {
    foreach ifc $CTRL_IFACES {
        connect_bd_intf_net [get_bd_intf_pins $cell/$ifc] \
            [get_bd_intf_pins sctl_ctrl/[format "M%02d_AXI" $idx]]
        incr idx
    }
}
puts "  控制通路接法 = [llength $core_names] 核 x [llength $CTRL_IFACES] 接口 = $n_ctrl 条"

# 控制：搬运器的 S_AXI_LITE 继续排在四个核之后（PS 用它写源/目的地址与长度、读状态）
foreach cell {dma_pipe dma_disp} {
    connect_bd_intf_net [get_bd_intf_pins $cell/s_axi_lite] \
        [get_bd_intf_pins sctl_ctrl/[format "M%02d_AXI" $idx]]
    incr idx
}
# 控制：最后一个是 RTL 寄存器叶（G1/G4/G5）。名字由端口名（s_axi_awaddr 等）推断而来。
connect_bd_intf_net [get_bd_intf_pins ps_regs/s_axi] \
    [get_bd_intf_pins sctl_ctrl/[format "M%02d_AXI" $idx]]
incr idx
puts "  控制通路合计 = $n_ctrl_tot 条（含搬运器 $N_DMA_LITE 条 S_AXI_LITE + 寄存器叶 1 条）"

# 数据：逐组接（data_masters 已在 §6.5 建好 = 核的 m_axi + 搬运 master）
set idx 0
foreach p $data_masters {
    connect_bd_intf_net $p [get_bd_intf_pins sctl_mem/[format "S%02d_AXI" $idx]]
    incr idx
}
puts "  数据通路接法 = $n_si 个 master -> $ps_slave"

# ---------------------------------------------------------------- 6.6 连通性自检
# 教训（2026-09-28 实测）：`validate_bd_design` **不会**因为「IP 的从接口没接」而失败。
# 当时四个核的 `s_axi_control_r` 全悬空，校验照样通过、一条告警都没有。
# 所以这里自己查一遍：各核的**每个**从接口与**每个** m_axi 都必须有连接。
foreach cell $core_names {
    foreach p [get_bd_intf_pins -quiet $cell/*] {
        if {[get_property MODE $p] ne "Slave"} { continue }
        if {[llength [get_bd_intf_nets -quiet -of_objects $p]] == 0} {
            puts "ERROR: $cell 的从接口 $p 没有连接"
            exit 1
        }
    }
    foreach p [get_bd_intf_pins -quiet $cell/m_axi_*] {
        if {[llength [get_bd_intf_nets -quiet -of_objects $p]] == 0} {
            puts "ERROR: $cell 的 m_axi $p 没有连接"
            exit 1
        }
    }
}
# 本版新增：RTL 单元 / 搬运器 / 位宽适配器
#   (a) 除 fast_alert 的 s_coef 与 ps_regs 的 coef（两者的成员都是**逐根**接的
#       标量/管脚，工具推断出来的 AXIS 接口本身不建网络）外，所有接口都必须有网络；
#   (b) 每个单元的非接口输入脚都必须有驱动（漏钉一根常量就会没有驱动）。
set CHK_CELLS {rtl_raw_preproc rtl_vertical_diff rtl_fast_alert rtl_confidence_engine \
               rtl_compositor bc_diff dwc_meas dwc_disp dma_pipe dma_disp \
               frm_diff frm_raw frm_disp ps_regs irq_concat irq_gnd}
set SKIP_INTF {rtl_fast_alert/s_coef ps_regs/coef}
foreach cell $CHK_CELLS {
    set members {}
    foreach ip [get_bd_intf_pins -quiet $cell/*] {
        if {[lsearch -exact $SKIP_INTF "$cell/[get_property NAME $ip]"] >= 0} { continue }
        if {[llength [get_bd_intf_nets -quiet -of_objects $ip]] == 0} {
            puts "ERROR: $cell 的接口 $ip 没有连接"
            exit 1
        }
        foreach mp [get_bd_pins -quiet -of_objects $ip] { lappend members [get_property NAME $mp] }
    }
    foreach p [get_bd_pins -quiet $cell/*] {
        if {[lsearch -exact $members [get_property NAME $p]] >= 0} { continue }
        if {[get_property DIR $p] ne "I"} { continue }
        if {[llength [get_bd_nets -quiet -of_objects $p]] == 0} {
            puts "ERROR: $cell 的输入 $p 没有驱动"
            exit 1
        }
    }
}
puts "  连通性自检 = 各核从接口/m_axi、RTL 单元接口与输入脚均已连接 ✓"

# ---------------------------------------------------------------- 7 地址映射
# 控制：每核 64 KiB 一段，连续排布。板端软件按同一顺序算基址。
#
# ⚠ 这里**必须先拿到 addr_seg 对象再传给 assign_bd_address**，不能直接塞字符串：
#   塞 'core_proj/s_axi_control/Reg' 会报
#     [Common 17-161] Invalid option value 'core_proj/s_axi_control/Reg' specified for 'objects'
#   所以下面按「枚举全部段 -> 按模式匹配」的方式取对象。
if {[catch {
    set ps_space [get_bd_addr_spaces -quiet zynq_ps/Data]
    if {[llength $ps_space] == 0} {
        error "找不到 zynq_ps/Data 地址空间"
    }
    set ps_space [lindex $ps_space 0]
    puts "  PS 地址空间 = $ps_space"

    puts "  可见的地址段："
    foreach s [get_bd_addr_segs -quiet] { puts "    $s" }

    # 控制段：每核**两个**接口，各 64 KiB，连续排布
    #   核 i 占两段：s_axi_control 在 BASE+(2i)*0x10000、s_axi_control_r 在 +0x10000
    # 板端软件按此算基址；顺序 = core_names 的顺序。
    set ctrl_base $CTRL_BASE
    foreach cell $core_names {
        foreach ifc $CTRL_IFACES {
            set segs {}
            foreach s [get_bd_addr_segs -quiet] {
                if {[string match "*/$ifc/Reg" $s] && [string match "*/$cell/*" $s]} {
                    lappend segs $s
                }
            }
            if {[llength $segs] == 0} { error "$cell 找不到 $ifc/Reg 地址段" }
            assign_bd_address -offset [format 0x%08X $ctrl_base] -range $CTRL_RANGE \
                -target_address_space $ps_space [lindex $segs 0]
            puts [format "  控制 %-12s %-16s 0x%08X + %s" $cell $ifc $ctrl_base $CTRL_RANGE]
            incr ctrl_base 0x10000
        }
    }

    # 搬运器的 S_AXI_LITE 控制段（AXI DMA 的寄存器组），继续排在四个核之后。
    # 板端软件按此写：源地址/目的地址/长度，并读回状态与中断。
    foreach cell {dma_pipe dma_disp} {
        set segs {}
        foreach s [get_bd_addr_segs -quiet] {
            if {[string match "*/$cell/*/Reg" $s]} { lappend segs $s }
        }
        if {[llength $segs] == 0} { error "$cell 找不到 S_AXI_LITE 的地址段" }
        assign_bd_address -offset [format 0x%08X $ctrl_base] -range $CTRL_RANGE \
            -target_address_space $ps_space [lindex $segs 0]
        puts [format "  控制 %-12s %-16s 0x%08X + %s" $cell "s_axi_lite" $ctrl_base $CTRL_RANGE]
        incr ctrl_base 0x10000
    }

    # RTL 寄存器叶（G1/G4/G5）的控制段：继续排在两个搬运器之后 -> 0xA00A0000。
    # 板端软件：见 board/ps_pl_regs.h（寄存器表与偏移）。
    # ⚠ 模块引用推断出来的 AXI-Lite 从口，其 memory map 名是 **小写 `reg0`**
    #   （IP 核才是 `Reg`，见上一次运行的段列表：/ps_regs/s_axi/reg0）。
    #   所以这里用 `-nocase` 匹配，不能照抄核那一段的 `*/…/Reg`。
    set segs {}
    foreach s [get_bd_addr_segs -quiet] {
        if {[string match -nocase "*/ps_regs/s_axi/*" $s]} { lappend segs $s }
    }
    if {[llength $segs] == 0} { error "ps_regs 找不到地址段（PL 控制页）" }
    assign_bd_address -offset [format 0x%08X $ctrl_base] -range $CTRL_RANGE \
        -target_address_space $ps_space [lindex $segs 0]
    puts [format "  控制 %-12s %-16s 0x%08X + %s" "ps_regs" "s_axi" $ctrl_base $CTRL_RANGE]
    incr ctrl_base 0x10000

    # 数据段：每个核的每个 m_axi 都是独立 master 地址空间，逐个映射。
    # ⚠ PS 的 DDR 段**不叫 `zynq_ps/Data`**，而是**按从端口分组**的：
    #     /zynq_ps/SAXIGP2/HP0_DDR_LOW    0x0_0000_0000 + 2 GiB
    #     /zynq_ps/SAXIGP2/HP0_DDR_HIGH   0x8_0000_0000 + 2 GiB
    #   （SAXIGP2 对应 HP0、SAXIGP3 对应 HP1；本设计只接了 HP0，所以只有
    #    SAXIGP2 那一组可达。实测把两组都列了出来。）
    #   **不传 -offset/-range**，用段自身的原生范围 —— 手抄地址是这类脚本最
    #   容易出错的地方，交给工具。
    set ddr_segs {}
    foreach s [get_bd_addr_segs -quiet] {
        if {[string match "*HP0_DDR_*" $s]} { lappend ddr_segs $s }
    }
    if {[llength $ddr_segs] == 0} { error "找不到 HP0 的 DDR 地址段" }
    puts "  DDR 段 = $ddr_segs"
    set n_sp 0
    foreach p $data_masters {
        foreach sp [get_bd_addr_spaces -quiet -of_objects $p] {
            foreach dseg $ddr_segs {
                catch { assign_bd_address -target_address_space $sp $dseg }
            }
            incr n_sp
        }
    }
    puts "  数据：$n_sp 个 master 地址空间 -> 上述 DDR 段"
} err]} {
    puts "ERROR: 地址映射失败：$err"
    exit 1
}

# ---------------------------------------------------------------- 8 校验与落盘
# Active-low J10 fan power, independent of PS/clocks/reset.
create_bd_port -dir O fan
create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant:1.1 fan_always_on
set_property -dict [list CONFIG.CONST_WIDTH {1} CONFIG.CONST_VAL {0}] [get_bd_cells fan_always_on]
connect_bd_net [get_bd_pins fan_always_on/dout] [get_bd_ports fan]
regenerate_bd_layout
if {[catch {validate_bd_design} err]} {
    puts "ERROR: validate_bd_design 失败：$err"
    puts "       块设计未通过校验，**不保存** —— 免得留下一个看着像成功的产物。"
    exit 1
}
save_bd_design
make_wrapper -files [get_files ${BD_NAME}.bd] -top
puts "==== 块设计 $BD_NAME 已保存，wrapper 已生成 ===="

# 地址映射回执：板端软件的基址必须照抄这里
puts "\n---- 地址映射（板端软件按此写寄存器；顺序即 core_names）----"
foreach seg [get_bd_addr_segs -quiet -of_objects [get_bd_cells -quiet $core_names]] {
    puts "  $seg"
}
puts "  core_names 顺序 = $core_names"

puts "\n---- 本脚本没做、也不该声称做完的 ----"
puts "  1) 综合 / 实现 / 时序收敛（WNS）/ 资源占用 / bitstream"
puts "     （综合由 src/fpga/scripts/build_pl.tcl 跑；本脚本只到 validate + wrapper）"
puts "  2) 板端 PS 软件：那套 s_axilite 偏移来自定点的四个顶层，"
puts "     **与双精度层的 ps_axi_top.h 不是一套**，须按新 IP 的 xps_*_hw.h 重写"
puts "  3) m_axi 位宽与突发长度：csynth 报 Max Widen Bitwidth = 0，"
puts "     这是 X19 里那条「访存带宽是瓶颈」的根源，块设计侧改不了，要回 HLS 侧改端口"
puts "\n---- 缺口状态（编号沿用 sim/results/bd_full_path_20260930.md §7）----"
puts "  G1) **已闭合（结构层）**：新增模块引用 pl_ctrl_regs（纯 Verilog），一个 AXI4-Lite"
puts "      从口 + 4 KiB 寄存器页，地址 0xA00A0000。五个 RTL 模块的 cfg_*/阈值全部改为"
puts "      由它驱动（xlconstant 钉扎已删除；2026-09-30 收口 G5 时连最后一条"
puts "      mask_rdata 常量也换成了 ps_regs 内的 432x8 RAM）。寄存器表见"
puts "      board/ps_pl_regs.h。除 CE_CFG bit3（饱和比例来源）外，复位默认值 = 上一版"
puts "      钉扎值；bit3 的默认 1 是本轮 G5 收口的有意变更（PS 清 0 即退回旧档）。"
puts "      未闭合的部分：表格/c_k 的**来源**仍要 PS 写（μ/v 表已可写可读；见 G5）。"
puts "  G2) **已闭合（结构层）**：frm_raw 插在 dma_pipe MM2S 与 raw_preproc 之间"
puts "      （补帧首 tuser + 每条 raw 行末 tlast）；frm_diff 插在 bc_diff/M01 与 dwc_meas"
puts "      之间（把 95 次/帧的块行 tlast 换成 1 次/帧的帧末 tlast）。fast_alert 那一支"
puts "      的块行 tlast 刻意不动。RTL 侧已 xsim 验证（见报告）。"
puts "  G3) **已闭合（结构层，2026-10-01）**：compositor 新增 vid_tready 输入（拉低冻结整条"
puts "      光栅：hcnt/vcnt + 输出流水 + BRAM 同步读），BD 里由 dwc_disp/S_AXIS 的 tready 经"
puts "      frm_disp 组合透传回它。DDR 写带宽不足时不再丢拍，而是光栅冻结、帧率自适应。"
puts "      同一条显示支路还把 compositor 的**行末 tlast** 整形为**帧末 tlast**"
puts "      （frm_disp，FRAME_BEATS=1920*1080=2,073,600）—— 否则 S2MM 会把一帧切成 1080 次传输。"
puts "      RTL 侧由 tb/compositor_tready_tb.sv 逐拍验证（tready=1 时行为与旧版一致）。"
puts "  G4) **已闭合（结构层）**：ps_regs 提供 wall/rec/osd 三组「地址 + 数据」寄存器，"
puts "      写数据寄存器时产生一拍本地写使能。上板时四分区里的原图/重建图由 PS 灌入。"
puts "  G5) **部分闭合（本轮再收两项）**："
puts "      已闭合（结构层）：c_k 系数流（FA_COEF_PUSH）、μ/v 小表、四项量的可写寄存器、"
puts "        饱和比例的 **PL 内自动来源**（sat_ratio_div，CE_CFG bit3 默认选它）、"
puts "        432 项掩膜表（ps_regs 内 432x8 RAM，写口 MASK_ADDR/MASK_DATA，上电 0xFF）——"
puts "        本脚本的**配置/阈值**通路已无任何 xlconstant 钉扎（唯一保留的 xlconstant 是"
puts "        6.3b 的 irq_gnd，属中断汇总的未用输入接 0，与配置无关）。"
puts "      未闭合：margin 的 PL 内来源（在 core_proj 的 s_axilite 寄存器里，"
puts "        需要给 PL 加 AXI4-Lite 主口读回）；ρ 的 PL 内来源（在 core_pdhg 的"
puts "        gmem1_data DDR 缓冲，需要 PS 读回或 DDR 读主口）；SNR（上游无产源）；"
puts "        几何哈希与 ps_model/psm 返回值的对接。"
puts "      逐项状态与代价见 sim/results/g5_g3_raw_fix_20261001.md。"
puts "  G6) **本轮新发现**：MM2S 的 M_AXIS_MM2S 是 TUSER_WIDTH 0，**不产生 tuser**；"
puts "      而 raw_preproc 在 ST_IDLE 等 `tvalid && tuser`。不补 frm_raw 这条链一帧都进不去。"
puts "  G7) **已闭合（结构层，2026-10-02）**：PL->PS 中断通路。原先两个 AXI DMA 的"
puts "      mm2s_introut / s2mm_introut 全部悬空、PL 无中断输出，PS 只能轮询 ap_done。"
puts "      现新增 xlconcat(8) + xlconstant 0，把三个 DMA 中断接到 zynq_ps/pl_ps_irq0"
puts "      （宽度按 PG201 的 pl_ps_irq0[7:0]，未用 In3..In7 接 0）。PS 侧驱动仍按"
puts "      ap_done 轮询，**中断为可选通路**；要走中断需设备树与唤醒循环，属独立改动。"
