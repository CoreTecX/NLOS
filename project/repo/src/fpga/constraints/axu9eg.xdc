# ============================================================================
# PenumbraSight 时序与实现约束 · src/fpga/constraints/axu9eg.xdc
#
# 目标板：ALINX AXU9EGB（核心板 XCZU9EG-2FFVB1156I，工业级 -2，FFVB1156），
#         与 build_hls.tcl / penumbrasight_bd.tcl 的 PART = xczu9eg-ffvb1156-2-i 一致。
#
# 器件串出处：ALINX 官方手册《AXU9EGB开发板用户手册》正文，三处表述一致 ——
#   「型号为 XCZU9EG-2FFVB1156I」「速度等级为 -2，工业级，封装为 FFVB1156」。
#   与 01_分析报告/08_阶段A产出/A2_资源预算.md 的 DSP = 2520 互相印证。
#
# ============================================================================
# 本文件为什么这么短 —— 这是一个结论，不是省略
# ============================================================================
#
# 整个设计的对外接口**全部在 PS 侧**：
#
#     相机 -> (USB/GigE) -> PS 上的 ARM 软件 -> AXI -> PL 里的四个 HLS 核
#                                              <- AXI <-
#
# PL 与外界之间**没有任何用户 I/O**（没有 LVDS、没有 DVP、没有 GPIO、
# 没有外接时钟）。所以：
#
#   · **不需要任何 PACKAGE_PIN / IOSTANDARD 约束** —— 没有要约束的引脚；
#   · 相机时钟是 PS 侧外设时钟，不进 PL，**不需要 create_clock**；
#   · 全 PL 设计只跑一个时钟（PS 的 pl_clk0），**没有跨时钟域，
#     不需要 set_clock_groups -asynchronous**；
#   · 复位用 proc_sys_reset（异步复位同步释放），**不需要手写 false_path**。
#
# 也就是说：一个 PS + AXI 加速器的设计，XDC 天然是薄的。
# 如果有人指望在这个文件里看到一堆引脚约束，那说明接口方案被理解错了 ——
# 本设计**不直接接相机**，这一点在 docs/hardware_freeze.md 里有完整说明。
#
# ⚠ 本文件在 Vivado 里是**被 read_xdc 读进去的**（src/fpga/scripts/build_pl.tcl §2）。
#    它的正确性依赖于上面那条「PL 无用户 I/O」的架构判断；一旦加入任何 PL 侧外设，
#    必须相应补 IOSTANDARD 与 PACKAGE_PIN，并重新审视时钟域划分。
#
# ============================================================================
# 2026-09-30 补：显示通路归属已核实 —— **DP 在 PS 侧**
# ============================================================================
# 结论：板载 mini-DP 由 PS 的 DisplayPort 控制器驱动，与 PL 无关。因此在把
# RTL 五模块（raw_preproc/vertical_diff/fast_alert/confidence_engine/compositor）
# 与搬运通路接进块设计之后，**仍无任何新增 PL 侧用户 I/O**，本文件继续保持
# 「无引脚约束、无相机时钟、单时钟域」的形态。这不是漏写，是归属决定的。
#
# 依据一（板卡手册）：《AXU9EGB开发板用户手册》第 35 页「(三)DP显示接口」原文：
#   「DisplayPort数据传输通道直接用ZU9EG的BANK505PSMGT驱动输出，MGT的LANE2和
#     LANE3TX信号以差分信号方式连接到DP连接器。DisplayPort辅助通道连接到PS的
#     MIO管脚上。」
#   同页引脚表：GT0_DP_TX_P/N = 505_TX3_P/N（V29/V30）、
#               GT1_DP_TX_P/N = 505_TX2_P/N（W31/W32）、
#               505_DP_CLK_P/N = 505_CLK2_P/N（U27/U28）、
#               DP_AUX_OUT/IN/OE/HPD = PS_MIO27/30/29/28。
#   即：数据通道在 PS 的 MGT（BANK505），AUX 在 PS 的 MIO —— **没有一条落在 PL**。
#
# 依据二（原理图）：底板原理图第 2 页网名同样只有 GT0/GT1_DP_TX_*、505_DP_CLKP/N
#   与 DP_OE/AUX_IN/AUX_OUT/HPD_MIO27..30，**没有任何 DP 信号落在 PL 侧 bank**。
#   （文件见 `99_临时工作区/hw/00_hardware/01_原理图结构图等硬件资料/底板/AXU9EGB底板原理图.pdf`，
#     出厂资料包 `开发板资料/AXU9EG/00_hardware.zip` 内同名。）
#
# 依据三（本仓库的 PS 预设，与上面两条吻合）：
#   src/fpga/bd/ps_preset_axu9eg_alinx.tcl 已启用 PS 侧 DP 控制器 ——
#     PSU__DISPLAYPORT__PERIPHERAL__ENABLE = 1
#     PSU__DISPLAYPORT__LANE0__ENABLE = 1 / LANE0__IO = GT Lane3   （= 505_TX3）
#     PSU__DISPLAYPORT__LANE1__ENABLE = 1 / LANE1__IO = GT Lane2   （= 505_TX2）
#     PSU__DPAUX__PERIPHERAL__ENABLE = 1 / PERIPHERAL__IO = MIO 27 .. 30
#   （行号 999~1009）
#
# 对块设计的直接后果：compositor 的 1080p RGB 流只是**帧缓冲的生产者**，经
# AXI DMA（S2MM）写进 DDR，由 PS 的 DP 控制器扫描输出。块设计里**不需要**
# v_tc / video-out / HDMI TX 之类的 PL 视频 IP，因此本文件也不需要任何视频引脚
# 约束，更不需要给「像素时钟」单独 create_clock（单时钟域仍是 pl_clk0）。
# 详细论证与综合结果见 sim/results/bd_full_path_20260930.md。
# ============================================================================

# ---------------------------------------------------------------- 1 器件校验
# 目标器件：**xczu9eg-ffvb1156-2-i**（ALINX AXU9EGB 核心板 XCZU9EG-2FFVB1156I，
# 速度等级 -2、工业级、封装 FFVB1156）。与 build_hls.tcl / penumbrasight_bd.tcl 的
# PART 同值。器件不符时应在工程侧暴露，**不在本文件里判断**。
#
# ⚠ 2026-09-30 实测纠正：本文件原先在这里写了
#       set target_part [get_property PART [current_project]]
#       if {$target_part ne "xczu9eg-ffvb1156-2-i"} { ... }
#   **这两句不是合法的 XDC 命令**，Vivado 在综合阶段报：
#       CRITICAL WARNING: [Designutils 20-1307] Command 'current_project' is not
#           supported in the xdc constraint file. [axu9eg.xdc:40]
#       CRITICAL WARNING: [Designutils 20-1307] Command 'if' is not supported in
#           the xdc constraint file. [axu9eg.xdc:41]
#   XDC 是**约束**语言（只接受 create_clock / set_property / get_ports 这类约束命令），
#   `current_project`、`if` 是 Tcl 脚本构造，写在这里既不起作用又制造 CRITICAL WARNING。
#   同一对告警在改动前的构建区（D:\psight\build_t2 的 synth_1/runme.log）**就已经是 2 条**，
#   属既有问题，不是本次引入。故此处改成注释，器件校验交给脚本侧
#   （build_pl.tcl 会打印 `器件 = [get_property PART [current_project]]`）。
#   注：本文件按仓库既有先例（2026-09-29 的 CFGBVS / CONFIG_VOLTAGE 一处）
#   把「实测结论 + 出处」写进注释，而不是留一段跑不起来的 Tcl。

# ---------------------------------------------------------------- 2 主时钟
# pl_clk0 由块设计从 PS 自动推导，周期由 PS 配置（PSU__CRL_APB__PL0_REF_CTRL__FREQMHZ）
# 决定，本工程设为 100 MHz。
#
# ★ 周期的**单一来源**是 src/fpga/scripts/clock_plan.tcl（10.0 ns = 100 MHz，
#   2026-10-02 起）。build_hls.tcl 与 penumbrasight_bd.tcl 都 source 它；本 xdc
#   是约束文件，Vivado **不允许**在 xdc 里 source 任意 Tcl（见本文件顶部
#   `current_project` / `if` 被 [Designutils 20-1307] 拒的实测记录），因此这里
#   **不再重复 10.000 这个字面量**，只留这条指路说明。改档只改 clock_plan.tcl。
#
# 为什么是 100 而不是 150/200（2026-09-28 X19 实测）：
#   200 MHz  ps_projection_fix Slack -1.90、ps_pdhg_fix -1.39   -> 违例
#   150 MHz  ps_pdhg_fix 仍 -0.51（瓶颈链路组合延迟 5.385 ns + 1.8 ns 不确定度）
#   100 MHz  四个定点核 Slack 全 0.00                            -> 全部收敛
# 这是「不追求更高主频」原则下的余量选择，不是板上限（ALINX 出厂工程跑 200）。
#
# 这里**刻意不再 create_clock**：BD 已经建了这个时钟，重复创建会产生两个
# 同名时钟，反而是引入约束错误。若将来 PL 侧加了独立晶振，再按下面模板加。
#
# 如果需要在 XDC 里显式覆盖周期（一般不需要），用（period 从 clock_plan.tcl 抄）：
#     create_clock -period 10.000 -name pl_clk0 [get_ports pl_clk0]

# ---------------------------------------------------------------- 3 时钟域
# 单时钟设计，无需 set_clock_groups。加入 PL 侧独立时钟源后，若两者无相位
# 关系，需要在下面打开对应的异步分组（示例，现不生效）：
#
#     set_clock_groups -asynchronous \
#         -group [get_clocks pl_clk0] \
#         -group [get_clocks ext_clk_in]

# ---------------------------------------------------------------- 4 复位
# rst_pl（proc_sys_reset）内部已做异步复位、同步释放，其跨域路径由 IP 自身
# 约束覆盖。无需手写 set_false_path —— 手写反而会掩盖真实的时序问题。

# ---------------------------------------------------------------- 5 生成位流
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]

# ALINX AXU9EGB 的配置模式（QSPI FLASH / JTAG）由核心板固定，**不在本文件设定**。
# 手册记核心板 QSPI FLASH 为 512 Mb。

# CFGBVS / CONFIG_VOLTAGE —— **本器件不支持，因此刻意不写**（2026-09-29 实测）。
#
# 这两条是 7 系列工程的常见套路，很容易被照抄到 UltraScale+ 上。本设计**不照抄**。
#
# 器件侧（XCZU9EG 属 Zynq UltraScale+，即 UltraScale+，与 7 系列语义不同）：
#   · UG570《UltraScale Architecture Configuration》v1.9.1 第 10-11 页：
#     「The configuration interface can operate only at 1.8V or 1.5V in the
#       UltraScale+ FPGAs. There is no CFGBVS pin in UltraScale+ devices.」
#     UltraScale+ **没有 CFGBVS 引脚**；该引脚位在迁移时变为 RSVDGND，必须接 GND。
#   · UG583《UltraScale Architecture PCB Design》v1.12.1 第 207 页：
#     「UltraScale+ FPGAs do not support 3.3V operation on bank 0」。
#   · 核心板原理图（`00_hardware.zip` 内「ACU9EG核心板原理图.pdf」）第 2 页
#     「Bank0 & BANK503」的 BANK0 符号（U1-1）**只引出 PUDC_B / POR_OVERRIDE /
#     VCCADC / GNDADC / VREFP / VREFN / DXP / DXN / VP / VN，没有 CFGBVS 引脚**；
#     第 22 页 PMIC（U12 TPS6508640）给出 BUCK5 = 1.8V -> VCC_AUX，配置专用域为 1.8V。
#
# 工具侧（本机 Vivado 2025.2 实测，探针 `src/fpga/scripts/probe_cfg_voltage.tcl`）：
#   两个属性在 `list_property [current_design]` 里**根本不存在**，设置时得到
#     WARNING: [Vivado 12-5460] The attribute CONFIG_VOLTAGE is not supported
#              in the xczu9eg-ffvb1156-2-i device
#     WARNING: [Vivado 12-5460] The attribute CFGBVS is not supported ...
#   走 read_xdc 只报警告、返回码 0（**不会中止实现**），但在 Tcl 脚本里直接调用
#   `set_property` 会返回错误（rc=1）。也就是说：写进 XDC 不会炸，但也**完全没有作用**。
#   · 结论：**不写**。写了是空转，还会让读的人误以为「按 7 系列的规矩设了配置电压」；
#     器件默认即 CONFIG_VOLTAGE = 1.8、CFGBVS = GND，与本板的 1.8V 配置域一致。
#
# ⚠ 一条被实测纠正的旧说法：曾以为「这两项留空导致位流只能 JTAG 加载」。**不成立。**
#    CFGBVS / CONFIG_VOLTAGE 只影响 DRC 与 IBIS/CSV 导出的电压一致性检查，**不决定
#    启动介质**。QSPI 启动取决于 MODE[3:0] 的设置、Flash 内位流头部（`write_cfgmem`
#    参数）与配置时钟，与本文件无关。完整证据见 `sim/results/cfg_voltage_probe.md`。

# ---------------------------------------------------------------- 6 待办（PL 侧接外设时）
# 下列内容**现在不需要**，列出来是为了标明「加外设时必须补哪几项」：
#
#   相机并口（若改为 PL 直采，而不是走 PS 侧 USB/GigE）：
#       set_property -dict {PACKAGE_PIN <引脚> IOSTANDARD LVCMOS18} [get_ports {cam_*}]
#       create_clock -period <像素时钟周期> -name cam_pclk [get_ports cam_pclk]
#       set_input_delay  -clock cam_pclk <ns> [get_ports {cam_data[*]}]
#       set_output_delay -clock cam_pclk <ns> [get_ports {cam_*}]
#
#   LED / 按键（仅用于状态指示，非数据通路）：
#       set_property -dict {PACKAGE_PIN <引脚> IOSTANDARD LVCMOS18} [get_ports {led_*}]
#
# 引脚号必须查 ALINX 随板资料里的「AXU9EG管脚定义.xls」与「AXU9EGB底板原理图」
# （均在 `开发板资料/AXU9EG/00_hardware.zip` 内），**不能从别处抄**。
#
# 可用 I/O 规模（ALINX 手册）：PL 侧 HP I/O 96 个、HD I/O 66 个、GTH 收发器若干
# （手册两处分别写 16 / 24，**以原理图为准**）。本设计当前**不占任何 PL 侧用户 I/O**，
# 所以上面那些约束一条都不需要 —— 这是架构决定的，不是漏写。

# AXU9EGB fan: low = full-speed on.
set_property -dict {PACKAGE_PIN J10 IOSTANDARD LVCMOS33 DRIVE 8} [get_ports fan]
