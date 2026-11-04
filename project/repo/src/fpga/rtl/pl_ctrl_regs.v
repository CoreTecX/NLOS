// ============================================================================
// pl_ctrl_regs.v  ·  PL 侧控制/回读寄存器叶（G1 + G4 + G5 的落地件）
// ---------------------------------------------------------------------------
// 为什么需要它（问题陈述，出处逐条给出）
// ---------------------------------------------------------------------------
//   G1：五个 RTL 模块的文件头都把 `cfg_*` 与阈值写成「AXI-Lite 影子配置（由
//       fpga_control 写，帧内不可变）」（例如 `raw_preproc.sv:190-194`），
//       但仓库里没有任何「AXI-Lite -> RTL 标量脚」的桥。上一版块设计只能用
//       xlconstant 钉死全部配置与阈值 —— 见 sim/results/bd_full_path_20260930.md §7 G1。
//   G4：`compositor.sv:199-222` 的 `wall_*` / `rec_*` / `osd_*` 是**本地写口**
//       （写使能 + 地址 + 数据），不是 AXI 从口，PS 无法直接写；上一版钉 0，
//       上板后四分区里的原图与重建图会是空的 —— 同报告 §7 G4。
//   G5：`fast_alert` 的 c_k / μ / v 与 `confidence_engine` 的四项实时量没有 PL 内
//       来源 —— 同报告 §7 G5 与 src/fpga/rtl/README.md F2。
//
// 本模块一次解决三件事（不新增第二套 AXI-Lite 基础设施）：
//   1) 一个 AXI4-Lite 从口 + 一页寄存器（4 KiB，ADDR_W=12），把五个 RTL 模块的
//      `cfg_*` / 阈值做成**可写**、把状态做成**可读**（G1）。
//   2) 把 compositor 的三个本地写口接上：写 `CMP_*_WADDR` / `CMP_*_WDATA` 时，
//      本模块自动产生**一拍**写使能（G4）。选这条而不是「改 compositor 从 DDR 读图」，
//      理由见报告：改动半径小、与 G1 共用同一条 AXI-Lite 基础设施、且不动
//      compositor 的读通路（避免它与扫描时序的对抗）。
//   3) 给 `fast_alert` 提供 c_k 流与 μ/v 小表（G5 的一部分）。
//   4) G5 剩余项里**能在 RTL 层闭合的两项**（2026-09-30 收口；复核与报告见
//      sim/results/g5_g3_raw_fix_20261001.md）：
//      a) 饱和比例：内部实例化 `sat_ratio_div`（源文件 src/fpga/rtl/sat_ratio_div.v），
//         由 `raw_preproc` 的 `sat_pix_cnt / total_pix_cnt` 算出 Q0.16 比值，
//         默认**直接驱动** confidence_engine 的 `sat_ratio_q016`（CE_CFG bit3=1）；
//         置 bit3=0 可退回「PS 写 CE_SAT」的手动档。
//      b) 432 项可观测性掩膜表：一块 432x8 的 RAM（写口由 MASK_ADDR / MASK_DATA
//         两个寄存器给出，读口由 confidence_engine 的 `mask_raddr` 驱动），
//         取代上一版的 `xlconstant 255`。上电内容 = 0xFF（与旧常量逐位相同）。
//      ⚠ 这两项都只在 RTL/块设计层闭合；板端是否真的能跑通要上板才算。
//
// ---------------------------------------------------------------------------
// 寄存器表（字节偏移；基址由块设计 §7 分配，上一版之后新增的一段）
// ---------------------------------------------------------------------------
//   0x000 RO ID          = 0x5053_0001（"PS" + 版本 1）
//   0x004 RO VERSION     = 1
//   0x008 RW SCRATCH     任意读写量（链路自检用）
//   0x00C RW CTRL        bit0 = 1 写：把全部 RW 寄存器**重载为默认值**（自清）
//   0x010 RO STAT_BUSY   b0 raw_busy b1 diff_busy b2 ce_busy b3 cmp_busy
//   0x014 RO STAT_STICKY b0 raw_ovf b1 raw_proto b2 raw_cfg_err
//                        b3 diff_ovf b4 diff_proto
//                        b5 fa_proto b6 fa_zero_den b7 fa_e_ovf b8 fa_cfg_err
//                        b9 ce_cfg_err
//   0x018 RO STAT_ALERT  b0 fa_alert_event [2:1] fa_direction
//                        b3 fa_centroid_valid b4 ce_low_conf b5 ce_force_gray
//                        [10:6] ce_gray_reason[4:0]（原有五项）
//                        [13:11] ce_gray_reason[7:5]（数据完整性，2026-10-02 新增）
//   0x01C RO CTR_RAW     raw_preproc frame_done 计数
//   0x020 RO CTR_DIFF    vertical_diff frame_done 计数
//   0x024 RO CTR_FA      fast_alert frame_done 计数
//   0x028 RO CTR_CE      confidence_engine done 计数
//   0x02C RO CTR_DISP    compositor frame_tick 计数
//   ---- raw_preproc ----
//   0x040 RW RAW_CFG0    [15:0] dark_level [19:16] frames [20] bayer_phase
//   0x044 RW RAW_CFG1    [15:0] crop_side
//   0x048 RO RAW_STAT    b0 ovf_sticky b1 proto_err b2 cfg_err b3 busy
//   0x04C RO RAW_SAT_CNT 本组饱和像素数
//   0x050 RO RAW_TOT_CNT 本组总像素数
//   ---- vertical_diff ----
//   0x060 RW DIFF_CFG0   b0 bypass [31:16] wall_n
//   0x064 RO DIFF_STAT   b0 ovf_diff b1 proto_err b2 busy
//   ---- fast_alert ----
//   0x080 RW FA_ETHR_LO  E 阈值低 32 位
//   0x084 RW FA_ETHR_HI  E 阈值高 16 位（[15:0]）
//   0x088 RW FA_CFG0     [15:0] k [31:16] eps
//   0x08C RW FA_CFG1     [15:0] dir_deadband [16] persist_en
//   0x090 RW FA_MINE_LO  min_energy 低 32 位
//   0x094 RW FA_MINE_HI  min_energy 高 16 位
//   0x098 RW FA_COEF_PUSH [15:0] c_k [18:16] index [19] last
//                        写一次 = 向 fast_alert 推**一拍**系数
//   0x09C RW FA_BG_ADDR  [2:0] 背景表写地址（= k）
//   0x0A0 RW FA_BG_MU    [15:0] μ_k；写 FA_BG_MU 时按 FA_BG_ADDR 落表
//   0x0A4 RW FA_BG_VAR   [15:0] v_k；同上
//   0x0A8 RO FA_SCORE_LO alert_score 低 32 位
//   0x0AC RO FA_SCORE_HI alert_score 高 16 位
//   0x0B0 RO FA_CENTROID alert_centroid_x
//   0x0B4 RO FA_DELTA_X  alert_delta_x（低 17 位，补码）
//   0x0B8 RO FA_STAT     b0 frame_done 计数非 0？（见下）——实际用于计数快照位
//   ---- confidence_engine ----
//   0x0C0 RW CE_SNR      snr_q016
//   0x0C4 RW CE_MARGIN   margin_q016
//   0x0C8 RW CE_RHO      rho_q016
//   0x0CC RW CE_SAT      sat_ratio_q016
//   0x0D0 RW CE_SNR_MIN  snr_min_q016
//   0x0D4 RW CE_MAR_MIN  margin_min_q016
//   0x0D8 RW CE_RHO_MAX  rho_max_q016
//   0x0DC RW CE_SAT_MAX  sat_max_q016
//   0x0E0 RW CE_LOWCONF  low_conf_q016
//   0x0E4 RW CE_CFG      [1:0] w_sel [2] geom_hash_ok [3] sat_src_pl（默认 1）
//   0x0E8 RO CE_CONF     global_conf
//   0x0EC RO CE_STAT     b0 low_conf b1 force_gray [6:2] gray_reason[4:0]
//                        b7 done b8 busy b9 cfg_err
//                        [12:10] gray_reason[7:5]（数据完整性，2026-10-02 新增）
//   ---- confidence_engine 的 PL 内自动来源（2026-09-30 新增）----
//   0x0F0 RO CE_SAT_DIV  由 sat_pix_cnt/total_pix_cnt 算出的 Q0.16 比例（floor）
//   0x0F4 RO CE_SATSTAT  b0 div_busy b1 sat_src_pl b2 sat_over_thresh b3 ratio_valid
//                        （b2 = 实际送进 CE 的比例 > CE_SAT_MAX，即 0.1% 比较）
//   0x0F8 RW MASK_ADDR   [8:0] 掩膜表写地址 0..431
//   0x0FC RW MASK_DATA   [7:0] **写本寄存器即写 mask_mem[MASK_ADDR]**
//   ---- compositor ----
//   0x100 RW CMP_X       cand_best_x
//   0x104 RW CMP_Y       cand_best_y
//   0x108 RW CMP_Z       cand_best_z
//   0x10C RW CMP_SELID   selected_id
//   0x110 RW CMP_CONF    conf_q016
//   0x114 RW CMP_CTRL    bit0 low_conf
//   0x118 RW CMP_WALL_ADDR [WALL_AW-1:0]
//   0x11C RW CMP_WALL_DATA [23:0] **写本寄存器即产生一拍 wall_we**
//   0x120 RW CMP_REC_ADDR  [REC_AW-1:0]
//   0x124 RW CMP_REC_DATA  [23:0] **写本寄存器即产生一拍 rec_we**
//   0x128 RW CMP_OSD_ADDR  [OSD_AW-1:0]
//   0x12C RW CMP_OSD_DATA  [7:0]  **写本寄存器即产生一拍 osd_we**
//
//   复位默认值刻意等于上一版 xlconstant 的钉扎值（bd_full_path_20260930.md §6.4），
//   这样「PS 一条寄存器都没写」时链上行为与上一版逐位一致。
//
// ---------------------------------------------------------------------------
// 使用顺序（板端软件）
// ---------------------------------------------------------------------------
//   1) 写 CMP_*_ADDR，再写 CMP_*_DATA —— 后者产生的一拍写使能即把像素写进
//      compositor 的本地 BRAM（一个像素两次 32 位写）。
//   2) 推 c_k：每帧对 k=0..K-1 写 K 次 FA_COEF_PUSH（值在 [15:0]、k 在 [18:16]、
//      最后一拍把 bit19 置 1）。必须在 fast_alert 的判决启动前写完（它按
//      coef_tlast 或收满 K_MAX 启动）。
//   3) 背景表：写 FA_BG_ADDR = k，再分别写 FA_BG_MU / FA_BG_VAR。
//   4) 掩膜表：写 MASK_ADDR = n（0..431），再写 MASK_DATA = 掩膜字节（写它即落表）。
//      不写就是上电的 0xFF（= 旧 xlconstant 的值，掩膜全 1）。若掩膜来自模型包，
//      PS 需在 confidence_engine 每次评估前把 432 项灌完（本模块不做静默保护）。
//   5) 饱和比例：默认（CE_CFG bit3 = 1）由 PL 的 sat_ratio_div 自动提供，
//      PS 什么都不用做；想改用手写值：先清 CE_CFG bit3，再写 CE_SAT。
//      比例与阈值（CE_SAT_MAX）的比较在 confidence_engine 内完成，CE_SATSTAT bit2
//      是同一条比较的可观测副本。
//
// ---------------------------------------------------------------------------
// 刻意保留的行为
// ---------------------------------------------------------------------------
//   [R1] AXI-Lite 的 AWREADY/WREADY/ARREADY 是**组合**给出（值取决于 VALID），
//        AXI4-Lite 允许从端这样接；好处是每笔传输只花 1~2 拍、没有额外的状态机。
//   [R2] 写脉冲（wall/rec/osd_we、coef_tvalid）是**寄存一拍**产生的，所以脉冲有效
//        那一拍，地址/数据寄存器里已经是新值 —— 顺序天然正确，PS 不必再插延时。
//   [R3] μ/v 表是**同步读**（1 拍延迟），与 `fast_alert.sv:406-407` 的
//        「BRAM 1 拍延迟，故下一状态才取 μ/v」一致。
//   [R4] GEOM_HASH_OK 默认 1（与上一版钉扎一致，见 bd_full_path §6.4）。
//   [R5] 本模块**不**产生 confidence_engine 的 start（仍由 vertical_diff/frame_done
//        驱动，见块设计 §6.4）；PS 想手动触发需要另加一路 OR，本轮没做。
//   [R6] 掩膜表是一块 432x8 的**简单双口 RAM**（一个写口走 AXI-Lite，一个读口走
//        confidence_engine 的 mask_raddr）。上电内容 0xFF = 旧 xlconstant 的值，
//        且**不参与复位清零**：RAM 内容用 `initial` 给初值（Vivado 对推断出的
//        BRAM 支持这种方式），若在复位分支里清零会把它推成寄存器阵列而不是 RAM。
//        读口是**同步读（1 拍延迟）**，与 confidence_engine 的 ST_MRD -> ST_MWR
//        两级一致（见该模块 ST_MWR 的注释）。写口与读口地址无关，PS 别在 CE
//        评估期间改表（本模块不拦）。
//   [R7] 饱和比例默认由 PL 内的 `sat_ratio_div` 提供（CE_CFG bit3 = 1）。这一位
//        的复位默认值刻意**不等于**上一版的「PS 寄存器档」——G5 的收口就是让这条
//        自动来源真的接上；要退回旧行为，PS 把 bit3 清 0 即可。
//   [R8] `sat_ratio_div` 的内部策略（输入稳定即发布、清零后保持）见该文件头；
//        本模块只把它的输出与手写寄存器做二选一，不改它的语义。
//
// ---------------------------------------------------------------------------
// 仿真状态
// ---------------------------------------------------------------------------
//   配套 tb/pl_ctrl_regs_tb.sv 已在本机 Vivado xsim 与 Icarus Verilog 14.0 下各跑过
//   （读写回环、默认值、写脉冲计数、字节选通、掩膜 RAM、饱和比例二选一、
//    CE 三项标量写寄存器后输入脚跟随、geom_hash_ok 由 CE_CFG bit2 控制），
//   比对点 90，结论见 sim/results/g5_g3_raw_fix_20261001.md。
//   另有 tb/sat_ratio_div_tb.sv 单独验比例器本体（比对点 21，同一份结论）。
//   本文件**仍未上板**。
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module pl_ctrl_regs #(
    parameter integer ADDR_W  = 12,   // 4 KiB 寄存器页（本模块按 12 写死位宽，勿改）
    parameter integer WALL_AW = 14,   // compositor 墙图写地址位宽 = clog2(96*96)
    parameter integer REC_AW  = 9,    // 重建图写地址位宽 = clog2(24*18)
    parameter integer OSD_AW  = 8,    // OSD 文本写地址位宽 = clog2(32*8)
    parameter integer K_MAX   = 8     // fast_alert 背景表项数 = clog2 上界
) (
    input  wire                    clk,
    input  wire                    rst_n,

    // ---------------- AXI4-Lite 从 ----------------
    input  wire [ADDR_W-1:0]       s_axi_awaddr,
    input  wire                    s_axi_awvalid,
    output wire                    s_axi_awready,
    input  wire [31:0]             s_axi_wdata,
    input  wire [3:0]              s_axi_wstrb,
    input  wire                    s_axi_wvalid,
    output wire                    s_axi_wready,
    output wire [1:0]              s_axi_bresp,
    output wire                    s_axi_bvalid,
    input  wire                    s_axi_bready,
    input  wire [ADDR_W-1:0]       s_axi_araddr,
    input  wire                    s_axi_arvalid,
    output wire                    s_axi_arready,
    output wire [31:0]             s_axi_rdata,
    output wire [1:0]              s_axi_rresp,
    output wire                    s_axi_rvalid,
    input  wire                    s_axi_rready,

    // ---------------- raw_preproc 配置 ----------------
    output wire [15:0]             cfg_dark_level,
    output wire [3:0]              cfg_frames,
    output wire [15:0]             cfg_crop_side,
    output wire                    cfg_bayer_phase,

    // ---------------- vertical_diff 配置 ----------------
    output wire                    cfg_bypass,
    output wire [15:0]             cfg_wall_n,

    // ---------------- fast_alert 配置 ----------------
    output wire [47:0]             cfg_e_thresh,
    output wire [15:0]             cfg_k,
    output wire [15:0]             cfg_eps,
    output wire                    cfg_persist_en,
    output wire [15:0]             cfg_dir_deadband,
    output wire [47:0]             cfg_min_energy,

    // ---------------- fast_alert 系数流 + 背景表 ----------------
    output wire [15:0]             coef_tdata,
    output wire [2:0]              coef_tindex,
    output wire                    coef_tvalid,
    output wire                    coef_tlast,
    input  wire [2:0]              bg_raddr,
    output wire [15:0]             bg_mu,
    output wire [15:0]             bg_var,

    // ---------------- confidence_engine 输入 ----------------
    output reg                     ce_start, // PS triggers after current results are published
    output wire [15:0]             ce_snr_q016,
    output wire [15:0]             ce_margin_q016,
    output wire [15:0]             ce_rho_q016,
    output wire [15:0]             ce_sat_ratio_q016,
    output wire                    ce_geom_hash_ok,
    // 数据完整性标志：转发给 confidence_engine 的硬闸门（最高优先项）。
    // 来源与 RAW_STAT / DIFF_STAT 用的是同一组状态脚（见下方 assign）。
    output wire                    ce_ovf_raw,     // raw_preproc ovf_sticky
    output wire                    ce_ovf_diff,    // vertical_diff ovf_diff
    output wire                    ce_proto_err,   // raw_proto_err | diff_proto_err
    output wire [15:0]             ce_snr_min_q016,
    output wire [15:0]             ce_margin_min_q016,
    output wire [15:0]             ce_rho_max_q016,
    output wire [15:0]             ce_sat_max_q016,
    output wire [15:0]             ce_low_conf_q016,
    output wire [1:0]              ce_w_sel,

    // ---------------- confidence_engine 掩膜表读口（432x8 RAM）----------------
    //   ce_mask_raddr 来自 confidence_engine 的 mask_raddr（组合输出），
    //   ce_mask_rdata 是**同步读**结果（1 拍延迟）—— 与 CE 的 ST_MRD -> ST_MWR
    //   两级对齐，见文件头 [R6]。
    input  wire [8:0]              ce_mask_raddr,
    output wire [7:0]              ce_mask_rdata,

    // ---------------- compositor 配置 ----------------
    output wire [15:0]             cmp_cand_best_x,
    output wire [15:0]             cmp_cand_best_y,
    output wire [15:0]             cmp_cand_best_z,
    output wire [15:0]             cmp_selected_id,
    output wire [15:0]             cmp_conf_q016,
    output wire                    cmp_low_conf,

    // ---------------- compositor 本地写口（G4 桥）----------------
    output wire                    wall_we,
    output wire [WALL_AW-1:0]      wall_waddr,
    output wire [23:0]             wall_wdata,
    output wire                    rec_we,
    output wire [REC_AW-1:0]       rec_waddr,
    output wire [23:0]             rec_wdata,
    output wire                    osd_we,
    output wire [OSD_AW-1:0]       osd_waddr,
    output wire [7:0]              osd_wdata,

    // ---------------- 状态回读输入 ----------------
    input  wire                    raw_ovf_sticky,
    input  wire                    raw_proto_err,
    input  wire                    raw_cfg_err,
    input  wire                    raw_busy,
    input  wire [31:0]             raw_sat_pix_cnt,
    input  wire [31:0]             raw_total_pix_cnt,
    input  wire                    raw_frame_done,

    input  wire                    diff_ovf,
    input  wire                    diff_proto_err,
    input  wire                    diff_busy,
    input  wire                    diff_frame_done,

    input  wire                    fa_alert_event,
    input  wire [1:0]              fa_alert_direction,
    input  wire [47:0]             fa_alert_score,
    input  wire [15:0]             fa_alert_centroid_x,
    input  wire [16:0]             fa_alert_delta_x,
    input  wire                    fa_alert_centroid_valid,
    input  wire                    fa_frame_done,
    input  wire                    fa_proto_err,
    input  wire                    fa_zero_den,
    input  wire                    fa_e_ovf,
    input  wire                    fa_cfg_err,

    input  wire [15:0]             ce_global_conf,
    input  wire                    ce_low_conf,
    input  wire                    ce_force_gray,
    input  wire [7:0]              ce_gray_reason,
    input  wire                    ce_done,
    input  wire                    ce_busy,
    input  wire                    ce_cfg_err,

    input  wire                    cmp_frame_tick,
    input  wire                    cmp_busy
);

    // ------------------------------------------------------------------------
    // 寄存器字地址（= 字节偏移 >> 2）
    // ------------------------------------------------------------------------
    localparam [9:0] W_ID         = 10'h000;
    localparam [9:0] W_CE_START = 10'h02F; // 0x0BC: write bit0 pulse; read 1 = capability
    localparam [9:0] W_VERSION    = 10'h001;
    localparam [9:0] W_SCRATCH    = 10'h002;
    localparam [9:0] W_CTRL       = 10'h003;
    localparam [9:0] W_STAT_BUSY  = 10'h004;
    localparam [9:0] W_STAT_STICK = 10'h005;
    localparam [9:0] W_STAT_ALERT = 10'h006;
    localparam [9:0] W_CTR_RAW    = 10'h007;
    localparam [9:0] W_CTR_DIFF   = 10'h008;
    localparam [9:0] W_CTR_FA     = 10'h009;
    localparam [9:0] W_CTR_CE     = 10'h00A;
    localparam [9:0] W_CTR_DISP   = 10'h00B;

    localparam [9:0] W_RAW_CFG0   = 10'h010;
    localparam [9:0] W_RAW_CFG1   = 10'h011;
    localparam [9:0] W_RAW_STAT   = 10'h012;
    localparam [9:0] W_RAW_SAT    = 10'h013;
    localparam [9:0] W_RAW_TOT    = 10'h014;

    localparam [9:0] W_DIFF_CFG0  = 10'h018;
    localparam [9:0] W_DIFF_STAT  = 10'h019;

    localparam [9:0] W_FA_ETHR_LO = 10'h020;
    localparam [9:0] W_FA_ETHR_HI = 10'h021;
    localparam [9:0] W_FA_CFG0    = 10'h022;
    localparam [9:0] W_FA_CFG1    = 10'h023;
    localparam [9:0] W_FA_MINE_LO = 10'h024;
    localparam [9:0] W_FA_MINE_HI = 10'h025;
    localparam [9:0] W_FA_COEF    = 10'h026;
    localparam [9:0] W_FA_BG_ADDR = 10'h027;
    localparam [9:0] W_FA_BG_MU   = 10'h028;
    localparam [9:0] W_FA_BG_VAR  = 10'h029;
    localparam [9:0] W_FA_SCO_LO  = 10'h02A;
    localparam [9:0] W_FA_SCO_HI  = 10'h02B;
    localparam [9:0] W_FA_CENT    = 10'h02C;
    localparam [9:0] W_FA_DELTA   = 10'h02D;
    localparam [9:0] W_FA_STAT    = 10'h02E;

    localparam [9:0] W_CE_SNR     = 10'h030;
    localparam [9:0] W_CE_MARGIN  = 10'h031;
    localparam [9:0] W_CE_RHO     = 10'h032;
    localparam [9:0] W_CE_SAT     = 10'h033;
    localparam [9:0] W_CE_SNR_MIN = 10'h034;
    localparam [9:0] W_CE_MAR_MIN = 10'h035;
    localparam [9:0] W_CE_RHO_MAX = 10'h036;
    localparam [9:0] W_CE_SAT_MAX = 10'h037;
    localparam [9:0] W_CE_LOWCONF = 10'h038;
    localparam [9:0] W_CE_CFG     = 10'h039;
    localparam [9:0] W_CE_CONF    = 10'h03A;
    localparam [9:0] W_CE_STAT    = 10'h03B;
    localparam [9:0] W_CE_SATDIV  = 10'h03C;   // 0x0F0 RO
    localparam [9:0] W_CE_SATSTAT = 10'h03D;   // 0x0F4 RO
    localparam [9:0] W_MASK_ADDR  = 10'h03E;   // 0x0F8 RW
    localparam [9:0] W_MASK_DATA  = 10'h03F;   // 0x0FC RW（写它即落表）

    localparam [9:0] W_CMP_X      = 10'h040;
    localparam [9:0] W_CMP_Y      = 10'h041;
    localparam [9:0] W_CMP_Z      = 10'h042;
    localparam [9:0] W_CMP_SELID  = 10'h043;
    localparam [9:0] W_CMP_CONF   = 10'h044;
    localparam [9:0] W_CMP_CTRL   = 10'h045;
    localparam [9:0] W_CMP_W_ADDR = 10'h046;
    localparam [9:0] W_CMP_W_DATA = 10'h047;
    localparam [9:0] W_CMP_R_ADDR = 10'h048;
    localparam [9:0] W_CMP_R_DATA = 10'h049;
    localparam [9:0] W_CMP_O_ADDR = 10'h04A;
    localparam [9:0] W_CMP_O_DATA = 10'h04B;

    localparam [31:0] ID_VALUE    = 32'h5053_0001;   // "PS" + 版本 1
    localparam [31:0] VER_VALUE   = 32'd1;

    // ------------------------------------------------------------------------
    // AXI4-Lite：组合 ready + 寄存响应（每笔 1~2 拍，见 [R1]）
    // ------------------------------------------------------------------------
    reg               bvalid;
    reg  [1:0]        bresp;
    reg               aw_en;
    reg               rvalid;
    reg  [1:0]        rresp;
    reg  [ADDR_W-1:0] araddr_lat;

    wire wr_can = ~bvalid & aw_en;
    assign s_axi_awready = wr_can & s_axi_awvalid & s_axi_wvalid;
    assign s_axi_wready  = wr_can & s_axi_awvalid & s_axi_wvalid;
    assign s_axi_bvalid  = bvalid;
    assign s_axi_bresp   = bresp;

    wire wr_now = s_axi_awready & s_axi_wready;
    wire [9:0] widx = s_axi_awaddr[ADDR_W-1:2];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bvalid <= 1'b0;
            bresp  <= 2'b00;
            aw_en  <= 1'b1;
        end else begin
            if (wr_now) begin
                bvalid <= 1'b1;
                bresp  <= 2'b00;
                aw_en  <= 1'b0;
            end else if (bvalid & s_axi_bready) begin
                bvalid <= 1'b0;
                aw_en  <= 1'b1;
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) ce_start <= 1'b0;
        else ce_start <= wr_now && (widx == W_CE_START) && s_axi_wstrb[0] && s_axi_wdata[0];
    end
    wire rd_now = ~rvalid & s_axi_arvalid;
    assign s_axi_arready = ~rvalid;
    assign s_axi_rvalid  = rvalid;
    assign s_axi_rresp   = rresp;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rvalid     <= 1'b0;
            rresp      <= 2'b00;
            araddr_lat <= {ADDR_W{1'b0}};
        end else begin
            if (rd_now) begin
                rvalid     <= 1'b1;
                rresp      <= 2'b00;
                araddr_lat <= s_axi_araddr;
            end else if (rvalid & s_axi_rready) begin
                rvalid <= 1'b0;
            end
        end
    end

    // ------------------------------------------------------------------------
    // 写字节合并（AXI-Lite 的 WSTRB）
    // ------------------------------------------------------------------------
    function [31:0] wr_merge;
        input [31:0] old_v;
        input [31:0] new_v;
        input [3:0]  strb;
        begin
            wr_merge[7:0]   = strb[0] ? new_v[7:0]   : old_v[7:0];
            wr_merge[15:8]  = strb[1] ? new_v[15:8]  : old_v[15:8];
            wr_merge[23:16] = strb[2] ? new_v[23:16] : old_v[23:16];
            wr_merge[31:24] = strb[3] ? new_v[31:24] : old_v[31:24];
        end
    endfunction

    // ------------------------------------------------------------------------
    // RW 寄存器（复位默认值 = 上一版 xlconstant 的钉扎值）
    // ------------------------------------------------------------------------
    reg [31:0] scratch;
    reg [31:0] raw_cfg0, raw_cfg1;
    reg [31:0] diff_cfg0;
    reg [31:0] fa_ethr_lo, fa_ethr_hi;
    reg [31:0] fa_cfg0, fa_cfg1;
    reg [31:0] fa_mine_lo, fa_mine_hi;
    reg [31:0] fa_coef, fa_bg_addr;
    reg [31:0] fa_bg_mu, fa_bg_var;
    reg [31:0] ce_snr, ce_margin, ce_rho, ce_sat;
    reg [31:0] ce_snr_min, ce_mar_min, ce_rho_max, ce_sat_max, ce_lowconf;
    reg [31:0] ce_cfg;
    reg [31:0] mask_addr, mask_data;
    reg [31:0] cmp_x, cmp_y, cmp_z, cmp_selid, cmp_conf, cmp_ctrl;
    reg [31:0] cmp_w_addr, cmp_w_data;
    reg [31:0] cmp_r_addr, cmp_r_data;
    reg [31:0] cmp_o_addr, cmp_o_data;

    // 软件重载默认值（写 CTRL bit0 = 1，自清）
    reg        sw_reload;

    // 背景表（K_MAX 项，同步读 1 拍，见 [R3]）
    reg [15:0] mu_mem  [0:K_MAX-1];
    reg [15:0] var_mem [0:K_MAX-1];
    reg [15:0] bg_mu_r, bg_var_r;
    integer    mi;

    // 写脉冲（见 [R2]）
    reg        p_wall_we, p_rec_we, p_osd_we, p_coef_v;

    // 帧计数（可观测性）
    reg [31:0] ctr_raw, ctr_diff, ctr_fa, ctr_ce, ctr_disp;

    always @(posedge clk or negedge rst_n) begin
        if ((!rst_n) || sw_reload) begin
            raw_cfg0  <= 32'd4 << 16;    // dark_level=0, frames=4, bayer_phase=0
            raw_cfg1  <= 32'd1920;       // crop_side = 2*BLOCK*WALL_N
            diff_cfg0 <= 32'h0060_0000;  // bypass=0, wall_n=96（geometry_b.yaml wall.n）
            fa_ethr_lo<= 32'd60000;
            fa_ethr_hi<= 32'd0;
            fa_cfg0   <= 32'h0001_0008;  // k = 8, eps = 1
            fa_cfg1   <= 32'h0001_0020;  // dir_deadband = 32, persist_en = 1
            fa_mine_lo<= 32'd1;
            fa_mine_hi<= 32'd0;
            fa_coef   <= 32'd0;
            fa_bg_addr<= 32'd0;
            fa_bg_mu  <= 32'd0;
            fa_bg_var <= 32'd0;
            ce_snr    <= 32'd0;
            ce_margin <= 32'd0;
            ce_rho    <= 32'd0;
            ce_sat    <= 32'd0;
            ce_snr_min<= 32'd16384;
            ce_mar_min<= 32'd16384;
            ce_rho_max<= 32'd16384;
            ce_sat_max<= 32'd66;
            ce_lowconf<= 32'd4096;
            ce_cfg    <= 32'd12;         // geom_hash_ok=1, w_sel=0, sat_src_pl=1（G5 收口）
            mask_addr <= 32'd0;
            mask_data <= 32'd0;
            cmp_x     <= 32'd3;
            cmp_y     <= 32'd4;
            cmp_z     <= 32'd1;
            cmp_selid <= 32'd148;
            cmp_conf  <= 32'd16384;
            cmp_ctrl  <= 32'd0;
            cmp_w_addr<= 32'd0;
            cmp_w_data<= 32'd0;
            cmp_r_addr<= 32'd0;
            cmp_r_data<= 32'd0;
            cmp_o_addr<= 32'd0;
            cmp_o_data<= 32'd0;
        end else begin
            // 每拍至多一笔写（wr_now 只在一拍为真），故用 if/else-if 链
            if (wr_now && (widx == W_RAW_CFG0)) raw_cfg0 <= wr_merge(raw_cfg0, s_axi_wdata, s_axi_wstrb);
            else if (wr_now && (widx == W_RAW_CFG1)) raw_cfg1 <= wr_merge(raw_cfg1, s_axi_wdata, s_axi_wstrb);
            else if (wr_now && (widx == W_DIFF_CFG0)) diff_cfg0 <= wr_merge(diff_cfg0, s_axi_wdata, s_axi_wstrb);
            else if (wr_now && (widx == W_FA_ETHR_LO)) fa_ethr_lo <= wr_merge(fa_ethr_lo, s_axi_wdata, s_axi_wstrb);
            else if (wr_now && (widx == W_FA_ETHR_HI)) fa_ethr_hi <= wr_merge(fa_ethr_hi, s_axi_wdata, s_axi_wstrb);
            else if (wr_now && (widx == W_FA_CFG0)) fa_cfg0 <= wr_merge(fa_cfg0, s_axi_wdata, s_axi_wstrb);
            else if (wr_now && (widx == W_FA_CFG1)) fa_cfg1 <= wr_merge(fa_cfg1, s_axi_wdata, s_axi_wstrb);
            else if (wr_now && (widx == W_FA_MINE_LO)) fa_mine_lo <= wr_merge(fa_mine_lo, s_axi_wdata, s_axi_wstrb);
            else if (wr_now && (widx == W_FA_MINE_HI)) fa_mine_hi <= wr_merge(fa_mine_hi, s_axi_wdata, s_axi_wstrb);
            else if (wr_now && (widx == W_FA_COEF)) fa_coef <= wr_merge(fa_coef, s_axi_wdata, s_axi_wstrb);
            else if (wr_now && (widx == W_FA_BG_ADDR)) fa_bg_addr <= wr_merge(fa_bg_addr, s_axi_wdata, s_axi_wstrb);
            else if (wr_now && (widx == W_FA_BG_MU)) fa_bg_mu <= wr_merge(fa_bg_mu, s_axi_wdata, s_axi_wstrb);
            else if (wr_now && (widx == W_FA_BG_VAR)) fa_bg_var <= wr_merge(fa_bg_var, s_axi_wdata, s_axi_wstrb);
            else if (wr_now && (widx == W_CE_SNR)) ce_snr <= wr_merge(ce_snr, s_axi_wdata, s_axi_wstrb);
            else if (wr_now && (widx == W_CE_MARGIN)) ce_margin <= wr_merge(ce_margin, s_axi_wdata, s_axi_wstrb);
            else if (wr_now && (widx == W_CE_RHO)) ce_rho <= wr_merge(ce_rho, s_axi_wdata, s_axi_wstrb);
            else if (wr_now && (widx == W_CE_SAT)) ce_sat <= wr_merge(ce_sat, s_axi_wdata, s_axi_wstrb);
            else if (wr_now && (widx == W_CE_SNR_MIN)) ce_snr_min <= wr_merge(ce_snr_min, s_axi_wdata, s_axi_wstrb);
            else if (wr_now && (widx == W_CE_MAR_MIN)) ce_mar_min <= wr_merge(ce_mar_min, s_axi_wdata, s_axi_wstrb);
            else if (wr_now && (widx == W_CE_RHO_MAX)) ce_rho_max <= wr_merge(ce_rho_max, s_axi_wdata, s_axi_wstrb);
            else if (wr_now && (widx == W_CE_SAT_MAX)) ce_sat_max <= wr_merge(ce_sat_max, s_axi_wdata, s_axi_wstrb);
            else if (wr_now && (widx == W_CE_LOWCONF)) ce_lowconf <= wr_merge(ce_lowconf, s_axi_wdata, s_axi_wstrb);
            else if (wr_now && (widx == W_CE_CFG)) ce_cfg <= wr_merge(ce_cfg, s_axi_wdata, s_axi_wstrb);
            else if (wr_now && (widx == W_MASK_ADDR)) mask_addr <= wr_merge(mask_addr, s_axi_wdata, s_axi_wstrb);
            else if (wr_now && (widx == W_MASK_DATA)) mask_data <= wr_merge(mask_data, s_axi_wdata, s_axi_wstrb);
            else if (wr_now && (widx == W_CMP_X)) cmp_x <= wr_merge(cmp_x, s_axi_wdata, s_axi_wstrb);
            else if (wr_now && (widx == W_CMP_Y)) cmp_y <= wr_merge(cmp_y, s_axi_wdata, s_axi_wstrb);
            else if (wr_now && (widx == W_CMP_Z)) cmp_z <= wr_merge(cmp_z, s_axi_wdata, s_axi_wstrb);
            else if (wr_now && (widx == W_CMP_SELID)) cmp_selid <= wr_merge(cmp_selid, s_axi_wdata, s_axi_wstrb);
            else if (wr_now && (widx == W_CMP_CONF)) cmp_conf <= wr_merge(cmp_conf, s_axi_wdata, s_axi_wstrb);
            else if (wr_now && (widx == W_CMP_CTRL)) cmp_ctrl <= wr_merge(cmp_ctrl, s_axi_wdata, s_axi_wstrb);
            else if (wr_now && (widx == W_CMP_W_ADDR)) cmp_w_addr <= wr_merge(cmp_w_addr, s_axi_wdata, s_axi_wstrb);
            else if (wr_now && (widx == W_CMP_W_DATA)) cmp_w_data <= wr_merge(cmp_w_data, s_axi_wdata, s_axi_wstrb);
            else if (wr_now && (widx == W_CMP_R_ADDR)) cmp_r_addr <= wr_merge(cmp_r_addr, s_axi_wdata, s_axi_wstrb);
            else if (wr_now && (widx == W_CMP_R_DATA)) cmp_r_data <= wr_merge(cmp_r_data, s_axi_wdata, s_axi_wstrb);
            else if (wr_now && (widx == W_CMP_O_ADDR)) cmp_o_addr <= wr_merge(cmp_o_addr, s_axi_wdata, s_axi_wstrb);
            else if (wr_now && (widx == W_CMP_O_DATA)) cmp_o_data <= wr_merge(cmp_o_data, s_axi_wdata, s_axi_wstrb);
        end
    end

    // SCRATCH 单列出来（它不参与「重载默认值」）
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) scratch <= 32'd0;
        else if (wr_now && (widx == W_SCRATCH)) scratch <= wr_merge(scratch, s_axi_wdata, s_axi_wstrb);
    end

    // 写 CTRL bit0 -> 下一拍把全部 RW 寄存器重载为默认值（自清）
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) sw_reload <= 1'b0;
        else        sw_reload <= (wr_now && (widx == W_CTRL) && s_axi_wdata[0]);
    end

    // ------------------------------------------------------------------------
    // 写脉冲：写 DATA 寄存器的**下一拍**产生一拍写使能（见 [R2]）
    // ------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            p_wall_we <= 1'b0;
            p_rec_we  <= 1'b0;
            p_osd_we  <= 1'b0;
            p_coef_v  <= 1'b0;
        end else begin
            p_wall_we <= wr_now && (widx == W_CMP_W_DATA);
            p_rec_we  <= wr_now && (widx == W_CMP_R_DATA);
            p_osd_we  <= wr_now && (widx == W_CMP_O_DATA);
            p_coef_v  <= wr_now && (widx == W_FA_COEF);
        end
    end

    // ------------------------------------------------------------------------
    // 背景表 μ/v：写脉冲落表；读口同步 1 拍（与 fast_alert 的预期一致，见 [R3]）
    // ------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (mi = 0; mi < K_MAX; mi = mi + 1) begin
                mu_mem[mi]  <= 16'd0;
                var_mem[mi] <= 16'd0;
            end
        end else begin
            if (wr_now && (widx == W_FA_BG_MU))  mu_mem[fa_bg_addr[2:0]]  <= s_axi_wdata[15:0];
            if (wr_now && (widx == W_FA_BG_VAR)) var_mem[fa_bg_addr[2:0]] <= s_axi_wdata[15:0];
        end
    end

    always @(posedge clk) begin
        bg_mu_r  <= mu_mem[bg_raddr];
        bg_var_r <= var_mem[bg_raddr];
    end

    // ------------------------------------------------------------------------
    // 掩膜表：432 x 8 简单双口 RAM（写口 = MASK_ADDR / MASK_DATA，读口 = ce_mask_raddr）
    //   [R6]：上电内容用 initial 给 0xFF（= 旧 xlconstant 的值），复位**不清**内容
    //   —— 在复位分支里清零会把推断出的 RAM 变成寄存器阵列。
    //   ram_style=block：本意是落 BRAM；**本次只做 RTL/xsim，未综合验证**，
    //   综合器是否真的选 BRAM 要等下一次综合看利用率报告。
    // ------------------------------------------------------------------------
    localparam integer MASK_N = 432;
    (* ram_style = "block" *) reg [7:0] mask_mem [0:MASK_N-1];
    reg [7:0] mask_rdata_q;
    integer   mp;

    initial begin
        for (mp = 0; mp < MASK_N; mp = mp + 1) mask_mem[mp] = 8'hFF;
    end

    always @(posedge clk) begin
        if (wr_now && (widx == W_MASK_DATA)) mask_mem[mask_addr[8:0]] <= s_axi_wdata[7:0];
        mask_rdata_q <= mask_mem[ce_mask_raddr];
    end

    assign ce_mask_rdata = mask_rdata_q;

    // ------------------------------------------------------------------------
    // 饱和比例（G5 的 PL 内自动来源之一）
    //   sat_pix_cnt / total_pix_cnt -> Q0.16。策略/取整口径见 sat_ratio_div.v 文件头；
    //   本模块只做「PL 算的」与「PS 写的 CE_SAT」的二选一（选择位 = CE_CFG bit3）。
    // ------------------------------------------------------------------------
    wire        sat_ratio_valid;
    wire        sat_ratio_busy;
    wire [15:0] sat_ratio_pl;
    wire        sat_src_pl      = ce_cfg[3];
    wire [15:0] sat_ratio_used  = sat_src_pl ? sat_ratio_pl : ce_sat[15:0];
    wire        sat_over_thresh = (sat_ratio_used > ce_sat_max[15:0]);

    sat_ratio_div u_sat_ratio (
        .clk         (clk),
        .rst_n       (rst_n),
        .sat_cnt     (raw_sat_pix_cnt),
        .total_cnt   (raw_total_pix_cnt),
        .ratio_q016  (sat_ratio_pl),
        .ratio_valid (sat_ratio_valid),
        .busy        (sat_ratio_busy)
    );

    // ------------------------------------------------------------------------
    // 帧计数（可观测性；不做任何判决）
    // ------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ctr_raw  <= 32'd0;
            ctr_diff <= 32'd0;
            ctr_fa   <= 32'd0;
            ctr_ce   <= 32'd0;
            ctr_disp <= 32'd0;
        end else begin
            if (raw_frame_done)  ctr_raw  <= ctr_raw  + 32'd1;
            if (diff_frame_done) ctr_diff <= ctr_diff + 32'd1;
            if (fa_frame_done)   ctr_fa   <= ctr_fa   + 32'd1;
            if (ce_done)         ctr_ce   <= ctr_ce   + 32'd1;
            if (cmp_frame_tick)  ctr_disp <= ctr_disp + 32'd1;
        end
    end

    // ------------------------------------------------------------------------
    // 配置输出
    // ------------------------------------------------------------------------
    assign cfg_dark_level  = raw_cfg0[15:0];
    assign cfg_frames      = raw_cfg0[19:16];
    assign cfg_bayer_phase = raw_cfg0[20];
    assign cfg_crop_side   = raw_cfg1[15:0];

    assign cfg_bypass      = diff_cfg0[0];
    assign cfg_wall_n      = diff_cfg0[31:16];

    assign cfg_e_thresh    = {fa_ethr_hi[15:0], fa_ethr_lo[31:0]};
    assign cfg_k           = fa_cfg0[15:0];
    assign cfg_eps         = fa_cfg0[31:16];
    assign cfg_dir_deadband= fa_cfg1[15:0];
    assign cfg_persist_en  = fa_cfg1[16];
    assign cfg_min_energy  = {fa_mine_hi[15:0], fa_mine_lo[31:0]};

    assign coef_tdata      = fa_coef[15:0];
    assign coef_tindex     = fa_coef[18:16];
    assign coef_tlast      = fa_coef[19];
    assign coef_tvalid     = p_coef_v;
    assign bg_mu           = bg_mu_r;
    assign bg_var          = bg_var_r;

    assign ce_snr_q016       = ce_snr[15:0];
    assign ce_margin_q016    = ce_margin[15:0];
    assign ce_rho_q016       = ce_rho[15:0];
    // 饱和比例：默认走 PL 的 sat_ratio_div（CE_CFG bit3=1），bit3=0 时退回 PS 的 CE_SAT
    assign ce_sat_ratio_q016 = sat_ratio_used;
    assign ce_geom_hash_ok   = ce_cfg[2];
    assign ce_snr_min_q016   = ce_snr_min[15:0];
    assign ce_margin_min_q016= ce_mar_min[15:0];
    assign ce_rho_max_q016   = ce_rho_max[15:0];
    assign ce_sat_max_q016   = ce_sat_max[15:0];
    assign ce_low_conf_q016  = ce_lowconf[15:0];
    assign ce_w_sel          = ce_cfg[1:0];

    // 数据完整性标志 -> confidence_engine 硬闸门（kP0 A5，2026-10-02）。
    // 与 RAW_STAT / DIFF_STAT 用同一组状态脚：这三个标志既进回读寄存器，
    // 也进 force_gray 硬闸门（原来只进前者）。
    assign ce_ovf_raw   = raw_ovf_sticky;
    assign ce_ovf_diff  = diff_ovf;
    assign ce_proto_err = raw_proto_err | diff_proto_err;

    assign cmp_cand_best_x = cmp_x[15:0];
    assign cmp_cand_best_y = cmp_y[15:0];
    assign cmp_cand_best_z = cmp_z[15:0];
    assign cmp_selected_id = cmp_selid[15:0];
    assign cmp_conf_q016   = cmp_conf[15:0];
    assign cmp_low_conf    = cmp_ctrl[0];

    assign wall_we    = p_wall_we;
    assign wall_waddr = cmp_w_addr[WALL_AW-1:0];
    assign wall_wdata = cmp_w_data[23:0];
    assign rec_we     = p_rec_we;
    assign rec_waddr  = cmp_r_addr[REC_AW-1:0];
    assign rec_wdata  = cmp_r_data[23:0];
    assign osd_we     = p_osd_we;
    assign osd_waddr  = cmp_o_addr[OSD_AW-1:0];
    assign osd_wdata  = cmp_o_data[7:0];

    // ------------------------------------------------------------------------
    // 读回（组合译码，地址保存在 araddr_lat，RVALID 期间稳定）
    // ------------------------------------------------------------------------
    reg [31:0] rd_word;

    always @(*) begin
        case (araddr_lat[ADDR_W-1:2])
            W_ID:         rd_word = ID_VALUE;
            W_CE_START:   rd_word = 32'd1;
            W_VERSION:    rd_word = VER_VALUE;
            W_SCRATCH:    rd_word = scratch;
            W_CTRL:       rd_word = 32'd0;
            W_STAT_BUSY:  rd_word = {28'd0, cmp_busy, ce_busy, diff_busy, raw_busy};
            W_STAT_STICK: rd_word = {22'd0, ce_cfg_err, fa_cfg_err, fa_e_ovf,
                                     fa_zero_den, fa_proto_err,
                                     diff_proto_err, diff_ovf,
                                     raw_cfg_err, raw_proto_err, raw_ovf_sticky};
            // 既有位布局不变（[10:0]）；新增的完整性位 ce_gray_reason[7:5] 放 [13:11]。
            W_STAT_ALERT: rd_word = {18'd0, ce_gray_reason[7:5], ce_gray_reason[4:0],
                                     ce_force_gray, ce_low_conf,
                                     fa_alert_centroid_valid,
                                     fa_alert_direction, fa_alert_event};
            W_CTR_RAW:    rd_word = ctr_raw;
            W_CTR_DIFF:   rd_word = ctr_diff;
            W_CTR_FA:     rd_word = ctr_fa;
            W_CTR_CE:     rd_word = ctr_ce;
            W_CTR_DISP:   rd_word = ctr_disp;

            W_RAW_CFG0:   rd_word = raw_cfg0;
            W_RAW_CFG1:   rd_word = raw_cfg1;
            W_RAW_STAT:   rd_word = {28'd0, raw_busy, raw_cfg_err, raw_proto_err, raw_ovf_sticky};
            W_RAW_SAT:    rd_word = raw_sat_pix_cnt;
            W_RAW_TOT:    rd_word = raw_total_pix_cnt;

            W_DIFF_CFG0:  rd_word = diff_cfg0;
            W_DIFF_STAT:  rd_word = {29'd0, diff_busy, diff_proto_err, diff_ovf};

            W_FA_ETHR_LO: rd_word = fa_ethr_lo;
            W_FA_ETHR_HI: rd_word = fa_ethr_hi;
            W_FA_CFG0:    rd_word = fa_cfg0;
            W_FA_CFG1:    rd_word = fa_cfg1;
            W_FA_MINE_LO: rd_word = fa_mine_lo;
            W_FA_MINE_HI: rd_word = fa_mine_hi;
            W_FA_COEF:    rd_word = fa_coef;
            W_FA_BG_ADDR: rd_word = fa_bg_addr;
            W_FA_BG_MU:   rd_word = fa_bg_mu;
            W_FA_BG_VAR:  rd_word = fa_bg_var;
            W_FA_SCO_LO:  rd_word = fa_alert_score[31:0];
            W_FA_SCO_HI:  rd_word = {16'd0, fa_alert_score[47:32]};
            W_FA_CENT:    rd_word = {16'd0, fa_alert_centroid_x};
            W_FA_DELTA:   rd_word = {15'd0, fa_alert_delta_x};
            W_FA_STAT:    rd_word = {28'd0, fa_cfg_err, fa_e_ovf, fa_zero_den, fa_proto_err};

            W_CE_SNR:     rd_word = ce_snr;
            W_CE_MARGIN:  rd_word = ce_margin;
            W_CE_RHO:     rd_word = ce_rho;
            W_CE_SAT:     rd_word = ce_sat;
            W_CE_SNR_MIN: rd_word = ce_snr_min;
            W_CE_MAR_MIN: rd_word = ce_mar_min;
            W_CE_RHO_MAX: rd_word = ce_rho_max;
            W_CE_SAT_MAX: rd_word = ce_sat_max;
            W_CE_LOWCONF: rd_word = ce_lowconf;
            W_CE_CFG:     rd_word = ce_cfg;
            W_CE_CONF:    rd_word = {16'd0, ce_global_conf};
            // 既有位布局不变（[9:0]）；新增完整性位 ce_gray_reason[7:5] 放 [12:10]。
            W_CE_STAT:    rd_word = {19'd0, ce_gray_reason[7:5], ce_cfg_err, ce_busy,
                                     ce_done, ce_gray_reason[4:0],
                                     ce_force_gray, ce_low_conf};
            W_CE_SATDIV:  rd_word = {16'd0, sat_ratio_pl};
            W_CE_SATSTAT: rd_word = {28'd0, sat_ratio_valid, sat_over_thresh,
                                     sat_src_pl, sat_ratio_busy};
            W_MASK_ADDR:  rd_word = mask_addr;
            W_MASK_DATA:  rd_word = mask_data;

            W_CMP_X:      rd_word = cmp_x;
            W_CMP_Y:      rd_word = cmp_y;
            W_CMP_Z:      rd_word = cmp_z;
            W_CMP_SELID:  rd_word = cmp_selid;
            W_CMP_CONF:   rd_word = cmp_conf;
            W_CMP_CTRL:   rd_word = cmp_ctrl;
            W_CMP_W_ADDR: rd_word = cmp_w_addr;
            W_CMP_W_DATA: rd_word = cmp_w_data;
            W_CMP_R_ADDR: rd_word = cmp_r_addr;
            W_CMP_R_DATA: rd_word = cmp_r_data;
            W_CMP_O_ADDR: rd_word = cmp_o_addr;
            W_CMP_O_DATA: rd_word = cmp_o_data;

            default:      rd_word = 32'd0;
        endcase
    end

    assign s_axi_rdata = rd_word;

endmodule

`default_nettype wire
