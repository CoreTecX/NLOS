/* ============================================================================
 * PenumbraSight · 板端 PS 侧寄存器映射（**PL 控制页 / 采集-告警-显示链**）
 * board/ps_pl_regs.h
 *
 * 给 ARM(PS) 侧软件用：裸寄存器读写，不依赖 Xilinx 生成的 driver。
 * 基址来自块设计 penumbrasight_bd.tcl §7 的**新增段**（排在两个 AXI DMA 之后）：
 *
 *     PS_PL_BASE = 0xA00A0000     段大小 64 KiB（实际只用了低 0x100 字节）
 *
 * 这一页挂在块设计里的模块引用 `ps_regs`（源文件 src/fpga/rtl/pl_ctrl_regs.v）上，
 * 由它驱动五个 RTL 模块（raw_preproc / vertical_diff / fast_alert /
 * confidence_engine / compositor）的 cfg_* 与阈值，并回读它们的状态。
 *
 * ----------------------------------------------------------------------------
 * ⚠ 与 board/ps_fix_regs.h 不是一套，两张表**不能混用**
 * ----------------------------------------------------------------------------
 *   ps_fix_regs.h   —— 四个 HLS 定点核（投影 / 低秩前向 / 低秩伴随 / PDHG）的
 *                      s_axi_control 与 s_axi_control_r，基址 0xA0000000 起。
 *   ps_pl_regs.h    —— 本文件，PL 预处理/告警/显示链的配置与状态，基址 0xA00A0000。
 *   两边的偏移、字段、默认值都不同；混用表现为「写了没反应」或「读到别人的值」，
 *   不会报错。
 *
 * ----------------------------------------------------------------------------
 * 这个文件解决的三件事（缺口编号沿用 sim/results/bd_full_path_20260930.md §7）
 * ----------------------------------------------------------------------------
 *   G1  上一版把五个 RTL 模块的 cfg_* / 阈值用 xlconstant 钉死，PS 改不了。
 *       现在全部从这里写。**复位默认值 = 上一版钉扎值**，所以「一条都不写」
 *       与上一版行为相同（依据：pl_ctrl_regs.v 的复位分支）。
 *   G4  compositor 的 wall / rec / osd 是**本地写口**（写使能 + 地址 + 数据），
 *       不是 AXI 从口。这里给出「地址寄存器 + 数据寄存器」，写数据寄存器的
 *       下一拍由硬件产生**一拍**写使能 —— 一个像素写两次 32 位。
 *   G5  给了 fast_alert 的 c_k 系数流推送口与 μ/v 背景小表；四个可信度实时量
 *       都可写。**2026-09-30 再收两项**：饱和比例的 PL 内自动来源
 *       （sat_ratio_div，见 PS_PL_CE_SAT_DIV / PS_PL_CE_SATSTAT）、
 *       432 项掩膜表（PS_PL_MASK_ADDR / PS_PL_MASK_DATA）。
 *       margin / ρ 的 PL 内来源仍未闭合 —— 它们分别在投影核的 s_axilite 寄存器
 *       （core_proj，0xA0000000 + 0x070 margin_q16，vld 在 0x074）与 PDHG 的
 *       DDR 缓冲（core_pdhg，s_axi_control_r 0x0DC 是 out_rho_q16[] 的**基址**）
 *       里，要 PL 自动取回得给 PL 加 AXI 主口；见
 *       sim/results/g5_g3_raw_fix_20261001.md §任务一。
 *       本文件因此给 margin/ρ 各一个 PS 侧写入助手：PS 从核寄存器/DDR 读回后
 *       写进 PS_PL_CE_MARGIN / PS_PL_CE_RHO —— **这是 PS 侧桥接，不是 PL 内来源**。
 *       geom_hash_ok（CE_CFG bit2）的来源同样在 PS 软件：
 *       ps_psm_check_header() / ps_model_render_allowed()；PL 内不做几何哈希计算。
 *
 * ----------------------------------------------------------------------------
 * 使用顺序
 * ----------------------------------------------------------------------------
 *   1) 链路自检：读 PS_PL_ID 应为 0x50530001；写/读 PS_PL_SCRATCH 应一致。
 *   2) 配置：按模块分组写 RW 寄存器（见下面的表）。
 *   3) 背景表：写 PS_PL_FA_BG_ADDR = k，再分别写 PS_PL_FA_BG_MU / PS_PL_FA_BG_VAR。
 *   4) 推 c_k：每帧对 k = 0..K-1 写一次 PS_PL_FA_COEF_PUSH，
 *      值放 [15:0]、k 放 [18:16]，最后一拍把 bit19 置 1。必须在 fast_alert
 *      开始判决前写完（它按 coef_tlast 或收满 K_MAX 启动）。
 *   5) 灌图：写 PS_PL_CMP_WALL_ADDR -> 写 PS_PL_CMP_WALL_DATA（后者产生一拍写使能）；
 *      rec / osd 同法。
 *   6) 回读状态：PS_PL_STAT_BUSY / STAT_STICKY / STAT_ALERT / 各模块 STAT 与帧计数。
 *   7) 掩膜表：对 n = 0..431 调 ps_pl_mask_entry()。不写就是上电的 0xFF。
 *   8) 饱和比例：默认不用管（CE_CFG bit3 = 1，PL 自动算）。要改用手写值，先清
 *      CE_CFG bit3、再写 PS_PL_CE_SAT；PS_PL_CE_SATSTAT bit2 是那条 0.1% 比较的副本。
 *   9) margin / ρ / 几何哈希（**都要 PS 参与**，PL 内无来源）：
 *      margin = 读 core_proj 0xA0000070（vld 0xA0000074）-> ps_pl_ce_set_margin()；
 *      ρ      = 按 core_pdhg 的 out_rho_q16 基址读 DDR -> ps_pl_ce_set_rho()；
 *      几何    = ps_model_render_allowed() / ps_psm_check_header() -> ps_pl_set_geom_hash_ok()。
 *      这三项决定了 confidence_engine 的 margin/ρ/几何灰屏判据，不写就是复位默认（0/0/1）。
 *   10) 想恢复出厂默认：往 PS_PL_CTRL 写 1（自清，把全部 RW 寄存器重载为默认值；
 *      PS_PL_SCRATCH 刻意不参与重载；**掩膜 RAM 的内容也不参与**，它是 RAM 不是寄存器）。
 *
 * ----------------------------------------------------------------------------
 * 未验证声明
 * ----------------------------------------------------------------------------
 * 本文件**尚未在板上跑过**：寄存器叶本身已在 Vivado xsim 与 Icarus Verilog 14.0
 * 下逐值验过（src/fpga/rtl/tb/pl_ctrl_regs_tb.sv，比对点 90，
 * 见 sim/results/g5_g3_raw_fix_20261001.md），但「PS 通过 AXI 真跑到这段地址」
 * 属板端实测，本文件不代表已实测。
 * 掩膜 RAM 与饱和比例除法器另有独立的模块级门禁
 * （src/fpga/rtl/tb/sat_ratio_div_tb.sv），三个表项都还没上板。
 * ==========================================================================*/

#ifndef PENUMBRASIGHT_PS_PL_REGS_H
#define PENUMBRASIGHT_PS_PL_REGS_H

#include <stdint.h>

/* 块设计里 PL 控制页的基址（penumbrasight_bd.tcl §7 的新增段）。 */
#define PS_PL_BASE                 0xA00A0000u

/* ============================================================ 链路自检 / 状态 */
#define PS_PL_ID                   0x000u  /* RO 0x50530001（"PS" + 版本 1）      */
#define PS_PL_VERSION              0x004u  /* RO = 1                              */
#define PS_PL_SCRATCH              0x008u  /* RW 任意读写量（链路自检；不参与重载）*/
#define PS_PL_CTRL                 0x00Cu  /* RW bit0 = 1 写：重载全部 RW 默认值   */
#define PS_PL_STAT_BUSY            0x010u  /* RO b0 raw b1 diff b2 ce b3 cmp       */
#define PS_PL_STAT_STICKY          0x014u  /* RO b0 raw_ovf b1 raw_proto b2 raw_cfg
                                            *    b3 diff_ovf b4 diff_proto
                                            *    b5 fa_proto b6 fa_zero_den
                                            *    b7 fa_e_ovf b8 fa_cfg_err b9 ce_cfg_err */
#define PS_PL_STAT_ALERT           0x018u  /* RO b0 alert_event [2:1] direction
                                            *    b3 centroid_valid b4 low_conf
                                            *    b5 force_gray [10:6] gray_reason[4:0]
                                            *    [13:11] gray_reason[7:5]（2026-10-02 新增）
                                            *    b11 raw ovf_sticky b12 diff ovf
                                            *    b13 proto_err                       */
#define PS_PL_CTR_RAW              0x01Cu  /* RO raw_preproc frame_done 计数       */
#define PS_PL_CTR_DIFF             0x020u  /* RO vertical_diff frame_done 计数     */
#define PS_PL_CTR_FA               0x024u  /* RO fast_alert frame_done 计数        */
#define PS_PL_CTR_CE               0x028u  /* RO confidence_engine done 计数       */
#define PS_PL_CTR_DISP             0x02Cu  /* RO compositor frame_tick 计数        */

/* ============================================================ raw_preproc */
#define PS_PL_RAW_CFG0             0x040u  /* RW [15:0] dark_level [19:16] frames
                                            *    [20] bayer_phase
                                            *    默认 0x00040000（frames=4）       */
#define PS_PL_RAW_CFG1             0x044u  /* RW [15:0] crop_side，默认 1920       */
#define PS_PL_RAW_STAT             0x048u  /* RO b0 ovf_sticky b1 proto_err
                                            *    b2 cfg_err b3 busy               */
#define PS_PL_RAW_SAT_CNT          0x04Cu  /* RO 本组饱和像素数                    */
#define PS_PL_RAW_TOT_CNT          0x050u  /* RO 本组总像素数                      */

#define PS_PL_RAW_FRAMES_1         0x0001u
#define PS_PL_RAW_FRAMES_4         0x0004u
#define PS_PL_RAW_FRAMES_8         0x0008u
/* 写 CFG0 时请按位拼装（dark 在 [15:0]、frames 在 [19:16]、bayer_phase 在 [20]），
 * 直接用 PS_PL_RAW_CFG0_MAKE()。 */

/* ============================================================ vertical_diff */
#define PS_PL_DIFF_CFG0            0x060u  /* RW bit0 bypass [31:16] wall_n
                                            *    默认 0x00600000（wall_n=96）       */
#define PS_PL_DIFF_STAT            0x064u  /* RO b0 ovf_diff b1 proto_err b2 busy  */

/* ============================================================ fast_alert */
#define PS_PL_FA_ETHR_LO           0x080u  /* RW E 阈值低 32 位（默认 60000）      */
#define PS_PL_FA_ETHR_HI           0x084u  /* RW E 阈值高 16 位（[15:0]）          */
#define PS_PL_FA_CFG0              0x088u  /* RW [15:0] k [31:16] eps
                                            *    默认 0x00010008（k=8, eps=1）     */
#define PS_PL_FA_CFG1              0x08Cu  /* RW [15:0] dir_deadband [16] persist_en
                                            *    默认 0x00010020（32 / 1）         */
#define PS_PL_FA_MINE_LO           0x090u  /* RW min_energy 低 32 位（默认 1）     */
#define PS_PL_FA_MINE_HI           0x094u  /* RW min_energy 高 16 位               */
#define PS_PL_FA_COEF_PUSH         0x098u  /* RW [15:0] c_k [18:16] index [19] last
                                            *    **写一次 = 推一拍系数**          */
#define PS_PL_FA_BG_ADDR           0x09Cu  /* RW [2:0] 背景表写地址（= k）         */
#define PS_PL_FA_BG_MU             0x0A0u  /* RW [15:0] μ_k；写它即按 BG_ADDR 落表 */
#define PS_PL_FA_BG_VAR            0x0A4u  /* RW [15:0] v_k；同上                  */
#define PS_PL_FA_SCORE_LO          0x0A8u  /* RO alert_score 低 32 位              */
#define PS_PL_FA_SCORE_HI          0x0ACu  /* RO alert_score 高 16 位              */
#define PS_PL_FA_CENTROID          0x0B0u  /* RO alert_centroid_x                  */
#define PS_PL_FA_DELTA_X           0x0B4u  /* RO alert_delta_x（低 17 位，补码）   */
#define PS_PL_FA_STAT              0x0B8u  /* RO b0 proto_err b1 zero_den b2 e_ovf
                                            *    b3 cfg_err                        */

/* ============================================================ confidence_engine */
#define PS_PL_CE_START              0x0BCu  /* W bit0=start pulse, R=1 capability */
#define PS_PL_CE_SNR               0x0C0u  /* RW snr_q016（Q0.16）                 */
#define PS_PL_CE_MARGIN            0x0C4u  /* RW margin_q016                       */
#define PS_PL_CE_RHO               0x0C8u  /* RW rho_q016                          */
#define PS_PL_CE_SAT               0x0CCu  /* RW sat_ratio_q016                    */
#define PS_PL_CE_SNR_MIN           0x0D0u  /* RW 默认 16384（0x4000）              */
#define PS_PL_CE_MAR_MIN           0x0D4u  /* RW 默认 16384                        */
#define PS_PL_CE_RHO_MAX           0x0D8u  /* RW 默认 16384                        */
#define PS_PL_CE_SAT_MAX           0x0DCu  /* RW 默认 66（0x0042 = 0.1% 的 Q0.16） */
#define PS_PL_CE_LOWCONF           0x0E0u  /* RW 默认 4096（0x1000）               */
#define PS_PL_CE_CFG               0x0E4u  /* RW [1:0] w_sel [2] geom_hash_ok
                                            *    [3] sat_src_pl
                                            *    默认 12（geom_hash_ok=1, sat_src_pl=1）
                                            *    bit3=1：饱和比例取 PL 的 sat_ratio_div；
                                            *    bit3=0：退回 PS 写的 PS_PL_CE_SAT      */
#define PS_PL_CE_CONF              0x0E8u  /* RO global_conf（Q0.16）              */
#define PS_PL_CE_STAT              0x0ECu  /* RO b0 low_conf b1 force_gray
                                            *    [6:2] gray_reason[4:0] b7 done b8 busy
                                            *    b9 cfg_err
                                            *    [12:10] gray_reason[7:5]（2026-10-02 新增）
                                            *    灰屏原因位（位图，可同时置位）：
                                            *      [0]饱和 [1]SNR [2]margin [3]ρ
                                            *      [4]几何哈希
                                            *      [5]raw ovf_sticky [6]diff ovf
                                            *      [7]proto_err
                                            *    [7:5] 为数据完整性标志，优先级最高，
                                            *    直接进 PL 的 force_gray 硬闸门           */

/* --------------------------- confidence_engine 的 PL 内来源（2026-09-30） */
#define PS_PL_CE_SAT_DIV           0x0F0u  /* RO 由 sat_pix_cnt/total_pix_cnt 算出的
                                            *    Q0.16 比例（整数 floor；sat==total 饱和
                                            *    到 0xFFFF）。清零后保持上次值。       */
#define PS_PL_CE_SATSTAT           0x0F4u  /* RO b0 div_busy b1 sat_src_pl
                                            *    b2 sat_over_thresh b3 ratio_valid
                                            *    b2 = 实际送进 CE 的比例 > CE_SAT_MAX
                                            *    （0.1%），是 CE 内部那条比较的副本   */
#define PS_PL_MASK_ADDR            0x0F8u  /* RW [8:0] 掩膜表写地址 0..431          */
#define PS_PL_MASK_DATA            0x0FCu  /* RW [7:0] **写本寄存器即写 mask[MASK_ADDR]**
                                            *    上电内容 0xFF（= 旧 xlconstant 的值）；
                                            *    复位不清内容（RAM，不是寄存器）      */

/* ============================================================ compositor */
#define PS_PL_CMP_X                 0x100u  /* RW cand_best_x，默认 3               */
#define PS_PL_CMP_Y                 0x104u  /* RW cand_best_y，默认 4               */
#define PS_PL_CMP_Z                 0x108u  /* RW cand_best_z，默认 1               */
#define PS_PL_CMP_SELID             0x10Cu  /* RW selected_id，默认 148             */
#define PS_PL_CMP_CONF              0x110u  /* RW conf_q016，默认 16384（0x4000）   */
#define PS_PL_CMP_CTRL              0x114u  /* RW bit0 low_conf                     */
#define PS_PL_CMP_WALL_ADDR         0x118u  /* RW [13:0] 墙图写地址（行*96+列）     */
#define PS_PL_CMP_WALL_DATA         0x11Cu  /* RW [23:0] RGB888；**写它 = 一拍 wall_we** */
#define PS_PL_CMP_REC_ADDR          0x120u  /* RW [8:0]  重建图写地址（行*24+列）   */
#define PS_PL_CMP_REC_DATA          0x124u  /* RW [23:0] RGB888；**写它 = 一拍 rec_we**  */
#define PS_PL_CMP_OSD_ADDR          0x128u  /* RW [7:0]  OSD 写地址（行*32+列）     */
#define PS_PL_CMP_OSD_DATA          0x12Cu  /* RW [7:0]  ASCII；**写它 = 一拍 osd_we**   */

/* ------------------------------------------------------------ 常用拼装助手 */
#define PS_PL_RAW_CFG0_MAKE(dark, frames, bayer) \
    (((uint32_t)(dark) & 0xFFFFu) | (((uint32_t)(frames) & 0xFu) << 16) | \
     (((uint32_t)(bayer) & 0x1u) << 20))
#define PS_PL_DIFF_CFG0_MAKE(bypass, wall_n) \
    (((uint32_t)(bypass) & 0x1u) | (((uint32_t)(wall_n) & 0xFFFFu) << 16))
#define PS_PL_FA_CFG0_MAKE(k, eps)   (((uint32_t)(k) & 0xFFFFu) | (((uint32_t)(eps) & 0xFFFFu) << 16))
#define PS_PL_FA_CFG1_MAKE(dbk, pe)  (((uint32_t)(dbk) & 0xFFFFu) | (((uint32_t)(pe) & 0x1u) << 16))
#define PS_PL_FA_COEF_MAKE(c, idx, last) \
    (((uint32_t)(c) & 0xFFFFu) | (((uint32_t)(idx) & 0x7u) << 16) | \
     (((uint32_t)(last) & 0x1u) << 19))

/* ------------------------------------------------------------ 访问助手 */
static inline void ps_pl_write32(volatile uint32_t *base, uint32_t off, uint32_t v)
{
    base[off >> 2] = v;
}

static inline uint32_t ps_pl_read32(volatile uint32_t *base, uint32_t off)
{
    return base[off >> 2];
}

/* 往 compositor 的本地写口灌一个像素：先写地址，再写数据（数据写产生一拍写使能）。 */
static inline void ps_pl_wall_pixel(volatile uint32_t *base, uint32_t idx, uint32_t rgb888)
{
    ps_pl_write32(base, PS_PL_CMP_WALL_ADDR, idx);
    ps_pl_write32(base, PS_PL_CMP_WALL_DATA, rgb888 & 0xFFFFFFu);
}

static inline void ps_pl_rec_pixel(volatile uint32_t *base, uint32_t idx, uint32_t rgb888)
{
    ps_pl_write32(base, PS_PL_CMP_REC_ADDR, idx);
    ps_pl_write32(base, PS_PL_CMP_REC_DATA, rgb888 & 0xFFFFFFu);
}

static inline void ps_pl_osd_char(volatile uint32_t *base, uint32_t idx, uint32_t ascii)
{
    ps_pl_write32(base, PS_PL_CMP_OSD_ADDR, idx);
    ps_pl_write32(base, PS_PL_CMP_OSD_DATA, ascii & 0xFFu);
}

/* 写背景表一项：地址 -> μ -> v。 */
static inline void ps_pl_bg_entry(volatile uint32_t *base, uint32_t k,
                                  uint32_t mu_q216, uint32_t var_q216)
{
    ps_pl_write32(base, PS_PL_FA_BG_ADDR, k & 0x7u);
    ps_pl_write32(base, PS_PL_FA_BG_MU,  mu_q216  & 0xFFFFu);
    ps_pl_write32(base, PS_PL_FA_BG_VAR, var_q216 & 0xFFFFu);
}

/* 写掩膜表一项（0..431）：地址 -> 数据。不写就是上电的 0xFF。
 * 掩膜来自模型包时，须在 confidence_engine 每次评估前灌完 432 项 —— PL 不做保护。 */
static inline void ps_pl_mask_entry(volatile uint32_t *base, uint32_t idx, uint32_t mask_byte)
{
    ps_pl_write32(base, PS_PL_MASK_ADDR, idx & 0x1FFu);
    ps_pl_write32(base, PS_PL_MASK_DATA, mask_byte & 0xFFu);
}

/* ---------------------------------------------------------------- CE_CFG 位域 */
#define PS_PL_CE_CFG_W_SEL_MASK   0x3u   /* [1:0] 融合方式                       */
#define PS_PL_CE_CFG_GEOM_BIT     2      /* [2]   geom_hash_ok（1=几何一致）     */
#define PS_PL_CE_CFG_SAT_PL_BIT   3      /* [3]   饱和比例来源（1=PL 自动/默认） */

/* 设置 geom_hash_ok（CE_CFG bit2），保持其余位不变。
 * 来源：ps_psm_check_header() 或 ps_model_render_allowed()（几何不一致时传 0）。
 * 读-改-写要在 confidence_engine 评估的间隙做。 */
static inline void ps_pl_set_geom_hash_ok(volatile uint32_t *base, uint32_t ok)
{
    uint32_t v = ps_pl_read32(base, PS_PL_CE_CFG);
    if (ok) v |= (1u << PS_PL_CE_CFG_GEOM_BIT);
    else    v &= ~(1u << PS_PL_CE_CFG_GEOM_BIT);
    ps_pl_write32(base, PS_PL_CE_CFG, v);
}

/* ---------------------------------------------------------------- margin / rho 的 PS 侧桥
 *
 * 说明（务必看清）：confidence_engine 的 margin / ρ **在 PL 内没有来源**。
 *   margin：core_proj（基址 0xA0000000）的 s_axi_control 寄存器 0x070（margin_q16，
 *           只读；valid 脉冲在 0x074，读清）。PS 读回后调 ps_pl_ce_set_margin()。
 *   ρ     ：core_pdhg（基址 0xA0060000）的 s_axi_control_r 寄存器 0x0DC 是
 *           out_rho_q16[] 的**DDR 基址**（uint32_t[3]，按通道）。PS 按基址读 DDR，
 *           取出通道值后调 ps_pl_ce_set_rho()。RTL 侧不参与。
 * 因此这两个助手是 **PS 侧桥接**，不是「PL 内自动来源」；要改成 PL 自动，需给 PL
 * 加一条 AXI4-Lite 主口（读 core_proj 寄存器）与一条能读 DDR 的主口（读 rho），
 * 属块设计级改动，见 sim/results/g5_g3_raw_fix_20261001.md §任务一。
 */
static inline void ps_pl_ce_set_margin(volatile uint32_t *base, uint32_t margin_q016)
{
    ps_pl_write32(base, PS_PL_CE_MARGIN, margin_q016 & 0xFFFFu);
}

static inline void ps_pl_ce_set_rho(volatile uint32_t *base, uint32_t rho_q016)
{
    ps_pl_write32(base, PS_PL_CE_RHO, rho_q016 & 0xFFFFu);
}

#endif /* PENUMBRASIGHT_PS_PL_REGS_H */
