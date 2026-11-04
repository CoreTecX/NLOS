// ============================================================================
// confidence_engine.sv  ·  PenumbraSight 可信度引擎（方案 §6.2 / §6.6 / §7 / §8.2）
// 依据：plan_teacher.md §6.6（全局可信度四项 + 五种强制灰屏条件）
//         「全局可信度由四项组成：输入 SNR、候选 margin、归一化残差 ρ、饱和比例。
//           像素可信度为预计算可观测性掩膜与全局可信度的乘积。
//           以下情况强制灰屏并提示原因：
//             饱和像素比例 > 0.1% / 输入 SNR 低于标定阈值 /
//             定位 margin 低于标定阈值 / ρ 超空白验证集 95 百分位的 1.5 倍 /
//             模型几何哈希与当前配置不一致」
//       §6.2（输入 SNR、margin、residual、mask；输出 global confidence、像素图）
//       §8.2（0x1C CONFIDENCE = Q0.16；STATUS bit3 low_conf；0x24 ERROR_CODE）
//       §9.3 必做对照 8（有可信度门控 / 无可信度门控）
//
// ---------------------------------------------------------------------------
// 设计意图
// ---------------------------------------------------------------------------
// 【意图 1】为什么可信度要「软分数」和「硬闸门」两套并存
//   方案 §6.6 的两句话其实是两种运算：
//     * 「全局可信度由四项组成」→ 是**连续性**的量，用于显示与排序，
//       所以做成 Q0.16 的软分数（0.0~1.0），可以加权/相乘/取最小；
//     * 「以下情况强制灰屏」→ 是**布尔**判据，用于安全门控，只有 0/1。
//   把两者混成一套（例如只看软分数是否小于阈值）会出问题：margin 恰好等于阈值
//   时软分数是 1.0 而硬判据要求灰屏，两种实现的边界行为不一致，验收时会出现
//   「同一份录像两台机器结论不同」。故本模块刻意把两套都算出来，软分数只进显示，
//   硬判据只进 force_gray，**互不代替**。
//
// 【意图 2】灰屏原因必须是位图而不是编码
//   多个条件可能同时成立（例如低 SNR 往往伴随大残差）。若用优先级编码只报第一个，
//   现场调试会丢掉信息——你看到「SNR 不足」，其实是几何哈希不匹配导致的连锁。
//   故 gray_reason 是 8 bit 位图（bit[4:0] 为原有五项质量指标，bit[7:5] 为
//   数据完整性标志），全列出来。
//
// 【意图 2b】数据完整性标志（ovf/proto_err）必须进硬闸门而不是只读寄存器
//   raw_preproc 的 ovf_sticky（满量程打包下「亮区被削平」）、vertical_diff 的
//   ovf_diff、两者的 proto_err 原本只写进 RO 状态寄存器。只读寄存器对「当帧出图」
//   没有任何约束：数据已经坏了，图照样往外送。故把这三条接进 force_gray，
//   且优先级高于原有五项质量指标（质量差可显示低可信度，数据坏只能灰屏）。
//
// 【意图 3】阈值全部来自标定集，本模块不做任何统计
//   §6.6 明确「阈值只用调参集设定，盲测集不重新调阈值」。这要求阈值是**常量输入**
//   而不是模块内的自适应逻辑。因此本模块没有任何「历史统计」状态：
//   相同输入 → 相同输出，可复现、可审计。这也是 §9.3 对照 8 能成立的前提。
//
// 【意图 4】ρ 的阈值是「空白验证集 95 百分位的 1.5 倍」，这个 1.5 倍在 PC 端算好
//   模块只接收 rho_max。**不要把 1.5 写进 RTL** —— 空白集的百分位换了实验台就要重算，
//   写死在硬件里会让「改标定必须重新综合」。
//
// ---------------------------------------------------------------------------
// 易错点
// ---------------------------------------------------------------------------
// (1) 四个量必须同一尺度（Q0.16）
//     SNR 可能是 dB、可能是线性比值；margin = (s1-s2)/(s1+ε) 天然是 0..1；
//     ρ 是归一化残差（无量纲）；饱和比例是 0..1。若上游把 SNR 按 dB 送进来，
//     这里的除法与阈值全部失效，而且**不会报错**——只会得到一个永远偏保守的
//     confidence 和偶尔的假灰屏。故本模块要求上游统一换算成 Q0.16 的无量纲可比量，
//     换算口径写在 cfg 里（见 ARM 侧 x12_ps_contract）。
//
// (2) 除零
//     四个阈值任何一个为 0 都会让除法爆表。本模块对 den==0 显式置 1 并顶 cfg_err，
//     绝不静默。
//
// (3) 掩膜是「预计算可观测性掩膜」，不是实时量
//     ps_psm_format.h 里 observable_mask 是 uint8[N]（N = hidden_h*hidden_w = 432），
//     随模型包一起装载。它描述的是「这个隐藏像素在当前几何下是否可观测」，
//     与当帧数据无关。把它和实时量混在一起会导致「换场景后掩膜失效却不自知」。
//
// (4) 432 个像素的可信度图是**8 bit**，不是 Q0.16
//     显示与后续 OSD 只需要 8 bit 的相对亮度；但**乘积要在 16 bit 上做再截断**，
//     若先把 global_conf 截成 8 bit 再乘掩膜，会引入一次可见的量化台阶。
//
// (5) 几何哈希不匹配的灰屏优先级最高
//     哈希不匹配意味着模型与几何配置对不上，此时任何图像都是错的。
//     本条来自 §6.6 最后一条，也是 geometry_b.yaml 里 geometry_hash_scope 那张
//     清单的直接后果：改了 planes/wall/hidden/occluder/forward_model 任一项，
//     哈希即变，必须灰屏提示，不许出图。
//
// ---------------------------------------------------------------------------
// 接口口径（端口一览；默认位宽见下方 parameter 列表）
// ---------------------------------------------------------------------------
//   输入
//     clk                   1      时钟
//     rst_n                 1      低有效异步复位
//     start                 1      一帧一次启动（自清，保持到 busy 拉起即可）
//     snr_q016        [15:0]        输入 SNR（已归一化为 Q0.16 可比量，见易错点 1）
//     margin_q016     [15:0]        定位 margin = (s1-s2)/(s1+ε)，Q0.16
//     rho_q016        [15:0]        归一化残差 ρ，Q0.16
//     sat_ratio_q016  [15:0]        饱和像素比例（0..1），Q0.16
//     geom_hash_ok          1      模型几何哈希 == 当前配置（见易错点 5）
//     snr_min_q016    [15:0]        SNR 标定阈值
//     margin_min_q016 [15:0]        margin 标定阈值
//     rho_max_q016    [15:0]        = 1.5 × 空白验证集 95 百分位（PC 端算好）
//     sat_max_q016    [15:0]        0.1% -> Q0.16 = 66 = 0x0042（round(0.001*65536)）
//     low_conf_q016   [15:0]        low_conf 判定阈值（STATUS bit3）
//     w_sel            [1:0]        融合：0=取最小 1=连乘 2=等权平均
//     mask_rdata       [7:0]        可观测性掩膜（随模型包装载，1 拍延迟回读）
//   输出
//     mask_raddr       [8:0]        掩膜读地址 0..431
//     global_conf     [15:0]        全局可信度 Q0.16 -> AXI-Lite 0x1C
//     low_conf              1       -> STATUS bit3
//     force_gray            1       强制灰屏（compositor 消费）
//     gray_reason      [7:0]        灰屏原因位图（bit0 饱和 / bit1 SNR / bit2 margin /
//                                   bit3 ρ / bit4 几何哈希 / bit5 raw ovf_sticky /
//                                   bit6 diff ovf / bit7 proto_err；bit[7:5] 优先）
//     conf_map_waddr   [9:0]        像素可信度写地址 0..431
//     conf_map_wdata   [7:0]        像素可信度（8 bit，见易错点 4）
//     conf_map_we           1       写使能
//     done                  1       一次评估完成（1 拍脉冲）
//     busy                  1       非 IDLE
//     cfg_err               1       配置非法（阈值或 ε 为 0 等）
//   对应文档：plan_teacher.md §6.6（四项可信度 + 五种强制灰屏条件）/
//             §6.2（confidence_engine 行：输入 SNR/margin/residual/mask；
//                   输出 global confidence / 像素图）/
//             §8.2（0x1C CONFIDENCE = Q0.16；STATUS bit3 low_conf）
//             src/software/include/psm_format.h（observable_mask: uint8[432]）
//
// ---------------------------------------------------------------------------
// 刻意保留的行为（不是 bug；改动前先确认是不是有意为之）
// ---------------------------------------------------------------------------
//   [R1] 软分数（连续量，供显示/排序）与硬闸门（布尔，供安全门控）**并存且互不代替**。
//        只保留一种会在阈值边界上产生「两台机器结论不同」的模糊地带（意图 1）。
//   [R2] gray_reason 是 8 bit **位图**而不是优先级编码：多个条件可同时成立，现场调试
//        需要看到全部原因（意图 2）。bit[4:0] 为原有五项，bit[7:5] 为数据完整性标志。
//   [R3] 模块内**没有任何历史统计**：阈值全是常量输入，相同输入 -> 相同输出。
//        §6.6 要求「盲测集不重新调阈值」，这也是 §9.3 对照 8 能成立的前提。
//   [R4] 「1.5 倍」刻意**不写进 RTL**，由 PC 端算好 rho_max 传入；写死会让「改标定
//        必须重新综合」。
//   [R5] 软分数采用「2 倍阈值 = 1.0」的口径（而不是「阈值 = 1.0」）。若按后者，硬闸门
//        一通过软分数就饱和到 1.0，SNR/margin 两项在融合里失去区分度（见 ST_SETUP
//        上方的注释）。这是**刻意**的口径选择，需与 Python 参考模型一致。
//   [R6] 像素可信度乘积在 16 bit 上做、最后才截成 8 bit（避免先截后乘的可见台阶）。
//   [R7] 灰屏时 global_conf 直接归零（不是只置 force_gray）：下游若漏用 force_gray，
//        可信度本身也已失效，构成第二道保险。
//   [R8] 除法复用与 fast_alert 同一套恢复余数结构（floor），便于两模块一起审计。
//
// ---------------------------------------------------------------------------
// 仿真状态声明
// ---------------------------------------------------------------------------
//   本文件**未在任何仿真器上运行过**（本机无 iverilog / Vivado / Verilator）。
//   文件内不含仿真结果。配套 tb/confidence_engine_tb.sv 是待运行的自检平台：
//   五项灰屏条件与灰屏原因位图是**逐值精确**比对，软分数按 Q0.16 整数整除比对。
// ============================================================================

// 与 tb/confidence_engine_tb.sv 取同一值：RTL 缺 timescale 时 xsim 会因
// [XSIM 43-4099] 拒绝 elaboration（见 sim/results/xsim_first_run_20260929.md §6.1）。
`timescale 1ns/1ps
`default_nettype none

module confidence_engine #(
    parameter int CONF_W  = 16,     // Q0.16（§8.2 的 0x1C CONFIDENCE）
    parameter int MASK_W  = 8,      // 掩膜/像素可信度位宽
    parameter int HID_N   = 432,    // 隐藏像素数（24x18 = psm_format.h 的 PS_PSM_HIDDEN_* 乘积）
    parameter int DIV_NW  = 48,     // 除法位宽（与 fast_alert 同构，便于代码审计）
    localparam int MAW  = $clog2(HID_N),      // 掩膜地址位宽
    localparam int DIVCW= $clog2(DIV_NW+1)
) (
    input  wire logic                clk,
    input  wire logic                rst_n,
    input  wire logic                start,            // 一帧一次

    // ---------------- 四项实时量（全部 Q0.16，见易错点 1）----------------
    input  wire logic [CONF_W-1:0]   snr_q016,         // 输入 SNR（已归一化为 0..1 可比量）
    input  wire logic [CONF_W-1:0]   margin_q016,      // 定位 margin = (s1-s2)/(s1+ε)
    input  wire logic [CONF_W-1:0]   rho_q016,         // 归一化残差 ρ
    input  wire logic [CONF_W-1:0]   sat_ratio_q016,   // 饱和像素比例（0..1）
    input  wire logic                geom_hash_ok,     // 模型几何哈希 == 当前配置（见易错点 5）

    // ---------------- 数据完整性标志（来自 raw_preproc / vertical_diff）----------------
    // 这三条是**硬闸门里优先级最高**的一组：它们是「数据本身已经不可信」的标志
    // （满量程打包削平 / 差分饱和 / AXIS 协议错），与 SNR / margin / ρ / sat / geom
    // 这些「质量指标」不同 —— 质量差可以显示低可信度，数据完整性坏了只能灰屏。
    // 出处：raw_preproc.sv:220-221,523-531 的 ovf_sticky/proto_err、
    //       vertical_diff.sv:337-355 的 ovf_diff/proto_err。
    input  wire logic                raw_ovf_sticky,   // raw_preproc 累积饱和（亮区被削平）
    input  wire logic                ovf_diff,         // vertical_diff 差分饱和
    input  wire logic                proto_err,        // 上游 AXIS 协议错（raw 或 diff）

    // ---------------- 标定阈值（Q0.16，PC 端算好）----------------
    input  wire logic [CONF_W-1:0]   snr_min_q016,
    input  wire logic [CONF_W-1:0]   margin_min_q016,
    input  wire logic [CONF_W-1:0]   rho_max_q016,     // = 1.5 x 空白验证集 95 百分位
    input  wire logic [CONF_W-1:0]   sat_max_q016,     // 0.1% -> Q0.16 = 66 = 0x0042（不是 0x0066）
    input  wire logic [CONF_W-1:0]   low_conf_q016,    // low_conf 判定阈值（STATUS bit3）
    input  wire logic [1:0]          w_sel,            // 融合方式：0=取最小 1=连乘 2=等权平均

    // ---------------- 可观测性掩膜读接口（模型包内 observable_mask）----------------
    output logic [MAW-1:0]      mask_raddr,
    input  wire logic [MASK_W-1:0]   mask_rdata,

    // ---------------- 输出 ----------------
    output logic [CONF_W-1:0]   global_conf,      // Q0.16 -> AXI-Lite 0x1C
    output logic                low_conf,         // -> STATUS bit3
    output logic                force_gray,       // 强制灰屏（compositor 消费）
    output logic [7:0]          gray_reason,      // 位图：见下（bit[7:5] 为数据完整性）
    output logic [MAW:0]        conf_map_waddr,   // 像素可信度写地址 0..HID_N-1
    output logic [MASK_W-1:0]   conf_map_wdata,
    output logic                conf_map_we,
    output logic                done,
    output logic                busy,
    output logic                cfg_err
);

    // 灰屏原因位图（意图 2：全列出来，不做优先级编码）
    localparam int GR_SAT     = 0;   // 饱和像素比例 > 0.1%
    localparam int GR_SNR     = 1;   // 输入 SNR 低于阈值
    localparam int GR_MARGIN  = 2;   // 定位 margin 低于阈值
    localparam int GR_RHO     = 3;   // ρ 超空白集 95 百分位 x 1.5
    localparam int GR_GEOM    = 4;   // 几何哈希与配置不一致
    // 数据完整性标志：优先级高于上面五项 —— 一旦置位，任何图像都不可信，
    // 与「质量差」不同，不能靠软分数表达（出处见端口处注释）。
    localparam int GR_OVF_RAW  = 5;  // raw_preproc 累积饱和（ovf_sticky）
    localparam int GR_OVF_DIFF = 6;  // vertical_diff 差分饱和（ovf_diff）
    localparam int GR_PROTO    = 7;  // 上游 AXIS 协议错（proto_err）

    // ------------------------------------------------------------------------
    // FSM
    // ------------------------------------------------------------------------
    localparam logic [2:0] ST_IDLE  = 3'd0;
    localparam logic [2:0] ST_SETUP = 3'd1;   // 组装第 sel 项的分子/分母
    localparam logic [2:0] ST_DIV   = 3'd2;   // DIV_NW 拍除法
    localparam logic [2:0] ST_CAP   = 3'd3;   // 收商，选下一项
    localparam logic [2:0] ST_FUSE  = 3'd4;   // 融合 + 硬闸门
    localparam logic [2:0] ST_MRD   = 3'd5;   // 像素图：发掩膜地址
    localparam logic [2:0] ST_MWR   = 3'd6;   // 像素图：写可信度
    localparam logic [2:0] ST_DONE  = 3'd7;

    logic [2:0]        state;
    logic [1:0]        sel;            // 0=snr 1=margin 2=rho 3=sat
    logic [CONF_W-1:0] s_snr, s_margin, s_rho, s_sat;   // 四个软分数（Q0.16）

    logic [DIV_NW-1:0] div_num;
    logic [DIV_NW:0]   div_rem;
    logic [DIV_NW:0]   div_den;
    logic [DIV_NW-1:0] div_q;
    logic [DIVCW-1:0]  div_cnt;
    logic              div_zero_den;

    logic [MAW:0]      map_idx;

    logic [CONF_W-1:0] conf_pre;       // 融合结果（未过闸门）
    logic [CONF_W-1:0] conf_gated;     // 过闸门后
    logic [7:0]        reason_pre;

    // ------------------------------------------------------------------------
    // 第 sel 项的分子/分母（组合）
    //
    // **满量程点取「2 倍阈值」而不是「阈值」**（这是写 TB 时算出来的结论）：
    //   若按「阈值 = 1.0」定义软分数（s = min(1, 实测/阈值)），则硬闸门一通过
    //   （实测 ≥ 阈值）软分数立刻等于 1.0 —— SNR 与 margin 两项在融合里就**完全
    //   失去区分度**，min 融合实际上退化成 min(rho 项, sat 项)。这不报错，只是
    //   「可信度看起来永远很健康」。
    //   改成「2 倍阈值 = 1.0」后：实测 = 阈值 -> 0.5，实测 = 2 倍阈值 -> 1.0，
    //   既保持了「越差越小」的单调性，又在门限附近留出了有意义的梯度。
    //   rho / sat 是「越小越好」，取 1 减去同一比值。
    // ------------------------------------------------------------------------
    logic [CONF_W:0]   sel_num_hi;
    logic [CONF_W-1:0] sel_den;
    always_comb begin
        case (sel)
            2'd0: begin sel_num_hi = {1'b0, snr_q016};      sel_den = snr_min_q016;    end
            2'd1: begin sel_num_hi = {1'b0, margin_q016};   sel_den = margin_min_q016; end
            2'd2: begin sel_num_hi = {1'b0, rho_q016};      sel_den = rho_max_q016;    end
            default: begin sel_num_hi = {1'b0, sat_ratio_q016}; sel_den = sat_max_q016; end
        endcase
    end

    // ------------------------------------------------------------------------
    // 硬闸门（意图 1：与软分数独立）
    // ------------------------------------------------------------------------
    always_comb begin
        reason_pre = 8'b0;
        // 数据完整性（最高优先）：三条任一置位即灰屏，且与质量指标并列进位图。
        if (raw_ovf_sticky)                   reason_pre[GR_OVF_RAW]  = 1'b1;
        if (ovf_diff)                         reason_pre[GR_OVF_DIFF] = 1'b1;
        if (proto_err)                        reason_pre[GR_PROTO]    = 1'b1;
        // 质量指标（原有五项，语义与位位置保持不变）
        if (sat_ratio_q016 >  sat_max_q016)   reason_pre[GR_SAT]    = 1'b1;
        if (snr_q016       <  snr_min_q016)   reason_pre[GR_SNR]    = 1'b1;
        if (margin_q016    <  margin_min_q016) reason_pre[GR_MARGIN] = 1'b1;
        if (rho_q016       >  rho_max_q016)   reason_pre[GR_RHO]    = 1'b1;
        if (!geom_hash_ok)                    reason_pre[GR_GEOM]   = 1'b1;
    end

    // ------------------------------------------------------------------------
    // 软分数融合（意图 1：只供显示/排序，不参与闸门）
    // ------------------------------------------------------------------------
    logic [CONF_W-1:0] fuse_min, fuse_prod, fuse_avg;
    logic [2*CONF_W-1:0] p01, p23, p_all;
    logic [CONF_W+1:0]   sum4;
    // 等权平均：**必须先把加法加宽到 CONF_W+2 位**。若直接写
    // (s_snr+s_margin+s_rho+s_sat)>>2 并赋给 16 bit，SV 会按上下文宽度做加法，
    // 四项相加时高位被截掉，得到的平均值会系统性偏小。
    assign sum4     = {2'b0, s_snr} + {2'b0, s_margin} + {2'b0, s_rho} + {2'b0, s_sat};
    assign fuse_avg = sum4 >> 2;
    always_comb begin
        // 取最小
        fuse_min = s_snr;
        if (s_margin < fuse_min) fuse_min = s_margin;
        if (s_rho    < fuse_min) fuse_min = s_rho;
        if (s_sat    < fuse_min) fuse_min = s_sat;

        // 连乘（每步 >>CONF_W 回到 Q0.16）
        p01       = s_snr * s_margin;
        p23       = s_rho * s_sat;
        p_all     = (p01 >> CONF_W) * (p23 >> CONF_W);
        fuse_prod = p_all >> CONF_W;
    end

    always_comb begin
        case (w_sel)
            2'd0:    conf_pre = fuse_min;
            2'd1:    conf_pre = fuse_prod;
            default: conf_pre = fuse_avg;
        endcase
    end

    // 过闸门：任一硬条件成立即灰屏，且全局可信度归零（§6.6）
    assign conf_gated = (reason_pre != 8'b0) ? {CONF_W{1'b0}} : conf_pre;

    // ------------------------------------------------------------------------
    // E 式除法（与 fast_alert 同一套恢复余数结构，便于审计）
    // ------------------------------------------------------------------------
    logic [DIV_NW:0] div_tmp;
    assign div_tmp = {div_rem[DIV_NW-1:0], div_num[DIV_NW-1]};

    // 商的截断：num = (实测 << CONF_W)，故 q 的 [CONF_W-1:0] 即「比值」的小数部分
    logic [CONF_W-1:0] q_sat;
    assign q_sat = (|div_q[DIV_NW-1:CONF_W]) ? {CONF_W{1'b1}} : div_q[CONF_W-1:0];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= ST_IDLE;
            sel          <= 2'd0;
            s_snr        <= '0;
            s_margin     <= '0;
            s_rho        <= '0;
            s_sat        <= '0;
            div_num      <= '0;
            div_rem      <= '0;
            div_den      <= '0;
            div_q        <= '0;
            div_cnt      <= '0;
            div_zero_den <= 1'b0;
            map_idx      <= '0;
            global_conf  <= '0;
            low_conf     <= 1'b0;
            force_gray   <= 1'b0;
            gray_reason  <= 8'b0;
            conf_map_waddr <= '0;
            conf_map_wdata <= '0;
            conf_map_we    <= 1'b0;
            done           <= 1'b0;
            cfg_err        <= 1'b0;
        end else begin
            done        <= 1'b0;
            conf_map_we <= 1'b0;

            case (state)
                // ------------------------------------------------------------
                ST_IDLE: begin
                    if (start) begin
                        sel     <= 2'd0;
                        div_cnt <= '0;
                        state   <= ST_SETUP;
                    end
                end

                // ------------------------------------------------------------
                ST_SETUP: begin
                    // num = 实测 << CONF_W（取比值的小数部分）；den = 2 * 阈值
                    div_num <= {{(DIV_NW-2*CONF_W-1){1'b0}}, sel_num_hi, {CONF_W{1'b0}}};
                    if (sel_den == '0) begin
                        div_den      <= {{(DIV_NW){1'b0}}, 1'b1};
                        div_zero_den <= 1'b1;
                        cfg_err      <= 1'b1;
                    end else begin
                        // 2*sel_den 需要 CONF_W+1 位，显式左移一位后放进 49 位的 div_den
                        div_den      <= {1'b0, {(DIV_NW-CONF_W-1){1'b0}}, sel_den, 1'b0};
                        div_zero_den <= 1'b0;
                    end
                    div_rem <= '0;
                    div_q   <= '0;
                    div_cnt <= DIVCW'(DIV_NW);
                    state   <= ST_DIV;
                end

                // ------------------------------------------------------------
                ST_DIV: begin
                    if (div_tmp >= div_den) begin
                        div_rem <= div_tmp - div_den;
                        div_q   <= {div_q[DIV_NW-2:0], 1'b1};
                    end else begin
                        div_rem <= div_tmp;
                        div_q   <= {div_q[DIV_NW-2:0], 1'b0};
                    end
                    div_num <= {div_num[DIV_NW-2:0], 1'b0};
                    div_cnt <= div_cnt - 1'b1;
                    if (div_cnt == DIVCW'(1)) state <= ST_CAP;
                end

                // ------------------------------------------------------------
                // 收商：snr / margin 直接取；rho / sat 取 1 - 比值
                ST_CAP: begin
                    case (sel)
                        2'd0: s_snr    <= q_sat;
                        2'd1: s_margin <= q_sat;
                        2'd2: s_rho    <= (q_sat >= {CONF_W{1'b1}}) ? {CONF_W{1'b0}}
                                                                    : ({CONF_W{1'b1}} - q_sat);
                        default: s_sat <= (q_sat >= {CONF_W{1'b1}}) ? {CONF_W{1'b0}}
                                                                    : ({CONF_W{1'b1}} - q_sat);
                    endcase

                    if (sel == 2'd3) begin
                        state <= ST_FUSE;
                    end else begin
                        sel   <= sel + 2'd1;
                        state <= ST_SETUP;
                    end
                end

                // ------------------------------------------------------------
                ST_FUSE: begin
                    global_conf <= conf_gated;
                    force_gray  <= (reason_pre != 8'b0);
                    gray_reason <= reason_pre;
                    low_conf    <= (conf_gated < low_conf_q016);
                    map_idx     <= '0;
                    state       <= ST_MRD;
                end

                // ------------------------------------------------------------
                // 像素可信度图：mask[n] * global_conf >> 16
                // 注意（易错点 4）：乘积在 16 bit 上做，最后才截成 8 bit
                ST_MRD: begin
                    state <= ST_MWR;
                end

                ST_MWR: begin
                    conf_map_waddr <= map_idx;
                    conf_map_wdata <= (mask_rdata * global_conf) >> CONF_W;
                    conf_map_we    <= 1'b1;

                    if (map_idx == (MAW+1)'(HID_N-1)) begin
                        state <= ST_DONE;
                    end else begin
                        map_idx <= map_idx + 1'b1;
                        state   <= ST_MRD;
                    end
                end

                // ------------------------------------------------------------
                ST_DONE: begin
                    done  <= 1'b1;
                    state <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

    assign mask_raddr = map_idx[MAW-1:0];
    assign busy       = (state != ST_IDLE);

endmodule

`default_nettype wire
