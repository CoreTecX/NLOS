// ============================================================================
// fast_alert.sv  ·  PenumbraSight 快速告警（方案 §6.1 / §6.2 / §6.6）
// 依据：plan_teacher.md §6.6（快速告警与可信度）
//         E = Σ_k (c_k - μ_k)^2 / (v_k + ε)
//         「E 超过标定集 99.5 百分位且连续两帧成立时输出出现/移动；
//           方向由 96x96 差分能量质心的横向变化给出；
//           阈值只用调参集设定，盲测集不重新调阈值」
//       §6.1（vertical_diff 的 9120xRGB 差分同时进 fast_alert 与投影引擎）
//       §6.2（fast_alert 输入 = 当前低维系数、背景均值方差；输出 = event/direction/score）
//       §9.2（告警目标 10 Hz；30 次试验准确率 ≥90%、误报 ≤5%）
//
// ---------------------------------------------------------------------------
// 为什么告警要独立成一路（「双速率输出」这条创新命题的落点）
// ---------------------------------------------------------------------------
//   重建是 16 次固定迭代 PDHG，一次几十毫秒；告警只是 K 个系数的加权平方和，
//   一帧几百拍就够。两者绑在一起，安全告警就被慢速逆问题阻塞了。
//   本模块**完全不依赖定位/重建结果**：c_k 由上游低维归约给出（通常是 U_K^T d，
//   或任何经标定的低维特征），阈值也是离线标定好的常量。
//
// ---------------------------------------------------------------------------
// 三个关键设计决策（附取舍理由）
// ---------------------------------------------------------------------------
// 【决策 1】E 的除法用「共享顺序长除法」，不用每拍 LUT 倒数
//   公式里有 K 次除法，最直觉的做法是放 K 个倒数 LUT 做 1/拍流水。但代价是：
//   LUT 的近似误差直接进 E，而 E 的阈值是「标定集 99.5 百分位」——一个**统计量**，
//   对系统偏差极敏感，LUT 的偏差会把百分位整体挪位。
//   本设计改为：K 个系数串行经过**一个** 48 位恢复余数除法器（每个 48 拍），
//   8 个系数 ≈ 400 拍/帧。10 Hz 告警下占用 ≈ 4000 拍/秒，在 150 MHz 上是 0.003%
//   的时间；用「几乎不存在的吞吐压力」换「整数精确 + 与 Python 定点模型逐位可比」。
//   → 除法是**向下取整（floor）**。bit-exact 的 Python 参考模型必须用同一种舍入，
//     否则 fixed_point_b.yaml 的 acceptance.bit_exact_tolerance_lsb=1 不达标。
//     这类「舍入方向不一致」是定点对拍里最常见的假阳性来源。
//   → 隐含约束：一次 E pass（约 K*(2+48) 拍）必须在下一帧系数到达前算完。
//     400 拍 vs 帧周期（10 Hz -> 1.5e7 拍）余量 4 个数量级。若将来把告警率推到
//     10 kHz 量级才会撞上，届时需要改成 K 路并行倒数或流水除法。
//
// 【决策 2】质心用 |d| 加权，而不是 d^2 加权
//   §6.6 只说「差分能量质心」。若用 d^2：差分是 signed 24 bit，d^2 要 46 bit，
//   而一帧有 95x96x3 = 27360 个样本要累加，48 bit 累加器必然溢出（2^46*2^15=2^61）。
//   改用 |d|：mom0 ≤ 27360*2^23 < 2^38，mom1 ≤ 95*2^38 < 2^45，48 bit 安全。
//   方向判据只取质心的**符号变化**（还带死区），对能量度量形式不敏感。
//
// 【决策 3】质心每次 E pass 后顺带更新，不单独触发
//   若质心自己触发一次 pass，就必须在「只有质心、没有系数」时发布结果——而发布
//   要拿 E 去判 alert，此时 E 是上一帧的陈旧值，会误报。因此：质心只在 E 的 pass
//   里更新（用 pending 标志）。代价是某帧缺系数时质心延后一帧，可接受。
//
// ---------------------------------------------------------------------------
// 易错点
// ---------------------------------------------------------------------------
// (1) 阈值必须在**同一套定点算术**里标定
//     用 float 算出的 99.5 百分位直接写进 cfg_e_thresh，会因为「整数除法截断 +
//     Q 格式不同」整体偏移。标定脚本必须在 Python 里复刻本模块的整数算法
//     （floor 除法、同样的 Q 格式），或用同一份 RTL 在板上跑标定集。
//
// (2) 低能量帧的质心会被粗归一化吃掉
//     质心除法前把 mom0/mom1 同时右移 16 位以压缩位宽；若 mom0 < 2^16（几乎没有
//     差分能量），分子会被截成 0，质心假报 0、方向乱跳。故必须用 cfg_min_energy
//     设下限：低于下限时**冻结上一次质心**而不是报 0。
//
// (3) 「连续两帧」是历史判据，不是当帧判据
//     alert_event = hist[0] & e_over_now（两帧持续）。只写当帧比较，误报率会明显
//     高于 §9.2 的 5%。但也要注意：两帧判据会让「出现」迟一帧，30 次试验统计时
//     必须统一口径，别一半按单帧一半按两帧。
//
// (4) ε 不能省
//     v_k 是方差，静态背景下可能极小甚至为 0（某系数在标定集中恒定）。没有 ε 就是
//     除零：不会崩，而是商变成全 1、E 爆表 → 永久误报。本设计对 den==0 显式换成 1
//     并把 zero_den 顶成粘滞标志。
//     [2026-09-30 修] 两条通路的除零守卫**口径统一**：E 通路（den = v+ε）与质心通路
//     （den = mom0>>16）任一为 0 都换成 1 并置同一 zero_den。E 通路因 cfg_eps>=1
//     结构上不可达（且 cfg_eps==0 在 E pass 前就 cfg_err 短路），实际触发者为质心通路。
//
// (5) 系数缓存只有一层
//     若下一帧系数在本帧 E pass 结束前到达，c_mem 会被覆盖（丢一帧）。当前帧率下
//     余量 4 个数量级（见决策 1）。若哪天把 K 调到 64 且告警率上到 kHz，必须加双缓存。
//
// ---------------------------------------------------------------------------
// 接口口径（端口一览；默认位宽见下方 parameter 列表）
// ---------------------------------------------------------------------------
//   输入
//     clk                  1        时钟
//     rst_n                1        低有效异步复位
//     s_coef_tdata   [15:0]         低维系数 c_k（Q2.16 signed），每帧 K 拍
//     s_coef_tindex   [2:0]         k 索引（协议自检）
//     s_coef_tvalid      1          系数有效
//     s_coef_tlast       1          第 K 拍
//     bg_mu          [15:0]         背景均值 μ_k（Q2.16 signed，BRAM 回读 1 拍延迟）
//     bg_var         [15:0]         背景方差 v_k（Q2.16 unsigned，同地址）
//     s_diff_tdata   [23:0]         差分流（signed 24 bit，仅用于质心）
//     s_diff_tvalid      1          差分有效
//     s_diff_tlast       1          差分块行末
//     s_diff_tuser       1          差分流帧首
//     s_diff_tcol     [6:0]         块列 0..95
//     cfg_e_thresh   [47:0]         E 阈值（标定集 99.5 百分位，同一套定点算术）
//     cfg_k          [15:0]         系数个数（默认 8 = loc_rank）
//     cfg_eps        [15:0]         ε（≥1，防除零）
//     cfg_persist_en     1          1 = 需连续两帧（§6.6 原文）
//     cfg_dir_deadband[15:0]        方向死区（Q0.8）
//     cfg_min_energy [47:0]         质心有效的最低能量（易错点 2）
//   输出
//     s_coef_tready      1          恒 1（写 c_mem 1 拍/个）
//     bg_raddr        [2:0]         背景统计读地址（= k）
//     s_diff_tready      1          恒 1（矩累加 1 拍/值）
//     alert_event        1          出现 / 移动（连续两帧判据）
//     alert_direction  [1:0] signed +1 右移 / -1 左移 / 0 死区内
//     alert_score    [47:0]         E（定点尺度）
//     alert_centroid_x[15:0]        当前横向能量质心（Q0.8）
//     alert_delta_x  [16:0] signed 质心横向变化（Q0.8）
//     alert_centroid_valid 1        本帧质心是否有效（低能量时冻结）
//     frame_done         1          一次判决完成（1 拍脉冲）
//     proto_err          1          系数索引不一致（粘滞）
//     zero_den           1          出现过 den==0（守卫换成 den=1，粘滞）
//                                   注：E 通路的守卫 den=v+ε>=1 结构上不可达，
//                                   实际由质心通路的 n0==0 触发（2026-09-30 统一口径）
//     e_ovf              1          E 累加饱和（粘滞；越界钉在 2^E_W-1）
//     cfg_err            1          配置非法
//   对应文档：plan_teacher.md §6.1（差分流同时进 fast_alert 与投影引擎）/
//             §6.2（fast_alert 行：输入=当前低维系数、背景均值方差；
//                   输出=event / direction / score）/ §6.6（E 式与两帧判据）
//             §6.3（分数/残差 48 或 64 bit）
//             src/software/include/psm_format.h（PS_PSM_LOC_RANK=8 = K）
//
// ---------------------------------------------------------------------------
// 刻意保留的行为（不是 bug；改动前先确认是不是有意为之）
// ---------------------------------------------------------------------------
//   [R1] E 的除法用「单个 48 位恢复余数长除法器串行复用」，**不用** K 路 LUT 倒数。
//        代价是每帧约 K*(2+48) 拍；收益是整数精确、与 Python 定点模型逐位可比。
//        **除法方向是 floor（向下取整）**，Python 参考模型必须用同一种舍入。
//   [R2] 质心用 |d| 加权而不是 d^2 加权（决策 2 已论证 48 bit 溢出问题）。这不是
//        图省事，是位宽硬约束下的有意选择。
//   [R3] 质心只在 E 的 pass 里顺带更新（pending 标志），**不单独触发**。避免「只有
//        质心、没有系数」时拿上一帧的陈旧 E 去判 alert 而误报。
//   [R4] 「连续两帧」用 hist 历史位实现：alert_event = hist[0] & e_over_now。
//        这让「出现」迟一帧；30 次试验统计时必须统一口径。
//   [R5] 低能量时**冻结上一次质心**（不报 0、不清方向），并置 alert_centroid_valid=0。
//        刻意不做「报 0」——那会让方向乱跳。
//   [R6] ε 有下限（cfg_eps==0 直接 cfg_err），且 den==0 时显式换成 1 并置 zero_den。
//        绝不静默除零。两条通路的守卫**口径统一**（E 通路 den=v+ε、质心通路 den=n0）：
//        任一为 0 都换成 1 并置同一粘滞标志；E 通路因 ε>=1 结构上不可达。
//   [R9] E 累加越界时**饱和**（钉在 2^E_W-1）而不是回绕截断（与 rtl/README §4
//        「禁止回绕截断」一致）。生产位宽 E_W=48 下该分支结构上不可达，见 [2026-09-30 修]。
//   [R7] 系数缓存只一层。当前帧率下余量 4 个数量级；若告警率上到 kHz 级且 K 调大，
//        必须加双缓存。这是**已知**的容量上限，不是遗漏。
//   [R8] 阈值只作常量输入，模块内**不做任何自适应统计**（§6.6：盲测集不重调阈值），
//        保证相同输入 -> 相同输出，可复现、可审计。
//
// ---------------------------------------------------------------------------
// 仿真状态声明
// ---------------------------------------------------------------------------
//   本文件**未在任何仿真器上运行过**（本机无 iverilog / Vivado / Verilator）。
//   文件内不含任何仿真的 E 值、误报率或时延数字。配套 tb/fast_alert_tb.sv 是待运行的
//   自检平台：E 一项是**逐值精确**比对（除法为整数精确），质心一项带容差。
// ============================================================================

// 与 tb/fast_alert_tb.sv 取同一值：RTL 缺 timescale 时 xsim 会因
// [XSIM 43-4099] 拒绝 elaboration（见 sim/results/xsim_first_run_20260929.md §6.1）。
`timescale 1ns/1ps
`default_nettype none

module fast_alert #(
    parameter int K_MAX     = 8,        // 低维系数上限（= psm_format.h 的 PS_PSM_LOC_RANK）
    parameter int WALL_N    = 96,       // 墙网格边长（差分列数）
    parameter int DIFF_BITS = 24,       // 差分流位宽（signed）
    parameter int COEF_W    = 16,       // 系数/均值位宽（Q2.16 signed）
    parameter int VAR_W     = 16,       // 方差位宽（Q2.16 unsigned）
    parameter int E_W       = 48,       // E 累加器位宽（§6.3「分数/残差 48 或 64」）
    parameter int DIV_NW    = 48,       // 长除法位宽（分子/商）
    parameter int CENT_FRAC = 8,        // 质心小数位（Q0.8：256 = 1 个块列）
    localparam int KW   = $clog2(K_MAX),      // k 索引位宽
    localparam int COLW = $clog2(WALL_N),     // 列索引位宽
    localparam int D2W  = 2*COEF_W + 2,       // d^2 位宽（17x17 -> 34）
    localparam int WCW  = DIFF_BITS + COLW,   // col*|d| 位宽
    localparam int DIVCW= $clog2(DIV_NW+1)    // 除法计数器位宽
) (
    input  wire logic                        clk,
    input  wire logic                        rst_n,

    // ---------------- 低维系数流：每帧 K 拍（k = 0..K-1）------------------
    input  wire logic [COEF_W-1:0]           s_coef_tdata,   // c_k，Q2.16 signed
    input  wire logic [KW-1:0]               s_coef_tindex,  // k 索引（协议自检）
    input  wire logic                        s_coef_tvalid,
    output logic                        s_coef_tready,  // 恒 1（写 c_mem 1 拍/个）
    input  wire logic                        s_coef_tlast,   // 第 K 拍

    // ---------------- 背景统计读接口（模型侧 BRAM，μ 与 v 同地址）----------
    output logic [KW-1:0]               bg_raddr,
    input  wire logic [COEF_W-1:0]           bg_mu,          // μ_k  Q2.16 signed
    input  wire logic [VAR_W-1:0]            bg_var,         // v_k   Q2.16 unsigned

    // ---------------- 差分流（vertical_diff 输出，只用来算质心）------------
    input  wire logic signed [DIFF_BITS-1:0] s_diff_tdata,
    input  wire logic                        s_diff_tvalid,
    output logic                        s_diff_tready,  // 恒 1（矩累加 1 拍/值）
    input  wire logic                        s_diff_tlast,   // 差分块行末
    input  wire logic                        s_diff_tuser,   // 差分流帧首
    input  wire logic [COLW-1:0]             s_diff_tcol,    // 块列 0..WALL_N-1

    // ---------------- 标定配置 ----------------
    input  wire logic [E_W-1:0]              cfg_e_thresh,     // 标定集 99.5 百分位（同定点尺度）
    input  wire logic [15:0]                 cfg_k,            // 系数个数（默认 8 = loc_rank）
    input  wire logic [VAR_W-1:0]            cfg_eps,          // ε（≥1，见易错点 4）
    input  wire logic                        cfg_persist_en,   // 1 = 需要连续两帧（§6.6 原文）
    input  wire logic [15:0]                 cfg_dir_deadband, // 方向死区（Q0.8）
    input  wire logic [E_W-1:0]              cfg_min_energy,   // 质心有效的最低能量（易错点 2）

    // ---------------- 输出 ----------------
    output logic                        alert_event,        // 出现 / 移动
    output logic signed [1:0]           alert_direction,    // +1 右移 / -1 左移 / 0 死区内
    output logic [E_W-1:0]              alert_score,        // E（定点尺度，见决策 1）
    output logic [15:0]                 alert_centroid_x,   // 当前质心 Q0.8
    output logic signed [16:0]          alert_delta_x,      // 横向变化 Q0.8（有符号）
    output logic                        alert_centroid_valid, // 本帧质心是否有效
    output logic                        frame_done,         // 1 拍脉冲：一次判决完成
    output logic                        proto_err,          // 系数索引不一致
    output logic                        zero_den,           // 出现过 den==0（守卫换成 1，粘滞）
    output logic                        e_ovf,              // E 累加饱和（粘滞）
    output logic                        cfg_err
);

    // ------------------------------------------------------------------------
    // 系数缓存（一层，见易错点 5）
    // ------------------------------------------------------------------------
    logic [COEF_W-1:0] c_mem [0:K_MAX-1];
    logic [KW-1:0]     coef_cnt;
    logic              coef_burst_rdy;

    // ------------------------------------------------------------------------
    // 差分流矩累加：mom0 = Σ|d|，mom1 = Σ col*|d|（见决策 2）
    // ------------------------------------------------------------------------
    logic [DIFF_BITS-1:0]      diff_mag;
    logic [WCW-1:0]            diff_wcol;
    assign diff_mag  = s_diff_tdata[DIFF_BITS-1] ? (~s_diff_tdata + 1'b1) : s_diff_tdata;
    assign diff_wcol = diff_mag * s_diff_tcol;

    logic [E_W-1:0] mom0, mom1;
    logic [E_W-1:0] mom0_pre, mom1_pre;
    logic [E_W-1:0] dwc_ext;
    assign dwc_ext  = {{(E_W-WCW){1'b0}}, diff_wcol};
    // 帧首那拍也要计入，故先「按 tuser 复位」再相加
    assign mom0_pre = s_diff_tuser ? {(E_W){1'b0}} : mom0;
    assign mom1_pre = s_diff_tuser ? {(E_W){1'b0}} : mom1;

    logic [COLW-1:0] diff_row;
    logic            cent_pending;
    logic [E_W-1:0]  mom0_lat, mom1_lat;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mom0         <= '0;
            mom1         <= '0;
            diff_row     <= '0;
            // cent_pending 不在本块赋值 —— 它归主状态机（见本块末与 ST_PUB 处的说明），
            // 2026-09-28 修：原先本块与主状态机都写它，构成双驱动。
            mom0_lat     <= '0;
            mom1_lat     <= '0;
        end else if (s_diff_tvalid) begin
            mom0 <= mom0_pre + diff_mag;
            mom1 <= mom1_pre + dwc_ext;

            if (s_diff_tuser) diff_row <= '0;

            if (s_diff_tlast) begin
                // 差分流共 (WALL_N-1) 行，收到最后一行末即一帧
                if (diff_row == COLW'((WALL_N-1)-1)) begin
                    mom0_lat     <= mom0_pre + diff_mag;
                    mom1_lat     <= mom1_pre + dwc_ext;
                    // cent_pending 的置位已移入主状态机（endcase 之后），见那里的说明。
                    mom0         <= '0;
                    mom1         <= '0;
                end else begin
                    diff_row <= diff_row + 1'b1;
                end
            end
        end
    end

    // ------------------------------------------------------------------------
    // FSM 与数据通路
    // ------------------------------------------------------------------------
    localparam logic [2:0] ST_IDLE  = 3'd0;
    localparam logic [2:0] ST_BGRD  = 3'd1;   // 发背景统计地址
    localparam logic [2:0] ST_SETUP = 3'd2;   // 组装分子/分母，启动除法
    localparam logic [2:0] ST_DIV   = 3'd3;   // DIV_NW 拍恢复余数除法
    localparam logic [2:0] ST_ACC   = 3'd4;   // 收商并累加
    localparam logic [2:0] ST_PUB   = 3'd5;   // 发布判决

    localparam logic OP_E = 1'b0;   // 阶段 1：E 的第 k 项
    localparam logic OP_C = 1'b1;   // 阶段 2：质心

    logic [2:0]        state;
    logic              op;
    logic [KW-1:0]     k_idx;

    // [2026-09-28 首次仿真修] d_s **必须带 signed**：原声明是无符号，于是
    //   d2_u = d_s * d_s 按无符号平方 —— 补码位型 130072（= -1000）平方得
    //   16,918,725,184 而非 1,000,000。后果：d 为负的项把 E 抬高约 1.7e4 倍
    //   （实测 T1 期望 64681、实得 67465976），并连带把「低能量应回落」的帧
    //   也顶在阈值以上。正的 d 看不出问题，所以只在含负差的项上暴露。
    //   注意宽度无需改：Verilog 的 `*` 结果位宽=两操作数之和 17+17=34，与 D2W 一致。
    logic signed [COEF_W:0] d_s;   // c_k - μ_k（Q2.16，17 bit **signed**）
    logic [D2W-1:0]    d2_u;       // d^2（非负）
    logic [VAR_W:0]    x_den;      // v_k + ε

    logic [DIV_NW-1:0] div_num;
    logic [DIV_NW:0]   div_rem;
    logic [DIV_NW:0]   div_den;
    logic [DIV_NW-1:0] div_q;
    logic [DIVCW-1:0]  div_cnt;
    logic              div_zero_den;

    logic [E_W-1:0]    e_acc;
    logic [E_W-1:0]    cent_q;
    logic              cent_ok;

    logic [15:0]       cent_prev;
    logic              cent_prev_ok;
    logic [1:0]        hist;

    // 粗归一化（见易错点 2）
    logic [E_W-1:0]    n0, n1;
    assign n0 = mom0_lat >> 16;
    assign n1 = mom1_lat >> 16;

    logic [DIV_NW:0]   div_tmp;
    assign div_tmp = {div_rem[DIV_NW-1:0], div_num[DIV_NW-1]};

    // 组合：d / d^2 / den / 背景地址
    assign d_s      = $signed(c_mem[k_idx]) - $signed(bg_mu);
    assign d2_u     = d_s * d_s;
    assign x_den    = {1'b0, bg_var} + {1'b0, cfg_eps};
    assign bg_raddr = k_idx;

    // E 累加（带饱和检测）
    logic [E_W:0]      acc_sum;
    logic [E_W-1:0]    acc_next;
    logic              acc_ovf;
    assign acc_sum  = {1'b0, e_acc} + {1'b0, div_q[E_W-1:0]};
    assign acc_ovf  = acc_sum[E_W];
    // [2026-09-30 修] 越界时**饱和**到满量程，不再取低位回绕。
    //   端口注释/README 写的是「E 累加饱和」，而原实现 `acc_sum[E_W-1:0]` 是回绕：
    //   实测（E_W 收窄到 32 的实例）E_true = 17,179,344,900 时 alert_score 得到
    //   4,294,443,012（= E_true mod 2^32）。回绕值可能落回阈值以下、静默压掉告警，
    //   与 rtl/README §4「禁止回绕截断」相悖，故改为饱和（越界即钉在 2^E_W-1）。
    //   注：生产位宽 E_W=48 下溢出**结构上不可达**（单次商 <= 2^34、cfg_k <= K_MAX
    //   时总和 < 2^40 << 2^48），故本改动对板端行为无影响，只是把这条不可达分支的
    //   语义与文档对齐。见 sim/results/rtl_four_issues_20260930.md。
    assign acc_next = acc_ovf ? {E_W{1'b1}} : acc_sum[E_W-1:0];

    logic e_over_w;
    assign e_over_w = (e_acc >= cfg_e_thresh);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state               <= ST_IDLE;
            op                  <= OP_E;
            k_idx               <= '0;
            e_acc               <= '0;
            div_num             <= '0;
            div_rem             <= '0;
            div_den             <= '0;
            div_q               <= '0;
            div_cnt             <= '0;
            div_zero_den        <= 1'b0;
            cent_q              <= '0;
            cent_ok             <= 1'b0;
            cent_pending        <= 1'b0;   // 只在本状态机赋值，见 endcase 处的说明
            cent_prev           <= '0;
            cent_prev_ok        <= 1'b0;
            hist                <= 2'b00;
            alert_event         <= 1'b0;
            alert_direction     <= 2'sd0;
            alert_score         <= '0;
            alert_centroid_x    <= '0;
            alert_delta_x       <= '0;
            alert_centroid_valid<= 1'b0;
            frame_done          <= 1'b0;
            coef_burst_rdy      <= 1'b0;
            zero_den            <= 1'b0;
            e_ovf               <= 1'b0;
        end else begin
            frame_done <= 1'b0;

            // 系数收齐：tlast，或收满 K_MAX（防上游漏 tlast 时卡死）
            if (s_coef_tvalid && s_coef_tready) begin
                if (s_coef_tlast || (coef_cnt == KW'(K_MAX-1))) coef_burst_rdy <= 1'b1;
            end

            case (state)
                // ------------------------------------------------------------
                ST_IDLE: begin
                    if (coef_burst_rdy) begin
                        coef_burst_rdy <= 1'b0;
                        e_acc  <= '0;
                        k_idx  <= '0;
                        op     <= OP_E;
                        if (cfg_err) begin
                            state <= ST_PUB;      // 配置非法：走完流程但不改判决
                        end else begin
                            state <= ST_BGRD;
                        end
                    end
                end

                // ------------------------------------------------------------
                // 发背景统计地址（BRAM 1 拍延迟，故下一状态才取 μ/v）
                ST_BGRD: state <= ST_SETUP;

                // ------------------------------------------------------------
                ST_SETUP: begin
                    if (op == OP_E) begin
                        div_num      <= {{(DIV_NW-D2W){1'b0}}, d2_u};
                        div_den      <= {1'b0, {(DIV_NW-VAR_W-1){1'b0}}, x_den};
                        div_zero_den <= (x_den == '0);
                        if (x_den == '0) div_den <= {{(DIV_NW){1'b0}}, 1'b1};
                    end else begin
                        // 质心：num = (mom1>>16) << CENT_FRAC；den = (mom0>>16)
                        div_num      <= {1'b0, n1[DIV_NW-CENT_FRAC-2:0], {CENT_FRAC{1'b0}}};
                        div_den      <= {1'b0, n0[DIV_NW-2:0]};
                        div_zero_den <= (n0 == '0);
                        if (n0 == '0) div_den <= {{(DIV_NW){1'b0}}, 1'b1};
                    end
                    div_rem <= '0;
                    div_q   <= '0;
                    div_cnt <= DIVCW'(DIV_NW);
                    state   <= ST_DIV;
                end

                // ------------------------------------------------------------
                // 恢复余数除法：q = floor(num / den)，不恢复余数的位串行写法
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

                    if (div_cnt == DIVCW'(1)) state <= ST_ACC;
                end

                // ------------------------------------------------------------
                ST_ACC: begin
                    if (op == OP_E) begin
                        e_acc <= acc_next;
                        if (acc_ovf)     e_ovf    <= 1'b1;
                        if (div_zero_den) zero_den <= 1'b1;

                        if (k_idx == KW'(cfg_k-1)) begin
                            if (cent_pending) begin
                                op      <= OP_C;
                                div_cnt <= '0;
                                state   <= ST_SETUP;   // 顺带算质心（决策 3）
                            end else begin
                                state <= ST_PUB;
                            end
                        end else begin
                            k_idx <= k_idx + 1'b1;
                            state <= ST_BGRD;
                        end
                    end else begin
                        cent_q  <= div_q[E_W-1:0];
                        cent_ok <= (n0 >= cfg_min_energy) && (n0 != '0);
                        // [2026-09-30 修] 与 E 通路的除零守卫统一：质心通路在 n0==0
                        //   （分子/分母守卫把 den 换成 1）时同样把 zero_den 顶成粘滞。
                        //   原实现只在 E 通路置位，而 E 通路的 den = v+ε >= 1 恒不成立
                        //   （cfg_eps==0 又在 E pass 之前就 cfg_err 短路），于是 zero_den
                        //   在 RTL 里永远不可能置起 —— 一个与端口文档不符的死标志。
                        //   质心通路的 n0 = mom0>>16 在静止场景确会为 0，是可达的守卫。
                        //   统一成「两条通路的除零守卫都置位」后，该标志才名副其实。
                        //   见 sim/results/rtl_four_issues_20260930.md。
                        if (div_zero_den) zero_den <= 1'b1;
                        state   <= ST_PUB;
                    end
                end

                // ------------------------------------------------------------
                ST_PUB: begin
                    alert_score <= e_acc;

                    if (!cfg_err) begin
                        // 连续两帧判据（易错点 3）：hist[0] 是上一帧的结论
                        hist <= {hist[0], e_over_w};
                        if (cfg_persist_en) alert_event <= hist[0] & e_over_w;
                        else                alert_event <= e_over_w;
                    end

                    if (op == OP_C) begin
                        cent_pending         <= 1'b0;
                        alert_centroid_valid <= cent_ok;
                        if (cent_ok) begin
                            alert_centroid_x <= div_q[15:0];
                            alert_delta_x    <= $signed({1'b0, div_q[15:0]}) -
                                                $signed({1'b0, cent_prev});
                            if (cent_prev_ok) begin
                                if (($signed({1'b0, div_q[15:0]}) - $signed({1'b0, cent_prev})) >
                                    $signed({1'b0, cfg_dir_deadband})) begin
                                    alert_direction <= 2'sd1;
                                end else if (($signed({1'b0, cent_prev}) -
                                              $signed({1'b0, div_q[15:0]})) >
                                             $signed({1'b0, cfg_dir_deadband})) begin
                                    alert_direction <= -2'sd1;
                                end else begin
                                    alert_direction <= 2'sd0;
                                end
                            end
                            cent_prev    <= div_q[15:0];
                            cent_prev_ok <= 1'b1;
                        end
                        // cent_ok==0：冻结上一次质心与方向（不写 cent_prev / direction）
                    end

                    frame_done <= 1'b1;
                    state      <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase

            // ------------------------------------------------------------
            // cent_pending 的置位（2026-09-28 从「差分流累加」那个 always_ff 移来）
            //
            // 原来它在另一个 always_ff 里置位、在本状态机的 ST_PUB 里清零，
            // 构成「同一触发器被两个 always_ff 驱动」。Vivado 报
            //   Synth 8-6859 multi-driven net on pin cent_pending
            //     1st driver 'cent_pending_reg__0/Q'  (fast_alert.sv:242)
            //     2nd driver 'cent_pending_reg/Q'     (fast_alert.sv:432)
            // 与 raw_preproc 的 br 不同，这里**两个驱动都是寄存器、没有常量驱动**，
            // 所以 Vivado 无法「保留常量、忽略另一个」，结果是综合结果不确定
            // （哪一路生效取决于内部顺序），硬件与 iverilog 的解析很可能不一致。
            //
            // 现统一到本状态机：清零仍在 ST_PUB 里（原 462 行），置位放在
            // endcase **之后** —— Verilog 语义上同一 always 块内后写者优先，
            // 于是同一拍同时发生「一帧差分收齐」与「质心算完」时**以置位为准**，
            // 保证新一帧的质心请求不会丢。
            // 这是本次修复里唯一带语义选择的地方：若要反转优先级，
            // 把这段移到 endcase 之前即可。
            // ------------------------------------------------------------
            if (s_diff_tvalid && s_diff_tlast && (diff_row == COLW'((WALL_N-1)-1)))
                cent_pending <= 1'b1;
        end
    end

    // ------------------------------------------------------------------------
    // 系数缓存写入与协议自检
    // ------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            coef_cnt  <= '0;
            proto_err <= 1'b0;
        end else if (s_coef_tvalid && s_coef_tready) begin
            c_mem[coef_cnt] <= s_coef_tdata;
            if (s_coef_tindex != coef_cnt) proto_err <= 1'b1;
            if (s_coef_tlast) coef_cnt <= '0;
            else              coef_cnt <= coef_cnt + 1'b1;
        end
    end

    assign s_coef_tready = 1'b1;
    assign s_diff_tready = 1'b1;

    // 配置检查
    always_comb begin
        cfg_err = (cfg_k == 16'd0) || (cfg_k > 16'(K_MAX)) || (cfg_eps == '0);
    end

// ----------------------------------------------------------------------------
// 临时调试探针（仅在 -DFA_DEBUG 下编译进来；默认不参与综合也不影响仿真）
// 用途：2026-09-28 定位「E 比期望偏大约 1043 倍」—— 打印每次 E 累加时的
//       系数/背景量/差/差平方/分母，以及除法器输出的商与累加值。
// ----------------------------------------------------------------------------
`ifdef FA_DEBUG
    int dbg_acc_cnt = 0;
    always_ff @(posedge clk) begin
        if (state == ST_ACC && dbg_acc_cnt < 24) begin
            $display("[DBG-ACC] op=%0d k_idx=%0d cfg_k=%0d cfg_err=%b e_acc:%0d -> %0d | q=%0d den=%0d | c=%0d mu=%0d v=%0d eps=%0d d=%0d d2=%0d xden=%0d",
                     op, k_idx, cfg_k, cfg_err, e_acc, acc_next, div_q, div_den,
                     c_mem[k_idx], $signed(bg_mu), bg_var, cfg_eps,
                     $signed(d_s), d2_u, x_den);
            dbg_acc_cnt <= dbg_acc_cnt + 1;
        end
    end
`endif

endmodule

`default_nettype wire
