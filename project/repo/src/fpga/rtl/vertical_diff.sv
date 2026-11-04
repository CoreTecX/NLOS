// ============================================================================
// vertical_diff.sv  ·  PenumbraSight 竖直邻域差分（方案 §6.1 / §6.2 / §6.3）
// 依据：plan_teacher.md §6.1（vertical_diff：9120×RGB signed 测量）
//       §6.2（输入 96x96x3 / 输出 9120x3 signed 24 bit，双行 BRAM）
//       §6.3（垂直差分 signed 24 bit）
//       configs/fixed_point_b.yaml（diff: 24 bit signed）
//       src/hardware/preprocess.hpp :: row_diff()（下一行减上一行）
//       sim/vectors/readme.md（diff_* 期望值）
//
// ---------------------------------------------------------------------------
// 数学定义（与论文 FISTA 基线、Python 参考实现严格一致）
// ---------------------------------------------------------------------------
//   diff[r][c][ch] = sum[(r+1)][c][ch] - sum[r][c][ch]     r = 0 .. WALL_N-2
//   输出尺寸 (WALL_N-1) x WALL_N x CH = 95 x 96 x 3 = 9120 x 3
//   正好等于 ps_psm_format.h 的 PS_PSM_MEAS_H=95 / PS_PSM_MEAS_W=96，
//   也等于 M = meas_h*meas_w = 9120（模型文件里 U 矩阵的行数）。
//
// ---------------------------------------------------------------------------
// 设计意图：为什么是「双行缓存 + 边收边算」，而不是先存整帧
// ---------------------------------------------------------------------------
//   差分只需要相邻两行。输入是行优先流，所以：
//     * 收到第 r 行时，第 r-1 行完整地躺在另一个 bank 里；
//     * 因此可以「边收第 r 行、边出第 r-1 行的差分」，存储只有 2 x WALL_N x CH
//       个 24 bit 字（96x3x2 = 576 字 ≈ 14 Kbit），会落在分布式 RAM / 一份
//       BRAM 的零头里；**不需要**存 9216 x 3 个块和。
//   bank 选择用行号奇偶：写 bank[row%2]，读 bank[1-row%2]，两者永不冲突，
//   于是读写可以在同一拍发生，吞吐 = 1 值/拍。
//
// ---------------------------------------------------------------------------
// 易错点
// ---------------------------------------------------------------------------
// (1) 差分的位宽是「刚刚够」的，不是「差不多够」
//     块和 < 2^23（packed / 8 帧口径，见 raw_preproc 易错点 3），
//     两数之差 < 2^23，正好塞进 signed 24 bit。这是**恰好**够，没有余量。
//     一旦 value_packing 切到 fullscale（块和 26 bit），diff 必须 27 bit，
//     而方案/config 里写的 24/26 都不够——这正是 fixed_point_b.yaml 反复警告的
//     「静默饱和」：图像上只表现为亮区被削平，极难回溯。本模块用 ovf_diff
//     粘滞标志把它顶出来。
//
// (2) 通道序必须在模块内自己数，不能信上游的 tch
//     本模块用内部 ch/col/row 计数器，并把 tch 与内部值逐拍比对（proto_err）。
//     上游少发一拍或多发一拍，会导致「整幅图错位一个通道」——画面依然像图，
//     只是 R/G/B 互相串位，靠肉眼几乎看不出来。
//
// (3) 第 0 行只写不出
//     第 0 行没有「上一行」，必须跳过错分。漏了这个判断会多出 96x3 = 288 个
//     垃圾差分，使 M 从 9120 变成 9408，与模型 U 矩阵的行数对不上，DMA 也不会
//     报错，只会在定位分数上表现为莫名其妙的偏差。
//
// (4) 旁路模式（§9.3 必做对照 2：原始测量 / 垂直差分）
//     cfg_bypass=1 时直通（不差分），此时输出 WALL_N 行 = 96x96x3，
//     供消融实验对照。**旁路时 M 变了，必须同时换模型或用固定位置模式**，
//     不要拿 9120 维的 U 去乘 9216 维的测量。
//
// ---------------------------------------------------------------------------
// 接口口径（端口一览；默认位宽见下方 parameter 列表）
// ---------------------------------------------------------------------------
//   输入
//     clk                1        时钟（与 AXIS 同源）
//     rst_n              1        低有效异步复位
//     cfg_bypass         1        1 = 旁路（直通，不做差分；§9.3 必做对照 2）
//     cfg_wall_n   [15:0]         期望行内块数（0 = 不检查；否则须等于 WALL_N）
//     s_meas_tdata [23:0]         块和（无符号；来自 raw_preproc）
//     s_meas_tch      [1:0]       通道 0=R/单色 1=G 2=B（内部逐拍比对）
//     s_meas_tvalid      1        块和有效
//     s_meas_tlast       1        块行末
//     s_meas_tuser       1        帧首块
//     m_diff_tready      1        下游就绪（反压）
//   输出
//     s_meas_tready      1        可收（1 拍流水；下游反压时拉低）
//     m_diff_tdata [23:0]         差分（signed 24 bit）
//     m_diff_tch      [1:0]       通道
//     m_diff_tvalid      1        差分有效
//     m_diff_tlast       1        差分块行末
//     m_diff_tuser       1        帧首拍
//     m_diff_trow     [6:0]       差分行号 0..94（= 输入行号-1；旁路时=输入行号）
//     m_diff_tcol     [6:0]       块列号 0..95
//     frame_done         1        整帧差分输出完毕（1 拍脉冲，**输出侧**判定）
//     ovf_diff           1        差分饱和（粘滞；位宽口径不符的告警闸）
//     proto_err          1        tch/tlast 与内部计数不一致（粘滞）
//     busy               1        流水或输入有效
//   对应文档：plan_teacher.md §6.1（vertical_diff：9120×RGB signed 测量）/
//             §6.2（输入 96×96×3 / 输出 9120×3 signed 24 bit、双行 BRAM）/
//             §6.3（垂直差分 signed 24 bit）
//             configs/fixed_point_b.yaml（diff: 24 bit signed）
//             src/hardware/preprocess.hpp :: row_diff()
//             src/software/include/psm_format.h（MEAS_H=95 / MEAS_W=96 -> M=9120）
//
// ---------------------------------------------------------------------------
// 刻意保留的行为（不是 bug；改动前先确认是不是有意为之）
// ---------------------------------------------------------------------------
//   [R1] 第 0 行「只写不出」是**刻意的**：漏掉这个判断会多出 96x3 个垃圾差分，
//        使 M 从 9120 变成 9408，与模型 U 的行数对不上且不报错。
//   [R2] 通道序由模块内部自己数，不信任上游 tch（并逐拍比对）。这是刻意加的冗余，
//        用来顶出「R/G/B 串位」这类肉眼几乎发现不了的错位。
//   [R3] frame_done 在**输出侧**判定（末拍真的被下游取走才算）。刻意不用输入侧，
//        否则会早 1~2 拍，上层若据此触发 DMA 会丢末尾几拍。
//   [R4] 旁路模式（cfg_bypass=1）刻意保留、不做「自动识别」。旁路时 M 变了
//        （9120 -> 9216），必须同时换模型或用固定位置模式 —— 模块**不会**替你拦。
//   [R5] 差分布局含「有符号饱和」而不是回绕（fixed_point_b.yaml: overflow=saturate）。
//        饱和会点亮 ovf_diff；这是刻意保留的告警闸，不是多余逻辑。
//   [R6] 只有 1 拍流水 + stall，**不设 FIFO**：差分的生产与消费同速时 1 拍足够；
//        若将来上游突发或下游长阻塞，需要在上层加缓冲，而不是在本模块内加。
//
// ---------------------------------------------------------------------------
// 仿真状态声明
// ---------------------------------------------------------------------------
//   本文件**未在任何仿真器上运行过**（本机无 iverilog / Vivado / Verilator）。
//   文件内不含仿真结果。配套 tb/vertical_diff_tb.sv 是待运行的自检平台。
// ============================================================================

// 与 tb/vertical_diff_tb.sv 取同一值：RTL 缺 timescale 时 xsim 会因
// [XSIM 43-4099] 拒绝 elaboration（见 sim/results/xsim_first_run_20260929.md §6.1）。
`timescale 1ns/1ps
`default_nettype none

module vertical_diff #(
    parameter int WALL_N    = 96,     // 墙网格边长
    parameter int CH        = 3,      // 通道数（1 = 黑白）
    parameter int IN_BITS   = 24,     // 输入块和位宽（无符号）
    parameter int DIFF_BITS = 24,     // 输出差分位宽（有符号）
    parameter int CHW       = 2,      // 通道索引位宽（3 通道 -> 2 bit）
    localparam int NW   = $clog2(WALL_N),           // 行/列索引位宽
    localparam int LINW = $clog2(WALL_N*CH),        // 行内线性地址位宽
    localparam int CHCW = (CH > 1) ? $clog2(CH) : 1 // 通道计数器位宽
) (
    input  wire logic                       clk,
    input  wire logic                       rst_n,

    // ---------------- 配置 ----------------
    input  wire logic                       cfg_bypass,     // 1 = 直通（不做差分的对照模式）
    input  wire logic [15:0]                cfg_wall_n,     // 期望行内块数（0 = 不检查）

    // ---------------- 输入：块和流（raw_preproc 输出）----------------
    input  wire logic [IN_BITS-1:0]         s_meas_tdata,
    input  wire logic [CHW-1:0]             s_meas_tch,
    input  wire logic                       s_meas_tvalid,
    output logic                       s_meas_tready,
    input  wire logic                       s_meas_tlast,   // 块行末
    input  wire logic                       s_meas_tuser,   // 帧首块

    // ---------------- 输出：差分流 ----------------
    output logic signed [DIFF_BITS-1:0] m_diff_tdata,
    output logic [CHW-1:0]              m_diff_tch,
    output logic                        m_diff_tvalid,
    input  wire logic                        m_diff_tready,
    output logic                        m_diff_tlast,   // 差分块行末
    output logic                        m_diff_tuser,   // 帧首拍
    output logic [NW-1:0]               m_diff_trow,   // 0 .. WALL_N-2
    output logic [NW-1:0]               m_diff_tcol,   // 0 .. WALL_N-1

    // ---------------- 状态 ----------------
    output logic                        frame_done,
    output logic                        ovf_diff,       // 差分饱和（位宽配置不一致的告警）
    output logic                        proto_err,      // tch/tlast 与内部计数不一致
    output logic                        busy
);

    // ------------------------------------------------------------------------
    // 双行缓存
    // ------------------------------------------------------------------------
    logic [IN_BITS-1:0] line_mem [0:1][0:WALL_N*CH-1];
    logic [IN_BITS-1:0] line_rdata;

    // ------------------------------------------------------------------------
    // 内部坐标计数器（见易错点 2）
    // ------------------------------------------------------------------------
    logic [CHCW-1:0] ch;
    logic [NW-1:0]   col;
    logic [NW-1:0]   row;

    logic            in_fire_w;
    logic            row_valid_w;     // 该行是否产生输出（第 0 行不产生）
    assign in_fire_w  = s_meas_tvalid && s_meas_tready;

    // --- 本拍实际使用的坐标（帧首拍 tuser 强制为 (0,0,0)）----------------------
    // [2026-09-28 首次仿真的修复] 原实现只在 always_ff 里对 tuser 拍写
    //   row<=0/col<=0/ch<=0，于是**这一拍**的坐标仍是寄存器旧值，而归零后
    //   **下一拍**读到的又是 (0,0,0) —— 等于 tuser 拍把整条坐标流推迟了一拍，
    //   而数据流没被推迟。后果（探针实测）：每帧从第 2 拍起数据比坐标超前一个
    //   (col,ch)，pay_addr 取到下一拍的地址 ⇒ 输出「坐标全对、数值整体错位一个
    //   通道」，并连带让内部 tch/tlast 比对**误报 proto_err**。
    //   修法：本拍坐标用组合覆盖（tuser -> 0），计数器按「本拍坐标的下一拍」推进。
    logic [NW-1:0]   row_use;
    logic [NW-1:0]   col_use;
    logic [CHCW-1:0] ch_use;
    assign row_use = (in_fire_w && s_meas_tuser) ? '0 : row;
    assign col_use = (in_fire_w && s_meas_tuser) ? '0 : col;
    assign ch_use  = (in_fire_w && s_meas_tuser) ? '0 : ch;

    assign row_valid_w = cfg_bypass | (row_use != '0);

    // 行内线性地址：列优先后通道，与生产侧的打拍顺序一致
    logic [LINW-1:0] pay_addr;
    assign pay_addr = LINW'(col_use) * LINW'(CH) + LINW'(ch_use);

    // ------------------------------------------------------------------------
    // 一拍流水（1-deep pipeline with stall）
    //   pipe_valid 为 0 时永远可以收新数据；为 1 时只有下游能收才再收。
    //   这样既不会丢拍，也不需要 FIFO。
    // ------------------------------------------------------------------------
    logic                  pipe_valid;
    logic [IN_BITS-1:0]    pipe_cur;
    logic [CHW-1:0]        pipe_ch;
    logic [NW-1:0]         pipe_row;
    logic [NW-1:0]         pipe_col;
    logic                  pipe_row_valid;

    assign s_meas_tready = ~pipe_valid | m_diff_tready;
    assign m_diff_tvalid = pipe_valid;

    logic out_fire_w;
    assign out_fire_w = m_diff_tvalid && m_diff_tready;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pipe_valid     <= 1'b0;
            pipe_cur       <= '0;
            pipe_ch        <= '0;
            pipe_row       <= '0;
            pipe_col       <= '0;
            pipe_row_valid <= 1'b0;
        end else if (in_fire_w) begin
            // 收新拍优先：若同时 out_fire，旧数据已被下游取走，覆盖即安全
            pipe_valid     <= row_valid_w;
            pipe_cur       <= s_meas_tdata;
            pipe_ch        <= ch_use;
            pipe_row       <= row_use;
            pipe_col       <= col_use;
            pipe_row_valid <= row_valid_w;
        end else if (out_fire_w) begin
            pipe_valid <= 1'b0;
        end
    end

    // ------------------------------------------------------------------------
    // 双 bank 读写
    //   写：bank = row[0]，地址 = 当前拍 (col, ch)
    //   读：bank = ~row[0]，地址 = 当前拍 (col, ch)
    //   → 读的是上一行（行号奇偶必然不同）同一个 (col,ch)
    // ------------------------------------------------------------------------
    logic rd_en_w, we_w;
    assign rd_en_w = in_fire_w && (!cfg_bypass) && (row_use != '0);
    assign we_w    = in_fire_w;

    always_ff @(posedge clk) begin
        if (rd_en_w) line_rdata <= line_mem[~row_use[0]][pay_addr];
    end

    always_ff @(posedge clk) begin
        if (we_w) line_mem[row_use[0]][pay_addr] <= s_meas_tdata;
    end

    // ------------------------------------------------------------------------
    // 差分与饱和
    // ------------------------------------------------------------------------
    localparam logic signed [IN_BITS:0] DIFF_MAX = (IN_BITS+1)'((1 << (DIFF_BITS-1)) - 1);
    localparam logic signed [IN_BITS:0] DIFF_MIN = -(IN_BITS+1)'(1 << (DIFF_BITS-1));

    logic signed [IN_BITS:0] d_full;
    logic signed [DIFF_BITS-1:0] d_sat;
    logic d_ovf_w;

    always_comb begin
        // 旁路模式下 pipe_cur 直接当输出（无符号 -> 有符号，值非负）
        d_full = $signed({1'b0, pipe_cur}) -
                 (cfg_bypass ? {(IN_BITS+1){1'b0}} : $signed({1'b0, line_rdata}));

        d_ovf_w = 1'b0;
        if (d_full > DIFF_MAX) begin
            d_sat   = {1'b0, {(DIFF_BITS-1){1'b1}}};   // 正饱和
            d_ovf_w = 1'b1;
        end else if (d_full < DIFF_MIN) begin
            d_sat   = {1'b1, {(DIFF_BITS-1){1'b0}}};   // 负饱和
            d_ovf_w = 1'b1;
        end else begin
            d_sat = d_full[DIFF_BITS-1:0];
        end
    end

    assign m_diff_tdata = d_sat;
    assign m_diff_tch   = pipe_ch;
    // 旁路模式：输出行号 = 输入行号；差分模式：输出行号 = 输入行号 - 1
    assign m_diff_trow  = cfg_bypass ? pipe_row : (pipe_row - 1'b1);
    assign m_diff_tcol  = pipe_col;
    assign m_diff_tlast = pipe_valid && (pipe_col == (WALL_N-1)) && (pipe_ch == CHCW'(CH-1));
    assign m_diff_tuser = pipe_valid && (pipe_row_valid) &&
                          (pipe_col == '0) && (pipe_ch == '0) &&
                          (cfg_bypass ? (pipe_row == '0) : (pipe_row == NW'(1)));

    // ------------------------------------------------------------------------
    // 坐标计数器与帧边界
    // ------------------------------------------------------------------------
    // 帧结束必须看**输出侧**：输入最后一拍被收走时，最后几个差分还在 1 拍流水里
    // （甚至被下游反压顶住）。若按输入侧出 frame_done，会早 1~2 拍，上层若以此
    // 触发 DMA 会丢掉末尾几拍。上层也可以直接以 m_diff_tlast 判流结束。
    logic frame_last_out_w;
    assign frame_last_out_w = out_fire_w && pipe_row_valid &&
                              (pipe_row == (WALL_N-1)) && (pipe_col == (WALL_N-1)) &&
                              (pipe_ch == CHCW'(CH-1));

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ch         <= '0;
            col        <= '0;
            row        <= '0;
            frame_done <= 1'b0;
        end else begin
            if (frame_last_out_w) frame_done <= 1'b1;
            else                  frame_done <= 1'b0;

            if (in_fire_w) begin
                // 帧同步：每帧第一拍必须带 tuser。本拍坐标已由 *_use 强制为 (0,0,0)，
                // 这里只负责把寄存器推进到**下一拍**的坐标 —— 不能像原实现那样
                // 「本拍归零、拍后仍是 0」，那会让整条坐标流比数据流慢一拍
                // （2026-09-28 首次仿真发现的根因，详见 *_use 处注释）。
                // 若上游忘了带 tuser，row 会一直往 WALL_N 以上涨（WALL_N 非 2 的幂时
                // 不会自动回绕）；此时应靠 proto_err 顶出来，而不是静默出错位。
                if (ch_use == CHCW'(CH-1)) begin
                    ch <= '0;
                    if (col_use == (WALL_N-1)) begin
                        col <= '0;
                        row <= row_use + 1'b1;
                    end else begin
                        col <= col_use + 1'b1;
                    end
                end else begin
                    ch <= ch_use + 1'b1;
                end
            end
        end
    end

    // ------------------------------------------------------------------------
    // 协议自检（见易错点 2）
    // ------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            proto_err <= 1'b0;
            ovf_diff  <= 1'b0;
        end else begin
            // 差分饱和粘滞
            if (d_ovf_w && pipe_valid && pipe_row_valid) ovf_diff <= 1'b1;

            // 上游 tch / tlast 必须与内部计数一致（用 *_use，使帧首拍按 (0,0,0) 比对）
            if (in_fire_w &&
                ((s_meas_tch != ch_use[CHW-1:0]) ||
                 (s_meas_tlast != ((col_use == (WALL_N-1)) && (ch_use == CHCW'(CH-1)))))) begin
                proto_err <= 1'b1;
            end

            // 配置一致性：行内块数必须等于墙网格边长（cfg_wall_n=0 表示不检查）
            if (cfg_wall_n != 16'd0 && cfg_wall_n != 16'(WALL_N)) proto_err <= 1'b1;
        end
    end

    assign busy = pipe_valid | s_meas_tvalid;

// ----------------------------------------------------------------------------
// 临时调试探针（仅在 -DVD_DEBUG 下编译进来；默认不参与综合也不影响仿真）
// 用途：2026-09-28 定位「数据比坐标超前一个 (col,ch)」时打印前若干拍的
//       输入、读地址、读回值、输出，让硬件自己说明对齐关系。
// ----------------------------------------------------------------------------
`ifdef VD_DEBUG
    int dbg_in_cnt  = 0;
    int dbg_out_cnt = 0;
    always_ff @(posedge clk) begin
        if (in_fire_w && dbg_in_cnt < 20) begin
            $display("[DBG-IN ] row=%0d col=%0d ch=%0d data=%0d rd_en=%b pay=%0d lrd=%0d rvv=%b",
                     row, col, ch, s_meas_tdata, rd_en_w, pay_addr, line_rdata, row_valid_w);
            dbg_in_cnt <= dbg_in_cnt + 1;
        end
        if (out_fire_w && dbg_out_cnt < 20) begin
            $display("[DBG-OUT] row=%0d col=%0d ch=%0d d_full=%0d cur=%0d lrd=%0d",
                     pipe_row, pipe_col, pipe_ch, d_full, pipe_cur, line_rdata);
            dbg_out_cnt <= dbg_out_cnt + 1;
        end
    end
`endif

endmodule

`default_nettype wire
