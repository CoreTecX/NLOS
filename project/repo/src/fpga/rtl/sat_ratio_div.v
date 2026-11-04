// ============================================================================
// sat_ratio_div.v  ·  饱和像素比例的比例器（G5：confidence_engine 的 sat_ratio_q016 的 PL 内来源）
// ---------------------------------------------------------------------------
// 为什么需要它（问题陈述，出处逐条给出）
// ---------------------------------------------------------------------------
//   `raw_preproc.sv` 只**累加**本组的饱和像素数与总像素数
//   （`raw_preproc.sv:497-508`；[R5] 明写「饱和度只在模块内累加、不比较阈值」，
//    P1 原则：不在 raw_preproc 里做除法）。
//   `src/fpga/rtl/readme.md` F6 记的缺口：「比例计算与 0.1% 阈值比较尚未接到
//   confidence_engine，需要一层 PS/PL 胶水（分频除法或先归一化再送 Q0.16）」。
//   本模块就是那一层除法：把 sat_pix_cnt / total_pix_cnt 变成 Q0.16 的比值。
//
//   阈值比较（比例 > 0.1%）**不在这里**——它已经在 `confidence_engine.sv:248`
//   （`sat_ratio_q016 > sat_max_q016` -> gray_reason bit0）。本模块只提供被比较的量。
//
// ---------------------------------------------------------------------------
// 语义与取整口径（必须与 confidence_engine 的其它三项一致）
// ---------------------------------------------------------------------------
//   ratio_q016 = floor( sat_cnt * 2^OUT_W / total_cnt )，越界饱和到 2^OUT_W-1。
//   * floor 是刻意的：`readme.md` §4 末段要求 confidence_engine / fast_alert 的除法
//     一律整数 floor（向下取整），Python 参考模型必须同一口径。
//   * sat_cnt == total_cnt 时比值恰为 1.0 = 2^16，**超出 16 bit 可表示范围**，
//     故饱和到 0xFFFF（与 `confidence_engine.sv` 的 q_sat 同口径）。
//
// ---------------------------------------------------------------------------
// 采样策略（这是本模块最容易误用的一点，务必读）
// ---------------------------------------------------------------------------
//   raw_preproc 的两个计数只在 ST_ACC 期间自增，而它们在 **ST_DONE 那拍被清零**
//   （`raw_preproc.sv:501-503`），随后的 `frame_done` 脉冲落在**清零之后**的拍上。
//   所以「用 frame_done 触发采样」会拿到 0/0 —— 这条陷阱写在这里免得重踩。
//   本模块改用**输入稳定即发布**的策略，不需要外部触发：
//
//     * HOLD：只有在 `total_cnt != 0` 且「本拍计数 != 上次已发布的计数」时才开一次除法；
//     * RUN ：48 拍恢复余数除法；期间任何一拍若输入变了，**本次结果作废**，回 HOLD；
//     * 只有「整段除法期间输入一字未变」的结果才发布并记为 last_pair。
//
//   于是：ST_ACC 期间（计数每拍都变）永远发布不出来；RMW/OUT 这段长稳定窗
//   （>27k 拍，见 raw_preproc 的意图 3）里一次除法就能跑完并发布；
//   ST_DONE 把计数清零后 total_cnt == 0 -> 停在 HOLD、**保留上一次发布值**。
//   下游 confidence_engine 由 `vertical_diff/frame_done` 触发，落在稳定窗之后，
//   读到的是本组最终比例。
//   **不做**「total==0 就发布 0」这种写法 —— 那会在清零瞬间把正确值冲掉。
//
//   中途丢帧（半帧就停）时策略同样成立：计数停在半途的值上，除法照跑、照发布
//   「半帧的比例」。本模块不猜、不静默重同步 —— 由上层用 raw_busy / 帧计数判。
//
// ---------------------------------------------------------------------------
// 端口
// ---------------------------------------------------------------------------
//   sat_cnt / total_cnt  raw_preproc 的两个计数（同拍、同组）
//   ratio_q016           已发布的比例（Q0.16）
//   ratio_valid          1 = 至少成功发布过一次（此后恒 1，不清零；复位为 0）
//   busy                 1 = 正在跑除法
//
// ---------------------------------------------------------------------------
// 仿真状态
// ---------------------------------------------------------------------------
//   配套 tb/sat_ratio_div_tb.sv（iverilog 与 xsim 各跑一遍），比对点 21，
//   结论见 sim/results/g5_g3_raw_fix_20261001.md（G5 收口报告的复核）。
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module sat_ratio_div #(
    parameter integer IN_W  = 32,   // 计数位宽（raw_preproc 的两个计数）
    parameter integer OUT_W = 16    // 比例位宽（Q0.16）
) (
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire [IN_W-1:0]       sat_cnt,
    input  wire [IN_W-1:0]       total_cnt,

    output reg  [OUT_W-1:0]      ratio_q016,
    output reg                   ratio_valid,
    output reg                   busy
);

    // 被除数 = sat_cnt << OUT_W，宽 IN_W+OUT_W；除数 = total_cnt（零扩展到同宽比较）
    localparam integer NW = IN_W + OUT_W;      // 48（默认参数下）
    localparam integer CW = $clog2(NW + 1);    // 迭代计数位宽
    // 余数寄存器：rem < den < 2^IN_W，故 (rem<<1)|bit < 2^(IN_W+1)，留 2 位余量
    localparam integer RW = IN_W + 2;

    localparam [0:0] ST_HOLD = 1'b0;
    localparam [0:0] ST_RUN  = 1'b1;

    reg  [0:0]      state;
    reg  [NW-1:0]   num_s;      // 被除数移位寄存器
    reg  [RW-1:0]   denx;       // 除数（零扩展）
    reg  [RW-1:0]   rem;        // 余数
    reg  [NW-1:0]   quo;        // 商（NW 位；低 OUT_W 位即比例）
    reg  [CW-1:0]   cnt;        // 剩余迭代数
    reg  [IN_W-1:0] sat_l, tot_l;             // 本次除法锁存的输入
    reg  [2*IN_W-1:0] last_pair;              // 上次成功发布时对应的计数对

    wire [2*IN_W-1:0] cur_pair = {sat_cnt, total_cnt};

    // 恢复余数除法的一步（全部组合）
    wire [RW-1:0]   rem_sh = {rem[RW-2:0], num_s[NW-1]};   // (rem<<1) | 当前被除数位
    wire            ge     = (rem_sh >= denx);
    wire [RW-1:0]   rem_n  = ge ? (rem_sh - denx) : rem_sh;
    wire [NW-1:0]   quo_n  = {quo[NW-2:0], ge};

    // 商 -> Q0.16：高过 OUT_W 的位非零即饱和（sat == total 时商恰为 2^OUT_W）
    wire            ratio_hi    = |quo_n[NW-1:OUT_W];
    wire [OUT_W-1:0] ratio_next = ratio_hi ? {OUT_W{1'b1}} : quo_n[OUT_W-1:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= ST_HOLD;
            num_s       <= {NW{1'b0}};
            denx        <= {RW{1'b0}};
            rem         <= {RW{1'b0}};
            quo         <= {NW{1'b0}};
            cnt         <= {CW{1'b0}};
            sat_l       <= {IN_W{1'b0}};
            tot_l       <= {IN_W{1'b0}};
            last_pair   <= {(2*IN_W){1'b0}};
            ratio_q016  <= {OUT_W{1'b0}};
            ratio_valid <= 1'b0;
            busy        <= 1'b0;
        end else begin
            case (state)
                // ----------------------------------------------------------
                // HOLD：等一个「非零且与上次发布不同」的计数对
                //   total_cnt == 0 时**保持在 HOLD**（保留上次发布值，见文件头）。
                // ----------------------------------------------------------
                ST_HOLD: begin
                    busy <= 1'b0;
                    if ((total_cnt != {IN_W{1'b0}}) && (cur_pair != last_pair)) begin
                        num_s <= {sat_cnt, {OUT_W{1'b0}}};
                        denx  <= {{(RW-IN_W){1'b0}}, total_cnt};
                        rem   <= {RW{1'b0}};
                        quo   <= {NW{1'b0}};
                        cnt   <= NW;            // 隐含截断到 CW 位（NW 恒在范围内）
                        sat_l <= sat_cnt;
                        tot_l <= total_cnt;
                        busy  <= 1'b1;
                        state <= ST_RUN;
                    end
                end

                // ----------------------------------------------------------
                // RUN：固定 NW 拍。输入中途一变即作废（不发布）。
                // ----------------------------------------------------------
                ST_RUN: begin
                    if (cur_pair != {sat_l, tot_l}) begin
                        // 输入在除法期间变了：本次结果作废，回 HOLD 用新值重开
                        busy  <= 1'b0;
                        state <= ST_HOLD;
                    end else if (cnt == {{(CW-1){1'b0}}, 1'b1}) begin
                        ratio_q016  <= ratio_next;
                        ratio_valid <= 1'b1;
                        last_pair   <= {sat_l, tot_l};
                        busy        <= 1'b0;
                        state       <= ST_HOLD;
                    end else begin
                        rem   <= rem_n;
                        quo   <= quo_n;
                        num_s <= {num_s[NW-2:0], 1'b0};
                        cnt   <= cnt - 1'b1;
                    end
                end

                default: state <= ST_HOLD;
            endcase
        end
    end

endmodule

`default_nettype wire
