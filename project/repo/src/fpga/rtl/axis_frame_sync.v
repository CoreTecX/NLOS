// ============================================================================
// axis_frame_sync.v  ·  AXIS 帧边界整形器（G2 修复件）
// ---------------------------------------------------------------------------
// 为什么需要它（问题陈述，出处逐条给出）
// ---------------------------------------------------------------------------
//   AXI DMA 的 S2MM 以 **TLAST** 结束一次传输（PG021）。块设计里差分流直接接到
//   `dma_pipe/S_AXIS_S2MM`，而 `vertical_diff.sv:286` 的
//       assign m_diff_tlast = pipe_valid && (pipe_col == WALL_N-1) && (pipe_ch == CH-1);
//   是**块行末**（`vertical_diff.sv:147` 的端口注释即写「差分块行末」），
//   一帧出现 (WALL_N-1) = 95 次。两者接在一起，结果是「一次传输 = 一个块行」，
//   一帧被切成 95 次搬运 —— 见 sim/results/bd_full_path_20260930.md §7 G2。
//
//   raw 输入侧是同一类错配、方向相反：MM2S 每次传输只在**末尾**置一次 TLAST
//   （`penumbrasight_dma_pipe_0_stub.v` 的 M_AXIS_MM2S：HAS_TLAST 1、TUSER_WIDTH 0），
//   而 `raw_preproc.sv:319-321` 期望 TLAST 落在**每一条 raw 行末**
//   （proto_err_set = ... (s_axis_tlast != raw_row_end) ...），且
//   `raw_preproc.sv:540` 在 ST_IDLE 里等 `s_axis_tvalid && s_axis_tuser` ——
//   而 MM2S 根本不产生 TUSER。即：不整形的话，raw_preproc **永远出不了 ST_IDLE**。
//
// 本模块做什么
// ---------------------------------------------------------------------------
//   只重塑流的两根边界信号，不改数据、不插缓冲、不加流水级：
//     * m_tuser：在本帧第 0 拍（按内部计数器判定）置起；
//     * m_tlast：ROW_BEATS > 0 时落在**每条行末**（每 ROW_BEATS 拍一次）；
//                ROW_BEATS = 0 时只落在**帧末**（每 FRAME_BEATS 拍一次）。
//   上游的 s_axis_tlast / s_axis_tuser **刻意不使用**：上游的 tlast 语义正是要
//   被替换掉的那个（块行末 / 每传输一次），拿它做输入只会把错误语义传下去；
//   s_axis_tlast / s_axis_tuser 保留在端口上只是为了让接口位型与上下游一致
//   （bc_diff 的 M01_AXIS 是 TUSER_WIDTH 1，raw_preproc 的 s_axis 也是）。
//
// 时序与反压
// ---------------------------------------------------------------------------
//   s_axis_tready = m_axis_tready，纯组合直通：**不插寄存器、不加 FIFO**，
//   故不改变原有握手时序，也不引入新的带宽损失。计数器只在
//   `tvalid && tready`（真正握手）时推进，所以下游反压多久都不会错位。
//
// ---------------------------------------------------------------------------
// 易错点 / 使用约束（改动前先读）
// ---------------------------------------------------------------------------
//   (1) FRAME_BEATS 必须是**恰好**一帧的拍数，一帧多一拍或欠一拍都会把 tlast
//       永远推到错误的位置上，而且**不会报错**，只表现为 DMA 搬出来的数据错位。
//       差分流（非旁路）：FRAME_BEATS = (WALL_N-1)*WALL_N*CH = 95*96*3 = 27360。
//       raw 输入：ROW_BEATS = RAW_SIDE = 2*BLOCK*WALL_N = 1920，
//                 FRAME_BEATS = RAW_SIDE*RAW_SIDE = 3,686,400。
//   (2) **旁路模式（vertical_diff 的 cfg_bypass=1）会改帧长**：输出变成
//       WALL_N*WALL_N*CH = 27648 拍。`vertical_diff.sv` [R4] 已经写明「旁路时 M 变了，
//       必须同时换模型……模块不会替你拦」——本整形器同样不替你拦：旁路下必须把
//       FRAME_BEATS 一起改成 27648，否则帧边界会漂。旁路是 §9.3 的消融对照，
//       不是默认档。
//   (3) 计数器靠「复位后第一拍 = 帧首拍」对齐。vertical_diff 的行/列计数器同样由
//       rst_n 复位、且每帧由输入 tuser 重新起头，所以两侧起点一致。
//       若上游中途丢帧（半帧就停），对齐会丢，需要靠复位恢复 —— 这是本模块
//       不试图兜住的边界（不猜、不静默重同步）。
//   (4) ROWS = FRAME_BEATS / ROW_BEATS 必须是整数；ROW_BEATS>0 时按参数做常量除法，
//       不整除会静默取整 —— 本模块用位宽断言式注释提醒（两处参数都写成算式）。
//
// ---------------------------------------------------------------------------
// 端口
// ---------------------------------------------------------------------------
//   s_axis_*  数据/valid/ready/last/user 直通入口（与上游同宽）
//   m_axis_*  整形后的出口（tdata/valid/ready 直通，tuser/tlast 由本模块产生）
// ---------------------------------------------------------------------------
// 仿真状态
// ---------------------------------------------------------------------------
//   配套 tb/axis_frame_sync_tb.sv 已在本机 Vivado xsim 下跑过，结论见
//   sim/results/g2_g1_g4_fix_20260930.md（含比对点数）。
// ============================================================================

`timescale 1ns/1ps

module axis_frame_sync #(
    // 数据位宽：差分流 24（三通道 8 bit），raw 输入 16
    parameter integer DW          = 24,
    // 一条行的拍数；0 = 不做行级 tlast，只在帧末置一次
    parameter integer ROW_BEATS   = 0,
    // 一帧的拍数（必须恰好等于上游一帧的握手拍数，见易错点 1）
    parameter integer FRAME_BEATS = 27360
) (
    input  wire               clk,
    input  wire               rst_n,

    // ---- AXIS 从（上游）----
    input  wire [DW-1:0]      s_axis_tdata,
    input  wire               s_axis_tuser,   // 保留端口，不使用（见文件头）
    input  wire               s_axis_tvalid,
    output wire               s_axis_tready,
    input  wire               s_axis_tlast,   // 保留端口，不使用（见文件头）

    // ---- AXIS 主（下游）----
    output wire [DW-1:0]      m_axis_tdata,
    output wire               m_axis_tuser,
    output wire               m_axis_tvalid,
    input  wire               m_axis_tready,
    output wire               m_axis_tlast
);

    // ROW_BEATS>0 时的行数（常量除法；两者必须整除，见文件头易错点 4）
    localparam integer ROWS = (ROW_BEATS > 0) ? (FRAME_BEATS / ROW_BEATS) : 1;

    // 行内计数与行计数。位宽按最大使用场景给足（raw: 1920 拍/行、1920 行）。
    reg [31:0] col_cnt;
    reg [15:0] row_cnt;

    wire beat_fire = s_axis_tvalid && s_axis_tready;

    // 边界判定（全部按**当前拍**的计数值组合判定，配合 tvalid 门控）
    wire row_last_w   = (ROW_BEATS > 0) ? (col_cnt == (ROW_BEATS - 1)) : 1'b0;
    wire frame_first_w = (col_cnt == 32'd0) && (row_cnt == 16'd0);
    wire frame_last_w  = (ROW_BEATS > 0)
                       ? (row_last_w && (row_cnt == (ROWS - 1)))
                       : (col_cnt == (FRAME_BEATS - 1));
    // tlast 落点：有行级语义时落在**每条行末**，否则只落在帧末（见文件头）
    wire tlast_w = (ROW_BEATS > 0) ? row_last_w : frame_last_w;

    assign s_axis_tready = m_axis_tready;
    assign m_axis_tvalid = s_axis_tvalid;          // 直通，不插寄存器
    assign m_axis_tdata  = s_axis_tdata;
    // tvalid 门控：无效拍绝不误置（任务一的显式要求之一）
    assign m_axis_tuser  = s_axis_tvalid && frame_first_w;
    assign m_axis_tlast  = s_axis_tvalid && tlast_w;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            col_cnt <= 32'd0;
            row_cnt <= 16'd0;
        end else if (beat_fire) begin
            if (frame_last_w) begin
                // 帧末：两个计数器一起归零（帧首拍从 0 重新起头）
                col_cnt <= 32'd0;
                row_cnt <= 16'd0;
            end else if (ROW_BEATS > 0) begin
                if (row_last_w) begin
                    col_cnt <= 32'd0;
                    row_cnt <= row_cnt + 16'd1;
                end else begin
                    col_cnt <= col_cnt + 32'd1;
                end
            end else begin
                col_cnt <= col_cnt + 32'd1;
            end
        end
    end

endmodule
