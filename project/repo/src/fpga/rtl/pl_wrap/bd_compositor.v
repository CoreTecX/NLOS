`timescale 1ns/1ps
// ============================================================================
// pl_wrap/bd_compositor.v · compositor 的 BD 结构壳（纯直通，无任何逻辑）
// 理由与约定见同目录 bd_raw_preproc.v 的文件头。
// 默认参数即 B 档冻结值：1080p60 时序 / WALL_N=96 / HID 24x18 / CAN 9x9x3 /
//   PIX_W=24 / 布局 96->480(x5)、24x18->720x540(x30)。
// 派生位宽（与 SV 核一致）：WALL_AW=$clog2(96*96)=14、REC_AW=$clog2(24*18)=9、
//   OSD_AW=$clog2(32*8)=8。
// 注意（与 RTL readme E3 相关）：本模块的 vid_* 是**自定时**的 1080p 流。
//   2026-10-01 起新增 vid_tready（G3）：BD 侧由下游位宽转换器/搬运器的 tready 反压，
//   0 时 compositor 冻结整条光栅（像素不丢、帧率随带宽自适应）。tready 恒 1 时
//   行为与旧版逐拍相同。见 sim/results/g5_g3_raw_fix_20261001.md。
// 端口名与位宽逐一对齐 src/fpga/rtl/compositor.sv。
// ============================================================================
module bd_compositor (
    input  wire        clk,
    input  wire        rst_n,

    // 视频输出：1080p RGB AXI4-Stream + 同步
    output wire [23:0] vid_tdata,
    output wire        vid_tvalid,
    output wire        vid_tlast,
    output wire        vid_tuser,
    output wire        vid_de,
    output wire        vid_hsync,
    output wire        vid_vsync,
    input  wire        vid_tready,   // G3：下游 AXIS 反压（0 = 冻结光栅）

    // 墙面 ROI 图像写口（96x96 RGB888）
    input  wire        wall_we,
    input  wire [13:0] wall_waddr,
    input  wire [23:0] wall_wdata,

    // 重建图写口（24x18）
    input  wire        rec_we,
    input  wire [8:0]  rec_waddr,
    input  wire [23:0] rec_wdata,

    // 定位 / 可信度
    input  wire [15:0] cand_best_x,
    input  wire [15:0] cand_best_y,
    input  wire [15:0] cand_best_z,
    input  wire [15:0] selected_id,
    input  wire [15:0] conf_q016,
    input  wire        low_conf,
    input  wire        force_gray,
    input  wire [7:0]  gray_reason,     // 与 compositor.sv / confidence_engine.sv 一致：8 bit 位图（bit[7:5] 为数据完整性）

    // OSD 文本写口
    input  wire        osd_we,
    input  wire [7:0]  osd_waddr,
    input  wire [7:0]  osd_wdata,

    // 状态
    output wire        frame_tick,
    output wire        busy
);

    compositor u_compositor (
        .clk          (clk),
        .rst_n        (rst_n),
        .vid_tdata    (vid_tdata),
        .vid_tvalid   (vid_tvalid),
        .vid_tlast    (vid_tlast),
        .vid_tuser    (vid_tuser),
        .vid_de       (vid_de),
        .vid_hsync    (vid_hsync),
        .vid_vsync    (vid_vsync),
        .vid_tready   (vid_tready),
        .wall_we      (wall_we),
        .wall_waddr   (wall_waddr),
        .wall_wdata   (wall_wdata),
        .rec_we       (rec_we),
        .rec_waddr    (rec_waddr),
        .rec_wdata    (rec_wdata),
        .cand_best_x  (cand_best_x),
        .cand_best_y  (cand_best_y),
        .cand_best_z  (cand_best_z),
        .selected_id  (selected_id),
        .conf_q016    (conf_q016),
        .low_conf     (low_conf),
        .force_gray   (force_gray),
        .gray_reason  (gray_reason),
        .osd_we       (osd_we),
        .osd_waddr    (osd_waddr),
        .osd_wdata    (osd_wdata),
        .frame_tick   (frame_tick),
        .busy         (busy)
    );

endmodule
