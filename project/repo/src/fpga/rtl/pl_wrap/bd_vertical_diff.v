`timescale 1ns/1ps
// ============================================================================
// pl_wrap/bd_vertical_diff.v · vertical_diff 的 BD 结构壳（纯直通，无任何逻辑）
// 理由与约定见同目录 bd_raw_preproc.v 的文件头。
// 默认参数即 B 档冻结值：WALL_N=96 / CH=3 / IN_BITS=24 / DIFF_BITS=24 / CHW=2。
// 端口名与位宽逐一对齐 src/fpga/rtl/vertical_diff.sv。
// ============================================================================
module bd_vertical_diff (
    input  wire               clk,
    input  wire               rst_n,

    // 配置
    input  wire               cfg_bypass,
    input  wire [15:0]        cfg_wall_n,

    // 输入：块和流（raw_preproc 输出）
    input  wire [23:0]        s_meas_tdata,
    input  wire [1:0]         s_meas_tch,
    input  wire               s_meas_tvalid,
    output wire               s_meas_tready,
    input  wire               s_meas_tlast,
    input  wire               s_meas_tuser,

    // 输出：差分流
    output wire signed [23:0] m_diff_tdata,
    output wire [1:0]         m_diff_tch,
    output wire               m_diff_tvalid,
    input  wire               m_diff_tready,
    output wire               m_diff_tlast,
    output wire               m_diff_tuser,
    output wire [6:0]         m_diff_trow,
    output wire [6:0]         m_diff_tcol,

    // 状态
    output wire               frame_done,
    output wire               ovf_diff,
    output wire               proto_err,
    output wire               busy
);

    vertical_diff u_vertical_diff (
        .clk           (clk),
        .rst_n         (rst_n),
        .cfg_bypass    (cfg_bypass),
        .cfg_wall_n    (cfg_wall_n),
        .s_meas_tdata  (s_meas_tdata),
        .s_meas_tch    (s_meas_tch),
        .s_meas_tvalid (s_meas_tvalid),
        .s_meas_tready (s_meas_tready),
        .s_meas_tlast  (s_meas_tlast),
        .s_meas_tuser  (s_meas_tuser),
        .m_diff_tdata  (m_diff_tdata),
        .m_diff_tch    (m_diff_tch),
        .m_diff_tvalid (m_diff_tvalid),
        .m_diff_tready (m_diff_tready),
        .m_diff_tlast  (m_diff_tlast),
        .m_diff_tuser  (m_diff_tuser),
        .m_diff_trow   (m_diff_trow),
        .m_diff_tcol   (m_diff_tcol),
        .frame_done    (frame_done),
        .ovf_diff      (ovf_diff),
        .proto_err     (proto_err),
        .busy          (busy)
    );

endmodule
