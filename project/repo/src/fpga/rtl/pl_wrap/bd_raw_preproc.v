`timescale 1ns/1ps
// ============================================================================
// pl_wrap/bd_raw_preproc.v · raw_preproc 的 BD 结构壳（纯直通，无任何逻辑）
//
// 为什么需要它：Vivado（2025.2 实测）的 IP integrator **不允许 SystemVerilog 文件
// 作为 module reference 的顶层**：
//     ERROR: [filemgmt 56-195] Reference 'raw_preproc' contains top file
//            '.../raw_preproc.sv' of type SystemVerilog.
//            This type is not allowed as the top file in the reference.
// （2026-09-30 探针复现，见 sim/results/bd_full_path_20260930.md §4.1）
// 因此用一个 .v 结构壳把 SV 核包一层，壳只做端口连接，不改任何行为、不设参数
// （默认参数即 B 档冻结值：WALL_N=96 / BLOCK=10 / CH=3 / COLOR=1 / BS_BITS=20 /
//   ACC_BITS=23 / OUT_BITS=24）。
//
// 端口名与位宽**逐一对齐 src/fpga/rtl/raw_preproc.sv**（不改名），这样 Vivado 的
// 接口推断能把 s_axis_* / m_meas_* 自动归成 AXIS 接口（实测：vlnv
// xilinx.com:interface:axis_rtl:1.0），BD 里据此连接。
// ============================================================================
module bd_raw_preproc (
    input  wire        clk,
    input  wire        rst_n,

    // AXI-Lite 影子配置（由 fpga_control 写，帧内不可变）
    input  wire [15:0] cfg_dark_level,
    input  wire [3:0]  cfg_frames,
    input  wire [15:0] cfg_crop_side,
    input  wire        cfg_bayer_phase,

    // AXI4-Stream 从：RAW16（一拍一像素）
    input  wire [15:0] s_axis_tdata,
    input  wire        s_axis_tvalid,
    output wire        s_axis_tready,
    input  wire        s_axis_tlast,
    input  wire        s_axis_tuser,

    // 测量流输出：WALL_N x WALL_N x CH 块和
    output wire [23:0] m_meas_tdata,
    output wire [1:0]  m_meas_tch,
    output wire        m_meas_tvalid,
    input  wire        m_meas_tready,
    output wire        m_meas_tlast,
    output wire        m_meas_tuser,
    output wire [13:0] m_meas_tblk,

    // 状态与统计
    output wire        frame_done,
    output wire        busy,
    output wire        ovf_sticky,
    output wire        proto_err,
    output wire        cfg_err,
    output wire [31:0] sat_pix_cnt,
    output wire [31:0] total_pix_cnt
);

    raw_preproc u_raw_preproc (
        .clk            (clk),
        .rst_n          (rst_n),
        .cfg_dark_level (cfg_dark_level),
        .cfg_frames     (cfg_frames),
        .cfg_crop_side  (cfg_crop_side),
        .cfg_bayer_phase(cfg_bayer_phase),
        .s_axis_tdata   (s_axis_tdata),
        .s_axis_tvalid  (s_axis_tvalid),
        .s_axis_tready  (s_axis_tready),
        .s_axis_tlast   (s_axis_tlast),
        .s_axis_tuser   (s_axis_tuser),
        .m_meas_tdata   (m_meas_tdata),
        .m_meas_tch     (m_meas_tch),
        .m_meas_tvalid  (m_meas_tvalid),
        .m_meas_tready  (m_meas_tready),
        .m_meas_tlast   (m_meas_tlast),
        .m_meas_tuser   (m_meas_tuser),
        .m_meas_tblk    (m_meas_tblk),
        .frame_done     (frame_done),
        .busy           (busy),
        .ovf_sticky     (ovf_sticky),
        .proto_err      (proto_err),
        .cfg_err        (cfg_err),
        .sat_pix_cnt    (sat_pix_cnt),
        .total_pix_cnt  (total_pix_cnt)
    );

endmodule
