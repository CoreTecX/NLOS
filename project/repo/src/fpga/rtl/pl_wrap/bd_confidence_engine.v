`timescale 1ns/1ps
// ============================================================================
// pl_wrap/bd_confidence_engine.v · confidence_engine 的 BD 结构壳（纯直通）
// 理由与约定见同目录 bd_raw_preproc.v 的文件头。
// 默认参数即 B 档冻结值：CONF_W=16 / MASK_W=8 / HID_N=432 / DIV_NW=48。
// 派生位宽（与 SV 核一致）：MAW=$clog2(HID_N)=9，故 conf_map_waddr 为 [9:0]。
// 端口名与位宽逐一对齐 src/fpga/rtl/confidence_engine.sv。
// ============================================================================
module bd_confidence_engine (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,

    // 四项实时量（全部 Q0.16）
    input  wire [15:0] snr_q016,
    input  wire [15:0] margin_q016,
    input  wire [15:0] rho_q016,
    input  wire [15:0] sat_ratio_q016,
    input  wire        geom_hash_ok,

    // 数据完整性标志（来自 raw_preproc / vertical_diff，最高优先闸门项）
    input  wire        raw_ovf_sticky,
    input  wire        ovf_diff,
    input  wire        proto_err,

    // 标定阈值（Q0.16，PC 端算好）
    input  wire [15:0] snr_min_q016,
    input  wire [15:0] margin_min_q016,
    input  wire [15:0] rho_max_q016,
    input  wire [15:0] sat_max_q016,
    input  wire [15:0] low_conf_q016,
    input  wire [1:0]  w_sel,

    // 可观测性掩膜读接口
    output wire [8:0]  mask_raddr,
    input  wire [7:0]  mask_rdata,

    // 输出
    output wire [15:0] global_conf,
    output wire        low_conf,
    output wire        force_gray,
    output wire [7:0]  gray_reason,
    output wire [9:0]  conf_map_waddr,
    output wire [7:0]  conf_map_wdata,
    output wire        conf_map_we,
    output wire        done,
    output wire        busy,
    output wire        cfg_err
);

    confidence_engine u_confidence_engine (
        .clk              (clk),
        .rst_n            (rst_n),
        .start            (start),
        .snr_q016         (snr_q016),
        .margin_q016      (margin_q016),
        .rho_q016         (rho_q016),
        .sat_ratio_q016   (sat_ratio_q016),
        .geom_hash_ok     (geom_hash_ok),
        .raw_ovf_sticky   (raw_ovf_sticky),
        .ovf_diff         (ovf_diff),
        .proto_err        (proto_err),
        .snr_min_q016     (snr_min_q016),
        .margin_min_q016  (margin_min_q016),
        .rho_max_q016     (rho_max_q016),
        .sat_max_q016     (sat_max_q016),
        .low_conf_q016    (low_conf_q016),
        .w_sel            (w_sel),
        .mask_raddr       (mask_raddr),
        .mask_rdata       (mask_rdata),
        .global_conf      (global_conf),
        .low_conf         (low_conf),
        .force_gray       (force_gray),
        .gray_reason      (gray_reason),
        .conf_map_waddr   (conf_map_waddr),
        .conf_map_wdata   (conf_map_wdata),
        .conf_map_we      (conf_map_we),
        .done             (done),
        .busy             (busy),
        .cfg_err          (cfg_err)
    );

endmodule
