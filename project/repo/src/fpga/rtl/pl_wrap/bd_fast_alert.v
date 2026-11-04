`timescale 1ns/1ps
// ============================================================================
// pl_wrap/bd_fast_alert.v · fast_alert 的 BD 结构壳（纯直通，无任何逻辑）
// 理由与约定见同目录 bd_raw_preproc.v 的文件头。
// 默认参数即 B 档冻结值：K_MAX=8 / WALL_N=96 / DIFF_BITS=24 / COEF_W=16 /
//   VAR_W=16 / E_W=48 / DIV_NW=48 / CENT_FRAC=8。
// 派生位宽（与 SV 核一致）：KW=$clog2(K_MAX)=3、COLW=$clog2(WALL_N)=7。
// 端口名与位宽逐一对齐 src/fpga/rtl/fast_alert.sv。
// ============================================================================
module bd_fast_alert (
    input  wire               clk,
    input  wire               rst_n,

    // 低维系数流：每帧 K 拍
    input  wire [15:0]        s_coef_tdata,
    input  wire [2:0]         s_coef_tindex,
    input  wire               s_coef_tvalid,
    output wire               s_coef_tready,
    input  wire               s_coef_tlast,

    // 背景统计读接口（模型侧 BRAM，μ 与 v 同地址）
    output wire [2:0]         bg_raddr,
    input  wire [15:0]        bg_mu,
    input  wire [15:0]        bg_var,

    // 差分流（vertical_diff 输出，只用来算质心）
    input  wire signed [23:0] s_diff_tdata,
    input  wire               s_diff_tvalid,
    output wire               s_diff_tready,
    input  wire               s_diff_tlast,
    input  wire               s_diff_tuser,
    input  wire [6:0]         s_diff_tcol,

    // 标定配置
    input  wire [47:0]        cfg_e_thresh,
    input  wire [15:0]        cfg_k,
    input  wire [15:0]        cfg_eps,
    input  wire               cfg_persist_en,
    input  wire [15:0]        cfg_dir_deadband,
    input  wire [47:0]        cfg_min_energy,

    // 输出
    output wire               alert_event,
    output wire signed [1:0]  alert_direction,
    output wire [47:0]        alert_score,
    output wire [15:0]        alert_centroid_x,
    output wire signed [16:0] alert_delta_x,
    output wire               alert_centroid_valid,
    output wire               frame_done,
    output wire               proto_err,
    output wire               zero_den,
    output wire               e_ovf,
    output wire               cfg_err
);

    fast_alert u_fast_alert (
        .clk                 (clk),
        .rst_n               (rst_n),
        .s_coef_tdata        (s_coef_tdata),
        .s_coef_tindex       (s_coef_tindex),
        .s_coef_tvalid       (s_coef_tvalid),
        .s_coef_tready       (s_coef_tready),
        .s_coef_tlast        (s_coef_tlast),
        .bg_raddr            (bg_raddr),
        .bg_mu               (bg_mu),
        .bg_var              (bg_var),
        .s_diff_tdata        (s_diff_tdata),
        .s_diff_tvalid       (s_diff_tvalid),
        .s_diff_tready       (s_diff_tready),
        .s_diff_tlast        (s_diff_tlast),
        .s_diff_tuser        (s_diff_tuser),
        .s_diff_tcol         (s_diff_tcol),
        .cfg_e_thresh        (cfg_e_thresh),
        .cfg_k               (cfg_k),
        .cfg_eps             (cfg_eps),
        .cfg_persist_en      (cfg_persist_en),
        .cfg_dir_deadband    (cfg_dir_deadband),
        .cfg_min_energy      (cfg_min_energy),
        .alert_event         (alert_event),
        .alert_direction     (alert_direction),
        .alert_score         (alert_score),
        .alert_centroid_x    (alert_centroid_x),
        .alert_delta_x       (alert_delta_x),
        .alert_centroid_valid(alert_centroid_valid),
        .frame_done          (frame_done),
        .proto_err           (proto_err),
        .zero_den            (zero_den),
        .e_ovf               (e_ovf),
        .cfg_err             (cfg_err)
    );

endmodule
