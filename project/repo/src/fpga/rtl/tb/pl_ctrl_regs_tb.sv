// ============================================================================
// tb/pl_ctrl_regs_tb.sv · pl_ctrl_regs（G1/G4 寄存器叶）自检测试平台
// ---------------------------------------------------------------------------
// 覆盖内容
// ---------------------------------------------------------------------------
//   T1 复位默认值：逐条读回，必须等于上一版 xlconstant 的钉扎值
//      （bd_full_path_20260930.md §6.4）—— 这条保证「PS 不写任何寄存器」时
//      链上行为与上一版一致。
//   T2 读写回环：SCRATCH 与几个 cfg 寄存器写-读一致；并同时核对**标量输出脚**
//      已经跟着变（写寄存器真的驱动到了 RTL 的 cfg_* 上，而不是只存在寄存器里）。
//   T3 写脉冲（G4）：写 CMP_WALL_DATA / CMP_REC_DATA / CMP_OSD_DATA 各产生**恰好一拍**
//      write（wall_we / rec_we / osd_we），且脉冲那一拍的地址/数据就是刚写的值；
//      只写地址寄存器**不**产生脉冲。
//   T4 系数流（G5）：写 FA_COEF_PUSH 产生恰好一拍 coef_tvalid，tdata/tindex/tlast
//      与写入值一致。
//   T5 背景表（G5）：写 FA_BG_ADDR + FA_BG_MU/FA_BG_VAR 后，按 bg_raddr 读回的
//      bg_mu/bg_var 正确（同步读 1 拍延迟）。
//   T6 状态回读：把输入状态脚置成已知值，读 STAT_STICKY / STAT_ALERT / CE_STAT 等，
//      逐位比对。
//   T7 帧计数：给 raw_frame_done 两个脉冲，读 CTR_RAW = 2。
//   T8 CTRL 重载默认值：改过 RAW_CFG1 之后写 CTRL bit0，再读回应回到 1920。
//   T9 字节选通：按 WSTRB 只写一个字节，检查其余字节保持不变。
//   T10 饱和比例的 PL 内来源（2026-09-30 补，G5 收口）：给 raw_sat_pix_cnt /
//      raw_total_pix_cnt 一对值，读 CE_SAT_DIV 应等于 floor(sat*2^16/total)；
//      CE_CFG bit3=1（默认）时 confidence_engine 的 sat 输入脚跟着这个值；
//      CE_CFG bit3=0 时退回 PS 写的 CE_SAT；CE_SATSTAT bit2 是「> CE_SAT_MAX」比较。
//   T11 掩膜表（2026-09-30 补）：写 MASK_ADDR/MASK_DATA 之后按 ce_mask_raddr 同步读回；
//      未写过的项保持上电值 0xFF（= 上一版 xlconstant 的值）。
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module pl_ctrl_regs_tb;

    localparam int AW = 12;

    logic clk = 1'b0;
    always #5 clk = ~clk;
    logic rst_n = 1'b0;

    int err_cnt = 0;
    int chk_cnt = 0;

    // ---------------- AXI-Lite 主机 ----------------
    logic [AW-1:0] awaddr;
    logic          awvalid;
    logic          awready;
    logic [31:0]   wdata;
    logic [3:0]    wstrb;
    logic          wvalid;
    logic          wready;
    logic [1:0]    bresp;
    logic          bvalid;
    logic          bready;
    logic [AW-1:0] araddr;
    logic          arvalid;
    logic          arready;
    logic [31:0]   rdata;
    logic [1:0]    rresp;
    logic          rvalid;
    logic          rready;

    // ---------------- 被验模块的标量脚 ----------------
    logic [15:0] cfg_dark_level, cfg_crop_side, cfg_wall_n, cfg_k, cfg_eps, cfg_dir_deadband;
    logic [3:0]  cfg_frames;
    logic        cfg_bayer_phase, cfg_bypass, cfg_persist_en;
    logic [47:0] cfg_e_thresh, cfg_min_energy;
    logic [15:0] coef_tdata;
    logic [2:0]  coef_tindex;
    logic        coef_tvalid, coef_tlast;
    logic [2:0]  bg_raddr;
    logic [15:0] bg_mu, bg_var;
    logic [15:0] ce_snr_q016, ce_margin_q016, ce_rho_q016, ce_sat_ratio_q016;
    logic        ce_geom_hash_ok;
    logic [15:0] ce_snr_min_q016, ce_margin_min_q016, ce_rho_max_q016, ce_sat_max_q016;
    logic [15:0] ce_low_conf_q016;
    logic ce_start;
    integer ce_start_count=0;
    always @(posedge clk) if(ce_start) ce_start_count <= ce_start_count+1;
    logic [1:0]  ce_w_sel;
    logic [8:0]  ce_mask_raddr;
    logic [7:0]  ce_mask_rdata;
    logic [15:0] cmp_cand_best_x, cmp_cand_best_y, cmp_cand_best_z, cmp_selected_id;
    logic [15:0] cmp_conf_q016;
    logic        cmp_low_conf;
    logic        wall_we, rec_we, osd_we;
    logic [13:0] wall_waddr;
    logic [23:0] wall_wdata;
    logic [8:0]  rec_waddr;
    logic [23:0] rec_wdata;
    logic [7:0]  osd_waddr, osd_wdata;

    // ---------------- 状态输入（TB 驱动）----------------
    logic        raw_ovf_sticky, raw_proto_err, raw_cfg_err, raw_busy;
    logic [31:0] raw_sat_pix_cnt, raw_total_pix_cnt;
    logic        raw_frame_done;
    logic        diff_ovf, diff_proto_err, diff_busy, diff_frame_done;
    logic        fa_alert_event;
    logic [1:0]  fa_alert_direction;
    logic [47:0] fa_alert_score;
    logic [15:0] fa_alert_centroid_x;
    logic [16:0] fa_alert_delta_x;
    logic        fa_alert_centroid_valid, fa_frame_done, fa_proto_err, fa_zero_den;
    logic        fa_e_ovf, fa_cfg_err;
    logic [15:0] ce_global_conf;
    logic        ce_low_conf, ce_force_gray;
    logic [7:0]  ce_gray_reason;
    // A5（2026-10-02）：pl_ctrl_regs 把 ovf/proto 状态脚转发给 confidence_engine 的硬闸门。
    // 这里接出来只为不为空脚（本 TB 的 T6 用 RAW_STAT/DIFF_STAT 已覆盖同一组脚）。
    logic        ce_ovf_raw, ce_ovf_diff, ce_proto_err;
    logic        ce_done, ce_busy, ce_cfg_err;
    logic        cmp_frame_tick, cmp_busy;

    pl_ctrl_regs dut (
        .clk(clk), .rst_n(rst_n),
        .s_axi_awaddr(awaddr), .s_axi_awvalid(awvalid), .s_axi_awready(awready),
        .s_axi_wdata(wdata), .s_axi_wstrb(wstrb), .s_axi_wvalid(wvalid), .s_axi_wready(wready),
        .s_axi_bresp(bresp), .s_axi_bvalid(bvalid), .s_axi_bready(bready),
        .s_axi_araddr(araddr), .s_axi_arvalid(arvalid), .s_axi_arready(arready),
        .s_axi_rdata(rdata), .s_axi_rresp(rresp), .s_axi_rvalid(rvalid), .s_axi_rready(rready),
        .cfg_dark_level(cfg_dark_level), .cfg_frames(cfg_frames),
        .cfg_crop_side(cfg_crop_side), .cfg_bayer_phase(cfg_bayer_phase),
        .cfg_bypass(cfg_bypass), .cfg_wall_n(cfg_wall_n),
        .cfg_e_thresh(cfg_e_thresh), .cfg_k(cfg_k), .cfg_eps(cfg_eps),
        .cfg_persist_en(cfg_persist_en), .cfg_dir_deadband(cfg_dir_deadband),
        .cfg_min_energy(cfg_min_energy),
        .coef_tdata(coef_tdata), .coef_tindex(coef_tindex),
        .coef_tvalid(coef_tvalid), .coef_tlast(coef_tlast),
        .bg_raddr(bg_raddr), .bg_mu(bg_mu), .bg_var(bg_var),
        .ce_snr_q016(ce_snr_q016), .ce_margin_q016(ce_margin_q016),
        .ce_rho_q016(ce_rho_q016), .ce_sat_ratio_q016(ce_sat_ratio_q016),
        .ce_geom_hash_ok(ce_geom_hash_ok),
        .ce_ovf_raw(ce_ovf_raw), .ce_ovf_diff(ce_ovf_diff), .ce_proto_err(ce_proto_err),
        .ce_snr_min_q016(ce_snr_min_q016), .ce_margin_min_q016(ce_margin_min_q016),
        .ce_rho_max_q016(ce_rho_max_q016), .ce_sat_max_q016(ce_sat_max_q016),
        .ce_start(ce_start), .ce_low_conf_q016(ce_low_conf_q016), .ce_w_sel(ce_w_sel),
        .ce_mask_raddr(ce_mask_raddr), .ce_mask_rdata(ce_mask_rdata),
        .cmp_cand_best_x(cmp_cand_best_x), .cmp_cand_best_y(cmp_cand_best_y),
        .cmp_cand_best_z(cmp_cand_best_z), .cmp_selected_id(cmp_selected_id),
        .cmp_conf_q016(cmp_conf_q016), .cmp_low_conf(cmp_low_conf),
        .wall_we(wall_we), .wall_waddr(wall_waddr), .wall_wdata(wall_wdata),
        .rec_we(rec_we), .rec_waddr(rec_waddr), .rec_wdata(rec_wdata),
        .osd_we(osd_we), .osd_waddr(osd_waddr), .osd_wdata(osd_wdata),
        .raw_ovf_sticky(raw_ovf_sticky), .raw_proto_err(raw_proto_err),
        .raw_cfg_err(raw_cfg_err), .raw_busy(raw_busy),
        .raw_sat_pix_cnt(raw_sat_pix_cnt), .raw_total_pix_cnt(raw_total_pix_cnt),
        .raw_frame_done(raw_frame_done),
        .diff_ovf(diff_ovf), .diff_proto_err(diff_proto_err), .diff_busy(diff_busy),
        .diff_frame_done(diff_frame_done),
        .fa_alert_event(fa_alert_event), .fa_alert_direction(fa_alert_direction),
        .fa_alert_score(fa_alert_score), .fa_alert_centroid_x(fa_alert_centroid_x),
        .fa_alert_delta_x(fa_alert_delta_x), .fa_alert_centroid_valid(fa_alert_centroid_valid),
        .fa_frame_done(fa_frame_done), .fa_proto_err(fa_proto_err),
        .fa_zero_den(fa_zero_den), .fa_e_ovf(fa_e_ovf), .fa_cfg_err(fa_cfg_err),
        .ce_global_conf(ce_global_conf), .ce_low_conf(ce_low_conf),
        .ce_force_gray(ce_force_gray), .ce_gray_reason(ce_gray_reason),
        .ce_done(ce_done), .ce_busy(ce_busy), .ce_cfg_err(ce_cfg_err),
        .cmp_frame_tick(cmp_frame_tick), .cmp_busy(cmp_busy)
    );

    assign bready = 1'b1;
    assign rready = 1'b1;

    // ------------------------------------------------------------------
    // AXI-Lite 主机任务
    // ------------------------------------------------------------------
    task automatic axi_write(input [AW-1:0] a, input [31:0] d, input [3:0] s);
        @(negedge clk);
        awaddr  = a;
        wdata   = d;
        wstrb   = s;
        awvalid = 1'b1;
        wvalid  = 1'b1;
        @(posedge clk);              // 本拍完成写（组合 ready）
        @(negedge clk);
        awvalid = 1'b0;
        wvalid  = 1'b0;
        wstrb   = 4'h0;
    endtask

    task automatic axi_read(input [AW-1:0] a, output [31:0] d);
        @(negedge clk);
        araddr  = a;
        arvalid = 1'b1;
        @(posedge clk);
        while (rvalid !== 1'b1) @(posedge clk);
        #1 d = rdata;
        @(negedge clk);
        arvalid = 1'b0;
    endtask

    task automatic chk32(input [31:0] got, input [31:0] exp, input [127:0] tag);
        chk_cnt++;
        if (got !== exp) begin
            $display("[ERR] %0s: got=0x%08x exp=0x%08x", tag, got, exp);
            err_cnt++;
        end
    endtask

    // ------------------------------------------------------------------
    // 写脉冲监视（G4/G5）
    // ------------------------------------------------------------------
    int wall_pulse = 0, rec_pulse = 0, osd_pulse = 0, coef_pulse = 0;
    logic [13:0] exp_w_addr; logic [23:0] exp_w_data;
    logic [8:0]  exp_r_addr; logic [23:0] exp_r_data;
    logic [7:0]  exp_o_addr; logic [7:0]  exp_o_data;
    logic [15:0] exp_c_data; logic [2:0]  exp_c_idx; logic exp_c_last;

    always @(posedge clk) begin
        if (rst_n) begin
            if (wall_we) begin
                chk_cnt++;
                wall_pulse++;
                if ((wall_waddr !== exp_w_addr) || (wall_wdata !== exp_w_data)) begin
                    $display("[ERR] wall_we 拍上 addr/data 不对 got=(%0d,%06x) exp=(%0d,%06x)",
                             wall_waddr, wall_wdata, exp_w_addr, exp_w_data);
                    err_cnt++;
                end
            end
            if (rec_we) begin
                chk_cnt++;
                rec_pulse++;
                if ((rec_waddr !== exp_r_addr) || (rec_wdata !== exp_r_data)) begin
                    $display("[ERR] rec_we 拍上 addr/data 不对");
                    err_cnt++;
                end
            end
            if (osd_we) begin
                chk_cnt++;
                osd_pulse++;
                if ((osd_waddr !== exp_o_addr) || (osd_wdata !== exp_o_data)) begin
                    $display("[ERR] osd_we 拍上 addr/data 不对");
                    err_cnt++;
                end
            end
            if (coef_tvalid) begin
                chk_cnt++;
                coef_pulse++;
                if ((coef_tdata !== exp_c_data) || (coef_tindex !== exp_c_idx) ||
                    (coef_tlast !== exp_c_last)) begin
                    $display("[ERR] coef 拍上值不对 got=(%04x,%0d,%b) exp=(%04x,%0d,%b)",
                             coef_tdata, coef_tindex, coef_tlast,
                             exp_c_data, exp_c_idx, exp_c_last);
                    err_cnt++;
                end
            end
        end
    end

    // ------------------------------------------------------------------
    // 主流程
    // ------------------------------------------------------------------
    logic [31:0] v;

    initial begin
        awaddr = '0; awvalid = 1'b0; wdata = '0; wstrb = 4'h0; wvalid = 1'b0;
        araddr = '0; arvalid = 1'b0;
        exp_w_addr = '0; exp_w_data = '0;
        exp_r_addr = '0; exp_r_data = '0;
        exp_o_addr = '0; exp_o_data = '0;
        exp_c_data = '0; exp_c_idx = '0; exp_c_last = 1'b0;

        raw_ovf_sticky=0; raw_proto_err=0; raw_cfg_err=0; raw_busy=0;
        raw_sat_pix_cnt=0; raw_total_pix_cnt=0; raw_frame_done=0;
        diff_ovf=0; diff_proto_err=0; diff_busy=0; diff_frame_done=0;
        fa_alert_event=0; fa_alert_direction=0; fa_alert_score=0;
        fa_alert_centroid_x=0; fa_alert_delta_x=0; fa_alert_centroid_valid=0;
        fa_frame_done=0; fa_proto_err=0; fa_zero_den=0; fa_e_ovf=0; fa_cfg_err=0;
        ce_global_conf=0; ce_low_conf=0; ce_force_gray=0; ce_gray_reason=0;
        ce_done=0; ce_busy=0; ce_cfg_err=0;
        cmp_frame_tick=0; cmp_busy=0;
        bg_raddr = 3'd0;
        ce_mask_raddr = 9'd0;

        repeat (8) @(posedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);

        // ---------------- T1 复位默认值 ----------------
        $display("[INFO] T1 复位默认值（必须等于上一版 xlconstant 的钉扎值）");
        axi_read(12'h000, v); chk32(v, 32'h5053_0001, "ID");
        axi_read(12'h004, v); chk32(v, 32'd1,          "VERSION");
        axi_read(12'h040, v); chk32(v, 32'd4 << 16,    "RAW_CFG0(frames=4)");
        axi_read(12'h044, v); chk32(v, 32'd1920,       "RAW_CFG1(crop=1920)");
        axi_read(12'h060, v); chk32(v, 32'h0060_0000,  "DIFF_CFG0(wall_n=96)");
        axi_read(12'h084, v); chk32(v, 32'd0,          "FA_ETHR_HI");
        axi_read(12'h088, v); chk32(v, 32'h0001_0008,  "FA_CFG0(k=8,eps=1)");
        axi_read(12'h08C, v); chk32(v, 32'h0001_0020,  "FA_CFG1(db=32,persist)");
        axi_read(12'h0DC, v); chk32(v, 32'd66,         "CE_SAT_MAX(0x0042)");
        axi_read(12'h0E4, v); chk32(v, 32'd12,         "CE_CFG(geom_ok=1,sat_src_pl=1)");
        axi_read(12'h100, v); chk32(v, 32'd3,          "CMP_X");
        axi_read(12'h118, v); chk32(v, 32'h0000_0000,  "CMP_WALL_ADDR");
        // 标量脚默认值
        chk32({28'd0, cfg_frames}, 32'd4, "cfg_frames 默认");
        chk32({16'd0, cfg_crop_side}, 32'd1920, "cfg_crop_side 默认");
        chk32({16'd0, cfg_wall_n}, 32'd96, "cfg_wall_n 默认");
        chk32({16'd0, cfg_k}, 32'd8, "cfg_k 默认");
        chk32({16'd0, cfg_eps}, 32'd1, "cfg_eps 默认");
        chk32({16'd0, cfg_e_thresh[47:32]}, 32'd0, "cfg_e_thresh 高位默认");

        // ---------------- T2 读写回环 + 输出脚 ----------------
        $display("[INFO] T2 读写回环（并把 cfg 脚跟着核对）");
        axi_write(12'h008, 32'hA5A5_1234, 4'hF);
        axi_read(12'h008, v); chk32(v, 32'hA5A5_1234, "SCRATCH 回读");

        axi_write(12'h040, 32'h0008_0100, 4'hF);   // dark=0x0100, frames=8
        axi_read(12'h040, v); chk32(v, 32'h0008_0100, "RAW_CFG0 回读");
        chk32({16'd0, cfg_dark_level}, 32'd256, "cfg_dark_level 跟随");
        chk32({28'd0, cfg_frames},     32'd8,   "cfg_frames 跟随");

        axi_write(12'h060, 32'h0030_0001, 4'hF);   // bypass=1, wall_n=0x30
        axi_read(12'h060, v); chk32(v, 32'h0030_0001, "DIFF_CFG0 回读");
        chk32({31'd0, cfg_bypass}, 32'd1,  "cfg_bypass 跟随");
        chk32({16'd0, cfg_wall_n}, 32'd48, "cfg_wall_n 跟随");

        axi_write(12'h088, 32'h0004_0002, 4'hF);   // k=2, eps=4
        chk32({16'd0, cfg_k},   32'd2, "cfg_k 跟随");
        chk32({16'd0, cfg_eps}, 32'd4, "cfg_eps 跟随");

        axi_write(12'h080, 32'h1122_3344, 4'hF);   // ethr_lo
        axi_write(12'h084, 32'h0000_0055, 4'hF);   // ethr_hi(16b) -> 0x55_11223344
        chk32(cfg_e_thresh[31:0], 32'h1122_3344, "cfg_e_thresh[31:0]");
        chk32({16'd0, cfg_e_thresh[47:32]}, 32'h55, "cfg_e_thresh[47:32]");

        axi_write(12'h110, 32'h0000_7FFF, 4'hF);   // conf_q016
        chk32({16'd0, cmp_conf_q016}, 32'h7FFF, "cmp_conf_q016 跟随");

        // ---------------- T9 字节选通 ----------------
        $display("[INFO] T9 字节选通（只写低字节）");
        axi_write(12'h008, 32'hA5A5_1234, 4'hF);
        axi_write(12'h008, 32'hFFFF_FF77, 4'h1);   // 只改 [7:0]
        axi_read(12'h008, v); chk32(v, 32'hA5A5_1277, "SCRATCH 字节选通");

        // ---------------- T3 三个写口（G4）----------------
        $display("[INFO] T3 compositor 三写口");
        // 墙图：先写地址（不应有脉冲），再写数据（应有 1 拍脉冲）
        axi_write(12'h118, 32'h0000_0ABC, 4'hF);
        repeat (4) @(posedge clk);
        chk32({28'd0, wall_pulse}, 32'd0, "只写地址不产生 wall_we");
        exp_w_addr = 14'h0ABC; exp_w_data = 24'h00_63_9C;
        axi_write(12'h11C, 32'h0000_639C, 4'hF);
        repeat (4) @(posedge clk);
        chk32({28'd0, wall_pulse}, 32'd1, "wall_we 恰好 1 拍");
        chk32({10'd0, wall_waddr}, 32'h0ABC, "wall_waddr 值");

        exp_r_addr = 9'h155; exp_r_data = 24'h00_FF_00;
        axi_write(12'h120, 32'h0000_0155, 4'hF);
        axi_write(12'h124, 32'h0000_FF00, 4'hF);
        repeat (4) @(posedge clk);
        chk32({28'd0, rec_pulse}, 32'd1, "rec_we 恰好 1 拍");
        chk32({23'd0, rec_waddr}, 32'h155, "rec_waddr 值");

        exp_o_addr = 8'h81; exp_o_data = 8'h41;
        axi_write(12'h128, 32'h0000_0081, 4'hF);
        axi_write(12'h12C, 32'h0000_0041, 4'hF);
        repeat (4) @(posedge clk);
        chk32({28'd0, osd_pulse}, 32'd1, "osd_we 恰好 1 拍");
        chk32({24'd0, osd_waddr}, 32'h81, "osd_waddr 值");

        // ---------------- T4 系数流（G5）----------------
        $display("[INFO] T4 c_k 系数流");
        exp_c_data = 16'h0102; exp_c_idx = 3'd3; exp_c_last = 1'b1;
        axi_write(12'h098, 32'h000B_0102, 4'hF);   // data=0x0102 idx=3 last=1（bit19=1, bits[18:16]=3）
        repeat (4) @(posedge clk);
        chk32({28'd0, coef_pulse}, 32'd1, "coef_tvalid 恰好 1 拍");

        // ---------------- T5 背景表（G5）----------------
        $display("[INFO] T5 μ/v 背景表");
        axi_write(12'h09C, 32'h0000_0002, 4'hF);   // FA_BG_ADDR = 2
        axi_write(12'h0A0, 32'h0000_1234, 4'hF);   // μ = 0x1234
        axi_write(12'h0A4, 32'h0000_5678, 4'hF);   // v = 0x5678
        bg_raddr = 3'd2;
        repeat (3) @(posedge clk);
        chk32({16'd0, bg_mu},  32'h1234, "bg_mu（同步 1 拍）");
        chk32({16'd0, bg_var}, 32'h5678, "bg_var（同步 1 拍）");
        axi_write(12'h0A0, 32'h0000_0000, 4'hF);   // 只读回不落表（未重写 ADDR）
        bg_raddr = 3'd1;
        repeat (3) @(posedge clk);
        chk32({16'd0, bg_mu}, 32'h0000, "bg_mu 表项 1 仍为 0");

        // ---------------- T6 状态回读 ----------------
        $display("[INFO] T6 状态回读");
        raw_ovf_sticky=1; raw_proto_err=1; raw_cfg_err=0; raw_busy=1;
        diff_ovf=1; diff_proto_err=0; diff_busy=0;
        fa_proto_err=0; fa_zero_den=1; fa_e_ovf=0; fa_cfg_err=0; ce_cfg_err=0;
        repeat (2) @(posedge clk);
        axi_read(12'h010, v); chk32(v, 32'h0000_0001, "STAT_BUSY(raw_busy=1 其余 0)");
        axi_read(12'h014, v); chk32(v, 32'h0000_004B, "STAT_STICKY");
        axi_read(12'h048, v); chk32(v, 32'h0000_000B, "RAW_STAT(ovf|proto|busy)");
        axi_read(12'h064, v); chk32(v, 32'h0000_0001, "DIFF_STAT");

        raw_sat_pix_cnt = 32'h0001_0203; raw_total_pix_cnt = 32'h0002_0304;
        repeat (2) @(posedge clk);
        axi_read(12'h04C, v); chk32(v, 32'h0001_0203, "RAW_SAT_CNT");
        axi_read(12'h050, v); chk32(v, 32'h0002_0304, "RAW_TOT_CNT");

        fa_alert_event=1; fa_alert_direction=2'b11; fa_alert_centroid_valid=1;
        ce_low_conf=1; ce_force_gray=0; ce_gray_reason=5'b00101;
        repeat (2) @(posedge clk);
        // b0=1, [2:1]=11 -> 0x7, b3=1 -> 0x8, b4=1 -> 0x10, [10:6]=00101 -> 0x140
        axi_read(12'h018, v); chk32(v, 32'h0000_015F, "STAT_ALERT");

        fa_alert_score = 48'h00AB_0000_0009;
        fa_alert_centroid_x = 16'h0BAD;
        fa_alert_delta_x = 17'h1_0001;
        ce_global_conf = 16'h4000;
        ce_done=1; ce_busy=1; ce_gray_reason=5'b00011; ce_force_gray=1; ce_low_conf=1;
        repeat (2) @(posedge clk);
        axi_read(12'h0A8, v); chk32(v, 32'h0000_0009, "FA_SCORE_LO");
        axi_read(12'h0AC, v); chk32(v, 32'h0000_00AB, "FA_SCORE_HI");
        axi_read(12'h0B0, v); chk32(v, 32'h0000_0BAD, "FA_CENTROID");
        axi_read(12'h0B4, v); chk32(v, 32'h0001_0001, "FA_DELTA_X");
        axi_read(12'h0E8, v); chk32(v, 32'h0000_4000, "CE_CONF");
        // b0 low_conf=1, b1 force_gray=1, [6:2] gray_reason=00011 -> 0xC,
        // b7 done=1 -> 0x80, b8 busy=1 -> 0x100, b9 cfg_err=0
        axi_read(12'h0EC, v); chk32(v, 32'h0000_018F, "CE_STAT");

        // ---------------- T7 帧计数 ----------------
        $display("[INFO] T7 帧计数");
        @(negedge clk); raw_frame_done = 1'b1;
        @(posedge clk); @(negedge clk); raw_frame_done = 1'b0;
        @(negedge clk); raw_frame_done = 1'b1;
        @(posedge clk); @(negedge clk); raw_frame_done = 1'b0;
        repeat (2) @(posedge clk);
        axi_read(12'h01C, v); chk32(v, 32'd2, "CTR_RAW=2");

        @(negedge clk); diff_frame_done = 1'b1;
        @(posedge clk); @(negedge clk); diff_frame_done = 1'b0;
        repeat (2) @(posedge clk);
        axi_read(12'h020, v); chk32(v, 32'd1, "CTR_DIFF=1");

        // ---------------- T8 CTRL 重载默认值 ----------------
        $display("[INFO] T8 CTRL 重载默认值");
        axi_write(12'h044, 32'h0000_0123, 4'hF);   // 改 RAW_CFG1
        axi_read(12'h044, v); chk32(v, 32'h0000_0123, "RAW_CFG1 已改");
        axi_write(12'h00C, 32'h0000_0001, 4'hF);   // CTRL bit0
        repeat (3) @(posedge clk);
        axi_read(12'h044, v); chk32(v, 32'd1920, "重载后 RAW_CFG1 回默认");
        // SCRATCH 刻意不参与「重载默认值」，应保持 T9 末尾的值
        axi_read(12'h008, v); chk32(v, 32'hA5A5_1277, "重载不改 SCRATCH");

        // ---------------- T10 饱和比例的 PL 内来源（G5 收口）----------------
        $display("[INFO] T10 饱和比例：PL 除法器 -> CE 输入脚（默认档）");
        raw_sat_pix_cnt = 32'd66; raw_total_pix_cnt = 32'd65536;   // 66/65536 -> 66
        repeat (80) @(posedge clk);
        axi_read(12'h0F0, v); chk32(v, 32'd66, "CE_SAT_DIV = floor(66*2^16/65536)");
        axi_read(12'h0F4, v); chk32((v >> 0) & 32'h1, 32'd0, "SATSTAT b0 div_busy=0");
        axi_read(12'h0F4, v); chk32((v >> 1) & 32'h1, 32'd1, "SATSTAT b1 sat_src_pl=1");
        axi_read(12'h0F4, v); chk32((v >> 2) & 32'h1, 32'd0, "SATSTAT b2 66>66 不成立");
        axi_read(12'h0F4, v); chk32((v >> 3) & 32'h1, 32'd1, "SATSTAT b3 ratio_valid=1");
        chk32({16'd0, ce_sat_ratio_q016}, 32'd66, "CE 的 sat 输入脚 = 66（PL 档）");
        chk32({16'd0, ce_sat_max_q016},   32'd66, "ce_sat_max 脚 = 66");

        raw_sat_pix_cnt = 32'd67;                     // 67/65536 -> 越过 0.1%
        repeat (80) @(posedge clk);
        axi_read(12'h0F0, v); chk32(v, 32'd67, "CE_SAT_DIV = 67");
        axi_read(12'h0F4, v); chk32((v >> 2) & 32'h1, 32'd1, "SATSTAT b2 67>66 成立");
        chk32({16'd0, ce_sat_ratio_q016}, 32'd67, "CE 的 sat 输入脚 = 67（PL 档）");

        $display("[INFO] T10b 退回手写档（CE_CFG bit3 = 0）");
        axi_write(12'h0E4, 32'd0, 4'hF);              // sat_src_pl = 0（geom_hash_ok 一并清）
        axi_write(12'h0CC, 32'h0000_1234, 4'hF);      // CE_SAT
        repeat (4) @(posedge clk);
        chk32({16'd0, ce_sat_ratio_q016}, 32'h1234, "手写档下 CE 输入 = 0x1234");
        axi_read(12'h0F4, v); chk32((v >> 1) & 32'h1, 32'd0, "SATSTAT b1 sat_src_pl=0");
        axi_read(12'h0F0, v); chk32(v, 32'd67, "CE_SAT_DIV 仍反映 PL 侧最新值 67");
        axi_write(12'h0E4, 32'd12, 4'hF);             // 还原默认档
        repeat (2) @(posedge clk);

        // ---------------- T11 掩膜表（432x8 RAM）----------------
        $display("[INFO] T11 掩膜表：上电 0xFF / 写后同步读出");
        ce_mask_raddr = 9'd431;
        repeat (3) @(posedge clk);
        chk32({24'd0, ce_mask_rdata}, 32'hFF, "未写过的项 = 0xFF（旧常量值）");
        axi_write(12'h0F8, 32'h0000_01AF, 4'hF);      // MASK_ADDR = 431
        axi_write(12'h0FC, 32'h0000_007E, 4'hF);      // MASK_DATA = 0x7E（写它落表）
        repeat (3) @(posedge clk);
        chk32({24'd0, ce_mask_rdata}, 32'h7E, "写后按 mask_raddr 读出 0x7E");
        axi_read(12'h0FC, v); chk32(v, 32'h7E,  "MASK_DATA 回读");
        axi_read(12'h0F8, v); chk32(v, 32'h1AF, "MASK_ADDR 回读 = 431");
        ce_mask_raddr = 9'd0;
        repeat (3) @(posedge clk);
        chk32({24'd0, ce_mask_rdata}, 32'hFF, "另一项仍为 0xFF");
        ce_mask_raddr = 9'd431;
        repeat (3) @(posedge clk);
        chk32({24'd0, ce_mask_rdata}, 32'h7E, "回到 431 仍为 0x7E（内容未丢）");

        // ---------------- T12 margin/rho/snr 寄存器 -> CE 输入脚（只到寄存器层）----------------
        // 说明：这三项在本模块里**只有 PS 写的寄存器**，PL 内没有自动来源
        //   （margin 在 core_proj 的 s_axilite、rho 在 core_pdhg 的 DDR 缓冲）。
        //   本用例只证明「写寄存器 -> CE 输入脚真的跟着变」，不是「PL 内来源已通」。
        $display("[INFO] T12 CE 三项标量：写寄存器 -> 输入脚跟随");
        axi_write(12'h0C0, 32'h0000_0AAA, 4'hF);   // CE_SNR
        chk32({16'd0, ce_snr_q016}, 32'h0AAA, "ce_snr_q016 跟随 CE_SNR");
        axi_write(12'h0C4, 32'h0000_2ABC, 4'hF);   // CE_MARGIN
        chk32({16'd0, ce_margin_q016}, 32'h2ABC, "ce_margin_q016 跟随 CE_MARGIN");
        axi_read (12'h0C4, v); chk32(v, 32'h2ABC, "CE_MARGIN 回读");
        axi_write(12'h0C8, 32'h0000_1357, 4'hF);   // CE_RHO
        chk32({16'd0, ce_rho_q016}, 32'h1357, "ce_rho_q016 跟随 CE_RHO");
        axi_read (12'h0C8, v); chk32(v, 32'h1357, "CE_RHO 回读");

        // ---------------- T13 geom_hash_ok：CE_CFG bit2 -> CE 输入脚 ----------------
        // 来源说明：CE_CFG bit2 由 PS 依 ps_psm_check_header()/ps_model_render_allowed()
        //   的返回值写入（本模块内没有几何哈希计算，故这不是「PL 自动来源」）。
        $display("[INFO] T13 geom_hash_ok：CE_CFG bit2 控制 CE 输入脚");
        axi_write(12'h0E4, 32'h0000_000C, 4'hF);   // bit2=1（geom ok）bit3=1（sat pl）
        chk32({31'd0, ce_geom_hash_ok}, 32'd1, "ce_geom_hash_ok=1（bit2=1）");
        axi_write(12'h0E4, 32'h0000_0008, 4'hF);   // bit2=0 -> 几何不一致
        chk32({31'd0, ce_geom_hash_ok}, 32'd0, "ce_geom_hash_ok=0（bit2=0）");
        axi_write(12'h0E4, 32'd12, 4'hF);          // 还原默认档
        repeat (2) @(posedge clk);

        axi_read(12'h0BC,v); chk32(v,32'd1,"CE_START capability");
        axi_write(12'h0BC,32'd1,4'hF);
        axi_write(12'h0BC,32'd0,4'hF);
        axi_write(12'h0BC,32'd1,4'hE);
        repeat(3) @(posedge clk);
        chk32(ce_start_count,32'd1,"CE_START exactly one pulse; bit and strobe respected");
        // ---------------- 汇总 ----------------
        repeat (4) @(posedge clk);
        if (err_cnt == 0)
            $display("=== pl_ctrl_regs_tb : PASS ===  (比对点 %0d 个)", chk_cnt);
        else
            $display("=== pl_ctrl_regs_tb : FAIL ===  错误 %0d 处 / 比对点 %0d 个",
                     err_cnt, chk_cnt);
        $finish;
    end

    initial begin
        #2000000;
        $display("[ERR] 超时");
        $display("=== pl_ctrl_regs_tb : FAIL (timeout) ===");
        $finish;
    end

endmodule

`default_nettype wire
