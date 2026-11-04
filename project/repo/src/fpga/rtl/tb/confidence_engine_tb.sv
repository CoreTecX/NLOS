// ============================================================================
// confidence_engine_tb.sv  ·  confidence_engine 自检测试平台
// ---------------------------------------------------------------------------
// ★★★ 仿真状态声明（必读）★★★
//   本测试平台**没有在本机运行过**（本机未安装 iverilog / Verilator / Vivado
//   xsim，无法编译或仿真）。本文件不含任何仿真结果或通过记录。
//
//   运行方式（本文件位于 src/fpga/rtl/tb/）：
//     iverilog -g2012 -o confidence_engine_tb.vvp confidence_engine_tb.sv ../confidence_engine.sv
//     vvp confidence_engine_tb.vvp
//   预期输出：末尾打印 "=== confidence_engine_tb : PASS ==="。
//
// ---------------------------------------------------------------------------
// 覆盖的内容（HID_N=16，便于把像素可信度图整张比对完）
// ---------------------------------------------------------------------------
//   T1 软分数与三种融合方式（w_sel=0 取最小 / 1 连乘 / 2 等权平均）
//      期望值由 TB 用**与 RTL 完全相同的整数公式**算出（floor 除法 + 「2 倍阈值
//      满量程」口径），因此是**严格相等**判据。
//
//   T2 五种强制灰屏条件逐条注入，核对 gray_reason 的**对应位**与 global_conf=0：
//        bit0 饱和比例超标 / bit1 SNR 不足 / bit2 margin 不足 /
//        bit3 ρ 超限 / bit4 几何哈希不一致。
//      另做一条「两项同时超标」的用例，验证 gray_reason 是**位图**而不是优先级编码。
//
//   T3 像素可信度图：16 个像素逐值核对 (mask * global_conf) >> 16。
//
//   T4 low_conf（STATUS bit3）：把判定阈值抬到当前可信度之上，核对 low_conf=1。
//
//   T5 数据完整性硬闸门（A5，2026-10-02）：ovf_sticky / ovf_diff / proto_err 三条
//       各自单独置位都能触发 force_gray 并点亮对应位（bit5/6/7）、global_conf 归零；
//       三条同时置位仍触发（位图全置）；与质量项（rho）并存时位图共存；
//       三条全清零时不误触发且软分数保留。
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module confidence_engine_tb;

    localparam int CONF_W = 16;
    localparam int MASK_W = 8;
    localparam int HID_N  = 16;     // TB 用 16 个隐藏像素
    localparam int DIV_NW = 48;
    localparam int MAW    = 4;      // clog2(16)

    logic clk = 1'b0;
    always #5 clk = ~clk;
    logic rst_n = 1'b0;

    int err_cnt = 0;
    int chk_cnt = 0;

    // ------------------------------------------------------------------
    // DUT
    // ------------------------------------------------------------------
    logic [CONF_W-1:0] snr_i, margin_i, rho_i, sat_i;
    logic              geom_ok;
    // A5（2026-10-02）：数据完整性标志（接进硬闸门，最高优先）
    logic              raw_ovf_i, ovf_diff_i, proto_err_i;
    logic [CONF_W-1:0] snr_min_i, margin_min_i, rho_max_i, sat_max_i, low_conf_i;
    logic [1:0]        w_sel_i;
    logic              start_i;
    logic [MAW-1:0]    mask_raddr_w;      // 注意：DUT 该端口为 [MAW-1:0]
    logic [MASK_W-1:0] mask_q;
    logic [CONF_W-1:0] global_conf_o;
    logic              low_conf_o, force_gray_o;
    logic [7:0]        gray_reason_o;
    logic [MAW:0]      cmap_addr;
    logic [MASK_W-1:0] cmap_data;
    logic              cmap_we;
    logic              done_o, busy_o, cfg_err_o;

    // 掩膜 BRAM 模型：1 拍延迟
    logic [MASK_W-1:0] mask_mem [0:HID_N-1];
    always_ff @(posedge clk) mask_q <= mask_mem[mask_raddr_w];

    confidence_engine #(
        .CONF_W(CONF_W), .MASK_W(MASK_W), .HID_N(HID_N), .DIV_NW(DIV_NW)
    ) dut (
        .clk(clk), .rst_n(rst_n), .start(start_i),
        .snr_q016(snr_i), .margin_q016(margin_i), .rho_q016(rho_i),
        .sat_ratio_q016(sat_i), .geom_hash_ok(geom_ok),
        .raw_ovf_sticky(raw_ovf_i), .ovf_diff(ovf_diff_i), .proto_err(proto_err_i),
        .snr_min_q016(snr_min_i), .margin_min_q016(margin_min_i),
        .rho_max_q016(rho_max_i), .sat_max_q016(sat_max_i),
        .low_conf_q016(low_conf_i), .w_sel(w_sel_i),
        .mask_raddr(mask_raddr_w), .mask_rdata(mask_q),
        .global_conf(global_conf_o), .low_conf(low_conf_o),
        .force_gray(force_gray_o), .gray_reason(gray_reason_o),
        .conf_map_waddr(cmap_addr), .conf_map_wdata(cmap_data),
        .conf_map_we(cmap_we), .done(done_o), .busy(busy_o), .cfg_err(cfg_err_o)
    );

    // ------------------------------------------------------------------
    // 与 RTL 同口径的参考模型
    // ------------------------------------------------------------------
    // 正向分数：min(0xFFFF, floor((meas << 16) / (2*thr)))
    function automatic int score_pos(input int meas, input int thr);
        longint n;
        if (thr == 0) return 0;
        n = (longint'(meas) << 16) / (2 * longint'(thr));
        if (n > 65535) n = 65535;
        return int'(n);
    endfunction

    // 反向分数（越小越好）：0xFFFF - min(0xFFFF, floor((meas << 16)/(2*thr)))
    function automatic int score_neg(input int meas, input int thr);
        int r;
        if (thr == 0) return 0;
        r = score_pos(meas, thr);
        return (65535 - r);
    endfunction

    function automatic int fuse_min4(input int a, input int b, input int c, input int d);
        int m;
        m = a;
        if (b < m) m = b;
        if (c < m) m = c;
        if (d < m) m = d;
        return m;
    endfunction

    // 连乘：((s0*s1)>>16) 与 ((s2*s3)>>16) 相乘后再 >>16，逐位镜像 RTL
    function automatic int fuse_prod4(input int a, input int b, input int c, input int d);
        longint p01, p23, pall;
        p01 = longint'(a) * longint'(b);
        p23 = longint'(c) * longint'(d);
        pall = (p01 >> 16) * (p23 >> 16);
        return int'(pall >> 16);
    endfunction

    function automatic int fuse_avg4(input int a, input int b, input int c, input int d);
        return ((a + b + c + d) >> 2);
    endfunction

    // ------------------------------------------------------------------
    // 期望值寄存器
    // ------------------------------------------------------------------
    logic [CONF_W-1:0] exp_conf_q016;
    logic              exp_gray;
    logic [7:0]        exp_reason;

    task automatic calc_expect(input int mode);
        int s0, s1, s2, s3, f;
        s0 = score_pos(snr_i,     snr_min_i);
        s1 = score_pos(margin_i,  margin_min_i);
        s2 = score_neg(rho_i,     rho_max_i);
        s3 = score_neg(sat_i,     sat_max_i);
        case (mode)
            0: f = fuse_min4(s0, s1, s2, s3);
            1: f = fuse_prod4(s0, s1, s2, s3);
            default: f = fuse_avg4(s0, s1, s2, s3);
        endcase
        exp_reason = 8'b0;
        // 数据完整性（bit[7:5]，优先级最高）
        if (raw_ovf_i)              exp_reason[5] = 1'b1;
        if (ovf_diff_i)             exp_reason[6] = 1'b1;
        if (proto_err_i)            exp_reason[7] = 1'b1;
        // 质量指标（原有五项，位位置不变）
        if (sat_i  > sat_max_i)     exp_reason[0] = 1'b1;
        if (snr_i  < snr_min_i)     exp_reason[1] = 1'b1;
        if (margin_i < margin_min_i) exp_reason[2] = 1'b1;
        if (rho_i  > rho_max_i)     exp_reason[3] = 1'b1;
        if (!geom_ok)               exp_reason[4] = 1'b1;
        exp_gray = (exp_reason != 8'b0);
        exp_conf_q016 = exp_gray ? 16'd0 : CONF_W'(f);
    endtask

    // ------------------------------------------------------------------
    // 一次运行
    // ------------------------------------------------------------------
    task automatic run_engine(input int mode);
        w_sel_i = 2'(mode);
        start_i = 1'b1;
        @(posedge clk);
        start_i = 1'b0;
        wait (done_o === 1'b1);
        @(posedge clk);
    endtask

    // 收集像素可信度图
    logic [MASK_W-1:0] cmap_got [0:HID_N-1];
    int  cmap_cnt = 0;

    always @(posedge clk) begin
        if (cmap_we) begin
            if (cmap_addr < (MAW+1)'(HID_N)) begin
                cmap_got[cmap_addr[MAW-1:0]] <= cmap_data;
            end
            cmap_cnt++;
        end
    end

    // ------------------------------------------------------------------
    // 主流程
    // ------------------------------------------------------------------
    int i, m;

    initial begin
        start_i = 1'b0;
        w_sel_i = 2'd0;
        geom_ok = 1'b1;
        raw_ovf_i = 1'b0; ovf_diff_i = 1'b0; proto_err_i = 1'b0;

        // 基准（全部通过硬闸门，且软分数不饱和 -> 便于逐值核对）
        snr_i        = 16'h4000;
        snr_min_i    = 16'h4000;   // 2*thr = 0x8000 -> 比值 0.5
        margin_i     = 16'h4000;
        margin_min_i = 16'h4000;   // 0.5
        rho_i        = 16'h1000;
        rho_max_i    = 16'h4000;   // 比值 0.25 -> 分数 0.75
        sat_i        = 16'h0020;
        sat_max_i    = 16'h0042;   // 0.1% 的规范 Q0.16 = 66（0x0042）；
                                   //   0x20/(2*0x42)=0.242 -> 分数约 0.758（未饱和）
                                   //   注：原写 0x0066（102，约 0.1556%）是沿用旧文档的
                                   //   错口径，见 sim/results/rtl_four_issues_20260930.md。
        low_conf_i   = 16'h1000;   // 远低于 0.5，故 low_conf 应为 0
        for (i = 0; i < HID_N; i++) mask_mem[i] = MASK_W'(i * 17);   // 0,17,...,255

        repeat (8) @(posedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);

        if (cfg_err_o !== 1'b0) begin $display("[ERR] cfg_err 意外置起"); err_cnt++; end

        // ================= T1：三种融合方式 =================
        for (m = 0; m < 3; m++) begin
            calc_expect(m);
            cmap_cnt = 0;
            run_engine(m);
            $display("[INFO] T1 w_sel=%0d 期望 conf=0x%04h 实得 conf=0x%04h",
                     m, exp_conf_q016, global_conf_o);
            chk_cnt++;
            if (global_conf_o !== exp_conf_q016) begin
                $display("[ERR] T1 w_sel=%0d conf 不符 got=0x%04h exp=0x%04h",
                         m, global_conf_o, exp_conf_q016);
                err_cnt++;
            end
            if (force_gray_o !== 1'b0) begin
                $display("[ERR] T1 w_sel=%0d 不应灰屏", m); err_cnt++;
            end
            if (gray_reason_o !== 8'b0) begin
                $display("[ERR] T1 w_sel=%0d gray_reason 应为 0，实得 %b",
                         m, gray_reason_o); err_cnt++;
            end
            if (low_conf_o !== 1'b0) begin
                $display("[ERR] T1 w_sel=%0d low_conf 应为 0", m); err_cnt++;
            end
            if (cmap_cnt != HID_N) begin
                $display("[ERR] T1 w_sel=%0d 像素图写入次数 %0d，期望 %0d",
                         m, cmap_cnt, HID_N); err_cnt++;
            end
            // T3：像素可信度图逐值核对
            for (i = 0; i < HID_N; i++) begin
                chk_cnt++;
                if (cmap_got[i] !== MASK_W'((mask_mem[i] * exp_conf_q016) >> 16)) begin
                    $display("[ERR] T3 像素 %0d 可信度不符 got=%0d exp=%0d",
                             i, cmap_got[i], (mask_mem[i]*exp_conf_q016)>>16);
                    err_cnt++;
                end
            end
        end

        // ================= T2：五种灰屏条件逐条注入 =================
        // 每条用例只破坏一个条件，核对「对应位 + 全局可信度归零」
        $display("[INFO] T2 逐条注入灰屏条件");
        // bit0 饱和比例超标
        sat_i = 16'h0067;                             // > sat_max(0x42)
        calc_expect(0); run_engine(0);
        chk_cnt++;
        if ((force_gray_o !== 1'b1) || (gray_reason_o[0] !== 1'b1) ||
            (gray_reason_o[4:1] !== 4'b0) || (global_conf_o !== '0)) begin
            $display("[ERR] T2 bit0(饱和) 判据不符 gray=%b conf=0x%04h",
                     gray_reason_o, global_conf_o);
            err_cnt++;
        end
        sat_i = 16'h0020;

        // bit1 SNR 不足
        snr_i = 16'h3FFF;                             // < snr_min(0x4000)
        calc_expect(0); run_engine(0);
        chk_cnt++;
        if ((force_gray_o !== 1'b1) || (gray_reason_o[1] !== 1'b1) ||
            (gray_reason_o !== 5'b00010) || (global_conf_o !== '0)) begin
            $display("[ERR] T2 bit1(SNR) 判据不符 gray=%b", gray_reason_o);
            err_cnt++;
        end
        snr_i = 16'h4000;

        // bit2 margin 不足
        margin_i = 16'h3FFF;
        calc_expect(0); run_engine(0);
        chk_cnt++;
        if ((force_gray_o !== 1'b1) || (gray_reason_o !== 5'b00100) ||
            (global_conf_o !== '0)) begin
            $display("[ERR] T2 bit2(margin) 判据不符 gray=%b", gray_reason_o);
            err_cnt++;
        end
        margin_i = 16'h4000;

        // bit3 rho 超限
        rho_i = 16'h4001;                             // > rho_max(0x4000)
        calc_expect(0); run_engine(0);
        chk_cnt++;
        if ((force_gray_o !== 1'b1) || (gray_reason_o !== 5'b01000) ||
            (global_conf_o !== '0)) begin
            $display("[ERR] T2 bit3(rho) 判据不符 gray=%b", gray_reason_o);
            err_cnt++;
        end
        rho_i = 16'h1000;

        // bit4 几何哈希不一致
        geom_ok = 1'b0;
        calc_expect(0); run_engine(0);
        chk_cnt++;
        if ((force_gray_o !== 1'b1) || (gray_reason_o !== 5'b10000) ||
            (global_conf_o !== '0)) begin
            $display("[ERR] T2 bit4(几何哈希) 判据不符 gray=%b", gray_reason_o);
            err_cnt++;
        end
        geom_ok = 1'b1;

        // 组合：sat + rho 同时超标 -> 位图必须同时置两位（不是优先级编码）
        sat_i = 16'h0067;
        rho_i = 16'h4001;
        calc_expect(0); run_engine(0);
        chk_cnt++;
        if (gray_reason_o !== 5'b01001) begin
            $display("[ERR] T2 组合项 gray_reason 应为 01001，实得 %b", gray_reason_o);
            err_cnt++;
        end
        sat_i = 16'h0020;
        rho_i = 16'h1000;

        // ================= T4：low_conf =================
        // 把判定阈值抬到当前可信度之上（w_sel=0 时 conf = 0x8000）
        low_conf_i = 16'h9000;
        calc_expect(0); run_engine(0);
        chk_cnt++;
        if (low_conf_o !== 1'b1) begin
            $display("[ERR] T4 low_conf 应为 1（阈值 0x9000 > conf 0x%04h）", exp_conf_q016);
            err_cnt++;
        end
        low_conf_i = 16'h1000;

        // ================= T5：数据完整性硬闸门（A5，最高优先）=================
        // 覆盖：三条各自单独置位都灰屏 / 同时置位仍灰屏（位图）/ 与质量项并存 /
        //       三条全清零时不误触发且软分数保留。
        $display("[INFO] T5 数据完整性 ovf/proto_err 接入 force_gray（最高优先）");

        // T5a raw ovf_sticky 单独置位 -> bit5，global_conf 归零
        raw_ovf_i = 1'b1;
        calc_expect(0); run_engine(0);
        chk_cnt++;
        if ((force_gray_o !== 1'b1) || (gray_reason_o !== 8'b0010_0000) ||
            (global_conf_o !== '0)) begin
            $display("[ERR] T5a raw_ovf 判据不符 gray=%b conf=0x%04h",
                     gray_reason_o, global_conf_o);
            err_cnt++;
        end
        raw_ovf_i = 1'b0;

        // T5b ovf_diff 单独置位 -> bit6
        ovf_diff_i = 1'b1;
        calc_expect(0); run_engine(0);
        chk_cnt++;
        if ((force_gray_o !== 1'b1) || (gray_reason_o !== 8'b0100_0000) ||
            (global_conf_o !== '0)) begin
            $display("[ERR] T5b ovf_diff 判据不符 gray=%b conf=0x%04h",
                     gray_reason_o, global_conf_o);
            err_cnt++;
        end
        ovf_diff_i = 1'b0;

        // T5c proto_err 单独置位 -> bit7
        proto_err_i = 1'b1;
        calc_expect(0); run_engine(0);
        chk_cnt++;
        if ((force_gray_o !== 1'b1) || (gray_reason_o !== 8'b1000_0000) ||
            (global_conf_o !== '0)) begin
            $display("[ERR] T5c proto_err 判据不符 gray=%b conf=0x%04h",
                     gray_reason_o, global_conf_o);
            err_cnt++;
        end
        proto_err_i = 1'b0;

        // T5d 三条同时置位 -> bit[7:5] 全置（位图，不是优先级编码）
        raw_ovf_i = 1'b1; ovf_diff_i = 1'b1; proto_err_i = 1'b1;
        calc_expect(0); run_engine(0);
        chk_cnt++;
        if ((force_gray_o !== 1'b1) || (gray_reason_o !== 8'b1110_0000) ||
            (global_conf_o !== '0)) begin
            $display("[ERR] T5d 三完整性同时 判据不符 gray=%b conf=0x%04h",
                     gray_reason_o, global_conf_o);
            err_cnt++;
        end

        // T5e 完整性 + 质量项（rho 超标）并存 -> 位图含 bit7/6/5/3，完整性仍最高优先
        rho_i = 16'h4001;                       // > rho_max(0x4000)
        calc_expect(0); run_engine(0);
        chk_cnt++;
        if ((force_gray_o !== 1'b1) || (gray_reason_o !== 8'b1110_1000) ||
            (global_conf_o !== '0)) begin
            $display("[ERR] T5e 完整性+rho 判据不符 gray=%b conf=0x%04h",
                     gray_reason_o, global_conf_o);
            err_cnt++;
        end
        rho_i = 16'h1000;

        // T5f 三条全清零、其余条件全通过 -> 不误触发，软分数保留
        raw_ovf_i = 1'b0; ovf_diff_i = 1'b0; proto_err_i = 1'b0;
        calc_expect(0); run_engine(0);
        chk_cnt++;
        if ((force_gray_o !== 1'b0) || (gray_reason_o !== 8'b0) ||
            (global_conf_o !== exp_conf_q016) || (exp_conf_q016 === '0)) begin
            $display("[ERR] T5f 全清零误触发 gray=%b conf=0x%04h exp=0x%04h",
                     gray_reason_o, global_conf_o, exp_conf_q016);
            err_cnt++;
        end

        // ================= 汇总 =================
        if (err_cnt == 0)
            $display("=== confidence_engine_tb : PASS ===  (比对点 %0d 个)", chk_cnt);
        else
            $display("=== confidence_engine_tb : FAIL ===  错误 %0d 处 / 比对点 %0d 个",
                     err_cnt, chk_cnt);
        $finish;
    end

    initial begin
        #2000000;
        $display("[ERR] 超时");
        $display("=== confidence_engine_tb : FAIL (timeout) ===");
        $finish;
    end

endmodule

`default_nettype wire
