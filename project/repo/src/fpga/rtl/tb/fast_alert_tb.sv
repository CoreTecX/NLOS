// ============================================================================
// fast_alert_tb.sv  ·  fast_alert 自检测试平台
// ---------------------------------------------------------------------------
// ★★★ 仿真状态声明（必读）★★★
//   本测试平台**没有在本机运行过**（本机未安装 iverilog / Verilator / Vivado
//   xsim，无法编译或仿真）。本文件不含任何仿真结果、误报率或时延数据。
//
//   运行方式（本文件位于 src/fpga/rtl/tb/）：
//     iverilog -g2012 -o fast_alert_tb.vvp fast_alert_tb.sv ../fast_alert.sv
//     vvp fast_alert_tb.vvp
//   预期输出：末尾打印 "=== fast_alert_tb : PASS ==="。
//
// ---------------------------------------------------------------------------
// 覆盖的内容（K=4, WALL_N=8 -> 差分流 7 行 x 8 列 x 3 通道 = 168 拍）
// ---------------------------------------------------------------------------
//   T1 E 的逐值精确比对
//        E = Σ_k floor( (c_k - μ_k)^2 / (v_k + ε) )   （整数 floor 除法，见 RTL 决策 1）
//      期望值由 TB 用**同一套整数运算**算出，因此是**严格相等**判据，不是容差。
//      同时校验 e_over / e_ovf / zero_den 与 alert_score。
//
//   T2 「连续两帧」判据
//      同一份超阈输入连跑两遍：第 1 遍 alert_event 必须为 0，第 2 遍必须为 1。
//      再换一份低于阈值的方差不重复，验证 alert_event 能回落（不会卡在 1）。
//
//   T3 质心与方向
//      差分流能量集中在 col=6，核对 alert_centroid_x 与**同口径**（先 >>16 归一化
//      再整除）的期望值严格相等；随后把能量移到 col=7，检查 direction=+1。
//      另做一次真实质心的旁证：|centroid/256 - 6| <= 1 个块列。
//
//   T4 低能量冻结（易错点 2）
//      mom0 低于 cfg_min_energy 时，alert_centroid_valid 必须为 0，
//      且质心保持上一次的值而不是假报 0。
//
//   2026-09-30 补齐（上板前盘点 sim/results/preboard_port_status_20260930.md §一 1.1
//   与 xsim_first_run_20260929.md §4.3/§8 记「比对点仅 6 个、溢出/除零/proto_err 未构造」）：
//   T5 E 累加溢出（e_ovf）
//      用**第二个实例** dut_o（E_W 收到 32 位）让 acc_ovf 真的置起，断言
//      e_ovf 粘滞为 1 且 alert_score 等于**饱和**值 2^E_W-1。
//      为什么必须收窄 E_W：生产位宽 E_W=48 下单次商 <= 2^34、合法 cfg_k <= K_MAX 时
//      总和 <= 2^37 << 2^48，e_ovf 结构上不可达（本 TB 在 T5 里打印该上界）。
//      [2026-09-30] RTL 已把溢出行为由「取低位回绕」改为「饱和」（与端口注释
//      「E 累加饱和」及 rtl/README §4「禁止回绕截断」一致），本用例随之断言饱和值。
//   T6 除零防护（den==0 / zero_den）
//      把 bg_var 全置 0（v_k=0，最坏情形）而 cfg_eps=1：x_den = v+ε = 1，
//      E 必须按 den=ε 精确算出，zero_den 必须**保持 0**。
//   T7 proto_err 注入
//      第 2 个系数的 tindex 送错值：proto_err 必须置起且粘滞（后续正常帧不回落）。
//   T8 cfg_eps=0（除零配置）被拒
//      cfg_eps=0 时 cfg_err 立即为 1；该帧走完流程但 e_acc/alert_score 归 0。
//      这条同时说明 **E 通路的** zero_den 不可达：cfg_eps>=1 使 den=v+ε>=1，
//      cfg_eps==0 又在 E pass 之前短路到 ST_PUB（见报告）。质心通路的守卫
//      n0==0 是可达的（2026-09-30 RTL 已统一：两条通路的守卫都置 zero_den），
//      本 TB 的 T3/T4 里 n0 恒非零，故全程 zero_den 保持 0。
//   T9 cfg_persist_en=0（单帧判据）
//      persist=0 时首帧超阈即报 alert_event=1（对照 T2 的两帧判据）。
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module fast_alert_tb;

    localparam int K_MAX     = 4;
    localparam int WALL_N    = 8;
    localparam int DIFF_BITS = 16;      // TB 用小位宽，方便核对
    localparam int COEF_W    = 16;
    localparam int VAR_W     = 16;
    localparam int E_W       = 48;
    localparam int DIV_NW    = 48;
    localparam int CENT_FRAC = 8;
    // T5 专用：把 E 累加器收窄到 32 位，让 acc_ovf 在 K=4 的合法规模下可达。
    localparam int EO_KMAX   = 4;
    localparam int E_W_OVF   = 32;
    localparam int NROW      = WALL_N-1;                 // 差分流 7 行
    localparam int NDIFF     = NROW*WALL_N*3;            // 168

    logic clk = 1'b0;
    always #5 clk = ~clk;
    logic rst_n = 1'b0;

    int err_cnt = 0;
    int chk_cnt = 0;

    // ------------------------------------------------------------------
    // 背景统计（TB 侧 BRAM 模型：1 拍延迟，与真 BRAM 一致）
    // ------------------------------------------------------------------
    logic [COEF_W-1:0] mu_mem  [0:K_MAX-1];
    logic [VAR_W-1:0]  var_mem [0:K_MAX-1];
    logic [COEF_W-1:0] mu_q, var_q;
    logic [1:0]        bg_raddr_w;

    always_ff @(posedge clk) begin
        mu_q  <= mu_mem[bg_raddr_w];
        var_q <= var_mem[bg_raddr_w];
    end

    // ------------------------------------------------------------------
    // 激励
    // ------------------------------------------------------------------
    logic [COEF_W-1:0]      s_coef_data;
    logic [1:0]             s_coef_idx;
    logic                   s_coef_valid, s_coef_ready, s_coef_last;
    logic signed [DIFF_BITS-1:0] s_diff_data;
    logic                   s_diff_valid, s_diff_ready, s_diff_last, s_diff_user;
    logic [2:0]             s_diff_col;      // COLW = clog2(8) = 3

    logic [E_W-1:0]         cfg_thresh, cfg_min_e;
    logic [15:0]            cfg_k, cfg_dead;
    logic [VAR_W-1:0]       cfg_eps;
    logic                   cfg_persist;

    logic                   alert_event, alert_cvalid, frame_done, proto_err, zero_den, e_ovf, cfg_err;
    logic signed [1:0]      alert_dir;
    logic [E_W-1:0]         alert_score;
    logic [15:0]            alert_cx;
    logic signed [16:0]     alert_dx;

    // 注意：bg 读接口由 DUT 输出地址（bg_raddr_w），TB 用真 BRAM 的 1 拍延迟模型回数据。

    fast_alert #(
        .K_MAX(K_MAX), .WALL_N(WALL_N), .DIFF_BITS(DIFF_BITS),
        .COEF_W(COEF_W), .VAR_W(VAR_W), .E_W(E_W), .DIV_NW(DIV_NW), .CENT_FRAC(8)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .s_coef_tdata(s_coef_data), .s_coef_tindex(s_coef_idx),
        .s_coef_tvalid(s_coef_valid), .s_coef_tready(s_coef_ready),
        .s_coef_tlast(s_coef_last),
        .bg_raddr(bg_raddr_w), .bg_mu(mu_q), .bg_var(var_q),
        .s_diff_tdata(s_diff_data), .s_diff_tvalid(s_diff_valid),
        .s_diff_tready(s_diff_ready), .s_diff_tlast(s_diff_last),
        .s_diff_tuser(s_diff_user), .s_diff_tcol(s_diff_col),
        .cfg_e_thresh(cfg_thresh), .cfg_k(cfg_k), .cfg_eps(cfg_eps),
        .cfg_persist_en(cfg_persist), .cfg_dir_deadband(cfg_dead),
        .cfg_min_energy(cfg_min_e),
        .alert_event(alert_event), .alert_direction(alert_dir),
        .alert_score(alert_score), .alert_centroid_x(alert_cx),
        .alert_delta_x(alert_dx), .alert_centroid_valid(alert_cvalid),
        .frame_done(frame_done), .proto_err(proto_err), .zero_den(zero_den),
        .e_ovf(e_ovf), .cfg_err(cfg_err)
    );

    // ==================================================================
    // DUT 3（T5 专用）：E 累加溢出路径
    //   E_W 收窄到 E_W_OVF = 32，使 acc_ovf 在 K=4 下可达。差分流不接
    //   （tvalid 恒 0），故 cent_pending 永不置起，每帧只走 E pass。
    // ==================================================================
    logic [COEF_W-1:0] o_coef_data;
    logic [1:0]        o_coef_idx;
    logic              o_coef_valid, o_coef_ready, o_coef_last;
    logic [1:0]        o_bg_raddr;
    logic [COEF_W-1:0] o_mu_mem  [0:EO_KMAX-1];
    logic [VAR_W-1:0]  o_var_mem [0:EO_KMAX-1];
    logic [COEF_W-1:0] o_mu_q, o_var_q;
    logic signed [DIFF_BITS-1:0] o_diff_data;
    logic              o_diff_valid, o_diff_last, o_diff_user;
    logic [2:0]        o_diff_col;
    logic [E_W_OVF-1:0] o_thresh, o_min_e, o_score;
    logic [15:0]       o_k, o_dead;
    logic [VAR_W-1:0]  o_eps;
    logic              o_persist;
    logic              o_event, o_cvalid, o_fdone, o_proto, o_zero, o_ovf, o_cfgerr;
    logic signed [1:0] o_dir;
    logic [15:0]       o_cx;
    logic signed [16:0] o_dx;

    assign o_diff_data  = '0;
    assign o_diff_valid = 1'b0;
    assign o_diff_last  = 1'b0;
    assign o_diff_user  = 1'b0;
    assign o_diff_col   = '0;

    always_ff @(posedge clk) begin
        o_mu_q  <= o_mu_mem[o_bg_raddr];
        o_var_q <= o_var_mem[o_bg_raddr];
    end

    fast_alert #(
        .K_MAX(EO_KMAX), .WALL_N(WALL_N), .DIFF_BITS(DIFF_BITS),
        .COEF_W(COEF_W), .VAR_W(VAR_W), .E_W(E_W_OVF), .DIV_NW(DIV_NW), .CENT_FRAC(8)
    ) dut_o (
        .clk(clk), .rst_n(rst_n),
        .s_coef_tdata(o_coef_data), .s_coef_tindex(o_coef_idx),
        .s_coef_tvalid(o_coef_valid), .s_coef_tready(o_coef_ready),
        .s_coef_tlast(o_coef_last),
        .bg_raddr(o_bg_raddr), .bg_mu(o_mu_q), .bg_var(o_var_q),
        .s_diff_tdata(o_diff_data), .s_diff_tvalid(o_diff_valid),
        .s_diff_tlast(o_diff_last),
        .s_diff_tuser(o_diff_user), .s_diff_tcol(o_diff_col),
        .cfg_e_thresh(o_thresh), .cfg_k(o_k), .cfg_eps(o_eps),
        .cfg_persist_en(o_persist), .cfg_dir_deadband(o_dead),
        .cfg_min_energy(o_min_e),
        .alert_event(o_event), .alert_direction(o_dir), .alert_score(o_score),
        .alert_centroid_x(o_cx), .alert_delta_x(o_dx),
        .alert_centroid_valid(o_cvalid), .frame_done(o_fdone),
        .proto_err(o_proto), .zero_den(o_zero), .e_ovf(o_ovf), .cfg_err(o_cfgerr)
    );

    // ------------------------------------------------------------------
    // 发送系数帧（K 拍）
    // ------------------------------------------------------------------
    task automatic send_coef(input logic [COEF_W-1:0] c0,
                             input logic [COEF_W-1:0] c1,
                             input logic [COEF_W-1:0] c2,
                             input logic [COEF_W-1:0] c3);
        logic [COEF_W-1:0] cc [0:3];
        int k;
        cc[0] = c0; cc[1] = c1; cc[2] = c2; cc[3] = c3;
        for (k = 0; k < K_MAX; k++) begin
            s_coef_data  = cc[k];
            s_coef_idx   = 2'(k);
            s_coef_last  = (k == K_MAX-1);
            s_coef_valid = 1'b1;
            @(posedge clk);            // s_coef_tready 恒 1，直接连发即可
        end
        s_coef_valid = 1'b0;
        s_coef_last  = 1'b0;
    endtask

    // ------------------------------------------------------------------
    // 发送差分帧：能量集中在 col = ecol，幅度 MAG
    // ------------------------------------------------------------------
    localparam int MAG = 30000;
    task automatic send_diff(input int ecol);
        int r, c, ch;
        logic signed [DIFF_BITS-1:0] d;
        for (r = 0; r < NROW; r++) begin
            for (c = 0; c < WALL_N; c++) begin
                for (ch = 0; ch < 3; ch++) begin
                    d = (c == ecol) ? DIFF_BITS'(MAG) : DIFF_BITS'(0);
                    s_diff_data  = d;
                    s_diff_col   = 3'(c);
                    s_diff_last  = (c == WALL_N-1) && (ch == 2);
                    s_diff_user  = (r == 0) && (c == 0) && (ch == 0);
                    s_diff_valid = 1'b1;
                    @(posedge clk);        // s_diff_tready 恒 1
                end
            end
        end
        s_diff_valid = 1'b0;
        s_diff_last  = 1'b0;
        s_diff_user  = 1'b0;
    endtask

    // ------------------------------------------------------------------
    // T5：向 dut_o 发送系数帧（K=EO_KMAX 拍，tindex 顺序正确）
    // ------------------------------------------------------------------
    task automatic send_coef_o(input logic [COEF_W-1:0] c0,
                               input logic [COEF_W-1:0] c1,
                               input logic [COEF_W-1:0] c2,
                               input logic [COEF_W-1:0] c3);
        logic [COEF_W-1:0] cc [0:3];
        int k;
        cc[0] = c0; cc[1] = c1; cc[2] = c2; cc[3] = c3;
        for (k = 0; k < EO_KMAX; k++) begin
            o_coef_data  = cc[k];
            o_coef_idx   = 2'(k);
            o_coef_last  = (k == EO_KMAX-1);
            o_coef_valid = 1'b1;
            @(posedge clk);
        end
        o_coef_valid = 1'b0;
        o_coef_last  = 1'b0;
    endtask

    // ------------------------------------------------------------------
    // T7：向主 DUT 发送**索引故意送错**的系数帧（第 2 个 k 送 tindex=3）
    // ------------------------------------------------------------------
    task automatic send_coef_badidx();
        logic [COEF_W-1:0] cc [0:3];
        logic [1:0]        ii [0:3];
        int k;
        cc[0] = 16'd1000; cc[1] = 16'd2000; cc[2] = 16'shFA24; cc[3] = 16'd3000;
        ii[0] = 2'd0;     ii[1] = 2'd1;     ii[2] = 2'd3;      ii[3] = 2'd3;
        for (k = 0; k < K_MAX; k++) begin
            s_coef_data  = cc[k];
            s_coef_idx   = ii[k];              // 第 2 拍（k=2）与 coef_cnt=2 不符
            s_coef_last  = (k == K_MAX-1);
            s_coef_valid = 1'b1;
            @(posedge clk);
        end
        s_coef_valid = 1'b0;
        s_coef_last  = 1'b0;
    endtask

    // ------------------------------------------------------------------
    // 期望值计算（与 RTL 同口径：整数 floor）
    // ------------------------------------------------------------------
    function automatic int exp_E(input int c0, input int m0, input int v0,
                                 input int c1, input int m1, input int v1,
                                 input int c2, input int m2, input int v2,
                                 input int c3, input int m3, input int v3,
                                 input int eps);
        int d, s;
        s = 0;
        d = c0-m0; s = s + ((d*d) / (v0+eps));
        d = c1-m1; s = s + ((d*d) / (v1+eps));
        d = c2-m2; s = s + ((d*d) / (v2+eps));
        d = c3-m3; s = s + ((d*d) / (v3+eps));
        exp_E = s;
    endfunction

    // 质心期望：与 RTL 完全同口径（先 >>16 再整除，参数 M0/M1 为原始矩）
    function automatic int exp_cent(input longint m0, input longint m1);
        longint n0, n1;
        n0 = m0 >> 16;
        n1 = m1 >> 16;
        if (n0 == 0) exp_cent = 0;
        else         exp_cent = (n1 << 8) / n0;
    endfunction

    // ------------------------------------------------------------------
    // 主流程
    // ------------------------------------------------------------------
    int e_exp;
    int cx_exp;
    longint mom0, mom1;
    int i;
    longint e_ovf_true;                    // T5：未截断的 E（> 2^32）
    logic [E_W_OVF-1:0] e_ovf_sat;         // T5：饱和后的期望码字（全 1 = 2^E_W_OVF-1）

    initial begin
        // 初值
        s_coef_valid = 1'b0; s_coef_last = 1'b0; s_coef_data = '0; s_coef_idx = '0;
        s_diff_valid = 1'b0; s_diff_last = 1'b0; s_diff_user = 1'b0;
        s_diff_data  = '0;   s_diff_col  = '0;
        cfg_k       = 16'(K_MAX);
        cfg_eps     = 16'd1;                 // ε = 1
        cfg_persist = 1'b1;                  // §6.6：需要连续两帧
        cfg_dead    = 16'd32;                // 死区 = 32/256 = 1/8 个块列
        cfg_thresh  = 48'd60000;
        cfg_min_e   = 48'd1;
        for (i = 0; i < K_MAX; i++) begin
            mu_mem[i]  = '0;
            var_mem[i] = 16'd100;
        end

        // dut_o 初值
        o_coef_valid = 1'b0; o_coef_last = 1'b0; o_coef_data = '0; o_coef_idx = '0;
        o_k       = 16'(EO_KMAX);
        o_eps     = 16'd1;
        o_persist = 1'b1;
        o_dead    = 16'd32;
        o_thresh  = {E_W_OVF{1'b1}};
        o_min_e   = '0;
        for (i = 0; i < EO_KMAX; i++) begin
            o_mu_mem[i]  = '0;
            o_var_mem[i] = 16'd0;
        end

        repeat (8) @(posedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);

        if (cfg_err !== 1'b0) begin $display("[ERR] cfg_err 意外置起"); err_cnt++; end

        // ================= T1：E 的逐值精确比对 =================
        //   c = [1000, 2000, -1500, 3000]（16 bit 有符号码字）
        //   μ = [0, 500, -500, 1000]
        //   v = [100, 200, 250, 100], ε = 1
        //   d = [1000, 1500, -1000, 2000]
        //   E = floor(1e6/101)+floor(2.25e6/201)+floor(1e6/251)+floor(4e6/101)
        $display("[INFO] T1 E 逐值比对（整数 floor 除法）");
        mu_mem[0] = 16'd0;      var_mem[0] = 16'd100;
        mu_mem[1] = 16'd500;    var_mem[1] = 16'd200;
        mu_mem[2] = 16'shFE0C;  // -500
        var_mem[2] = 16'd250;
        mu_mem[3] = 16'd1000;   var_mem[3] = 16'd100;

        e_exp = exp_E(1000,0,100, 2000,500,200, -1500,-500,250, 3000,1000,100, 1);
        $display("[INFO] T1 期望 E = %0d（阈值 %0d）", e_exp, cfg_thresh);

        send_coef(16'd1000, 16'd2000, 16'shFA24, 16'd3000);   // -1500 = 0xFA24
        wait (frame_done === 1'b1);
        @(posedge clk);
        chk_cnt++;
        if (alert_score !== 48'(e_exp)) begin
            $display("[ERR] T1 E 不符：got=%0d exp=%0d", alert_score, e_exp); err_cnt++;
        end else begin
            $display("[INFO] T1 E 精确匹配 = %0d", alert_score);
        end
        if (e_ovf    !== 1'b0) begin $display("[ERR] T1 e_ovf 置起");    err_cnt++; end
        if (zero_den !== 1'b0) begin $display("[ERR] T1 zero_den 置起"); err_cnt++; end
        if (proto_err!== 1'b0) begin $display("[ERR] T1 proto_err 置起");err_cnt++; end

        // ================= T2：连续两帧判据 =================
        $display("[INFO] T2 连续两帧判据（第 1 遍应为 0，第 2 遍应为 1）");
        // 第 1 遍（E 已超阈，但历史为空）
        if (alert_event !== 1'b0) begin
            $display("[ERR] T2 第 1 遍 alert_event 应为 0（两帧判据尚未满足）"); err_cnt++;
        end
        send_coef(16'd1000, 16'd2000, 16'shFA24, 16'd3000);
        wait (frame_done === 1'b1);
        @(posedge clk);
        chk_cnt++;
        if (alert_event !== 1'b1) begin
            $display("[ERR] T2 第 2 遍 alert_event 应为 1"); err_cnt++;
        end else begin
            $display("[INFO] T2 第 2 遍 alert_event = 1（两帧持续成立）");
        end

        // 换一份大方差 -> E 低于阈值，验证能回落
        for (i = 0; i < K_MAX; i++) var_mem[i] = 16'd10000;
        send_coef(16'd1000, 16'd2000, 16'shFA24, 16'd3000);
        wait (frame_done === 1'b1);
        @(posedge clk);
        chk_cnt++;
        if (alert_event !== 1'b0) begin
            $display("[ERR] T2 低 E 时 alert_event 应回落为 0"); err_cnt++;
        end else begin
            $display("[INFO] T2 低 E 时 alert_event 回落为 0");
        end

        // 恢复小方差，供 T3 使用
        mu_mem[0] = 16'd0;      var_mem[0] = 16'd100;
        mu_mem[1] = 16'd500;    var_mem[1] = 16'd200;
        mu_mem[2] = 16'shFE0C;  var_mem[2] = 16'd250;
        mu_mem[3] = 16'd1000;   var_mem[3] = 16'd100;

        // ================= T3：质心与方向 =================
        $display("[INFO] T3 质心与方向：能量 col=6 -> col=7");
        send_diff(6);
        // 期望矩：|d| = MAG，每行每通道一次（列 6），共 NROW*3 个样本
        mom0 = longint'(NROW) * 3 * MAG;
        mom1 = longint'(NROW) * 3 * MAG * 6;
        cx_exp = exp_cent(mom0, mom1);
        $display("[INFO] T3 第 1 帧期望质心 = %0d (Q0.8) ≈ %0.2f 列",
                 cx_exp, cx_exp/256.0);

        send_coef(16'd1000, 16'd2000, 16'shFA24, 16'd3000);
        wait (frame_done === 1'b1);
        @(posedge clk);
        chk_cnt++;
        if (alert_cvalid !== 1'b1) begin
            $display("[ERR] T3 质心应有效（mom0 已高于下限）"); err_cnt++;
        end
        if (alert_cx !== 16'(cx_exp)) begin
            $display("[ERR] T3 质心不符：got=%0d exp=%0d", alert_cx, cx_exp); err_cnt++;
        end else begin
            $display("[INFO] T3 质心精确匹配 = %0d", alert_cx);
        end
        // 真实质心旁证：应在 col=6 的一个块列以内
        if (!((alert_cx >= 16'((6*256) - 256)) && (alert_cx <= 16'((6*256) + 256)))) begin
            $display("[ERR] T3 质心偏离真实值 col=6 超过 1 列：%0d", alert_cx); err_cnt++;
        end

        // 第 2 帧：能量移到 col=7，方向应为 +1
        send_diff(7);
        send_coef(16'd1000, 16'd2000, 16'shFA24, 16'd3000);
        wait (frame_done === 1'b1);
        @(posedge clk);
        chk_cnt++;
        if (alert_dir !== 2'sd1) begin
            $display("[ERR] T3 方向应为 +1，实得 %0d", alert_dir); err_cnt++;
        end else begin
            $display("[INFO] T3 方向 = +1（右移），delta = %0d Q0.8", alert_dx);
        end

        // ================= T4：低能量冻结 =================
        $display("[INFO] T4 低能量冻结：把 cfg_min_energy 抬到远高于当前 mom0");
        cfg_min_e = 48'h1_0000_0000;     // 远大于 mom0
        send_diff(6);
        send_coef(16'd1000, 16'd2000, 16'shFA24, 16'd3000);
        wait (frame_done === 1'b1);
        @(posedge clk);
        chk_cnt++;
        if (alert_cvalid !== 1'b0) begin
            $display("[ERR] T4 低能量时 alert_centroid_valid 应为 0"); err_cnt++;
        end else begin
            $display("[INFO] T4 低能量时质心被判无效（冻结上一次，未假报 0）");
        end

        // ================= T5：E 累加溢出（e_ovf，专用窄位宽实例）=================
        //   生产位宽下不可达的反证：单次商 <= 2^34，合法 cfg_k <= K_MAX=4 时
        //   总和 <= 4*65535^2 = 17179344900 < 2^48。故用 E_W=32 的 dut_o 触发。
        $display("[INFO] T5 E 累加溢出：E_W=%0d，d=65535, den=1, K=%0d", E_W_OVF, EO_KMAX);
        $display("[INFO] T5 生产位宽上界 = %0d < 2^%0d（e_ovf 不可达）",
                 longint'(K_MAX) * 64'd4294836225, E_W);
        for (i = 0; i < EO_KMAX; i++) begin
            o_mu_mem[i]  = 16'sh8000;    // μ = -32768
            o_var_mem[i] = 16'd0;        // v = 0（最坏，den = ε = 1）
        end
        o_thresh  = {E_W_OVF{1'b1}};
        o_min_e   = '0;
        o_k       = 16'(EO_KMAX);
        o_eps     = 16'd1;
        o_persist = 1'b1;
        // c = +32767 -> d = 32767 - (-32768) = 65535，d^2 = 4294836225
        e_ovf_true = longint'(EO_KMAX) * 64'd4294836225;
        e_ovf_sat  = {E_W_OVF{1'b1}};    // 饱和后的期望码字 = 2^E_W_OVF - 1
        $display("[INFO] T5 期望 E_true = %0d，饱和后 = %0d", e_ovf_true, e_ovf_sat);
        send_coef_o(16'sh7FFF, 16'sh7FFF, 16'sh7FFF, 16'sh7FFF);
        wait (o_fdone === 1'b1);
        @(posedge clk);
        chk_cnt++;
        if (o_ovf !== 1'b1) begin
            $display("[ERR] T5 e_ovf 未置起（E_true=%0d 已超 2^%0d）", e_ovf_true, E_W_OVF);
            err_cnt++;
        end else begin
            $display("[INFO] T5 e_ovf 正确置起（粘滞）");
        end
        chk_cnt++;
        if (o_score !== e_ovf_sat) begin
            $display("[ERR] T5 溢出后 alert_score=%0d 期望(饱和)=%0d", o_score, e_ovf_sat);
            err_cnt++;
        end else begin
            $display("[INFO] T5 溢出后 alert_score 逐值匹配饱和值 = %0d", o_score);
        end

        // ================= T6：v_k=0 的除零防护（ε 生效）=================
        //   bg_var 全 0 而 cfg_eps=1：x_den = v + ε = 1。E 必须按 den=ε 精确算出，
        //   zero_den 必须保持 0（cfg_eps>=1 使 den 不可能为 0）。
        $display("[INFO] T6 除零防护：v=0, eps=1 -> den=1");
        cfg_k       = 16'(K_MAX);
        cfg_eps     = 16'd1;
        cfg_persist = 1'b1;
        cfg_min_e   = 48'd1;
        mu_mem[0] = 16'd0;      var_mem[0] = 16'd0;
        mu_mem[1] = 16'd500;    var_mem[1] = 16'd0;
        mu_mem[2] = 16'shFE0C;  var_mem[2] = 16'd0;
        mu_mem[3] = 16'd1000;   var_mem[3] = 16'd0;
        e_exp = exp_E(1000,0,0, 2000,500,0, -1500,-500,0, 3000,1000,0, 1);
        send_coef(16'd1000, 16'd2000, 16'shFA24, 16'd3000);
        wait (frame_done === 1'b1);
        @(posedge clk);
        chk_cnt++;
        if (alert_score !== 48'(e_exp)) begin
            $display("[ERR] T6 E(v=0) 不符：got=%0d exp=%0d", alert_score, e_exp); err_cnt++;
        end else begin
            $display("[INFO] T6 E(v=0, den=ε) 精确匹配 = %0d", alert_score);
        end
        chk_cnt++;
        if (zero_den !== 1'b0) begin
            $display("[ERR] T6 zero_den 被置起（ε 防护下不应触发）"); err_cnt++;
        end else begin
            $display("[INFO] T6 zero_den 保持 0（ε 已挡住 den==0）");
        end

        // ================= T7：proto_err 注入（系数索引不一致）=================
        $display("[INFO] T7 系数索引注入错误 -> proto_err");
        cfg_eps   = 16'd1;
        cfg_k     = 16'(K_MAX);
        mu_mem[0] = 16'd0;      var_mem[0] = 16'd100;
        mu_mem[1] = 16'd500;    var_mem[1] = 16'd200;
        mu_mem[2] = 16'shFE0C;  var_mem[2] = 16'd250;
        mu_mem[3] = 16'd1000;   var_mem[3] = 16'd100;
        send_coef_badidx();
        wait (frame_done === 1'b1);
        @(posedge clk);
        chk_cnt++;
        if (proto_err !== 1'b1) begin
            $display("[ERR] T7 proto_err 未置起"); err_cnt++;
        end else begin
            $display("[INFO] T7 proto_err 正确置起");
        end
        // 粘滞性：随后发一帧**索引正确**的系数，proto_err 不得回落
        send_coef(16'd1000, 16'd2000, 16'shFA24, 16'd3000);
        wait (frame_done === 1'b1);
        @(posedge clk);
        chk_cnt++;
        if (proto_err !== 1'b1) begin
            $display("[ERR] T7 proto_err 应保持粘滞，却回落了"); err_cnt++;
        end else begin
            $display("[INFO] T7 proto_err 保持粘滞（正常帧不回落）");
        end

        // ================= T8：cfg_eps=0（除零配置）被拒 =================
        //   这条说明 **E 通路**的 zero_den 不可达：cfg_eps 下限 >= 1，=0 时在 E pass
        //   之前短路。（质心通路的守卫 n0==0 可达，2026-09-30 RTL 已统一两者口径。）
        $display("[INFO] T8 cfg_eps=0 -> cfg_err=1（不静默除零）");
        cfg_eps = 16'd0;
        repeat (2) @(posedge clk);
        chk_cnt++;
        if (cfg_err !== 1'b1) begin
            $display("[ERR] T8 cfg_eps=0 未报 cfg_err"); err_cnt++;
        end else begin
            $display("[INFO] T8 cfg_err 置起（配置被拒）");
        end
        send_coef(16'd1000, 16'd2000, 16'shFA24, 16'd3000);
        wait (frame_done === 1'b1);
        @(posedge clk);
        chk_cnt++;
        if (alert_score !== 48'd0) begin
            $display("[ERR] T8 cfg_err 帧后 alert_score 应为 0，实得 %0d", alert_score); err_cnt++;
        end else begin
            $display("[INFO] T8 cfg_err 帧走完流程但 e_acc/alert_score 归 0");
        end

        // ================= T9：cfg_persist_en=0（单帧判据）=================
        $display("[INFO] T9 persist=0：首帧超阈即报 alert_event=1");
        cfg_eps     = 16'd1;
        cfg_persist = 1'b0;
        mu_mem[0] = 16'd0;      var_mem[0] = 16'd100;
        mu_mem[1] = 16'd500;    var_mem[1] = 16'd200;
        mu_mem[2] = 16'shFE0C;  var_mem[2] = 16'd250;
        mu_mem[3] = 16'd1000;   var_mem[3] = 16'd100;
        send_coef(16'd1000, 16'd2000, 16'shFA24, 16'd3000);   // E = 64681 > 60000
        wait (frame_done === 1'b1);
        @(posedge clk);
        chk_cnt++;
        if (alert_event !== 1'b1) begin
            $display("[ERR] T9 persist=0 时首帧应报 alert_event=1"); err_cnt++;
        end else begin
            $display("[INFO] T9 persist=0 首帧 alert_event = 1（单帧判据）");
        end

        // ================= 汇总 =================
        if (err_cnt == 0)
            $display("=== fast_alert_tb : PASS ===  (比对点 %0d 个)", chk_cnt);
        else
            $display("=== fast_alert_tb : FAIL ===  错误 %0d 处 / 比对点 %0d 个",
                     err_cnt, chk_cnt);
        $finish;
    end

    initial begin
        #5000000;
        $display("[ERR] 超时");
        $display("=== fast_alert_tb : FAIL (timeout) ===");
        $finish;
    end

endmodule

`default_nettype wire
