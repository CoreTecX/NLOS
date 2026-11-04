// ============================================================================
// raw_preproc_tb.sv  ·  raw_preproc 自检测试平台
// ---------------------------------------------------------------------------
// ★★★ 仿真状态声明（必读）★★★
//   本测试平台**没有在本机运行过**。本机未安装 iverilog / Verilator / Vivado
//   xsim，无法编译或仿真，因此本文件**不附带任何仿真结果、通过/失败记录、
//   波形或覆盖率数据**。它是一份「待运行」的自检脚本，不是测试报告。
//
//   运行方式（在有工具链的机器上；本文件位于 src/fpga/rtl/tb/）：
//     iverilog -g2012 -o raw_preproc_tb.vvp raw_preproc_tb.sv ../raw_preproc.sv
//     vvp raw_preproc_tb.vvp
//     或 Vivado xsim（xvlog / xelab / xsim 三步）
//   预期输出：末尾打印 "=== raw_preproc_tb : PASS ==="。
//
// ---------------------------------------------------------------------------
// 覆盖的内容
// ---------------------------------------------------------------------------
//   T1 彩色路径：COLOR=1 / BLOCK=2 / WALL_N=4 -> RAW 16x16，cfg_frames=4
//      同一份确定性 RAW 连发 4 帧，逐值比对 4 帧累积后的 4x4x3 块和（48 拍）、
//      块行末 tlast 位置、tuser 位置，以及 ovf_sticky / proto_err / cfg_err。
//      判据是**逐值相等**（sim/vectors/readme.md 的 P1：预处理链不做除法、
//      不做舍入，期望值是整数，必须严格相等）。
//
//   T2 暗场扣除：cfg_dark_level=100，cfg_frames=1，逐值比对 48 拍
//      参考模型为 max(0, pix - dark)。这条同时覆盖「先减后处理、负值饱和到 0」。
//
//   T3 黑白路径：COLOR=0 的第二个 DUT 实例，每块 1 拍输出。
//
//   T4 负例自检：TB 另算一份「跨行分组（错误）」的期望值，断言它与正确值不同。
//      若两者相同，说明这组激励无法区分正确/错误的分块索引 —— 测试向量失效，
//      必须换激励（对应 sim/vectors 里 v4_edge_cases 的跨行对照思路）。
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module raw_preproc_tb;

    // ------------------------------------------------------------------
    // 参数：小尺寸配置，让 TB 的周期数可控
    //   彩色：BLK_SIDE = 2*BLOCK = 4，RAW_SIDE = 2*BLOCK*WALL_N = 16
    //   黑白：BLK_SIDE = BLOCK = 2，RAW_SIDE = BLOCK*WALL_N = 8
    //   [2026-09-28 修正] 黑白的这两个值原写成 4 / 16，与下面
    //     BLK_SIDE_M = BLOCK、RAW_SIDE_M = BLOCK*WALL_N 的实际取值（BLOCK=2）
    //     不符 —— **代码是对的、注释是错的**。这个笔误一度让「第 6 个缺陷」的
    //     缺口系数推不出来（见 sim/results/rtl_root_cause_fast_alert_raw_preproc.md）。
    // ------------------------------------------------------------------
    localparam int WALL_N  = 4;
    localparam int BLOCK   = 2;
    localparam int IN_BITS = 16;
    localparam int V_BITS  = 12;
    localparam int MAXF    = 4;
    // [2026-09-28 首次仿真修] 原为 2，但 DUT 只允许 cfg_frames ∈ {1,4,8}
    //   （见 raw_preproc.sv 的 cfg_frames 端口注释与 cfg_err 判据）。2 是非法值
    //   ⇒ 内部 cfg_err=1 ⇒ 处理不推进 ⇒ frame_done 永不置起 ⇒ 本 TB 的
    //   `wait (c_frame_done)` 挂死，由看门狗在 #5000000 处强制 $finish。
    //   取 4：既是合法值，又正好等于 MAXF(=4)，且仍保留"多帧累积"的用例意图
    //   （不退回 1，否则测不到累加器）。
    localparam int NFRM    = 4;                    // T1 累积 4 帧（1/4/8 之一）
    localparam int DARK    = 100;                  // T2 暗电平

    localparam int RAW_SIDE_C = 2*BLOCK*WALL_N;    // 16
    localparam int BLK_SIDE_C = 2*BLOCK;           // 4
    localparam int RAW_SIDE_M = BLOCK*WALL_N;      // 8（原注释写 16）
    localparam int BLK_SIDE_M = BLOCK;             // 2（原注释写 4，跟着 BLOCK=2 时应为 2）
    localparam int NBLK       = WALL_N*WALL_N;     // 16
    // [2026-09-28 首次仿真修] m_meas_tblk 的端口位宽是 DUT 的 BW = $clog2(WALL_N^2)，
    //   随 WALL_N 变化（WALL_N=96 时 14 位，本 tb 的 WALL_N=4 时 4 位）。原先把 tb 侧
    //   的接收信号写死成 [13:0]，于是 iverilog 报「expects 4 bit(s), given 14」。
    //   这里跟着参数算出来，位宽告警消失，且换 WALL_N 时不需要再改 tb。
    localparam int BW         = $clog2(NBLK);      // = DUT 的 BW

    logic clk = 1'b0;
    always #5 clk = ~clk;        // 100 MHz（仅 TB 用，不代表实现频率）
    logic rst_n = 1'b0;

    int err_cnt = 0;
    int chk_cnt = 0;

    // ------------------------------------------------------------------
    // 激励数据
    // ------------------------------------------------------------------
    logic [IN_BITS-1:0] frame_c [0:NFRM-1][0:RAW_SIDE_C*RAW_SIDE_C-1];
    logic [IN_BITS-1:0] frame_m [0:NFRM-1][0:RAW_SIDE_M*RAW_SIDE_M-1];

    // 伪随机但确定性的像素（12 bit 有效，落在 packed 口径的 0..4095）
    function automatic logic [IN_BITS-1:0] pix_of(input int f, input int r, input int c);
        int v;
        v = (r*37 + c*101 + f*197) % 4093;
        pix_of = IN_BITS'(v);
    endfunction

    // ------------------------------------------------------------------
    // 参考模型（与 preprocess.hpp 的 block_sum 同口径：块内求和，零舍入）
    //   彩色 RGGB：(偶数行,偶数列)=R，(偶数行,奇数列)=G1，
    //              (奇数行,偶数列)=G2，(奇数行,奇数列)=B；G = G1 + G2（P2）
    // ------------------------------------------------------------------
    logic [31:0] exp_c      [0:NBLK-1][0:2];   // T1 期望
    logic [31:0] exp_c_dark [0:NBLK-1][0:2];   // T2 期望（减暗电平后）
    logic [31:0] exp_c_wrong[0:NBLK-1][0:2];   // T4 负例：跨行分组
    logic [31:0] exp_m      [0:NBLK-1];

    int wrong_diff_cnt;

    function automatic int sub_dark(input int p, input int dark);
        sub_dark = (p > dark) ? (p - dark) : 0;
    endfunction

    task automatic build_expectations();
        int br, bc, ri, ci, r, c, f, blk, flat;
        int ch;
        for (blk = 0; blk < NBLK; blk++) begin
            for (ch = 0; ch < 3; ch++) begin
                exp_c[blk][ch]       = 32'd0;
                exp_c_dark[blk][ch]  = 32'd0;
                exp_c_wrong[blk][ch] = 32'd0;
            end
            exp_m[blk] = 32'd0;
        end

        // ---- 正确口径：块 = 4x4 原始像素，绝不可跨块行 ----
        for (br = 0; br < WALL_N; br++) begin
            for (bc = 0; bc < WALL_N; bc++) begin
                blk = br*WALL_N + bc;
                for (ri = 0; ri < BLK_SIDE_C; ri++) begin
                    for (ci = 0; ci < BLK_SIDE_C; ci++) begin
                        r = br*BLK_SIDE_C + ri;
                        c = bc*BLK_SIDE_C + ci;
                        for (f = 0; f < NFRM; f++) begin
                            if ((ri % 2 == 0) && (ci % 2 == 0)) begin
                                exp_c[blk][0] = exp_c[blk][0] + pix_of(f, r, c);
                            end else if ((ri % 2 == 1) && (ci % 2 == 1)) begin
                                exp_c[blk][2] = exp_c[blk][2] + pix_of(f, r, c);
                            end else begin
                                exp_c[blk][1] = exp_c[blk][1] + pix_of(f, r, c);
                            end
                            // T2：只用第 0 帧，且先减暗电平
                            if (f == 0) begin
                                if ((ri % 2 == 0) && (ci % 2 == 0)) begin
                                    exp_c_dark[blk][0] = exp_c_dark[blk][0] + sub_dark(pix_of(0,r,c), DARK);
                                end else if ((ri % 2 == 1) && (ci % 2 == 1)) begin
                                    exp_c_dark[blk][2] = exp_c_dark[blk][2] + sub_dark(pix_of(0,r,c), DARK);
                                end else begin
                                    exp_c_dark[blk][1] = exp_c_dark[blk][1] + sub_dark(pix_of(0,r,c), DARK);
                                end
                            end
                        end
                    end
                end
            end
        end

        // ---- T4 错误口径：按展平索引每 16 个像素分一组（会跨 raw 行）----
        for (f = 0; f < NFRM; f++) begin
            for (flat = 0; flat < RAW_SIDE_C*RAW_SIDE_C; flat++) begin
                blk = flat / (BLK_SIDE_C*BLK_SIDE_C);
                ri  = (flat % (BLK_SIDE_C*BLK_SIDE_C)) / BLK_SIDE_C;
                ci  = flat % BLK_SIDE_C;
                r   = flat / RAW_SIDE_C;
                c   = flat % RAW_SIDE_C;
                if ((ri % 2 == 0) && (ci % 2 == 0)) begin
                    exp_c_wrong[blk][0] = exp_c_wrong[blk][0] + pix_of(f, r, c);
                end else if ((ri % 2 == 1) && (ci % 2 == 1)) begin
                    exp_c_wrong[blk][2] = exp_c_wrong[blk][2] + pix_of(f, r, c);
                end else begin
                    exp_c_wrong[blk][1] = exp_c_wrong[blk][1] + pix_of(f, r, c);
                end
            end
        end

        wrong_diff_cnt = 0;
        for (blk = 0; blk < NBLK; blk++) begin
            for (ch = 0; ch < 3; ch++) begin
                if (exp_c[blk][ch] !== exp_c_wrong[blk][ch]) wrong_diff_cnt++;
            end
        end

        // ---- 黑白：块 = 4x4 直接求和 ----
        for (br = 0; br < WALL_N; br++) begin
            for (bc = 0; bc < WALL_N; bc++) begin
                blk = br*WALL_N + bc;
                for (ri = 0; ri < BLK_SIDE_M; ri++) begin
                    for (ci = 0; ci < BLK_SIDE_M; ci++) begin
                        r = br*BLK_SIDE_M + ri;
                        c = bc*BLK_SIDE_M + ci;
                        for (f = 0; f < NFRM; f++) exp_m[blk] = exp_m[blk] + pix_of(f, r, c);
                    end
                end
            end
        end
    endtask

    // ------------------------------------------------------------------
    // DUT 1：彩色
    // ------------------------------------------------------------------
    logic                c_s_tvalid, c_s_tready, c_s_tlast, c_s_tuser;
    logic [IN_BITS-1:0]  c_s_tdata;
    logic [23:0]         c_m_tdata;
    logic [1:0]          c_m_tch;
    logic                c_m_tvalid, c_m_tready, c_m_tlast, c_m_tuser, c_frame_done;
    logic                c_busy, c_ovf, c_proto_err, c_cfg_err;
    logic [BW-1:0]       c_m_tblk;
    logic                c_rst_n;
    logic [IN_BITS-1:0]  c_dark;
    logic [3:0]          c_frames;

    raw_preproc #(
        .IN_BITS(IN_BITS), .VALID_BITS(V_BITS), .BLOCK(BLOCK), .WALL_N(WALL_N),
        .CH(3), .COLOR(1'b1), .MAX_FRAMES(MAXF),
        .BS_BITS(20), .ACC_BITS(23), .OUT_BITS(24)
    ) dut_c (
        .clk(clk), .rst_n(c_rst_n),
        .cfg_dark_level(c_dark), .cfg_frames(c_frames),
        .cfg_crop_side(16'(RAW_SIDE_C)), .cfg_bayer_phase(1'b0),
        .s_axis_tdata(c_s_tdata), .s_axis_tvalid(c_s_tvalid), .s_axis_tready(c_s_tready),
        .s_axis_tlast(c_s_tlast), .s_axis_tuser(c_s_tuser),
        .m_meas_tdata(c_m_tdata), .m_meas_tch(c_m_tch), .m_meas_tvalid(c_m_tvalid),
        .m_meas_tready(c_m_tready), .m_meas_tlast(c_m_tlast), .m_meas_tuser(c_m_tuser),
        .m_meas_tblk(c_m_tblk),
        .frame_done(c_frame_done), .busy(c_busy), .ovf_sticky(c_ovf),
        .proto_err(c_proto_err), .cfg_err(c_cfg_err),
        .sat_pix_cnt(), .total_pix_cnt()
    );

    // ------------------------------------------------------------------
    // DUT 2：黑白
    // ------------------------------------------------------------------
    logic                m_s_tvalid, m_s_tready, m_s_tlast, m_s_tuser;
    logic [IN_BITS-1:0]  m_s_tdata;
    logic [23:0]         m_m_tdata;
    logic [0:0]          m_m_tch;
    logic                m_m_tvalid, m_m_tready, m_m_tlast, m_m_tuser, m_frame_done;
    logic                m_busy, m_ovf, m_proto_err, m_cfg_err;
    logic [BW-1:0]       m_m_tblk;
    logic                m_rst_n;

    raw_preproc #(
        .IN_BITS(IN_BITS), .VALID_BITS(V_BITS), .BLOCK(BLOCK), .WALL_N(WALL_N),
        .CH(1), .COLOR(1'b0), .MAX_FRAMES(MAXF),
        .BS_BITS(20), .ACC_BITS(23), .OUT_BITS(24)
    ) dut_m (
        .clk(clk), .rst_n(m_rst_n),
        .cfg_dark_level(16'd0), .cfg_frames(4'(NFRM)),
        .cfg_crop_side(16'(RAW_SIDE_M)), .cfg_bayer_phase(1'b0),
        .s_axis_tdata(m_s_tdata), .s_axis_tvalid(m_s_tvalid), .s_axis_tready(m_s_tready),
        .s_axis_tlast(m_s_tlast), .s_axis_tuser(m_s_tuser),
        .m_meas_tdata(m_m_tdata), .m_meas_tch(m_m_tch), .m_meas_tvalid(m_m_tvalid),
        .m_meas_tready(m_m_tready), .m_meas_tlast(m_m_tlast), .m_meas_tuser(m_m_tuser),
        .m_meas_tblk(m_m_tblk),
        .frame_done(m_frame_done), .busy(m_busy), .ovf_sticky(m_ovf),
        .proto_err(m_proto_err), .cfg_err(m_cfg_err),
        .sat_pix_cnt(), .total_pix_cnt()
    );

    // ------------------------------------------------------------------
    // 发送一拍（彩色）
    //   握手在**下降沿**判断：若下降沿时 tready 为高，则紧接着的上升沿
    //   DUT 一定会采样到 tvalid=1，恰好被接收一拍。
    //   若写成「先 @(posedge) 再查 tready」，会在边沿后读到 DUT 已翻转的
    //   （可能已经拉低的）tready，导致同一像素被发两次 —— 这是 AXIS 激励
    //   里最典型的重复发拍错误，会表现为「块和恰好是两倍」。
    // ------------------------------------------------------------------
    task automatic send_pixel_c(input logic [IN_BITS-1:0] d,
                                input bit is_first, input bit is_last);
        c_s_tdata  = d;
        c_s_tuser  = is_first;
        c_s_tlast  = is_last;
        c_s_tvalid = 1'b1;
        forever begin
            @(negedge clk);
            if (c_s_tready === 1'b1) begin
                @(posedge clk);
                break;
            end
        end
    endtask

    task automatic send_frame_c(input int f, input int raw_side);
        int r, c, idx;
        idx = 0;
        for (r = 0; r < raw_side; r++) begin
            for (c = 0; c < raw_side; c++) begin
                send_pixel_c(frame_c[f][idx], (r == 0 && c == 0), (c == raw_side-1));
                idx++;
            end
        end
        c_s_tvalid = 1'b0;
        c_s_tuser  = 1'b0;
        c_s_tlast  = 1'b0;
    endtask

    // ------------------------------------------------------------------
    // 发送一拍（黑白）
    // ------------------------------------------------------------------
    task automatic send_pixel_m(input logic [IN_BITS-1:0] d,
                                input bit is_first, input bit is_last);
        m_s_tdata  = d;
        m_s_tuser  = is_first;
        m_s_tlast  = is_last;
        m_s_tvalid = 1'b1;
        forever begin
            @(negedge clk);
            if (m_s_tready === 1'b1) begin
                @(posedge clk);
                break;
            end
        end
    endtask

    task automatic send_frame_m(input int f, input int raw_side);
        int r, c, idx;
        idx = 0;
        for (r = 0; r < raw_side; r++) begin
            for (c = 0; c < raw_side; c++) begin
                send_pixel_m(frame_m[f][idx], (r == 0 && c == 0), (c == raw_side-1));
                idx++;
            end
        end
        m_s_tvalid = 1'b0;
        m_s_tuser  = 1'b0;
        m_s_tlast  = 1'b0;
    endtask

    // ------------------------------------------------------------------
    // 输出比对（彩色）：exp_ref 由 phase 切换
    // ------------------------------------------------------------------
    logic [31:0] exp_ref [0:NBLK-1][0:2];
    int  c_beat = 0;
    bit  c_chk_en = 1'b0;

    always @(posedge clk) begin
        if (c_m_tvalid && c_m_tready) begin
            if (c_chk_en) begin
                if (c_beat < NBLK*3) begin
                    chk_cnt++;
                    if (c_m_tdata !== 24'(exp_ref[c_beat/3][c_beat%3])) begin
                        $display("[ERR] (彩色) beat=%0d blk=%0d ch=%0d got=%0d exp=%0d",
                                 c_beat, c_beat/3, c_beat%3, c_m_tdata,
                                 exp_ref[c_beat/3][c_beat%3]);
                        err_cnt++;
                    end
                    if (c_m_tch !== 2'(c_beat%3)) begin
                        $display("[ERR] (彩色) beat=%0d tch=%0d 期望=%0d",
                                 c_beat, c_m_tch, c_beat%3);
                        err_cnt++;
                    end
                    if (c_m_tlast !== (((c_beat%3) == 2) &&
                                       (((c_beat/3) % WALL_N) == (WALL_N-1)))) begin
                        $display("[ERR] (彩色) beat=%0d tlast 位置错误", c_beat);
                        err_cnt++;
                    end
                    if (c_m_tuser !== (c_beat == 0)) begin
                        $display("[ERR] (彩色) beat=%0d tuser 位置错误", c_beat);
                        err_cnt++;
                    end
                end else begin
                    $display("[ERR] (彩色) 输出拍数超出预期 %0d", c_beat);
                    err_cnt++;
                end
                c_beat++;
            end
        end
    end

    // ------------------------------------------------------------------
    // 输出比对（黑白）
    // ------------------------------------------------------------------
    int m_beat = 0;
    always @(posedge clk) begin
        if (m_m_tvalid && m_m_tready) begin
            if (m_beat < NBLK) begin
                chk_cnt++;
                if (m_m_tdata !== 24'(exp_m[m_beat])) begin
                    $display("[ERR] (黑白) blk=%0d got=%0d exp=%0d",
                             m_beat, m_m_tdata, exp_m[m_beat]);
                    err_cnt++;
                end
                if (m_m_tlast !== ((m_beat % WALL_N) == (WALL_N-1))) begin
                    $display("[ERR] (黑白) blk=%0d tlast 位置错误", m_beat);
                    err_cnt++;
                end
            end else begin
                $display("[ERR] (黑白) 输出拍数超出预期 %0d", m_beat);
                err_cnt++;
            end
            m_beat++;
        end
    end

    // ------------------------------------------------------------------
    // 主流程
    // ------------------------------------------------------------------
    int f, i, blk, ch;
    initial begin
        c_s_tvalid = 1'b0; c_s_tuser = 1'b0; c_s_tlast = 1'b0; c_s_tdata = '0;
        m_s_tvalid = 1'b0; m_s_tuser = 1'b0; m_s_tlast = 1'b0; m_s_tdata = '0;
        c_m_tready = 1'b1;      // 本 TB 恒就绪；测反压时把它改成周期抖动即可
        m_m_tready = 1'b1;
        c_rst_n = 1'b0; m_rst_n = 1'b0;
        c_dark = 16'd0; c_frames = 4'(NFRM);

        // 生成激励
        for (f = 0; f < NFRM; f++) begin
            for (i = 0; i < RAW_SIDE_C*RAW_SIDE_C; i++)
                frame_c[f][i] = pix_of(f, i / RAW_SIDE_C, i % RAW_SIDE_C);
            for (i = 0; i < RAW_SIDE_M*RAW_SIDE_M; i++)
                frame_m[f][i] = pix_of(f, i / RAW_SIDE_M, i % RAW_SIDE_M);
        end
        build_expectations();

        // T4 负例自检
        if (wrong_diff_cnt == 0) begin
            $display("[ERR] T4 失效：正确口径与跨行错误口径的期望完全相同，激励无法区分错误");
            err_cnt++;
        end else begin
            $display("[INFO] T4 负例有效：跨行错误口径与正确口径有 %0d 处不同", wrong_diff_cnt);
        end

        repeat (8) @(posedge clk);
        c_rst_n = 1'b1; m_rst_n = 1'b1;
        repeat (4) @(posedge clk);

        // ================= T1：彩色，2 帧累积 =================
        $display("[INFO] T1 彩色路径：%0d 帧 x %0dx%0d", NFRM, RAW_SIDE_C, RAW_SIDE_C);
        for (blk = 0; blk < NBLK; blk++)
            for (ch = 0; ch < 3; ch++) exp_ref[blk][ch] = exp_c[blk][ch];
        c_beat   = 0;
        c_chk_en = 1'b1;
        for (f = 0; f < NFRM; f++) send_frame_c(f, RAW_SIDE_C);
        wait (c_frame_done === 1'b1);
        repeat (4) @(posedge clk);
        c_chk_en = 1'b0;

        if (c_beat != NBLK*3) begin
            $display("[ERR] T1 输出拍数 %0d，期望 %0d", c_beat, NBLK*3); err_cnt++;
        end
        if (c_ovf       !== 1'b0) begin $display("[ERR] T1 ovf_sticky 被置起"); err_cnt++; end
        if (c_proto_err !== 1'b0) begin $display("[ERR] T1 proto_err 被置起");  err_cnt++; end
        if (c_cfg_err   !== 1'b0) begin $display("[ERR] T1 cfg_err 被置起");    err_cnt++; end

        // ================= T2：暗场扣除（逐值）=================
        $display("[INFO] T2 暗场扣除：dark=%0d，单帧", DARK);
        c_dark   = 16'(DARK);
        c_frames = 4'd1;
        for (blk = 0; blk < NBLK; blk++)
            for (ch = 0; ch < 3; ch++) exp_ref[blk][ch] = exp_c_dark[blk][ch];
        c_beat   = 0;
        c_chk_en = 1'b1;
        send_frame_c(0, RAW_SIDE_C);
        wait (c_frame_done === 1'b1);
        repeat (4) @(posedge clk);
        c_chk_en = 1'b0;

        if (c_beat != NBLK*3) begin
            $display("[ERR] T2 输出拍数 %0d，期望 %0d", c_beat, NBLK*3); err_cnt++;
        end
        if (c_ovf !== 1'b0) begin $display("[ERR] T2 ovf_sticky 被置起"); err_cnt++; end

        // 复位回默认配置，避免影响后续扩展用例
        c_dark   = 16'd0;
        c_frames = 4'(NFRM);

        // ================= T3：黑白 =================
        $display("[INFO] T3 黑白路径：%0d 帧 x %0dx%0d", NFRM, RAW_SIDE_M, RAW_SIDE_M);
        for (f = 0; f < NFRM; f++) send_frame_m(f, RAW_SIDE_M);
        wait (m_frame_done === 1'b1);
        repeat (4) @(posedge clk);

        if (m_beat != NBLK) begin
            $display("[ERR] T3 输出拍数 %0d，期望 %0d", m_beat, NBLK); err_cnt++;
        end
        if (m_ovf       !== 1'b0) begin $display("[ERR] T3 ovf_sticky 被置起"); err_cnt++; end
        if (m_proto_err !== 1'b0) begin $display("[ERR] T3 proto_err 被置起");  err_cnt++; end
        if (m_cfg_err   !== 1'b0) begin $display("[ERR] T3 cfg_err 被置起");    err_cnt++; end

        // ================= 汇总 =================
        if (err_cnt == 0)
            $display("=== raw_preproc_tb : PASS ===  (比对点 %0d 个)", chk_cnt);
        else
            $display("=== raw_preproc_tb : FAIL ===  错误 %0d 处 / 比对点 %0d 个",
                     err_cnt, chk_cnt);
        $finish;
    end

    // 超时保护
    initial begin
        #5000000;
        $display("[ERR] 超时：TB 未在预期周期数内结束");
        $display("=== raw_preproc_tb : FAIL (timeout) ===");
        $finish;
    end

endmodule

`default_nettype wire
