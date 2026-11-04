// ============================================================================
// vertical_diff_tb.sv  ·  vertical_diff 自检测试平台
// ---------------------------------------------------------------------------
// ★★★ 仿真状态声明（必读）★★★
//   本测试平台**没有在本机运行过**（本机未安装 iverilog / Verilator / Vivado
//   xsim，无法编译或仿真）。本文件不含任何仿真结果、波形或通过记录。
//
//   运行方式（本文件位于 src/fpga/rtl/tb/）：
//     iverilog -g2012 -o vertical_diff_tb.vvp vertical_diff_tb.sv ../vertical_diff.sv
//     vvp vertical_diff_tb.vvp
//   预期输出：末尾打印 "=== vertical_diff_tb : PASS ==="。
//
// ---------------------------------------------------------------------------
// 覆盖的内容（WALL_N=4, CH=3 -> 输入 48 拍，输出 36 拍）
// ---------------------------------------------------------------------------
//   T1 正常差分：4 行 x 4 块 x 3 通道，逐值比对 36 拍
//        diff[r][c][ch] = sum[r+1][c][ch] - sum[r][c][ch]
//      同时校验 m_diff_trow/tcol/tch、tlast（**块行末**：该行最后一个块的最后一个
//      通道）、tuser（帧首拍）。
//      **并在 T1 期间对 m_diff_tready 施加周期性反压**，专门打 1 拍流水的
//      stall 路径：若握手（~pipe_valid | ready）写错，反压时会出现丢拍或重复拍。
//
//   T2 旁路模式（cfg_bypass=1，§9.3 必做对照 2「原始测量 / 垂直差分」）
//      输出 4 行 = 48 拍，值等于输入（非负，装进 signed 容器）。
//
//   T3 故障注入：故意把某一拍的 tch 送错，断言 proto_err 粘滞置起。
//      这条覆盖的是「通道串位」——画面仍像图，只是 R/G/B 互相错位，
//      靠肉眼几乎发现不了，必须由硬件自检顶出来。
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module vertical_diff_tb;

    localparam int WALL_N    = 4;
    localparam int CH        = 3;
    localparam int IN_BITS   = 24;
    localparam int DIFF_BITS = 24;
    localparam int NIN       = WALL_N*WALL_N*CH;         // 48
    localparam int NOUT      = (WALL_N-1)*WALL_N*CH;     // 36
    localparam int NOUT_BP   = WALL_N*WALL_N*CH;         // 48

    logic clk = 1'b0;
    always #5 clk = ~clk;
    logic rst_n = 1'b0;

    int err_cnt = 0;
    int chk_cnt = 0;

    // ------------------------------------------------------------------
    // 输入数据：3 通道块和（无符号 24 bit）
    // ------------------------------------------------------------------
    logic [IN_BITS-1:0] in_sum [0:NIN-1];

    function automatic logic [IN_BITS-1:0] sum_of(input int r, input int c, input int ch);
        sum_of = IN_BITS'(1000 + r*700 + c*130 + ch*17 + ((r*c*ch*29) % 101));
    endfunction

    // ------------------------------------------------------------------
    // 期望值
    // ------------------------------------------------------------------
    logic signed [DIFF_BITS-1:0] exp_diff [0:NOUT-1];     // T1
    logic signed [DIFF_BITS-1:0] exp_byp  [0:NOUT_BP-1];  // T2
    int exp_row [0:NOUT-1];                               // T1 坐标
    int exp_col [0:NOUT-1];
    int exp_ch  [0:NOUT-1];

    task automatic build_expectations();
        int r, c, ch, k;
        for (r = 0; r < WALL_N; r++)
            for (c = 0; c < WALL_N; c++)
                for (ch = 0; ch < CH; ch++) begin
                    k = (r*WALL_N + c)*CH + ch;
                    in_sum[k] = sum_of(r, c, ch);
                end
        // T1：差分（下一行减上一行）
        for (r = 0; r+1 < WALL_N; r++) begin
            for (c = 0; c < WALL_N; c++) begin
                for (ch = 0; ch < CH; ch++) begin
                    k = (r*WALL_N + c)*CH + ch;
                    exp_diff[k] = $signed({1'b0, sum_of(r+1,c,ch)}) -
                                  $signed({1'b0, sum_of(r, c,ch)});
                    exp_row[k] = r;
                    exp_col[k] = c;
                    exp_ch[k]  = ch;
                end
            end
        end
        // T2：旁路（等于输入）
        for (r = 0; r < WALL_N; r++)
            for (c = 0; c < WALL_N; c++)
                for (ch = 0; ch < CH; ch++) begin
                    k = (r*WALL_N + c)*CH + ch;
                    exp_byp[k] = $signed({1'b0, sum_of(r,c,ch)});
                end
    endtask

    // ------------------------------------------------------------------
    // DUT
    // ------------------------------------------------------------------
    logic                        s_valid, s_ready, s_last, s_user;
    logic [IN_BITS-1:0]          s_data;
    logic [1:0]                  s_ch;
    logic signed [DIFF_BITS-1:0] m_data;
    logic [1:0]                  m_ch;
    logic                        m_valid, m_ready, m_last, m_user;
    logic [1:0]                  m_row, m_col;      // NW = clog2(4) = 2
    logic                        frame_done, ovf_diff, proto_err, busy;
    logic                        cfg_bypass;
    logic [15:0]                 cfg_wall_n;

    // 反压发生器：bp_en=1 时每 3 拍拉低 1 拍
    logic bp_en = 1'b0;
    logic [1:0] bp_cnt = 2'd0;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) bp_cnt <= 2'd0;
        else        bp_cnt <= bp_cnt + 2'd1;
    end
    assign m_ready = bp_en ? (bp_cnt != 2'd0) : 1'b1;

    vertical_diff #(
        .WALL_N(WALL_N), .CH(CH), .IN_BITS(IN_BITS),
        .DIFF_BITS(DIFF_BITS), .CHW(2)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .cfg_bypass(cfg_bypass), .cfg_wall_n(cfg_wall_n),
        .s_meas_tdata(s_data), .s_meas_tch(s_ch), .s_meas_tvalid(s_valid),
        .s_meas_tready(s_ready), .s_meas_tlast(s_last), .s_meas_tuser(s_user),
        .m_diff_tdata(m_data), .m_diff_tch(m_ch), .m_diff_tvalid(m_valid),
        .m_diff_tready(m_ready), .m_diff_tlast(m_last), .m_diff_tuser(m_user),
        .m_diff_trow(m_row), .m_diff_tcol(m_col),
        .frame_done(frame_done), .ovf_diff(ovf_diff), .proto_err(proto_err),
        .busy(busy)
    );

    // ------------------------------------------------------------------
    // 发一拍
    //
    // ★ 2026-09-28 修正：首次仿真暴露出的 **TB 竞态**（不是 DUT 缺陷）
    //   原实现在「上一个上升沿刚过」的同一时间步里用**阻塞赋值**改 s_data/s_ch，
    //   而 DUT 的 always_ff 也在同一个上升沿采样 s_meas_tdata —— 两者谁先执行取决于
    //   仿真器的调度顺序（iverilog 下 initial 的续行先跑），于是 DUT 采到的是**下一拍**
    //   的数据与通道，而坐标计数器仍是本拍的。逐拍反算可证实：
    //       got[k] = in[k+1] - 上一行在第 k 个 (col,ch) 的值
    //   例如 got[0]=717 = in(1,0,1) 1717 - in(0,0,0) 1000。
    //   同一条根因也解释了 T1 的假 proto_err：DUT 把「下一拍的 tch」与「本拍计数」比对，
    //   必然报错。
    //
    //   修法：激励一律在**下降沿之后**更新，离采样沿半个周期，DUT 采到的必是本拍数据；
    //   被反压时保持同一拍不变，直到某个上升沿 tready 为高（标准 AXIS 行为）。
    // ------------------------------------------------------------------
    task automatic send_beat(input logic [IN_BITS-1:0] d, input logic [1:0] chn,
                             input bit is_last, input bit is_user);
        @(negedge clk);
        s_data  = d;
        s_ch    = chn;
        s_last  = is_last;
        s_user  = is_user;
        s_valid = 1'b1;
        @(posedge clk);
        while (s_ready !== 1'b1) @(posedge clk);   // 反压：保持本拍，等下一个上升沿
    endtask

    // 正常发一帧：块行 -> 块 -> 通道
    task automatic send_frame(input bit corrupt_ch0);
        int r, c, ch, k;
        for (r = 0; r < WALL_N; r++) begin
            for (c = 0; c < WALL_N; c++) begin
                for (ch = 0; ch < CH; ch++) begin
                    k = (r*WALL_N + c)*CH + ch;
                    // T3：故意把第一拍的 tch 送错
                    // ★ 2026-09-28 修正：tlast 必须是**块行末**（该行最后一个块的最后一个
                    //   通道），原实现写成「每个块末」(ch==CH-1)，与 DUT 的口径
                    //   （s_meas_tlast = 块行末）以及本 TB 自己的输出校验都不一致 ——
                    //   DUT 会因此把每一块都判成协议错。
                    send_beat(in_sum[k],
                              (corrupt_ch0 && k == 0) ? 2'd2 : 2'(ch),
                              (c == WALL_N-1) && (ch == CH-1), (k == 0));
                end
            end
        end
        // 收尾也要避开上升沿：若在上一个上升沿的同一时间步把 s_valid 拉低，
        // DUT 在同一沿可能看到 valid=0，**最后一拍会被丢掉**（同一条竞态根因）。
        @(negedge clk);
        s_valid = 1'b0;
        s_last  = 1'b0;
        s_user  = 1'b0;
    endtask

    // ------------------------------------------------------------------
    // 输出比对
    // ------------------------------------------------------------------
    logic signed [DIFF_BITS-1:0] exp_ref [0:NOUT_BP-1];
    int  o_beat = 0;
    bit  o_chk_en = 1'b0;
    bit  o_is_bypass = 1'b0;

    always @(posedge clk) begin
        if (m_valid && m_ready) begin
            if (o_chk_en) begin
                if (o_beat < (o_is_bypass ? NOUT_BP : NOUT)) begin
                    chk_cnt++;
                    if (m_data !== exp_ref[o_beat]) begin
                        $display("[ERR] beat=%0d tdata got=%0d exp=%0d",
                                 o_beat, m_data, exp_ref[o_beat]);
                        err_cnt++;
                    end
                    if (o_is_bypass) begin
                        // 旁路：坐标 = 输入坐标
                        if ((m_row !== 2'(o_beat/(WALL_N*CH))) ||
                            (m_col !== 2'((o_beat/CH) % WALL_N)) ||
                            (m_ch  !== 2'(o_beat % CH))) begin
                            $display("[ERR] 旁路 beat=%0d 坐标错 row=%0d col=%0d ch=%0d",
                                     o_beat, m_row, m_col, m_ch);
                            err_cnt++;
                        end
                        if (m_last !== (((o_beat % CH) == CH-1) &&
                                        (((o_beat/CH) % WALL_N) == WALL_N-1))) begin
                            $display("[ERR] 旁路 beat=%0d tlast 位置错", o_beat);
                            err_cnt++;
                        end
                        if (m_user !== (o_beat == 0)) begin
                            $display("[ERR] 旁路 beat=%0d tuser 位置错", o_beat);
                            err_cnt++;
                        end
                    end else begin
                        // 差分：坐标 = 输出坐标（行已减 1）
                        if ((m_row !== 2'(o_beat/(WALL_N*CH))) ||
                            (m_col !== 2'((o_beat/CH) % WALL_N)) ||
                            (m_ch  !== 2'(o_beat % CH))) begin
                            $display("[ERR] 差分 beat=%0d 坐标错 row=%0d col=%0d ch=%0d",
                                     o_beat, m_row, m_col, m_ch);
                            err_cnt++;
                        end
                        if (m_last !== (((o_beat % CH) == CH-1) &&
                                        (((o_beat/CH) % WALL_N) == WALL_N-1))) begin
                            $display("[ERR] 差分 beat=%0d tlast 位置错", o_beat);
                            err_cnt++;
                        end
                        if (m_user !== (o_beat == 0)) begin
                            $display("[ERR] 差分 beat=%0d tuser 位置错", o_beat);
                            err_cnt++;
                        end
                    end
                end else begin
                    $display("[ERR] 输出拍数超出预期 %0d", o_beat);
                    err_cnt++;
                end
                o_beat++;
            end
        end
    end

    // ------------------------------------------------------------------
    // 主流程
    // ------------------------------------------------------------------
    int i;
    initial begin
        s_valid = 1'b0; s_last = 1'b0; s_user = 1'b0; s_data = '0; s_ch = '0;
        cfg_bypass = 1'b0;
        cfg_wall_n = 16'(WALL_N);      // 与 DUT 的 WALL_N 一致，否则 proto_err
        build_expectations();

        repeat (8) @(posedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);

        // ================= T1：正常差分 + 反压 =================
        $display("[INFO] T1 正常差分（%0d 拍入 / %0d 拍出），并对 tready 施加周期反压", NIN, NOUT);
        for (i = 0; i < NOUT; i++) exp_ref[i] = exp_diff[i];
        o_is_bypass = 1'b0;
        o_beat      = 0;
        o_chk_en    = 1'b1;
        bp_en       = 1'b1;            // 打开反压，打 stall 路径
        send_frame(1'b0);
        wait (frame_done === 1'b1);
        repeat (8) @(posedge clk);
        o_chk_en = 1'b0;
        bp_en    = 1'b0;

        if (o_beat != NOUT) begin
            $display("[ERR] T1 输出拍数 %0d，期望 %0d", o_beat, NOUT); err_cnt++;
        end
        if (ovf_diff  !== 1'b0) begin $display("[ERR] T1 ovf_diff 被置起");  err_cnt++; end
        if (proto_err !== 1'b0) begin $display("[ERR] T1 proto_err 被置起"); err_cnt++; end

        // ================= T2：旁路模式 =================
        $display("[INFO] T2 旁路模式（cfg_bypass=1，%0d 拍出）", NOUT_BP);
        cfg_bypass = 1'b1;
        for (i = 0; i < NOUT_BP; i++) exp_ref[i] = exp_byp[i];
        o_is_bypass = 1'b1;
        o_beat      = 0;
        o_chk_en    = 1'b1;
        send_frame(1'b0);
        wait (frame_done === 1'b1);
        repeat (8) @(posedge clk);
        o_chk_en = 1'b0;

        if (o_beat != NOUT_BP) begin
            $display("[ERR] T2 输出拍数 %0d，期望 %0d", o_beat, NOUT_BP); err_cnt++;
        end
        cfg_bypass = 1'b0;

        // ================= T3：故障注入（tch 串位）=================
        $display("[INFO] T3 故障注入：第 0 拍 tch 故意送错，期望 proto_err 置起");
        o_chk_en = 1'b0;               // 本用例不比对数据，只看告警
        send_frame(1'b1);
        repeat (64) @(posedge clk);
        if (proto_err !== 1'b1) begin
            $display("[ERR] T3 proto_err 未被置起：通道串位没有被顶出来");
            err_cnt++;
        end else begin
            $display("[INFO] T3 proto_err 正确置起（粘滞）");
        end

        // ================= 汇总 =================
        if (err_cnt == 0)
            $display("=== vertical_diff_tb : PASS ===  (比对点 %0d 个)", chk_cnt);
        else
            $display("=== vertical_diff_tb : FAIL ===  错误 %0d 处 / 比对点 %0d 个",
                     err_cnt, chk_cnt);
        $finish;
    end

    initial begin
        #2000000;
        $display("[ERR] 超时");
        $display("=== vertical_diff_tb : FAIL (timeout) ===");
        $finish;
    end

endmodule

`default_nettype wire
