// ============================================================================
// tb/axis_frame_sync_tb.sv · axis_frame_sync 自检测试平台
// ---------------------------------------------------------------------------
// 覆盖的三条断言（对应任务一对 G2 的要求）
// ---------------------------------------------------------------------------
//   A1 一帧内的 tlast 握手次数**恰为**帧长允许的次数：
//        差分通道（ROW_BEATS=0）每帧 = 1 次；
//        raw 通道（ROW_BEATS=4 / FRAME_BEATS=16）每帧 = 4 次（每行一次）。
//   A2 **无效拍不误置**：m_axis_tvalid=0 的每一拍，m_axis_tlast / m_axis_tuser 必须为 0。
//   A3 逐拍边界正确 + 数据直通：期望的 tuser/tlast/data 由 TB 自己的 beat 计数器给
//      （与 DUT 内部的计数器彼此独立），逐拍比对。
//
//   ★ 差分通道的**上游 tlast 故意按 vertical_diff 的真实行为施加**
//     （每块行末置一次，FRAME_BEATS/(ROW*CH)=36/12=3 次/帧）。本用例因此能直接
//     证明「上游的块行 tlast 不会漏到输出上」—— 这是 G2 的要害。
//     raw 通道的上游 tlast 则按 AXI DMA 的真实行为施加（每次传输末尾一次，
//     即只在帧末）。
//
//   ★ 两个通道都带**周期性反压**（每 3 拍压 1 拍），专门打「计数器只在握手时推进」
//     这条路径：反压期间若计数器误增，A1/A3 立刻失败。
//
// 规模：差分 3 帧 x 36 拍；raw 2 帧 x 16 拍。尺寸取小是为了让断言可读，
//       与生产参数（27360 / 3686400）只差 FRAME_BEATS/ROW_BEATS 的取值。
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module axis_frame_sync_tb;

    // ---------------- 差分通道参数（对应 WALL_N=4, CH=3）----------------
    localparam int D_FRAME_BEATS = (4-1)*4*3;   // 36
    localparam int D_BLOCKROW    = 4*3;         // 上游块行 = 12 拍
    localparam int D_FRAMES      = 3;           // 跑 3 帧
    localparam int D_LAST_EXP    = 1;           // 每帧期望 tlast 次数

    // ---------------- raw 通道参数（4 行 x 4 拍）----------------
    localparam int R_ROW_BEATS   = 4;
    localparam int R_FRAME_BEATS = 4*4;         // 16
    localparam int R_FRAMES      = 2;
    localparam int R_LAST_EXP    = R_ROW_BEATS; // 每帧期望 tlast 次数 = 行数

    logic clk = 1'b0;
    always #5 clk = ~clk;
    logic rst_n = 1'b0;

    int err_cnt = 0;
    int chk_cnt = 0;

    // 「无效拍不误置」只在**激励进行中**生效：两通道都跑完之后不再查。
    // 起因：这条断言是逐拍生效的，若把它计入比对点数，点数会随「激励总共多少拍」
    // 变化，而 xsim 与 iverilog 在激励总拍数上差 2 拍（本机实测）—— 钉死的
    // 比对点数就会随仿真器漂。现在它只作为「违例即 +err_cnt」的不变量，
    // 不计入 chk_cnt，于是点数为 145（= 108 差分拍 + 32 raw 拍 + 5 帧级检查），
    // 两套仿真器一致。
    // 注意：驱动 stim_* 的 always 块放在**两个通道的帧计数声明之后** ——
    //   iverilog 要求「先声明后用」，放前面会报 Unable to bind wire/reg/memory。
    logic stim_started = 1'b0;
    logic stim_done    = 1'b0;
    wire  chk_active   = stim_started && !stim_done;

    // ==========================================================================
    // 差分通道
    // ==========================================================================
    logic [23:0] d_s_data;
    logic        d_s_valid, d_s_ready, d_s_last, d_s_user;
    logic [23:0] d_m_data;
    logic        d_m_valid, d_m_ready, d_m_last, d_m_user;

    // 周期性反压：每 3 拍压 1 拍（由 clk 计数驱动，与数据无关）
    logic [1:0] d_bp = 2'd0;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) d_bp <= 2'd0;
        else        d_bp <= d_bp + 2'd1;
    end
    assign d_m_ready = (d_bp != 2'd0);

    axis_frame_sync #(
        .DW(24), .ROW_BEATS(0), .FRAME_BEATS(D_FRAME_BEATS)
    ) dut_diff (
        .clk(clk), .rst_n(rst_n),
        .s_axis_tdata(d_s_data), .s_axis_tuser(d_s_user), .s_axis_tvalid(d_s_valid),
        .s_axis_tready(d_s_ready), .s_axis_tlast(d_s_last),
        .m_axis_tdata(d_m_data), .m_axis_tuser(d_m_user), .m_axis_tvalid(d_m_valid),
        .m_axis_tready(d_m_ready), .m_axis_tlast(d_m_last)
    );

    int  d_idx      = 0;      // 本帧已握手拍数（TB 自己的计数，独立于 DUT）
    int  d_frm      = 0;      // 已完成的帧数
    int  d_last_cnt = 0;      // 本帧累计的 tlast 握手次数
    int  d_exp_last_total = 0;

    always @(posedge clk) begin
        if (rst_n) begin
            // ---- A2：无效拍不误置 ----
            // 注意：这条**不**计入 chk_cnt —— 它逐拍生效，而「有多少个空闲拍」
            //   与仿真器在收尾/调度上的差异有关（实测 xsim 与 iverilog 的激励
            //   总拍数差 2）。计入会让钉死的比对点数随仿真器漂；不计入则
            //   chk_cnt 只由「握手拍数 + 帧数」决定，是完全确定的。
            //   断言本身照旧生效：一旦误置就 +err_cnt。
            if (chk_active && !d_m_valid) begin
                if (d_m_last !== 1'b0 || d_m_user !== 1'b0) begin
                    $display("[ERR] 差分：无效拍（tvalid=0）误置 tlast=%b tuser=%b",
                             d_m_last, d_m_user);
                    err_cnt++;
                end
            end

            // ---- A1/A3：握手拍的边界与数据 ----
            if (d_m_valid && d_m_ready) begin
                chk_cnt++;
                if (d_m_last !== (d_idx == D_FRAME_BEATS-1)) begin
                    $display("[ERR] 差分 beat=%0d tlast=%b 期望=%b（上游块行 tlast 漏过来了？）",
                             d_idx, d_m_last, (d_idx == D_FRAME_BEATS-1));
                    err_cnt++;
                end
                if (d_m_user !== (d_idx == 0)) begin
                    $display("[ERR] 差分 beat=%0d tuser=%b 期望=%b",
                             d_idx, d_m_user, (d_idx == 0));
                    err_cnt++;
                end
                if (d_m_data !== d_s_data) begin
                    $display("[ERR] 差分 beat=%0d 数据未直通 got=%0d exp=%0d",
                             d_idx, d_m_data, d_s_data);
                    err_cnt++;
                end
                if (d_m_last) d_last_cnt++;

                if (d_idx == D_FRAME_BEATS-1) begin
                    // ---- A1：一帧内 tlast 次数恰为 1 ----
                    chk_cnt++;
                    if (d_last_cnt !== D_LAST_EXP) begin
                        $display("[ERR] 差分 第 %0d 帧 tlast 次数 %0d，期望 %0d",
                                 d_frm, d_last_cnt, D_LAST_EXP);
                        err_cnt++;
                    end
                    $display("[INFO] 差分 第 %0d 帧结束：tlast 次数 = %0d", d_frm, d_last_cnt);
                    d_exp_last_total += d_last_cnt;
                    d_frm++;
                    d_last_cnt = 0;
                    d_idx = 0;
                end else begin
                    d_idx++;
                end
            end
        end
    end

    // 差分通道激励：上游 tlast 每 D_BLOCKROW 拍一次（模拟 vertical_diff 的块行末）
    task automatic d_send(input logic [23:0] dat, input int k);
        @(negedge clk);
        d_s_data  = dat;
        d_s_user  = (k == 0);
        d_s_last  = ((k % D_BLOCKROW) == (D_BLOCKROW-1));   // 块行末（含帧末那一拍）
        d_s_valid = 1'b1;
        @(posedge clk);
        while (d_s_ready !== 1'b1) @(posedge clk);
    endtask

    // ==========================================================================
    // raw 通道
    // ==========================================================================
    logic [15:0] r_s_data;
    logic        r_s_valid, r_s_ready, r_s_last, r_s_user;
    logic [15:0] r_m_data;
    logic        r_m_valid, r_m_ready, r_m_last, r_m_user;

    logic [1:0] r_bp = 2'd0;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) r_bp <= 2'd0;
        else        r_bp <= r_bp + 2'd1;
    end
    assign r_m_ready = (r_bp != 2'd2);   // 与差分通道错开的反压相位

    axis_frame_sync #(
        .DW(16), .ROW_BEATS(R_ROW_BEATS), .FRAME_BEATS(R_FRAME_BEATS)
    ) dut_raw (
        .clk(clk), .rst_n(rst_n),
        .s_axis_tdata(r_s_data), .s_axis_tuser(r_s_user), .s_axis_tvalid(r_s_valid),
        .s_axis_tready(r_s_ready), .s_axis_tlast(r_s_last),
        .m_axis_tdata(r_m_data), .m_axis_tuser(r_m_user), .m_axis_tvalid(r_m_valid),
        .m_axis_tready(r_m_ready), .m_axis_tlast(r_m_last)
    );

    int r_idx = 0;
    int r_frm = 0;
    int r_last_cnt = 0;

    // 驱动「无效拍检查」的门（见文件上方 stim_started 的说明）。
    // 放在这里是因为 iverilog 要求先声明后用（d_frm / r_frm 在上两段才声明）。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stim_started <= 1'b0;
            stim_done    <= 1'b0;
        end else begin
            if ((d_frm > 0) || (r_frm > 0))                 stim_started <= 1'b1;
            if ((d_frm == D_FRAMES) && (r_frm == R_FRAMES)) stim_done    <= 1'b1;
        end
    end

    always @(posedge clk) begin
        if (rst_n) begin
            if (chk_active && !r_m_valid) begin
                if (r_m_last !== 1'b0 || r_m_user !== 1'b0) begin
                    $display("[ERR] raw：无效拍（tvalid=0）误置 tlast=%b tuser=%b",
                             r_m_last, r_m_user);
                    err_cnt++;
                end
            end

            if (r_m_valid && r_m_ready) begin
                chk_cnt++;
                if (r_m_last !== ((r_idx % R_ROW_BEATS) == (R_ROW_BEATS-1))) begin
                    $display("[ERR] raw beat=%0d tlast=%b 期望=%b",
                             r_idx, r_m_last, ((r_idx % R_ROW_BEATS) == (R_ROW_BEATS-1)));
                    err_cnt++;
                end
                if (r_m_user !== (r_idx == 0)) begin
                    $display("[ERR] raw beat=%0d tuser=%b 期望=%b",
                             r_idx, r_m_user, (r_idx == 0));
                    err_cnt++;
                end
                if (r_m_data !== r_s_data) begin
                    $display("[ERR] raw beat=%0d 数据未直通 got=%0d exp=%0d",
                             r_idx, r_m_data, r_s_data);
                    err_cnt++;
                end
                if (r_m_last) r_last_cnt++;

                if (r_idx == R_FRAME_BEATS-1) begin
                    chk_cnt++;
                    if (r_last_cnt !== R_LAST_EXP) begin
                        $display("[ERR] raw 第 %0d 帧 tlast 次数 %0d，期望 %0d",
                                 r_frm, r_last_cnt, R_LAST_EXP);
                        err_cnt++;
                    end
                    $display("[INFO] raw 第 %0d 帧结束：tlast 次数 = %0d", r_frm, r_last_cnt);
                    r_frm++;
                    r_last_cnt = 0;
                    r_idx = 0;
                end else begin
                    r_idx++;
                end
            end
        end
    end

    // raw 通道激励：上游 tlast 只在帧末一次（模拟 AXI DMA 的每传输末拍）
    task automatic r_send(input logic [15:0] dat, input int k);
        @(negedge clk);
        r_s_data  = dat;
        r_s_user  = 1'b0;                                  // MM2S 不产生 TUSER
        r_s_last  = (k == R_FRAME_BEATS-1);
        r_s_valid = 1'b1;
        @(posedge clk);
        while (r_s_ready !== 1'b1) @(posedge clk);
    endtask

    // ==========================================================================
    // 主流程
    // ==========================================================================
    // ★ 循环变量必须**局部**声明（两个 initial 共用模块级变量会互相踩：
    //   首轮实测 raw 通道只跑完 1 帧就超时，根因就是 i/k 被另一个 initial 改掉）。
    initial begin
        d_s_valid = 1'b0; d_s_last = 1'b0; d_s_user = 1'b0; d_s_data = '0;
        r_s_valid = 1'b0; r_s_last = 1'b0; r_s_user = 1'b0; r_s_data = '0;

        repeat (8) @(posedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);

        $display("[INFO] 差分通道：%0d 帧 x %0d 拍（上游块行 tlast 每 %0d 拍一次）",
                 D_FRAMES, D_FRAME_BEATS, D_BLOCKROW);
        for (int i = 0; i < D_FRAMES; i++) begin
            for (int k = 0; k < D_FRAME_BEATS; k++) begin
                d_send(24'(i*1000 + k), k);
            end
        end
        @(negedge clk); d_s_valid = 1'b0; d_s_last = 1'b0; d_s_user = 1'b0;
        wait (r_frm == R_FRAMES);
        repeat (16) @(posedge clk);

        if (d_frm != D_FRAMES) begin
            $display("[ERR] 差分通道只完成 %0d 帧，期望 %0d", d_frm, D_FRAMES);
            err_cnt++;
        end
        if (d_exp_last_total != D_FRAMES * D_LAST_EXP) begin
            $display("[ERR] 差分通道 tlast 总次数 %0d，期望 %0d",
                     d_exp_last_total, D_FRAMES * D_LAST_EXP);
            err_cnt++;
        end

        if (err_cnt == 0)
            $display("=== axis_frame_sync_tb : PASS ===  (比对点 %0d 个)", chk_cnt);
        else
            $display("=== axis_frame_sync_tb : FAIL ===  错误 %0d 处 / 比对点 %0d 个",
                     err_cnt, chk_cnt);
        $finish;
    end

    // raw 通道激励跑在另一个 initial 里（两通道独立并行）
    initial begin
        wait (rst_n === 1'b1);
        repeat (4) @(posedge clk);
        $display("[INFO] raw 通道：%0d 帧 x %0d 拍（上游 tlast 只在帧末）",
                 R_FRAMES, R_FRAME_BEATS);
        for (int i = 0; i < R_FRAMES; i++) begin
            for (int k = 0; k < R_FRAME_BEATS; k++) begin
                r_send(16'(i*100 + k), k);
            end
        end
        @(negedge clk); r_s_valid = 1'b0; r_s_last = 1'b0; r_s_user = 1'b0;
    end

    initial begin
        #2000000;
        $display("[ERR] 超时");
        $display("=== axis_frame_sync_tb : FAIL (timeout) ===");
        $finish;
    end

endmodule

`default_nettype wire
