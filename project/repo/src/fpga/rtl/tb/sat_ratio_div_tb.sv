// ============================================================================
// tb/sat_ratio_div_tb.sv · sat_ratio_div（饱和像素比例除法器）自检测试平台
// ---------------------------------------------------------------------------
// 覆盖内容（每一项都对应 sat_ratio_div.v 文件头里的一条约定）
// ---------------------------------------------------------------------------
//   T1 total==0 时**不发布**（保持 0 / valid 0）—— 对应文件头「不清零就冲掉正确值」那条。
//   T2 普通比例：floor(sat*2^16/total) 逐步核对（含 floor 与「整除」两类）。
//   T3 0.1% 边界：sat/total = 66/65536 -> 66；67/65536 -> 67（CE 的阈值是 66）。
//   T4 sat == total -> 商恰为 2^16，超出 16 bit，**饱和到 0xFFFF**。
//   T5 B 档量级：sat=1,000,000 / total=14,745,600（3,686,400 x 4 帧）-> 4444。
//   T6 清零保持：发布后再把两个计数清零，比例**保持**上一次发布值、valid 仍为 1
//      （这是 raw_preproc 在 ST_DONE 清零、frame_done 落在清零之后的直接后果）。
//   T7 全程变更中途改输入：除法跑到一半改输入 -> 本次作废，最终只发布「新输入对」的值。
//   T8 连续变化（模拟 ST_ACC 每拍自增）：一段时间内不产生任何「中间值」；
//      固定下来后发布的正是稳定值。
//   T9 复位：清零 ratio/valid/busy。
//
// 另有一条**逐拍不变量**由监视器检查（不计入比对点，理由与 axis_frame_sync_tb 相同：
// 计入会让点数随激励拍数漂，而本 TB 的激励拍数本身由实现细节决定）：
//   每当 ratio_q016 **发生变化**，它前面必须有一段 >= 47 拍的输入稳定窗。
//   违反即 +err_cnt（这正是「拿半途的计数发布」这种错的表现）。
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module sat_ratio_div_tb;

    localparam int IW = 32;
    localparam int OW = 16;

    logic clk = 1'b0;
    always #5 clk = ~clk;
    logic rst_n = 1'b0;

    logic [IW-1:0] sat_cnt   = '0;
    logic [IW-1:0] total_cnt = '0;
    logic [OW-1:0] ratio_q016;
    logic          ratio_valid;
    logic          busy;

    int err_cnt = 0;
    int chk_cnt = 0;

    sat_ratio_div #(.IN_W(IW), .OUT_W(OW)) dut (
        .clk(clk), .rst_n(rst_n),
        .sat_cnt(sat_cnt), .total_cnt(total_cnt),
        .ratio_q016(ratio_q016), .ratio_valid(ratio_valid), .busy(busy)
    );

    // ------------------------------------------------------------------
    // 逐拍不变量监视器（只计 err，不计 chk）
    //   stable_cnt = 连续「本拍计数对 == 上一拍计数对」的拍数
    // ------------------------------------------------------------------
    logic [2*IW-1:0] pair_d      = '0;
    logic [7:0]      stable_cnt  = '0;
    logic [OW-1:0]   ratio_d     = '0;
    int              pub_cnt     = 0;

    always @(posedge clk) begin
        if (!rst_n) begin
            pair_d     <= '0;
            stable_cnt <= '0;
            ratio_d    <= '0;
            pub_cnt    <= 0;
        end else begin
            pair_d <= {sat_cnt, total_cnt};
            if ({sat_cnt, total_cnt} == pair_d)
                stable_cnt <= (stable_cnt == 8'hFF) ? 8'hFF : (stable_cnt + 8'd1);
            else
                stable_cnt <= 8'd0;

            ratio_d <= ratio_q016;
            if (ratio_q016 !== ratio_d) begin
                pub_cnt = pub_cnt + 1;
                if (stable_cnt < 8'd47) begin
                    $display("[ERR] 发布了未经 ~48 拍稳定窗的比例：value=%0d stable_cnt=%0d @%0t",
                             ratio_q016, stable_cnt, $time);
                    err_cnt++;
                end
            end
        end
    end

    // ------------------------------------------------------------------
    task automatic chk16(input [OW-1:0] got, input [OW-1:0] exp, input [127:0] tag);
        chk_cnt++;
        if (got !== exp) begin
            $display("[ERR] %0s: got=%0d (0x%04x) exp=%0d (0x%04x)", tag, got, got, exp, exp);
            err_cnt++;
        end
    endtask

    task automatic chk1(input logic got, input logic exp, input [127:0] tag);
        chk_cnt++;
        if (got !== exp) begin
            $display("[ERR] %0s: got=%b exp=%b", tag, got, exp);
            err_cnt++;
        end
    endtask

    task automatic set_counts(input [IW-1:0] s, input [IW-1:0] t);
        sat_cnt   = s;
        total_cnt = t;
    endtask

    task automatic settle(input int n);
        repeat (n) @(posedge clk);
    endtask

    // ------------------------------------------------------------------
    initial begin
        $display("[INFO] T1 total==0 不发布");
        settle(6);
        rst_n = 1'b1;
        settle(4);
        chk1(ratio_valid, 1'b0, "T1 valid=0");
        chk16(ratio_q016, 16'd0, "T1 ratio=0");
        chk1(busy, 1'b0, "T1 busy=0");

        // ---------------- T2 普通比例 ----------------
        $display("[INFO] T2 floor 口径与整除口径");
        set_counts(32'd432, 32'd921600);           // 432*65536/921600 = 30.72 -> 30
        settle(60);
        chk16(ratio_q016, 16'd30, "T2 432/921600 -> 30");
        chk1(ratio_valid, 1'b1, "T2 valid=1");

        set_counts(32'd1000, 32'd4000);            // 1000*65536/4000 = 16384（整除）
        settle(60);
        chk16(ratio_q016, 16'd16384, "T2 1000/4000 -> 16384");

        set_counts(32'd3, 32'd100000);             // 3*65536/100000 = 1.966 -> 1（floor）
        settle(60);
        chk16(ratio_q016, 16'd1, "T2 3/100000 -> 1");

        // ---------------- T3 0.1% 边界（CE 阈值 sat_max = 66）----------------
        $display("[INFO] T3 0.1%% 边界（66/65536 恰为阈值）");
        set_counts(32'd66, 32'd65536);
        settle(60);
        chk16(ratio_q016, 16'd66, "T3 66/65536 -> 66（== 阈值，不超）");
        set_counts(32'd67, 32'd65536);
        settle(60);
        chk16(ratio_q016, 16'd67, "T3 67/65536 -> 67（> 阈值）");

        // ---------------- T4 sat == total -> 饱和 ----------------
        $display("[INFO] T4 sat==total 饱和到 0xFFFF");
        set_counts(32'd1000, 32'd1000);
        settle(60);
        chk16(ratio_q016, 16'hFFFF, "T4 1000/1000 -> 0xFFFF");

        // ---------------- T5 B 档量级 ----------------
        $display("[INFO] T5 B 档量级（饱和像素百万级 / 一组 14.7M 像素）");
        set_counts(32'd1000000, 32'd14745600);     // -> 4444
        settle(60);
        chk16(ratio_q016, 16'd4444, "T5 1e6/14,745,600 -> 4444");

        // ---------------- T6 清零保持 ----------------
        $display("[INFO] T6 清零保持（模拟 raw_preproc 在 ST_DONE 清零两个计数）");
        set_counts(32'd2000000, 32'd14745600);     // -> 8888
        settle(60);
        chk16(ratio_q016, 16'd8888, "T6 2e6/14,745,600 -> 8888");
        set_counts(32'd0, 32'd0);                  // 清零
        settle(120);
        chk16(ratio_q016, 16'd8888, "T6 清零后比例保持");
        chk1(ratio_valid, 1'b1, "T6 清零后 valid 仍为 1");
        chk1(busy, 1'b0, "T6 清零后 busy=0");

        // ---------------- T7 除法中途改输入 -> 作废 ----------------
        $display("[INFO] T7 除法跑到一半改输入，本次作废");
        set_counts(32'd1000, 32'd2000);            // 若发布会是 0x8000
        settle(10);                                // 仍在 48 拍除法里
        set_counts(32'd100, 32'd100000);           // 100*65536/100000 = 65.536 -> 65
        settle(90);
        chk16(ratio_q016, 16'd65, "T7 只发布新输入对的值 65");

        // ---------------- T8 连续变化（模拟 ST_ACC）----------------
        $display("[INFO] T8 连续变化期间不发布中间值");
        for (int i = 0; i < 200; i++) begin
            sat_cnt = 32'd100 + i;
            @(posedge clk);
        end
        chk16(ratio_q016, 16'd65, "T8 变化期间保持上一个已发布值");
        set_counts(32'd300, 32'd100000);           // 300*65536/100000 = 196.608 -> 196
        settle(90);
        chk16(ratio_q016, 16'd196, "T8 稳定后发布 196");

        // ---------------- T9 复位 ----------------
        $display("[INFO] T9 复位清零");
        rst_n = 1'b0;
        settle(4);
        chk16(ratio_q016, 16'd0, "T9 复位后 ratio=0");
        chk1(ratio_valid, 1'b0, "T9 复位后 valid=0");
        chk1(busy, 1'b0, "T9 复位后 busy=0");
        rst_n = 1'b1;
        settle(4);

        // ---------------- 汇总 ----------------
        repeat (4) @(posedge clk);
        if (err_cnt == 0)
            $display("=== sat_ratio_div_tb : PASS ===  (比对点 %0d 个)", chk_cnt);
        else
            $display("=== sat_ratio_div_tb : FAIL ===  错误 %0d 处 / 比对点 %0d 个",
                     err_cnt, chk_cnt);
        $finish;
    end

    initial begin
        #2000000;
        $display("[ERR] 超时");
        $display("=== sat_ratio_div_tb : FAIL (timeout) ===");
        $finish;
    end

endmodule

`default_nettype wire
