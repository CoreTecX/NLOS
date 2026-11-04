// ============================================================================
// raw_preproc_full_tb.sv  ·  raw_preproc **全尺寸**（B 档 1920x1920）跑通台
// ---------------------------------------------------------------------------
// 为什么单独一个 TB
// ---------------------------------------------------------------------------
//   raw_preproc_tb 是小尺寸（WALL_N=4 / BLOCK=2 -> 16x16），覆盖概念但从不触及
//   全尺寸的地址链与计数器值域。`sim/results/xsim_first_run_20260929.md` §8 与
//   `pl_t2_rebuild_and_seam_20260929.md` §七 第 2 条都记「全尺寸 xsim 未跑」。
//   本 TB 就是补那一条：B 档几何（BLOCK=10 / WALL_N=96），RAW_SIDE=1920，NFRM=4。
//
// 为什么不用「先建期望数组、再发激励」的写法
// ---------------------------------------------------------------------------
//   全尺寸下 4 帧共 14,745,600 个像素。若像小尺寸 TB 那样先跑一遍 20x20x96x96x4
//   的参考循环，xsim 会为同一个像素算两次、且参考循环本身就要上千万次嵌套迭代。
//   本 TB 改为**把期望累加融合进发送循环**：每发一个像素就同时把它加到对应块/通道
//   的期望里。省掉一整趟参考扫描，也省掉存放帧的数组（像素由 pix_of 现算）。
//
// 判据（逐值严格相等；P1：预处理链不做除法、不做舍入）
// ---------------------------------------------------------------------------
//   输出 96x96x3 = 27,648 拍，每拍：
//     m_meas_tdata  == exp_c[blk][ch]
//     m_meas_tch    == ch
//     m_meas_tlast  == (ch==2 && 块列==WALL_N-1)
//     m_meas_tuser  == (beat==0)
//   另核 ovf_sticky / proto_err / cfg_err 全 0、输出拍数恰为 27,648。
//
// 与 bit-exact 对拍台的分工
// ---------------------------------------------------------------------------
//   `raw_preproc_bitexact_tb.sv` 走文件、期望在 Python 侧（对拍）；
//   本 TB 的期望在 SV 内（自检），只做**全尺寸跑通 + 全量逐值**，两者互补。
//
// 已实测的可行性（见 sim/results/preboard_test_gap_20260930.md §任务6）
// ---------------------------------------------------------------------------
//   本机实测：单次运行墙钟时间与峰值内存记录在该报告里。若耗时或内存超出可接受
//   范围，报告同时给出分块/抽帧替代方案（NFRM=1 或按块行切片）。
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module raw_preproc_full_tb;

    localparam int WALL_N   = 96;
    localparam int BLOCK    = 10;
    localparam int IN_BITS  = 16;
    localparam int V_BITS   = 12;
    localparam int MAXF     = 8;
    localparam int NFRM     = 4;
    localparam int RAW_SIDE = 2 * BLOCK * WALL_N;   // 1920
    localparam int BLK_SIDE = 2 * BLOCK;            // 20
    localparam int NBLK     = WALL_N * WALL_N;      // 9216
    localparam int BW       = $clog2(NBLK);         // 14

    logic clk = 1'b0;
    always #5 clk = ~clk;                            // 100 MHz 仿真时钟
    logic rst_n = 1'b0;

    int err_cnt = 0;
    int chk_cnt = 0;

    // 期望块和：只存这一张表（9216 x 3 x 32 bit ≈ 110 KB），不发帧数组。
    logic [31:0] exp_c [0:NBLK-1][0:2];

    function automatic logic [IN_BITS-1:0] pix_of(input int f, input int r, input int c);
        int v;
        v = (r*37 + c*101 + f*197) % 4093;           // packed 12 bit 内
        pix_of = IN_BITS'(v);
    endfunction

    // ------------------------------------------------------------------ DUT
    logic                s_tvalid, s_tready, s_tlast, s_tuser;
    logic [IN_BITS-1:0]  s_tdata;
    logic [23:0]         m_tdata;
    logic [1:0]          m_tch;
    logic                m_tvalid, m_tready, m_tlast, m_tuser, frame_done;
    logic                busy, ovf, proto, cfg_err;
    logic [BW-1:0]       m_tblk;
    logic [IN_BITS-1:0]  cfg_dark;
    logic [3:0]          cfg_frames;

    raw_preproc #(
        .IN_BITS(IN_BITS), .VALID_BITS(V_BITS), .BLOCK(BLOCK), .WALL_N(WALL_N),
        .CH(3), .COLOR(1'b1), .MAX_FRAMES(MAXF),
        .BS_BITS(20), .ACC_BITS(23), .OUT_BITS(24)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .cfg_dark_level(cfg_dark), .cfg_frames(cfg_frames),
        .cfg_crop_side(16'(RAW_SIDE)), .cfg_bayer_phase(1'b0),
        .s_axis_tdata(s_tdata), .s_axis_tvalid(s_tvalid), .s_axis_tready(s_tready),
        .s_axis_tlast(s_tlast), .s_axis_tuser(s_tuser),
        .m_meas_tdata(m_tdata), .m_meas_tch(m_tch), .m_meas_tvalid(m_tvalid),
        .m_meas_tready(m_tready), .m_meas_tlast(m_tlast), .m_meas_tuser(m_tuser),
        .m_meas_tblk(m_tblk),
        .frame_done(frame_done), .busy(busy), .ovf_sticky(ovf),
        .proto_err(proto), .cfg_err(cfg_err),
        .sat_pix_cnt(), .total_pix_cnt()
    );

    // ------------------------------------------------------------------ 输出逐值比对
    int beat = 0;
    always @(posedge clk) begin
        if (m_tvalid && m_tready) begin
            if (beat < NBLK*3) begin
                chk_cnt++;
                if (m_tdata !== 24'(exp_c[beat/3][beat%3])) begin
                    if (err_cnt < 8)
                        $display("[ERR] beat=%0d blk=%0d ch=%0d got=%0d exp=%0d",
                                 beat, beat/3, beat%3, m_tdata, exp_c[beat/3][beat%3]);
                    err_cnt++;
                end
                if (m_tch !== 2'(beat%3)) begin
                    if (err_cnt < 8) $display("[ERR] beat=%0d tch=%0d", beat, m_tch);
                    err_cnt++;
                end
                if (m_tlast !== (((beat%3)==2) && (((beat/3)%WALL_N)==(WALL_N-1)))) begin
                    if (err_cnt < 8) $display("[ERR] beat=%0d tlast 位置错误", beat);
                    err_cnt++;
                end
                if (m_tuser !== (beat==0)) begin
                    if (err_cnt < 8) $display("[ERR] beat=%0d tuser 位置错误", beat);
                    err_cnt++;
                end
            end else begin
                if (err_cnt < 8) $display("[ERR] 输出拍数超出预期 %0d", beat);
                err_cnt++;
            end
            beat++;
        end
    end

    // ------------------------------------------------------------------ 激励
    task automatic send_pixel(input logic [IN_BITS-1:0] d,
                              input bit is_first, input bit is_last);
        s_tdata  = d;
        s_tuser  = is_first;
        s_tlast  = is_last;
        s_tvalid = 1'b1;
        forever begin
            @(negedge clk);
            if (s_tready === 1'b1) begin
                @(posedge clk);
                break;
            end
        end
    endtask

    // ------------------------------------------------------------------ 主流程
    int f, r, c, blk, ri, ci, br, bc;
    logic [31:0] pix;
    time t_start, t_end;

    initial begin
        s_tvalid = 1'b0; s_tlast = 1'b0; s_tuser = 1'b0; s_tdata = '0;
        m_tready = 1'b1;
        cfg_dark = 16'd0;
        cfg_frames = 4'(NFRM);

        for (blk = 0; blk < NBLK; blk++) begin
            exp_c[blk][0] = 32'd0;
            exp_c[blk][1] = 32'd0;
            exp_c[blk][2] = 32'd0;
        end

        repeat (8) @(posedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);

        $display("[INFO] 全尺寸：WALL_N=%0d BLOCK=%0d RAW_SIDE=%0d NFRM=%0d",
                 WALL_N, BLOCK, RAW_SIDE, NFRM);
        t_start = $time;

        for (f = 0; f < NFRM; f++) begin
            for (r = 0; r < RAW_SIDE; r++) begin
                for (c = 0; c < RAW_SIDE; c++) begin
                    pix = 32'(pix_of(f, r, c));
                    // 期望累加（与发送融合，省掉第二趟参考扫描）
                    br = r / BLK_SIDE;  ri = r % BLK_SIDE;
                    bc = c / BLK_SIDE;  ci = c % BLK_SIDE;
                    blk = br*WALL_N + bc;
                    if ((ri % 2 == 0) && (ci % 2 == 0))      exp_c[blk][0] = exp_c[blk][0] + pix;
                    else if ((ri % 2 == 1) && (ci % 2 == 1)) exp_c[blk][2] = exp_c[blk][2] + pix;
                    else                                     exp_c[blk][1] = exp_c[blk][1] + pix;

                    send_pixel(IN_BITS'(pix), (r == 0) && (c == 0), (c == RAW_SIDE-1));
                end
            end
        end
        s_tvalid = 1'b0; s_tlast = 1'b0; s_tuser = 1'b0;

        wait (frame_done === 1'b1);
        repeat (4) @(posedge clk);
        t_end = $time;

        if (beat != NBLK*3) begin
            $display("[ERR] 输出拍数 %0d，期望 %0d", beat, NBLK*3); err_cnt++;
        end
        if (ovf      !== 1'b0) begin $display("[ERR] ovf_sticky 置起"); err_cnt++; end
        if (proto    !== 1'b0) begin $display("[ERR] proto_err 置起");  err_cnt++; end
        if (cfg_err  !== 1'b0) begin $display("[ERR] cfg_err 置起");    err_cnt++; end

        $display("[INFO] 输出拍数=%0d 比对点=%0d 仿真时间=%0t ns（墙钟见运行脚本）",
                 beat, chk_cnt, t_end - t_start);
        if (err_cnt == 0)
            $display("=== raw_preproc_full_tb : PASS ===  (比对点 %0d 个)", chk_cnt);
        else
            $display("=== raw_preproc_full_tb : FAIL ===  错误 %0d 处 / 比对点 %0d 个",
                     err_cnt, chk_cnt);
        $finish;
    end

    // 看门狗：全尺寸预期约 1.5e7 拍（=1.5e8 ns）。留 2 倍余量。
    initial begin
        #300000000;
        $display("[ERR] 超时");
        $display("=== raw_preproc_full_tb : FAIL (timeout) ===");
        $finish;
    end

endmodule

`default_nettype wire
