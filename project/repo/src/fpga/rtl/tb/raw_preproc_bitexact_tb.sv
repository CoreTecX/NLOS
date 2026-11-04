// ============================================================================
// raw_preproc_bitexact_tb.sv  ·  raw_preproc 与 Python 定点参考链的 bit-exact 对拍台
// ---------------------------------------------------------------------------
// 为什么单独一个 TB（而不扩 raw_preproc_tb）
// ---------------------------------------------------------------------------
//   raw_preproc_tb 的期望值由 TB **自己**在 SV 里复刻一遍（自证），覆盖不了
//   RTL README 的 B4：「与 Python 定点参考模型逐向量 bit-exact」。本 TB 把
//   激励与结果都落到**文件**（双方都能读的中间格式），由 Python 侧生成输入、
//   独立算期望、再逐值比对。分开放，避免把「自证」与「对拍」混在一个文件里。
//
// 中间格式（纯文本，逐行十进制，双方都能读）
// ---------------------------------------------------------------------------
//   输入文件（Python 生成，TB 只读）：
//       第 1 行：   <NFRM> <DARK>
//       之后 NFRM*RAW_SIDE*RAW_SIDE 行：每行一个 RAW16 码字（0..65535），
//                 按帧 -> 行 -> 列的光栅序排列（与 AXI4-Stream 输入同序）。
//   输出文件（TB 写，Python 读）：
//       每个输出拍一行：<m_meas_tdata> <m_meas_tch> <m_meas_tlast> <m_meas_tuser> <m_meas_tblk>
//       顺序即流出顺序（块行 -> 块列 -> 通道）。
//
// 几何（与小尺寸自检 TB 保持一致，便于交叉核对）
// ---------------------------------------------------------------------------
//   COLOR=1 / BLOCK=2 / WALL_N=4 -> BLK_SIDE = 2*BLOCK = 4，RAW_SIDE = 16，
//   一帧 16x16=256 个 RAW16；输出 WALL_N*WALL_N*3 = 48 拍。
//   这不是全尺寸（1920x1920）；全尺寸的可行性与替代方案见
//   sim/results/preboard_test_gap_20260930.md §任务6。
//
// 本 TB 不做的事
// ---------------------------------------------------------------------------
//   · 不做任何独立断言：它只**导出**（激励进、结果出），判据在 Python 侧。
//     这样「期望值只有一份」（Python 那套），避免 SV 与 Python 两份期望各自漂移。
//   · 不测跨行错误口径、黑白路径、反压（那些在 raw_preproc_tb 里）。
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module raw_preproc_bitexact_tb;

    localparam int WALL_N   = 4;
    localparam int BLOCK    = 2;
    localparam int IN_BITS  = 16;
    localparam int V_BITS   = 12;
    localparam int MAXF     = 8;
    localparam int RAW_SIDE = 2 * BLOCK * WALL_N;   // 16
    localparam int BLK_SIDE = 2 * BLOCK;            // 4
    localparam int NPIX     = RAW_SIDE * RAW_SIDE;  // 256
    localparam int NBLK     = WALL_N * WALL_N;      // 16
    localparam int BW       = $clog2(NBLK);         // 4

    localparam int MAXN = MAXF * NPIX;              // 输入文件最大行数

    logic clk = 1'b0;
    always #5 clk = ~clk;
    logic rst_n = 1'b0;

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

    // ------------------------------------------------------------------ 激励文件
    logic [IN_BITS-1:0] pix_mem [0:MAXN-1];
    int   nfrm, dark_i, out_cnt, err_cnt;
    int   fd, ofd, code, i, k, idx, r, c;
    string in_path  = "bitexact_in.txt";
    string out_path = "bitexact_out.txt";

    // 输出落盘（每拍一行），同时计数
    always @(posedge clk) begin
        if (m_tvalid && m_tready && ofd != 0) begin
            $fwrite(ofd, "%0d %0d %0d %0d %0d\n",
                    m_tdata, m_tch, m_tlast, m_tuser, m_tblk);
            out_cnt++;
        end
    end

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

    initial begin
        s_tvalid = 1'b0; s_tlast = 1'b0; s_tuser = 1'b0; s_tdata = '0;
        m_tready = 1'b1;                  // 本 TB 不测反压，恒就绪

        if (!$value$plusargs("IN=%s", in_path))  in_path  = "bitexact_in.txt";
        if (!$value$plusargs("OUT=%s", out_path)) out_path = "bitexact_out.txt";

        fd = $fopen(in_path, "r");
        if (fd == 0) begin
            $display("[ERR] 打不开输入向量 %s", in_path);
            $display("=== raw_preproc_bitexact_tb : FAIL ===  (输入打不开)");
            $finish;
        end
        // 第 1 行：帧数 + 暗电平
        if ($fscanf(fd, "%d %d", nfrm, dark_i) != 2) begin
            $display("[ERR] 输入头（NFRM DARK）读不到");
            $display("=== raw_preproc_bitexact_tb : FAIL ===  (头部)");
            $finish;
        end
        if (!((nfrm == 1) || (nfrm == 4) || (nfrm == 8))) begin
            $display("[ERR] NFRM=%0d 非法（RTL 只接受 1/4/8）", nfrm);
            $display("=== raw_preproc_bitexact_tb : FAIL ===  (NFRM)");
            $finish;
        end
        for (i = 0; i < nfrm * NPIX; i++) begin
            if ($fscanf(fd, "%d", code) != 1) begin
                $display("[ERR] 输入向量第 %0d 个值读不到", i);
                $display("=== raw_preproc_bitexact_tb : FAIL ===  (短读)");
                $finish;
            end
            pix_mem[i] = IN_BITS'(code);
        end
        $fclose(fd);

        ofd = $fopen(out_path, "w");
        if (ofd == 0) begin
            $display("[ERR] 打不开输出文件 %s", out_path);
            $display("=== raw_preproc_bitexact_tb : FAIL ===  (输出打不开)");
            $finish;
        end

        cfg_dark   = IN_BITS'(dark_i);
        cfg_frames = 4'(nfrm);
        out_cnt    = 0;

        repeat (8) @(posedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);

        for (k = 0; k < nfrm; k++) begin
            idx = k * NPIX;
            for (r = 0; r < RAW_SIDE; r++) begin
                for (c = 0; c < RAW_SIDE; c++) begin
                    send_pixel(pix_mem[idx], (r == 0) && (c == 0), (c == RAW_SIDE-1));
                    idx++;
                end
            end
        end
        s_tvalid = 1'b0; s_tlast = 1'b0; s_tuser = 1'b0;

        wait (frame_done === 1'b1);
        repeat (4) @(posedge clk);
        $fclose(ofd);
        ofd = 0;

        $display("=== raw_preproc_bitexact_tb : DONE ===  n_out=%0d ovf=%b proto=%b cfg=%b dark=%0d nfrm=%0d",
                 out_cnt, ovf, proto, cfg_err, cfg_dark, nfrm);
        $finish;
    end

    initial begin
        #5000000;
        $display("[ERR] 超时");
        $display("=== raw_preproc_bitexact_tb : FAIL (timeout) ===");
        $finish;
    end

endmodule

`default_nettype wire
