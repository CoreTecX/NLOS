// ============================================================================
// tb/compositor_tready_tb.sv · compositor 的 G3 反压（vid_tready）自检
// ---------------------------------------------------------------------------
// 为什么单独一个 TB，而不是并进 compositor_tb
// ---------------------------------------------------------------------------
//   compositor_tb 的整帧金模型按 `vid_tvalid` 计拍（T1 数 de/tlast/tuser 拍数、
//   T2 用 k 计数推 (h,v)）；插入反压会让同一拍 tvalid 被数两次 → 判据失真。
//   所以反压行为另起一个**小而快**的 TB：只跑到第一条墙面行，验三件事。
//
// 验什么（每一拍都断言）
// ---------------------------------------------------------------------------
//   A. 进入墙面区（vid_tdata 高字节 = 0x80，来自预置的墙图 ramp）且 vid_tvalid=1；
//   B. tready=0 期间，输出**逐拍冻结**：tdata/tvalid 恒等，且 tlast/tuser/hsync/vsync
//      也不变 —— 即「同一拍像素被保持」，这正是 AXIS 的握手语义；
//   C. tready 拉高后，**先交付被冻结的那一拍**（不丢、不跳），随后光栅恢复推进。
//
// 判据：末尾 `=== compositor_tready_tb : PASS ===`，比对点写死在 .py 门禁里。
// 运行：
//   iverilog -g2012 -o compositor_tready_tb.vvp compositor_tready_tb.sv ../compositor.sv
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module compositor_tready_tb;

    localparam int NWALL = 96*96;   // 9216

    logic clk = 1'b0;
    always #5 clk = ~clk;
    logic rst_n = 1'b0;

    int err_cnt = 0;
    int chk_cnt = 0;

    // ---------------- DUT 端口 ----------------
    logic [23:0] vid_tdata;
    logic        vid_tvalid, vid_tlast, vid_tuser, vid_de, vid_hsync, vid_vsync;
    logic        vid_tready;
    logic        frame_tick, busy;

    logic        wall_we;
    logic [13:0] wall_waddr;
    logic [23:0] wall_wdata;
    logic        rec_we;
    logic [8:0]  rec_waddr;
    logic [23:0] rec_wdata;

    logic [15:0] cand_best_x, cand_best_y, cand_best_z, selected_id, conf_q016;
    logic        low_conf, force_gray;
    logic [4:0]  gray_reason;

    logic        osd_we;
    logic [7:0]  osd_waddr, osd_wdata;

    compositor dut (
        .clk(clk), .rst_n(rst_n),
        .vid_tdata(vid_tdata), .vid_tvalid(vid_tvalid), .vid_tlast(vid_tlast),
        .vid_tuser(vid_tuser), .vid_de(vid_de), .vid_hsync(vid_hsync),
        .vid_vsync(vid_vsync), .vid_tready(vid_tready),
        .wall_we(wall_we), .wall_waddr(wall_waddr), .wall_wdata(wall_wdata),
        .rec_we(rec_we), .rec_waddr(rec_waddr), .rec_wdata(rec_wdata),
        .cand_best_x(cand_best_x), .cand_best_y(cand_best_y), .cand_best_z(cand_best_z),
        .selected_id(selected_id), .conf_q016(conf_q016), .low_conf(low_conf),
        .force_gray(force_gray), .gray_reason(gray_reason),
        .osd_we(osd_we), .osd_waddr(osd_waddr), .osd_wdata(osd_wdata),
        .frame_tick(frame_tick), .busy(busy)
    );

    task automatic chk(input logic ok, input [127:0] tag);
        chk_cnt++;
        if (ok !== 1'b1) begin
            $display("[ERR] %0s", tag);
            err_cnt++;
        end
    endtask

    // 冻结快照
    logic [23:0] fz_tdata;
    logic        fz_tvalid, fz_tlast, fz_tuser, fz_hsync, fz_vsync;

    int          i, k, guard;
    logic        moved;
    logic        found;

    initial begin
        // 默认激励
        vid_tready = 1'b1;
        wall_we = 1'b0; wall_waddr = '0; wall_wdata = '0;
        rec_we = 1'b0; rec_waddr = '0; rec_wdata = '0;
        cand_best_x = '0; cand_best_y = '0; cand_best_z = '0;
        selected_id = '0; conf_q016 = '0; low_conf = 1'b0;
        force_gray = 1'b0; gray_reason = '0;
        osd_we = 1'b0; osd_waddr = '0; osd_wdata = '0;

        // ------------------------------------------------------------------
        // 复位期内预置墙图：wall_mem[i] = 0x800000 + i（ramp，单调，便于判「是否推进」）
        // ------------------------------------------------------------------
        rst_n = 1'b0;
        repeat (4) @(posedge clk);
        for (i = 0; i < NWALL; i = i + 1) begin
            @(negedge clk);
            wall_we    = 1'b1;
            wall_waddr = i[13:0];
            wall_wdata = 24'h800000 | i[23:0];
            @(posedge clk);
        end
        @(negedge clk);
        wall_we = 1'b0;
        repeat (4) @(posedge clk);

        rst_n = 1'b1;

        // ------------------------------------------------------------------
        // A. 等到进入墙面区的一拍（tdata 高字节 = 0x80）
        // ------------------------------------------------------------------
        $display("[INFO] compositor_tready_tb : A 等待墙面区");
        // 墙面区在 (hcnt,hcnt_d)∈[240,720) 且 vcnt_d∈[30,510)；首帧要到第 31 行
        // （约 31*H_TOTAL = 68,200 拍）才第一次同时满足，故等待窗口取 120,000 拍。
        guard   = 0;
        found   = 1'b0;
        for (i = 0; (i < 120000) && (found === 1'b0); i = i + 1) begin
            @(negedge clk);
            guard = guard + 1;
            if ((vid_tvalid === 1'b1) && (vid_tdata[23:16] === 8'h80)) found = 1'b1;
        end
        if (found !== 1'b1) begin
            $display("[ERR] 等不到墙面区有效像素（%0d 拍）", guard);
            $display("=== compositor_tready_tb : FAIL ===  错误 1 处");
            $finish;
        end
        chk(vid_tvalid === 1'b1, "墙面区 vid_tvalid=1");
        chk(vid_tdata[23:16] === 8'h80, "墙面区 tdata 来自预置 ramp");

        // ------------------------------------------------------------------
        // B. 反压：tready=0，输出必须逐拍冻结
        // ------------------------------------------------------------------
        $display("[INFO] compositor_tready_tb : B 拉低 tready，验逐拍冻结");
        @(negedge clk);
        vid_tready = 1'b0;
        @(negedge clk);              // 经过一个 posedge 后 hold 已生效
        fz_tdata  = vid_tdata;
        fz_tvalid = vid_tvalid;
        fz_tlast  = vid_tlast;
        fz_tuser  = vid_tuser;
        fz_hsync  = vid_hsync;
        fz_vsync  = vid_vsync;

        chk(fz_tvalid === 1'b1, "冻结起点的 vid_tvalid=1");
        for (k = 0; k < 10; k = k + 1) begin
            @(negedge clk);
            chk(vid_tdata  === fz_tdata,  "冻结期间 tdata 恒定");
            chk(vid_tvalid === fz_tvalid, "冻结期间 tvalid 恒定");
        end
        chk(vid_tlast === fz_tlast, "冻结期间 tlast 恒定");
        chk(vid_tuser === fz_tuser, "冻结期间 tuser 恒定");
        chk(vid_hsync === fz_hsync, "冻结期间 hsync 恒定");
        chk(vid_vsync === fz_vsync, "冻结期间 vsync 恒定");

        // ------------------------------------------------------------------
        // C. 释放：先交付被冻结那一拍，再恢复推进
        // ------------------------------------------------------------------
        $display("[INFO] compositor_tready_tb : C 拉高 tready，验不丢拍");
        @(negedge clk);
        vid_tready = 1'b1;
        // 本拍（tready 已=1、tvalid=1）握手交付的应正是被冻结的那一拍
        chk(vid_tdata === fz_tdata, "释放后首拍交付的即被冻结的那一拍（不跳拍）");

        moved = 1'b0;
        for (k = 0; k < 10; k = k + 1) begin
            @(negedge clk);
            if (vid_tdata !== fz_tdata) moved = 1'b1;
        end
        chk(moved === 1'b1, "释放后光栅恢复推进（10 拍内 tdata 有变化）");

        // ------------------------------------------------------------------
        repeat (2) @(posedge clk);
        if (err_cnt == 0)
            $display("=== compositor_tready_tb : PASS ===  (比对点 %0d 个)", chk_cnt);
        else
            $display("=== compositor_tready_tb : FAIL ===  错误 %0d 处 / 比对点 %0d 个",
                     err_cnt, chk_cnt);
        $finish;
    end

    initial begin
        #5000000;
        $display("[ERR] 超时");
        $display("=== compositor_tready_tb : FAIL (timeout) ===");
        $finish;
    end

endmodule

`default_nettype wire
