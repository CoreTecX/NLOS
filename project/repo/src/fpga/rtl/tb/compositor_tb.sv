// ============================================================================
// compositor_tb.sv  ·  compositor 自检测试平台
// ---------------------------------------------------------------------------
// ★★★ 仿真状态声明（必读）★★★
//   本测试平台**没有在本机运行过**（本机未安装 iverilog / Verilator / Vivado
//   xsim，无法编译或仿真）。本文件不含任何仿真结果、像素截图、通过记录或覆盖率。
//   它是一份「待运行」的自检脚本，不是测试报告。
//
//   运行方式（本文件位于 src/fpga/rtl/tb/）：
//     iverilog -g2012 -o compositor_tb.vvp compositor_tb.sv ../compositor.sv
//     vvp compositor_tb.vvp
//   预期输出：末尾打印 "=== compositor_tb : PASS ==="。
//
//   预计仿真时长：本 TB 要跑完整的 1080p 帧周期（2200x1125 = 2,475,000 拍/帧），
//   T2 整帧逐像素比对约 2,073,600 次，三段用例合计约 3 个帧周期（约 74 ms 仿真时间）。
//   iverilog 下预计数十秒到数分钟量级 —— 这是「整帧逐像素」换来的代价，属预期。
//
// ---------------------------------------------------------------------------
// 覆盖的内容（使用 DUT 默认参数：1080p60 / 墙 96 / 隐藏 24x18 / 候选 9x9x3）
// ---------------------------------------------------------------------------
//   T1 时序契约（整帧）：
//        de 拍数 = H_ACTIVE*V_ACTIVE；tlast 拍数 = V_ACTIVE；
//        tuser 拍数 = 1；frame_tick 拍数 = 1；
//        hsync 低拍数 = H_SYNC*V_TOTAL；vsync 低拍数 = V_SYNC*H_TOTAL。
//
//   T2 逐像素金模型比对（整帧）：
//        以 tuser 为帧首锚点，用 k 计数推出 (h,v) = (k % H_ACTIVE, k / H_ACTIVE)，
//        与 TB 内复刻的四分区 / 最近邻取址 / 候选网格 / 可信度条判据逐值比较。
//        OSD 文本区只做「必为文字前景色或背景色」的成员判据 —— TB **不**复制字库，
//        因为字库是可视化资源而非算法口径（真正需要逐字形比对的场合应在板上截图核对）。
//
//   T3 灰屏覆盖（force_gray=1）：整屏 C_GRAY，左上 8x8 按 gray_reason 着色；
//        并确认**分区边框线仍覆盖在灰屏之上**（C_DIV 优先于灰屏，见下「注意」）。
//
//   T4 恢复（force_gray=0）：再次做金模型比对（覆盖 TL/TR 两个上图分区），
//        证明灰屏退出后渲染回到正常。
//
// ---------------------------------------------------------------------------
// 注意（本 TB 在写金模型时发现、需要用仿真确认的一处交互）
// ---------------------------------------------------------------------------
//   compositor.sv 的像素优先级是：灰屏/分区渲染 -> 边框(border_d) -> 消隐(black)。
//   因此 **border_d 会覆盖灰屏**：在 h∈{0,959,960,1919} 或 v∈{0,539,540,1079}
//   这 8 条线上，即使 force_gray=1，输出也是 C_DIV 而不是 C_GRAY。
//   金模型按同样的优先级实现。若评审认为「灰屏必须绝对整屏」，需要把 border_d 的
//   优先级降到 force_gray 之下 —— 那是**行为变更**，不是修 bug。
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module compositor_tb;

    // ------------------------------------------------------------------
    // 与 DUT 默认参数一致的几何/时序常量（金模型必须逐值复刻这些公式）
    // ------------------------------------------------------------------
    localparam int H_ACTIVE = 1920;
    localparam int H_SYNC   = 44;
    localparam int H_TOTAL  = 2200;   // 1920 + 88 + 44 + 148
    localparam int V_ACTIVE = 1080;
    localparam int V_SYNC   = 5;
    localparam int V_TOTAL  = 1125;   // 1080 + 4 + 5 + 36

    localparam int RGN_W = H_ACTIVE/2;   // 960
    localparam int RGN_H = V_ACTIVE/2;   // 540

    localparam int WALL_N = 96;
    localparam int HID_W  = 24;
    localparam int HID_H  = 18;
    localparam int NWALL  = WALL_N*WALL_N;   // 9216
    localparam int NREC   = HID_W*HID_H;     // 432
    localparam int NOSD   = 32*8;            // 256

    // 布局常量（与 compositor.sv 的 localparam 逐条对应）
    localparam int WALL_DST   = 480;                       // 96 x 5
    localparam int WALL_OFF_X = (RGN_W - WALL_DST)/2;       // 240
    localparam int WALL_OFF_Y = (RGN_H - WALL_DST)/2;       // 30
    localparam int REC_DST_W  = 720;                        // 24 x 30
    localparam int REC_DST_H  = 540;                        // 18 x 30
    localparam int REC_OFF_X  = (RGN_W - REC_DST_W)/2;      // 120
    localparam int REC_OFF_Y  = (RGN_H - REC_DST_H)/2;      // 0
    localparam int OSD_OFF_X  = 40;
    localparam int OSD_OFF_Y  = 40;
    localparam int CAN_PITCH  = 32;
    localparam int CAN_CELL   = 24;
    localparam int CAN_SPAN   = 9*CAN_PITCH - (CAN_PITCH - CAN_CELL);   // 280
    localparam int CAN_OFF_X  = (RGN_W - CAN_SPAN)/2;                   // 340
    localparam int CAN_OFF_Y  = (RGN_H - CAN_SPAN)/2 - 40;              // 90
    localparam int BAR_X0     = 60;
    localparam int BAR_W_MAX  = 400;
    localparam int BAR_Y0     = 480;
    localparam int BAR_H      = 20;

    // 颜色常量（RGB888，与 DUT 一致）
    localparam logic [23:0] C_BG      = 24'h18_18_18;
    localparam logic [23:0] C_DIV     = 24'h40_40_40;
    localparam logic [23:0] C_PANEL   = 24'h00_00_00;
    localparam logic [23:0] C_GRID    = 24'h00_60_00;
    localparam logic [23:0] C_BEST    = 24'hFF_FF_00;
    localparam logic [23:0] C_BAR_BG  = 24'h30_00_00;
    localparam logic [23:0] C_BAR_FG  = 24'h00_FF_60;
    localparam logic [23:0] C_GRAY    = 24'h80_80_80;
    localparam logic [23:0] C_TEXT_FG = 24'hE0_E0_E0;
    localparam logic [23:0] C_TEXT_BG = 24'h10_10_10;
    localparam logic [23:0] C_REASON0 = 24'hFF_00_00;   // 饱和比例
    localparam logic [23:0] C_REASON1 = 24'hFF_A0_00;   // SNR
    localparam logic [23:0] C_REASON2 = 24'h00_A0_FF;   // margin
    localparam logic [23:0] C_REASON3 = 24'hC0_00_FF;   // rho
    localparam logic [23:0] C_REASON4 = 24'hFF_FF_FF;   // 几何哈希不一致
    // 数据完整性（bit[7:5]，优先级最高；与 compositor.sv 逐值一致）
    localparam logic [23:0] C_REASON5 = 24'h00_FF_7F;   // raw ovf_sticky
    localparam logic [23:0] C_REASON6 = 24'h7F_00_FF;   // diff ovf
    localparam logic [23:0] C_REASON7 = 24'hFF_00_7F;   // proto_err

    logic clk = 1'b0;
    always #5 clk = ~clk;            // 100 MHz 仅 TB 用；不代表实际像素时钟(148.5M)
    logic rst_n = 1'b0;

    int err_cnt = 0;
    int chk_cnt = 0;

    // ------------------------------------------------------------------
    // DUT 端口
    // ------------------------------------------------------------------
    logic [23:0] vid_tdata;
    logic        vid_tvalid, vid_tlast, vid_tuser, vid_de, vid_hsync, vid_vsync;
    logic        frame_tick, busy;

    logic        wall_we;
    logic [13:0] wall_waddr;
    logic [23:0] wall_wdata;
    logic        rec_we;
    logic [8:0]  rec_waddr;
    logic [23:0] rec_wdata;

    logic [15:0] cand_best_x, cand_best_y, cand_best_z, selected_id, conf_q016;
    logic        low_conf, force_gray;
    logic [7:0]  gray_reason;

    logic        osd_we;
    logic [7:0]  osd_waddr, osd_wdata;

    compositor dut (
        .clk(clk), .rst_n(rst_n),
        .vid_tdata(vid_tdata), .vid_tvalid(vid_tvalid), .vid_tlast(vid_tlast),
        .vid_tuser(vid_tuser), .vid_de(vid_de), .vid_hsync(vid_hsync),
        .vid_vsync(vid_vsync),
        .vid_tready(1'b1),   // G3（2026-10-01）：本 TB 验自由运行口径，恒不反压。
                             // 反压行为由 compositor_tready_tb.sv 单独验（避免改本 TB 的
                             // 整帧金模型：本 TB 的 T1/T2 按 vid_tvalid 计拍，不按握手计拍）。
        .wall_we(wall_we), .wall_waddr(wall_waddr), .wall_wdata(wall_wdata),
        .rec_we(rec_we), .rec_waddr(rec_waddr), .rec_wdata(rec_wdata),
        .cand_best_x(cand_best_x), .cand_best_y(cand_best_y), .cand_best_z(cand_best_z),
        .selected_id(selected_id), .conf_q016(conf_q016), .low_conf(low_conf),
        .force_gray(force_gray), .gray_reason(gray_reason),
        .osd_we(osd_we), .osd_waddr(osd_waddr), .osd_wdata(osd_wdata),
        .frame_tick(frame_tick), .busy(busy)
    );

    // ------------------------------------------------------------------
    // TB 侧内存镜像（与 DUT 内 BRAM 内容逐值相同，供金模型查表）
    // ------------------------------------------------------------------
    logic [23:0] wall_img [0:NWALL-1];
    logic [23:0] rec_img  [0:NREC-1];
    logic [7:0]  osd_img  [0:NOSD-1];

    int bar_fill_c;      // 可信度条填充宽度（金模型用）

    // ------------------------------------------------------------------
    // 金模型：给定 (h,v) 算出期望像素
    //   exp_pix    : 期望颜色
    //   exp_is_osd : 该像素落在 OSD 文本区（只做成员判据）
    // ------------------------------------------------------------------
    logic [23:0] exp_pix;
    bit          exp_is_osd;

    task automatic model_pix(input int h, input int v);
        int  sx, sy;
        bit  in_tl, in_tr, in_bl, in_br, border;
        in_tl  = (h <  RGN_W)              && (v <  RGN_H);
        in_tr  = (h >= RGN_W) && (h < 2*RGN_W) && (v <  RGN_H);
        in_bl  = (h <  RGN_W)              && (v >= RGN_H);
        in_br  = (h >= RGN_W) && (h < 2*RGN_W) && (v >= RGN_H);
        border = (h == 0) || (h == RGN_W-1) || (h == RGN_W) || (h == H_ACTIVE-1) ||
                 (v == 0) || (v == RGN_H-1) || (v == RGN_H) || (v == V_ACTIVE-1);

        exp_pix    = C_BG;
        exp_is_osd = 1'b0;

        if (in_tl) begin
            // 左上：墙面 ROI（96x96 最近邻放大到 480x480，居中）
            if ((h >= WALL_OFF_X) && (h < WALL_OFF_X + WALL_DST) &&
                (v >= WALL_OFF_Y) && (v < WALL_OFF_Y + WALL_DST)) begin
                sx = ((h - WALL_OFF_X) * WALL_N) / WALL_DST;
                sy = ((v - WALL_OFF_Y) * WALL_N) / WALL_DST;
                exp_pix = wall_img[sy*WALL_N + sx];
            end else begin
                exp_pix = C_PANEL;
            end
        end else if (in_tr) begin
            // 右上：隐藏图像重建（24x18 最近邻放大到 720x540，居中）
            if ((h >= RGN_W + REC_OFF_X) && (h < RGN_W + REC_OFF_X + REC_DST_W) &&
                (v >= REC_OFF_Y) && (v < REC_OFF_Y + REC_DST_H)) begin
                sx = ((h - (RGN_W + REC_OFF_X)) * HID_W) / REC_DST_W;
                sy = ((v - REC_OFF_Y) * HID_H) / REC_DST_H;
                exp_pix = rec_img[sy*HID_W + sx];
            end else begin
                exp_pix = C_PANEL;
            end
        end else if (in_bl) begin
            // 左下：候选网格（9x9，节距 32，格 24）+ 可信度条
            exp_pix = C_BG;
            if ((h >= CAN_OFF_X) && (h < CAN_OFF_X + CAN_SPAN) &&
                (v >= CAN_OFF_Y) && (v < CAN_OFF_Y + CAN_SPAN) &&
                (((h - CAN_OFF_X) % CAN_PITCH) < CAN_CELL) &&
                (((v - CAN_OFF_Y) % CAN_PITCH) < CAN_CELL)) begin
                if ((((h - CAN_OFF_X)/CAN_PITCH) == cand_best_x) &&
                    (((v - CAN_OFF_Y)/CAN_PITCH) == cand_best_y)) exp_pix = C_BEST;
                else                                                exp_pix = C_GRID;
            end
            if ((h >= BAR_X0) && (h < BAR_X0 + BAR_W_MAX) &&
                (v >= BAR_Y0) && (v < BAR_Y0 + BAR_H)) begin
                if ((h - BAR_X0) < bar_fill_c) exp_pix = C_BAR_FG;
                else                           exp_pix = C_BAR_BG;
            end
        end else if (in_br) begin
            // 右下：遥测文本（32 列 x 8 行，8x8 字形，原点 (1000,580)）
            exp_pix = C_TEXT_BG;
            if ((h >= RGN_W + OSD_OFF_X) && (h < RGN_W + OSD_OFF_X + 32*8) &&
                (v >= RGN_H + OSD_OFF_Y) && (v < RGN_H + OSD_OFF_Y + 8*8)) begin
                exp_is_osd = 1'b1;   // 具体字形不比对（TB 不含字库）
            end
        end

        // 边框优先级最高（见文件头「注意」）
        if (border) begin
            exp_pix    = C_DIV;
            exp_is_osd = 1'b0;
        end
    endtask

    // 逐像素比对
    task automatic chk_pixel(input int h, input int v, input logic [23:0] got);
        model_pix(h, v);
        chk_cnt++;
        if ((^got) === 1'bx) begin
            $display("[ERR] (h=%0d,v=%0d) vid_tdata 含 X", h, v);
            err_cnt++;
        end else if (exp_is_osd) begin
            if (!((got === C_TEXT_FG) || (got === C_TEXT_BG))) begin
                $display("[ERR] (h=%0d,v=%0d) OSD 区颜色异常 got=0x%06h", h, v, got);
                err_cnt++;
            end
        end else if (got !== exp_pix) begin
            $display("[ERR] (h=%0d,v=%0d) 像素不符 got=0x%06h exp=0x%06h",
                     h, v, got, exp_pix);
            err_cnt++;
        end
    endtask

    // 灰屏期逐像素比对（优先级：边框 > 左上 8x8 原因块 > 纯灰）
    task automatic chk_gray(input int kk, input logic [23:0] got);
        int          h, v;
        bit          border;
        logic [23:0] rc, e;
        h = kk % H_ACTIVE;
        v = kk / H_ACTIVE;
        rc = gray_reason[7] ? C_REASON7 :
             gray_reason[6] ? C_REASON6 :
             gray_reason[5] ? C_REASON5 :
             gray_reason[4] ? C_REASON4 :
             gray_reason[0] ? C_REASON0 :
             gray_reason[1] ? C_REASON1 :
             gray_reason[2] ? C_REASON2 :
             gray_reason[3] ? C_REASON3 : 24'h00_00_00;
        border = (h == 0) || (h == RGN_W-1) || (h == RGN_W) || (h == H_ACTIVE-1) ||
                 (v == 0) || (v == RGN_H-1) || (v == RGN_H) || (v == V_ACTIVE-1);
        if (border)             e = C_DIV;
        else if ((h < 8) && (v < 8)) e = rc;
        else                    e = C_GRAY;

        chk_cnt++;
        if (got !== e) begin
            $display("[ERR] 灰屏 (h=%0d,v=%0d) got=0x%06h exp=0x%06h", h, v, got, e);
            err_cnt++;
        end
    endtask

    // ------------------------------------------------------------------
    // 帧级检查引擎
    //   run_chk=1 后，从下一个 tuser 起开始检查：
    //     * 逐像素金模型（帧首锚点 k=(h,v) 反推）
    //     * 时序计数（do_counts=1 时以「下一个 tuser」为结束点，恰一整帧）
    //     * 或 do_counts=0 且 chk_limit>0 时，检查 chk_limit 个像素即结束
    // ------------------------------------------------------------------
    bit run_chk   = 1'b0;
    bit chk_done  = 1'b0;
    bit started   = 1'b0;
    bit do_counts = 1'b0;
    int chk_limit = 0;
    int k_cnt = 0;
    int de_cnt = 0, tlast_cnt = 0, tuser_cnt = 0, ftick_cnt = 0;
    int hs_low = 0, vs_low = 0;

    always @(posedge clk) begin
        if (run_chk) begin
            // 前两个分支的次序很关键：
            //   1) 见到「第二个 tuser」时该拍**不计入任何计数**（它是下一帧的首拍）；
            //   2) 其它拍先做像素/帧首处理（帧首处理会把 started 置 1），
            //      再在最后统一计数 —— 这样 **tuser1 的当拍也被计入**。
            // 于是计数窗口恰好是 [tuser1, tuser2)，长度 = V_TOTAL*H_TOTAL 拍，
            // hsync/vsync 低电平计数才正好等于 H_SYNC*V_TOTAL / V_SYNC*H_TOTAL。
            if ((vid_tvalid === 1'b1) && (vid_tuser === 1'b1) && (started === 1'b1)) begin
                run_chk  = 1'b0;
                chk_done = 1'b1;
            end else begin
                if (vid_tvalid === 1'b1) begin
                    if (vid_tuser === 1'b1) begin
                        // 帧首：k=0 对应 (h,v)=(0,0)
                        started   = 1'b1;
                        de_cnt    = 1;
                        tlast_cnt = (vid_tlast === 1'b1) ? 1 : 0;
                        tuser_cnt = 1;
                        k_cnt     = 1;
                        chk_pixel(0, 0, vid_tdata);
                    end else if (started === 1'b1) begin
                        chk_pixel(k_cnt % H_ACTIVE, k_cnt / H_ACTIVE, vid_tdata);
                        k_cnt = k_cnt + 1;
                        de_cnt = de_cnt + 1;
                        if (vid_tlast === 1'b1) tlast_cnt = tlast_cnt + 1;
                        if ((do_counts === 1'b0) && (chk_limit > 0) &&
                            (de_cnt >= chk_limit)) begin
                            run_chk  = 1'b0;
                            chk_done = 1'b1;
                        end
                    end
                end

                if (started === 1'b1) begin
                    if (vid_hsync === 1'b0)  hs_low    = hs_low    + 1;
                    if (vid_vsync === 1'b0)  vs_low    = vs_low    + 1;
                    if (frame_tick === 1'b1) ftick_cnt = ftick_cnt + 1;
                end
            end
        end
    end

    // 灰屏期检查引擎（检查前 8 行，足以覆盖原因块与纯灰区）
    bit gray_run  = 1'b0;
    bit gray_done = 1'b0;
    bit g_started = 1'b0;
    int g_k   = 0;
    int g_cnt = 0;

    always @(posedge clk) begin
        if (gray_run) begin
            if (vid_tvalid === 1'b1) begin
                if (vid_tuser === 1'b1) begin
                    g_started = 1'b1;
                    g_k       = 0;
                    chk_gray(0, vid_tdata);
                    g_cnt     = 1;
                end else if (g_started == 1'b1) begin
                    g_k = g_k + 1;
                    chk_gray(g_k, vid_tdata);
                    g_cnt = g_cnt + 1;
                    if (g_cnt >= 8*H_ACTIVE) begin
                        gray_run  = 1'b0;
                        gray_done = 1'b1;
                    end
                end
            end
        end
    end

    // ------------------------------------------------------------------
    // 预载存储器
    // ------------------------------------------------------------------
    task automatic preload();
        int i;
        // 统一在**下降沿**驱动激励：上升沿留给 DUT 采样，避免 TB 与 DUT 在同一时刻
        // 竞争（这是 TB 里「第一拍丢失 / 同一拍被采两次」最常见的来源）。
        @(negedge clk);
        for (i = 0; i < NWALL; i = i + 1) begin
            wall_img[i] = {8'(i*7), 8'(i*13), 8'(i*29)};
            wall_we     = 1'b1;
            wall_waddr  = 14'(i);
            wall_wdata  = wall_img[i];
            @(negedge clk);
        end
        wall_we = 1'b0;
        @(negedge clk);

        for (i = 0; i < NREC; i = i + 1) begin
            rec_img[i] = {8'(i*3), 8'(255-i), 8'(i*5)};
            rec_we     = 1'b1;
            rec_waddr  = 9'(i);
            rec_wdata  = rec_img[i];
            @(negedge clk);
        end
        rec_we = 1'b0;
        @(negedge clk);

        for (i = 0; i < NOSD; i = i + 1) begin
            osd_img[i] = 8'h30 + 8'(i % 10);   // '0'..'9' 循环，全是已定义字形
            osd_we     = 1'b1;
            osd_waddr  = 8'(i);
            osd_wdata  = osd_img[i];
            @(negedge clk);
        end
        osd_we = 1'b0;
    endtask

    // ------------------------------------------------------------------
    // 主流程
    // ------------------------------------------------------------------
    initial begin
        // 初值
        wall_we = 1'b0; wall_waddr = '0; wall_wdata = '0;
        rec_we  = 1'b0; rec_waddr  = '0; rec_wdata  = '0;
        osd_we  = 1'b0; osd_waddr  = '0; osd_wdata  = '0;
        force_gray  = 1'b0;
        gray_reason = 5'b0;
        low_conf    = 1'b0;
        cand_best_x = 16'd3;
        cand_best_y = 16'd4;
        cand_best_z = 16'd1;
        selected_id = 16'd148;            // 3*81 + 4*9 + 1（仅遥测用，不影响像素）
        conf_q016   = 16'h4000;           // 0.5 -> 填充宽度 100
        bar_fill_c  = (int'(conf_q016) * BAR_W_MAX) >> 16;

        repeat (8) @(posedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);

        // ================= T0：预载 =================
        $display("[INFO] T0 预载存储器（墙 %0d / 重建 %0d / OSD %0d 字）",
                 NWALL, NREC, NOSD);
        preload();

        // ================= T1/T2：整帧时序 + 逐像素金模型 =================
        $display("[INFO] T1/T2 整帧时序与逐像素比对（约 %0d 像素）",
                 H_ACTIVE*V_ACTIVE);
        force_gray = 1'b0;
        started    = 1'b0;
        do_counts  = 1'b1;
        chk_limit  = 0;
        k_cnt = 0; de_cnt = 0; tlast_cnt = 0; tuser_cnt = 0; ftick_cnt = 0;
        hs_low = 0; vs_low = 0;
        chk_done   = 1'b0;
        @(negedge clk);          // 使能放在下降沿，避免与 always 块争用同一时刻
        run_chk    = 1'b1;
        wait (chk_done === 1'b1);
        repeat (4) @(posedge clk);

        $display("[INFO] T1 de=%0d tlast=%0d tuser=%0d ftick=%0d hs_low=%0d vs_low=%0d",
                 de_cnt, tlast_cnt, tuser_cnt, ftick_cnt, hs_low, vs_low);
        chk_cnt++;
        if (de_cnt !== H_ACTIVE*V_ACTIVE) begin
            $display("[ERR] T1 de 拍数 %0d，期望 %0d", de_cnt, H_ACTIVE*V_ACTIVE);
            err_cnt++;
        end
        chk_cnt++;
        if (tlast_cnt !== V_ACTIVE) begin
            $display("[ERR] T1 tlast 拍数 %0d，期望 %0d", tlast_cnt, V_ACTIVE); err_cnt++;
        end
        chk_cnt++;
        if (tuser_cnt !== 1) begin
            $display("[ERR] T1 tuser 拍数 %0d，期望 1", tuser_cnt); err_cnt++;
        end
        chk_cnt++;
        if (ftick_cnt !== 1) begin
            $display("[ERR] T1 frame_tick 拍数 %0d，期望 1", ftick_cnt); err_cnt++;
        end
        chk_cnt++;
        if (hs_low !== H_SYNC*V_TOTAL) begin
            $display("[ERR] T1 hsync 低拍数 %0d，期望 %0d", hs_low, H_SYNC*V_TOTAL);
            err_cnt++;
        end
        chk_cnt++;
        if (vs_low !== V_SYNC*H_TOTAL) begin
            $display("[ERR] T1 vsync 低拍数 %0d，期望 %0d", vs_low, V_SYNC*H_TOTAL);
            err_cnt++;
        end

        // ================= T3：灰屏覆盖 =================
        $display("[INFO] T3 灰屏覆盖（force_gray=1，reason=bit4 几何哈希）");
        force_gray  = 1'b1;
        gray_reason = 5'b10000;
        g_started   = 1'b0;
        g_cnt       = 0;
        gray_done   = 1'b0;
        @(negedge clk);
        gray_run    = 1'b1;
        wait (gray_done === 1'b1);
        repeat (4) @(posedge clk);
        // PS low_conf must independently suppress stale or ambiguous images.
        force_gray=1'b0;low_conf=1'b1;
        g_started=1'b0;g_cnt=0;gray_done=1'b0;
        @(negedge clk);gray_run=1'b1;
        wait(gray_done===1'b1);
        repeat(4) @(posedge clk);low_conf=1'b0;
        gray_reason = 5'b0;

        // ================= T4：退出灰屏后恢复正常渲染 =================
        //   从 tuser 起检查前 60 行：v=0..29 覆盖 TR 的重建图与 TL 的面板，
        //   v=30..59 起覆盖 TL 的墙面 ROI（WALL_OFF_Y=30），四类像素都能打到
        $display("[INFO] T4 退出灰屏（force_gray=0），复查前 %0d 个像素", 60*H_ACTIVE);
        force_gray = 1'b0;
        started    = 1'b0;
        do_counts  = 1'b0;
        chk_limit  = 60*H_ACTIVE;
        chk_done   = 1'b0;
        @(negedge clk);
        run_chk    = 1'b1;
        wait (chk_done === 1'b1);
        repeat (4) @(posedge clk);

        // ================= 汇总 =================
        if (err_cnt == 0)
            $display("=== compositor_tb : PASS ===  (比对点 %0d 个)", chk_cnt);
        else
            $display("=== compositor_tb : FAIL ===  错误 %0d 处 / 比对点 %0d 个",
                     err_cnt, chk_cnt);
        $finish;
    end

    // 超时保护（3 个帧周期约 74 ms -> 留 200 ms 余量）
    initial begin
        #200_000_000;
        $display("[ERR] 超时：TB 未在预期周期数内结束");
        $display("=== compositor_tb : FAIL (timeout) ===");
        $finish;
    end

endmodule

`default_nettype wire
