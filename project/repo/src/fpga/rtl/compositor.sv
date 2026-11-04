// ============================================================================
// compositor.sv  ·  PenumbraSight 显示合成器（方案 §6.2 / §7）
// 依据：plan_teacher.md §6.2（compositor：墙面图、重建图、状态文字/图标 -> 1080p RGB AXIS）
//       §7（显示画面固定为四区）
//           左上：相机看到的墙面 ROI
//           右上：隐藏图像重建
//           左下：遮挡位置网格、最佳/次佳分数和可信度
//           右下：帧率、采集延迟、PL 计算延迟、秩、位宽、迭代、资源和功耗
//       §7（「允许 CPU 把 24x18 结果上采样到显示尺寸，但竞赛演示优先由 PL
//            compositor 完成最近邻/双线性放大和叠加，以强化 FPGA 核心性」）
//       §6.6（强制灰屏时 compositor 必须整屏变灰并提示原因）
//
// ---------------------------------------------------------------------------
// 设计意图
// ---------------------------------------------------------------------------
// 【意图 1】为什么放大必须放在 PL（「强化 FPGA 核心性」的硬要求）
//   §7 明确写了：允许 CPU 上采样，但**优先由 PL 完成**。所以本模块自己带最近邻
//   放大：24x18 -> 720x540 恰好 30 倍、96x96 -> 480x480 恰好 5 倍。
//   「恰好整除」不是巧合，是**故意选的比例**：只有整数倍放大才能用「取商」实现
//   （src = local * N / DST），不需要 Bresenham 累加器，省掉两个逐行/逐列余数
//   累加器及其状态。若要换成非整数倍（如 24x18 -> 640x480），必须补 Bresenham。
//
// 【意图 2】为什么坐标要打一拍
//   墙图/重建图在 BRAM 里，同步读有 1 拍延迟。若像素选择直接用当前扫描坐标，
//   图像会整体错位 1 像素（在 5 倍放大下表现为每条扫描线的错位，肉眼可见的竖纹）。
//   故：BRAM 地址用**当前**坐标算，所有渲染判断用**延迟一拍**的坐标，两者天然对齐。
//
// 【意图 3】OSD 文本用分布式 RAM 异步读，不占 BRAM
//   文本缓冲只有 32x8 = 256 字节。若用同步读 BRAM，等于为 2 Kbit 再加一级流水，
//   整个渲染通路就要多打一拍（墙图/重建图都得再存一拍）。256 字节异步读在 LUTRAM
//   里只要几十个 LUT6，比多一级 24 bit 流水便宜得多。
//
// 【意图 4】灰屏是「整屏覆盖」，不是「把图像调暗」
//   §6.6 要求强制灰屏并提示原因。若只是把图像乘个系数，屏幕上仍会残留「看起来像图」
//   的内容，评审与操作员无法区分「低可信度」与「真实低对比度场景」。故 force_gray
//   时整屏覆盖纯灰，只在左上角保留一个 8x8 的按 gray_reason 着色块用于现场定位。
//
// ---------------------------------------------------------------------------
// 易错点
// ---------------------------------------------------------------------------
// (1) 像素时钟不是 PL 主时钟
//     1080p60 的像素时钟是 148.5 MHz，PL 主时钟标称 150 MHz —— 两者**不是同一个
//     时钟**。本模块内部按单时钟实现；BRAM 写口（来自 PDHG/ARM 域）若不同源，必须在
//     上层加异步 FIFO 或双口 BRAM。直接跨接会得到「偶尔一帧撕裂」而不是稳定错位，
//     极难定位；这也正好命中 §9.2 的稳定性要求（连续 30 min 无花屏）。
//
// (2) 整数倍放大是硬约束
//     WALL_DST 必须能被 WALL_N 整除、REC_DST_W/H 必须能被 HID_W/H 整除。
//     若改参数导致不整除，src = local*N/DST 会退化成「右侧若干源像素被丢弃」——
//     画面仍然是「一张图」，只是边缘多一条被拉伸的缝，不会报错。
//
// (3) OSD 字符必须落在 8 像素网格上
//     OSD_OFF_X/Y 必须是 8 的倍数，否则 lx[2:0] 取到的不是「字符内列」，字形会被
//     切成锯齿。这里取 40 与 40，都是 8 的倍数 ✓。
//     **更隐蔽的一层**：文本区的绝对原点 = (RGN_W + OSD_OFF_X, RGN_H + OSD_OFF_Y)
//     = (1000, 580)。水平 1000 % 8 = 0（碰巧对齐），竖直 580 % 8 = 4（不对齐）。
//     所以「字形内行号」必须写成 (vcnt_d - 580)[2:0]，直接取 vcnt_d[2:0] 会得到
//     一个整体滚动 4 像素的字形（表现为「字在抖」而不是「字错位」，极易误判为
//     时序问题去查 Fmax）。
//
// (4) 内部字库只有数字与少量符号
//     为了不把整张字库塞进 RTL 淹没真正的逻辑，内部 ROM 只含 0-9、':'、'.'、' '、
//     '-'、'/'。遥测页要字母时改用外部字库（把 font_row 换成 BRAM 读口即可，
//     接口已按 (char,row) -> 8 bit 预留）。未定义字符渲染为空白（不乱码、不崩溃）。
//
// (5) 位宽上下文陷阱（本文件最容易写错的地方）
//     `a * b / c` 赋给一个窄位宽信号时，SV 会按**目标宽度**做中间运算，乘法先被
//     截断再除，结果全错且不报错。本文件所有「坐标 -> 源像素 -> 线性地址」的换算
//     都显式用 16 位中间量 + 显式切片，不依赖上下文推断。
//
// (6) 渲染 MUX 是组合逻辑
//     pix 由 BRAM 输出经组合 MUX 直达 vid_tdata。148.5 MHz 下这是一段较长的组合
//     路径；若时序不收敛，应在 vid_tdata 前再插一级寄存器，同时把 tvalid/tlast/
//     tuser 一起延迟一拍（本文件出于「逐值可比」的可读性考虑保留组合输出）。
//
// ---------------------------------------------------------------------------
// 接口口径（端口一览；默认位宽见下方 parameter 列表）
// ---------------------------------------------------------------------------
//   输入
//     clk                    1       视频/像素时钟域（见易错点 1；**不是** PL 主时钟）
//     rst_n                  1       低有效异步复位
//     wall_we                1       墙面 ROI 写使能（ARM/采集侧填充 96x96 RGB888）
//     wall_waddr      [13:0]         墙面 ROI 写地址（行*96+列）
//     wall_wdata      [23:0]         墙面 ROI RGB888
//     rec_we                 1       重建图写使能（PDHG 输出 24x18）
//     rec_waddr        [8:0]         重建图写地址（行*24+列）
//     rec_wdata       [23:0]         重建图 RGB888（灰度可复制到三通道）
//     cand_best_x     [15:0]         最佳候选网格索引 0..8
//     cand_best_y     [15:0]         0..8
//     cand_best_z     [15:0]         0..2（决定显示哪一层 z；当前版本未用，预留）
//     selected_id     [15:0]         线性候选号 0..242（当前版本未用，预留/遥测）
//     conf_q016       [15:0]         CONFIDENCE 寄存器 0x1C（Q0.16）
//     low_conf               1       PS 质量闸门；与 force_gray 一同覆盖为灰屏
//     force_gray             1       来自 confidence_engine；整屏覆盖纯灰
//     gray_reason     [7:0]          灰屏原因位图（左上 8x8 着色）
//     osd_we                 1       OSD 文本写使能（8x8 字形，ASCII）
//     osd_waddr        [7:0]         行*32+列
//     osd_wdata        [7:0]         字符 ASCII
//   输出
//     vid_tdata       [23:0]         RGB888
//     vid_tvalid             1       = de（严格门控）
//     vid_tlast              1       行末
//     vid_tuser              1       帧首
//     vid_de                 1       数据有效（同 vid_tvalid）
//     vid_hsync              1       行同步，低有效
//     vid_vsync              1       场同步，低有效
//   输入（G3，2026-10-01 新增）
//     vid_tready             1       下游 AXIS 反压；0 时冻结整条光栅（hcnt/vcnt + 输出流水
//                                    + BRAM 同步读），把当前这拍像素保持到被接收。
//                                    接通后 vid_* 是带 tready 的标准 AXIS 主，不再是自由运行。
//     frame_tick             1       帧首 1 拍脉冲（遥测时基）
//     busy                   1       恒 1（显示是自由运行通路）
//   对应文档：plan_teacher.md §6.2（compositor：墙面图、重建图、状态文字/图标 ->
//             1080p RGB AXIS）/ §7（四分区固定布局）/ §6.6（强制灰屏并提示原因）/
//             §7（「优先由 PL compositor 完成最近邻/双线性放大和叠加」）
//             configs/geometry_b.yaml（墙 96、隐藏 24x18、候选 9x9x3）
//             src/software/include/psm_format.h（HIDDEN_W=24 / HIDDEN_H=18）
//             （PS 侧对应实现参考 src/software/ui/ui_layout.hpp）
//
// ---------------------------------------------------------------------------
// 刻意保留的行为（不是 bug；改动前先确认是不是有意为之）
// ---------------------------------------------------------------------------
//   [R1] 放大放在 PL 做（最近邻，整数倍：96->480 = x5、24x18->720x540 = x30）。
//        这不是为了省事：§7 明确「优先由 PL 完成放大，以强化 FPGA 核心性」。
//        整数倍是**刻意**选的比例，只用取商即可，不需要 Bresenham。
//   [R2] BRAM 地址用**当前**扫描坐标算、渲染判断用**延迟一拍**的坐标。这是刻意对齐
//        同步读的 1 拍延迟；改成同一拍会让图像整体错位一像素（5 倍放大下呈竖纹）。
//   [R3] OSD 文本用分布式 RAM **异步读**，不占 BRAM、不插流水级（只有 256 字节）。
//   [R4] 内部字库刻意只含 0-9 : . - / 与空格。遥测页要字母时换成外部字库（接口已按
//        (char,row) -> 8 bit 预留），**不要**往 RTL 里塞整张字库淹没真实逻辑。
//   [R5] 灰屏是**整屏覆盖**纯灰 + 左上 8x8 原因着色块，不是「把图像调暗」。刻意让
//        「低可信度」与「真实低对比度场景」在屏幕上无法混淆（意图 4）。
//        注意优先级：**分区边框线（C_DIV）覆盖在灰屏之上**（边框是 UI 元素，刻意
//        保留可见）。若评审要求灰屏绝对整屏，需把 border_d 降到 force_gray 之下 ——
//        那是行为变更，不是 bug 修复；tb/compositor_tb.sv 已按当前优先级做金模型。
//   [R6] 像素 MUX 是**组合逻辑**直达 vid_tdata（保留可读性 / 逐值易比对）。若 148.5 MHz
//        时序不收敛，需在输出前插一级寄存器并把 tvalid/tlast/tuser 一起延迟一拍 ——
//        这是已知的时序折衷点，不是遗漏的流水级。
//   [R7] 单时钟域实现：本模块**不**内建跨域同步。BRAM 写口若与像素时钟不同源，必须
//        在上层加异步 FIFO 或双口 BRAM（见易错点 1）。
//   [R8] 不读模型 / 不做缩放插值（最近邻而已）；selected_id / cand_best_z / low_conf
//        等端口当前版本**故意**只引出、不参与像素选择，留给遥测页与后续增强。
//
// ---------------------------------------------------------------------------
// 仿真状态声明
// ---------------------------------------------------------------------------
//   本文件**未在任何仿真器上运行过**（本机无 iverilog / Vivado / Verilator）。
//   文件内不含任何仿真的像素截图或时序数据。配套 tb/compositor_tb.sv 是待运行的自检
//   平台：分区边界、最近邻取址、灰屏覆盖、de/hsync/vsync/tlast/tuser 计数均逐值比对。
// ============================================================================

// 与 tb/compositor_tb.sv 取同一值：RTL 缺 timescale 时 xsim 会因
// [XSIM 43-4099] 拒绝 elaboration（见 sim/results/xsim_first_run_20260929.md §6.1）。
`timescale 1ns/1ps
`default_nettype none

module compositor #(
    // ---- 视频时序（1080p60 CEA-861 常用值）----
    parameter int H_ACTIVE = 1920,
    parameter int H_FP     = 88,
    parameter int H_SYNC   = 44,
    parameter int H_BP     = 148,
    parameter int V_ACTIVE = 1080,
    parameter int V_FP     = 4,
    parameter int V_SYNC   = 5,
    parameter int V_BP     = 36,
    // ---- 图像尺寸（configs/geometry_b.yaml / psm_format.h）----
    parameter int WALL_N   = 96,      // 墙面 ROI 96x96
    parameter int HID_W    = 24,      // 隐藏图像 24x18
    parameter int HID_H    = 18,
    parameter int CAN_NX   = 9,       // 候选网格 9x9x3 = 243
    parameter int CAN_NY   = 9,
    parameter int CAN_NZ   = 3,
    parameter int PIX_W    = 24,      // RGB888
    // ---- 布局（见意图 1 与易错点 2：全部整数倍）----
    parameter int WALL_DST = 480,     // 96 x 5
    parameter int REC_DST_W= 720,     // 24 x 30
    parameter int REC_DST_H= 540,     // 18 x 30
    parameter int OSD_COLS = 32,      // 文本区 32 列 x 8 行
    parameter int OSD_ROWS = 8,
    localparam int RGN_W   = H_ACTIVE/2,
    localparam int RGN_H   = V_ACTIVE/2,
    localparam int H_TOTAL = H_ACTIVE + H_FP + H_SYNC + H_BP,
    localparam int V_TOTAL = V_ACTIVE + V_FP + V_SYNC + V_BP,
    localparam int CNT_W   = $clog2(H_TOTAL),
    localparam int LCNT_W  = $clog2(V_TOTAL),
    localparam int WALL_AW = $clog2(WALL_N*WALL_N),
    localparam int REC_AW  = $clog2(HID_W*HID_H),
    localparam int OSD_AW  = $clog2(OSD_COLS*OSD_ROWS)
) (
    input  wire logic                    clk,            // 视频/像素时钟域（见易错点 1）
    input  wire logic                    rst_n,

    // ---------------- 视频输出：1080p RGB AXI4-Stream + 同步 ----------------
    output logic [PIX_W-1:0]        vid_tdata,
    output logic                    vid_tvalid,     // = de（严格门控，见易错点 5）
    output logic                    vid_tlast,      // 行末
    output logic                    vid_tuser,      // 帧首
    output logic                    vid_de,
    output logic                    vid_hsync,      // 低有效
    output logic                    vid_vsync,      // 低有效
    // G3（2026-10-01）：下游 AXIS 反压。0 = 冻结光栅，把当前这拍像素保持到下游接收。
    // 接通后 vid_* 成为一个带 tready 的标准 AXIS 主；tready 恒 1 时行为与旧版逐拍相同。
    // 背景：旧版 vid_* 没有 tready（自由运行），接到 dma_disp/S2MM 时带宽不足会丢拍且
    // 不反压 —— 见 sim/results/bd_full_path_20260930.md §7 G3。
    input  wire logic                    vid_tready,

    // ---------------- 墙面 ROI 图像写口（ARM/采集侧填充，96x96 RGB888）----------------
    input  wire logic                    wall_we,
    input  wire logic [WALL_AW-1:0]      wall_waddr,
    input  wire logic [PIX_W-1:0]        wall_wdata,

    // ---------------- 重建图写口（PDHG 输出 24x18；灰度可复制到 RGB 三通道）----------------
    input  wire logic                    rec_we,
    input  wire logic [REC_AW-1:0]       rec_waddr,
    input  wire logic [PIX_W-1:0]        rec_wdata,

    // ---------------- 定位 / 可信度 ----------------
    input  wire logic [15:0]             cand_best_x,    // 最佳候选网格索引（0..8）
    input  wire logic [15:0]             cand_best_y,    // 0..8
    input  wire logic [15:0]             cand_best_z,    // 0..2（决定显示哪一层 z）
    input  wire logic [15:0]             selected_id,    // 线性候选号（0..242）
    input  wire logic [15:0]             conf_q016,      // CONFIDENCE 寄存器 0x1C
    input  wire logic                    low_conf,       // STATUS bit3
    input  wire logic                    force_gray,     // 来自 confidence_engine
    input  wire logic [7:0]              gray_reason,    // 灰屏原因位图

    // ---------------- OSD 文本写口（8x8 字形，ASCII）----------------
    input  wire logic                    osd_we,
    input  wire logic [OSD_AW-1:0]       osd_waddr,      // 行*32 + 列
    input  wire logic [7:0]              osd_wdata,

    // ---------------- 状态 ----------------
    output logic                    frame_tick,     // 帧首 1 拍脉冲（遥测时基）
    output logic                    busy
);

    // ------------------------------------------------------------------------
    // 颜色常量（RGB888）
    // ------------------------------------------------------------------------
    localparam logic [PIX_W-1:0] C_BG      = 24'h18_18_18;
    localparam logic [PIX_W-1:0] C_DIV     = 24'h40_40_40;
    localparam logic [PIX_W-1:0] C_PANEL   = 24'h00_00_00;
    localparam logic [PIX_W-1:0] C_GRID    = 24'h00_60_00;
    localparam logic [PIX_W-1:0] C_BEST    = 24'hFF_FF_00;
    localparam logic [PIX_W-1:0] C_BAR_BG  = 24'h30_00_00;
    localparam logic [PIX_W-1:0] C_BAR_FG  = 24'h00_FF_60;
    localparam logic [PIX_W-1:0] C_GRAY    = 24'h80_80_80;
    localparam logic [PIX_W-1:0] C_TEXT_FG = 24'hE0_E0_E0;
    localparam logic [PIX_W-1:0] C_TEXT_BG = 24'h10_10_10;

    // ------------------------------------------------------------------------
    // 存储
    // ------------------------------------------------------------------------
    logic [PIX_W-1:0] wall_mem [0:WALL_N*WALL_N-1];
    logic [PIX_W-1:0] rec_mem  [0:HID_W*HID_H-1];
    logic [7:0]       osd_mem  [0:OSD_COLS*OSD_ROWS-1];

    always_ff @(posedge clk) begin
        if (wall_we) wall_mem[wall_waddr] <= wall_wdata;
    end
    always_ff @(posedge clk) begin
        if (rec_we) rec_mem[rec_waddr] <= rec_wdata;
    end
    always_ff @(posedge clk) begin
        if (osd_we) osd_mem[osd_waddr] <= osd_wdata;
    end

    // ------------------------------------------------------------------------
    // 时序发生器（当前坐标）
    // ------------------------------------------------------------------------
    logic [CNT_W-1:0]  hcnt;
    logic [LCNT_W-1:0] vcnt;

    logic line_end, frame_end;
    assign line_end  = (hcnt == CNT_W'(H_TOTAL-1));
    assign frame_end = line_end && (vcnt == LCNT_W'(V_TOTAL-1));

    // G3 反压：vid_tvalid = active_d，所以「有一拍有效像素而下游未接收」时冻结整条光栅。
    // 冻结必须同时覆盖三处寄存器（hcnt/vcnt、输出流水 *_d、BRAM 同步读），否则只有计数器
    // 停住、输出流水仍会前进，光栅不会真正停顿（这是本改动的关键点）。
    // tready 恒 1 时 hold 恒 0，逐拍行为与旧版完全一致（compositor_tb 的整帧金模型不变）。
    // 这里只声明 net；具体赋值放在 active_d 声明之后（Verilog 要求先声明后引用）。
    wire hold;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hcnt <= '0;
            vcnt <= '0;
        end else if (hold) begin
            // 冻结（G3）：本拍像素保持到下游 tready 拉高
        end else if (line_end) begin
            hcnt <= '0;
            if (frame_end) vcnt <= '0;
            else           vcnt <= vcnt + 1'b1;
        end else begin
            hcnt <= hcnt + 1'b1;
        end
    end

    logic active_w;
    assign active_w = (hcnt < CNT_W'(H_ACTIVE)) && (vcnt < LCNT_W'(V_ACTIVE));

    // ------------------------------------------------------------------------
    // 布局常量
    // ------------------------------------------------------------------------
    // TL：x[0,RGN_W)      y[0,RGN_H)       墙图 480x480 居中于 (240,30)
    // TR：x[RGN_W,2RGN_W) y[0,RGN_H)       重建 720x540 居中于 (120,0)
    // BL：x[0,RGN_W)      y[RGN_H,2RGN_H)  候选网格 + 可信度条
    // BR：x[RGN_W,2RGN_W) y[RGN_H,2RGN_H)  遥测文本
    localparam int WALL_OFF_X = (RGN_W - WALL_DST)/2;       // 240
    localparam int WALL_OFF_Y = (RGN_H - WALL_DST)/2;       // 30
    localparam int REC_OFF_X  = (RGN_W - REC_DST_W)/2;      // 120
    localparam int REC_OFF_Y  = (RGN_H - REC_DST_H)/2;      // 0
    localparam int OSD_OFF_X  = 40;                         // 8 的倍数（易错点 3）
    localparam int OSD_OFF_Y  = 40;
    // 候选网格：9 格，格 24 px、节距 32 px，总 280 px
    localparam int CAN_PITCH  = 32;
    localparam int CAN_CELL   = 24;
    localparam int CAN_SPAN   = CAN_NX*CAN_PITCH - (CAN_PITCH-CAN_CELL);  // 280
    localparam int CAN_OFF_X  = (RGN_W - CAN_SPAN)/2;       // 340
    localparam int CAN_OFF_Y  = (RGN_H - CAN_SPAN)/2 - 40;  // 90
    // 可信度条：x 60..459、y 480..499
    localparam int BAR_X0     = 60;
    localparam int BAR_W_MAX  = 400;
    localparam int BAR_Y0     = 480;
    localparam int BAR_H      = 20;

    // ------------------------------------------------------------------------
    // 最近邻取址（易错点 5：全部用显式 16 bit 中间量）
    //   源像素 = (局部坐标 * 源边长) / 目标边长
    // ------------------------------------------------------------------------
    logic [15:0] wall_sx, wall_sy, rec_sx, rec_sy;

    assign wall_sx = (({4'b0, hcnt} - 16'(WALL_OFF_X)) * 16'(WALL_N)) / 16'(WALL_DST);
    assign wall_sy = (({5'b0, vcnt} - 16'(WALL_OFF_Y)) * 16'(WALL_N)) / 16'(WALL_DST);
    assign rec_sx  = (({4'b0, hcnt} - 16'(RGN_W + REC_OFF_X)) * 16'(HID_W)) / 16'(REC_DST_W);
    assign rec_sy  = (({5'b0, vcnt} - 16'(REC_OFF_Y)) * 16'(HID_H)) / 16'(REC_DST_H);

    // 线性地址（显式加宽，再切片；范围已由尺寸保证不越界）
    logic [19:0] wall_lin;
    logic [15:0] rec_lin;
    assign wall_lin = ({4'b0, wall_sy} * 20'(WALL_N)) + {4'b0, wall_sx};
    assign rec_lin  = ({4'b0, rec_sy}  * 16'(HID_W))  + {4'b0, rec_sx};

    logic [WALL_AW-1:0] wall_raddr;
    logic [REC_AW-1:0]  rec_raddr;
    assign wall_raddr = wall_lin[WALL_AW-1:0];
    assign rec_raddr  = rec_lin[REC_AW-1:0];

    // 同步读（地址用当前坐标）。G3：反压期间一并冻结，保证输出像素稳定。
    logic [PIX_W-1:0] wall_rdata, rec_rdata;
    always_ff @(posedge clk) begin
        if (!hold) begin
            wall_rdata <= wall_mem[wall_raddr];
            rec_rdata  <= rec_mem[rec_raddr];
        end
    end

    // ------------------------------------------------------------------------
    // 坐标与控制打一拍（渲染用，见意图 2）
    // ------------------------------------------------------------------------
    logic              active_d, line_last_d, frame_first_d, hsync_d, vsync_d;
    logic [CNT_W-1:0]  hcnt_d;
    logic [LCNT_W-1:0] vcnt_d;

    // G3 反压（见上文 wire hold 处的说明）。active_d 在此处才可见，故赋值放这里。
    assign hold = active_d && ~vid_tready;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hcnt_d        <= '0;
            vcnt_d        <= '0;
            active_d      <= 1'b0;
            line_last_d   <= 1'b0;
            frame_first_d <= 1'b0;
            hsync_d       <= 1'b1;
            vsync_d       <= 1'b1;
        end else if (hold) begin
            // 冻结（G3）：输出流水与光栅同步停住
        end else begin
            hcnt_d        <= hcnt;
            vcnt_d        <= vcnt;
            active_d      <= active_w;
            line_last_d   <= active_w && (hcnt == CNT_W'(H_ACTIVE-1));
            frame_first_d <= active_w && (hcnt == '0) && (vcnt == '0);
            hsync_d       <= ~(hcnt < CNT_W'(H_SYNC));
            vsync_d       <= ~(vcnt < LCNT_W'(V_SYNC));
        end
    end

    assign vid_de     = active_d;
    assign vid_tvalid = active_d;                       // 严格门控（易错点 5）
    assign vid_tlast  = line_last_d;
    assign vid_tuser  = frame_first_d;
    assign vid_hsync  = hsync_d;
    assign vid_vsync  = vsync_d;
    assign frame_tick = line_last_d && (vcnt_d == LCNT_W'(V_ACTIVE-1));

    // ------------------------------------------------------------------------
    // 字形 ROM（8x8，只含数字与少量符号，见易错点 4）
    // ------------------------------------------------------------------------
    function automatic logic [7:0] font_row(input logic [7:0] ch,
                                            input logic [2:0] row);
        logic [7:0] r;
        r = 8'h00;
        case (ch)
            8'h30: case (row) 3'd0: r=8'h3C; 3'd1: r=8'h66; 3'd2: r=8'h6E;
                              3'd3: r=8'h76; 3'd4: r=8'h66; 3'd5: r=8'h66;
                              3'd6: r=8'h3C; default: r=8'h00; endcase // '0'
            8'h31: case (row) 3'd0: r=8'h18; 3'd1: r=8'h38; 3'd2: r=8'h18;
                              3'd3: r=8'h18; 3'd4: r=8'h18; 3'd5: r=8'h18;
                              3'd6: r=8'h7E; default: r=8'h00; endcase // '1'
            8'h32: case (row) 3'd0: r=8'h3C; 3'd1: r=8'h66; 3'd2: r=8'h06;
                              3'd3: r=8'h0C; 3'd4: r=8'h18; 3'd5: r=8'h30;
                              3'd6: r=8'h7E; default: r=8'h00; endcase // '2'
            8'h33: case (row) 3'd0: r=8'h3C; 3'd1: r=8'h66; 3'd2: r=8'h06;
                              3'd3: r=8'h1C; 3'd4: r=8'h06; 3'd5: r=8'h66;
                              3'd6: r=8'h3C; default: r=8'h00; endcase // '3'
            8'h34: case (row) 3'd0: r=8'h0C; 3'd1: r=8'h1C; 3'd2: r=8'h3C;
                              3'd3: r=8'h6C; 3'd4: r=8'h7E; 3'd5: r=8'h0C;
                              3'd6: r=8'h0C; default: r=8'h00; endcase // '4'
            8'h35: case (row) 3'd0: r=8'h7E; 3'd1: r=8'h60; 3'd2: r=8'h7C;
                              3'd3: r=8'h06; 3'd4: r=8'h06; 3'd5: r=8'h66;
                              3'd6: r=8'h3C; default: r=8'h00; endcase // '5'
            8'h36: case (row) 3'd0: r=8'h1C; 3'd1: r=8'h30; 3'd2: r=8'h60;
                              3'd3: r=8'h7C; 3'd4: r=8'h66; 3'd5: r=8'h66;
                              3'd6: r=8'h3C; default: r=8'h00; endcase // '6'
            8'h37: case (row) 3'd0: r=8'h7E; 3'd1: r=8'h06; 3'd2: r=8'h0C;
                              3'd3: r=8'h18; 3'd4: r=8'h30; 3'd5: r=8'h30;
                              3'd6: r=8'h30; default: r=8'h00; endcase // '7'
            8'h38: case (row) 3'd0: r=8'h3C; 3'd1: r=8'h66; 3'd2: r=8'h66;
                              3'd3: r=8'h3C; 3'd4: r=8'h66; 3'd5: r=8'h66;
                              3'd6: r=8'h3C; default: r=8'h00; endcase // '8'
            8'h39: case (row) 3'd0: r=8'h3C; 3'd1: r=8'h66; 3'd2: r=8'h66;
                              3'd3: r=8'h3E; 3'd4: r=8'h06; 3'd5: r=8'h0C;
                              3'd6: r=8'h38; default: r=8'h00; endcase // '9'
            8'h3A: case (row) 3'd1: r=8'h18; 3'd2: r=8'h18; 3'd5: r=8'h18;
                              3'd6: r=8'h18; default: r=8'h00; endcase // ':'
            8'h2E: case (row) 3'd5: r=8'h18; 3'd6: r=8'h18; default: r=8'h00; endcase // '.'
            8'h2D: case (row) 3'd3: r=8'h7E; default: r=8'h00; endcase          // '-'
            8'h2F: case (row) 3'd0: r=8'h06; 3'd1: r=8'h0C; 3'd2: r=8'h0C;
                              3'd3: r=8'h18; 3'd4: r=8'h30; 3'd5: r=8'h30;
                              3'd6: r=8'h60; default: r=8'h00; endcase          // '/'
            default: r = 8'h00;                                                 // 未定义 -> 空白
        endcase
        font_row = r;
    endfunction

    // ------------------------------------------------------------------------
    // 渲染（全部使用延迟坐标）
    // ------------------------------------------------------------------------
    logic              in_tl, in_tr, in_bl, in_br;
    assign in_tl = (hcnt_d <  CNT_W'(RGN_W)) && (vcnt_d < LCNT_W'(RGN_H));
    assign in_tr = (hcnt_d >= CNT_W'(RGN_W)) && (hcnt_d < CNT_W'(2*RGN_W)) &&
                   (vcnt_d <  LCNT_W'(RGN_H));
    assign in_bl = (hcnt_d <  CNT_W'(RGN_W)) && (vcnt_d >= LCNT_W'(RGN_H));
    assign in_br = (hcnt_d >= CNT_W'(RGN_W)) && (hcnt_d < CNT_W'(2*RGN_W)) &&
                   (vcnt_d >= LCNT_W'(RGN_H));

    // 面板内的图像覆盖判定（用延迟坐标）
    logic in_wall_pic, in_rec_pic;
    assign in_wall_pic = (hcnt_d >= CNT_W'(WALL_OFF_X)) &&
                         (hcnt_d <  CNT_W'(WALL_OFF_X + WALL_DST)) &&
                         (vcnt_d >= LCNT_W'(WALL_OFF_Y)) &&
                         (vcnt_d <  LCNT_W'(WALL_OFF_Y + WALL_DST));
    assign in_rec_pic  = (hcnt_d >= CNT_W'(RGN_W + REC_OFF_X)) &&
                         (hcnt_d <  CNT_W'(RGN_W + REC_OFF_X + REC_DST_W)) &&
                         (vcnt_d >= LCNT_W'(REC_OFF_Y)) &&
                         (vcnt_d <  LCNT_W'(REC_OFF_Y + REC_DST_H));

    // 分区边框
    logic border_d;
    assign border_d = (hcnt_d == '0) || (hcnt_d == CNT_W'(RGN_W-1)) ||
                      (hcnt_d == CNT_W'(RGN_W)) ||
                      (hcnt_d == CNT_W'(H_ACTIVE-1)) ||
                      (vcnt_d == '0) || (vcnt_d == LCNT_W'(RGN_H-1)) ||
                      (vcnt_d == LCNT_W'(RGN_H)) ||
                      (vcnt_d == LCNT_W'(V_ACTIVE-1));

    // 候选网格（左下）
    logic [CNT_W-1:0]  glx, gly;
    logic              grid_hit, grid_best;
    assign glx = hcnt_d - CNT_W'(CAN_OFF_X);
    assign gly = vcnt_d - LCNT_W'(CAN_OFF_Y);
    assign grid_hit = in_bl &&
                      (hcnt_d >= CNT_W'(CAN_OFF_X)) &&
                      (hcnt_d <  CNT_W'(CAN_OFF_X + CAN_SPAN)) &&
                      (vcnt_d >= LCNT_W'(CAN_OFF_Y)) &&
                      (vcnt_d <  LCNT_W'(CAN_OFF_Y + CAN_SPAN)) &&
                      ((glx % CNT_W'(CAN_PITCH)) < CNT_W'(CAN_CELL)) &&
                      ((gly % LCNT_W'(CAN_PITCH)) < LCNT_W'(CAN_CELL));
    assign grid_best = grid_hit &&
                       ((glx / CNT_W'(CAN_PITCH)) == CNT_W'(cand_best_x)) &&
                       ((gly / LCNT_W'(CAN_PITCH)) == LCNT_W'(cand_best_y));

    // 可信度条（左下）：填充宽度 = conf/65536 * BAR_W_MAX
    logic [19:0]       bar_fill;
    logic              bar_bg, bar_fg;
    assign bar_fill = ({4'b0, conf_q016} * 20'(BAR_W_MAX)) >> 16;
    assign bar_bg   = in_bl && (hcnt_d >= CNT_W'(BAR_X0)) &&
                      (hcnt_d <  CNT_W'(BAR_X0 + BAR_W_MAX)) &&
                      (vcnt_d >= LCNT_W'(BAR_Y0)) &&
                      (vcnt_d <  LCNT_W'(BAR_Y0 + BAR_H));
    assign bar_fg   = bar_bg && ({4'b0, (hcnt_d - CNT_W'(BAR_X0))} < bar_fill);

    // OSD 文本（右下）
    logic [15:0] osd_row_s, osd_col_s, osd_lin;
    logic        osd_hit;
    logic [7:0]  osd_char;
    logic [7:0]  glyph_bits;
    logic [2:0]  glyph_row, glyph_col;
    assign osd_hit = in_br &&
                     (hcnt_d >= CNT_W'(RGN_W + OSD_OFF_X)) &&
                     (hcnt_d <  CNT_W'(RGN_W + OSD_OFF_X + OSD_COLS*8)) &&
                     (vcnt_d >= LCNT_W'(RGN_H + OSD_OFF_Y)) &&
                     (vcnt_d <  LCNT_W'(RGN_H + OSD_OFF_Y + OSD_ROWS*8));
    // **字形内的行列必须相对文本区原点取模**，不能直接取 vcnt[2:0]：
    //   原点 = RGN_H + OSD_OFF_Y = 540 + 40 = 580，而 580 % 8 = 4 ≠ 0，
    //   直接取 vcnt_d[2:0] 会让每行字形整体滚动 4 像素（看起来像「字在抖」）。
    //   水平方向 960 + 40 = 1000 恰好是 8 的倍数，所以不改也对，但为一致性一并改。
    // **SV 不允许对括号表达式做位选**（`(a - b)[2:0]` 会被 iverilog 判为 syntax error，
    // 2026-09-28 首次编译时发现），故先落到中间线网再选位。
    // 等价性：在 LCNT_W/CNT_W 位无符号运算下，低 3 位即 (a-b) mod 8，与原意相同。
    wire [LCNT_W-1:0] glyph_row_full = vcnt_d - LCNT_W'(RGN_H + OSD_OFF_Y);
    wire [CNT_W-1:0]  glyph_col_full = hcnt_d - CNT_W'(RGN_W + OSD_OFF_X);
    assign glyph_row = glyph_row_full[2:0];
    assign glyph_col = glyph_col_full[2:0];
    // 线性地址显式加宽（易错点 5）：行 = oly/8，列 = olx/8
    assign osd_row_s = ({5'b0, vcnt_d} - 16'(RGN_H + OSD_OFF_Y)) >> 3;
    assign osd_col_s = ({4'b0, hcnt_d} - 16'(RGN_W + OSD_OFF_X)) >> 3;
    assign osd_lin   = osd_row_s * 16'(OSD_COLS) + osd_col_s;
    // 异步读（意图 3）：256 字节走 LUTRAM
    assign osd_char   = osd_mem[osd_lin[OSD_AW-1:0]];
    assign glyph_bits = font_row(osd_char, glyph_row);

    // ------------------------------------------------------------------------
    // 像素选择
    // ------------------------------------------------------------------------
    logic [PIX_W-1:0] pix;
    always_comb begin
        pix = C_BG;

        if (force_gray || low_conf) begin
            // 整屏覆盖纯灰（意图 4）；左上角 8x8 块按原因着色
            pix = C_GRAY;
            if ((hcnt_d < 8) && (vcnt_d < 8)) begin
                // 优先级（高 -> 低）：数据完整性(bit[7:5]) > 几何哈希(bit4) >
                // 饱和(bit0) > SNR(bit1) > margin(bit2) > ρ(bit3)。
                // 完整性标志最高：数据本身不可信时，先暴露「数据坏了」而不是「质量差」。
                pix = gray_reason[7] ? 24'hFF_00_7F :   // proto_err（最高优先）
                      gray_reason[6] ? 24'h7F_00_FF :   // diff ovf
                      gray_reason[5] ? 24'h00_FF_7F :   // raw ovf_sticky
                      gray_reason[4] ? 24'hFF_FF_FF :   // 几何哈希不一致
                      gray_reason[0] ? 24'hFF_00_00 :   // 饱和比例
                      gray_reason[1] ? 24'hFF_A0_00 :   // SNR
                      gray_reason[2] ? 24'h00_A0_FF :   // margin
                      gray_reason[3] ? 24'hC0_00_FF :   // rho
                      24'h00_00_00;
            end
        end else if (in_tl) begin
            pix = in_wall_pic ? wall_rdata : C_PANEL;   // 左上：墙面 ROI
        end else if (in_tr) begin
            pix = in_rec_pic ? rec_rdata : C_PANEL;     // 右上：隐藏图像重建
        end else if (in_bl) begin
            pix = C_BG;                                 // 左下：候选网格 + 可信度条
            if (grid_hit) pix = grid_best ? C_BEST : C_GRID;
            if (bar_bg)   pix = bar_fg ? C_BAR_FG : C_BAR_BG;
        end else if (in_br) begin
            pix = C_TEXT_BG;                            // 右下：遥测文本
            if (osd_hit) pix = glyph_bits[3'd7 - glyph_col] ? C_TEXT_FG : C_TEXT_BG;
        end

        if (border_d) pix = C_DIV;

        // 消隐期不上屏（易错点 5 的反面：消隐期像素值不参与显示）
        if (!active_d) pix = 24'h00_00_00;
    end

    assign vid_tdata = pix;
    assign busy      = 1'b1;   // 显示是自由运行通路，永远在扫

endmodule

`default_nettype wire
