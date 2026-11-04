// ============================================================================
// raw_preproc.sv  ·  PenumbraSight RAW/Bayer 预处理流水
// 依据：plan_teacher.md §6.1（顶层数据流）/ §6.2（模块接口）/ §6.3（定点格式）
//       configs/geometry_b.yaml（墙网格 96、隐藏 24x18、D=1.03）
//       configs/fixed_point_b.yaml（位宽链与 value_packing）
//       src/hardware/preprocess.hpp（同算法的 HLS 参考实现）
//       sim/vectors/README.md（P1 只用整数 / P2 双绿不求平均）
//
// ---------------------------------------------------------------------------
// 本模块做什么（方案 §6.2 表中 raw_preproc 那一行）
// ---------------------------------------------------------------------------
//   RAW16 帧流（ROI 已由 ARM/DataMover 裁好，AXI DataMover 从 DDR 环缓搬出）
//     -> 减暗场偏置（先减后处理，与 preprocess.hpp 一致）
//     -> Bayer 解交织取 R / G1 / G2 / B（黑白时单通道直通）
//     -> 双绿取「和」G = G1 + G2
//     -> 10x10 分块累加（块内不跨行、不求平均、不做除法 = P1）
//     -> 1/4/8 帧时域累积（直接求和，末端不移位 = 官方数据实测口径）
//     -> 输出 WALL_N x WALL_N x CH 的块和流（默认 96x96x3）
//
// ---------------------------------------------------------------------------
// 三个必须讲清的设计意图
// ---------------------------------------------------------------------------
// 【意图 1】为什么块和寄存器是「一整块行」而不是「一个块」
//   解交织后是 960x960 的平面，10x10 分块得到 96x96 个块。输入流是**光栅序**
//   （先行后列），也就是说：raw 的第 0 行的 1920 个像素同时属于第 0 块行的
//   全部 96 个块。所以「只留一个块累加器」在光栅序下根本不成立——必须为当前
//   块行的 WALL_N 个块各留一个累加器（本设计：WALL_N x CH 个 BSW 位寄存器，
//   96x3=288 个，约 6 Kbit FF）。一个块行扫完（BLK_SIDE 条 raw 行），这 96 个
//   块**同时**完成，于是写回是「96 连发」而不是逐块零散写。
//
// 【意图 2】为什么帧累积要过一块 BRAM
//   1/4/8 帧累积意味着「同一个块的块和要在帧之间相加」，而帧是串行进来的，
//   所以必须有个 WALL_N^2 x (CH*OUT_BITS) 的累加器数组（96x96x72 bit
//   ≈ 663 Kbit ≈ 19 个 BRAM36）。目标器件 XCZU9EG 的 Block RAM 总量为 32.1 Mb，
//   这 19 块约占 2%，完全放得下。注意 ZU9EG 在 EG 系列里**没有 UltraRAM**
//   （A2 已记），所以这 19 块只能落在 BRAM 上 —— 而 32.1 Mb 足够。
//   资源基线见 01_分析报告/08_阶段A产出/A2_资源预算.md。
//   frm==0 时直接写（省掉一次清零扫描），frm>0 时读-改-写。
//
// 【意图 3】为什么输出要单独走一趟读出
//   累积完成后的值全在 BRAM 里，必须再扫一遍把它变成流。这一趟 96x96=9216 个
//   块、每块 CH 拍，共 27648 拍（约 0.18 ms @150 MHz），相对整帧 3.7 M 像素的
//   输入时间可忽略，故用「扫一遍」换掉一整套旁路寄存器是最划算的。
//
// ---------------------------------------------------------------------------
// 易错点（都是这个仓库里真实踩过的，改动前先读）
// ---------------------------------------------------------------------------
// (1) 跨行分块（v4_edge_cases 的 crossrow_wrong）
//     把二维索引展平后按 100 个像素分组，会在块行边界处把下一行的像素算进上
//     一行。本实现用 (ci, ri, bc, br) 四个独立计数器表达块内坐标，结构上不可能
//     跨行；并且用 proto_err 把「tlast 与块列计数器不一致」直接顶成粘滞错误。
//
// (2) 块和位宽：G 通道比 R/B 宽 1 bit
//     G = G1 + G2 是「和」不是平均（P2），所以 G 的块和最大是 R/B 的 2 倍。
//     BLOCK=10 时 R/B 需要 19 bit、G 需要 20 bit。因此内部累加器统一用
//     BSW = BS_BITS+1 = 21 bit，别只按 R/B 算。
//
// (3) 输出总线的符号性
//     方案 §6.2 把 raw_preproc 的输出写成「signed 24 bit」。块和本身是**非负**
//     的无符号量，本设计按「signed 24 bit 容器装非负值」实现：由于
//     ACC_BITS = 23，8 帧累积的最大值 6,552,000 < 2^23 = 8,388,608，MSB 恒为 0，
//     两种解释数值一致。**若哪天 ACC_BITS 被迫加到 24，MSB 会被点亮**，那时
//     这个「兼容写法」立刻变成符号错误，务必重新评估。
//
// (4) value_packing 一改，这个模块是第一个炸的
//     fixed_point_b.yaml 的 value_packing=packed（12 bit 右对齐）时，
//     块和 19->20 bit、8 帧累积 23 bit、差分 24 bit 全部自洽。
//     若首帧实拍发现相机是左移 4 位（0..65520），块和会跳到 23 bit、8 帧累积
//     26 bit、差分需要 27 bit（方案里写的 26 也不够）。本模块的 sat_add 会在
//     越界时**饱和并点亮 ovf_sticky**，这就是防止「亮区被静默削平」的那道闸。
//     只改本文件的位宽参数不够，差分级（vertical_diff）的 DIFF_BITS 必须同时加宽。
//
// (5) 饱和度统计是给 confidence_engine 用的
//     §6.6 要求「饱和像素比例 > 0.1% 强制灰屏」。本模块顺手统计整组累积范围内
//     的饱和像素数与总像素数（复位/组结束清零），ARM 或 confidence_engine 相除
//     即可。**不要**在模块内做除法（P1 原则）。
//
// ---------------------------------------------------------------------------
// 接口口径（端口一览；默认位宽见下方 parameter 列表）
// ---------------------------------------------------------------------------
//   输入
//     clk                  1      主/像素时钟（与 AXIS 同源；跨域见上层异步 FIFO）
//     rst_n                1      低有效异步复位
//     cfg_dark_level [15:0]       暗场偏置（先减后处理，负结果饱和到 0）
//     cfg_frames      [3:0]       累积帧数，只允许 1/4/8（其它值置 cfg_err）
//     cfg_crop_side  [15:0]       期望裁剪边长（0 = 不检查；须等于 2*BLOCK*WALL_N）
//     cfg_bayer_phase    1        0 = RGGB（左上为 R），1 = BGGR
//     s_axis_tdata   [15:0]       RAW16 容器（有效 12 bit，packed 右对齐）
//     s_axis_tvalid      1        像素有效
//     s_axis_tlast       1        帧内最后一个像素（须与块列计数一致，否则 proto_err）
//     s_axis_tuser       1        帧内第一个像素（SOF）
//     m_meas_tready      1        下游就绪（反压）
//   输出
//     s_axis_tready      1        仅 ST_ACC 期间为高；块行写回/输出读出期间拉低
//     m_meas_tdata  [23:0]        块和（signed 24 bit 容器装非负值，见易错点 3）
//     m_meas_tch      [1:0]       0=R（或单色）1=G 2=B
//     m_meas_tvalid      1        块和有效
//     m_meas_tlast       1        块行末
//     m_meas_tuser       1        帧首块
//     m_meas_tblk   [BW-1:0]      块线性索引（调试/坐标核对用）
//                                 语义 = 块的行优先线性索引 br*WALL_N + bc，
//                                 取值域 0..WALL_N^2-1，**恰好等于输出循环的
//                                 out_idx**（acc_mem 就是按这个索引存的，见
//                                 rmw_addr）。位宽 = $clog2(WALL_N*WALL_N)，
//                                 **随 WALL_N 变**：WALL_N=96 时是 14 位，
//                                 WALL_N=4（tb 的小尺寸配置）时是 4 位。
//                                 [2026-09-30 修] 原实现写的是
//                                 `BW'(out_idx)*BW'(WALL_N) + BW'(obc)`：out_idx
//                                 本身已经是线性索引，再乘一次 WALL_N 属重复计数，
//                                 且整个表达式在 BW 位宽上求值、结果被截断。
//                                 小尺寸实测（WALL_N=4/BW=4）16 个块里 12 个错、
//                                 只剩 4 个不同取值；B 档（WALL_N=96/BW=14）9216
//                                 个块里 9198 个错、只剩 1536 个不同取值。
//                                 现改为直接输出 out_idx。详见
//                                 sim/results/rtl_four_issues_20260930.md。
//     frame_done         1        一组累积完成（1 拍脉冲）
//     busy               1        非 IDLE / DONE
//     ovf_sticky         1        累积饱和（粘滞；位宽口径与相机不符的告警闸）
//     proto_err          1        tlast/tuser 与内部计数不一致（粘滞）
//     cfg_err            1        配置非法
//     sat_pix_cnt   [31:0]        本组内饱和像素数（§6.6 原料；本模块不做除法）
//     total_pix_cnt [31:0]        本组内总像素数
//   对应文档：plan_teacher.md §6.1（数据流）/ §6.2（模块接口表 raw_preproc 行）/
//             §6.3（定点格式：RAW 12-in-16、块和 20、8 帧累积 23）
//             configs/geometry_b.yaml（wall.n=96、BLOCK=10）
//             configs/fixed_point_b.yaml（bit_chain / value_packing）
//             src/hardware/preprocess.hpp（同算法 HLS 参考）
//             sim/vectors/README.md（P1 只用整数 / P2 双绿不求平均）
//
// ---------------------------------------------------------------------------
// 刻意保留的行为（不是 bug；改动前先确认是不是有意为之）
// ---------------------------------------------------------------------------
//   [R1] 双绿取「和」不取平均，且不做中间舍入（P2）。G 通道块和因此比 R/B 宽 1 bit，
//        内部累加器统一按 BSW = BS_BITS+1 设计。**不要**为了「三通道整齐」把它压平。
//   [R2] 全程不做除法、不做舍入（P1）。块和是整数累加；帧累积直接求和，末端不移位、
//        不乘倒数。归一化（Q2.16 meas_norm）刻意留给下游，本模块不越界。
//   [R3] 帧累积走 BRAM 读-改-写，且 frm==0 时**跳过读**（省一次全扫）。若改成
//        「先清零再逐帧累加」，会多出一次 9216 拍扫描 —— 那是行为变化，不是优化。
//   [R4] 输出用 signed 容器装非负值（易错点 3 已论证 MSB 恒为 0）。这是为对齐 §6.2
//        「96x96x3 signed 24 bit」的口径而刻意保留，不是类型笔误。
//   [R5] 饱和度只在模块内**累加**、不比较阈值（P1）。比例与阈值判定放在
//        confidence_engine（§6.6「>0.1% 强制灰屏」）。
//   [R6] 不实现 AXI4-MM 读主机。§6.2 的「AXI4-MM」指 DDR 侧取数通道，到本模块
//        边界已由 AXI DataMover 转成 AXI4-Stream；本模块刻意只做 AXIS 从端，
//        以便逐像素流水与单元测试。
//   [R7] 帧间在 ST_IDLE 持续清零块累加器，等价于「每帧开始前清零」，因此**没有**
//        独立的清零扫描状态。这是有意省掉的状态，不要误判为漏了复位。
//
// ---------------------------------------------------------------------------
// 仿真状态声明
// ---------------------------------------------------------------------------
//   本文件**没有在任何仿真器上跑过**：本机未安装 iverilog / Vivado / Verilator，
//   无法编译或仿真。文件内不存在任何仿真波形、覆盖率或时延数据。
//   配套的 raw_preproc_tb.sv 是待运行的自检测试平台，不是已通过的测试报告。
// ============================================================================

// 与 tb/raw_preproc_tb.sv 取同一值：RTL 缺 timescale 时 xsim 会因
// [XSIM 43-4099] 拒绝 elaboration（见 sim/results/xsim_first_run_20260929.md §6.1）。
`timescale 1ns/1ps
`default_nettype none

module raw_preproc #(
    // ---- 相机与几何（冻结值见 configs/geometry_b.yaml、configs/camera_b.yaml）----
    parameter int  IN_BITS    = 16,          // 相机容器位宽
    parameter int  VALID_BITS = 12,          // 有效位深（packed 右对齐时 = 12）
    parameter int  BLOCK      = 10,          // 10x10 分块（方案 §6.1）
    parameter int  WALL_N     = 96,          // 墙网格边长
    parameter int  CH         = 3,           // 3 = 彩色（RGB），1 = 黑白
    parameter bit  COLOR      = 1'b1,        // 1 = Bayer 解交织，0 = 单通道直通
    parameter int  MAX_FRAMES = 8,           // 最大累积帧数（1/4/8）
    // ---- 位宽链（configs/fixed_point_b.yaml / §6.3）----
    parameter int  BS_BITS    = 20,          // 单帧块和：packed 口径 12+7=19，留 1 bit 余量
    parameter int  ACC_BITS   = 23,          // 8 帧累积：packed 口径（方案 §6.3）
    parameter int  OUT_BITS   = 24,          // 输出总线（signed 容器装非负值，见易错点 3）
    // ---- 派生（写在参数表里，端口可直接引用）----
    localparam int CHW      = (CH > 1) ? 2 : 1,                    // 通道索引位宽
    localparam int CHCW     = (CH > 1) ? $clog2(CH) : 1,           // 通道计数器位宽
    localparam int RAW_SIDE = COLOR ? (2*BLOCK*WALL_N) : (BLOCK*WALL_N), // 裁剪边长 1920 / 960
    localparam int BLK_SIDE = COLOR ? (2*BLOCK) : BLOCK,           // 一个块占的 raw 边长 20 / 10
    localparam int CIW      = $clog2(BLK_SIDE),                    // 块内计数位宽
    localparam int NW       = $clog2(WALL_N),                      // 块行列索引位宽
    localparam int BW       = $clog2(WALL_N*WALL_N),               // 块线性索引位宽
    localparam int FW       = $clog2(MAX_FRAMES + 1),              // 帧计数位宽
    localparam int BSW      = BS_BITS + 1,                         // 块内累加器（给 G1+G2 多留 1 bit）
    localparam int WORD_W   = CH * OUT_BITS                        // BRAM 单字 = 全部通道
) (
    input  wire logic                    clk,
    input  wire logic                    rst_n,              // 低有效异步复位

    // ---------------- AXI-Lite 影子配置（由 fpga_control 写，帧内不可变）--------
    input  wire logic [IN_BITS-1:0]      cfg_dark_level,     // 暗场偏置
    input  wire logic [3:0]              cfg_frames,         // 1 / 4 / 8
    input  wire logic [15:0]             cfg_crop_side,      // 期望裁剪边长（0 = 不检查）
    input  wire logic                    cfg_bayer_phase,    // 0 = RGGB，1 = BGGR

    // ---------------- AXI4-Stream 从：RAW16（一拍一像素）----------------------
    // 说明：§6.2 的「AXI4-MM」指 DDR 侧取数通道；到本模块边界已由 AXI DataMover
    //       转成 AXI4-Stream（方案 §6.1 的数据流），故此处用 AXIS 接口。
    //       ps_contract.hpp 的 64 bit/4 像素打包在此前展开为逐像素，块累加逻辑不变。
    input  wire logic [IN_BITS-1:0]      s_axis_tdata,
    input  wire logic                    s_axis_tvalid,
    output logic                    s_axis_tready,      // 块行写回/输出读出期间拉低
    input  wire logic                    s_axis_tlast,       // 帧内最后一个像素
    input  wire logic                    s_axis_tuser,       // 帧内第一个像素（SOF）

    // ---------------- 测量流输出：WALL_N x WALL_N x CH 块和 ------------------
    //   顺序：行优先 -> 块列 -> 通道(R,G,B)；tlast 在每块行末；
    //   tuser 在整帧第一块第一通道；反压由 m_meas_tready 控制。
    output logic [OUT_BITS-1:0]     m_meas_tdata,
    output logic [CHW-1:0]          m_meas_tch,         // 0=R(或单色) 1=G 2=B
    output logic                    m_meas_tvalid,
    input  wire logic                    m_meas_tready,
    output logic                    m_meas_tlast,       // 块行末
    output logic                    m_meas_tuser,       // 帧首块
    output logic [BW-1:0]           m_meas_tblk,        // 块线性索引（调试/坐标核对）

    // ---------------- 状态与统计 --------------------------------------------
    output logic                    frame_done,         // 1 拍脉冲：一组累积完成
    output logic                    busy,
    output logic                    ovf_sticky,         // 累积饱和（位宽配置不一致的告警）
    output logic                    proto_err,          // tlast/tuser 与内部计数不一致
    output logic                    cfg_err,            // 配置非法
    output logic [31:0]             sat_pix_cnt,        // 本组内饱和像素数（§6.6 用）
    output logic [31:0]             total_pix_cnt       // 本组内总像素数
);

    // ------------------------------------------------------------------------
    // 常量
    // ------------------------------------------------------------------------
    // 饱和判定门限：packed 口径下 12 bit 满量程 = 4095。
    // 左移口径（fullscale）时这里应改成 16 bit 满量程，见易错点 4。
    localparam logic [IN_BITS-1:0] FULLSCALE = IN_BITS'((1 << VALID_BITS) - 1);

    // FSM 状态（用普通编码而不是 enum，便于被老工具链读）
    localparam logic [2:0] ST_IDLE     = 3'd0;  // 等 SOF（组起点/帧间）
    localparam logic [2:0] ST_ACC      = 3'd1;  // 逐像素分块累加
    localparam logic [2:0] ST_RMW_RD   = 3'd2;  // 帧累积：读旧值
    localparam logic [2:0] ST_RMW_WB   = 3'd3;  // 帧累积：写新值
    localparam logic [2:0] ST_OUT_RD   = 3'd4;  // 输出：读块和
    localparam logic [2:0] ST_OUT_BEAT = 3'd5;  // 输出：打 CH 拍
    localparam logic [2:0] ST_DONE     = 3'd6;  // 组结束脉冲

    // ------------------------------------------------------------------------
    // 状态与计数器
    // ------------------------------------------------------------------------
    logic [2:0]     state;
    logic [CIW-1:0] ci;          // 块内列（0..BLK_SIDE-1）
    logic [CIW-1:0] ri;          // 块内行（0..BLK_SIDE-1）
    logic [NW-1:0]  bc;          // 块列（0..WALL_N-1）
    logic [NW-1:0]  br;          // 块行（0..WALL_N-1）
    logic [FW-1:0]  frm;         // 当前累积到第几帧
    logic [NW-1:0]  rmw_cnt;     // 写回连发计数（0..WALL_N-1）
    // [2026-09-28 首次仿真修 / 第二轮修正] 块行累加器清零脉冲：一条块行写回完毕、
    //   要回 ST_ACC 收下一块行时，必须把 bs_* 清零，否则下一行会带着上一行的残值
    //   继续累加。
    //   ★ 第二轮修正的要害：这个脉冲必须**组合地**落在「写回态的最后一拍」，
    //   绝不能落在那之后的第一个 ST_ACC 拍（见下方 assign 处的完整论证）。
    logic           bs_clr;
    logic [BW-1:0]  out_idx;     // 输出块线性索引
    logic [NW-1:0]  obc;         // 输出块列（用于 tlast）
    logic [CHCW-1:0] out_ch;     // 输出通道计数

    // ------------------------------------------------------------------------
    // 块内累加器：当前块行的 WALL_N 个块，每块每通道一个（见意图 1）
    // ------------------------------------------------------------------------
    logic [BSW-1:0] bs_r [0:WALL_N-1];
    logic [BSW-1:0] bs_g [0:WALL_N-1];
    logic [BSW-1:0] bs_b [0:WALL_N-1];

    // ------------------------------------------------------------------------
    // 帧累积 BRAM：WALL_N^2 个字，每字 = {B, G, R} 拼在一起
    // 单读口 + 单写口 = 简单双口 BRAM，96x96x72bit ≈ 663 Kbit
    // ------------------------------------------------------------------------
    logic [WORD_W-1:0] acc_mem [0:WALL_N*WALL_N-1];
    logic [WORD_W-1:0] rmw_word;     // RMW 读出的旧值（1 拍延迟）
    logic [WORD_W-1:0] out_word;     // 输出阶段读出的值（1 拍延迟）

    // ------------------------------------------------------------------------
    // 输入像素：暗场扣除 + 通道判定
    // ------------------------------------------------------------------------
    logic [IN_BITS:0] pix_ext, dark_ext, pix_dc;
    assign pix_ext  = {1'b0, s_axis_tdata};
    assign dark_ext = {1'b0, cfg_dark_level};
    // 先减后处理：暗电平估高时会出负数，必须饱和到 0，否则后面块和会绕回成巨大正数
    assign pix_dc   = (pix_ext > dark_ext) ? (pix_ext - dark_ext) : {(IN_BITS+1){1'b0}};

    logic [1:0] bayer_ph;   // {块内行奇偶, 块内列奇偶}
    assign bayer_ph = {ri[0], ci[0]};

    // 通道判定：RGGB -> (0,0)=R (0,1)=G (1,0)=G (1,1)=B；BGGR 交换 R/B。
    // 注意：相位只由块内坐标奇偶决定，而 BLK_SIDE 一定是偶数（10/20），
    //       所以 bayer_ph 与「绝对 raw 坐标奇偶」一致，不需要 br/bc 参与。
    // [2026-09-28 首次仿真] 原为 always_comb，iverilog 14 对它报
    //   「always_comb process has no sensitivities」（该 simulator 对 always_comb
    //   敏感性的分析不完整）。这里改写为**连续赋值**：语义逐字不变，但任何仿真器
    //   都会得到同一套敏感性，消掉一条可能掩盖真问题的噪声告警。
    //   正确性由 raw_preproc_tb 的 T1/T2 三个通道的块和逐值比对覆盖。
    logic [1:0] pix_ch;
    assign pix_ch = (!COLOR)                     ? 2'd0    // 黑白：全部进 R 累加器
                  : (bayer_ph == 2'b01 ||
                     bayer_ph == 2'b10)          ? 2'd1    // 两路绿
                  : (bayer_ph == 2'b00)          ? (cfg_bayer_phase ? 2'd2 : 2'd0)
                                                 : (cfg_bayer_phase ? 2'd0 : 2'd2);

    // ------------------------------------------------------------------------
    // 握手与行/块边界
    // ------------------------------------------------------------------------
    logic in_fire;
    assign in_fire       = s_axis_tvalid && s_axis_tready;
    assign s_axis_tready = (state == ST_ACC);

    logic raw_row_end;   // 当前 raw 行最后一个像素
    logic row_end;       // 当前块行最后一个像素（BLK_SIDE 条 raw 行扫完）
    assign raw_row_end = in_fire && (ci == (BLK_SIDE-1)) && (bc == (WALL_N-1));
    assign row_end     = raw_row_end && (ri == (BLK_SIDE-1));

    // 协议自检：tlast 必须恰好落在 raw 行末；tuser 只能出现在块行/帧首
    logic proto_err_set;
    assign proto_err_set = in_fire &&
                           ((s_axis_tlast != raw_row_end) ||
                            (s_axis_tuser && !((bc == '0) && (ci == '0) && (ri == '0) && (br == '0))));

    // ------------------------------------------------------------------------
    // 饱和加（块和累加用）：越界饱和而不是回绕
    // ------------------------------------------------------------------------
    function automatic logic [OUT_BITS-1:0] sat_add(logic [OUT_BITS-1:0] a,
                                                    logic [OUT_BITS-1:0] b);
        logic [OUT_BITS:0] s;
        s = {1'b0, a} + {1'b0, b};
        sat_add = s[OUT_BITS] ? {OUT_BITS{1'b1}} : s[OUT_BITS-1:0];
    endfunction

    function automatic logic sat_ovf(logic [OUT_BITS-1:0] a,
                                     logic [OUT_BITS-1:0] b);
        logic [OUT_BITS:0] s;
        s = {1'b0, a} + {1'b0, b};
        sat_ovf = s[OUT_BITS];
    endfunction

    // 当前 rmw_cnt 对应的块和字（B 在高位、R 在低位，与输出通道顺序一致）
    logic [WORD_W-1:0] bs_word;
    always_comb begin
        bs_word = {(WORD_W){1'b0}};
        bs_word[0*OUT_BITS +: OUT_BITS] = bs_r[rmw_cnt];
        if (CH >= 2) bs_word[1*OUT_BITS +: OUT_BITS] = bs_g[rmw_cnt];
        if (CH >= 3) bs_word[2*OUT_BITS +: OUT_BITS] = bs_b[rmw_cnt];
    end

    logic [WORD_W-1:0] sum_word;
    logic              sum_ovf;
    always_comb begin
        sum_word = {(WORD_W){1'b0}};
        sum_ovf  = 1'b0;
        for (int c = 0; c < CH; c++) begin
            sum_word[c*OUT_BITS +: OUT_BITS] =
                sat_add(rmw_word[c*OUT_BITS +: OUT_BITS], bs_word[c*OUT_BITS +: OUT_BITS]);
            sum_ovf = sum_ovf | sat_ovf(rmw_word[c*OUT_BITS +: OUT_BITS],
                                        bs_word[c*OUT_BITS +: OUT_BITS]);
        end
    end

    // frm==0：直接写（省掉清零扫描）；frm>0：写饱和和
    logic [WORD_W-1:0] mem_wdata;
    assign mem_wdata = (frm == '0) ? bs_word : sum_word;

    localparam logic [BW-1:0] MEM_LAST = BW'(WALL_N*WALL_N - 1);
    logic [BW-1:0] rmw_addr;
    assign rmw_addr = BW'(br) * BW'(WALL_N) + BW'(rmw_cnt);

    // ------------------------------------------------------------------------
    // 块内累加（ST_ACC）与清零（ST_IDLE / 块行写回末拍）
    // ------------------------------------------------------------------------
    // ★ 2026-09-28 第二轮修（本模块第 6 个缺陷）：清零脉冲改在「写回态的最后一拍」
    //   组合产生，而不是在它之后的 ST_ACC 第一拍由寄存器发出。
    //   原写法（`bs_clr <= 1'b1` 与 `state <= ST_ACC` 同拍写入）会让清零**正好落在
    //   ST_ACC 的第一拍**，而那一拍的 `s_axis_tready` 已经为高：写回期间 tready=0，
    //   TB 一直把下一个像素顶在 valid 上等（见 raw_preproc_tb.sv 的 send_pixel_c），
    //   于是 tready 一起来握手立刻完成 —— 这个**已经握手成功**的像素却被清零分支
    //   挡在累加之外，等价于「计数器照常前进、累加器少加一个像素」。
    //   位置恒为「新块行的第 1 个块 (bc=0) 的第 1 个像素 (ri=ci=0)」，即块线性索引
    //   br*WALL_N —— 与实测「错误只出现在 blk=4/8/12、块 0 正常」完全一致
    //   （块 0 走的是 ST_IDLE -> ST_ACC，tready 在 ST_IDLE 为低，不存在这一拍）。
    //   数值也逐位对得上：TB 激励 pix_of(f,r,c)=(37r+101c+197f)%4093，该像素
    //   r=br*BLK_SIDE、c=0，逐帧缺口 = 37*BLK_SIDE*br + 197f，NFRM 帧求和 =
    //   148*BLK_SIDE*br + 197*NFRM*(NFRM-1)/2。实测 T1（BLK_SIDE=4,NFRM=4）
    //   br=1 得 592+1182=1774、br=2 得 2366、br=3 得 2958；T2（单帧、暗电平 100）
    //   得 48/196/344；T3（BLK_SIDE=2）得 1478/1774/2070 —— 九个数全部吻合。
    //   修法安全性的两个前提：① ST_RMW_WB 期间 tready=0，不可能有像素在途；
    //   ② bs_word 是对 bs_* 的**组合读**，而非阻塞赋值要到拍末才生效，所以
    //   本拍写回用的数据仍然是旧值（正确），下一拍（ST_ACC 第一拍）累加器已为 0。
    assign bs_clr = (state == ST_RMW_WB) && (rmw_cnt == (WALL_N-1)) && (br != (WALL_N-1));

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < WALL_N; i++) begin
                bs_r[i] <= '0;
                bs_g[i] <= '0;
                bs_b[i] <= '0;
            end
        end else if ((state == ST_IDLE) || bs_clr) begin
            // 帧间/等待 SOF 期间持续清零，等价于「每帧开始前清累加器」，
            // 不需要额外的清零扫描状态。
            // bs_clr：每条块行写回的**最后一拍**再清一次（块行之间回 ST_ACC，不再经过
            //   ST_IDLE，故不能只靠上面的条件）。
            // ★ 不变量：这两条清零路径都**不允许与 in_fire 撞车**。ST_IDLE 期间
            //   tready=0（s_axis_tready = state==ST_ACC），ST_RMW_WB 期间同样 tready=0，
            //   所以清零永远发生在「不可能握手成功」的拍上。若哪天把清零挪到 ST_ACC
            //   的第一拍（曾经就是这么写的），那一拍握手成功的像素会被静默丢弃 ——
            //   这正是第 6 个缺陷。改动前请先确认这条不变量。 
            for (int i = 0; i < WALL_N; i++) begin
                bs_r[i] <= '0;
                bs_g[i] <= '0;
                bs_b[i] <= '0;
            end
        end else if (in_fire) begin
            if (pix_ch == 2'd0) begin
                bs_r[bc] <= bs_r[bc] + BSW'(pix_dc);
            end else if (pix_ch == 2'd1) begin
                // G1 与 G2 以「和」的形式进入同一个累加器（P2：不做中间舍入）
                bs_g[bc] <= bs_g[bc] + BSW'(pix_dc);
            end else begin
                bs_b[bc] <= bs_b[bc] + BSW'(pix_dc);
            end
        end
    end

    // ------------------------------------------------------------------------
    // 块内/块列 计数器
    //   **br（块行）不在这里** —— 它只在本文件末尾的主状态机里赋值。
    //   2026-09-28 修：原先 br 在本块（清 ST_IDLE）与主状态机（ST_RMW_WB 自增）
    //   **两处**赋值，构成「同一触发器被两个 always_ff 驱动」。Vivado 综合时
    //   把本块那两处恒为 0 的赋值折叠成一个 GND 常量驱动，然后报
    //     Synth 8-6859 multi-driven net on pin Q，1st driver 'br_reg[k]__0/Q'，2nd driver 'GND'
    //     Synth 8-6858 ... constant driver which has been preserved, other driver is ignored
    //   —— **保留了 GND、忽略了寄存器**，于是硬件上 br 恒为 0，
    //   「最后一条块行」（br == WALL_N-1）永远不成立、帧永远收不了尾。
    //   共 7 位 × 3 条 = 21 条 critical warning，全部指到 proto_err_set 那行的 br。
    //   iverilog 不报（两个 always 块驱动同一变量的解析规则与综合器不同），
    //   所以 make rtl-sim 全绿并不能发现它。br 的清零已移入主状态机的复位段
    //   与三处 state <= ST_IDLE，时序与原先等价（都在进入 IDLE 的同一边沿生效）。
    // ------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ci <= '0;
            ri <= '0;
            bc <= '0;
        end else if (state == ST_IDLE) begin
            ci <= '0;
            ri <= '0;
            bc <= '0;
        end else if (in_fire) begin
            if (row_end) begin
                // 块行收尾：交给 RMW 阶段写回，块内计数器归零。
                // **注意 br 不在这里自增**——RMW 阶段要用 br 判断「这是不是最后一条
                // 块行」（br == WALL_N-1），若提前自增，WALL_N 为 2 的幂时 br 会回绕，
                // 末块行判定将永远不成立。br 的自增放在 ST_RMW_WB 里。
                ci <= '0;
                bc <= '0;
                ri <= '0;
            end else if (ci == (BLK_SIDE-1)) begin
                ci <= '0;
                if (bc == (WALL_N-1)) begin
                    bc <= '0;
                    ri <= ri + 1'b1;     // 换 raw 行
                end else begin
                    bc <= bc + 1'b1;     // 换块列
                end
            end else begin
                ci <= ci + 1'b1;
            end
        end
    end

    // ------------------------------------------------------------------------
    // 帧累积 BRAM 读口（两个阶段共用一块存储，靠状态互斥）
    // ------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (state == ST_RMW_RD) begin
            rmw_word <= acc_mem[rmw_addr];
        end else if (state == ST_OUT_RD) begin
            // acc_mem 是**按块线性索引**存放的（写回用 rmw_addr = br*WALL_N + rmw_cnt，
            // 共 WALL_N*WALL_N 格），因此读回也必须是同一个线性索引 out_idx。
            // 2026-09-28 首次仿真：曾误以为读地址漏了 obc 而改成 out_idx*WALL_N+obc，
            // **方向错了** —— 真正的缺陷在输出状态机的循环嵌套（见 ST_OUT_BEAT）。
            out_word <= acc_mem[out_idx];
        end
    end

    always_ff @(posedge clk) begin
        if (state == ST_RMW_WB) acc_mem[rmw_addr] <= mem_wdata;
    end

    // ------------------------------------------------------------------------
    // 饱和度统计（§6.6「饱和像素比例 > 0.1% 强制灰屏」的原料）
    // 只在 ST_ACC 期统计；组结束（ST_DONE）清零，故是「本组累积范围」的统计量。
    // ------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sat_pix_cnt   <= 32'd0;
            total_pix_cnt <= 32'd0;
        end else if (state == ST_DONE) begin
            sat_pix_cnt   <= 32'd0;
            total_pix_cnt <= 32'd0;
        end else if (in_fire) begin
            total_pix_cnt <= total_pix_cnt + 32'd1;
            if (s_axis_tdata >= FULLSCALE) sat_pix_cnt <= sat_pix_cnt + 32'd1;
        end
    end

    // ------------------------------------------------------------------------
    // 主状态机
    // ------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= ST_IDLE;
            frm         <= '0;
            rmw_cnt     <= '0;
            br          <= '0;      // br 只在本状态机里赋值，见上「块内/块列 计数器」处的说明
            out_idx     <= '0;
            obc         <= '0;
            out_ch      <= '0;
            frame_done  <= 1'b0;
            ovf_sticky  <= 1'b0;
            proto_err   <= 1'b0;
        end else begin
            frame_done <= 1'b0;

            if (proto_err_set) proto_err <= 1'b1;
            // 溢出告警只在「读-改-写」阶段有意义：frm==0 直接写 bs_word，
            // 此时 rmw_word 是未初始化内容，若不做 frm 门控会误报。
            if (sum_ovf && (state == ST_RMW_WB) && (frm != '0)) ovf_sticky <= 1'b1;

            case (state)
                // ---------------- 等 SOF（组起点，也用于帧间对齐）---------------
                ST_IDLE: begin
                    rmw_cnt <= '0;
                    out_idx <= '0;
                    obc     <= '0;
                    out_ch  <= '0;
                    if (s_axis_tvalid && s_axis_tuser && !cfg_err) begin
                        state <= ST_ACC;
                    end
                end

                // ---------------- 逐像素分块累加 ----------------
                ST_ACC: begin
                    if (row_end) begin
                        rmw_cnt <= '0;
                        // frm==0 直接写，省一次读；frm>0 走读-改-写
                        state <= (frm == '0) ? ST_RMW_WB : ST_RMW_RD;
                    end
                end

                // ---------------- 帧累积：读旧值 ----------------
                ST_RMW_RD: state <= ST_RMW_WB;

                // ---------------- 帧累积：写新值（含 96 连发收尾判断）----------
                ST_RMW_WB: begin
                    if (rmw_cnt == (WALL_N-1)) begin
                        // 一条块行的 96 个块全部写回完毕
                        if (br == (WALL_N-1)) begin
                            // 本帧所有块行都写完了
                            if (frm == FW'(cfg_frames-1)) begin
                                // 组累积完成 -> 扫一遍 BRAM 出流
                                out_idx <= '0;
                                obc     <= '0;
                                out_ch  <= '0;
                                state   <= ST_OUT_RD;
                            end else begin
                                frm     <= frm + 1'b1;
                                rmw_cnt <= '0;
                                br      <= '0;        // 下一帧从第 0 条块行重新开始
                                state   <= ST_IDLE;   // 等下一帧的 SOF
                            end
                        end else begin
                            br      <= br + 1'b1;     // 前进到下一块行
                            rmw_cnt <= '0;
                            // ★ 2026-09-28 首次仿真修：这里必须回 **ST_ACC** 去收下一
                            //   块行的像素。原实现是又进写回态（frm==0 ? ST_RMW_WB :
                            //   ST_RMW_RD），于是**没有输入也能继续跑**：rmw_cnt 在
                            //   0..WALL_N-1 上空转、每 WALL_N 拍把 br 加一，直到
                            //   br==WALL_N-1 就宣告本帧完成。
                            //   探针实测（-DRP_DEBUG）：16x16 的帧只吃了 64 个像素
                            //   （= 一个块行）就在 16 拍内把 br 走完（(WALL_N 个块行)
                            //   ×(WALL_N 个条目) = 4×4 = 16，与状态迁移时刻 cyc 69->85
                            //   完全吻合），随后 frm 加 1、回到 ST_IDLE；本帧剩余 192 个
                            //   像素既没被累加、也让 TB 的 wait(frame_done) 永久挂死。
                            //   注：rmw_cnt 的界 (WALL_N-1) 与 rmw_addr 的构成
                            //   (br*WALL_N + rmw_cnt) 是**对的**，不要一起改。
                            state   <= ST_ACC;
                            // 注：块行累加器的清零由**组合**的 bs_clr 在**本拍**（写回
                            //   末拍）完成，刻意不在这里发寄存器脉冲 —— 若在本拍发出，
                            //   它会在下一个 ST_ACC 拍生效，正好吞掉那一拍握手成功的
                            //   像素。完整论证见 bs_clr 的 assign 处。
                        end
                    end else begin
                        rmw_cnt <= rmw_cnt + 1'b1;
                        state   <= (frm == '0) ? ST_RMW_WB : ST_RMW_RD;
                    end
                end

                // ---------------- 输出：读块和 ----------------
                ST_OUT_RD: state <= ST_OUT_BEAT;

                // ---------------- 输出：打 CH 拍 ----------------
                ST_OUT_BEAT: begin
                    if (m_meas_tvalid && m_meas_tready) begin
                        if (out_ch == CHCW'(CH-1)) begin
                            // 本块最后一拍：前进到下一块（**线性索引**）
                            if (out_idx == MEM_LAST) begin
                                state <= ST_DONE;
                            end else begin
                                out_idx <= out_idx + 1'b1;
                                // ★ 2026-09-28 首次仿真修：原实现把 obc 也当成一层循环
                                //   （out_idx × obc 嵌套），于是 16 格的 acc_mem 被输出
                                //   64 次（192 拍，期望 48 拍），且同一块行的 4 个块共用
                                //   一格。
                                //   obc 的正确定位只是「块行内列号」，仅用于 tlast；
                                //   它跟随 out_idx 同步环绕，**不参与寻址**。
                                obc     <= (obc == (WALL_N-1)) ? '0 : obc + 1'b1;
                                out_ch  <= '0;
                                state   <= ST_OUT_RD;
                            end
                        end else begin
                            out_ch <= out_ch + 1'b1;   // 同一块的下一通道，读数据保持
                        end
                    end
                end

                // ---------------- 组结束 ----------------
                ST_DONE: begin
                    frame_done <= 1'b1;
                    frm        <= '0;
                    br         <= '0;        // 组结束，块行计数器归零
                    state      <= ST_IDLE;
                end

                default: begin
                    state <= ST_IDLE;
                    br    <= '0;
                end
            endcase
        end
    end

    // ------------------------------------------------------------------------
    // 输出通道（组合）
    // ------------------------------------------------------------------------
    assign m_meas_tvalid = (state == ST_OUT_BEAT);
    assign m_meas_tdata  = out_word[out_ch*OUT_BITS +: OUT_BITS];
    assign m_meas_tch    = out_ch[CHW-1:0];
    assign m_meas_tlast  = (state == ST_OUT_BEAT) && (obc == (WALL_N-1)) &&
                           (out_ch == CHCW'(CH-1));
    // ★ 2026-09-28 首次仿真修：原缺 `(obc == '0)`，于是**块行 0 的每个块首拍都拉 tuser**
    //   （实测 TB 在 beat 3/6/9 报「tuser 位置错误」）。tuser 语义是「帧首块」，
    //   帧首块 = 块行 0 的第 0 块第 0 通道，三个条件缺一不可。
    assign m_meas_tuser  = (state == ST_OUT_BEAT) && (out_idx == '0) &&
                           (obc == '0) && (out_ch == '0);
    // [2026-09-30 修] 块线性索引 = br*WALL_N + bc，**就是 out_idx 本身**
    //   （acc_mem 按 rmw_addr = br*WALL_N + rmw_cnt 存放，输出阶段用同一索引读出）。
    //   原实现 `BW'(out_idx)*BW'(WALL_N) + BW'(obc)` 把「已经是线性索引」的 out_idx
    //   再乘一次 WALL_N，且整个表达式在 BW 位宽上求值被截断 —— 小尺寸 16 块错 12、
    //   B 档 9216 块错 9198（只剩 1536 个不同取值）。直接输出 out_idx 才与端口语义
    //   「块线性索引」以及位宽 clog2(WALL_N^2) 自洽。见 sim/results/rtl_four_issues_20260930.md。
    assign m_meas_tblk   = out_idx;

    assign busy = (state != ST_IDLE) && (state != ST_DONE);

    // ------------------------------------------------------------------------
    // 配置检查（把 ps_contract.hpp 的 ps_validate_spec 搬到硬件里）
    //   * 帧数只允许 1/4/8（方案 §6.3 的位宽链是按这三个值推的）
    //   * 裁剪边长必须等于 2*BLOCK*WALL_N（彩色），否则墙网格静默取整
    // ------------------------------------------------------------------------
    always_comb begin
        cfg_err = 1'b0;
        if (!((cfg_frames == 4'd1) || (cfg_frames == 4'd4) || (cfg_frames == 4'd8))) begin
            cfg_err = 1'b1;
        end
        if (cfg_frames > MAX_FRAMES) cfg_err = 1'b1;
        if (cfg_crop_side != 16'd0 && cfg_crop_side != 16'(RAW_SIDE)) cfg_err = 1'b1;
    end

// ----------------------------------------------------------------------------
// 临时调试探针（仅在 -DRP_DEBUG 下编译进来；默认不参与综合也不影响仿真）
// 用途：2026-09-28 定位「T1 卡死、frame_done 永不置起」—— 打印状态迁移、
//       前若干输入/输出拍，以及每 2000 周期的心跳（心跳能直接暴露卡在哪一状态）。
// ----------------------------------------------------------------------------
`ifdef RP_DEBUG
    int         dbg_cyc     = 0;
    int         dbg_in_cnt  = 0;
    int         dbg_out_cnt = 0;
    logic [2:0] dbg_state_d = 3'h7;
    always_ff @(posedge clk) begin
        if (rst_n) begin
            dbg_cyc <= dbg_cyc + 1;
            if (state !== dbg_state_d) begin
                $display("[DBG-ST ] %0d -> %0d  frm=%0d in=%0d out=%0d cyc=%0d",
                         dbg_state_d, state, frm, dbg_in_cnt, dbg_out_cnt, dbg_cyc);
                dbg_state_d <= state;
            end
            if (in_fire) begin
                if (dbg_in_cnt < 10)
                    $display("[DBG-IN ] in#%0d data=%0d tuser=%b tlast=%b", dbg_in_cnt,
                             s_axis_tdata, s_axis_tuser, s_axis_tlast);
                dbg_in_cnt <= dbg_in_cnt + 1;
            end
            if (m_meas_tvalid && m_meas_tready) begin
                if (dbg_out_cnt < 20)
                    $display("[DBG-OUT] out#%0d tblk=%0d tch=%0d data=%0d tlast=%b tuser=%b",
                             dbg_out_cnt, m_meas_tblk, m_meas_tch, m_meas_tdata,
                             m_meas_tlast, m_meas_tuser);
                dbg_out_cnt <= dbg_out_cnt + 1;
            end
            if (dbg_cyc % 3000 == 0)
                $display("[DBG-HB ] cyc=%0d state=%0d frm=%0d in=%0d out=%0d cfg_err=%b proto=%b",
                         dbg_cyc, state, frm, dbg_in_cnt, dbg_out_cnt, cfg_err, proto_err);
        end
    end
`endif

endmodule

`default_nettype wire
