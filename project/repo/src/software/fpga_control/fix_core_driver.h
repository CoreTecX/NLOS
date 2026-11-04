/* ============================================================================
 * PenumbraSight · 板端 PS 侧「定点四核」调用层
 * src/software/fpga_control/fix_core_driver.h
 *
 * 这一层把 board/fix_core.{h,c} 的「配置 -> 启动 -> 等待 -> 读结果」四个动作
 * 编排成一条可跑的四核流水：
 *
 *     打开核（×4） -> 写 m_axi 缓冲基址 -> 写标量 -> ap_start -> 等 ap_done
 *     -> 读回带 _VLD 的结果
 *
 * 与双精度层的隔离（硬约束）
 * ---------------------------------------------------------------------------
 * 本层**只**用 board/ps_fix_regs.h 的偏移与 board/fix_core.h 的原语，基址在
 * 0xA0000000 起的八个 64 KiB 段。它与 src/software/include/ps_regs.h 的
 * 16 字窗口（0x00~0x3C）**不是一套寄存器**：标量个数、名字、位定义、偏移全不同。
 * 两者混用的症状是「作业能起，结果全是垃圾」，不报错。故：
 *   · 本层不 include ps_regs.h；
 *   · 下面的 #error 挡一次「先 include 双精度层再 include 本层」的误用。
 *
 * 顺序约束（本层负责落实，原语在 fix_core）
 * ---------------------------------------------------------------------------
 *   1) 64 位基址**先写 HI 再写 LO** —— 由 ps_fix_set_base() 保证；本层只调它，
 *      绝不自己拆高低半字（见 .c 里的说明）。
 *   2) 带 _VLD 的结果**先读数据再读 VLD** —— 由 ps_fix_read_vld()/read_vld64()
 *      保证（VLD 是 ap_vld，读清）。
 *   3) 四核启动顺序固定 proj -> lr_fwd -> lr_adj -> pdhg（见 ps_fix_core_order）。
 *
 * psm 包长/偏移一律从格式定义头文件取
 * ---------------------------------------------------------------------------
 * 模型候选记录内的 U/S/V/mask 偏移会由并行任务改动（可能加 s_max 字段），
 * 故本层**不硬编码**任何包长或数组偏移：全部走 psm_format.h 的
 * ps_psm_* 帮你算（见 ps_fix_model_view）。
 *
 * 帧量程接缝（2026-09-30 补齐）
 * ---------------------------------------------------------------------------
 * RTL 预处理链（raw_preproc -> vertical_diff）出的是 signed 24 bit 原始整数差分，
 * 定点投影核要的是**通道优先的 Q2.16 码字** `meas[ch*M + m]`。两者之间缺一级
 * 信号调理，此前只在 Python（build 区 seam_prepare.py）里有。本层现在把它接进
 * 流程，唯一入口是：
 *   · ps_fix_prepare_meas()      RTL 输出拍 -> job->meas 缓冲（走 meas_norm）
 *   · ps_fix_fold_frame_scale()  该帧共享指数 -> 「码字 -> 绝对测量」乘数
 *   · ps_fix_proj_attach_meas()  上面两步的编排：填 job->meas 并回报乘数
 * 口径沿用 src/software/meas_norm（权威 = src/model/.../quantize.py 的
 * shared_exponent），本层**不另立一套**。注意这一步是**纯内存操作、不碰寄存器**，
 * 所以影子窗口也能跑通（见 src/software/test_meas_bridge.c）。
 *
 * 未验证声明
 * ---------------------------------------------------------------------------
 * **尚未在板上跑过。** 本层提供两条窗口：
 *   · ps_fix_window_open_shadow  主机影子内存：可立即运行，验的是寄存器映射、
 *     基址 HI/LO、VLD 读序、四核顺序、结果字段拼接 —— 与 fpga_control 的桩件
 *     同一类，**不构成任何硬件或性能结论**。
 *   · ps_fix_window_open_uio     真板 /dev/uioN + mmap：已按标准 UIO 填实
 *     （逻辑在 fpga_control/uio_backend.{h,c}，系统调用在 uio_backend_posix.c）。
 *     主机（非 Linux）构建下没有 POSIX 后端，调用返回 PS_FIX_DRV_ERR_UNAVAILABLE。
 *     **系统调用本身未在板上跑过**；设备树/UIO 节点前提与板上待验清单见
 *     sim/results/uio_entry_20261001.md。
 * ==========================================================================*/

#ifndef PENUMBRASIGHT_FIX_CORE_DRIVER_H
#define PENUMBRASIGHT_FIX_CORE_DRIVER_H

/* 与双精度层隔离：同一翻译单元里先 include ps_regs.h 再 include 本头 -> 直接报错。 */
#if defined(PENUMBRASIGHT_PS_REGS_H)
#error "fix_core_driver 与双精度层 ps_regs.h 不可混用（见本文件头「与双精度层的隔离」）"
#endif

#include <stddef.h>
#include <stdint.h>

#include "fix_core.h"      /* board/fix_core.h -> board/ps_fix_regs.h */
#include "psm_format.h"    /* 模型包字段与偏移（不硬编码） */
#include "meas_norm.h"     /* RTL 24 位打包拍 -> 定点核 meas（本层的信号调理级） */

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------- 本层返回码 */
/* fix_core.h 已定义 PS_FIX_OK / PS_FIX_ERR_TIMEOUT / PS_FIX_ERR_ARG。 */
#define PS_FIX_DRV_ERR_UNAVAILABLE (-3)  /* 本平台无 UIO 后端（非 Linux，如主机构建）*/
#define PS_FIX_DRV_ERR_ALIGN       (-4)  /* m_axi 缓冲区基址未按 PS_PSM_ARRAY_ALIGN 对齐 */
#define PS_FIX_DRV_ERR_ARGS        (-5)  /* 作业参数缺失或自相矛盾 */
/* 下面五个与 uio_backend.h 的 PS_UIO_ERR_* 同数字，ps_fix_window_open_uio 直接透传。 */
#define PS_FIX_DRV_ERR_NO_DEVICE   (-6)  /* 扫描 /sys/class/uio 找不到匹配设备 */
#define PS_FIX_DRV_ERR_PERM        (-7)  /* open(/dev/uioN) 被拒（权限/占用） */
#define PS_FIX_DRV_ERR_MAP         (-8)  /* mmap(map0) 失败 */
#define PS_FIX_DRV_ERR_MAP_SIZE    (-9)  /* map0 的 addr/size 读不到，或覆盖不了四核窗口 */
#define PS_FIX_DRV_ERR_IO          (-10) /* read/close 等 I/O 失败 */
/* 增益入口拒绝（A1）：候选的 u_scale/v_scale 不是**精确的 2 的幂**，或指数超 int8。
 * 这类模型不得上板（否则 kGainExpInvalid 会静默退回 Q16.16，gain_q16 取整到 0，
 * 核输出全零）。判据见 docs/gain_representation.md §五。 */
#define PS_FIX_DRV_ERR_GAIN_EXP    (-11)

/* ---------------------------------------------------------------- 映射窗口 */

/* 窗口种类的取值。0 表示未打开。 */
#define PS_FIX_WIN_NONE   0
#define PS_FIX_WIN_SHADOW 1   /* 主机影子内存 */
#define PS_FIX_WIN_UIO    2   /* 真板 /dev/uioN + mmap */

typedef struct {
    volatile uint32_t *window;       /* 窗口首地址（覆盖 0xA0000000 起的八段） */
    uint32_t           window_phys;  /* 窗口物理基址（= PS_FIX_CORE_PROJ_CTRL） */
    uint32_t           window_bytes; /* 窗口字节数（四核八段建议 512 KiB） */
    int                kind;         /* PS_FIX_WIN_* */
    void              *storage;      /* 影子内存后端指向调用方给的内存 */
    int                fd;           /* UIO 后端预留 */
} ps_fix_window_t;

/* 主机影子窗口。storage 由调用方提供并在生命周期内保持有效；bytes 至少覆盖
 * 从 PS_FIX_CORE_PROJ_CTRL 到最后一个核 ctrl_r 段末，建议 512 KiB。 */
int  ps_fix_window_open_shadow(ps_fix_window_t *w, void *storage, uint32_t bytes);

/* 真板窗口：/dev/uioN + mmap（标准 UIO）。
 *   uio_dev 语义：
 *     "/dev/uioN"            直接打开该设备；
 *     "auto" / NULL / ""     扫 /sys/class/uio/uioN/name，取含
 *                            PS_UIO_DEFAULT_TOKEN（"penumbrasight"）的那个；
 *     其它字符串             当作 sysfs name 里的令牌扫描。
 * 需要的映射窗口 = 四核八段 512 KiB（map0 的 addr+size 必须覆盖
 * [PS_FIX_CORE_PROJ_CTRL, +0x80000)）。成功返回 PS_FIX_OK 并把 w->fd 设为设备 fd；
 * 失败返回 PS_FIX_DRV_ERR_* 并把可读原因写进 err（可为 NULL/0）。
 * 中断不在此后端内：四个核按 CTRL.ap_done 轮询（fix_core.c），不用阻塞 read()。 */
int  ps_fix_window_open_uio(ps_fix_window_t *w, const char *uio_dev,
                            char *err, size_t errcap);

void ps_fix_window_close(ps_fix_window_t *w);

/* ---------------------------------------------------------------- 四核管线 */

#define PS_FIX_CORE_ID_PROJ 0   /* ps_projection_fix  0xA0000000 / 0xA0010000 */
#define PS_FIX_CORE_ID_FWD  1   /* ps_lr_forward_fix  0xA0020000 / 0xA0030000 */
#define PS_FIX_CORE_ID_ADJ  2   /* ps_lr_adjoint_fix  0xA0040000 / 0xA0050000 */
#define PS_FIX_CORE_ID_PDHG 3   /* ps_pdhg_fix        0xA0060000 / 0xA0070000 */

/* 阶段追踪回调（可选）。自检用它核对四核顺序；nullptr 则不打点。 */
typedef void (*ps_fix_trace_fn)(void *ctx, int core_id, const char *stage);

typedef struct {
    ps_fix_core_t    cores[4];   /* 顺序同 PS_FIX_CORE_ID_* */
    ps_fix_window_t *win;
    int              opened;
    int              shadow;     /* 1 = 影子窗口（等 done 用瞬时完成替身）*/
    ps_fix_trace_fn  trace;
    void            *trace_ctx;
} ps_fix_pipeline_t;

/* 打开四个核（只是把句柄绑到映射窗口上，不写任何寄存器）。 */
int ps_fix_pipeline_open(ps_fix_pipeline_t *p, ps_fix_window_t *w);

/* 四核固定顺序：proj -> lr_fwd -> lr_adj -> pdhg。返回的数组长度为 4，只读。 */
const int *ps_fix_core_order(int *n_out);

/* ---------------------------------------------------------------- 模型视图 */

/* 一条候选记录内各数组的**物理**地址（由 psm_format.h 的偏移算出，
 * 不硬编码包长）。index 为候选序号。 */
typedef struct {
    uint64_t cand_phys;    /* 记录基址 */
    uint64_t U_phys;
    uint64_t S_phys;
    uint64_t V_phys;
    uint64_t mask_phys;
    uint64_t s_max_phys;   /* 候选头里的 s_max（若当前格式提供；否则为 0） */
    int      has_s_max;    /* 1 = 格式含 s_max 字段，可用 s_max_phys 取值 */
    uint64_t record_bytes; /* 单条记录字节数（来自 ps_psm_record_bytes_hdr）*/
    uint64_t u_bytes;      /* U 数组字节数 = rank_max * meas_len * 2 */
} ps_fix_model_view_t;

/* model_phys 是模型文件在 DDR 里的物理基址（不是 0xA0.. 那一段，见 regmap §六）。
 * 失败返回负码；成功返回 PS_FIX_OK。 */
int ps_fix_model_view(const ps_psm_header_t *h, uint64_t model_phys,
                      uint32_t index, ps_fix_model_view_t *out);

/* ---------------------------------------------------------------- S 幅度折算 */
/*
 * 把候选的 s_scale（psm 候选头 +12 的 float，S 码字的步长，本仓库恒为 2^-14）与
 * s_max（候选头 +20 的 float，该候选未归一化的最大奇异值）折成定点核 **S 级**要的
 * 两个标量 s_gain_q16 / s_exp，写进 board/ps_fix_regs.h 的对应寄存器。
 *
 * 折算口径（与 src/fpga/hls/ps_fixed.hpp 的 exp_to_gain_shift 逐字对齐，不另立一套）
 * ---------------------------------------------------------------------------
 * 核在 S 级只把一个**标量乘数**乘到 S 码字上（lr_operator_fixed.cpp 阶段 2：
 * t = z*S; v = mul_shift_round(t, sg, sh_s)）。该乘数只有两种取法，**互斥**：
 *
 *   exp 有效： sg = 2^16、sh_s = kShiftZToS - exp   -> 乘数 = 2^exp        （2 的幂，精确）
 *   exp 无效： sg = s_gain_q16、sh_s = kShiftZToS   -> 乘数 = s_gain_q16/2^16（Q16.16）
 *
 * 要还原的绝对幅度乘数是 m = s_scale * s_max（S 码字 -> 绝对奇异值 σ）。
 * 与 docs/gain_representation.md 的两条纪律一致：scale_to_exp2 对非 2 的幂返回无效
 * 而**不返回近似值**；非 2 的幂落到保留的 Q16.16 路径。因此本折算：
 *   · m 是 2 的幂 -> 走 exp 支路（exact = 1，rel_err 恒 0）；
 *   · 否则        -> 走 Q16.16 支路，s_exp 置 PS_FIX_GAIN_EXP_INVALID，
 *                    相对偏差上界 = 0.5 / (m * 2^16) = 0.5 / s_gain_q16。
 *
 * 为什么 S 级的量化误差是一个**整体缩放**误差：乘数只作用在一个标量上、对所有 rank
 * 相同，故它只把整个算子按 (1+rel_err) 等比缩放，不引入逐元素噪声。
 *
 * 写入的寄存器（见表与 ps_fix_regmap.md §4.2，三个核的 S 偏移相同）：
 *   s_gain_q16 -> LRF_S_GAIN_Q16 / LRA_S_GAIN_Q16 / PDHG_S_GAIN_Q16  (均 0x038)
 *   s_exp      -> LRF_S_EXP      / LRA_S_EXP      / PDHG_S_EXP      (均 0x050)
 * U/V 两件由调用方按 u_scale/v_scale 照旧填，本折算只动 S 的两极。
 */

/* 与 ps::kGainExpInvalid 同值（ps_fixed.hpp）。用它表示「本候选走 Q16.16 支路」。 */
#define PS_FIX_GAIN_EXP_INVALID 0x7FFFFFFF

typedef struct {
    int32_t s_gain_q16;   /* 乘数 = s_gain_q16 / 2^16（s_exp 无效时由核采用） */
    int32_t s_exp;        /* 乘数 = 2^s_exp（有效时由核采用）；否则 PS_FIX_GAIN_EXP_INVALID */
    float   multiplier;   /* 实际生效的乘数：S 码字 -> 绝对奇异值的比例 */
    float   rel_err;      /* multiplier 相对 s_scale*s_max 的相对偏差 */
    int     exact;        /* 1 = 走幂次支路，rel_err 恒 0 */
} ps_fix_s_fold_t;

/* 由两个标量折算（模型幅度，不含帧量程）。s_scale/s_max 必须为正；否则返回 ARGS。 */
int ps_fix_fold_s_scale(float s_scale, float s_max, ps_fix_s_fold_t *out);

/* 通用入口：给定要乘到 S 码字上的**完整乘数** m，算 (s_gain_q16, s_exp)。
 * 生产用法 m = s_scale * s_max * (s_f / s_d)：s_scale*s_max 是模型幅度（见上一条），
 * (s_f/s_d) 是本帧为把算子压进 Q6.18 图像域而选的量程比（与 X20 的 c 同源，
 * 例：B 档 s_f=1、s_d=32）。m 必须为正；否则返回 PS_FIX_DRV_ERR_ARGS。 */
int ps_fix_fold_multiplier(double m, ps_fix_s_fold_t *out);

/* 直接吃 psm 候选记录头（psm_format.h 的布局）：从 +12 读 s_scale、+20 读 s_max 后
 * 折算。cand_base 指向该候选记录的起点（不是 64 字节文件头）。 */
int ps_fix_fold_candidate_s(const uint8_t *cand_base, ps_fix_s_fold_t *out);

/* 把折算结果写进四核共用的增益结构（只动 S 两件，U/V 保持调用方已填的值）。 */
void ps_fix_gain_apply_s(ps_fix_gain_t *g, const ps_fix_s_fold_t *f);

/* 从 psm 候选记录头把**整组**增益填好：U 用 +8 的 u_scale、V 用 +16 的 v_scale
 * （各按 2 的幂指数下发，与 scale_to_exp2 同口径），S 用 ps_fix_fold_candidate_s。
 * 只读候选头 24 字节，不碰 U/S/V 数组。
 *
 * A1 入口拒绝（不是静默回退）
 * ---------------------------------------------------------------------------
 * u_scale / v_scale 必须是**精确的 2 的幂**且指数落在 int8（[-128,127]）内；
 * 否则返回 PS_FIX_DRV_ERR_GAIN_EXP，**不**把 exp 置无效后就地走 Q16.16。
 * 原因：非 2 的幂（或过小）的 scale 折成 Q16.16 时 gain_q16 = round(scale*2^16)
 * 常为 0，核输出整段归零 —— 这是静默错数据，不是降精度。B 档用
 * quantize_symmetric_pow2 生成，天然满足；换成旧的连续 scale 模型即触发。
 * S 的乘数是 s_scale*s_max，s_max 一般不是 2 的幂，故 S 仍保留 Q16.16 支路（合法）。
 *
 * 失败返回负码；成功返回 PS_FIX_OK。 */
int ps_fix_gain_from_candidate(const uint8_t *cand_base, ps_fix_gain_t *out);

/* 只校验候选头的 u_scale/v_scale（+8/+16）是否为可精确表示的 2 的幂且指数在
 * int8 内。返回 PS_FIX_OK 或 PS_FIX_DRV_ERR_GAIN_EXP（可见 ps_fix_gain_from_candidate
 * 的「A1 入口拒绝」）。供模型装载/上板前批量校验用，不做任何折算。 */
int ps_fix_check_candidate_gain_scales(const uint8_t *cand_base);

/* ---------------------------------------------------------------- 作业描述 */

/* 定位（投影核）作业：五个 m_axi 缓冲区 + 标量。 */
typedef struct {
    uint64_t meas, u_codes, gain_q16, score_out, score_ch;
    uint32_t P, K, M, nx, ny, nz, margin_thresh_q16;
} ps_fix_proj_job_t;

/* 低秩前向作业。 */
typedef struct {
    uint64_t U, S, V, f, y, wk_z, wk_zs;
    uint32_t K, M, N;
    ps_fix_gain_t gain;
} ps_fix_lrf_job_t;

/* 低秩伴随作业。 */
typedef struct {
    uint64_t U, S, V, r, g, wk_w, wk_ws;
    uint32_t K, M, N;
    ps_fix_gain_t gain;
} ps_fix_lra_job_t;

/* PDHG 重建作业。 */
typedef struct {
    uint64_t U, S, V, d3, f3;
    ps_fix_pdhg_scratch_t scratch;
    ps_fix_pdhg_outputs_t out;
    uint32_t K, M, N;
    ps_fix_gain_t gain;
    ps_fix_pdhg_params_t params;
} ps_fix_pdhg_job_t;

/* ---------------------------------------------------------------- 帧量程接缝（meas_norm） */
/*
 * 这一节是「RTL 预处理链输出 -> 定点投影核入参」那一级的编排入口。
 * 口径权威在 src/software/meas_norm（其本身逐位对拍 quantize.shared_exponent），
 * 本层只做：
 *   1. 把 RTL 的 24 位打包拍（低 24 bit = signed 差分）转成通道优先 Q2.16 的
 *      meas[ch*M + m]（m = r*cols + c）；
 *   2. 把该帧的**共享指数**折成乘数，接到投影核的 gain 上。
 *
 * 为什么乘数要单独给：meas_norm 把每帧压成 Q2.16 时用了一个**逐帧**指数 e，
 * 码字 -> 绝对测量的步长是 d_scale = 2^(e - frac_bits)。投影核的 gain 是
 * Q16.16 的「码字 -> 分数」乘数，故总的「码字 -> 绝对测量」乘数
 *     m = u_scale * d_scale        （u_scale = 该候选的 psm 候选头 +8）
 * 就是 T2 接缝用的 gain = u_scale * d_scale * 2^16 的那个乘数。
 * 少了 d_scale，同一份码字在不同帧量程下会被当成同一个物理幅度 —— 这正是
 * meas_norm_rtl_to_fixcore_20260929.md §七 第 5 条记的未做项。
 */

/* 一帧信号调理：beats 是 DMA 进来的 RTL 输出拍（低 24 bit = signed 差分，
 * 其余位忽略），rows/cols 是**差分帧**尺寸（vertical_diff 对 WALL_N x WALL_N
 * 出 (WALL_N-1) x WALL_N，故 B 档为 95 x 96），channels 为 3。
 * meas_cf 输出通道优先块，长度须 >= rows*cols*channels。
 * frame 可为 NULL（只出码字）；非空时写回本帧指数/包络/饱和统计。
 * 返回 PS_FIX_OK / PS_FIX_DRV_ERR_ARGS。 */
int ps_fix_prepare_meas(const uint32_t *rtl24_beats, uint32_t rows, uint32_t cols,
                        uint32_t channels, int32_t *meas_cf,
                        ps_meas_norm_result_t *frame);

/* 本帧量程的 2 的幂指数：d_scale = 2^返回值 = 2^(exponent - frac_bits)。
 * 与 meas_norm 的 divisor_exp = frac_bits - exponent 互逆。 */
int ps_fix_meas_scale_exponent(const ps_meas_norm_result_t *frame);

/* 本帧量程的乘数值（= 2^exponent 的那一支）。不用 libm，板端可直接调。 */
double ps_fix_meas_d_scale(const ps_meas_norm_result_t *frame);

/* 帧量程折算结果：把 meas_norm 的本帧量程与候选的 u_scale 折成
 * 「码字 -> 绝对测量」的总乘数。幂次支路（exact=1）下 exp 有效、gain_q16 被核忽略。 */
typedef struct {
    double  multiplier;   /* = u_scale * d_scale */
    int32_t gain_q16;     /* Q16.16 支路：round_half_even(multiplier * 65536) */
    int32_t exp;          /* 幂次支路的指数；否则 PS_FIX_GAIN_EXP_INVALID */
    int     exact;        /* 1 = 2 的幂支路（rel_err 恒 0） */
    int     d_scale_exp;  /* d_scale = 2^d_scale_exp */
} ps_fix_frame_scale_t;

/* 由 meas_norm 的帧结果 + 候选 u_scale 折算。u_scale <= 0 或 frame 为空返回 ARGS。 */
int ps_fix_fold_frame_scale(const ps_meas_norm_result_t *frame, float u_scale,
                            ps_fix_frame_scale_t *out);

/* 编排：把 meas 缓冲的**物理地址**绑进投影作业，并（可选）回报帧量程折算。
 * 本函数只改 job->meas，不写任何寄存器、不启动核 —— 上板时 meas_phys 由调用方
 * 的 DDR 分配器给出；影子窗口里可传调用方缓冲的地址（须按 PS_PSM_ARRAY_ALIGN 对齐，
 * 否则 ps_fix_run_projection 会返回 PS_FIX_DRV_ERR_ALIGN）。 */
int ps_fix_proj_attach_meas(ps_fix_proj_job_t *job, uint64_t meas_phys,
                            float u_scale, const ps_meas_norm_result_t *frame,
                            ps_fix_frame_scale_t *scale_out);

/* ---------------------------------------------------------------- 单核执行 */

/* 每个 run 依次做：校验基址对齐 -> 写基址与标量 -> ap_start -> 等 done -> 读结果。
 * poll_limit 是轮询次数上限（不依赖任何 OS 计时器）。 */
int ps_fix_run_projection(ps_fix_pipeline_t *p, const ps_fix_proj_job_t *job,
                          uint32_t poll_limit, ps_fix_proj_result_t *out);
int ps_fix_run_forward(ps_fix_pipeline_t *p, const ps_fix_lrf_job_t *job,
                       uint32_t poll_limit, ps_fix_lr_result_t *out);
int ps_fix_run_adjoint(ps_fix_pipeline_t *p, const ps_fix_lra_job_t *job,
                       uint32_t poll_limit, ps_fix_lr_result_t *out);
int ps_fix_run_pdhg(ps_fix_pipeline_t *p, const ps_fix_pdhg_job_t *job,
                    uint32_t poll_limit, ps_fix_pdhg_result_t *out);

/* ---------------------------------------------------------------- 四核顺序 */

typedef struct {
    const ps_fix_proj_job_t *proj;   /* nullptr 则跳过该核 */
    const ps_fix_lrf_job_t  *fwd;
    const ps_fix_lra_job_t  *adj;
    const ps_fix_pdhg_job_t *pdhg;
} ps_fix_sequence_t;

typedef struct {
    ps_fix_proj_result_t  proj;
    ps_fix_lr_result_t    fwd;
    ps_fix_lr_result_t    adj;
    ps_fix_pdhg_result_t  pdhg;
    int stages_ok;          /* 成功执行的核数 */
    int aborted_at;         /* 首个失败核的 core_id；全部成功为 -1 */
} ps_fix_sequence_result_t;

/* 严格按 proj -> fwd -> adj -> pdhg 顺序执行；任一核失败即中止并返回其返回码。
 * 顺序由 ps_fix_core_order() 给出，本函数不重排、不并行。 */
int ps_fix_run_sequence(ps_fix_pipeline_t *p, const ps_fix_sequence_t *seq,
                        uint32_t poll_limit, ps_fix_sequence_result_t *out);

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif /* PENUMBRASIGHT_FIX_CORE_DRIVER_H */
