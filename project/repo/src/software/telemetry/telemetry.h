// ============================================================================
// telemetry.h —— 遥测记录（方案 §7 第 4 条）
//
// 方案原文：「telemetry：记录帧率、曝光、温度、功耗、候选分数、残差和迭代次数」。
//
// 为什么落成 CSV 而不是二进制/JSON
// ---------------------------------------------------------------------------
//   * 这些量是给人看、给报告贴表用的，列名必须自解释；
//   * CSV 能被 Excel / pandas 直接读，做曲线不需要再写解析器；
//   * 一次演示的时长也就几千行，体积不构成问题。
// （跨进程的**实时**通道仍走 ps_ipc.hpp 的定长消息；CSV 是「事后可复算」的那一份，
// 两者不是一回事，不要互相替代。）
//
// 列（17 列，顺序即契约；ps_telemetry_format_header 与 format_row 必须一致）
// ---------------------------------------------------------------------------
//   seq               本进程内的记录序号（从 0 起）
//   iso_time          UTC 时间戳（秒精度；真正的时序用 t_mono_us）
//   t_mono_us         单调微秒（**算延迟与帧率只认这一列**）
//   frame_rate_milli  瞬时帧率 x1000（千分之一 fps；避免浮点跨 ABI）
//   exposure_us       实际生效曝光（微秒）
//   temperature_mc    温度（毫摄氏度；避免浮点）
//   power_mw          功耗（毫瓦）
//   selected_id       定位输出 / 固定输入
//   best_score_q16    最佳候选分数（Q0.16）
//   second_score_q16  次佳候选分数（Q0.16）
//   margin_q16        (s1-s2)/(s1+eps)（Q0.16）
//   confidence_q16    全局可信度（Q0.16）
//   residual_q16      归一化残差 ρ（Q0.16）
//   iterations        迭代次数
//   rank              重建秩
//   latency_cycles    PL 处理周期
//   latency_us        PL 处理微秒（= cycles / 150 MHz）
//
// 关于「Q0.16 记成整数」这条口径
// ---------------------------------------------------------------------------
// margin / confidence / residual 都是 [0,1] 的比值。直接用 double 写 CSV 会引入
// 平台相关的位数差异（同样 0.5000 在两台机器上可能差最后一位），而报告里要拿
// 这些数做门限比较。故一律按 Q0.16 存整数，展示或计算时再 /65536
// （换算函数只有 ps_regs.h 的 ps_reg_q16_to_double 一份）。
// ============================================================================

#ifndef PENUMBRASIGHT_TELEMETRY_H
#define PENUMBRASIGHT_TELEMETRY_H

#include <stddef.h>
#include <stdint.h>

#include "ps_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------- 返回码

#define PS_TELEM_OK         0
#define PS_TELEM_ERR_OPEN   (-1)
#define PS_TELEM_ERR_ARG    (-2)
#define PS_TELEM_ERR_WRITE  (-3)

// ---------------------------------------------------------------- 单行

typedef struct {
    uint64_t seq;
    char     iso_time[24];       // "YYYY-MM-DDTHH:MM:SSZ" + NUL
    uint64_t t_mono_us;          // 单调微秒
    uint32_t frame_rate_milli;   // fps x1000
    int64_t  exposure_us;
    int32_t  temperature_mc;     // 毫摄氏度
    uint32_t power_mw;
    uint32_t selected_id;
    uint16_t best_score_q16;
    uint16_t second_score_q16;
    uint16_t margin_q16;
    uint16_t confidence_q16;
    uint16_t residual_q16;
    uint16_t iterations;
    uint16_t rank;
    uint64_t latency_cycles;
    uint64_t latency_us;
} ps_telemetry_row_t;

#define PS_TELEM_COLUMNS 17u

void ps_telemetry_row_init(ps_telemetry_row_t* r);

// 从作业结果灌入「定位/重建」那一半列（selected_id / confidence / margin /
// iterations / rank / latency）。这样 telemetry 不需要重新解释 fpga_control 的字段。
//
// 与 ps_telemetry_set_scores 的先后关系（**这是契约，不是风格**）：
//   两者都能写 margin_q16，**后调用者胜**。约定顺序是先 ps_telemetry_row_from_result
//   （PL 回读的整批字段），再用 ps_telemetry_set_scores 写候选分数与由分数算出的
//   margin。反过来调用时，set_scores 算出来的 margin 会被作业结果里的 margin 覆盖 ——
//   于是「分数与 margin 不匹配」而没有任何报错，所以顺序必须按契约来。
void ps_telemetry_row_from_result(ps_telemetry_row_t* r,
                                  const ps_reg_job_result_t* res);

// 候选分数：best/second -> margin = (s1-s2)/(s1+eps)，Q0.16。
// eps 用 1 个 Q0.16 单位（即 1/65536），避免 s1 = 0 时除零。
// 返回写入的 margin_q16。
uint16_t ps_telemetry_set_scores(ps_telemetry_row_t* r, uint16_t best_q16,
                                 uint16_t second_q16);

// 温度/功耗：这两项来自板载传感器（ina226 / sysfs 温度节点），主机上是记账。
void ps_telemetry_set_power(ps_telemetry_row_t* r, int32_t temperature_mc,
                            uint32_t power_mw);

// ISO8601 UTC 时间戳（秒精度）。取不到时间时写成 "1970-01-01T00:00:00Z"
// 并**不**静默留空 —— 空列会让下游把 CSV 读错列。
void ps_telemetry_iso_now(char* out, size_t cap);

// ---------------------------------------------------------------- 格式化

int ps_telemetry_format_header(char* out, size_t cap);          // 返回字符数
int ps_telemetry_format_row(const ps_telemetry_row_t* r,
                           char* out, size_t cap);               // 返回字符数
unsigned ps_telemetry_row_columns(const char* csv_line);        // 数逗号 -> 列数

// ---------------------------------------------------------------- 写出

typedef struct {
    void*    f;              // FILE*（不暴露 stdio，保持头文件干净）
    char     path[192];
    uint64_t rows;
    int      header_written;
    uint64_t first_mono_us;
    uint64_t last_mono_us;
    int      have_first;
    // 汇总用的运行累加量（ps_telemetry_summary 读它们，不必回读文件）
    uint64_t latency_sum_us;
    uint64_t latency_max_us;
    uint16_t confidence_min_q16;
    uint16_t residual_max_q16;
    int      have_confidence;
} ps_telemetry_t;

// 打开 CSV 并写表头。路径必须是纯 ASCII（窄字符 fopen 的现实约束见 model_loader.c）。
int ps_telemetry_open(ps_telemetry_t* t, const char* csv_path);

// 写一行。seq 为 0 且 t_mono_us 为 0 时由本函数补上（单调时钟）——
// 这样调用方不必每行都自己填时间。
int ps_telemetry_write(ps_telemetry_t* t, const ps_telemetry_row_t* r);

void ps_telemetry_close(ps_telemetry_t* t);

// ---------------------------------------------------------------- 汇总

typedef struct {
    uint64_t rows;
    double   fps_avg;              // rows / 覆盖时长
    uint64_t span_us;
    double   latency_us_avg;
    uint64_t latency_us_max;
    uint16_t confidence_min_q16;
    uint16_t residual_max_q16;
} ps_telemetry_summary_t;

// 由已写出的行算汇总。需要 ps_telemetry_write 里顺手累加的统计量。
int ps_telemetry_summary(const ps_telemetry_t* t, ps_telemetry_summary_t* out);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // PENUMBRASIGHT_TELEMETRY_H
