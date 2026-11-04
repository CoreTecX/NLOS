// ============================================================================
// telemetry.c —— 遥测 CSV 的写出与汇总（纯 C）
//
// 分工：
//   * ps_telemetry_format_header / format_row：把一行渲染成 CSV 文本，
//     不碰 IO。主机自检靠这两条做**确定性**比对（同一行渲染两次必须逐字节相同，
//     列数必须等于 PS_TELEM_COLUMNS），否则 CSV 一旦列错位，读表的人只会看到
//     「数字都有，但温度那列是功耗」这种不报错的错。
//   * ps_telemetry_open/write/close：落盘 + 顺手累加汇总量。
//
// 时间：真正用来算帧率与延迟的是**单调微秒**（t_mono_us）。ISO 时间戳只是给人看的
// （墙钟可能被 NTP 调整，不能拿来算差值）。这两者的分工在 ps_platform.hpp 里也有
// 同样的说明，不要混用。
//
// 板端记录来源
// ---------------------------------------------------------------------------
//   帧率 / 曝光    <- camera_service（帧号与时间戳，ps_ipc.hpp 的 PsFrameMeta）
//   候选分数 / 残差 / 迭代 <- fpga_control 的 JobResult + PL 回读
//   温度 / 功耗     <- 板载传感器（AXU9EGB 上常见是 sysfs 的温度节点与 INA 系列
//                     功耗监测芯片；读法见本文件 TODO）。主机上没有传感器，
//                     故由调用方传入（自检里用固定值，绝不编造「实测」数值）。
// ============================================================================

#include "telemetry.h"

#include <stdio.h>
#include <string.h>
#include <time.h>

// ---------------------------------------------------------------- 平台小件

static int ps_telem_path_is_ascii(const char* s) {
    if (!s) return 0;
    for (; *s != '\0'; ++s) {
        const unsigned char c = (unsigned char)*s;
        if (c < 0x20u || c > 0x7Eu) return 0;
    }
    return 1;
}

// ---------------------------------------------------------------- 行初始化

void ps_telemetry_row_init(ps_telemetry_row_t* r) {
    if (!r) return;
    memset(r, 0, sizeof(*r));
    ps_telemetry_iso_now(r->iso_time, sizeof(r->iso_time));
}

void ps_telemetry_row_from_result(ps_telemetry_row_t* r,
                                  const ps_reg_job_result_t* res) {
    if (!r || !res) return;
    r->selected_id = res->selected_id;
    r->confidence_q16 = res->confidence_q16;
    r->margin_q16 = res->margin_q16;
    r->iterations = res->iterations;
    r->rank = res->rank;
    r->latency_cycles = res->latency_cycles;
    r->latency_us = res->latency_us ? res->latency_us
                                    : ps_reg_cycles_to_us(res->latency_cycles);
}

uint16_t ps_telemetry_set_scores(ps_telemetry_row_t* r, uint16_t best_q16,
                                 uint16_t second_q16) {
    if (!r) return 0u;
    r->best_score_q16 = best_q16;
    r->second_score_q16 = second_q16;
    // margin = (s1 - s2) / (s1 + eps)。eps 取 1 个 Q0.16 单位，避免 s1 = 0 除零；
    // 若 s2 > s1（次佳反而更高，说明上游把顺序搞反了），margin 记 0 而不是负数：
    // 负 margin 在 Q0.16 里会回绕成一个巨大的正数，是本项目刻意防的一类错。
    const uint32_t s1 = (uint32_t)best_q16;
    const uint32_t s2 = (uint32_t)second_q16;
    if (s2 >= s1) {
        r->margin_q16 = 0u;
        return 0u;
    }
    const uint64_t num = (uint64_t)(s1 - s2) * (uint64_t)PS_REG_Q16_ONE;
    const uint64_t den = (uint64_t)s1 + 1ull;   // eps = 1/65536
    uint64_t q = num / den;
    if (q > 0xFFFFull) q = 0xFFFFull;
    r->margin_q16 = (uint16_t)q;
    return r->margin_q16;
}

void ps_telemetry_set_power(ps_telemetry_row_t* r, int32_t temperature_mc,
                            uint32_t power_mw) {
    if (!r) return;
    r->temperature_mc = temperature_mc;
    r->power_mw = power_mw;
    // TODO(板端)：把这两个值改成从传感器读回来的真实值 ——
    //   温度：/sys/class/thermal/thermal_zone*/temp（单位毫摄氏度）
    //   功耗：INA260/INA226 的 sysfs（或 hwmon）读数 = 母线电压 x 电流
    // 现在这两个参数由调用方传入，是为了**不在主机上编造实测值**。
}

void ps_telemetry_iso_now(char* out, size_t cap) {
    if (!out || cap < 21u) {
        if (out && cap > 0u) out[0] = '\0';
        return;
    }
    const time_t now = time(NULL);
    struct tm tmv;
    memset(&tmv, 0, sizeof(tmv));
    int ok = 0;
#if defined(_WIN32)
    ok = (gmtime_s(&tmv, &now) == 0) ? 1 : 0;
#else
    ok = (gmtime_r(&now, &tmv) != NULL) ? 1 : 0;
#endif
    if (!ok || tmv.tm_year <= 0) {
        // 取不到时间也不留空列：空列会让下游把 CSV 读错列
        memcpy(out, "1970-01-01T00:00:00Z", 21u);
        return;
    }
    if (strftime(out, cap, "%Y-%m-%dT%H:%M:%SZ", &tmv) == 0u) {
        memcpy(out, "1970-01-01T00:00:00Z", 21u);
    }
}

// ---------------------------------------------------------------- 格式化

int ps_telemetry_format_header(char* out, size_t cap) {
    if (!out || cap == 0u) return -1;
    static const char* kHeader =
        "seq,iso_time,t_mono_us,frame_rate_milli,exposure_us,temperature_mc,"
        "power_mw,selected_id,best_score_q16,second_score_q16,margin_q16,"
        "confidence_q16,residual_q16,iterations,rank,latency_cycles,latency_us";
    const size_t n = strlen(kHeader);
    if (n + 1u > cap) {
        out[0] = '\0';
        return -1;
    }
    memcpy(out, kHeader, n + 1u);
    return (int)n;
}

int ps_telemetry_format_row(const ps_telemetry_row_t* r, char* out, size_t cap) {
    if (!out || cap == 0u) return -1;
    out[0] = '\0';
    if (!r) return -1;
    const int n = snprintf(
        out, cap,
        "%llu,%s,%llu,%u,%lld,%ld,%u,%u,%u,%u,%u,%u,%u,%u,%u,%llu,%llu",
        (unsigned long long)r->seq, r->iso_time,
        (unsigned long long)r->t_mono_us, (unsigned)r->frame_rate_milli,
        (long long)r->exposure_us, (long)r->temperature_mc,
        (unsigned)r->power_mw, (unsigned)r->selected_id,
        (unsigned)r->best_score_q16, (unsigned)r->second_score_q16,
        (unsigned)r->margin_q16, (unsigned)r->confidence_q16,
        (unsigned)r->residual_q16, (unsigned)r->iterations, (unsigned)r->rank,
        (unsigned long long)r->latency_cycles,
        (unsigned long long)r->latency_us);
    if (n < 0) {
        out[0] = '\0';
        return -1;
    }
    return (n < (int)cap) ? n : -1;   // 截断必须报错，不许写出半行 CSV
}

unsigned ps_telemetry_row_columns(const char* csv_line) {
    if (!csv_line) return 0u;
    unsigned cols = 1u;   // 无逗号 = 1 列（空串也算 1 列，由调用方判断）
    const char* p = csv_line;
    for (; *p != '\0'; ++p) {
        if (*p == ',') ++cols;
    }
    return cols;
}

// ---------------------------------------------------------------- 落盘

int ps_telemetry_open(ps_telemetry_t* t, const char* csv_path) {
    if (!t || !csv_path) return PS_TELEM_ERR_ARG;
    memset(t, 0, sizeof(*t));
    if (!ps_telem_path_is_ascii(csv_path)) {
        fprintf(stderr,
                "[telemetry] 拒绝非 ASCII 路径（窄字符 fopen 会乱码）：%s\n",
                csv_path);
        return PS_TELEM_ERR_OPEN;
    }
    {
        const size_t n = strlen(csv_path);
        const size_t k = (n < sizeof(t->path) - 1u) ? n : (sizeof(t->path) - 1u);
        memcpy(t->path, csv_path, k);
        t->path[k] = '\0';
    }
    FILE* f = fopen(csv_path, "wb");
    if (!f) {
        fprintf(stderr, "[telemetry] 打不开 CSV 输出 %s\n", csv_path);
        return PS_TELEM_ERR_OPEN;
    }
    t->f = (void*)f;
    t->confidence_min_q16 = 0xFFFFu;

    char hdr[256];
    if (ps_telemetry_format_header(hdr, sizeof(hdr)) < 0) {
        fclose(f);
        t->f = NULL;
        return PS_TELEM_ERR_WRITE;
    }
    if (fprintf(f, "%s\n", hdr) < 0) {
        fclose(f);
        t->f = NULL;
        return PS_TELEM_ERR_WRITE;
    }
    t->header_written = 1;
    return PS_TELEM_OK;
}

int ps_telemetry_write(ps_telemetry_t* t, const ps_telemetry_row_t* r) {
    if (!t || !r) return PS_TELEM_ERR_ARG;
    if (!t->f) return PS_TELEM_ERR_ARG;

    ps_telemetry_row_t row = *r;
    // seq 未填则用本文件的计数补齐；时间戳同理。这样调用方不必每行都自己填。
    if (row.seq == 0u && t->rows > 0u) row.seq = t->rows;
    if (row.t_mono_us == 0u) {
        row.t_mono_us = (uint64_t)time(NULL) * 1000000ull;   // 秒粒度占位
    }
    if (row.iso_time[0] == '\0') {
        ps_telemetry_iso_now(row.iso_time, sizeof(row.iso_time));
    }

    char line[512];
    if (ps_telemetry_format_row(&row, line, sizeof(line)) < 0) {
        return PS_TELEM_ERR_WRITE;
    }
    if (fprintf((FILE*)t->f, "%s\n", line) < 0) return PS_TELEM_ERR_WRITE;

    // 顺手累加汇总量：事后算平均延迟/最小可信度就不必回读文件。
    t->latency_sum_us += row.latency_us;
    if (row.latency_us > t->latency_max_us) t->latency_max_us = row.latency_us;
    if (!t->have_confidence || row.confidence_q16 < t->confidence_min_q16) {
        t->confidence_min_q16 = row.confidence_q16;
        t->have_confidence = 1;
    }
    if (row.residual_q16 > t->residual_max_q16) t->residual_max_q16 = row.residual_q16;

    if (!t->have_first || t->first_mono_us > row.t_mono_us) t->first_mono_us = row.t_mono_us;
    if (t->last_mono_us < row.t_mono_us) t->last_mono_us = row.t_mono_us;
    t->have_first = 1;

    t->rows++;
    return PS_TELEM_OK;
}

void ps_telemetry_close(ps_telemetry_t* t) {
    if (!t) return;
    if (t->f) {
        fclose((FILE*)t->f);
        t->f = NULL;
    }
}

// ---------------------------------------------------------------- 汇总

int ps_telemetry_summary(const ps_telemetry_t* t, ps_telemetry_summary_t* out) {
    if (!t || !out) return PS_TELEM_ERR_ARG;
    memset(out, 0, sizeof(*out));
    out->rows = t->rows;
    out->span_us = (t->last_mono_us > t->first_mono_us)
                       ? (t->last_mono_us - t->first_mono_us)
                       : 0u;
    // 帧率口径：rows / span。span == 0（只有一行或时间戳相同）时给 0，
    // **不要**给一个除零得出的 inf —— inf 写进报告会显得像致命错误。
    out->fps_avg = (out->span_us > 0u)
                       ? ((double)t->rows * 1.0e6 / (double)out->span_us)
                       : 0.0;
    out->latency_us_avg = (t->rows > 0u)
                              ? ((double)t->latency_sum_us / (double)t->rows)
                              : 0.0;
    out->latency_us_max = t->latency_max_us;
    out->confidence_min_q16 = t->have_confidence ? t->confidence_min_q16 : 0u;
    out->residual_max_q16 = t->residual_max_q16;
    return PS_TELEM_OK;
}
