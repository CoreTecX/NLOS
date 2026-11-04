// ============================================================================
// ui_server.c —— 指令解析 + 状态机 + 文本版四区画面（纯 C）
//
// 本文件的定位很明确：**它是「按键 / 网页 / 串口」三种输入源的公共下游**。
// 三种输入源最终都是把一行字符串喂给 ps_ui_exec()，因此这里只认字符串，
// 不认任何 UI 框架。板上接 compositor 的 OSD / 网页前端时，改的只是「谁调用
// ps_ui_exec」，这个文件一行不动。
//
// 三个刻意的设计选择
// ---------------------------------------------------------------------------
//   1. 参数一律走 ps_regs.h 的允许集合（frames/iters/rank），非法值当场拒绝并
//      计入 rejected，而**不是**宽容地取最近的合法值。宽容处理会让「设了 40
//      结果跑的是 32」这种事一直没人发现。
//   2. 每次成功修改自增 rev。上层据此决定「要不要重写寄存器」，避免每帧都把
//      全部寄存器重写一遍。
//   3. 渲染时 metrics 传 NULL 就显示 "--"，不显示 0。0 和「未知」在画面上
//      长得一样，而它们的含义完全不同（0 帧率 vs 还没测）。
// ============================================================================

#include "ui_server.h"

// 隐藏图尺寸（18x24）从冻结的 psm_format.h 取，不在 UI 里抄一份常量：
// 画面上写死的数字一旦与模型格式漂移，演示时没人会发现。
#include "psm_format.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

// ---------------------------------------------------------------- 小工具

typedef struct {
    char*  buf;
    size_t cap;
    size_t used;
} ps_ui_sb_t;

static void ps_ui_sb_init(ps_ui_sb_t* sb, char* buf, size_t cap) {
    sb->buf = buf;
    sb->cap = cap;
    sb->used = 0u;
    if (buf && cap > 0u) buf[0] = '\0';
}

static void ps_ui_sb_fmt(ps_ui_sb_t* sb, const char* fmt, ...) {
    if (!sb->buf || sb->cap == 0u || !fmt) return;
    if (sb->used + 1u >= sb->cap) return;
    va_list ap;
    va_start(ap, fmt);
    const int n = vsnprintf(sb->buf + sb->used, sb->cap - sb->used, fmt, ap);
    va_end(ap);
    if (n > 0) {
        const size_t adv = (size_t)n;
        sb->used = (sb->used + adv < sb->cap) ? (sb->used + adv) : (sb->cap - 1u);
    }
}

// 只接受可打印 ASCII 的小写化比较：命令来自按键/网页，编码不确定，
// 故先老老实实按 ASCII 折叠大小写。
static void ps_ui_lower_ascii(char* s) {
    if (!s) return;
    for (; *s != '\0'; ++s) {
        if (*s >= 'A' && *s <= 'Z') *s = (char)(*s - 'A' + 'a');
    }
}

static int ps_ui_eq(const char* a, const char* b) {
    if (!a || !b) return 0;
    while (*a != '\0' && *b != '\0') {
        if (*a != *b) return 0;
        ++a; ++b;
    }
    return (*a == '\0' && *b == '\0') ? 1 : 0;
}

// 严格十进制解析：拒绝空串、尾随垃圾、负数。宽松解析会把 "4x" 读成 4。
static int ps_ui_parse_u32(const char* s, uint32_t* out) {
    if (!s || *s == '\0' || !out) return 0;
    uint64_t v = 0u;
    const char* p = s;
    for (; *p != '\0'; ++p) {
        if (*p < '0' || *p > '9') return 0;
        v = v * 10u + (uint64_t)(*p - '0');
        if (v > 0xFFFFFFFFull) return 0;
    }
    *out = (uint32_t)v;
    return 1;
}

// 切词：空白与 '=' 都是分隔符；'#' 起为注释；最多 4 个词，每个词 <= 31 字符。
// 超长的词直接截断（命令是有限集，截断不会造成误命中，因为后面还有允许集合检查）。
static unsigned ps_ui_tokenize(const char* line, char toks[4][32]) {
    unsigned n = 0u;
    const char* p = line;
    while (*p != '\0' && n < 4u) {
        while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n' || *p == '=') ++p;
        if (*p == '\0' || *p == '#') break;
        size_t k = 0u;
        while (*p != '\0' && *p != ' ' && *p != '\t' && *p != '\r' &&
               *p != '\n' && *p != '=' && *p != '#') {
            if (k + 1u < 32u) toks[n][k++] = *p;
            ++p;
        }
        toks[n][k] = '\0';
        ps_ui_lower_ascii(toks[n]);
        if (k > 0u) ++n;
    }
    return n;
}

static int ps_ui_reply(char* out, size_t cap, const char* fmt, ...) {
    if (!out || cap == 0u) return 0;
    va_list ap;
    va_start(ap, fmt);
    (void)vsnprintf(out, cap, fmt, ap);
    va_end(ap);
    return 0;
}

// ---------------------------------------------------------------- 名字/初始化

const char* ps_ui_page_name(uint32_t page) {
    return (page == PS_UI_PAGE_PERF) ? "perf" : "main";
}
const char* ps_ui_loc_name(uint32_t selfloc) {
    return selfloc ? "selfloc" : "fixed";
}
const char* ps_ui_color_name(uint32_t rgb) {
    return rgb ? "rgb" : "mono";
}

void ps_ui_state_init(ps_ui_state_t* s) {
    if (!s) return;
    memset(s, 0, sizeof(*s));
    // 复位默认值 = §8.2 表格里的默认档（自定位 / RGB / 4 帧 / 16 迭代 / rank 64）
    s->selfloc = 1u;
    s->rgb = 1u;
    s->frames = PS_REG_DEFAULT_FRAME_COUNT;
    s->iters = PS_REG_DEFAULT_ITER_COUNT;
    s->rank = PS_REG_DEFAULT_RANK;
    s->candidate_count = PS_REG_DEFAULT_CANDIDATE_COUNT;
    s->fixed_candidate_id = 0;
    s->page = PS_UI_PAGE_MAIN;
    s->rev = 0u;
    s->rejected = 0u;
}

void ps_ui_metrics_init(ps_ui_metrics_t* m) {
    if (!m) return;
    memset(m, 0, sizeof(*m));
}

const char* ps_ui_help(void) {
    return
        "loc self|fixed    切换自定位 / 固定位置（等价：selfloc / fixed）\n"
        "color mono|rgb    灰度 / RGB 三通道\n"
        "frames 1|4|8      帧累积数（§8.2 FRAME_COUNT）\n"
        "iters 8|12|16|32  迭代次数（§8.2 ITER_COUNT）\n"
        "rank 32|48|64|96  重建秩（§8.2 RANK）\n"
        "cand N            候选数（1..243，默认 243）\n"
        "fixed-id N        固定位置模式的候选编号\n"
        "page main|perf    主画面 / 性能页\n"
        "status            打印当前状态与四区画面\n"
        "reset             回到 §8.2 默认档\n"
        "help              本表\n"
        "quit              退出\n";
}

// ---------------------------------------------------------------- 状态 -> 配置

void ps_ui_to_job_config(const ps_ui_state_t* s, ps_reg_job_config_t* cfg) {
    if (!cfg) return;
    memset(cfg, 0, sizeof(*cfg));
    if (!s) return;
    cfg->frame_count = s->frames;
    cfg->iter_count = s->iters;
    cfg->rank = s->rank;
    cfg->candidate_count = s->candidate_count;
    cfg->selfloc = s->selfloc;
    cfg->rgb = s->rgb;
    cfg->fixed_candidate_id = s->fixed_candidate_id;
    cfg->flags = 0u;
}

void ps_ui_from_job_config(ps_ui_state_t* s, const ps_reg_job_config_t* cfg) {
    if (!s || !cfg) return;
    s->frames = cfg->frame_count;
    s->iters = cfg->iter_count;
    s->rank = cfg->rank;
    if (cfg->candidate_count != 0u) s->candidate_count = cfg->candidate_count;
    s->selfloc = cfg->selfloc ? 1u : 0u;
    s->rgb = cfg->rgb ? 1u : 0u;
    s->fixed_candidate_id = cfg->fixed_candidate_id;
    s->rev++;
}

// ---------------------------------------------------------------- 渲染

static void ps_ui_render_perf(ps_ui_sb_t* sb, const ps_ui_state_t* s,
                              const ps_ui_metrics_t* m) {
    ps_ui_sb_fmt(sb, "[性能页]（右下铺开；§7：帧率/采集延迟/PL 延迟/秩/位宽/迭代/资源/功耗）\n");
    if (!m) {
        ps_ui_sb_fmt(sb, "  帧率     -- \n  采集延迟 --\n  PL 延迟  --\n"
                         "  秩/位宽  %u / --\n  迭代     %u\n  功耗     --\n",
                     (unsigned)s->rank, (unsigned)s->iters);
        return;
    }
    ps_ui_sb_fmt(sb, "  帧率         %.1f fps\n", m->fps);
    ps_ui_sb_fmt(sb, "  曝光         %lld us\n", (long long)m->exposure_us);
    ps_ui_sb_fmt(sb, "  采集延迟     %llu us\n",
                 (unsigned long long)m->capture_latency_us);
    ps_ui_sb_fmt(sb, "  PL 计算延迟  %llu us\n",
                 (unsigned long long)m->pl_latency_us);
    ps_ui_sb_fmt(sb, "  秩 / 位宽    %u / %u bit\n", (unsigned)m->rank,
                 (unsigned)m->bits);
    ps_ui_sb_fmt(sb, "  迭代         %u\n", (unsigned)m->iterations);
    ps_ui_sb_fmt(sb, "  功耗         %u mW\n", (unsigned)m->power_mw);
    ps_ui_sb_fmt(sb, "  已处理帧数   %llu\n",
                 (unsigned long long)m->frames_processed);
    if (m->pl_latency_us > 0u && m->capture_latency_us > 0u) {
        // PL 占比：方案 §7 的取向是「强化 FPGA 核心性」，这个比例是它的量化口径。
        const double pl = (double)m->pl_latency_us;
        const double cap = (double)m->capture_latency_us;
        ps_ui_sb_fmt(sb, "  PL/采集 延迟比 %.2f\n", pl / cap);
    }
}

static void ps_ui_render_main(ps_ui_sb_t* sb, const ps_ui_state_t* s,
                              const ps_ui_metrics_t* m) {
    ps_ui_sb_fmt(sb,
        "+--------------------------+---------------------------+\n"
        "| [左上] 相机墙面 ROI      | [右上] 隐藏图像重建       |\n");
    if (m) {
        ps_ui_sb_fmt(sb,
            "|  RAW Bayer -> %ux...      |  %ux%u -> 显示尺寸        |\n",
            (unsigned)m->bits, (unsigned)PS_PSM_HIDDEN_H,
            (unsigned)PS_PSM_HIDDEN_W);
    } else {
        ps_ui_sb_fmt(sb, "|  RAW Bayer（位宽 --）    |  %ux%u -> 显示尺寸        |\n",
                     (unsigned)PS_PSM_HIDDEN_H, (unsigned)PS_PSM_HIDDEN_W);
    }
    ps_ui_sb_fmt(sb,
        "+--------------------------+---------------------------+\n"
        "| [左下] 定位网格与可信度  | [右下] 性能摘要           |\n");
    if (m) {
        ps_ui_sb_fmt(sb, "|  selected_id = %-4u/%-4u |  帧率 %.1f fps            |\n",
                     (unsigned)m->selected_id, (unsigned)m->candidate_count,
                     m->fps);
        ps_ui_sb_fmt(sb, "|  最佳/次佳 = %.3f/%.3f  |  采集 %llu us / PL %llu us |\n",
                     ps_reg_q16_to_double(m->best_score_q16),
                     ps_reg_q16_to_double(m->second_score_q16),
                     (unsigned long long)m->capture_latency_us,
                     (unsigned long long)m->pl_latency_us);
        ps_ui_sb_fmt(sb, "|  可信度 = %.4f           |  rank %-3u 位宽 %-2u 迭代 %-3u |\n",
                     ps_reg_q16_to_double(m->confidence_q16),
                     (unsigned)m->rank, (unsigned)m->bits,
                     (unsigned)m->iterations);
        ps_ui_sb_fmt(sb, "|  模式 = %-8s        |  功耗 %u mW              |\n",
                     ps_ui_loc_name(s->selfloc), (unsigned)m->power_mw);
    } else {
        ps_ui_sb_fmt(sb, "|  selected_id = --/%-4u   |  帧率 --                  |\n",
                     (unsigned)s->candidate_count);
        ps_ui_sb_fmt(sb, "|  最佳/次佳 = --/--       |  采集 -- / PL --          |\n");
        ps_ui_sb_fmt(sb, "|  可信度 = --             |  rank %-3u 位宽 -- 迭代 %-3u |\n",
                     (unsigned)s->rank, (unsigned)s->iters);
        ps_ui_sb_fmt(sb, "|  模式 = %-8s        |  功耗 -- mW              |\n",
                     ps_ui_loc_name(s->selfloc));
    }
    ps_ui_sb_fmt(sb, "+--------------------------+---------------------------+\n");
}

int ps_ui_render(const ps_ui_state_t* s, const ps_ui_metrics_t* m,
                 char* out, size_t cap) {
    if (!s || !out || cap == 0u) return -1;
    ps_ui_sb_t sb;
    ps_ui_sb_init(&sb, out, cap);

    ps_ui_sb_fmt(&sb, "=== PenumbraSight UI ===\n");
    ps_ui_sb_fmt(&sb, "模式=%s 颜色=%s 帧数=%u 迭代=%u 秩=%u 候选=%u 固定编号=%d 页面=%s rev=%u\n",
                 ps_ui_loc_name(s->selfloc), ps_ui_color_name(s->rgb),
                 (unsigned)s->frames, (unsigned)s->iters, (unsigned)s->rank,
                 (unsigned)s->candidate_count, (int)s->fixed_candidate_id,
                 ps_ui_page_name(s->page), (unsigned)s->rev);

    if (s->page == PS_UI_PAGE_PERF) {
        // 性能页把右下铺开，其余三区保留标题（方案 §7：显示画面固定四区，
        // 切页只是把右下放大，不是换一套画面）。
        ps_ui_sb_fmt(&sb, "[左上] 相机墙面 ROI    [右上] 隐藏图像重建    [左下] 定位网格\n");
        ps_ui_render_perf(&sb, s, m);
    } else {
        ps_ui_render_main(&sb, s, m);
    }
    return (int)sb.used;
}

// ---------------------------------------------------------------- 指令执行

static int ps_ui_reject(ps_ui_state_t* s, char* out, size_t cap, const char* why) {
    s->rejected++;
    ps_ui_reply(out, cap, "拒绝：%s（输入 help 看指令表）", why);
    return PS_UI_ERR_BAD_ARG;
}

int ps_ui_exec(ps_ui_state_t* s, const char* line, char* out, size_t cap) {
    if (!s || !line) return PS_UI_ERR_ARG;
    if (out && cap > 0u) out[0] = '\0';

    char toks[4][32];
    const unsigned n = ps_ui_tokenize(line, toks);
    if (n == 0u) return PS_UI_OK;    // 空行 / 纯注释

    // ---- 无参命令 ----
    if (ps_ui_eq(toks[0], "quit") || ps_ui_eq(toks[0], "exit")) {
        return PS_UI_QUIT;
    }
    if (ps_ui_eq(toks[0], "help") || ps_ui_eq(toks[0], "?")) {
        ps_ui_reply(out, cap, "%s", ps_ui_help());
        return PS_UI_NOTE;
    }
    if (ps_ui_eq(toks[0], "status")) {
        char buf[1024];
        if (ps_ui_render(s, NULL, buf, sizeof(buf)) < 0) return PS_UI_ERR_ARG;
        ps_ui_reply(out, cap, "%s", buf);
        return PS_UI_NOTE;
    }
    if (ps_ui_eq(toks[0], "reset")) {
        const uint32_t keep_rev = s->rev + 1u;
        const uint32_t keep_rej = s->rejected;
        ps_ui_state_init(s);
        s->rev = keep_rev;
        s->rejected = keep_rej;
        ps_ui_reply(out, cap, "已复位到 §8.2 默认档：4 帧 / 16 迭代 / rank 64 / 自定位 / RGB");
        return PS_UI_OK;
    }

    // ---- 位置模式 ----
    if (ps_ui_eq(toks[0], "selfloc") || ps_ui_eq(toks[0], "fixed") ||
        ps_ui_eq(toks[0], "loc")) {
        const char* arg = ps_ui_eq(toks[0], "loc") ? (n > 1u ? toks[1] : "") : toks[0];
        if (ps_ui_eq(arg, "self") || ps_ui_eq(arg, "selfloc") || ps_ui_eq(arg, "auto")) {
            s->selfloc = 1u;
            s->rev++;
            ps_ui_reply(out, cap, "模式 = 自定位（CONTROL.selfloc = 1）");
            return PS_UI_OK;
        }
        if (ps_ui_eq(arg, "fixed") || ps_ui_eq(arg, "fix")) {
            s->selfloc = 0u;
            s->rev++;
            ps_ui_reply(out, cap, "模式 = 固定位置（CONTROL.selfloc = 0，使用 fixed-id）");
            return PS_UI_OK;
        }
        return ps_ui_reject(s, out, cap, "loc 的参数只能是 self 或 fixed");
    }

    // ---- 颜色 ----
    if (ps_ui_eq(toks[0], "color") || ps_ui_eq(toks[0], "colour")) {
        const char* arg = (n > 1u) ? toks[1] : "";
        if (ps_ui_eq(arg, "mono") || ps_ui_eq(arg, "gray") || ps_ui_eq(arg, "grey")) {
            s->rgb = 0u;
            s->rev++;
            ps_ui_reply(out, cap, "颜色 = 灰度（CONTROL.rgb = 0）");
            return PS_UI_OK;
        }
        if (ps_ui_eq(arg, "rgb") || ps_ui_eq(arg, "color")) {
            s->rgb = 1u;
            s->rev++;
            ps_ui_reply(out, cap, "颜色 = RGB（CONTROL.rgb = 1）");
            return PS_UI_OK;
        }
        return ps_ui_reject(s, out, cap, "color 的参数只能是 mono 或 rgb");
    }

    // ---- 帧数 ----
    if (ps_ui_eq(toks[0], "frames") || ps_ui_eq(toks[0], "frame")) {
        uint32_t v = 0u;
        if (n < 2u || !ps_ui_parse_u32(toks[1], &v)) {
            return ps_ui_reject(s, out, cap, "frames 需要数字参数");
        }
        if (!ps_reg_frame_count_valid(v)) {
            return ps_ui_reject(s, out, cap, "frames 只允许 1 / 4 / 8（§8.2）");
        }
        s->frames = v;
        s->rev++;
        ps_ui_reply(out, cap, "帧累积数 = %u（FRAME_COUNT）", (unsigned)v);
        return PS_UI_OK;
    }

    // ---- 迭代 ----
    if (ps_ui_eq(toks[0], "iters") || ps_ui_eq(toks[0], "iter")) {
        uint32_t v = 0u;
        if (n < 2u || !ps_ui_parse_u32(toks[1], &v)) {
            return ps_ui_reject(s, out, cap, "iters 需要数字参数");
        }
        if (!ps_reg_iter_count_valid(v)) {
            return ps_ui_reject(s, out, cap, "iters 只允许 8 / 12 / 16 / 32（§8.2）");
        }
        s->iters = v;
        s->rev++;
        ps_ui_reply(out, cap, "迭代次数 = %u（ITER_COUNT）", (unsigned)v);
        return PS_UI_OK;
    }

    // ---- 秩 ----
    if (ps_ui_eq(toks[0], "rank")) {
        uint32_t v = 0u;
        if (n < 2u || !ps_ui_parse_u32(toks[1], &v)) {
            return ps_ui_reject(s, out, cap, "rank 需要数字参数");
        }
        if (!ps_reg_rank_valid(v)) {
            return ps_ui_reject(s, out, cap, "rank 只允许 32 / 48 / 64 / 96（§8.2）");
        }
        s->rank = v;
        s->rev++;
        ps_ui_reply(out, cap, "重建秩 = %u（RANK）", (unsigned)v);
        return PS_UI_OK;
    }

    // ---- 候选数 ----
    if (ps_ui_eq(toks[0], "cand") || ps_ui_eq(toks[0], "candidates")) {
        uint32_t v = 0u;
        if (n < 2u || !ps_ui_parse_u32(toks[1], &v)) {
            return ps_ui_reject(s, out, cap, "cand 需要数字参数");
        }
        if (v == 0u || v > PS_REG_DEFAULT_CANDIDATE_COUNT) {
            return ps_ui_reject(s, out, cap,
                                "cand 必须在 1..243（B 档网格 9x9x3 = 243）");
        }
        s->candidate_count = v;
        s->rev++;
        ps_ui_reply(out, cap, "候选数 = %u（CANDIDATE_COUNT）", (unsigned)v);
        return PS_UI_OK;
    }

    // ---- 固定编号 ----
    if (ps_ui_eq(toks[0], "fixed-id") || ps_ui_eq(toks[0], "fixedid")) {
        uint32_t v = 0u;
        if (n < 2u || !ps_ui_parse_u32(toks[1], &v)) {
            return ps_ui_reject(s, out, cap, "fixed-id 需要数字参数");
        }
        if (v >= s->candidate_count) {
            return ps_ui_reject(s, out, cap, "fixed-id 必须 < 候选数（否则定位输出越界）");
        }
        s->fixed_candidate_id = (int32_t)v;
        s->rev++;
        ps_ui_reply(out, cap, "固定编号 = %u（SELECTED_ID 输入）", (unsigned)v);
        return PS_UI_OK;
    }

    // ---- 页面 ----
    if (ps_ui_eq(toks[0], "page")) {
        const char* arg = (n > 1u) ? toks[1] : "";
        if (ps_ui_eq(arg, "main")) {
            s->page = PS_UI_PAGE_MAIN;
            s->rev++;
            ps_ui_reply(out, cap, "页面 = 主画面（四区）");
            return PS_UI_OK;
        }
        if (ps_ui_eq(arg, "perf") || ps_ui_eq(arg, "performance")) {
            s->page = PS_UI_PAGE_PERF;
            s->rev++;
            ps_ui_reply(out, cap, "页面 = 性能页");
            return PS_UI_OK;
        }
        return ps_ui_reject(s, out, cap, "page 的参数只能是 main 或 perf");
    }

    s->rejected++;
    ps_ui_reply(out, cap, "拒绝：未知指令「%s」（输入 help 看指令表）", toks[0]);
    return PS_UI_ERR_UNKNOWN_CMD;
}
