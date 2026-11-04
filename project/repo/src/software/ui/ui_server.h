// ============================================================================
// ui_server.h —— 显示/交互（方案 §7 第 5 条）
//
// 方案原文：「ui_server：按键/网页切换固定位置、自定位、灰度/RGB、1/4/8 帧和性能页」。
//
// 主机上没有 GUI，也不该为了「跑个自检」去拖一个窗口系统进来
// ---------------------------------------------------------------------------
// 所以本模块只做两件事，且这两件事恰好就是 ui 的全部实质：
//   1. **命令解析**：把按键 / 网页表单 / 串口命令行收敛成同一个指令集
//      （ps_ui_exec），任何输入源都只是把字符串喂进来；
//   2. **状态机**：合法的状态迁移 + 与 §8.2 寄存器一一对应的作业配置
//      （ps_ui_to_job_config）。
// 渲染（ps_ui_render）输出的是**文本版四区画面**，用于联调时肉眼核对；
// 板上把同一个状态机接到 compositor 的 OSD 寄存器/网页前端即可。
//
// UI 只发配置，不参与重建（方案 §7 明确）
// ---------------------------------------------------------------------------
// 本模块**不碰**任何像素、不做重建、不做上采样。它产出的唯一东西是一个
// ps_reg_job_config_t，由 fpga_control 写进寄存器。
//
// 显示画面固定为四区（方案 §7）
// ---------------------------------------------------------------------------
//   左上：相机看到的墙面 ROI
//   右上：隐藏图像重建
//   左下：遮挡位置网格、最佳/次佳分数和可信度
//   右下：帧率、采集延迟、PL 计算延迟、秩、位宽、迭代、资源和功耗
// 性能页（page=perf）把右下的内容铺开，其余三区保留标题（演示时切换用）。
//
// 「1/4/8 帧」与「8/12/16/32 迭代」为什么也要在 UI 上
// ---------------------------------------------------------------------------
// 帧数与迭代数直接决定延迟：方案 §5.3 的定位 ≤100 ms / 16 次重建 ≤300 ms 就是
// 在特定档位上说的。演示时切换档位必须同时把 RANK/ITER/FRAME 三个寄存器一起改，
// 否则会出现「迭代改成 32 但仍按 16 的预算报延迟」这类量纲错位。
// 故 ps_ui_to_job_config 一次写全四个字段，且都用 ps_regs.h 的允许集合校验。
// ============================================================================

#ifndef PENUMBRASIGHT_UI_SERVER_H
#define PENUMBRASIGHT_UI_SERVER_H

#include <stddef.h>
#include <stdint.h>

#include "ps_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------- 执行结果

#define PS_UI_OK              0
#define PS_UI_QUIT            1   // 收到 quit：调用方结束交互循环
#define PS_UI_ERR_UNKNOWN_CMD 2   // 未知命令
#define PS_UI_ERR_BAD_ARG     3   // 参数不在允许集合
#define PS_UI_ERR_ARG         4   // 入参为空等调用错误
#define PS_UI_NOTE           5   // 命令执行了，但只是查询（status/help）

// ---------------------------------------------------------------- 页面/模式

#define PS_UI_PAGE_MAIN 0u   // 四区主画面
#define PS_UI_PAGE_PERF 1u   // 性能页（右下铺开）

// ---------------------------------------------------------------- 状态

// 状态 = 一份「作业配置」+ 一个页面号 + 一个版本号。
// rev（revision）每次成功修改自增：上层靠它判断「要不要重新写寄存器」，
// 而不是每次轮询都把所有寄存器重写一遍。
typedef struct {
    uint32_t selfloc;            // 1 = 自定位；0 = 固定位置
    uint32_t rgb;                // 1 = RGB；0 = 灰度
    uint32_t frames;             // 1 / 4 / 8
    uint32_t iters;              // 8 / 12 / 16 / 32
    uint32_t rank;               // 32 / 48 / 64 / 96
    uint32_t candidate_count;    // 默认 243
    int32_t  fixed_candidate_id; // selfloc = 0 时使用
    uint32_t page;               // PS_UI_PAGE_*
    uint32_t rev;                // 成功修改次数
    uint32_t rejected;           // 被拒的指令数（非法参数/未知命令）
} ps_ui_state_t;

void ps_ui_state_init(ps_ui_state_t* s);

// ---------------------------------------------------------------- 指令

// 执行一条指令。out/cap 为可选的人读回执（可为 NULL/0）。
//   返回 PS_UI_OK / PS_UI_QUIT / PS_UI_NOTE / PS_UI_ERR_*
// 指令集（大小写不敏感，'=' 与空格等价，'#' 起始为注释）：
//   loc self|fixed            切换自定位 / 固定位置
//   selfloc | fixed           上面两条的简写
//   color mono|rgb            灰度 / RGB
//   frames 1|4|8              帧累积数
//   iters 8|12|16|32          迭代次数
//   rank 32|48|64|96          重建秩
//   cand N                    候选数（1..243）
//   fixed-id N                固定位置模式的候选编号
//   page main|perf            切页
//   status                    打印当前状态与四区画面
//   reset                     回到复位默认值（§8.2 的默认档）
//   help                      打印指令表
//   quit / exit               退出
int ps_ui_exec(ps_ui_state_t* s, const char* line, char* out, size_t cap);

// 板端/网页前端共用的一份指令表文本（帮助与错误提示都从它出）。
const char* ps_ui_help(void);

// ---------------------------------------------------------------- 渲染

// 显示用的实时量。**不存在这里的东西一律不显示** —— 免得出现「画面上的帧率
// 是编的」这种情况。主机自检用固定值填，并在输出里显式标注为样例数据。
typedef struct {
    double   fps;
    int64_t  exposure_us;
    uint64_t capture_latency_us;   // 采集到 DDR 可读
    uint64_t pl_latency_us;        // PL 计算延迟
    uint32_t rank;
    uint32_t bits;                 // 输入有效位深
    uint32_t iterations;
    uint32_t power_mw;
    uint32_t selected_id;
    uint16_t confidence_q16;
    uint16_t best_score_q16;
    uint16_t second_score_q16;
    uint32_t candidate_count;
    uint64_t frames_processed;
} ps_ui_metrics_t;

void ps_ui_metrics_init(ps_ui_metrics_t* m);

// 渲染文本版画面（主画面或性能页，取决于 state->page）。
// 返回写入的字符数（不含 NUL）；cap == 0 或指针为空返回 -1。
int ps_ui_render(const ps_ui_state_t* s, const ps_ui_metrics_t* m,
                 char* out, size_t cap);

// ---------------------------------------------------------------- 与寄存器对接

// 状态 -> 作业配置（ui 唯一的产出）。**不写寄存器**：写寄存器是 fpga_control 的事。
void ps_ui_to_job_config(const ps_ui_state_t* s, ps_reg_job_config_t* cfg);

// 作业配置 -> 状态（读回寄存器后的反向同步，供 ui 显示真实档位；
// 板上「UI 显示的是 PS 里以为的值、PL 里其实是另一个值」正是靠这个函数暴露的）。
void ps_ui_from_job_config(ps_ui_state_t* s, const ps_reg_job_config_t* cfg);

const char* ps_ui_page_name(uint32_t page);
const char* ps_ui_loc_name(uint32_t selfloc);
const char* ps_ui_color_name(uint32_t rgb);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // PENUMBRASIGHT_UI_SERVER_H
