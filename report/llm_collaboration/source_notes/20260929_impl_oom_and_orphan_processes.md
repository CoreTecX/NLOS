# 实现阶段 OOM 与孤儿进程：并发数按内存定，不按核数定

- 日期：2026-09-29
- 主题：`impl_1` 阶段 8 路并发 OOM、run 状态复位、孤儿 Vivado 进程
- 任务指令（用户原文）：**「跑完需要多长时间」** → **「那先不跑了,还有什么能做的吗」**
- 结论：单个子运行要 3~4 GB，8 路并发当场 OOM；`JOBS` 默认改回 1

## 一、现象：`-jobs 8` 当场失败

实现阶段用 `-jobs 8` 启动，8 个子运行同时起，**当场失败**。失败子运行的 `runme.log`
末行是：

```
out of memory allocating 8388640 bytes
```

`core_lr_fwd` / `core_lr_adj` 死于 OOM。

## 二、实测：单个子运行就要 3~4 GB

| 项 | 数值 |
|---|---|
| 物理内存 | **15.5 GB** |
| 逻辑核 | **18** |
| 单子运行峰值 `zynq_ps` | 3,971 MB |
| 单子运行峰值 `sctl_ctrl` | 3,276 MB |
| 单子运行峰值 `core_proj` | 3,167 MB |
| 单子运行峰值 `rst_pl` | 2,964 MB |
| 失败时系统可用内存 | **1.9 GB** |

**单个子运行就要 3~4 GB**。8 个同时起远超 15.5 GB，**串行也救不了**（可用内存只剩 1.9 GB，
连一个子运行都放不下）。

## 三、两个次生问题

### 3.1 run 留在错误态

上次 OOM 把 run 留在错误态，重跑报：

```
ERROR: [Common 17-69] Run 'synth_1' needs to be reset before launching.
```

**修法**：脚本加**幂等重置** —— 按 `PROGRESS` 判断，只有真跑到 `100%` 的才沿用，
其余一律 `reset_run`。

### 3.2 孤儿 Vivado 进程

父进程被停掉后，**留下孤儿 Vivado 进程**继续跑综合、占 3.7 GB。清掉后可用内存从
**1.9 GB 回到 5.8 GB**。

**教训**：停批次任务要**连带清子进程**；长任务一律 `Tee-Object` 落盘，不只靠终端回看。

## 四、处置

`build_pl.tcl` 的 `JOBS` **默认改为 1**，并支持环境变量覆盖：

```tcl
set JOBS [expr {[info exists ::env(PS_PL_JOBS)] ? $::env(PS_PL_JOBS) : 1}]
```

```powershell
# 不改文件即可调整
PS_PL_JOBS=1 vivado -mode batch -source src/fpga/scripts/build_pl.tcl
```

**并发数应按内存定，不是按核数定。**

## 五、工时估算（用户问「跑完需要多长时间」时的答复）

| 阶段 | 估算 |
|---|---|
| 小 OOC 4 个 | 约 6 分钟 |
| HLS 核 4 个 | 约 40 分钟 |
| 顶层 `synth_1` | 约 15~30 分钟 |
| `impl` 到 bitstream | 约 60~90 分钟 |
| **全流程合计** | **约 2~2.5 小时** |

用户据此回复「那先不跑了,还有什么能做的吗」，批次任务暂停。

## 六、遗留边界

- 实现 / 时序收敛 / bitstream 均未完成，本文件只记录 OOM 现象、内存实测与处置。
- 上述子运行峰值是失败那一次运行中观测到的值，不代表所有配置下的稳定峰值。
- **后续（2026-09-30）**：按本文件的串行处置（`JOBS=1`）实跑，实现到 bitstream 已成功，
  内存峰值 7,429 MB、耗时约 1 小时 34 分。见 `sim/results/pl_bitstream_first_pass_20260930.md`。

关联证据：`src/fpga/scripts/build_pl.tcl`
