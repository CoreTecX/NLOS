# PL 耗时标定与访存带宽瓶颈，以及 RTL 五模块的 `input logic` 门禁

- 日期：2026-09-28
- 主题：PL 耗时标定（latency calibration）、M1 综合、带宽瓶颈
- 任务指令（用户原文）：**「按 100 MHz 继续，先解决 PL 耗时标定和 M1 综合」**
  （此前已明确「配置降低和精度不够目前可以接受，不需要改变，现在主要的任务是把没有做的先做完」）
- 结论：PL 耗时瓶颈是**访存带宽**，不是 DSP；RTL 五模块在 Vivado 下原本一个都综合不了

## 一、问题：csynth 报的顶层 Latency 是噪声

生产档 `csynth_design` 报出的顶层 Latency 是 **1.24e12 周期**。根因：`K` / `M` / `N` /
`n_iter` / `n_rows` / `n_cols` 全是运行期 `s_axilite` 参数，HLS 在综合期不知道它们是多少，
只能按 maxbound `2^28` 外推。报告里可见 `Count = 268435456`，正是 `2^28`。因此生产档的
绝对延迟数字不能填进标定结论。

## 二、修法：新增 `cal` 档（`make hls-cal`）

`cal` 档与生产档 **同源、同顶层**，只把 B 档尺寸与迭代数**赋成编译期常量**，循环即可静态
定界，csynth 报的延迟才是真值。算法一行不改。

## 三、实测（B 档，100 MHz）

| 顶层 | Latency (cycles) | PL 耗时 @100 MHz |
|---|---|---|
| `projection` | 53,235,954 | **532.4 ms** |
| `lr_forward` | 636,951 | **6.37 ms** |
| `lr_adjoint` | 613,602 | **6.14 ms** |
| `pdhg` | 未取到 | — |

`pdhg` 未取到的原因：Vitis 前端 clang **段错误**
（`clang frontend command failed due to signal`），不是 csynth 报错。

## 四、根因（本文件核心结论）：瓶颈是访存带宽，不是 DSP

三个核加起来只用 **50 个 DSP，占器件 2%**。瓶颈在 **m_axi 一拍一个字**，
`Max Widen Bitwidth = 0`（未放宽位宽）。量级核对：

```
一次调用要读的字数 = 3 × M × K × P = 3 × 9120 × 8 × 243 = 53.2 M 个字
32 bit m_axi @ 100 MHz = 400 MB/s
53.2 M ÷ 400 MB/s = 532.4 ms
```

**与实测周期数逐位吻合。** 这条吻合反过来证明标定数字可信。

## 五、四条出路（如实列出，未全部执行）

| # | 出路 | 说明 |
|---|---|---|
| 1 | `meas` 改块交织布局，或 m_axi 位宽提到 128~256 bit | 带宽 4~8 倍 |
| 2 | 低秩算子的 `x` 向量在片上缓存一次、复用 K 次 | 从根上消掉 K 倍重复读 |
| 3 | 升频（200 MHz） | 收不住，且只是线性改善 |
| 4 | 加 DSP | **无效** —— 瓶颈不在乘法器 |

## 六、RTL 侧同一批：五模块原本一个都综合不了

五个 RTL 模块在 Vivado 下原本**一个都综合不了**：`default_nettype none` 下 Vivado 拒绝
`input logic`，只接受 `input wire logic`。

**最小复现对照**（三组全部失败 → 改后全部通过）：

| 方式 | 改前 | 改后 |
|---|---|---|
| `read_verilog -sv` | 失败 | 通过 |
| `read_verilog` | 失败 | 通过 |
| 工程模式 `add_files` | 失败 | 通过 |

根因是**端口声明写法**，与按哪种语言解析无关。原地改 **74 处**，`make rtl-sim` 复跑 **5/5 PASS**。

## 七、探针脚本的坑

第一版批量替换**没有限定语法范围**，把函数/任务里的 `input logic` **形参**也改了
（那里加线网类型非法），`iverilog` 直接报 `Syntax error in task/function port declaration`。
**修法**：回滚后加限定 —— **只在模块端口列表区间内替换**。

关联证据：`sim/results/x19_hls_resources_and_timing.md`
