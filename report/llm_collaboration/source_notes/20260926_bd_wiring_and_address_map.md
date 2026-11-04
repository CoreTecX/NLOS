# 块设计接线与地址分配：旧脚本四处实质错误与 s_axi_control_r 漏接

- 日期：2026-09-26
- 主题：块设计接线（block design wiring）与定点 IP 地址分配
- 任务指令（用户原文）：**「好的，先执行块设计接线和定点 IP 地址分配」**
- 结论：`validate_bd_design` 通过（耗时 1 分 06 秒）、退出码 0、wrapper 已生成；但通过校验
  不等于正确，本文件记录一次被校验放过的漏接

## 一、背景：旧 `penumbrasight_bd.tcl` 从未被 Vivado 跑过

旧版脚本落盘时本机没有 Vivado，跑不了 `validate_bd_design`，因此带着**四处实质错误**：

| # | 旧脚本 | 实测事实 | 若照旧版上板的后果 |
|---|---|---|---|
| 1 | IP 用双精度层 `ps_*_axi` | XCZU9EG 的 DSP48E2 是 27×18 乘法器，做不了双精度 | **本来就上不了板** |
| 2 | 设 `CONFIG.PSU__USE__S_AXI_GP2 {1}` | 本 PS 没有该端口；真实端口是 `M_AXI_HPM0_FPD`、`S_AXI_HP0_FPD`、`S_AXI_HP1_FPD` | 该设置是空设 |
| 3 | 从未连 PS 的 AXI 时钟脚 | 需连 `maxihpm0_fpd_aclk`、`saxihp0_fpd_aclk`、`saxihp1_fpd_aclk`（**小写缩写，与端口名不是简单拼接**，靠按后缀枚举才拿到） | PS 侧 AXI 无时钟 |
| 4 | 每个核只接了 `m_axi_gmem0_model` 一组 | 四核共 **11 组** m_axi，另 7 组悬空 | 板上「读得到模型、**写不出结果**」 |

另修三处 Tcl 层面的错误（都会让脚本中途失败）：

| 写法 | 问题 | 正确做法 |
|---|---|---|
| `mkdir -p` | 不是 Tcl 命令 | `file mkdir` |
| `update_ip_catalog -add_ip` | 要的是 `component.xml` 的路径，且必须先把 `ip_repo_paths` 注册进工程 | 否则报 `[IP_Flow 19-925]` |
| `assign_bd_address` 直接收字符串段名 | 报 `[Common 17-161]` | 先取到段对象再赋址 |

## 二、地址映射（板端软件要照抄这一段）

### 控制段（各 64 KiB，每核两段）

| 核 | `s_axi_control` | `s_axi_control_r` |
|---|---|---|
| `projection` | `0xA0000000` | `0xA0010000` |
| `lr_forward` | `0xA0020000` | `0xA0030000` |
| `lr_adjoint` | `0xA0040000` | `0xA0050000` |
| `pdhg` | `0xA0060000` | `0xA0070000` |

> **2026-10-01 补**：本表（2026-09-26 当轮）只含四个核的 8 段。此后块设计又在其后
> 追加了 `dma_pipe` `0xA0080000`、`dma_disp` `0xA0090000` 与 `ps_regs` `0xA00A0000`
> 三段，控制段合计 **11 段**。本文正文按当日事实保留；当前权威表见
> `sim/results/bd_wiring_and_address_map.md` §3。

### 数据侧

每个 `m_axi` master 映射到 PS 的 DDR 段：

- `/zynq_ps/SAXIGP2/HP0_DDR_LOW`（`0x0`，2 GiB）
- `/zynq_ps/SAXIGP2/HP0_DDR_HIGH`

**易错点**：PS 的 DDR 段不叫 `zynq_ps/Data`，而是**按从端口分组**的
`/zynq_ps/SAXIGP2/HP0_DDR_*`。

## 三、自我纠错轨迹（本文件核心）

**第一轮**：脚本改完跑通 `validate_bd_design` 后，AI 判断「块设计这一层算完了」。

**被推翻的触发**：用户随后问「现在有什么需要注意的问题吗」。复查定点核 IP 的
`component.xml`，发现**每个定点核有两个 AXI-Lite 从接口**：

| 接口 | 装什么 |
|---|---|
| `s_axi_control` | 标量：尺寸、增益、迭代参数、回读标志与结果 |
| `s_axi_control_r` | **m_axi 缓冲区的基址寄存器**（`meas` / `u_codes` / `score_out` 等） |

当时只接了前者，**四条 `s_axi_control_r` 全部悬空**。

**关键教训**：`validate_bd_design` 照样通过，**连一条「未连接」告警都没报**。若按那一版
上板，核拿不到缓冲区基址、只会按地址 0 读写，板上必错，且现场极难定位（作业能启动、
结果全是垃圾）。这条与 RTL 侧「`synth_design` 0 error 不等于能上板」是同一类教训。

**修法**：

1. 控制互联从 4 条扩到 **8 条**；
2. 地址重排为**每核两段**（见 §二控制段表）；
3. 新增**连通性自检** —— 遍历每个从接口与每个 `m_axi`，**悬空即报错退出**。

## 四、结果与遗留边界

| 项 | 结果 |
|---|---|
| `validate_bd_design` | 通过 |
| 耗时 | 1 分 06 秒 |
| 退出码 | 0 |
| wrapper | 已生成 |

- 通过校验不等于 AXI 接口参数（位宽、ID 宽度、突发长度）已核对；工具未验的部分不声称已核对。
- 综合、实现、时序收敛、bitstream、板端软件都未做，本文件只记录「块设计这一层」。
  **后续（2026-09-30）**：综合、实现、时序收敛与 bitstream 均已跑通（位流 17 MB，
  WNS +0.769 / WHS +0.011 ns，DRC 无违例）；板端软件仍未在板上验证。
  见 `sim/results/pl_bitstream_first_pass_20260930.md`。

关联证据：`sim/results/bd_wiring_and_address_map.md`、`sim/results/ps_fix_regmap.md`、`docs/port_completeness.md`
