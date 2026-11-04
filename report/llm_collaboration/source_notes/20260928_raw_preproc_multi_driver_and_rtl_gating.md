# raw_preproc 的 21 条 critical warning：`br` 双驱动与 RTL 综合门禁

- 日期：2026-09-28
- 主题：`raw_preproc` 多驱动（multi-driven net）根因与修正
- 任务指令（用户原文）：**「先查 raw_preproc 的 21 条 critical warning」**
- 结论：`br` 被两个 `always_ff` 驱动，硬件上恒为 0，帧永远收不了尾；修前那个「资源很省」是假象

## 一、现象

首次综合报 21 条**同一类** critical warning：

```
CRITICAL WARNING: [Synth 8-6859] multi-driven net on pin Q ...
CRITICAL WARNING: [Synth 8-6858] multi-driven net Q is connected to at least one
                                 constant driver which has been preserved, other driver is ignored
```

`fast_alert` 另有 2 条。**21 条 = 7 位 × 3 条**。

## 二、根因：`br` 被两个 `always_ff` 驱动

块行计数器 `br` 的赋值分散在两个 `always_ff`：

| 位置 | 行为 |
|---|---|
| 计数器块 `raw_preproc.sv:417` | 在复位与 `ST_IDLE` **清它** |
| 主状态机 `raw_preproc.sv:490` | 在 `ST_RMW_WB` 里做 `br <= br + 1'b1` |

Vivado 把前者恒 0 的赋值折叠成 **GND 常量驱动**，于是「**保留 GND、忽略寄存器**」。
后果：**硬件上 `br` 恒为 0** → 「最后一条块行」（`br == WALL_N-1`）永不成立 →
**帧永远收不了尾**。

`fast_alert` 的 `cent_pending` 同类：`:237` 置位、`:462` 清零。与 `br` 不同的是**两个驱动
都是寄存器、没有常量驱动**，Vivado 无法「保留常量、忽略另一个」，结果是**综合结果不确定**，
硬件与仿真很可能不一致。

## 三、铁证：修前 BRAM 只有 4

修前 `raw_preproc` 的 BRAM 只有 **4**，而 `96×96×3×23 bit` 的帧累积器本身要约 **19 个
BRAM36**。累积存储根本没被按意图建出来。

| 项 | 修前 | 修后 |
|---|---|---|
| BRAM | **4** | **18** |
| LUT | 2,140 | 4,340 |
| FF | 3,261 | 6,289 |

**修之前那个「资源很省」是假象。** BRAM 由 4 回到 18，正是「累积存储没被建出来」这句判断
的直接证据。

## 四、关键教训

1. **`iverilog` 对「两个 `always` 块驱动同一变量」不当错误。** 所以 `make rtl-sim` 全绿
   **不能**证明 RTL 能上板。
2. **`synth_design` 的 critical warning 必须当成门禁看。** 只数 error 会漏掉这一类。
3. `validate_bd_design` 通过、`rtl-sim` 全绿、`synth_design` 0 error，都不等于「硬件上是
   对的」—— 这三件事在 2026-09-26 的块设计漏接与 2026-09-28 的这一批里各出现一次。

## 五、自我纠错轨迹

- 第一轮判定「仿真 5/5 PASS、综合 0 error，可以进入下一步」。用户提出查 21 条 critical
  warning 后，复查 `synth_design` 的 warning 段，才发现这批被「只数 error」漏掉的多驱动告警。
- 判据随之改写：把 critical warning 数量纳入综合门禁，而非只看 error 数量。
- 修法：`br` 的清零移入主状态机（复位段与三处 `state <= ST_IDLE`），计数器块不再碰它；
  `cent_pending` 统一到主状态机。重综合后两模块均为 **0 errors / 0 critical warnings**，
  `make rtl-sim` 仍 5/5 PASS。

## 六、遗留边界

- 修前 `raw_preproc` 的 BRAM=4 与修后 BRAM=18 是同一份 RTL 在两个版本下的实测值，
  两者的 RTL 差异仅限 `br` / `cent_pending` 的驱动归属。
- 五个 RTL 模块的 WNS 全为正，但顶层 `synth_1` / 实现 / bitstream 均未跑，资源与时序的
  最终口径未定。
  **后续（2026-09-30）**：顶层综合（2026-09-29）、实现与 bitstream 均已跑通，最终口径已定 ——
  实现后全设计 LUT 44.39% / FF 30.59% / DSP 7.78% / Block RAM Tile 0.60%，
  10.000 ns 下 WNS +0.769 / WHS +0.011 ns。见 `sim/results/pl_bitstream_first_pass_20260930.md`。

关联证据：`sim/results/rtl_root_cause_fast_alert_raw_preproc.md`、`sim/results/x19_hls_resources_and_timing.md`
