# 性能与资源对比

| 配置/指标 | 数值 | 依据与范围 |
|---|---:|---|
| D11_rgb，100 MHz，同步，2900迭代 | 46.61687498 s | board/results/profile_comparison/fallback_sync/acceptance.json |
| D11_rgb，150 MHz，overlap，2900迭代 | 28.90424933 s | board/results/profile_comparison/performance_overlap/acceptance.json |
| 组合方案加速 | 约1.613倍 | 同输入；频率与调度同时变化，非单因素消融 |
| CLB LUT | 41829 / 274080，15.26% | build/evidence/reports/implementation_utilization.rpt |
| CLB registers | 59543 / 548160，10.86% | 同上 |
| routed WNS / WHS | +1.120 / +0.010 ns | build/evidence/reports/routed_audit.json |
| DRC error / black box | 0 / 0 | 同上 |

这份资源表对应已归档的 performance 硬件，不把缺失的优化前资源数据补成零，也不宣称资源降低百分比。原综合、实现、时钟及 HLS 报告完整保留，供逐项复核。历史 p8_assessment、p10_critical_path 中的理论模型保持原标记，不混入上述实测表。
