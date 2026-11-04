# 可复用技能包

本目录供其他 FPGA 项目复用，不要求使用本项目板卡、数据或算法。16 条经验详见 [experience_catalog.md](experience_catalog.md)，每条包含适用场景、使用方法、已验证效果、失效条件和来源。

| 内容 | 用法 |
|---|---|
| prompts/review_workflow.md | 用证据约束算法移植、软硬件划分和报告审阅 |
| templates/verification_plan.md | 复用验收计划与性能对比表 |
| scripts/check_names.py | 检查仓库文件名是否合规 |
| scripts/compare_arrays.py | 独立比较两份 NumPy 数值结果 |
| scripts/check_vivado_reports.py | 从 routed 审核 JSON 检查时序与 DRC |
| experience_catalog.md | 16 条既有经验，保留其原有有效范围与历史背景 |

上述脚本均可 --help。检查脚本不能替代功能测试。此技能包不使用 PYNQ，也不声称获得通用 PYNQ Skill 加分。

提炼过程：从固定点增益归零、块设计漏接、动态循环延迟外推、RTL 多驱动与实现内存不足等观察中抽取触发条件；把仅适用于本题的路径与常量剥离；以正反例确认检查脚本会识别失败。具体回溯案例见 report/llm_collaboration/source_notes。
