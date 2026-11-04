# Non-Line-of-Sight Sensing and Early Warning System

工程基线 `20261024-nb18-rc1`，完整算法 `20261017-p789-p1011-v1`，硬件 `20261016-rp2-p56-v1`。本版整理既有源码与可追溯结果，统一英文路径，报告正文允许中文。

| 推荐目录 | 本工程位置 |
|---|---|
| src | src/README.md → project/repo/src、project/releases/performance/src |
| sim | sim、project/repo/tests、project/releases/performance/reports |
| build | build、project/releases/performance/scripts |
| board | board、project/repo/board |
| data | project/data（97例描述与去重资产） |
| skill | skill/README.md（16条经验、提示词、模板与校验脚本） |
| report | report（设计/技术报告、优化对比、AI协作Markdown说明） |

七功能模块入口为 01_fpga_hardware 至 07_documentation_licenses，具体路径见 modules.json。README.md 为模板允许的大小写例外，其余路径统一小写英文、数字、下划线、连字符或扩展名点号。算法案例ID和HDL模块名不是文件路径，不改变其语义。

## 使用

```sh
python -m pip install -r requirements.txt
python tools/verify_package.py
python tools/check_submission.py
python tools/run_reconstruction.py --case Chair_mushroom --output ../ps_mushroom_result
```

输入案例ID保持原大小写；新文件路径已同步更新。Windows x64、Linux AArch64提供冻结原生CPU求解器；板端PL和电脑CPU入口区分，见 docs/build_run.md。运行前先取得Git LFS实体文件。构建与验证产物的原始历史记录保留日期与范围，不把较早离线状态改写为较晚上板状态。

## 报告与材料

项目方提供的完整技术报告：[report/design_report.pdf](report/design_report.pdf)。

大模型协作说明 report/llm_collaboration/README.md 已经团队审核确认。性能与资源依据见 report/optimization_comparison.md，材料核对入口见 docs/review_checklist.md。本次不运行相机、FPGA、重建或Vivado；只运行只读打包检查及技能脚本正反例。

原始原理与公开数据：Saunders, Murray-Bruce, Goyal, Nature (2019), Computational periscopy with an ordinary digital camera。许可见 license 与 07_documentation_licenses/third_party.md；第三方许可不被项目许可替代。

## Windows 文件路径

推荐直接使用 nlos 文件夹，或将 nlos.zip 解压到较短的目录。已将导出 IP 的两个浮点仿真模型归档到 project/releases/performance/hardware_evidence/exported_ip/sim；HDL 文件内容和模块名保持原样。原工具报告中的历史路径保留，原路径到当前路径的对应关系见 docs/name_map.json。
