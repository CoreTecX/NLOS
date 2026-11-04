# 97 例数据与复现

基线 `20261024-nb18-rc1` 的 97 条 PFY 输入全部包含在 `project/data/batch_manifest.json`。每个描述文件关联测量 PFY、模型 PFM 和冻结参考 PFG，并保留字节数与 SHA256。相同模型只在 assets 中保存一次。原始 RAW 现场记录与 SC 系列数据不放入本工程。

使用 `python tools/run_reconstruction.py --case <案例ID> --output <新目录>` 逐例执行。案例 ID 直接取 batch_manifest.json 的 items[].id。此入口使用 --input 读取匹配描述，不依赖 performance/data 中的少量示例集合。

PFM/PFY/PFG 是自定义格式，不能按同名通用图像格式解释。参考 PFG 用于数值校验，不替换求解结果。图像输出为 36×29 原生网格，放大图采用最近邻插值。

97 例数据来源是冻结论文匹配测量集；数据重建、相机实时预览、实体场景重建分别保留原接口语义。这里不把这些输入混写成同一种测试。
