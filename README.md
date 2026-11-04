# 非视域智能感知预警系统

Intelligent Non-Line-of-Sight Sensing and Early Warning System

团队：CoreTecX。工程基线：`20261024-nb18-rc1`；完整算法：`20261017-p789-p1011-v1`；硬件：`20261016-rp2-p56-v1`。

本工程面向 ALINX AXU9EGB / XCZU9EG，包含半影非视域成像算法、FP64 重建、PS/PL 协同计算、相机采集、动态检测、范围调整和控制显示。沿用既有提交代码基础，保留统一源码布局，另补齐 97 例匹配输入、模型及数值参考，便于下载后复现。

## 七个模块

| 目录 | 内容 |
|---|---|
| 01_FPGA硬件工程 | RTL/HLS、自定义 IP、BD 重建、引脚与时序约束 |
| 02_完整重建算法 | FP64 求解、光传输接口、DCT、TV/非负约束、停止判据、PS/PL 协同 |
| 03_相机采集与预处理 | GigE 通信、RAW 接收、格式转换、参数配置与预处理 |
| 04_动态检测与范围调整 | 动态检测、双模式控制、配准与范围配置 |
| 05_控制与显示程序 | 电脑控制窗口、板端双栏显示、任务控制与结果保存 |
| 06_运行与验证资源 | 配置、冻结位流、可执行文件、97 例索引和示例入口 |
| 07_使用说明与许可证 | 使用说明、依赖与原始许可证 |

各模块 README 指向 `project/` 中的实际实现，不重复复制共享代码。`docs/` 为本次工程说明，`tools/` 为便携入口。

## 快速使用

先取得完整 Git LFS 文件。Windows x64 或 Linux AArch64 安装兼容的 Python、NumPy、SciPy、Pillow；电脑窗口另需 Python Tkinter。

```sh
python -m pip install -r requirements.txt
python tools/verify_package.py
python tools/run_reconstruction.py --case Chair_mushroom --output ../ps_mushroom_result
python tools/run_reconstruction.py --case D11_rgb --output ../ps_rgb_result
python tools/run_reconstruction.py --case D11_tommy --output ../ps_tommy_result
```

输出目录须为尚不存在的工程外目录。上述入口执行 CPU 完整算法，不访问相机或 FPGA；输出包含重建图、数组及独立数值检查。它与板端 PL 执行、实时相机预览是不同入口。

## 工程文档

- [目录与接口](docs/工程目录与模块.md)
- [环境、构建与部署](docs/构建与运行.md)
- [97 例数据与复现](docs/数据与复现.md)
- [本地提交检查](docs/提交说明.md)
- [第三方来源](07_使用说明与许可证/THIRD_PARTY.md)

本目录是工程文件集合，不包含 Git 历史、个人实时日志、聊天记录、SC 操作记录或工具实现缓存；程序正常生成结果和日志的功能保留。没有执行实体测试或重新综合。

原始成像原理、算法与公开数据来源：Saunders, Murray-Bruce, Goyal, *Computational periscopy with an ordinary digital camera*, Nature (2019)，Computational-Periscopy/Ordinary-Camera。项目许可证及第三方版权分别保留，根目录 LICENSE 不替代第三方许可。
