# 本工程运行依赖

Python 科学计算依赖见根 requirements.txt；开发依赖见 requirements-dev.txt。Windows x64、Linux AArch64 的冻结原生求解器在 project/releases/performance/bin；Windows DLL 在 project/runtime/windows。

requirements-aarch64-cp37.txt 是原 ARM Python 3.7 环境的精确哈希锁，本工程未携带 wheelhouse。安装需按实际 Python ABI/系统选取兼容发行包，不直接使用缺失的离线目录。Tkinter 和相机厂商 SDK/驱动由对应平台提供。

FPGA 重建工具、编译选项及路径适配见 docs/build_run.md。
