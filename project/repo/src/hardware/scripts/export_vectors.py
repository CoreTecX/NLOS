# -*- coding: utf-8 -*-
"""把测试向量（npz）导出为 HLS 主机测试可读的小端二进制。

输出目录：src/hardware/hwdata/
  *.bin          按原生 dtype 存放的连续数组
  manifest.txt   字段清单（供 C++ 测试与人工核对）

约定：所有 .bin 都是「连续、小端、原生 dtype」，C++ 侧按同名类型直接读。
"""
import os

import numpy as np

# 路径从本文件位置推导，不写死绝对路径。原因：同一份源码需在**两个检出**里都能跑 ——
#   ① 工作区检出（含中文路径，用于文档 / 分析 / Python 参考实现）
#   ② ASCII 构建区 D:\psight\penumbrasight（Vivado / HLS 用，路径必须全 ASCII）
# 两者由 git 同步（说明见 docs/environment_ascii_build.md）。写死任一边，都会让另一边
# 把向量导出到错误的树里。
_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                     os.pardir, os.pardir, os.pardir))
VEC = os.path.join(_REPO, "sim", "vectors")
OUT = os.path.join(_REPO, "src", "hardware", "hwdata")

FILES = ["v1_block_small.npz", "v2_frame_full.npz", "v3_accum_frames.npz",
         "v4_edge_cases.npz"]

os.makedirs(OUT, exist_ok=True)
log = []


def say(s=""):
    log.append(s)
    print(s, flush=True)


total_bytes = 0
for fn in FILES:
    p = os.path.join(VEC, fn)
    if not os.path.exists(p):
        say(f"缺失：{fn}")
        continue
    z = np.load(p)
    stem = fn.replace(".npz", "")
    say("=" * 78)
    say(f"{fn}")
    say("=" * 78)
    for k in z.files:
        a = z[k]
        say(f"  {k:34s} shape={str(a.shape):22s} dtype={a.dtype}  size={a.size}")
        # 跳过超大数组的落盘（v2 的 1920x1920 已足够大，但仍导出）
        arr = np.ascontiguousarray(a)
        name = f"{stem}__{k}"
        arr.tofile(os.path.join(OUT, name + ".bin"))
        total_bytes += arr.nbytes
        if a.size <= 24:
            say(f"       值 = {a.ravel()}")
    say()

say(f"合计导出 {total_bytes / 1024 / 1024:.2f} MB -> {OUT}")

with open(os.path.join(OUT, "manifest.txt"), "w", encoding="utf-8") as fh:
    fh.write("\n".join(log))
print(f"\n清单写入 {os.path.join(OUT, 'manifest.txt')}")
