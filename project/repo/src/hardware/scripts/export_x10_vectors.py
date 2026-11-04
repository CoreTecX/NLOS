# -*- coding: utf-8 -*-
"""X10 测试向量导出：官方第 1 轮搜索网格。

导出
----
    x10_grid.txt     纯文本：x/y/z 三组网格值与候选枚举顺序
    x10_grid.bin     float32[18+18+3]，顺序 x 慢、z 快，与 E_round1 的 C 展平一致
    x10_meta.bin     int32[4] = nx, ny, nz, n_cand

候选枚举顺序（与 numpy 对 (18,18,3) 做 ravel 一致）：
    k = ix*(ny*nz) + iy*nz + iz
这样 C++ 输出的第 k 个能量可以直接与 `localization_energy_cubes.npz` 里的
`E_round1[ix,iy,iz]` 对齐，不需要任何重排。

用法
----
    python src/hardware/scripts/export_x10_vectors.py \
        --data-root "D:\\学习\\26秋嵌入式\\论文数据集\\Ordinary-Camera-master\\Data"
"""

from __future__ import annotations

import argparse
import io
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_HW = os.path.dirname(_HERE)
_SRC = os.path.dirname(_HW)
_SRC_MODEL = os.path.join(_SRC, "model")
if _SRC_MODEL not in sys.path:
    sys.path.insert(0, _SRC_MODEL)

from penumbrasight_model import io_mat  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-root", required=True)
    ap.add_argument("--out-dir", default=os.path.join(_HW, "hwdata"))
    args = ap.parse_args()

    cfg = os.path.join(args.data_root, "TestPosChair", "chair_test.mat")
    d = io_mat.load_mat(cfg)
    xv = np.asarray(d["xhatvals"], dtype=np.float64).reshape(-1)
    yv = np.asarray(d["yhatvals"], dtype=np.float64).reshape(-1)
    zv = np.asarray(d["zhatvals"], dtype=np.float64).reshape(-1)
    nx, ny, nz = xv.size, yv.size, zv.size
    K = nx * ny * nz
    print("[info] 官方第 1 轮网格 %d x %d x %d = %d 个候选" % (nx, ny, nz, K))

    os.makedirs(args.out_dir, exist_ok=True)
    np.ascontiguousarray(np.concatenate([xv, yv, zv]),
                         dtype=np.float32).tofile(
        os.path.join(args.out_dir, "x10_grid.bin"))
    np.array([nx, ny, nz, K], dtype=np.int32).tofile(
        os.path.join(args.out_dir, "x10_meta.bin"))

    lines = [
        "# X10 官方第 1 轮搜索网格",
        "nx = %d" % nx,
        "ny = %d" % ny,
        "nz = %d" % nz,
        "n_cand = %d" % K,
        "# 候选枚举顺序：k = ix*(ny*nz) + iy*nz + iz",
        "# 与 localization_energy_cubes.npz 的 E_round1[ix,iy,iz] 用 C 展平后同序",
        "xhatvals = " + " ".join("%.10g" % v for v in xv),
        "yhatvals = " + " ".join("%.10g" % v for v in yv),
        "zhatvals = " + " ".join("%.10g" % v for v in zv),
    ]
    with io.open(os.path.join(args.out_dir, "x10_grid.txt"), "w",
                 encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print("  -> x10_grid.bin  x10_meta.bin  x10_grid.txt")
    print("      x: %.5f .. %.5f" % (xv.min(), xv.max()))
    print("      y: %.5f .. %.5f" % (yv.min(), yv.max()))
    print("      z: %.5f .. %.5f" % (zv.min(), zv.max()))
    print("[done] 输出目录 %s" % args.out_dir)


if __name__ == "__main__":
    main()
