"""X4d —— 论文口径 A 与 MATLAB simA 的逐值对标

对同一份参照量（ref_out/raw_chair_va*__simA.f64）跑若干变体，看哪一种能落到
相对误差 ≤ 1e-6。变体只切换规格书里标记为「待实测」或「待定」的开关：
  mz_reversed  —— Mz 是否用反向的 z 数组
  多边形个数    —— 是否补上椅腿
  视角因子      —— va1 对 va1、va0 对 va0
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_MODEL_DIR = os.path.abspath(os.path.join(_HERE, ".."))
if _MODEL_DIR not in sys.path:
    sys.path.insert(0, _MODEL_DIR)
from penumbrasight_model import forward as F             # noqa: E402
from penumbrasight_model import forward_paper as FP      # noqa: E402


def load_raw(raw_dir, tag, name):
    with open(os.path.join(raw_dir, "raw_%s__%s.shape" % (tag, name))) as fh:
        shape = tuple(int(s) for s in fh.read().split())
    a = np.fromfile(os.path.join(raw_dir, "raw_%s__%s.f64" % (tag, name)), dtype="<f8")
    return a.reshape(shape, order="F")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-root", required=True)
    ap.add_argument("--raw-dir", required=True)
    ap.add_argument("--report", default=None)
    args = ap.parse_args()

    out = open(args.report, "w", encoding="utf-8") if args.report else sys.stdout
    p = lambda *a: print(*a, file=out)

    cfg = os.path.join(args.data_root, "TestPosChair", "chair_test.mat")
    setup = F.load_setup(cfg)
    occ_corner = load_raw(args.raw_dir, "chair_va1", "occ_corner")   # (4,3,2)

    simA1 = load_raw(args.raw_dir, "chair_va1", "simA")
    simA0 = load_raw(args.raw_dir, "chair_va0", "simA")
    p("参照 simA va1 %s   va0 %s" % (simA1.shape, simA0.shape))
    p("|simA_va1|_F = %.9e    max = %.9e" % (np.linalg.norm(simA1), np.abs(simA1).max()))

    polys2 = FP.default_polygons(setup, occ_corner)
    polys1 = FP.default_polygons(setup, None)
    p("\n多边形（来自 occ_corner）：")
    for i, q in enumerate(polys2):
        p("  #%d  x[%.6f, %.6f]  z[%.6f, %.6f]" % (i + 1, q[0], q[1], q[2], q[3]))
    p("单矩形退化口径： x[%.6f, %.6f]  z[%.6f, %.6f]"
      % (polys1[0][0], polys1[0][1], polys1[0][2], polys1[0][3]))

    cases = [
        ("va1 · Mz反向 · 双多边形", simA1, dict(view_angle=True,  mz_reversed=True,  polygons=polys2)),
        ("va1 · Mz正向 · 双多边形", simA1, dict(view_angle=True,  mz_reversed=False, polygons=polys2)),
        ("va1 · Mz反向 · 单矩形",   simA1, dict(view_angle=True,  mz_reversed=True,  polygons=polys1)),
        ("va0 · 无视角 · 双多边形", simA0, dict(view_angle=False, mz_reversed=True,  polygons=polys2)),
    ]

    p("\n" + "=" * 84)
    p("逐值对标")
    p("=" * 84)
    p("%-26s %14s %14s %14s %10s" % ("变体", "相对误差", "max|dA|", "相关系数", "耗时s"))
    best = None
    for name, ref, kw in cases:
        t0 = time.time()
        A, meta = FP.build_paper_A(setup, **kw)
        el = time.time() - t0
        err = float(np.linalg.norm(A - ref) / np.linalg.norm(ref))
        dmax = float(np.abs(A - ref).max())
        c = float(np.corrcoef(A.reshape(-1), ref.reshape(-1))[0, 1])
        p("%-26s %14.6e %14.6e %14.9f %10.1f" % (name, err, dmax, c, el))
        if best is None or err < best[1]:
            best = (name, err, A, ref, meta)

    p("\n最优变体： %s   相对误差 %.6e" % (best[0], best[1]))
    A, ref, meta = best[2], best[3], best[4]
    p("  |A|_F = %.9e   |simA|_F = %.9e   比值 %.9e"
      % (np.linalg.norm(A), np.linalg.norm(ref), np.linalg.norm(ref) / np.linalg.norm(A)))
    p("  max|A| = %.9e   max|simA| = %.9e" % (np.abs(A).max(), np.abs(ref).max()))

    # 逐元素抽样对照，便于肉眼判断是「量级差」还是「形状差」
    p("\n逐元素抽样（前十行第一列）：")
    p("  %-16s %-16s" % ("我们的 A", "MATLAB simA"))
    for k in range(10):
        p("  %-16.9e %-16.9e" % (A[k, 0], ref[k, 0]))

    # 非零元集合
    nz_a = (A != 0)
    nz_r = (ref != 0)
    p("\n非零元： 我们 %d  MATLAB %d  交集 %d  完全相同 %s"
      % (int(nz_a.sum()), int(nz_r.sum()), int((nz_a & nz_r).sum()),
         bool(np.array_equal(nz_a, nz_r))))

    # 逐列相对误差的分位
    ca = np.linalg.norm(A, axis=0)
    cr = np.linalg.norm(ref, axis=0)
    ok = cr > 0
    colr = np.linalg.norm(A[:, ok] - ref[:, ok], axis=0) / cr[ok]
    p("\n逐列相对误差： 中位 %.3e   90分位 %.3e   max %.3e"
      % (float(np.median(colr)), float(np.percentile(colr, 90)), float(colr.max())))

    if out is not sys.stdout:
        out.close()
        print("报告已写入 %s" % args.report)


if __name__ == "__main__":
    main()
