"""X4e —— 用「论文口径的 A」跑 972 个候选的定位

与 x4c 的区别：x4c 用的 A 来自 `forward.build_transport_matrix`（我们自己的实现，
缺视角因子、块积分方式不同、单矩形）。本脚本用 `forward_paper.build_paper_A`，
即与 MATLAB `simA` 逐值抽样完全相同的那一版（相对误差 4.57e-2，全部落在遮挡边界）。

目的：把「算子选错（差分 vs 不差分）」这个大变量先定下来，暂不抠边界精度。

对照：
  x4c 结论（我们的 A）
    O    差分 + κ0.75 无上限   → top-1 (13,17,1)  K=27
    P    不差分 + κ0.75 上限10 → top-1 (13,17,2)  K=1
  真值 ll = (0.506, 0.594, 0.242)
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
from penumbrasight_model import forward as F            # noqa: E402
from penumbrasight_model import forward_paper as FP     # noqa: E402
from penumbrasight_model import io_mat                  # noqa: E402

_G = {}
TRUTH = np.array([0.506, 0.594, 0.242])


def load_raw(raw_dir, tag, name):
    with open(os.path.join(raw_dir, "raw_%s__%s.shape" % (tag, name))) as fh:
        shape = tuple(int(s) for s in fh.read().split())
    a = np.fromfile(os.path.join(raw_dir, "raw_%s__%s.f64" % (tag, name)), dtype="<f8")
    return a.reshape(shape, order="F")


def _energy_detail(G, z, kappa, kmax=None):
    G = 0.5 * (G + G.T)
    lam, V = np.linalg.eigh(G)
    lam = np.maximum(lam[::-1], 0.0)
    V = V[:, ::-1]
    s = np.sqrt(lam)
    if s.size == 0 or s[0] <= 0:
        return 0.0, 0, 0.0
    keep = np.flatnonzero(s >= kappa * s[0])
    n_passed = int(keep.size)
    if kmax is not None:
        keep = keep[:kmax]
    if keep.size == 0:
        return 0.0, n_passed, float(s[0])
    c = V[:, keep].T @ z
    return float(np.sum((c ** 2) / lam[keep])), n_passed, float(s[0])


def _init(cfg_path, raw_dir):
    d = io_mat.load_mat(cfg_path)
    _G["setup"] = F.load_setup(cfg_path)
    _G["y2d"] = np.asarray(d["meas_g"], dtype=np.float64)
    _G["occ"] = load_raw(raw_dir, "chair_va1", "occ_corner")
    _G["polys"] = FP.default_polygons(_G["setup"], _G["occ"])
    return True


def _one(args):
    ix, iy, iz, ll = args
    setup = _G["setup"]
    s2 = setup.with_occluder_ll(ll)
    A, _m = FP.build_paper_A(s2, view_angle=True, mz_reversed=True,
                             polygons=_G["polys"])
    y = _G["y2d"].reshape(-1)
    z = A.T @ y
    G = A.T @ A
    del A
    e_cap, nk, s1 = _energy_detail(G, z, 0.75, kmax=10)
    e_nc, _, _ = _energy_detail(G, z, 0.75, kmax=None)
    del G, z
    return (ix, iy, iz, e_cap, nk, e_nc, s1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-root", required=True)
    ap.add_argument("--raw-dir", required=True)
    ap.add_argument("--workers", type=int, default=0)
    ap.add_argument("--report", default=None)
    args = ap.parse_args()

    out = open(args.report, "w", encoding="utf-8") if args.report else sys.stdout
    p = lambda *a: print(*a, file=out)

    cfg = os.path.join(args.data_root, "TestPosChair", "chair_test.mat")
    _init(cfg, args.raw_dir)
    d = io_mat.load_mat(cfg)
    xv = np.asarray(d["xhatvals"], dtype=np.float64).reshape(-1)
    yv = np.asarray(d["yhatvals"], dtype=np.float64).reshape(-1)
    zv = np.asarray(d["zhatvals"], dtype=np.float64).reshape(-1)
    nx, ny, nz = xv.size, yv.size, zv.size
    p("配置 %s" % cfg)
    p("论文口径 A（va1, Mz 反向, 双多边形）；网格 %d x %d x %d = %d"
      % (nx, ny, nz, nx * ny * nz))
    p("测量用 meas_g（与 x4c 同源，便于对照）；真值 ll = %s"
      % np.array2string(TRUTH, precision=4))

    jobs = [(i, j, m, np.array([xv[i], yv[j], zv[m]]))
            for i in range(nx) for j in range(ny) for m in range(nz)]

    nw = args.workers or min(18, os.cpu_count() or 1)
    from concurrent.futures import ProcessPoolExecutor
    t0 = time.time()
    with ProcessPoolExecutor(max_workers=nw, initializer=_init,
                             initargs=(cfg, args.raw_dir)) as ex:
        rows = list(ex.map(_one, jobs, chunksize=1))
    el = time.time() - t0
    p("完成 %d 个候选，用时 %.1f s（%d 进程）" % (len(rows), el, nw))

    arr = np.array(rows, dtype=np.float64)
    idx = arr[:, :3].astype(int)

    p("\n" + "=" * 80)
    p("K（通过 κ=0.75 的成分个数）在全网格上的分布")
    p("=" * 80)
    nk = arr[:, 4].astype(int)
    vals, cnts = np.unique(nk, return_counts=True)
    for v, c in zip(vals, cnts):
        p("  K=%-3d  %4d 个候选" % (v, c))
    p("  K 的最小/中位/最大 = %d / %d / %d" % (nk.min(), int(np.median(nk)), nk.max()))

    for name, col in (("论文规则 κ0.75 + K≤10", 3), ("κ0.75 不加 K 上限", 5)):
        e = arr[:, col]
        k = int(np.argmax(e))
        ijk = tuple(int(v) for v in idx[k])
        ll = (float(xv[ijk[0]]), float(yv[ijk[1]]), float(zv[ijk[2]]))
        dist = float(np.linalg.norm(np.array(ll) - TRUTH))
        p("\n" + "=" * 80)
        p("变体： %s" % name)
        p("=" * 80)
        p("  top-1 网格序号 idx=%s" % (ijk,))
        p("  top-1 ll = (%.5f, %.5f, %.5f)" % ll)
        p("  E = %.6e   该处 K = %d   s1 = %.6e"
          % (e[k], int(arr[k, 4]), arr[k, 6]))
        p("  与真值距离 = %.5f m" % dist)
        p("  x 偏差 %.5f  y 偏差 %.5f  z 偏差 %.5f"
          % (ll[0] - TRUTH[0], ll[1] - TRUTH[1], ll[2] - TRUTH[2]))
        top = np.argsort(-e)[:6]
        p("  前 6 名：")
        for t in top:
            ti = tuple(int(v) for v in idx[t])
            p("    idx=%-12s ll=(%.5f, %.5f, %.5f)  E=%.6e  K=%d  距真值 %.5f"
              % (ti, xv[ti[0]], yv[ti[1]], zv[ti[2]], e[t], int(arr[t, 4]),
                 float(np.linalg.norm([xv[ti[0]] - TRUTH[0],
                                       yv[ti[1]] - TRUTH[1],
                                       zv[ti[2]] - TRUTH[2]]))))

    # 与 x4c 的「不差分、我们的 A」结果对照
    p("\n" + "=" * 80)
    p("对照 x4c（不差分 + 我们的 A，同测量）： top-1 为 idx=(13,17,2) ll=(0.52941, 0.65000, 0.27500)")
    p("=" * 80)
    e = arr[:, 3]
    j = np.argmax(e)
    ti = tuple(int(v) for v in idx[j])
    p("  论文口径 A 的 top-1： idx=%s ll=(%.5f, %.5f, %.5f)"
      % (ti, xv[ti[0]], yv[ti[1]], zv[ti[2]]))
    p("  换 A 之后 top-1 %s"
      % ("不变" if ti == (13, 17, 2) else "**改变了**"))

    if out is not sys.stdout:
        out.close()
        print("报告已写入 %s" % args.report)


if __name__ == "__main__":
    main()
