"""X4g —— 用「定位档」参数重跑论文的全局轮

修正 X4e 用错档的问题（X4f）：
  分辨率   downsamp_fact=4 → numPixels = floor(1008/16) = 63  → A 为 3969×1044
  块积分   Ndiscr_mon = 2（全局轮）
  网格     xhatvals = linspace(0.1, 0.6, 5)  共 125 个候选
           yhatvals = linspace(0.1, 0.6, 5)
           zhatvals = linspace(0.0, 0.3, 5)
  规则     κ = sigma_th1 = 0.75，K ≤ 10

真值（重建口径）ll = (0.506, 0.594, 0.242)。全局网格上的最近点应为
  x: 0.475（0.1..0.6 步长 0.125）  y: 0.600   z: 0.225
即 idx 应为 (3, 4, 3)。这是本轮的预登记预测。

测量按同一 factor 降采样：meas_g 126 → 63（2×2 块平均；load_image1 的确切口径待核）。
"""
from __future__ import annotations

import argparse
import dataclasses
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
PRED_IDX = (3, 4, 3)      # 网格上离真值最近的序号


def load_raw(raw_dir, tag, name):
    with open(os.path.join(raw_dir, "raw_%s__%s.shape" % (tag, name))) as fh:
        shape = tuple(int(s) for s in fh.read().split())
    a = np.fromfile(os.path.join(raw_dir, "raw_%s__%s.f64" % (tag, name)), dtype="<f8")
    return a.reshape(shape, order="F")


def downsample2(a):
    """126 → 63：2×2 块平均。与 load_image1 的确切口径待核，先用最朴素的做法。"""
    h, w = a.shape
    return a.reshape(h // 2, 2, w // 2, 2).mean(axis=(1, 3))


def _energy_detail(G, z, kappa, kmax=None):
    G = 0.5 * (G + G.T)
    lam, V = np.linalg.eigh(G)
    lam = np.maximum(lam[::-1], 0.0)
    V = V[:, ::-1]
    s = np.sqrt(lam)
    if s.size == 0 or s[0] <= 0:
        return 0.0, 0, 0.0
    keep = np.flatnonzero(s >= kappa * s[0])
    n_pass = int(keep.size)
    if kmax is not None:
        keep = keep[:kmax]
    if keep.size == 0:
        return 0.0, n_pass, float(s[0])
    c = V[:, keep].T @ z
    return float(np.sum((c ** 2) / lam[keep])), n_pass, float(s[0])


def _init(cfg_path, raw_dir, nw, ndiscr):
    d = io_mat.load_mat(cfg_path)
    setup = F.load_setup(cfg_path)
    _G["setup"] = dataclasses.replace(setup, wall=dataclasses.replace(setup.wall, n=nw))
    _G["y"] = downsample2(np.asarray(d["meas_g"], dtype=np.float64)).reshape(-1)
    _G["polys"] = FP.default_polygons(setup, load_raw(raw_dir, "chair_va1", "occ_corner"))
    _G["ndiscr"] = ndiscr
    return True


def _one(args):
    i, j, m, ll = args
    s2 = _G["setup"].with_occluder_ll(ll)
    A, _ = FP.build_paper_A(s2, ndiscr=_G["ndiscr"], view_angle=True,
                            mz_reversed=True, polygons=_G["polys"])
    z = A.T @ _G["y"]
    G = A.T @ A
    del A
    e, nk, s1 = _energy_detail(G, z, 0.75, kmax=10)
    del G, z
    return (i, j, m, e, nk, s1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-root", required=True)
    ap.add_argument("--raw-dir", required=True)
    ap.add_argument("--num-pixels", type=int, default=63)
    ap.add_argument("--ndiscr", type=int, default=2)
    ap.add_argument("--workers", type=int, default=0)
    ap.add_argument("--report", default=None)
    args = ap.parse_args()

    out = open(args.report, "w", encoding="utf-8") if args.report else sys.stdout
    p = lambda *a: print(*a, file=out)

    cfg = os.path.join(args.data_root, "TestPosChair", "chair_test.mat")
    _init(cfg, args.raw_dir, args.num_pixels, args.ndiscr)
    xv = np.linspace(0.1, 0.6, 5)
    yv = np.linspace(0.1, 0.6, 5)
    zv = np.linspace(0.0, 0.3, 5)
    p("定位档参数： numPixels=%d（A 为 %d x %d）、Ndiscr_mon=%d、网格 5x5x5=125"
      % (args.num_pixels, args.num_pixels ** 2, 5 * 5 * 5, args.ndiscr))
    p("测量： meas_g 降采样 126 -> %d" % args.num_pixels)
    p("预登记预测： 最近点 idx=%s  ll=(%.3f, %.3f, %.3f)"
      % (PRED_IDX, xv[PRED_IDX[0]], yv[PRED_IDX[1]], zv[PRED_IDX[2]]))

    jobs = [(i, j, m, np.array([xv[i], yv[j], zv[m]]))
            for i in range(5) for j in range(5) for m in range(5)]

    nw = args.workers or min(18, os.cpu_count() or 1)
    from concurrent.futures import ProcessPoolExecutor
    t0 = time.time()
    with ProcessPoolExecutor(max_workers=nw, initializer=_init,
                             initargs=(cfg, args.raw_dir, args.num_pixels,
                                       args.ndiscr)) as ex:
        rows = list(ex.map(_one, jobs, chunksize=1))
    p("完成 %d 个候选，用时 %.1f s" % (len(rows), time.time() - t0))

    arr = np.array(rows, dtype=np.float64)
    nk = arr[:, 4].astype(int)
    vals, cnts = np.unique(nk, return_counts=True)
    p("\nK 分布： " + "  ".join("K=%d:%d个" % (v, c) for v, c in zip(vals, cnts)))
    p("K 最小/中位/最大 = %d / %d / %d" % (nk.min(), int(np.median(nk)), nk.max()))

    e = arr[:, 3]
    order = np.argsort(-e)
    p("\n能量前 8 名（ll 为搜索坐标；报告坐标 y' = D - y）：")
    for t in order[:8]:
        i, j, m = int(arr[t, 0]), int(arr[t, 1]), int(arr[t, 2])
        p("  idx=(%d,%d,%d) ll=(%.3f, %.3f, %.3f)  y'=%.3f  E=%.6e  K=%d"
          % (i, j, m, xv[i], yv[j], zv[m], 1.03 - yv[j], e[t], int(arr[t, 4])))

    k = int(order[0])
    ti = (int(arr[k, 0]), int(arr[k, 1]), int(arr[k, 2]))
    p("\ntop-1 idx=%s；预登记预测 idx=%s → %s"
      % (ti, PRED_IDX, "**命中**" if ti == PRED_IDX else "**未命中**"))

    # 唯一值个数：判断能量是否有区分度（X4e 时前六名逐位相同）
    uniq = np.unique(np.round(e, 6)).size
    p("能量取值（留 6 位）的互异个数： %d / %d %s"
      % (uniq, e.size, "← 退化，说明仍无区分度" if uniq < e.size // 2 else ""))

    # 真值最近点的排名
    tgt = PRED_IDX
    mask = ((arr[:, 0] == tgt[0]) & (arr[:, 1] == tgt[1]) & (arr[:, 2] == tgt[2]))
    rank = int((e > e[mask][0]).sum()) + 1
    p("真值最近点 idx=%s 的排名： 第 %d / 125，E=%.6e" % (tgt, rank, e[mask][0]))

    if out is not sys.stdout:
        out.close()
        print("报告已写入 %s" % args.report)


if __name__ == "__main__":
    main()
