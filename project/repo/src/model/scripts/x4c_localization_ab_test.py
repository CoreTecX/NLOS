"""X4c —— 定量复现：论文的定位口径 vs 我们的口径

背景（X4 第二步）：论文 `Functions/occluderposgridsearch.m` 的定位是

    [U,S,~] = svd(simA,'econ');  vecS = diag(S)/S(1);
    K = min(sum(vecS>sigma_th), 10);
    proj = norm(U(:,1:K)'*meas.r(:));

即 **A 本身**（不做任何差分）+ 归一化奇异值阈值 + **K 上限 10** + 逐 RGB 通道再平均。
而我们当前的 `localization.evaluate_candidate` 默认 `differenced=True, kappa=0.75`，且
`projection_energy_detail` **没有 K 上限**。

本脚本在**同一份数据**（chair_test.mat 的 meas_g）与**同一个官方第 1 轮网格**（972 个候选）
上，跑四个变体，把「差分方向」与「K 上限」这两个变量分开量：

    O     差分 + κ=0.75，不设上限      ← 我们当前口径
    P     不差分 + κ=0.75，上限 10     ← 论文口径
    P_nc  不差分 + κ=0.75，不设上限    ← 只换算子，不换截断
    O_c   差分 + κ=0.75，上限 10       ← 只换截断，不换算子

看 top-1 是否改变、能量体之间是否同序，就能判断这两个变量各自的影响。
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
from penumbrasight_model import forward as F          # noqa: E402
from penumbrasight_model import io_mat, localization as L  # noqa: E402

_G = {}


def _init(cfg_path):
    d = io_mat.load_mat(cfg_path)
    _G["setup"] = F.load_setup(cfg_path)
    _G["y2d"] = np.asarray(d["meas_g"], dtype=np.float64)
    _G["yd"] = L.apply_row_difference_img(_G["y2d"])
    return _G["y2d"].shape


def _energy_detail(G, z, kappa, kmax=None):
    """返回 (能量, 保留成分数, s1)。保留判据在奇异值上，并按需截到前 kmax 个。"""
    G = 0.5 * (G + G.T)
    lam, V = np.linalg.eigh(G)
    lam = np.maximum(lam[::-1], 0.0)
    V = V[:, ::-1]
    s = np.sqrt(lam)
    if s.size == 0 or s[0] <= 0:
        return 0.0, 0, 0.0
    keep = np.flatnonzero(s >= kappa * s[0])          # 已按降序
    if kmax is not None:
        keep = keep[:kmax]
    if keep.size == 0:
        return 0.0, 0, float(s[0])
    c = V[:, keep].T @ z
    e = float(np.sum((c ** 2) / lam[keep]))
    return e, int(keep.size), float(s[0])


def _one(args):
    ix, iy, iz, ll = args
    setup = _G["setup"]
    s2 = setup.with_occluder_ll(ll)
    A = F.build_transport_matrix(s2)
    n = setup.wall.n

    # 不差分（论文口径）
    Ad = L.apply_row_difference_A(A, n)
    y_nd = _G["y2d"].reshape(-1)
    z_nd = A.T @ y_nd
    G_nd = A.T @ A
    del A
    e_P, k_P, _ = _energy_detail(G_nd, z_nd, 0.75, kmax=10)
    e_Pnc, k_Pnc, _ = _energy_detail(G_nd, z_nd, 0.75, kmax=None)
    del G_nd, z_nd

    # 差分（我们当前口径）
    y_d = _G["yd"]
    z_d = Ad.T @ y_d
    G_d = Ad.T @ Ad
    del Ad
    e_O, k_O, _ = _energy_detail(G_d, z_d, 0.75, kmax=None)
    e_Oc, k_Oc, _ = _energy_detail(G_d, z_d, 0.75, kmax=10)
    del G_d, z_d

    return (ix, iy, iz, e_O, k_O, e_P, k_P, e_Pnc, k_Pnc, e_Oc, k_Oc)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-root", required=True)
    ap.add_argument("--workers", type=int, default=0)
    ap.add_argument("--report", default=None)
    args = ap.parse_args()

    cfg = os.path.join(args.data_root, "TestPosChair", "chair_test.mat")
    shp = _init(cfg)
    d = io_mat.load_mat(cfg)
    xv = np.asarray(d["xhatvals"], dtype=np.float64).reshape(-1)
    yv = np.asarray(d["yhatvals"], dtype=np.float64).reshape(-1)
    zv = np.asarray(d["zhatvals"], dtype=np.float64).reshape(-1)
    nx, ny, nz = xv.size, yv.size, zv.size
    print("配置 %s" % cfg)
    print("meas_g 形状 %s   第 1 轮网格 %d x %d x %d = %d"
          % (shp, nx, ny, nz, nx * ny * nz))

    jobs = [(i, j, m, np.array([xv[i], yv[j], zv[m]]))
            for i in range(nx) for j in range(ny) for m in range(nz)]

    nw = args.workers or min(18, os.cpu_count() or 1)
    t0 = time.time()
    from concurrent.futures import ProcessPoolExecutor
    with ProcessPoolExecutor(max_workers=nw, initializer=_init,
                             initargs=(cfg,)) as ex:
        rows = list(ex.map(_one, jobs, chunksize=4))
    el = time.time() - t0
    print("完成 %d 个候选，用时 %.1f s（%d 进程）" % (len(rows), el, nw))

    arr = np.array([[r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8], r[9], r[10]]
                    for r in rows], dtype=np.float64)
    out = open(args.report, "w", encoding="utf-8") if args.report else sys.stdout
    p = lambda *a: print(*a, file=out)

    variants = [("O    差分+κ.75,无上限（我们当前）", 3), ("P    不差分+κ.75,上限10（论文）", 5),
                ("P_nc 不差分+κ.75,无上限", 7), ("O_c  差分+κ.75,上限10", 9)]
    ks = {"O": 4, "P": 6, "P_nc": 8, "O_c": 10}

    p("\n" + "=" * 78)
    p("四变体的 top-1（网格序号，与 x10_grid.bin 同序：k = ix*(ny*nz)+iy*nz+iz）")
    p("=" * 78)
    results = {}
    for name, col in variants:
        e = arr[:, col]
        k = int(np.argmax(e))
        ix, iy, iz = int(arr[k, 0]), int(arr[k, 1]), int(arr[k, 2])
        ll = (xv[ix], yv[iy], zv[iz])
        kk = int(arr[k, ks[name.split()[0]]])
        results[name.split()[0]] = (ix, iy, iz, ll, e[k], kk, e)
        p("  %-34s idx=(%2d,%2d,%2d)  ll=(%.5f, %.5f, %.5f)  E=%.6e  保留K=%d"
          % (name, ix, iy, iz, ll[0], ll[1], ll[2], e[k], kk))

    p("\n" + "=" * 78)
    p("变体之间的能量体相关性（判断是否只是单调变换）")
    p("=" * 78)
    names = ["O", "P", "P_nc", "O_c"]
    for a in range(len(names)):
        for b in range(a + 1, len(names)):
            ea, eb = results[names[a]][6], results[names[b]][6]
            c = float(np.corrcoef(ea, eb)[0, 1])
            p("  %-6s vs %-6s  相关系数 %+.6f   Spearman 需另算"
              % (names[a], names[b], c))

    # Spearman（用秩相关，避免依赖 scipy）
    def rank(v):
        o = np.argsort(np.argsort(v))
        return o.astype(np.float64)
    p("\n  秩相关（Spearman）：")
    for a in range(len(names)):
        for b in range(a + 1, len(names)):
            ea, eb = results[names[a]][6], results[names[b]][6]
            c = float(np.corrcoef(rank(ea), rank(eb))[0, 1])
            p("  %-6s vs %-6s  %.6f" % (names[a], names[b], c))

    p("\n" + "=" * 78)
    p("结论")
    p("=" * 78)
    same_nodiff = results["O"][:3] == results["P"][:3]
    p("  换算子（差分→不差分）且换截断后，top-1 %s"
      % ("不变" if same_nodiff else "**改变了**"))
    p("  只换算子（O vs P_nc）：top-1 %s"
      % ("不变" if results["O"][:3] == results["P_nc"][:3] else "改变"))
    p("  只换截断（O vs O_c）：top-1 %s"
      % ("不变" if results["O"][:3] == results["O_c"][:3] else "改变"))

    if out is not sys.stdout:
        out.close()
        print("报告已写入 %s" % args.report)


if __name__ == "__main__":
    main()
