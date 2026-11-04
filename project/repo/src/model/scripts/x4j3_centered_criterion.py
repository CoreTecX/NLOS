# -*- coding: utf-8 -*-
"""x4j3 —— 去均值（交流分量）后的定位判据。

x4j2 的 oracle 实验给出的关键事实：
    固定场景真值扫遮挡物位置时，
      corr(A(p)@x_gt, y)       的 argmax **恰好在真值网格点**（0.0108 m，= 网格分辨率）
      最优尺度下的 LS 相对残差 的 argmin **偏了 0.098 m**
两者只差一点：Pearson 相关**减了均值**，LS 残差没有。

所以：**定位信息在图像的去均值（交流）分量里**，被直流亮场挡住了。
这也解释了论文为什么失败——`occluderposgridsearch` 直接吃原始 `meas.r(:)`，
而 A 的最大奇异方向（占 s1/s2 ≈ 5.5 倍）就是那个平滑亮场。

预登记预测：去均值后，
  - 中心化 A 的 s2/s1 应显著上升（不再近秩 1）；
  - 至少有一个判据能把 argmax 打进 ≤ 0.03 m（= 网格分辨率地板）。
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_MODEL_DIR = os.path.abspath(os.path.join(_HERE, os.pardir))
if _MODEL_DIR not in sys.path:
    sys.path.insert(0, _MODEL_DIR)
from penumbrasight_model import forward_loc as FL   # noqa: E402

REPO = os.path.abspath(os.path.join(_MODEL_DIR, os.pardir, os.pardir))
RESULTS = os.path.join(REPO, "sim", "results")
DEFAULT_MAT = (r"D:\学习\26秋嵌入式\论文数据集\Ordinary-Camera-master"
               r"\Data\TestPosD11\image_test_bur20.mat")

CRITS = ["C4_fullrank_raw", "C6_c_K1", "C7_c_s0001", "C8_c_fullrank", "C9_c_LSfit"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mat", default=DEFAULT_MAT)
    ap.add_argument("--nx", type=int, default=9)
    ap.add_argument("--ny", type=int, default=7)
    ap.add_argument("--nz", type=int, default=5)
    ap.add_argument("--xlo", type=float, default=0.33)
    ap.add_argument("--xhi", type=float, default=0.62)
    ap.add_argument("--ylo", type=float, default=0.42)
    ap.add_argument("--yhi", type=float, default=0.70)
    ap.add_argument("--zlo", type=float, default=0.13)
    ap.add_argument("--zhi", type=float, default=0.30)
    ap.add_argument("--downsample", type=int, default=4)
    ap.add_argument("--out", default=os.path.join(RESULTS, "x4j3_centered.json"))
    args = ap.parse_args()

    st = FL.d11_loc_setup()
    truth = np.asarray(st.occ_ll_true, dtype=float)
    meas, gt = FL.load_measurement(args.mat, args.downsample)
    gx = np.linspace(args.xlo, args.xhi, args.nx)
    gy = np.linspace(args.ylo, args.yhi, args.ny)
    gz = np.linspace(args.zlo, args.zhi, args.nz)
    shp = (args.nx, args.ny, args.nz)

    y_raw = meas[:, :, 0].reshape(-1, order="F")
    y_c = y_raw - y_raw.mean()

    print("=" * 76)
    print("x4j3 去均值判据")
    print("=" * 76)
    print("真值 [%.4f %.4f %.4f]   网格 %d×%d×%d" % (*truth, args.nx, args.ny, args.nz))
    bp = np.array([gx[np.argmin(abs(gx - truth[0]))], gy[np.argmin(abs(gy - truth[1]))],
                   gz[np.argmin(abs(gz - truth[2]))]])
    floor = float(np.linalg.norm(bp - truth))
    print("网格分辨率地板（最近网格点到真值）= %.6f m" % floor)
    print("y 直流占比: mean=%.4e  std=%.4e  |y_c|/|y|=%.6f"
          % (y_raw.mean(), y_raw.std(), np.linalg.norm(y_c) / np.linalg.norm(y_raw)))

    plan = FL.make_plan(st, 2, downsamp_factor=args.downsample)
    crit = {k: np.zeros(shp) for k in CRITS}
    s2s1_raw = np.zeros(shp)
    s2s1_c = np.zeros(shp)
    y2_raw = float(y_raw @ y_raw)
    y2_c = float(y_c @ y_c)

    t0 = time.time()
    n = 0
    for ix in range(args.nx):
        for iy in range(args.ny):
            for iz in range(args.nz):
                A = FL.build_A_from_plan(plan, [gx[ix], gy[iy], gz[iz]])
                Ac = A - A.mean(axis=0, keepdims=True)

                # ---- 原始（对照组）----
                Gr = A.T @ A
                Gr = 0.5 * (Gr + Gr.T)
                lr, Vr = np.linalg.eigh(Gr)
                lr = np.maximum(lr, 0.0)
                sr = np.sqrt(lr)
                s2s1_raw[ix, iy, iz] = sr[-2] / sr[-1]
                br = A.T @ y_raw
                cr = Vr.T @ br
                g2 = (cr ** 2) / np.where(lr > 0, lr, np.inf)
                crit["C4_fullrank_raw"][ix, iy, iz] = np.sqrt(g2.sum())

                # ---- 去均值 ----
                Gc = Ac.T @ Ac
                Gc = 0.5 * (Gc + Gc.T)
                lc, Vc = np.linalg.eigh(Gc)
                lc = np.maximum(lc, 0.0)
                sc = np.sqrt(lc)
                s2s1_c[ix, iy, iz] = sc[-2] / sc[-1]
                bc = Ac.T @ y_c
                cc = Vc.T @ bc
                gc2 = (cc ** 2) / np.where(lc > 0, lc, np.inf)
                nlam = len(gc2)
                cum = np.cumsum(gc2)

                for name, sg in (("C6_c_K1", 0.75), ("C7_c_s0001", 1e-3)):
                    K = int(np.sum(sc / sc[-1] > sg))
                    if K <= 0:
                        crit[name][ix, iy, iz] = 0.0
                    elif K >= nlam:
                        crit[name][ix, iy, iz] = np.sqrt(cum[-1])
                    else:
                        crit[name][ix, iy, iz] = np.sqrt(cum[-1] - cum[nlam - K - 1])

                crit["C8_c_fullrank"][ix, iy, iz] = np.sqrt(cum[-1])

                # C9：截断最小二乘拟合的相关系数（尺度无关，等价于最优尺度下的相对残差）
                Kt = int(np.sum(sc / sc[-1] > 0.02))
                if Kt > 0:
                    idx = slice(nlam - Kt, nlam)
                    xh = Vc[:, idx] @ (cc[idx] / lc[idx])
                    yp = Ac @ xh
                    if np.linalg.norm(yp) > 0:
                        crit["C9_c_LSfit"][ix, iy, iz] = float(
                            np.corrcoef(yp, y_c)[0, 1])

                n += 1
        print("  x=%d/%d  %.1f s" % (ix + 1, args.nx, time.time() - t0))

    print("\n完成，%d 个候选，用时 %.1f s\n" % (n, time.time() - t0))

    print("=" * 76)
    print("A 的谱形状：去均值前后")
    print("=" * 76)
    print("  原始   s2/s1: min=%.4f max=%.4f mean=%.4f"
          % (s2s1_raw.min(), s2s1_raw.max(), s2s1_raw.mean()))
    print("  去均值 s2/s1: min=%.4f max=%.4f mean=%.4f"
          % (s2s1_c.min(), s2s1_c.max(), s2s1_c.mean()))

    print()
    print("=" * 76)
    print("结果（成功线 ≤ %.4f m = 网格地板；边缘 ≤ 0.05 m）" % floor)
    print("=" * 76)
    hdr = "%-20s %-24s %-11s %-12s %s" % ("判据", "argmax", "到真值(m)", "top1/top2", "判定")
    print(hdr)
    print("-" * len(hdr))
    rows = []
    for name in CRITS:
        surf = crit[name]
        i, j, k = np.unravel_index(int(np.argmax(surf)), shp)
        p = np.array([gx[i], gy[j], gz[k]])
        d = float(np.linalg.norm(p - truth))
        f = np.sort(surf.ravel())[::-1]
        gap = (f[0] - f[1]) / abs(f[0]) if f[0] != 0 else float("nan")
        verdict = "失败" if d > 0.05 else ("边缘" if d > floor * 1.5 else "成功")
        print("%-20s [%.4f %.4f %.4f]   %-11.6f %-12.5f %s"
              % (name, p[0], p[1], p[2], d, gap, verdict))
        rows.append(dict(name=name, p=p.tolist(), d=d, gap=gap, verdict=verdict))

    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(dict(truth=truth.tolist(), floor=floor, rows=rows,
                       s2s1_raw_mean=float(s2s1_raw.mean()),
                       s2s1_c_mean=float(s2s1_c.mean())), f,
                  ensure_ascii=False, indent=2)
    np.savez_compressed(args.out.replace(".json", "_surf.npz"), **crit,
                        s2s1_raw=s2s1_raw, s2s1_c=s2s1_c)
    print("\n结果: %s" % args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
