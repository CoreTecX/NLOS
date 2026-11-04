# -*- coding: utf-8 -*-
"""x4k3 —— 从 x4k 的 npz 里把所有"报告要引用的数字"一次性摊平（不建 A、不重算）。

要回答的是**可分辨裕度**问题：判据要在 315 个候选里挑出真值，
它沿网格的**起伏幅度**必须大于模型误差与噪声。所以这里对每个场景算：

  `裕度 = (真值点取值 − 全网格最优取值) / 全网格最优取值`    （0 = 真值就是最优）

对比三族的裕度：oracle（已知场景真值）、NNLS 插件（估计场景）、方案口径子空间能量。
若 oracle 的裕度都接近 0 甚至为负，就说明**模型+测量这一对本身不含可用的位置信息**，
再换估计器也没用 —— 这正是 bu 与 mushroom 给出的分岔。
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_MODEL_DIR = os.path.abspath(os.path.join(_HERE, os.pardir))
REPO = os.path.abspath(os.path.join(_MODEL_DIR, os.pardir, os.pardir))
RESULTS = os.path.join(REPO, "sim", "results")

SCENES = ["bu", "mushroom"]


def _stat(name, surf, ti, minimize):
    f = surf.ravel()
    best = float(f.min() if minimize else f.max())
    tv = float(surf[ti])
    # 真值点相对全网格最优的"落后量"（负值 = 真值就是最优）
    lag = (tv - best) / abs(best) if best else float("nan")
    return name, best, tv, lag


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenes", default=",".join(SCENES))
    ap.add_argument("--prefix", default="x4k_diff",
                    help="npz 前缀；X4l 复核用 x4k_nomirror")
    args = ap.parse_args()

    for sc in args.scenes.split(","):
        p = os.path.join(RESULTS, "%s_%s_surf.npz" % (args.prefix, sc))
        if not os.path.exists(p):
            print("缺文件，跳过：%s" % p)
            continue
        d = np.load(p, allow_pickle=False)
        gx, gy, gz = d["gx"], d["gy"], d["gz"]
        truth = d["truth"]
        ti = tuple(int(np.argmin(abs(g - t))) for g, t in zip((gx, gy, gz), truth))
        floor = float(np.linalg.norm(
            np.array([gx[ti[0]], gy[ti[1]], gz[ti[2]]]) - truth))

        print("=" * 88)
        print("场景 %-9s  真值 [%.4f %.4f %.4f]   物理地板 %.6f m"
              % (sc, truth[0], truth[1], truth[2], floor))
        print("=" * 88)

        print("\n[1] 差分吃掉了多少能量（越小 = 直流亮场去得越干净）")
        for c, nm in enumerate("rgb"):
            print("    %s 通道  ||D_y y|| / ||y|| = %.6f" %
                  (nm, float(np.sqrt(d["dnorm2"][c] / d["ynorm2"][c]))))

        print("\n[2] 能量与裕度：三族判据在 315 个候选上的表现")
        print("    %-34s %-14s %-14s %-12s" % ("判据", "全网格最优", "真值点取值", "真值落后量"))
        print("    " + "-" * 76)
        rows = []
        rows.append(_stat("O1 oracle 相关性  未差分", d["or_corr_nodiff"], ti, False))
        rows.append(_stat("O2 oracle 相关性  差分后", d["or_corr_diff"], ti, False))
        rows.append(_stat("O3 oracle 相对残差 未差分", d["or_relres_nodiff"], ti, True))
        rows.append(_stat("O4 oracle 相对残差 差分后", d["or_relres_diff"], ti, True))
        rows.append(_stat("T1g NNLS 相对残差 差分后", d["surf_T1g_diff_nnls"], ti, True))
        for (nm, best, tv, lag) in rows:
            print("    %-34s %-14.6f %-14.6f %+9.2f%%" % (nm, best, tv, 100 * lag))

        print("\n    方案口径子空间能量（最大化为好）")
        for key, lab in (("surf_T1a_diff_K8", "T1a 垂直差分 K=8"),
                         ("surf_T1d_diff_K1", "T1d 垂直差分 K=1"),
                         ("surf_T1f_diffx_K8", "T1f 水平差分 K=8"),
                         ("surf_B0_paper_K1", "B0 论文原口径 K=1")):
            s = d[key].sum(axis=-1)
            nm, best, tv, lag = _stat(lab, s, ti, False)
            print("    %-34s %-14.4g %-14.4g %+9.2f%%" % (nm, best, tv, 100 * lag))

        print("\n[3] 差分后 B 的谱（真值最近网格点；A 作对照）")
        spec_lam_none = d["lamv"][0][ti]
        lam_z = d["lamv"][1][ti]
        s_none = np.sqrt(np.maximum(spec_lam_none, 0))
        s_z = np.sqrt(np.maximum(lam_z, 0))
        print("    未差分的 s_k/s_1 前 8: %s" % np.array2string(
            s_none[:8] / s_none[0], precision=4))
        print("    差分后 的 s_k/s_1 前 8: %s" % np.array2string(
            s_z[:8] / s_z[0], precision=4))
        print("    σ=0.75 规则的 K：未差分 %d ；差分 %d"
              % (int((s_none / s_none[0] > 0.75).sum()),
                 int((s_z / s_z[0] > 0.75).sum())))

        # 全网格上 K(σ) 的范围
        sz = np.sqrt(np.maximum(d["lamv"][1], 0))
        kil = (sz / np.maximum(sz[..., :1], 1e-300) > 0.75).sum(axis=-1)
        print("    差分后 K(σ=0.75) 在 315 个候选上的范围：%d ~ %d（中位 %.1f）"
              % (kil.min(), kil.max(), float(np.median(kil))))
        print()


if __name__ == "__main__":
    sys.exit(main())
