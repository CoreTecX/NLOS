# -*- coding: utf-8 -*-
"""x4k4 —— T1 的四联图（只读 x4k 的 npz，不建 A）。

四格要讲的一件事：**差分把"平坦且由噪声决定"换成了"很有把握地选错"。**

  (a) 谱：未差分的 A 是近秩 1（s₂/s₁ = 0.185），差分后成一片高台
      （前 8 个都 > 0.84，σ=0.75 的 K 从 1 涨到 13）→ 数值退化确实被打破了
  (b) K 扫描：换个 K 会不会就好了 → 不会，argmax 在 0.05~0.21 m 之间乱跳
  (c) 真值落后量：各判据的"真值点比全网格最优差多少"
      —— 论文原口径只差 0.2%（平到没信息），方案口径差 42~44%（有把握地选错）
  (d) 方向消融：垂直 vs 水平，两个场景谁赢是反的 → 方向优势是噪声
"""

from __future__ import annotations

import os
import sys

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt                      # noqa: E402

CJK = ["Noto Sans CJK SC", "WenQuanYi Micro Hei", "Microsoft YaHei",
       "SimHei", "Source Han Sans SC", "DejaVu Sans"]

_HERE = os.path.dirname(os.path.abspath(__file__))
_MODEL_DIR = os.path.abspath(os.path.join(_HERE, os.pardir))
REPO = os.path.abspath(os.path.join(_MODEL_DIR, os.pardir, os.pardir))
RESULTS = os.path.join(REPO, "sim", "results")

SCENES = ["bu", "mushroom"]
LAB = {"bu": "'BU' 字母（只有 r 通道有内容）", "mushroom": "蘑菇（三通道都有内容）"}
KS = [1, 2, 3, 4, 5, 6, 8, 10, 13, 16, 20, 24, 32, 48, 64]


def _setup_font():
    from matplotlib import font_manager
    have = {f.name for f in font_manager.fontManager.ttflist}
    for c in CJK:
        if c in have:
            matplotlib.rcParams["font.sans-serif"] = [c] + CJK
            break
    else:
        matplotlib.rcParams["font.sans-serif"] = CJK
    matplotlib.rcParams["axes.unicode_minus"] = False


def sub_energy(proj, lam, K, weight):
    p = proj[..., :K]
    l = lam[..., :K][..., None, :]
    if weight == "inv":
        return (p ** 2 / np.maximum(l, 1e-300)).sum(axis=-1)
    if weight == "one":
        return (p ** 2).sum(axis=-1)
    return (p ** 2 * l).sum(axis=-1)


def main():
    _setup_font()
    D = {}
    for sc in SCENES:
        D[sc] = dict(np.load(os.path.join(RESULTS, "x4k_diff_%s_surf.npz" % sc),
                             allow_pickle=False))
    fig, ax = plt.subplots(2, 2, figsize=(14.4, 9.6))
    colors = {"bu": "#c0392b", "mushroom": "#2471a3"}

    # ---------------- (a) 谱 ----------------
    a = ax[0, 0]
    d = D["bu"]
    ti = tuple(int(np.argmin(abs(g - t))) for g, t in zip(
        (d["gx"], d["gy"], d["gz"]), d["truth"]))
    n = 20
    for oi, (lab, c, ls) in enumerate((("未差分 A（论文原口径）", "#7f8c8d", "-"),
                                       ("差分后 B = D_y A（方案口径）", "#c0392b", "-"))):
        s = np.sqrt(np.maximum(d["lamv"][oi][ti], 0))
        a.plot(range(1, n + 1), s[:n] / s[0], ls, color=c, lw=2, marker="o", ms=3, label=lab)
    a.axhline(0.75, color="k", ls="--", lw=1.2)
    a.text(n - 0.4, 0.77, "σ = 0.75 阈值", ha="right", fontsize=9)
    a.annotate("K ≡ 1", xy=(2, 0.185), xytext=(5.2, 0.34), fontsize=10, color="#7f8c8d",
               arrowprops=dict(arrowstyle="->", color="#7f8c8d"))
    a.annotate("K = 13（一片高台）", xy=(13, 0.79), xytext=(8.0, 0.55), fontsize=10,
               color="#c0392b", arrowprops=dict(arrowstyle="->", color="#c0392b"))
    a.set_xlabel("奇异值序号 k")
    a.set_ylabel("s$_k$ / s$_1$")
    a.set_title("(a) 差分把「近秩 1」换成了「高台」\n数值退化确实被打破（真值位置处）", fontsize=11)
    a.set_ylim(0, 1.05)
    a.grid(alpha=0.3)
    a.legend(fontsize=9, loc="center right")

    # ---------------- (b) K 扫描 ----------------
    b = ax[0, 1]
    for sc in SCENES:
        d = D[sc]
        ti = tuple(int(np.argmin(abs(g - t))) for g, t in zip(
            (d["gx"], d["gy"], d["gz"]), d["truth"]))
        floor = float(np.linalg.norm(np.array(
            [d["gx"][ti[0]], d["gy"][ti[1]], d["gz"][ti[2]]]) - d["truth"]))
        ds_ = []
        for K in KS:
            surf = sub_energy(d["proj"][1], d["lamv"][1], K, "inv").sum(axis=-1)
            i = np.unravel_index(int(np.argmax(surf)), surf.shape)
            p = np.array([d["gx"][i[0]], d["gy"][i[1]], d["gz"][i[2]]])
            ds_.append(float(np.linalg.norm(p - d["truth"])))
        b.plot(KS, ds_, color=colors[sc], lw=2, marker="o", ms=4, label=LAB[sc])
        b.axhline(floor, color=colors[sc], ls=":", lw=1)
    b.axhline(0.03, color="k", ls="--", lw=1.2)
    b.text(64, 0.034, "成功线 0.03 m", fontsize=9, ha="right")
    b.axhline(0.0157, color="green", ls=":", lw=1.5)
    b.text(64, 0.0088, "物理地板 0.0157 m（网格分辨率）", fontsize=8.5, color="green",
           ha="right")
    b.axvline(8, color="k", ls="-.", lw=1)
    b.text(8.6, 0.19, "方案冻结值 K = 8", fontsize=9)
    b.set_xscale("log")
    b.set_xticks([1, 2, 3, 4, 5, 6, 8, 10, 13, 16, 20, 24, 32, 48, 64])
    b.get_xaxis().set_major_formatter(matplotlib.ticker.ScalarFormatter())
    b.set_xlabel("保留阶数 K")
    b.set_ylabel("argmax 到真值的距离（m）")
    b.set_title("(b) 扫遍 K = 1…64（垂直差分、1/λ 加权）：\n没有一个 K 落到地板上，argmax 在 0.05~0.21 m 之间乱跳", fontsize=11)
    b.grid(alpha=0.3)
    b.legend(fontsize=8.5, loc="upper left")

    # ---------------- (c) 真值落后量 ----------------
    c_ax = ax[1, 0]
    keys = [("surf_B0_paper_K1", "论文原口径\nK=1", False, "sum"),
            ("surf_T1a_diff_K8", "方案口径\nD_y, K=8", False, "sum"),
            ("or_relres_diff", "oracle 残差\n（已知场景真值）", True, None),
            ("surf_T1g_diff_nnls", "NNLS 插件残差\n（估计场景）", True, None)]
    x = np.arange(len(keys))
    w = 0.36
    for si, sc in enumerate(SCENES):
        d = D[sc]
        ti = tuple(int(np.argmin(abs(g - t))) for g, t in zip(
            (d["gx"], d["gy"], d["gz"]), d["truth"]))
        vals = []
        for k, _, minimize, mode in keys:
            if mode == "sum":
                s = d[k].sum(axis=-1)
                vals.append(100 * (float(s[ti]) - float(s.max())) / abs(float(s.max())))
            else:
                s = d[k]
                vals.append(100 * (float(s[ti]) - float(s.min())) / abs(float(s.min())))
        c_ax.bar(x + (si - 0.5) * w, vals, w, color=colors[sc], label=LAB[sc])
    c_ax.axhline(0, color="k", lw=1)
    for xx in x[1:]:
        c_ax.axvline(xx - 0.5, color="0.85", lw=1)
    c_ax.set_xticks(x)
    c_ax.set_xticklabels([k[1] for k in keys], fontsize=9)
    c_ax.set_ylabel("真值点取值相对全网格最优的落后量（%）\n← 0 = 真值就是最优；正值 = 选到了别处")
    c_ax.set_title("(c) 真值落后量：论文原口径只差 0.2%（平到没信息）；\n方案口径差 42~44%（很有把握地选错）", fontsize=11)
    c_ax.grid(alpha=0.3, axis="y")
    c_ax.legend(fontsize=9)

    # ---------------- (d) 方向消融 ----------------
    d_ax = ax[1, 1]
    ops = ["none", "z", "x"]
    oplab = ["不差分", "垂直 D$_y$（方案指定）", "水平 D$_x$"]
    x = np.arange(len(ops))
    for si, sc in enumerate(SCENES):
        d = D[sc]
        vals = []
        for oi in range(3):
            surf = sub_energy(d["proj"][oi], d["lamv"][oi], 8, "inv").sum(axis=-1)
            i = np.unravel_index(int(np.argmax(surf)), surf.shape)
            p = np.array([d["gx"][i[0]], d["gy"][i[1]], d["gz"][i[2]]])
            vals.append(float(np.linalg.norm(p - d["truth"])))
        d_ax.bar(x + (si - 0.5) * w, vals, w, color=colors[sc], label=LAB[sc])
        for xx, v in zip(x + (si - 0.5) * w, vals):
            d_ax.text(xx, v + 0.004, "%.3f" % v, ha="center", fontsize=8.5)
    d_ax.axhline(0.03, color="k", ls="--", lw=1.2)
    d_ax.text(2.42, 0.032, "成功线 0.03 m", fontsize=9, ha="right")
    d_ax.axhline(0.0157, color="green", ls=":", lw=1.5)
    d_ax.text(2.42, 0.006, "物理地板 0.0157 m", fontsize=8.5, color="green", ha="right")
    d_ax.set_xticks(x)
    d_ax.set_xticklabels(oplab, fontsize=9.5)
    d_ax.set_ylabel("argmax 到真值的距离（m）")
    d_ax.set_ylim(0, 0.24)
    d_ax.set_title("(d) 方向消融（K=8）：垂直在 mushroom 上赢、在 bu 上输，\n方向优势随场景反过来 → 是噪声不是结构", fontsize=11)
    d_ax.grid(alpha=0.3, axis="y")
    d_ax.legend(fontsize=9)

    fig.suptitle("T1 定位差分重测 —— 差分这个「装置」对了，方案的具体口径仍然不定位",
                 fontsize=14, y=0.995)
    fig.tight_layout(rect=(0, 0, 1, 0.975))
    out = os.path.join(RESULTS, "x4k_diff_localization.png")
    fig.savefig(out, dpi=155)
    print("图: %s" % out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
