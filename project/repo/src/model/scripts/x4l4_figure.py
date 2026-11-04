# -*- coding: utf-8 -*-
"""x4l4 —— 开关消融四联图：忠实模型 vs 去掉 z 镜像。

只读两组 x4k 的 npz：
    x4k_diff_<scene>_surf.npz        忠实原文（mz_mirrored=True）
    x4k_nomirror_<scene>_surf.npz    去掉 z 镜像（mz_mirrored=False）

布局刻意做成 **每格只有 2 组 × 2 柱**（组 = 场景），这样横轴标签只有
"bu" 与 "mushroom" 两个，不会像挤 8 根柱时那样糊在一起。

前两格用「真值落后量」而不是原始值画柱：0% 就是"真值即最优"，
轴不用截断，柱高直接可读，也不会因为截断柱轴而夸张差异。
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
SLAB = {"bu": "bu", "mushroom": "mushroom"}
PFX = {"base": "x4k_diff", "nomirror": "x4k_nomirror"}
KLAB = {"base": "忠实原文（z 镜像保留）", "nomirror": "去掉 z 镜像"}
CLR = {"base": "#8a9099", "nomirror": "#c0392b"}


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


def _ti(d):
    return tuple(int(np.argmin(abs(g - t))) for g, t in zip(
        (d["gx"], d["gy"], d["gz"]), d["truth"]))


def _deficit(d, key, minimize):
    """真值点相对全网格最优的落后量（%），**恒为非负**。0 = 真值即最优。

    取最大值的量（相关性）与取最小值的量（残差）方向相反，这里统一成
    「真值比最优差多少」，否则会画出负柱、符号也读不出来。
    """
    s = d[key]
    if s.ndim == 4:
        s = s.sum(axis=-1)
    tv = float(s[_ti(d)])
    best = float(s.min() if minimize else s.max())
    if best == 0:
        return float("nan")
    return ((tv - best) if minimize else (best - tv)) / abs(best) * 100.0


def _pair(d, key, minimize):
    """(真值点取值, 全网格最优取值)，用于柱顶第二个标签。"""
    s = d[key]
    if s.ndim == 4:
        s = s.sum(axis=-1)
    return float(s[_ti(d)]), float(s.min() if minimize else s.max())


def _argmax_dist(d, key):
    s = d[key]
    if s.ndim == 4:
        s = s.sum(axis=-1)
    idx = np.unravel_index(int(np.argmax(s)), s.shape)
    p = np.array([d["gx"][idx[0]], d["gy"][idx[1]], d["gz"][idx[2]]])
    return float(np.linalg.norm(p - d["truth"]))


def _two_group(ax, val_of, lab_of, fmt="%.3f", ylabel="", title="", ylim=None,
               hlines=()):
    """2 组（bu / mushroom）× 2 柱（忠实 / 去镜像）的标准画法。"""
    x = np.arange(len(SCENES))
    w = 0.34
    if ylim:
        ax.set_ylim(*ylim)                      # 先定轴，柱顶标注的偏移才准
    off = 0.016 * (ax.get_ylim()[1] - ax.get_ylim()[0])
    for si, k in enumerate(("base", "nomirror")):
        vals = [val_of(sc, k) for sc in SCENES]
        xx = x + (si - 0.5) * w
        ax.bar(xx, vals, w, color=CLR[k], label=KLAB[k])
        for p, v, sc in zip(xx, vals, SCENES):
            ax.text(p, v + off, lab_of(sc, k, v, fmt), ha="center", fontsize=8.5)
    for y, lab, col, ls in hlines:
        ax.axhline(y, color=col, ls=ls, lw=1.3)
        # 标签放左侧空白处：柱子都在 x≈-0.17 / 0.17 / 0.83 / 1.17，
        # 右侧放会被 mushroom 的柱盖住
        ax.text(-0.55, y, lab, fontsize=8.5, color=col, ha="left", va="bottom")
    ax.set_xticks(x)
    ax.set_xticklabels([SLAB[s] for s in SCENES], fontsize=10)
    ax.set_xlim(-0.6, len(SCENES) - 0.4)
    ax.set_ylabel(ylabel)
    ax.set_title(title, fontsize=11)
    ax.grid(alpha=0.3, axis="y")


def main():
    _setup_font()
    D = {}
    for sc in SCENES:
        for k, p in PFX.items():
            D[(sc, k)] = dict(np.load(
                os.path.join(RESULTS, "%s_%s_surf.npz" % (p, sc)), allow_pickle=False))

    fig, ax = plt.subplots(2, 2, figsize=(12.8, 9.4))

    def _lab_deficit(key, minimize):
        """柱顶：落后量 + 「真值取值 → 全网格最优取值」。"""
        def f(sc, k, v, fmt):
            tv, best = _pair(D[(sc, k)], key, minimize)
            same = abs(tv - best) <= 1e-9 * max(1.0, abs(best))
            tail = "真值即最优" if same else "%.4f → %.4f" % (tv, best)
            return "%.2f%%\n%s" % (v, tail)
        return f

    # ---------- (a) 未差分 oracle 相关性：真值落后量 ----------
    a = ax[0, 0]
    _two_group(a, lambda sc, k: _deficit(D[(sc, k)], "or_corr_nodiff", False),
               _lab_deficit("or_corr_nodiff", False),
               ylabel="真值点落后全网格最优（%）",
               title="(a) 未差分域：oracle 相关性（已知场景真值）\n柱顶小字 = 真值取值 → 全网格最优",
               ylim=(-0.001, 0.42))
    a.axhline(0, color="k", lw=1)
    a.legend(fontsize=9, loc="upper left")

    # ---------- (b) 差分后 oracle 相对残差：真值落后量 ----------
    b = ax[0, 1]
    _two_group(b, lambda sc, k: _deficit(D[(sc, k)], "or_relres_diff", True),
               _lab_deficit("or_relres_diff", True),
               ylabel="真值点落后全网格最优（%）",
               title="(b) 差分域：oracle 相对残差（已知场景真值）\n柱顶小字 = 真值取值 → 全网格最优",
               ylim=(0, 23))
    b.legend(fontsize=9, loc="upper left")

    # ---------- (c) 方案口径 argmax 距离 ----------
    c = ax[1, 0]
    _two_group(c, lambda sc, k: _argmax_dist(D[(sc, k)], "surf_T1a_diff_K8"),
               lambda sc, k, v, fmt: fmt % v,
               fmt="%.3f m",
               ylabel="argmax 到真值的距离（m）",
               title="(c) 方案口径 $D_y$ + K=8：去镜像把两个场景\n都推进到 0.05 m 以内，仍未进 0.03 m",
               ylim=(0, 0.135),
               hlines=[(0.03, "成功线 0.03 m", "k", "--"),
                       (0.015684, "物理地板 0.0157 m", "green", ":")])
    c.legend(fontsize=9, loc="upper right")

    # ---------- (d) 谱 ----------
    d_ = ax[1, 1]
    n = 16
    for k in ("base", "nomirror"):
        dd = D[("bu", k)]
        ti = _ti(dd)
        for oi, (nm, mk) in enumerate((("A（未差分）", "o"), ("B（差分后）", "s"))):
            s = np.sqrt(np.maximum(dd["lamv"][oi][ti], 0))
            d_.plot(range(1, n + 1), s[:n] / s[0], marker=mk, ms=3.2, lw=1.9,
                    color=CLR[k], alpha=(1.0 if oi == 1 else 0.45),
                    label="%s · %s" % (nm, KLAB[k]))
    d_.axhline(0.75, color="k", ls="--", lw=1.3)
    d_.text(n - 0.4, 0.77, "σ = 0.75 阈值", ha="right", fontsize=9)
    d_.set_xlabel("奇异值序号 k")
    d_.set_ylabel("s$_k$ / s$_1$")
    d_.set_title("(d) 去掉 z 镜像把 A 的 s$_2$/s$_1$ 从 0.185 抬到 0.283，\nB 的高台从 13 阶收窄到 7 阶（谱形确实变了）",
                 fontsize=11)
    d_.set_ylim(0, 1.06)
    d_.grid(alpha=0.3)
    d_.legend(fontsize=8.5, loc="center right")

    fig.suptitle("X4l 开关消融 —— 唯一跨场景一致的杠杆是视角因子的 z 镜像，但仍不足以定位",
                 fontsize=13.5, y=0.995)
    fig.tight_layout(rect=(0, 0, 1, 0.955))
    out = os.path.join(RESULTS, "x4l_switch_ablation.png")
    fig.savefig(out, dpi=150)
    print("图: %s" % out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
