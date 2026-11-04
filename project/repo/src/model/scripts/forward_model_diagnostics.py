# -*- coding: utf-8 -*-
"""T0-04 前向模型诊断：投影能量景观的方向性与官方估计的对照。

V5 冒烟测试发现投影能量在真值位置附近随 z 单调上升，说明在朴素配置下
能量峰值并不落在真值。本脚本用于定位原因，回答三个问题：

  D1  官方公布的估计位置 best_pos（及其各轮细化值）在本模型下得分如何？
      若官方估计的得分高于真值，说明本模型与官方模型方向一致，
      偏差来自模型本身或真值标注；若低于真值，说明本模型与官方模型不一致。
  D2  把测量与矩阵同时做垂直差分（S011 的差分框架）后，景观方向是否改变？
  D3  截断阈值 kappa 是否改变景观方向？（官方第一轮用 0.75）

产出：sim/results/geometry_diagnostics.md 与 fig_energy_landscape.png
"""

from __future__ import annotations

import argparse
import io
import os
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC_MODEL = os.path.dirname(_HERE)
if _SRC_MODEL not in sys.path:
    sys.path.insert(0, _SRC_MODEL)

from penumbrasight_model import forward as F  # noqa: E402
from penumbrasight_model import io_mat  # noqa: E402

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    _HAVE_MPL = True
except Exception:  # noqa: BLE001
    _HAVE_MPL = False

CJK = ["Noto Sans CJK SC", "WenQuanYi Micro Hei", "Microsoft YaHei", "SimHei", "DejaVu Sans"]
KAPPAS = (0.75, 0.5, 0.05, 0.0)


def setup_font() -> str:
    if not _HAVE_MPL:
        return "(matplotlib 不可用)"
    from matplotlib import font_manager
    avail = {f.name for f in font_manager.fontManager.ttflist}
    chosen = next((n for n in CJK if n in avail), None)
    if chosen:
        matplotlib.rcParams["font.sans-serif"] = [chosen] + CJK
    matplotlib.rcParams["axes.unicode_minus"] = False
    return chosen or "(无中文字体)"


def diff_rows(A: np.ndarray, n: int) -> np.ndarray:
    """对按行排布的测量矩阵与 A 同时施加垂直差分。"""
    Ar = A.reshape(n, n, -1)
    return (Ar[1:, :, :] - Ar[:-1, :, :]).reshape(-1, A.shape[1])


def diff_img(y2d: np.ndarray) -> np.ndarray:
    return (y2d[1:, :] - y2d[:-1, :]).reshape(-1)


def energies(A: np.ndarray, y: np.ndarray, kappas=KAPPAS) -> dict:
    """一次特征分解，给出多个 kappa 下的投影能量。"""
    z = A.T @ y
    G = A.T @ A
    G = 0.5 * (G + G.T)
    lam, V = np.linalg.eigh(G)
    del G
    lam = lam[::-1]
    V = V[:, ::-1]
    lam = np.maximum(lam, 0.0)
    s = np.sqrt(lam)
    c = V.T @ z
    out = {}
    for k in kappas:
        keep = (s >= k * s[0]) if k > 0 else np.ones(s.size, dtype=bool)
        out[k] = float(np.sum((c[keep] ** 2) / lam[keep]))
        out[f"K@{k}"] = int(keep.sum())
    return out


def evaluate(setup, y2d, ll, use_diff, kappas=KAPPAS):
    s2 = setup.with_occluder_ll(ll)
    A = F.build_transport_matrix(s2)
    y = diff_img(y2d) if use_diff else np.asarray(y2d, dtype=np.float64).reshape(-1)
    if use_diff:
        A = diff_rows(A, setup.wall.n)
    e = energies(A, y, kappas)
    del A
    return e


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-root", required=True)
    ap.add_argument("--out-dir", default="sim/results")
    args = ap.parse_args()

    font = setup_font()
    os.makedirs(args.out_dir, exist_ok=True)
    cfg = os.path.join(args.data_root, "TestPosChair", "chair_test.mat")
    setup = F.load_setup(cfg)
    d = io_mat.load_mat(cfg)
    y2d = np.asarray(d["meas_g"], dtype=np.float64)
    n = setup.wall.n

    true_ll = setup.occluder.ll.copy()
    best = {k: np.asarray(d[k], dtype=np.float64).reshape(-1)
            for k in ("best_pos", "best_pos_2", "best_pos_3", "best_pos_4")}

    print(f"[info] 真值 LL = {true_ll}")
    print(f"[info] 遮挡物中心 = {setup.occluder.center}")
    for k, v in best.items():
        print(f"[info] {k:11s} = {v}")

    # ---- 候选位置：真值、官方各轮估计、z 方向扫描 ----
    cands = [("真值 LL", true_ll)]
    for k, v in best.items():
        cands.append((f"官方 {k}（LL 解读）", v))
        cands.append((f"官方 {k}（中心解读）", v - setup.occluder.size[[0, 0, 1]] / 2))
    for dz in (-0.06, -0.04, -0.02, 0.02, 0.04, 0.06):
        cands.append((f"z {dz:+.0%}".replace("%", "m"), true_ll + np.array([0, 0, dz])))

    results = {}
    for tag, use_diff in (("原始 y", False), ("垂直差分 y", True)):
        print(f"[run] {tag} ...")
        rows = []
        for name, ll in cands:
            t0 = time.time()
            e = evaluate(setup, y2d, ll, use_diff)
            rows.append({"name": name, "ll": ll.tolist(), "e": e,
                         "elapsed": time.time() - t0})
            print(f"       {name:24s} E(0.75)={e[0.75]:.8g}  "
                  f"K={e['K@0.75']:4d}  {e['elapsed'] if False else ''}{time.time()-t0:.2f}s")
        results[tag] = rows

    if _HAVE_MPL:
        make_fig(results, true_ll, args.out_dir)
    write_report(os.path.join(args.out_dir, "geometry_diagnostics.md"),
                 setup, true_ll, best, results, font)
    print(f"[done] {os.path.join(args.out_dir, 'geometry_diagnostics.md')}")


def make_fig(results, true_ll, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(13.5, 5.0), constrained_layout=True)
    for ax, (tag, rows) in zip(axes, results.items()):
        names = [r["name"] for r in rows]
        base = rows[0]["e"][0.75]
        vals = [100 * (r["e"][0.75] / base - 1) for r in rows]
        colors = ["tab:red" if "真值" in nm else
                  ("tab:blue" if "官方" in nm else "tab:gray") for nm in names]
        ax.barh(np.arange(len(names)), vals, color=colors)
        ax.set_yticks(np.arange(len(names)))
        ax.set_yticklabels(names, fontsize=7.5)
        ax.axvline(0.0, color="k", lw=1)
        ax.set_xlabel("投影能量相对真值位置的偏差（%）")
        ax.set_title(f"{tag}，κ = 0.75", fontsize=11)
        ax.grid(alpha=0.3, axis="x")
    fig.suptitle("投影能量景观：真值位置、官方各轮估计与 z 方向扫描的对照", fontsize=12)
    fig.savefig(os.path.join(out_dir, "fig_energy_landscape.png"), dpi=130)
    plt.close(fig)


def write_report(path, setup, true_ll, best, results, font):
    L = []
    L.append("# 前向模型诊断报告（T0-04 补充）\n")
    L.append(f"- 生成时间：{time.strftime('%Y-%m-%d %H:%M:%S')}")
    L.append(f"- 中文字体：`{font}`")
    L.append(f"- 真值遮挡物参考角 LL = {true_ll.tolist()}")
    L.append(f"- 遮挡物尺寸 = {setup.occluder.size.tolist()}，位于 y = {setup.occluder.y}\n")

    L.append("## 1. 起因\n")
    L.append("冒烟测试发现投影能量在真值位置附近**沿 z 单调上升**，峰值不在真值处。")
    L.append("本诊断回答三个问题：官方公布的估计位置在本模型下得分如何；垂直差分是否改变结论；")
    L.append("截断阈值 κ 是否改变结论。\n")

    L.append("## 2. 官方位置估计的两种解读\n")
    L.append("`best_pos` 系列未标注其参考点，故同时尝试两种解读。遮挡物尺寸 7.5cm，")
    L.append("两种解读下矩形位置相差半个边长，对应约 3.75cm 的系统偏移。\n")
    L.append("| 轮次 | 值 |")
    L.append("|---|---|")
    for k, v in best.items():
        L.append(f"| {k} | {v.tolist()} |")
    L.append("")

    for tag, rows in results.items():
        L.append(f"## 3. 结果：{tag}\n")
        ks = [k for k in rows[0]["e"] if isinstance(k, float)]
        hdr = " | ".join(f"E(κ={k})" for k in ks)
        sub = " | ".join("---" for _ in ks)
        L.append(f"| 候选位置 | {hdr} | 保留奇异值数(κ=0.75) |")
        L.append(f"|---|{sub}|---|")
        for r in rows:
            cells = " | ".join(f"{r['e'][k]:.8g}" for k in ks)
            L.append(f"| {r['name']} | {cells} | {r['e']['K@0.75']} |")
        L.append("")
        base = rows[0]["e"][0.75]
        ranked = sorted(rows, key=lambda r: -r["e"][0.75])
        L.append(f"κ=0.75 下能量最高的三个位置：")
        for r in ranked[:3]:
            L.append(f"- {r['name']}（相对真值 {100*(r['e'][0.75]/base-1):+.3f}%）")
        L.append("")

    L.append("## 4. 判读方式\n")
    L.append("- 若**官方估计的能量高于真值位置**，说明本模型的景观方向与官方一致，")
    L.append("  偏差来自模型细节或真值标注，而非几何结构错误；")
    L.append("- 若**官方估计低于真值而扫描显示能量沿某方向单调上升**，说明本模型与官方模型")
    L.append("  在该方向上不一致，需要补齐官方未公开的细节（角度响应 μ 的函数形式、")
    L.append("  截断策略、差分与否）后才能做定位。\n")

    L.append("## 5. 已知的模型缺口\n")
    L.append("| 缺口 | 状态 | 影响 |")
    L.append("|---|---|---|")
    L.append("| 显示器角度响应 μ 的函数形式 | 官方仅公开 `viewAngleCorrection=1` 开关 | ")
    L.append("系统性的辐射度量偏差，会平移能量景观 |")
    L.append("| 墙点采样约定（分块中心 vs 角点） | 采用分块中心，未验证 | 约半个分块的几何偏移 |")
    L.append("| 定位时是否对 y 做垂直差分 | S011 只说明重建前的差分 | 可能整体改变景观方向 |")
    L.append("| `best_pos` 的参考点定义 | 未标注 | 可能造成 3.75cm 系统偏移 |")
    L.append("")

    with io.open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(L))


if __name__ == "__main__":
    main()
