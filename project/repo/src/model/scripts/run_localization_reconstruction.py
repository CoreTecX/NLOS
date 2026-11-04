# -*- coding: utf-8 -*-
"""T0-05 定位与重建：完整链路驱动脚本。

流程：
  1. 粗到细定位 —— 采用官方四轮搜索网格与截断阈值，逐轮与官方 best_pos 比对
  2. 能量图对照 —— 与官方 proj_yOnRangeA 系列做 Pearson 相关
  3. TV-PDHG 重建 —— 在真值位置与各估计位置重建，与官方 ground_truth1 比对
  4. 出报告与图

**关于搜索网格的说明**：第 2~4 轮的网格直接取自官方配置文件的
`xhatvals2/3/4` 等字段，即官方当年实际使用的细化区间。因此本脚本检验的是
「给定官方网格，本实现的能量函数是否给出与官方一致的最优点」，
而不是「本实现自己的粗到细细化策略是否同样收敛」。

**关于求解预条件**：差分测量的量级可达 1e6，直接求解会使数据项达到 1e12，
而 TV 项仅 1e5，两者相差 7 个量级，λ 无法用常规量级表达。因此在求解器之外
对测量与矩阵做统一缩放（除以测量 RMS），缩放系数记录在结果中，
重建结果再乘回原始量级。`reconstruction.tv_pdhg` 本身保持为不含预条件的纯求解器。

用法（在仓库根目录）：
  python src/model/scripts/run_localization_reconstruction.py \
      --data-root "<官方 Data 目录>"
"""

from __future__ import annotations

import argparse
import io
import json
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
from penumbrasight_model import localization as L  # noqa: E402
from penumbrasight_model import reconstruction as R  # noqa: E402

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    _HAVE_MPL = True
except Exception:  # noqa: BLE001
    _HAVE_MPL = False

CJK = ["Noto Sans CJK SC", "WenQuanYi Micro Hei", "Microsoft YaHei", "SimHei", "DejaVu Sans"]


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


# ---------------------------------------------------------------- 重建


def solve_channel(A_d: np.ndarray, b_raw: np.ndarray, shape, lam: float,
                  n_iter: int, L2: float):
    """单通道求解：先按测量 RMS 归一化，求解后把结果乘回原始量级。"""
    rms = float(np.sqrt(np.mean(b_raw ** 2)))
    if rms <= 0:
        raise ValueError("测量 RMS 为零")
    b_n = b_raw / rms
    res = R.tv_pdhg(A_d, b_n, shape, lam=lam, n_iter=n_iter, L2=L2)
    res.image = res.image * rms            # 还原到原始量级
    return res


def reconstruct_at(setup, meas_rgb, ll, lam, n_iter, tag, cache={}):
    """在给定遮挡物位置重建三通道。

    cache 以位置字符串为键复用 A 与 L2，避免重复构造。
    """
    key = np.round(np.asarray(ll, dtype=np.float64), 6).tobytes()
    if key not in cache:
        A = F.build_transport_matrix(setup.with_occluder_ll(ll))
        A_d = L.apply_row_difference_A(A, setup.wall.n)
        L2 = R.operator_norm_sq(A_d, setup.monitor.shape, n_iter=15)
        cache[key] = (A_d, L2)
        del A
    A_d, L2 = cache[key]

    preds, obj = [], None
    for c in range(3):
        b_raw = L.apply_row_difference_img(np.asarray(meas_rgb[:, :, c], dtype=np.float64))
        res = solve_channel(A_d, b_raw, setup.monitor.shape, lam, n_iter, L2)
        preds.append(res.image)
        if c == 0:
            obj = res.objective
    return {"tag": tag, "ll": np.asarray(ll, dtype=np.float64).tolist(),
            "psnr": None, "pearson": None, "objective": obj,
            "L2": L2, "_pred": np.dstack(preds)}


def eval_quality(rec, gt_rgb):
    rec["psnr"] = R.psnr_scale_matched(rec["_pred"], gt_rgb)
    rec["pearson"] = R.pearson(rec["_pred"], gt_rgb)
    return rec


# ---------------------------------------------------------------- 定位

#: 官方第 k 轮能量图 / best_pos 的字段后缀
_ROUND_SUFFIX = {1: "", 2: "_2", 3: "_3", 4: "_4"}
#: 官方三个色通道的能量图字段前缀（官方 `p_est = mean(p_est_rgb)`）
_CHANNEL_KEYS = ("proj_yOnRangeA", "proj_yOnRangeA_g", "proj_yOnRangeA_b")


def _round_suffix(k: int) -> str:
    return _ROUND_SUFFIX.get(k, f"_{k}")


def _suffix_to_round(suffix: str) -> int:
    return 1 if suffix == "" else int(suffix.lstrip("_"))


def _thresh_key(k: int) -> str:
    return "thresh" if k == 1 else f"thresh_{k}"


def official_thresh_index(d, k: int) -> int:
    """官方第 k 轮**实际使用**的阈值档下标（0 基）。

    官方 `proj_yOnRangeA{k}` 是 4 维数组，第 4 维是阈值档列表 `threshold` 的
    下标；而该轮真正使用的那一档是 `thresh{k}`（1 基）。于是 0 基下标

        t = thresh{k} - 1

    这条口径由「三通道 3D argmax 的均值逐位等于 `best_pos{k}`」反解得到
    （`sim/results/x27_fc_order_root_cause.md` §2.4：4/4 轮 `max|Δ下标| = 0`）。
    """
    key = _thresh_key(k)
    if key not in d:
        return 0
    return int(np.asarray(d[key]).reshape(-1)[0]) - 1


def _nearest_index(values: np.ndarray, target: float) -> float:
    """线性插值反解 target 在（升序）网格 values 上的浮点下标。"""
    return float(np.interp(target, values, np.arange(values.size)))


def best_pos_readout_check(d, grids) -> list:
    """硬判据：官方 `proj_yOnRangeA{k}[..., thresh_k-1]` 的三通道 3D argmax
    均值是否逐位复现 `best_pos{k}`。

    判据来自官方自己的代码：`occluderposgridsearch.m` 里 `p_est = mean(p_est_rgb)`，
    而 `p_est_rgb` 是**逐通道三维 argmax**。所以「三通道 argmax 的均值」必须逐位
    等于文件里存的 `best_pos{k}`。这条与本次搜索规模无关，四轮全覆盖。
    """
    rows = []
    for k in (1, 2, 3, 4):
        bp_key = "best_pos" if k == 1 else f"best_pos_{k}"
        if bp_key not in d:
            continue
        suffix = _round_suffix(k)
        t = official_thresh_index(d, k)
        gx, gy, gz = (grids.get(f"r{k}_x"), grids.get(f"r{k}_y"),
                      grids.get(f"r{k}_z"))
        if gx is None or gy is None or gz is None:
            continue
        idxs = []
        for ch in _CHANNEL_KEYS:
            key = ch + suffix
            if key not in d:
                break
            cube = np.asarray(d[key], dtype=np.float64)
            if cube.ndim != 4 or t >= cube.shape[3]:
                break
            idxs.append(np.array(np.unravel_index(np.argmax(cube[:, :, :, t]),
                                                  cube.shape[:3]), dtype=np.float64))
        if len(idxs) != len(_CHANNEL_KEYS):
            continue
        mean_idx = np.mean(idxs, axis=0)                       # 浮点下标
        bp = np.asarray(d[bp_key], dtype=np.float64).reshape(-1)
        fidx = np.array([_nearest_index(gx, bp[0]), _nearest_index(gy, bp[1]),
                         _nearest_index(gz, bp[2])])
        delta = mean_idx - fidx
        rows.append({
            "round": k, "best_pos": bp.tolist(),
            "thresh_key": _thresh_key(k), "thresh_k": t + 1, "t": t,
            "mean_argmax_index": [round(float(v), 6) for v in mean_idx],
            "best_pos_index": [round(float(v), 6) for v in fidx],
            "max_abs_index_delta": float(np.max(np.abs(delta))),
            "pass": bool(np.max(np.abs(delta)) < 1e-9)})
    return rows


def compare_to_official(cfg_path, rounds, grids=None):
    """**修正后**的比对口径。

    与旧口径（见 `compare_to_official_legacy`）的三处差异：

    1. **轮次配对** —— 第 k 轮只与官方 `*{suffix_k}` 比。旧代码在第 2 轮的内层
       循环里把 `""/_2/_3/_4` 都试了一遍，于是官方第 3、4 轮的图（切片形状
       恰好也是 (6,6)）通过了形状检查、被当成「第 2 轮」与本实现第 2 轮的
       立方体比 —— 这是「argmax 恒为 [0,2]」的真正来源；
    2. **阈值档** —— 官方第 4 维是阈值档，取 `t = thresh{k} - 1`（旧代码恒取 0，
       对第 2、3 轮是错档）；
    3. **比较口径** —— 以**全 3D 立方体**为主，同时给出「官方 argmax 所在 z 层」
       与「本实现 argmax 所在 z 层」两个切片数（旧代码只用 z=0，而第 1 轮的
       真实峰值在 z=1）。

    另外给出两项与本次搜索规模无关的量：
      * `best_pos_repro`：官方自身读出口径的硬判据（4 轮全覆盖）；
      * `official_family_band`：官方**自己**换阈值档的两两 3D r，用作「同族上界」。
    """
    d = io_mat.load_mat(cfg_path)
    if grids is None:
        grids = L.load_search_grids(cfg_path)
    out = {"design": [], "best_pos_repro": [], "best_pos_comparison": [],
           "energy_map_comparison": [], "energy_map_comparison_legacy": [],
           "official_family_band": []}

    # ---- (0) 读出口径设计表：逐轮 suffix / thresh_k / t / 档数 ----
    for k in (1, 2, 3, 4):
        key = _CHANNEL_KEYS[0] + _round_suffix(k)
        if key not in d:
            continue
        shp = np.asarray(d[key]).shape
        t = official_thresh_index(d, k)
        out["design"].append({
            "round": k, "official_key": key, "suffix": _round_suffix(k),
            "thresh_key": _thresh_key(k), "thresh_k": t + 1, "t": t,
            "n_thresholds": int(shp[3]) if len(shp) == 4 else 1,
            "official_shape": [int(v) for v in shp]})

    # ---- (1) 硬判据：官方 best_pos 逐位复现 ----
    out["best_pos_repro"] = best_pos_readout_check(d, grids)

    # ---- (2) 逐轮位置比对：best_pos{k} ↔ 本实现第 k 轮 ----
    for i, r in enumerate(rounds, 1):
        key = "best_pos" if i == 1 else f"best_pos_{i}"
        if key in d:
            off = np.asarray(d[key], dtype=np.float64).reshape(-1)
            out["best_pos_comparison"].append({
                "round": i, "official": off.tolist(),
                "mine": r.best_ll.tolist(),
                "delta": (r.best_ll - off).tolist(),
                "dist_m": float(np.linalg.norm(r.best_ll - off))})

    # ---- (3) 修正后：同轮、同档、全 3D + 两个 z 层切片 ----
    for i, r in enumerate(rounds, 1):
        suffix = _round_suffix(i)
        t = official_thresh_index(d, i)
        for pre in _CHANNEL_KEYS:
            key = pre + suffix
            if key not in d:
                continue
            off4 = np.asarray(d[key], dtype=np.float64)
            if off4.ndim == 4:
                if t >= off4.shape[3]:
                    continue
                off_cube = off4[:, :, :, t]
            elif off4.ndim == 3:
                off_cube = off4
            else:
                continue
            mine = np.asarray(r.energies, dtype=np.float64)
            if off_cube.shape != mine.shape:
                continue        # 修正后不再跨轮错配：形状不同就跳过
            oa = np.unravel_index(np.argmax(off_cube), off_cube.shape)
            ma = np.unravel_index(np.argmax(mine), mine.shape)
            out["energy_map_comparison"].append({
                "round": i, "official_key": key, "t": t,
                "official_shape": [int(v) for v in off4.shape],
                "mine_shape": [int(v) for v in mine.shape],
                "pearson_3d": R.pearson(mine, off_cube),
                "pearson_z_at_official_argmax": R.pearson(
                    mine[:, :, oa[2]], off_cube[:, :, oa[2]]),
                "pearson_z_at_mine_argmax": R.pearson(
                    mine[:, :, ma[2]], off_cube[:, :, ma[2]]),
                "pearson_z0": R.pearson(mine[:, :, 0], off_cube[:, :, 0]),
                "official_argmax": [int(v) for v in oa],
                "mine_argmax": [int(v) for v in ma]})

    # ---- (4) 官方同族上界：官方自己换阈值档的两两 3D r ----
    for k in (1, 2, 3, 4):
        key = _CHANNEL_KEYS[0] + _round_suffix(k)
        if key not in d:
            continue
        off4 = np.asarray(d[key], dtype=np.float64)
        if off4.ndim != 4:
            continue
        rs = [R.pearson(off4[:, :, :, a], off4[:, :, :, b])
              for a in range(off4.shape[3]) for b in range(a + 1, off4.shape[3])]
        if rs:
            rs = np.asarray(rs, dtype=np.float64)
            out["official_family_band"].append({
                "round": k, "n_pairs": int(rs.size),
                "min": float(rs.min()), "median": float(np.median(rs)),
                "max": float(rs.max())})

    return out


def compare_to_official_legacy(cfg_path, rounds):
    """**修正前**的比对口径 —— 仅用于出「修正前」对照表，不要用于验收。

    完整复刻旧代码的三处缺陷：第 4 维恒取 0、切片恒取 z=0、内层循环对同一轮
    把 `""/_2/_3/_4` 四个后缀都试一遍。调用时请只传**旧运行当时的轮次**
    （x25 那次只跑了第 1、2 轮），否则错配后的表会随轮数改变、无法与旧记录对齐。
    """
    d = io_mat.load_mat(cfg_path)
    rows = []
    for i, r in enumerate(rounds, 1):
        for pre in _CHANNEL_KEYS:
            for suffix in ("", "_2", "_3", "_4"):
                key = pre + suffix
                if key not in d:
                    continue
                off = np.asarray(d[key], dtype=np.float64)
                if off.ndim == 4:
                    sl = off[:, :, 0, 0]
                elif off.ndim == 3:
                    sl = off[:, :, 0]
                else:
                    continue
                if sl.shape != r.energies.shape[:2]:
                    continue
                mine = r.energies[:, :, 0] if r.energies.shape[2] > 0 else r.energies
                if mine.shape != sl.shape:
                    continue
                rows.append({
                    "round_used_as": i, "official_key": key,
                    "official_is_round": _suffix_to_round(suffix),
                    "pearson": R.pearson(mine, sl),
                    "official_argmax": [int(v) for v in np.unravel_index(
                        np.argmax(sl), sl.shape)],
                    "mine_argmax": [int(v) for v in np.unravel_index(
                        np.argmax(mine), mine.shape)]})
    return rows


# ---------------------------------------------------------------- 出图


def fig_localization(rounds, cfg, out_png):
    n = len(rounds)
    fig, axes = plt.subplots(1, n + 1, figsize=(3.2 * (n + 1), 3.5),
                             constrained_layout=True)
    for ax, r in zip(axes, rounds):
        E = r.energies[:, :, 0]
        im = ax.imshow(E.T, cmap="viridis", origin="lower", aspect="auto",
                       interpolation="nearest")
        am = np.unravel_index(np.argmax(E), E.shape)
        ax.plot(am[0], am[1], "r*", ms=14)
        ax.set_title(f"第 {r.index} 轮 κ={r.kappa}\n最优 {np.round(r.best_ll, 4)}",
                     fontsize=8.5)
        ax.set_xlabel("x 网格", fontsize=9)
        ax.set_ylabel("y 深度网格", fontsize=9)
        fig.colorbar(im, ax=ax, fraction=0.046, pad=0.02)
    d = io_mat.load_mat(cfg)
    off = np.asarray(d["proj_yOnRangeA"], dtype=np.float64)[:, :, 0, 0]
    ax = axes[-1]
    im = ax.imshow(off.T, cmap="viridis", origin="lower", aspect="auto",
                   interpolation="nearest")
    am = np.unravel_index(np.argmax(off), off.shape)
    ax.plot(am[0], am[1], "r*", ms=14)
    ax.set_title("官方 proj_yOnRangeA\n第 1 轮、κ 档 0", fontsize=8.5)
    ax.set_xlabel("x 网格", fontsize=9)
    fig.colorbar(im, ax=ax, fraction=0.046, pad=0.02)
    fig.suptitle("粗到细定位：本实现能量景观（前四格）与官方公布能量图（末格）对照",
                 fontsize=12)
    fig.savefig(out_png, dpi=130)
    plt.close(fig)


def fig_reconstruction(gt, recs, out_png):
    n = len(recs)
    fig, axes = plt.subplots(2, n + 1, figsize=(2.5 * (n + 1), 4.9),
                             constrained_layout=True)
    def norm(a):
        m = float(np.max(a)) if np.max(a) > 0 else 1.0
        return np.clip(a / m, 0, 1)
    axes[0, 0].imshow(norm(gt)); axes[0, 0].set_title("官方真值", fontsize=9.5)
    axes[1, 0].imshow(norm(gt[:, :, 0]), cmap="gray")
    axes[1, 0].set_title("真值 R 通道", fontsize=9.5)
    for ax, rec in zip(axes[0, 1:], recs):
        ax.imshow(norm(rec["_pred"]))
        ax.set_title(f"{rec['tag']}\nPSNR {rec['psnr']:.1f} dB  r={rec['pearson']:.3f}",
                     fontsize=8.5)
    for ax, rec in zip(axes[1, 1:], recs):
        ax.imshow(norm(rec["_pred"][:, :, 0]), cmap="gray")
    for ax in axes.ravel():
        ax.set_xticks([]); ax.set_yticks([])
    fig.suptitle("TV-PDHG 重建（上：RGB 合成；下：R 通道）", fontsize=12)
    fig.savefig(out_png, dpi=130)
    plt.close(fig)


def fig_lambda(lam_rows, out_png):
    fig, ax = plt.subplots(figsize=(7.2, 4.4), constrained_layout=True)
    lams = [r["lam"] for r in lam_rows]
    ax.semilogx(lams, [r["psnr"] for r in lam_rows], "o-", label="PSNR (dB)")
    ax.set_xlabel("TV 权重 lambda")
    ax.set_ylabel("PSNR (dB)")
    ax.grid(alpha=0.3, which="both")
    ax2 = ax.twinx()
    ax2.semilogx(lams, [r["pearson"] for r in lam_rows], "s--", color="tab:orange",
                 label="Pearson r")
    ax2.set_ylabel("Pearson r", color="tab:orange")
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, fontsize=9.5, loc="best")
    ax.set_title("TV 权重对重建质量的影响")
    fig.savefig(out_png, dpi=130)
    plt.close(fig)


def fig_objective(recs, out_png):
    fig, ax = plt.subplots(figsize=(7.2, 4.3), constrained_layout=True)
    for rec in recs:
        if not rec["objective"]:
            continue
        ax.semilogy([o["iter"] for o in rec["objective"]],
                    [o["obj"] for o in rec["objective"]], "o-", ms=3.5,
                    label=rec["tag"])
    ax.set_xlabel("迭代次数")
    ax.set_ylabel("目标函数值（归一化系统）")
    ax.set_title("PDHG 目标函数收敛性")
    ax.grid(alpha=0.3, which="both")
    ax.legend(fontsize=9)
    fig.savefig(out_png, dpi=130)
    plt.close(fig)


# ---------------------------------------------------------------- 报告


def write_report(path, setup, rounds, cmp, lam_rows, lam_used, recs, font,
                 elapsed, n_iter):
    M = []
    M.append("# 定位与重建验证报告（T0-05）\n")
    M.append(f"- 生成时间：{time.strftime('%Y-%m-%d %H:%M:%S')}")
    M.append(f"- 中文字体：`{font}`；总耗时 {elapsed:.0f} s；重建迭代 {n_iter} 次")
    M.append("- 数据来源：官方 `TestPosChair`，配置与测量取自 `chair_test.mat`\n")

    M.append("## 1. 方法要点\n")
    M.append("定位在**差分框架**下进行（`y_d = D y`，`A_d = D A`），依据是 T0-04 的实测结论：")
    M.append("不做差分时 κ=0.75 只保留 1 个奇异成分，能量评价失去分辨能力。")
    M.append("重建采用 **PDHG** 而非论文的 FISTA，原因是 FISTA 需要各向同性 TV 的邻近算子，")
    M.append("该算子无闭式解、需内层迭代，不利于后续定点化；PDHG 每步只含矩阵向量乘与")
    M.append("逐像素运算，误差传播更容易界定。\n")
    M.append(f"**搜索网格来源**：第 2~4 轮网格取自官方配置的 `xhatvals2/3/4` 等字段，")
    M.append(f"即官方当年实际使用的细化区间。因此本项检验的是「给定官方网格，本实现的")
    M.append(f"能量函数是否给出与官方一致的最优点」，而非本实现自身的细化策略。\n")

    M.append("## 2. 粗到细定位结果\n")
    M.append("| 轮次 | κ | 网格 | 候选数 | 最优位置 | 能量 | 保留成分 | 耗时 s |")
    M.append("|---|---|---|---|---|---|---|---|")
    for r in rounds:
        M.append(f"| {r.index} | {r.kappa} | {r.x_values.size}x{r.y_values.size}x"
                 f"{r.z_values.size} | {r.n_candidates} | "
                 f"{np.round(r.best_ll, 4).tolist()} | {r.best_energy:.6g} | "
                 f"{r.n_kept_at_best} | {r.elapsed:.0f} |")
    M.append("")

    M.append("## 3. 与官方位置估计的逐轮比对\n")
    if cmp["best_pos_comparison"]:
        M.append("| 轮次 | 官方 | 本实现 | 偏差 (x,y,z) m | 欧氏距离 m |")
        M.append("|---|---|---|---|---|")
        for row in cmp["best_pos_comparison"]:
            M.append(f"| {row['round']} | {np.round(row['official'],4).tolist()} | "
                     f"{np.round(row['mine'],4).tolist()} | "
                     f"{np.round(row['delta'],4).tolist()} | {row['dist_m']:.4f} |")
    else:
        M.append("（无可比对字段）")
    M.append("")

    M.append("## 4. 能量图与官方公布结果的相关系数\n")
    M.append("**口径说明**：官方 `proj_yOnRangeA{k}` 是 4 维数组，第 4 维是阈值档；")
    M.append("该轮实际使用的那一档由 `thresh{k}` 决定，取 0 基下标 `t = thresh{k} - 1`。")
    M.append("本节所有「修正后」的数都满足：**同轮配对（第 k 轮 ↔ `{suffix_k}`）+ 正确档 t + 全 3D 立方体**。\n")

    des = cmp.get("design") or []
    if des:
        M.append("### 4.1 官方读出口径（本次修正）\n")
        M.append("| 轮次 | 官方字段 | 官方形状 | thresh 键 | thresh_k | 实际档 t | 档数 |")
        M.append("|---|---|---|---|---|---|---|")
        for row in des:
            M.append(f"| {row['round']} | `{row['official_key']}` | {row['official_shape']} | "
                     f"`{row['thresh_key']}` | {row['thresh_k']} | **{row['t']}** | "
                     f"{row['n_thresholds']} |")
        M.append("")

    rep = cmp.get("best_pos_repro") or []
    if rep:
        M.append("### 4.2 硬判据：官方 `best_pos` 逐位复现\n")
        M.append("判据：`mean(三通道 3D argmax 的浮点下标) == best_pos{k}` 的反解下标，阈值 `1e-9`。")
        M.append("这条只依赖官方数据，与本次搜索规模无关，四轮全覆盖。\n")
        M.append("| 轮次 | best_pos | 三通道 argmax 均值下标 | best_pos 反解下标 | max_abs(Δ下标) | 判定 |")
        M.append("|---|---|---|---|---|---|")
        for row in rep:
            M.append(f"| {row['round']} | {np.round(row['best_pos'], 6).tolist()} | "
                     f"{row['mean_argmax_index']} | {row['best_pos_index']} | "
                     f"{row['max_abs_index_delta']:.3e} | "
                     f"{'PASS' if row['pass'] else 'FAIL'} |")
        M.append("")

    emc = cmp.get("energy_map_comparison") or []
    if emc:
        M.append("### 4.3 修正后：同轮同档的能量图相关（全 3D 为主）\n")
        M.append("| 轮次 | 官方字段 | 档 t | r（全 3D） | r（官方 argmax z 层） | "
                 "r（本实现 argmax z 层） | r（z=0） | 官方 argmax | 本实现 argmax |")
        M.append("|---|---|---|---|---|---|---|---|---|")
        for row in emc:
            M.append(f"| {row['round']} | `{row['official_key']}` | {row['t']} | "
                     f"**{row['pearson_3d']:.4f}** | "
                     f"{row['pearson_z_at_official_argmax']:.4f} | "
                     f"{row['pearson_z_at_mine_argmax']:.4f} | "
                     f"{row['pearson_z0']:.4f} | "
                     f"{row['official_argmax']} | {row['mine_argmax']} |")
        M.append("")
        M.append("argmax 均为 3D 全立方体下标 `[ix, iy, iz]`（不是 z=0 切片）。\n")

    band = cmp.get("official_family_band") or []
    if band:
        M.append("### 4.4 官方同族上界（官方自己换阈值档的两两 3D r）\n")
        M.append("用作「两个不同泛函在同网格上本该有的相关量级」的参照带。\n")
        M.append("| 轮次 | 档对数 | 最小 r | 中位 r | 最大 r |")
        M.append("|---|---|---|---|---|")
        for row in band:
            M.append(f"| {row['round']} | {row['n_pairs']} | {row['min']:.4f} | "
                     f"{row['median']:.4f} | {row['max']:.4f} |")
        M.append("")

    leg = cmp.get("energy_map_comparison_legacy") or []
    if leg:
        M.append("### 4.5 修正前口径（x25 那次运行：只跑 2 轮，恒取 t=0、恒取 z=0、跨后缀错配）\n")
        M.append("| 第几轮的位置 | 官方字段 | 该字段实际属第几轮 | Pearson r | 官方 argmax | 本实现 argmax |")
        M.append("|---|---|---|---|---|---|")
        for row in leg:
            warn = "" if row["round_used_as"] == row["official_is_round"] else " ⚠错配"
            M.append(f"| {row['round_used_as']} | `{row['official_key']}` | "
                     f"{row['official_is_round']}{warn} | {row['pearson']:.4f} | "
                     f"{row['official_argmax']} | {row['mine_argmax']} |")
        M.append("")
    if not (des or rep or emc):
        M.append("（形状不匹配或字段缺失）")
        M.append("")

    M.append("## 5. 重建结果\n")
    if lam_rows:
        M.append("### TV 权重选择\n")
        M.append("| lambda | PSNR dB | Pearson r |")
        M.append("|---|---|---|")
        for row in lam_rows:
            M.append(f"| {row['lam']:g} | {row['psnr']:.2f} | {row['pearson']:.4f} |")
        M.append(f"\n选定 lambda = **{lam_used:g}**。")
        M.append("**注意**：该参数在同一场景上选出，PSNR 偏乐观；正式评估需在独立盲测场景重做。\n")
    if recs:
        M.append("### 各位置的重建质量\n")
        M.append("| 重建所用位置 | 坐标 | PSNR dB | Pearson r |")
        M.append("|---|---|---|---|")
        for r in recs:
            M.append(f"| {r['tag']} | {np.round(r['ll'],4).tolist()} | "
                     f"{r['psnr']:.2f} | {r['pearson']:.4f} |")
        M.append("")
        M.append("PSNR 为**标度匹配后**的数值：重建结果与真值之间只差一个未知全局增益")
        M.append("（辐射度标定未知），故先用最小二乘估出该增益再计算。\n")

    M.append("## 6. 产出物\n")
    M.append("| 文件 | 内容 |")
    M.append("|---|---|")
    M.append("| `sim/results/localization_reconstruction.md` | 本报告 |")
    M.append("| `sim/results/localization_reconstruction_summary.json` | 全部数值明细 |")
    M.append("| `sim/results/localization_energy_cubes.npz` | 各轮能量立方体 |")
    M.append("| `sim/results/fig_localization.png` | 能量景观与官方对照 |")
    M.append("| `sim/results/fig_reconstruction.png` | 重建结果对比 |")
    M.append("| `sim/results/fig_lambda.png` | TV 权重影响 |")
    M.append("| `sim/results/fig_objective.png` | PDHG 收敛曲线 |")
    M.append("")

    with io.open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(M))


# ---------------------------------------------------------------- 复用

_CUBE_PREFIX = "E_round"


def load_reused_cubes(path: str) -> dict:
    """从已有 npz 读回 E_round{k} 能量立方体（只读，float64）。"""
    out: dict = {}
    if not path or not os.path.exists(path):
        return out
    with np.load(path) as z:
        for k in z.files:
            if k.startswith(_CUBE_PREFIX) and k[len(_CUBE_PREFIX):].isdigit():
                out[int(k[len(_CUBE_PREFIX):])] = np.asarray(z[k], dtype=np.float64)
    return out


def round_from_cube(index: int, kappa: float, grids, E: np.ndarray):
    """由已存的能量立方体恢复一个 `L.SearchRound`（只需 argmax 与能量）。

    复用的 cube 是 float32 存的，这里转回 float64 再取 argmax；
    对第 1、2 轮实测 argmax 与 float64 结果逐位一致（见复核报告 §重跑实测）。
    """
    xv = np.asarray(grids[f"r{index}_x"], dtype=np.float64)
    yv = np.asarray(grids[f"r{index}_y"], dtype=np.float64)
    zv = np.asarray(grids[f"r{index}_z"], dtype=np.float64)
    if E.shape != (xv.size, yv.size, zv.size):
        raise ValueError(f"{_CUBE_PREFIX}{index} 形状 {E.shape} 与该轮网格 "
                         f"{(xv.size, yv.size, zv.size)} 不符")
    bi = np.unravel_index(np.argmax(E), E.shape)
    best_ll = np.array([xv[bi[0]], yv[bi[1]], zv[bi[2]]])
    return L.SearchRound(index=index, kappa=kappa, x_values=xv, y_values=yv,
                         z_values=zv, energies=E, best_ll=best_ll,
                         best_energy=float(E[bi]), n_candidates=int(E.size),
                         elapsed=0.0, n_kept_at_best=0)


# ---------------------------------------------------------------- 主流程


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-root", required=True)
    ap.add_argument("--out-dir", default="sim/results")
    ap.add_argument("--max-rounds", type=int, default=4)
    ap.add_argument("--n-iter", type=int, default=400)
    ap.add_argument("--lam-candidates", default="1e-4,1e-3,1e-2,1e-1,1,10")
    ap.add_argument("--skip-recon", action="store_true")
    ap.add_argument("--skip-search", action="store_true",
                    help="一个候选都不算：全部轮次从 --reuse-cubes 读")
    ap.add_argument("--reuse-cubes", default="sim/results/localization_energy_cubes.npz",
                    help="复用能量立方体的 npz 路径（读键 E_round{k}）")
    ap.add_argument("--rounds", default="",
                    help="只重算这些轮次（逗号分隔，如 3,4）；其余轮次从 "
                         "--reuse-cubes 读。留空 = 全部重算")
    args = ap.parse_args()

    font = setup_font()
    out = args.out_dir
    os.makedirs(out, exist_ok=True)
    cfg = os.path.join(args.data_root, "TestPosChair", "chair_test.mat")

    t_start = time.time()
    setup = F.load_setup(cfg)
    d = io_mat.load_mat(cfg)
    meas = np.dstack([np.asarray(d[f"meas_{c}"], dtype=np.float64) for c in "rgb"])
    gt = np.asarray(d["ground_truth1"], dtype=np.float64)
    grids = L.load_search_grids(cfg)
    print("[info] 网格： " + "  ".join(
        f"{k}={v.size}" for k, v in grids.items() if k.endswith("_x")), flush=True)

    # ---- 阶段 1：定位 ----
    kappas = (0.75, 0.5, 0.05, 0.05)
    reused = load_reused_cubes(args.reuse_cubes)
    wanted = set(range(1, args.max_rounds + 1))
    if args.rounds.strip():
        wanted = {int(v) for v in args.rounds.split(",") if v.strip()}
    if args.skip_search:
        wanted = set()

    print("\n=== 阶段 1：粗到细定位 ===", flush=True)
    print(f"   重算轮次={sorted(wanted)}  复用轮次="
          f"{sorted(reused) if reused else '（无）'}", flush=True)
    rounds = []
    for i in range(1, args.max_rounds + 1):
        tag = f"r{i}"
        kappa = kappas[min(i - 1, len(kappas) - 1)]
        if i in wanted and f"{tag}_x" in grids:
            r = L.search_grid(setup, meas[:, :, 1], grids[f"{tag}_x"],
                              grids[f"{tag}_y"], grids[f"{tag}_z"],
                              kappa=kappa, index=i, progress=True)
        elif i in reused and f"{tag}_x" in grids:
            r = round_from_cube(i, kappa, grids, reused[i])
            print(f"   第 {i} 轮（复用 {args.reuse_cubes}）："
                  f"最优 ll={np.round(r.best_ll, 5).tolist()}  "
                  f"E={r.best_energy:.6g}", flush=True)
        else:
            continue
        rounds.append(r)
    if rounds:
        np.savez_compressed(
            os.path.join(out, "localization_energy_cubes.npz"),
            **{f"E_round{r.index}": r.energies.astype(np.float32) for r in rounds})
        for r in rounds:
            print(f"   第 {r.index} 轮：最优 ll={np.round(r.best_ll,5).tolist()}  "
                  f"E={r.best_energy:.6g}  保留 {r.n_kept_at_best}  "
                  f"耗时 {r.elapsed:.0f}s", flush=True)

    # ---- 阶段 2：比对 ----
    cmp = {"design": [], "best_pos_repro": [], "best_pos_comparison": [],
           "energy_map_comparison": [], "energy_map_comparison_legacy": [],
           "official_family_band": []}
    if rounds:
        print("\n=== 阶段 2：与官方结果比对 ===", flush=True)
        cmp = compare_to_official(cfg, rounds, grids=grids)
        # 「修正前」对照：复刻旧口径，且只用旧运行当时的轮次（x25 只跑 2 轮）
        cmp["energy_map_comparison_legacy"] = compare_to_official_legacy(
            cfg, rounds[:2])
        for row in cmp["best_pos_comparison"]:
            print(f"   第 {row['round']} 轮：官方 {np.round(row['official'],4).tolist()}"
                  f"  本实现 {np.round(row['mine'],4).tolist()}"
                  f"  距离 {row['dist_m']:.4f} m", flush=True)
        for row in cmp["best_pos_repro"]:
            print(f"   [硬判据] 第 {row['round']} 轮 {row['thresh_key']}={row['thresh_k']}"
                  f" (t={row['t']})：三通道 argmax 均值下标="
                  f"{row['mean_argmax_index']}  best_pos 反解下标="
                  f"{row['best_pos_index']}  max_abs(Δ)="
                  f"{row['max_abs_index_delta']:.3e}  "
                  f"{'PASS' if row['pass'] else 'FAIL'}", flush=True)
        for row in cmp["energy_map_comparison"]:
            print(f"   [修正后] {row['official_key']}(t={row['t']})："
                  f"r(3D)={row['pearson_3d']:.4f}  "
                  f"r(z@官方argmax)={row['pearson_z_at_official_argmax']:.4f}  "
                  f"官方 argmax={row['official_argmax']}  "
                  f"本实现 argmax={row['mine_argmax']}", flush=True)
        for row in cmp["energy_map_comparison_legacy"]:
            mark = "" if row["round_used_as"] == row["official_is_round"] else " <-错配"
            print(f"   [修正前] 拿 {row['official_key']}（实为第 "
                  f"{row['official_is_round']} 轮）当第 {row['round_used_as']} 轮比："
                  f"r={row['pearson']:.4f}  官方 argmax={row['official_argmax']}  "
                  f"本实现 argmax={row['mine_argmax']}{mark}", flush=True)
        for row in cmp["official_family_band"]:
            print(f"   [同族上界] 第 {row['round']} 轮官方各档两两 3D r："
                  f"min={row['min']:.4f} 中位={row['median']:.4f} "
                  f"max={row['max']:.4f}（{row['n_pairs']} 对）", flush=True)

    # ---- 阶段 3：重建 ----
    recs, lam_rows, lam_used = [], [], None
    cache: dict = {}
    if not args.skip_recon:
        print("\n=== 阶段 3：TV-PDHG 重建 ===", flush=True)
        true_ll = np.asarray(d["Occluder"], dtype=np.float64)[0]
        cands = [float(v) for v in args.lam_candidates.split(",")]
        print(f"   在真值位置扫描 lambda（{len(cands)} 个）：", flush=True)
        for lam in cands:
            rec = reconstruct_at(setup, meas, true_ll, lam, args.n_iter,
                                 "lambda扫描", cache)
            eval_quality(rec, gt)
            lam_rows.append({"lam": lam, "psnr": rec["psnr"],
                             "pearson": rec["pearson"]})
            print(f"      lambda={lam:<8g} PSNR={rec['psnr']:6.2f} dB  "
                  f"r={rec['pearson']:.4f}", flush=True)
        lam_used = max(lam_rows, key=lambda r: r["psnr"])["lam"]
        print(f"   选定 lambda = {lam_used:g}", flush=True)

        targets = [("真值位置", true_ll)]
        if rounds:
            targets.append(("本实现第1轮", rounds[0].best_ll))
            targets.append(("本实现末轮", rounds[-1].best_ll))
        if "best_pos_4" in d:
            targets.append(("官方best_pos_4",
                            np.asarray(d["best_pos_4"], dtype=np.float64).reshape(-1)))
        for tag, ll in targets:
            print(f"   重建 {tag} ll={np.round(ll,4).tolist()} ...", flush=True)
            rec = reconstruct_at(setup, meas, ll, lam_used, args.n_iter, tag, cache)
            eval_quality(rec, gt)
            print(f"      PSNR={rec['psnr']:.2f} dB  r={rec['pearson']:.4f}", flush=True)
            recs.append(rec)

    # ---- 阶段 4：出图 ----
    if _HAVE_MPL:
        print("\n=== 阶段 4：出图 ===", flush=True)
        if rounds:
            fig_localization(rounds, cfg, os.path.join(out, "fig_localization.png"))
        if recs:
            fig_reconstruction(gt, recs, os.path.join(out, "fig_reconstruction.png"))
            fig_objective(recs, os.path.join(out, "fig_objective.png"))
        if lam_rows:
            fig_lambda(lam_rows, os.path.join(out, "fig_lambda.png"))

    summary = {"rounds": [r.as_dict() for r in rounds], "comparison": cmp,
               "lambda_selection": lam_rows, "lambda_used": lam_used,
               "n_iter": args.n_iter,
               "reconstructions": [{k: v for k, v in r.items() if k != "_pred"}
                                   for r in recs],
               "elapsed_s": time.time() - t_start}
    with io.open(os.path.join(out, "localization_reconstruction_summary.json"),
                 "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2, default=float)

    write_report(os.path.join(out, "localization_reconstruction.md"),
                 setup, rounds, cmp, lam_rows, lam_used, recs, font,
                 time.time() - t_start, args.n_iter)
    print(f"\n[done] 总耗时 {time.time()-t_start:.0f}s", flush=True)


if __name__ == "__main__":
    main()
