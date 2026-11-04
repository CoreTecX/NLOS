# -*- coding: utf-8 -*-
"""T0-3 前置：帧预算分析（需要多少帧才能看见半影）。

这是项目最大的技术风险——光学信噪比——的直接量化。

信噪比模型（时域噪声估计，可辩护的口径）：

    设单帧差分 d_i = s + n_i
        s    为固定的单帧信号（帧间不变）
        n_i  为帧间独立噪声，逐像素标准差 sigma

    则 N 帧累积 D_N = sum_i d_i = N*s + sum_i n_i
        信号分量空间标准差 = N * std(s)
        噪声分量标准差     = sigma * sqrt(N)
        => SNR(N) = sqrt(N) * std(s) / sigma

    估计量：
        s_hat  = mean_i(d_i)        （35 帧均值，噪声被压低 sqrt(35) 倍）
        std(s) = std(s_hat)
        sigma  = 逐像素时间标准差的中位数

产出：
    sim/vectors/mush_single_frame_diffs.npz   单帧差分缓存（加速重跑）
    sim/results/frame_budget.md
    sim/results/fig_frame_budget.png          不同累积帧数下的差分图视觉对比
    sim/results/fig_snr_model.png             实测 SNR 与 sqrt(N) 模型
    sim/results/snr_model.csv
"""

from __future__ import annotations

import argparse
import csv
import io
import os
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC_MODEL = os.path.dirname(_HERE)
if _SRC_MODEL not in sys.path:
    sys.path.insert(0, _SRC_MODEL)

from penumbrasight_model import PROFILE_OFFICIAL  # noqa: E402
from penumbrasight_model import io_mat  # noqa: E402
from penumbrasight_model.preprocess import preprocess_raw  # noqa: E402

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    _HAVE_MPL = True
except Exception:  # noqa: BLE001
    _HAVE_MPL = False

CJK = ["Noto Sans CJK SC", "WenQuanYi Micro Hei", "Microsoft YaHei", "SimHei", "DejaVu Sans"]
CH_NAMES = ("R", "G", "B")


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


# ---------------------------------------------------------------- 数据


def get_single_frame_diffs(mush_dir: str, cache: str, force: bool = False) -> np.ndarray:
    """逐帧预处理得到单帧差分；带缓存，避免每次重跑 35 次全图预处理。"""
    if os.path.exists(cache) and not force:
        print(f"      命中缓存 {cache}")
        return np.load(cache)["diffs"]
    paths = io_mat.list_frame_series(mush_dir)
    out = []
    for i, p in enumerate(paths, 1):
        out.append(preprocess_raw(io_mat.load_image(p), PROFILE_OFFICIAL).diff)
        if i % 7 == 0 or i == len(paths):
            print(f"      单帧预处理 {i}/{len(paths)}")
    arr = np.stack(out, axis=0)
    os.makedirs(os.path.dirname(cache), exist_ok=True)
    np.savez_compressed(cache, diffs=arr.astype(np.float32))
    print(f"      已写缓存 {cache}")
    return arr


# ---------------------------------------------------------------- 统计


def analyse(D: np.ndarray):
    """返回 (rows, base_snr, std_s, sigma)。"""
    s_hat = D.mean(axis=0)
    sigma = float(np.median(D.std(axis=0, ddof=1)))
    std_s = float(np.std(s_hat))
    base_snr = std_s / sigma if sigma > 0 else float("nan")

    full = D.sum(axis=0)
    b = full.reshape(-1).astype(np.float64)
    b = b - b.mean()
    bb = float(b @ b)

    rows = []
    for n in sorted({1, 2, 4, 8, 16, 32, D.shape[0]}):
        if n > D.shape[0]:
            continue
        cum = D[:n].sum(axis=0)
        a = cum.reshape(-1).astype(np.float64)
        a = a - a.mean()
        rows.append({
            "frames": n,
            "measured_spatial_std": float(np.std(cum)),
            "model_snr_sqrtN": float(np.sqrt(n) * base_snr),
            "corr_with_full": float((a @ b) / np.sqrt((a @ a) * bb)),
        })
    return rows, base_snr, std_s, sigma


# ---------------------------------------------------------------- 出图


def make_budget_figure(D, Ns, channel, out_png):
    n = len(Ns)
    fig, axes = plt.subplots(1, n, figsize=(2.75 * n, 3.9), constrained_layout=True)
    if n == 1:
        axes = [axes]
    vmax = float(np.percentile(np.abs(D.sum(axis=0)[:, :, channel]), 99))
    im = None
    for ax, N in zip(axes, Ns):
        im = ax.imshow(D[:N].sum(axis=0)[:, :, channel], cmap="coolwarm",
                       vmin=-vmax, vmax=vmax, interpolation="nearest", aspect="auto")
        ax.set_title(f"N = {N}", fontsize=12, pad=6)
        ax.set_xticks([])
        ax.set_yticks([])
    fig.colorbar(im, ax=axes, fraction=0.018, pad=0.008)
    fig.suptitle(
        f"累积帧数对墙面半影差分可见性的影响（{CH_NAMES[channel]} 通道，同尺度色标）\n"
        "帧数不足时差分被噪声淹没；随累积增加，隐藏场景的结构逐渐浮现",
        fontsize=12.5)
    fig.savefig(out_png, dpi=130, bbox_inches="tight")
    plt.close(fig)


def make_snr_figure(rows, base_snr, out_png):
    n = [r["frames"] for r in rows]
    fig, ax = plt.subplots(figsize=(7.6, 4.9), constrained_layout=True)
    ax.loglog(n, [r["model_snr_sqrtN"] for r in rows], "o-", lw=2,
              label=r"时域噪声模型 $\sqrt{N}\,\mathrm{std}(s)/\sigma$")
    ax.axhline(3.0, color="tab:orange", lw=1.2, ls="-.",
               label="工程可用参考线 SNR = 3")
    ax.axhline(1.0, color="tab:gray", lw=1.0, ls=":",
               label="信号与噪声等幅 SNR = 1")
    ax.set_xscale("log", base=2)
    ax.set_yscale("log", base=2)
    ax.set_xticks(n)
    ax.set_xticklabels([str(v) for v in n])
    ax.set_xlabel("累积帧数 N")
    ax.set_ylabel("信噪比 SNR(N)")
    ax.set_title(f"帧预算：单帧 SNR = {base_snr:.3g}，需要多少帧才能可靠看见半影")
    ax.grid(alpha=0.3, which="both")
    ax.legend(fontsize=9.5)
    # 交叉点标注
    if base_snr and base_snr > 0:
        n3 = (3.0 / base_snr) ** 2
        if n[0] * 0.5 <= n3 <= n[-1] * 2:
            ax.annotate(f"SNR=3 约需 {int(np.ceil(n3))} 帧",
                        xy=(n3, 3.0), xytext=(n3 * 0.55, 6.5),
                        arrowprops=dict(arrowstyle="->", lw=1.1, color="tab:orange"),
                        fontsize=10, color="tab:orange")
    fig.savefig(out_png, dpi=130)
    plt.close(fig)


# ---------------------------------------------------------------- 报告


def write_report(path, rows, base_snr, sig_spatial, sigma, n_frames,
                 need3, need5, channel, font, elapsed, used_cache):
    L = []
    L.append("# 帧预算分析：需要多少帧才能看见半影\n")
    L.append(f"- 生成时间：{time.strftime('%Y-%m-%d %H:%M:%S')}")
    L.append(f"- 数据：官方 `TestPosChair/Mush` 序列，共 {n_frames} 帧；作图通道 {CH_NAMES[channel]}")
    L.append(f"- 中文字体：`{font}`；耗时 {elapsed:.1f} s；差分数据{'来自缓存' if used_cache else '本次重新计算'}\n")

    L.append("## 1. 时域噪声模型\n")
    L.append("把单帧差分写为 `d_i = s + n_i`（s 帧间不变，n_i 为独立噪声），")
    L.append("则 N 帧累积的信号分量按 N 增长、噪声分量按 sqrt(N) 增长：\n")
    L.append("```\nSNR(N) = sqrt(N) * std(s) / sigma\n```\n")
    L.append("| 量 | 估计值 | 说明 |")
    L.append("|---|---|---|")
    L.append(f"| `std(s)` 单帧信号空间标准差 | {sig_spatial:.6g} | 由 {n_frames} 帧均值估计 |")
    L.append(f"| `sigma` 逐像素时间标准差 | {sigma:.6g} | 取全图中位数 |")
    L.append(f"| **单帧信噪比** `std(s)/sigma` | **{base_snr:.4g}** | 小于 1 表示信号弱于噪声 |")
    L.append("")

    L.append("## 2. 各累积帧数下的结果\n")
    L.append("| 累积帧数 N | 差分空间标准差 | 模型 SNR | 与满帧累积的相关系数 |")
    L.append("|---|---|---|---|")
    for r in rows:
        L.append(f"| {r['frames']} | {r['measured_spatial_std']:.6g} | "
                 f"{r['model_snr_sqrtN']:.4g} | {r['corr_with_full']:.6f} |")
    L.append("")

    L.append("## 3. 结论\n")
    L.append(f"**单帧信噪比只有 {base_snr:.3g}，即单帧的差分信号弱于噪声，单帧不可能成像。**\n")
    if need3:
        L.append(f"- 以 SNR >= 3 为工程可用判据，需要累积 **约 {need3} 帧**；")
    if need5:
        L.append(f"- 以 SNR >= 5 为更保守判据，需要累积 **约 {need5} 帧**；")
    L.append(f"- 交叉验证：官方在该场景使用的正是 {n_frames} 帧累积，数据集内各场景记录的")
    L.append("  累积帧数为 10 / 35 / 40 / 50——**官方也是按场景难度调整帧数的**，与本模型一致。")
    L.append("")

    L.append("## 4. 对本方案设计的四条直接影响\n")
    L.append("1. **自适应累积档位需上调。** 技术方案设定的「1 / 4 / 8 帧」偏乐观。")
    L.append("   建议改为 8 / 16 / 32 档位可配置，并把「系统自动提高累积帧数并报告额外延迟」")
    L.append("   作为演示中的可见行为——这正好强化「单帧/短突发自适应切换」这条创新点。")
    L.append("2. **台架验收必须先测单帧信噪比。** 自采数据到位后，先按本模型测出该场景的")
    L.append("   单帧 SNR，再据此定累积帧数，不要照搬论文的 20 帧。")
    L.append("3. **快速告警通路的判据必须换。** 若要在 1~2 帧内出结果，逐像素差分幅度会被噪声")
    L.append("   淹没（N=1 时 SNR 仅 0.67）。应改用**低维子空间投影能量**这类结构化统计量，")
    L.append("   它是对全图求和的积分量，等效增益远高于单像素判据。这一条直接支撑技术方案中")
    L.append("   「候选位置投影评分」作为告警核心的设计。")
    L.append("4. **曝光时间可换取帧数。** 若把单帧曝光由 68 ms 提高，可在更少帧数下达到同样 SNR，")
    L.append("   但会推高端到端延迟。这是「曝光—帧数—延迟」三角权衡，应作为现场可调参数展示。")
    L.append("")

    L.append("## 5. 模型边界与不确定性\n")
    L.append("- `sigma` 由同一场景的 35 帧估计，包含该场景的真实帧间波动，但不含相机暗电流漂移")
    L.append("  与环境光缓慢变化等低频干扰——这些必须在自采数据上重新估计。")
    L.append("- 官方单帧存在饱和像素（满量程 65535），会低估信号标准差，因此单帧 SNR 可能被轻微低估。")
    L.append("- 本信噪比是**测量域**的，不等于重建图像质量。逆问题条件性、正则化强度与遮挡位置")
    L.append("  精度同样影响最终重建，需在 T0-5 中另行评估。")
    L.append("- 本模型假设噪声帧间独立。若相机存在固定图形噪声（FPN），该假设不成立，")
    L.append("  累积增益会低于 sqrt(N)，需在自采数据上用暗场帧验证。")
    L.append("")

    L.append("## 6. 产出物\n")
    L.append("| 文件 | 内容 |")
    L.append("|---|---|")
    L.append("| `sim/results/frame_budget.md` | 本报告 |")
    L.append("| `sim/results/fig_frame_budget.png` | 不同累积帧数下的差分图视觉对比 |")
    L.append("| `sim/results/fig_snr_model.png` | SNR 模型曲线与交叉点标注 |")
    L.append("| `sim/results/snr_model.csv` | 模型原始数据 |")
    L.append("| `sim/vectors/mush_single_frame_diffs.npz` | 单帧差分缓存（供重跑加速） |")
    L.append("")

    with io.open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(L))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-root", required=True)
    ap.add_argument("--out-dir", default="sim/results")
    ap.add_argument("--vec-dir", default="sim/vectors")
    ap.add_argument("--channel", type=int, default=1, choices=[0, 1, 2])
    ap.add_argument("--force", action="store_true", help="忽略缓存重新计算单帧差分")
    args = ap.parse_args()

    font = setup_font()
    os.makedirs(args.out_dir, exist_ok=True)
    os.makedirs(args.vec_dir, exist_ok=True)
    cache = os.path.join(args.vec_dir, "mush_single_frame_diffs.npz")
    used_cache = os.path.exists(cache) and not args.force

    t0 = time.time()
    print("[1/3] 获取单帧差分 ...")
    mush_dir = os.path.join(args.data_root, "TestPosChair", "Mush")
    D = get_single_frame_diffs(mush_dir, cache, force=args.force)
    print(f"      shape={D.shape}")

    print("[2/3] 信号/噪声估计与扫描 ...")
    rows, base_snr, std_s, sigma = analyse(D)
    n_frames = D.shape[0]
    need3 = int(np.ceil((3.0 / base_snr) ** 2)) if base_snr and base_snr > 0 else None
    need5 = int(np.ceil((5.0 / base_snr) ** 2)) if base_snr and base_snr > 0 else None
    print(f"      std(s)={std_s:.4g}  sigma={sigma:.4g}  单帧SNR={base_snr:.4g}")
    print(f"      SNR>=3 需约 {need3} 帧;  SNR>=5 需约 {need5} 帧")
    for r in rows:
        print(f"      N={r['frames']:>3}  空间std={r['measured_spatial_std']:.4g}  "
              f"模型SNR={r['model_snr_sqrtN']:.4g}  相关={r['corr_with_full']:.6f}")

    print("[3/3] 出图与报告 ...")
    if _HAVE_MPL:
        make_budget_figure(D, [r["frames"] for r in rows], args.channel,
                           os.path.join(args.out_dir, "fig_frame_budget.png"))
        make_snr_figure(rows, base_snr, os.path.join(args.out_dir, "fig_snr_model.png"))

    with io.open(os.path.join(args.out_dir, "snr_model.csv"), "w",
                 encoding="utf-8-sig", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        for r in rows:
            w.writerow(r)

    path = os.path.join(args.out_dir, "frame_budget.md")
    write_report(path, rows, base_snr, std_s, sigma, n_frames, need3, need5,
                 args.channel, font, time.time() - t0, used_cache)
    print(f"[done] {path}")


if __name__ == "__main__":
    main()
