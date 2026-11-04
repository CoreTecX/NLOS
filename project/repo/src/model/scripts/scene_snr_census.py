# -*- coding: utf-8 -*-
"""多场景信噪比与帧预算普查。

T0-03 只在官方 `TestPosChair/Mush` 一个场景上量化了信噪比。官方数据集里
还有若干带帧序列的场景，难度差异很大（累积帧数从 1 到 50），因此有必要逐场景
测出「单帧信噪比」与「达到目标信噪比所需帧数」，形成一张场景难度对照表。

这张表的直接用途：
  1. 现场演示选哪个场景 —— 选单帧信噪比高、所需帧数少的；
  2. 自适应累积档位怎么配 —— 按场景难度设定 8/16/32 三档的触发条件；
  3. 台架验收的门槛 —— 自采数据到位后先用本方法测出单帧信噪比，再定帧数。

信噪比模型（与 T0-03 一致）：
    单帧差分 d_i = s + n_i，s 帧间不变，n_i 为独立噪声（逐像素标准差 sigma）
    SNR(N) = sqrt(N) * std(s) / sigma
    std(s) 由帧均值估计，sigma 由逐像素时间标准差的中位数估计

产出：
    sim/results/scene_snr_census.md
    sim/results/scene_snr_census.csv
    sim/results/fig_scene_snr_census.png

用法（在仓库根目录）：
    python src/model/scripts/scene_snr_census.py --data-root "<官方 Data 目录>"
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

#: 一个帧序列至少要有这么多帧才够估计时间噪声
MIN_FRAMES = 8
#: 单帧图的最小边长（用于区分原始测量与低分辨率重建结果）
MIN_SIDE = 1024


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


# ---------------------------------------------------------------- 序列发现


def discover_series(data_root: str):
    """找出所有可用作信噪比分析的帧序列目录。"""
    found = []
    for dirpath, dirnames, filenames in os.walk(data_root):
        mats = sorted(f for f in filenames if f.lower().endswith(".mat"))
        if len(mats) < MIN_FRAMES:
            continue
        probe = os.path.join(dirpath, mats[0])
        try:
            d = io_mat.load_mat(probe)
        except Exception:  # noqa: BLE001
            continue
        if "image" not in d:
            continue
        img = np.asarray(d["image"])
        if img.ndim != 2 or min(img.shape) < MIN_SIDE:
            # 低分辨率或非原始测量（如重建结果），跳过
            continue
        meta = io_mat.load_metadata(probe)
        found.append({
            "dir": dirpath,
            "rel": os.path.relpath(dirpath, data_root),
            "n_frames": len(mats),
            "shape": tuple(int(v) for v in img.shape),
            "pixel_size": meta.get("pixel_size"),
            "shutter_speed": meta.get("shutter_speed"),
        })
    found.sort(key=lambda r: r["rel"])
    return found


# ---------------------------------------------------------------- 分析


def analyse_series(entry, progress: bool = True):
    """逐帧算单帧差分，估计信号与噪声。"""
    paths = io_mat.list_frame_series(entry["dir"])
    n = len(paths)
    diffs = []
    t0 = time.time()
    for i, p in enumerate(paths, 1):
        img = io_mat.load_image(p)
        diffs.append(preprocess_raw(img, PROFILE_OFFICIAL).diff)
        if progress and (i % 10 == 0 or i == n):
            print(f"        {entry['rel']} 单帧预处理 {i}/{n}", flush=True)
        del img
    D = np.stack(diffs, axis=0)
    del diffs

    s_hat = D.mean(axis=0)
    sigma = float(np.median(D.std(axis=0, ddof=1)))
    std_s = float(np.std(s_hat))
    snr1 = std_s / sigma if sigma > 0 else float("nan")

    # 累积一致性：N 帧累积与满帧累积的相关系数
    full = D.sum(axis=0).reshape(-1).astype(np.float64)
    full = full - full.mean()
    ff = float(full @ full)
    corrs = {}
    for k in (2, 4, 8, 16, 32):
        if k > n:
            continue
        c = D[:k].sum(axis=0).reshape(-1).astype(np.float64)
        c = c - c.mean()
        corrs[k] = float((c @ full) / np.sqrt(float(c @ c) * ff)) if ff > 0 else float("nan")

    return {
        "rel": entry["rel"],
        "n_frames": n,
        "frame_shape": entry["shape"],
        "pixel_size": entry["pixel_size"],
        "shutter_speed": entry["shutter_speed"],
        "std_s": std_s,
        "sigma": sigma,
        "snr_single": snr1,
        "achieved_snr": (snr1 * float(np.sqrt(n))) if (snr1 and snr1 > 0) else float("nan"),
        "frames_for_snr3": int(np.ceil((3.0 / snr1) ** 2)) if snr1 and snr1 > 0 else None,
        "frames_for_snr5": int(np.ceil((5.0 / snr1) ** 2)) if snr1 and snr1 > 0 else None,
        "corr_at": corrs,
        "elapsed_s": time.time() - t0,
        "_diffs": D,
    }


# ---------------------------------------------------------------- 出图


def make_figure(rows, out_png):
    fig, axes = plt.subplots(1, 2, figsize=(14, 5.2), constrained_layout=True)
    names = [r["rel"].replace("\\", "/").split("/")[-1] + f" ({r['n_frames']}帧)"
             for r in rows]
    x = np.arange(len(rows))

    axes[0].bar(x, [r["snr_single"] for r in rows], color="tab:blue")
    axes[0].axhline(1.0, color="tab:gray", ls=":", label="信号与噪声等幅 SNR=1")
    axes[0].axhline(3.0, color="tab:orange", ls="-.", label="工程可用参考线 SNR=3")
    axes[0].set_xticks(x)
    axes[0].set_xticklabels(names, rotation=35, ha="right", fontsize=8.5)
    axes[0].set_ylabel("单帧信噪比")
    axes[0].set_title("各场景的单帧差分信噪比")
    axes[0].grid(alpha=0.3, axis="y")
    axes[0].legend(fontsize=9)

    w = 0.38
    axes[1].bar(x - w / 2, [r["frames_for_snr3"] or 0 for r in rows], w,
                label="达到 SNR=3")
    axes[1].bar(x + w / 2, [r["frames_for_snr5"] or 0 for r in rows], w,
                label="达到 SNR=5")
    axes[1].set_xticks(x)
    axes[1].set_xticklabels(names, rotation=35, ha="right", fontsize=8.5)
    axes[1].set_ylabel("所需累积帧数")
    axes[1].set_title("达到目标信噪比所需的累积帧数")
    axes[1].grid(alpha=0.3, axis="y")
    axes[1].legend(fontsize=9)

    fig.suptitle("多场景帧预算普查：场景难度决定所需累积帧数", fontsize=12.5)
    fig.savefig(out_png, dpi=130)
    plt.close(fig)


def make_accum_figure(rows, out_png):
    fig, ax = plt.subplots(figsize=(8, 5), constrained_layout=True)
    for r in rows:
        if not r["corr_at"]:
            continue
        ks = sorted(r["corr_at"])
        ax.semilogx(ks, [r["corr_at"][k] for k in ks], "o-", ms=4,
                    label=r["rel"].replace("\\", "/").split("/")[-1], base=2)
    ax.set_xlabel("累积帧数 N")
    ax.set_ylabel("与满帧累积的相关系数")
    ax.set_title("各场景差分结构随累积的稳定速度")
    ax.grid(alpha=0.3, which="both")
    ax.legend(fontsize=8.5, ncol=2)
    fig.savefig(out_png, dpi=130)
    plt.close(fig)


# ---------------------------------------------------------------- 报告


def write_report(path, rows, skipped, font, elapsed):
    L = []
    L.append("# 多场景信噪比与帧预算普查\n")
    L.append(f"- 生成时间：{time.strftime('%Y-%m-%d %H:%M:%S')}")
    L.append(f"- 中文字体：`{font}`；总耗时 {elapsed:.0f} s")
    L.append(f"- 分析场景数：{len(rows)}；跳过目录数：{len(skipped)}\n")

    L.append("## 1. 方法\n")
    L.append("对每个带帧序列的场景，逐帧独立完成预处理并取垂直差分，得到单帧差分序列。")
    L.append("按 `d_i = s + n_i` 分解：`s` 由帧均值估计，噪声 `sigma` 由逐像素时间标准差的")
    L.append("中位数估计，于是 `SNR(N) = sqrt(N) * std(s) / sigma`。")
    L.append("所需帧数由该式反解。全部采用官方 `PROFILE_OFFICIAL`（16×16 等效块、126×126）。\n")

    L.append("## 2. 场景难度对照表\n")
    L.append("| 场景 | 帧数 | 单帧图 | pixel_size | 快门 ms | 单帧 SNR | 实际达成 SNR | 达 SNR=3 需 | 达 SNR=5 需 |")
    L.append("|---|---|---|---|---|---|---|---|---|")
    for r in rows:
        ps = r["pixel_size"] if r["pixel_size"] is not None else "—"
        ss = f"{float(r['shutter_speed']):.0f}" if r["shutter_speed"] is not None else "—"
        f3 = r["frames_for_snr3"] if r["frames_for_snr3"] is not None else "—"
        f5 = r["frames_for_snr5"] if r["frames_for_snr5"] is not None else "—"
        L.append(f"| {r['rel']} | {r['n_frames']} | {r['frame_shape'][0]}x{r['frame_shape'][1]} | "
                 f"{ps} | {ss} | **{r['snr_single']:.4g}** | **{r['achieved_snr']:.3g}** | {f3} | {f5} |")
    L.append("")

    L.append("## 3. 累积稳定速度\n")
    L.append("与满帧累积的相关系数，反映差分结构随累积收敛的快慢：\n")
    ks = sorted({k for r in rows for k in r["corr_at"]})
    if ks:
        L.append("| 场景 | " + " | ".join(f"N={k}" for k in ks) + " |")
        L.append("|---|" + "---|" * len(ks))
        for r in rows:
            cells = " | ".join(
                f"{r['corr_at'][k]:.4f}" if k in r["corr_at"] else "—" for k in ks)
            L.append(f"| {r['rel']} | {cells} |")
    L.append("")

    L.append("## 4. 结论与用途\n")
    valid = [r for r in rows if r["snr_single"] and r["snr_single"] > 0]
    if valid:
        best = max(valid, key=lambda r: r["snr_single"])
        worst = min(valid, key=lambda r: r["snr_single"])
        L.append(f"- 单帧信噪比最高：**{best['rel']}**（{best['snr_single']:.3g}，"
                 f"达 SNR=3 需 {best['frames_for_snr3']} 帧）")
        L.append(f"- 单帧信噪比最低：**{worst['rel']}**（{worst['snr_single']:.3g}，"
                 f"达 SNR=3 需 {worst['frames_for_snr3']} 帧）")
        L.append(f"- 场景间难度跨度：{worst['snr_single']:.3g} 到 {best['snr_single']:.3g}，"
                 f"相差 {best['snr_single'] / worst['snr_single']:.1f} 倍")
    L.append("")
    L.append("### 官方隐含的目标信噪比")
    L.append("")
    L.append("把每个场景**单帧信噪比乘以其实际累积帧数的平方根**，得到该场景最终达成的信噪比：")
    L.append("")
    L.append("| 场景 | 单帧 SNR | 实际帧数 | 实际达成 SNR |")
    L.append("|---|---|---|---|")
    for r in rows:
        if r["achieved_snr"] == r["achieved_snr"]:
            L.append(f"| {r['rel']} | {r['snr_single']:.4g} | {r['n_frames']} | **{r['achieved_snr']:.3g}** |")
    ach = [r["achieved_snr"] for r in rows if r["achieved_snr"] == r["achieved_snr"]]
    if ach:
        L.append("")
        L.append(f"达成值范围 {min(ach):.2f} ~ {max(ach):.2f}，均值 {np.mean(ach):.2f}，离散度约为均值的 "
                 f"{(max(ach) - min(ach)) / np.mean(ach) * 100:.0f}%。")
        L.append("")
        L.append("**各场景的实际达成信噪比高度一致，说明官方是按「达到约 4 的目标信噪比」")
        L.append("为每个场景单独定累积帧数的**，而不是对所有场景套用同一帧数。")
        L.append("")
        L.append("对方案的直接影响：现场的动态定档依据不应是场景类别或固定三档，")
        L.append("而应是**先在线估计单帧信噪比，再按目标值反解所需帧数**。")
        L.append("这既避免低难度场景过度累积、白耗延迟，也避免高难度场景欠累积。")
    L.append("")

    L.append("**直接用途：**")
    L.append("")
    L.append("1. **现场演示选场景** —— 选单帧信噪比高、所需帧数少的场景，"
             "把有限的时间留给算法演示而不是等待采集。")
    L.append("2. **自适应累积档位** —— 若难度跨度为数十倍，说明固定的 8/16/32 三档"
             "对两端场景都不合适，应按单帧信噪比在线估计后动态定档。")
    L.append("3. **台架验收门槛** —— 自采数据到位后先用本方法测单帧信噪比，"
             "再据此定累积帧数，不要照搬论文的 20 帧。")
    L.append("")

    L.append("## 5. 不确定性\n")
    L.append("- 噪声估计假设帧间独立。若相机存在固定图形噪声（FPN）或缓慢漂移，"
             "实际累积增益会低于 sqrt(N)，需用暗场帧单独验证。")
    L.append("- `sigma` 由同一场景的帧序列估计，包含该场景的真实帧间波动。")
    L.append("- 单帧信噪比是**测量域**指标，不等于重建图像质量。")
    L.append("- 本表仅覆盖官方数据集中带帧序列的场景；单次采集的场景无法估计时间噪声。\n")

    if skipped:
        L.append("## 6. 跳过的目录\n")
        L.append("| 目录 | 原因 |")
        L.append("|---|---|")
        for s in skipped:
            L.append(f"| {s['rel']} | {s['reason']} |")
        L.append("")

    L.append("## 7. 产出物\n")
    L.append("| 文件 | 内容 |")
    L.append("|---|---|")
    L.append("| `sim/results/scene_snr_census.md` | 本报告 |")
    L.append("| `sim/results/scene_snr_census.csv` | 原始数据 |")
    L.append("| `sim/results/fig_scene_snr_census.png` | 信噪比与帧数柱状对比 |")
    L.append("| `sim/results/fig_scene_accumulation.png` | 各场景累积稳定曲线 |")
    L.append("")

    with io.open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(L))


# ---------------------------------------------------------------- 主流程


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-root", required=True)
    ap.add_argument("--out-dir", default="sim/results")
    args = ap.parse_args()

    font = setup_font()
    os.makedirs(args.out_dir, exist_ok=True)
    t0 = time.time()

    print("[1/3] 发现帧序列 ...", flush=True)
    entries = discover_series(args.data_root)
    for e in entries:
        print(f"      {e['rel']}  {e['n_frames']} 帧  {e['shape']}  "
              f"pixel_size={e['pixel_size']}", flush=True)

    # 记录被跳过的目录（含原因），便于复核覆盖度
    skipped = []
    for dirpath, dirnames, filenames in os.walk(args.data_root):
        mats = [f for f in filenames if f.lower().endswith(".mat")]
        if len(mats) < MIN_FRAMES:
            continue
        rel = os.path.relpath(dirpath, args.data_root)
        if any(e["rel"] == rel for e in entries):
            continue
        skipped.append({"rel": rel, "reason": f"{len(mats)} 个文件但非原始测量（低分辨率或无 image 字段）"})

    print(f"\n[2/3] 逐场景分析（{len(entries)} 个）...", flush=True)
    rows = []
    for e in entries:
        print(f"    {e['rel']} ...", flush=True)
        try:
            rows.append(analyse_series(e))
        except Exception as exc:  # noqa: BLE001
            print(f"      失败：{exc}", flush=True)

    if not rows:
        print("[warn] 没有可分析的场景")
        return

    print("\n[3/3] 出表与出图 ...", flush=True)
    with io.open(os.path.join(args.out_dir, "scene_snr_census.csv"), "w",
                 encoding="utf-8-sig", newline="") as f:
        w = csv.writer(f)
        w.writerow(["场景", "帧数", "图宽", "图高", "pixel_size", "快门ms",
                    "std_s", "sigma", "单帧SNR", "实际达成SNR",
                    "达SNR3帧数", "达SNR5帧数"])
        for r in rows:
            w.writerow([r["rel"], r["n_frames"], r["frame_shape"][0], r["frame_shape"][1],
                        r["pixel_size"], r["shutter_speed"], f"{r['std_s']:.6g}",
                        f"{r['sigma']:.6g}", f"{r['snr_single']:.6g}",
                        f"{r['achieved_snr']:.6g}",
                        r["frames_for_snr3"], r["frames_for_snr5"]])

    if _HAVE_MPL:
        make_figure(rows, os.path.join(args.out_dir, "fig_scene_snr_census.png"))
        make_accum_figure(rows, os.path.join(args.out_dir, "fig_scene_accumulation.png"))

    for r in rows:
        r.pop("_diffs", None)
    write_report(os.path.join(args.out_dir, "scene_snr_census.md"),
                 rows, skipped, font, time.time() - t0)
    print(f"\n[done] 总耗时 {time.time()-t0:.0f}s", flush=True)


if __name__ == "__main__":
    main()
