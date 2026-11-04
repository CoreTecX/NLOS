# -*- coding: utf-8 -*-
"""T0-04 几何前向模型验证。

验证项：

  V1  半影结构对齐    —— 把预测的半影结构与官方测量图对齐，标定图像方向约定
  V2  遮挡物的作用    —— 复现论文 S008「无遮挡物时 A 的行过于相似」这一核心论断
  V3  计算视场        —— 按 S008 的定义计算计算视场，复现 Fig. 3
  V4  可见性实现交叉验证 —— 闭式解 vs 暴力步进采样（论文 L=64 的做法）
  V5  投影能量冒烟测试 —— 在真值位置与其邻域上比较投影能量，确认定位机制可用

产出：
  sim/results/geometry_validation.md
  sim/results/fig_penumbra_alignment.png
  sim/results/fig_conditioning.png
  sim/results/fig_computational_fov.png

用法（在仓库根目录）：
  python src/model/scripts/validate_geometry.py --data-root "<官方 Data 目录>"
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


# ---------------------------------------------------------------- 工具


def zscore(a: np.ndarray) -> np.ndarray:
    a = np.asarray(a, dtype=np.float64)
    s = a.std()
    return (a - a.mean()) / s if s > 0 else a - a.mean()


def corr(a: np.ndarray, b: np.ndarray) -> float:
    x, yv = zscore(a).reshape(-1), zscore(b).reshape(-1)
    return float(np.dot(x, yv) / x.size)


def vdiff2d(a: np.ndarray) -> np.ndarray:
    return a[1:, :] - a[:-1, :]


#: 候选方向约定：名字 -> 作用在规范预测图上的变换
#: 规范预测图的行索引对应 z 递增、列索引对应 x 递增
ORIENTATIONS = {
    "行=z升 列=x升": lambda p: p,
    "行=z升 列=x降": lambda p: p[:, ::-1],
    "行=z降 列=x升": lambda p: p[::-1, :],
    "行=z降 列=x降": lambda p: p[::-1, ::-1],
    "行=x升 列=z升": lambda p: p.T,
    "行=x升 列=z降": lambda p: p.T[:, ::-1],
    "行=x降 列=z升": lambda p: p[::-1, :].T,
    "行=x降 列=z降": lambda p: p[::-1, ::-1].T,
}


def pick_orientation(pred: np.ndarray, meas: np.ndarray):
    """在 8 种方向约定中选出与测量图最相关的一种。"""
    rows = []
    for name, tf in ORIENTATIONS.items():
        q = tf(pred)
        if q.shape != meas.shape:
            continue
        rows.append({"name": name, "corr": corr(q, meas),
                     "corr_d": corr(vdiff2d(q), vdiff2d(meas))})
    rows.sort(key=lambda r: -r["corr"])
    return rows


# ---------------------------------------------------------------- V1


def v1_alignment(setup, meas_rgb, gt_rgb):
    """半影结构对齐 + 完整前向模型对照。"""
    res = {}

    # 纯几何：每个墙点能看到多少比例的隐藏场景
    vf = F.visible_fraction(setup)
    meas_lum = meas_rgb.mean(axis=2)
    res["visible_fraction_orient"] = pick_orientation(vf, meas_lum)

    # 完整前向：用官方真值场景渲染，逐通道与测量比对
    chan = {}
    for c, nm in enumerate("RGB"):
        pred = F.forward_render(setup, gt_rgb[:, :, c].astype(np.float64))
        rows = pick_orientation(pred, meas_rgb[:, :, c])
        chan[nm] = rows
    res["forward_orient"] = chan

    # 采用最优约定，报告最终相关
    best = res["visible_fraction_orient"][0]
    res["best_orientation"] = best["name"]
    res["visible_fraction_corr"] = best["corr"]
    res["visible_fraction_corr_diff"] = best["corr_d"]

    tf = ORIENTATIONS[best["name"]]
    res["forward_corr"] = {
        nm: chan[nm][0]["corr"] for nm in "RGB"
    }
    res["forward_corr_diff"] = {
        nm: chan[nm][0]["corr_d"] for nm in "RGB"
    }
    res["_vf_aligned"] = tf(vf)
    res["_meas_lum"] = meas_lum
    res["_forward_aligned"] = {
        nm: ORIENTATIONS[chan[nm][0]["name"]](
            F.forward_render(setup, gt_rgb[:, :, c].astype(np.float64)))
        for c, nm in enumerate("RGB")
    }
    return res


# ---------------------------------------------------------------- V2


def v2_conditioning(setup):
    """遮挡物对 A 的条件性的影响。"""
    res = {}
    for tag, with_occ in (("有遮挡物", True), ("无遮挡物", False)):
        A = F.build_transport_matrix(setup, with_occluder=with_occ)
        s = F.singular_spectrum(A, top=40)
        res[tag] = {
            "singular_top10": s[:10].tolist(),
            "s1_over_s40": float(s[0] / s[-1]) if s[-1] > 0 else float("inf"),
            "decay_ratio_s10_over_s1": float(s[9] / s[0]) if s.size >= 10 else float("nan"),
            "mean_abs_col_corr": F.mean_abs_column_correlation(A),
            "rank_visible_frac": float((s > s[0] * 1e-6).mean()),
        }
        del A
    return res


# ---------------------------------------------------------------- V3


def v3_computational_fov(setup):
    """按 S008 的定义计算计算视场。"""
    V = F.visibility(setup)
    M = V.shape[0]
    n_vis = V.sum(axis=0)                  # 每个隐藏像素被多少个墙点看到
    both = (n_vis > 0) & (n_vis < M)
    cfo = both.reshape(setup.monitor.shape)
    return {
        "n_scene_pixels": int(V.shape[1]),
        "n_never_visible": int((n_vis == 0).sum()),
        "n_always_visible": int((n_vis == M).sum()),
        "n_computational_fov": int(both.sum()),
        "fraction": float(both.mean()),
        "_mask": cfo,
    }


# ---------------------------------------------------------------- V4


def v4_visibility_crosscheck(setup, n_wall=24, n_scene=18, L=64):
    """闭式解 vs 暴力步进采样：在缩小的配置上逐元素比对。"""
    Yw = setup.wall.y
    Yo = setup.occluder.y
    x0, x1 = setup.occluder.x_range
    z0, z1 = setup.occluder.z_range

    wx = np.linspace(setup.wall.x_min, setup.wall.x_max, n_wall)
    wz = np.linspace(setup.wall.z_min, setup.wall.z_max, n_wall)
    mx = np.linspace(setup.monitor.x_centers.min(), setup.monitor.x_centers.max(), n_scene)
    mz = np.linspace(setup.monitor.z_centers.min(), setup.monitor.z_centers.max(), n_scene)

    # 闭式解
    t = Yo / Yw
    qx = (1 - t) * mx[None, :] + t * wx[:, None]
    qz = (1 - t) * mz[None, :] + t * wz[:, None]
    closed = ~((qx >= x0) & (qx <= x1) & (qz >= z0) & (qz <= z1))

    # 暴力步进：沿连线取 L 个采样点，检查中间段是否落入遮挡物矩形
    brute = np.zeros((n_wall, n_scene), dtype=bool)
    s = np.linspace(0.0, 1.0, L + 2)[1:-1]        # 去掉两个端点
    for j in range(n_wall):
        for i in range(n_scene):
            ys = s * Yw
            xs = mx[i] + s * (wx[j] - mx[i])
            zs = mz[i] + s * (wz[j] - mz[i])
            hit = np.any((ys >= Yo - 1e-9) & (ys <= Yo + 1e-9) &
                         (xs >= x0) & (xs <= x1) & (zs >= z0) & (zs <= z1))
            if not hit:
                # 用最近的采样层近似判断
                k = np.argmin(np.abs(ys - Yo))
                hit = bool((xs[k] >= x0) and (xs[k] <= x1) and
                           (zs[k] >= z0) and (zs[k] <= z1))
            brute[j, i] = not hit

    return {
        "n_wall": n_wall, "n_scene": n_scene, "L": L,
        "agree": float((closed == brute).mean()),
        "n_disagree": int((closed != brute).sum()),
        "total": int(closed.size),
    }


# ---------------------------------------------------------------- V5


def _A_and_y(setup, y2d, differenced):
    """构造 A 与 y，必要时对两者同时施加垂直差分。"""
    A = F.build_transport_matrix(setup)
    n = setup.wall.n
    if differenced:
        D = np.zeros((n - 1, n))
        idx = np.arange(n - 1)
        D[idx, idx] = -1.0
        D[idx, idx + 1] = 1.0
        n2 = n * n
        # 按行差分：(n-1)*n x n*n
        I = np.zeros((n, n))
        A_r = A.reshape(n, n, -1)
        y_r = np.asarray(y2d, dtype=np.float64).reshape(n, n)
        # 先按行差分再展平，与 A 的行块对应
        y_d = (y_r[1:, :] - y_r[:-1, :]).reshape(-1)
        A_d = (A_r[1:, :, :] - A_r[:-1, :, :]).reshape(-1, A.shape[1])
        return A_d, y_d
    return A, np.asarray(y2d, dtype=np.float64).reshape(-1)


def v5_projection_smoke(setup, meas_g, offsets, differenced=False, kappa=0.75):
    """在真值位置与邻域上比较投影能量。"""
    rows = []
    ll0 = setup.occluder.ll.copy()
    for name, off in offsets:
        ll = ll0 + np.asarray(off, dtype=np.float64)
        s2 = F.OpticalSetup(
            wall=setup.wall, monitor=setup.monitor,
            occluder=F.Occluder(ll=ll, size=setup.occluder.size),
            view_angle_correction=setup.view_angle_correction,
        )
        t0 = time.time()
        A, y = _A_and_y(s2, meas_g, differenced)
        e = F.projection_energy(A, y, kappa=kappa)
        rows.append({"name": name, "ll": ll.tolist(), "energy": e,
                     "elapsed": time.time() - t0})
        del A, y
    base = rows[0]["energy"]
    for r in rows:
        r["rel_to_true"] = r["energy"] / base if base else float("nan")
    return rows


# ---------------------------------------------------------------- 出图


def fig_alignment(res, out_png):
    fig, axes = plt.subplots(2, 3, figsize=(14, 8.4), constrained_layout=True)
    vf = res["_vf_aligned"]
    lum = res["_meas_lum"]

    def im(ax, a, title, cmap="inferno"):
        h = ax.imshow(a, cmap=cmap, interpolation="nearest", aspect="auto")
        ax.set_title(title, fontsize=10.5)
        ax.set_xticks([]); ax.set_yticks([])
        fig.colorbar(h, ax=ax, fraction=0.046, pad=0.02)

    im(axes[0, 0], vf, "预测：可见场景比例（纯几何）")
    im(axes[0, 1], lum, "实测：墙面亮度")
    im(axes[0, 2], zscore(lum) - zscore(vf),
       "差异（实测 - 预测，标准化）", cmap="coolwarm")

    im(axes[1, 0], vdiff2d(vf), "预测差分", cmap="coolwarm")
    im(axes[1, 1], vdiff2d(lum), "实测差分", cmap="coolwarm")
    fa = res["_forward_aligned"]["G"]
    im(axes[1, 2], fa, "完整前向渲染（用官方真值场景，G 通道）")

    fig.suptitle(
        f"半影结构对齐（方向约定：{res['best_orientation']}）\n"
        f"可见比例与实测亮度相关 r = {res['visible_fraction_corr']:.3f}；"
        f"差分后 r = {res['visible_fraction_corr_diff']:.3f}；"
        f"完整前向渲染 r = {res['forward_corr']['G']:.3f}",
        fontsize=12)
    fig.savefig(out_png, dpi=130)
    plt.close(fig)


def fig_conditioning(res, out_png):
    fig, ax = plt.subplots(1, 2, figsize=(13, 4.8), constrained_layout=True)
    for tag, mk in (("有遮挡物", "o-"), ("无遮挡物", "s--")):
        s = np.array(res[tag]["singular_top10"])
        ax[0].semilogy(np.arange(1, s.size + 1), s / s[0], mk, label=tag)
    ax[0].set_xlabel("奇异值序号 k")
    ax[0].set_ylabel(r"$\sigma_k / \sigma_1$")
    ax[0].set_title("A 的奇异值衰减：遮挡物是否改善条件性")
    ax[0].grid(alpha=0.3, which="both")
    ax[0].legend(fontsize=9.5)

    names = ["mean_abs_col_corr", "decay_ratio_s10_over_s1"]
    labels = ["列间平均绝对相关系数", r"$\sigma_{10}/\sigma_1$"]
    x = np.arange(len(names))
    w = 0.36
    for i, tag in enumerate(("有遮挡物", "无遮挡物")):
        ax[1].bar(x + (i - 0.5) * w, [res[tag][n] for n in names], w, label=tag)
    ax[1].set_xticks(x)
    ax[1].set_xticklabels(labels, fontsize=9.5)
    ax[1].set_yscale("log")
    ax[1].set_title("可区分性指标对比")
    ax[1].grid(alpha=0.3, axis="y", which="both")
    ax[1].legend(fontsize=9.5)
    fig.savefig(out_png, dpi=130)
    plt.close(fig)


def fig_cfo(res, out_png):
    m = res["_mask"]
    fig, ax = plt.subplots(1, 2, figsize=(12, 4.4), constrained_layout=True)
    h = ax[0].imshow(m, cmap="gray", interpolation="nearest", aspect="auto")
    ax[0].set_title("计算视场掩膜（1 = 对部分相机视场可见、对另一部分被挡）",
                    fontsize=10.5)
    ax[0].set_xlabel("隐藏像素列（x 方向，36）")
    ax[0].set_ylabel("隐藏像素行（z 方向，29）")
    fig.colorbar(h, ax=ax[0], fraction=0.046, pad=0.02)
    ax[1].bar(["计算视场内", "从不被看见", "始终可见"],
              [res["n_computational_fov"], res["n_never_visible"], res["n_always_visible"]])
    ax[1].set_ylabel("隐藏像素个数")
    ax[1].set_title(f"计算视场占比 {res['fraction']:.1%}（共 {res['n_scene_pixels']} 个）",
                    fontsize=10.5)
    ax[1].grid(alpha=0.3, axis="y")
    fig.savefig(out_png, dpi=130)
    plt.close(fig)


# ---------------------------------------------------------------- 报告


def write_report(path, setup, v1, v2, v3, v4, v5, font, elapsed):
    L = []
    L.append("# 几何前向模型验证报告（T0-04）\n")
    L.append(f"- 生成时间：{time.strftime('%Y-%m-%d %H:%M:%S')}")
    L.append(f"- 中文字体：`{font}`；总耗时 {elapsed:.1f} s")
    L.append("- 数据来源：官方 `TestPosChair`，几何参数取自 `chair_test.mat`\n")

    L.append("## 0. 几何配置（从官方配置直接读取，未作假设）\n")
    L.append("| 项目 | 值 |")
    L.append("|---|---|")
    for k, v in setup.summary().items():
        L.append(f"| {k} | {v} |")
    L.append("")
    L.append("三个平面互相平行：显示器 y=0、遮挡物 y=0.594、墙 y=1.03。")
    L.append("这一结构使光线与遮挡物平面的交点有闭式解 `q = (1-t)·m + t·w`（t = y_o / y_w），")
    L.append(f"因此不需要论文 Methods 中的 L=64 步进采样。\n")

    L.append("## 1. V1 半影结构对齐\n")
    L.append("**图像方向约定**是在本项中实测确定的，不是假设的。8 种约定的相关系数：\n")
    L.append("| 方向约定 | 可见比例 r | 差分后 r |")
    L.append("|---|---|---|")
    for r in v1["visible_fraction_orient"]:
        L.append(f"| {r['name']} | {r['corr']:.4f} | {r['corr_d']:.4f} |")
    L.append("")
    L.append(f"采用约定：**{v1['best_orientation']}**\n")
    L.append("| 对照量 | 与实测的相关系数 |")
    L.append("|---|---|")
    L.append(f"| 预测可见比例（纯几何） | {v1['visible_fraction_corr']:.4f} |")
    L.append(f"| 预测可见比例的垂直差分 | {v1['visible_fraction_corr_diff']:.4f} |")
    for nm in "RGB":
        L.append(f"| 完整前向渲染（{nm} 通道，用官方真值场景） | {v1['forward_corr'][nm]:.4f} |")
    for nm in "RGB":
        L.append(f"| 完整前向渲染的垂直差分（{nm} 通道） | {v1['forward_corr_diff'][nm]:.4f} |")
    L.append("")
    L.append("「完整前向渲染」取官方真值场景作为 f、经由本实现的 A 渲染墙面，")
    L.append("因此同时检验了几何与辐射度量。\n")

    L.append("## 2. V2 遮挡物对条件性的作用（复现论文 S008）\n")
    L.append("| 指标 | 有遮挡物 | 无遮挡物 |")
    L.append("|---|---|---|")
    for key in ("decay_ratio_s10_over_s1", "s1_over_s40",
                "mean_abs_col_corr", "rank_visible_frac"):
        L.append(f"| {key} | {v2['有遮挡物'][key]:.6g} | {v2['无遮挡物'][key]:.6g} |")
    L.append("")
    L.append("论文 S008 的论断是「若没有遮挡物，矩阵 A 的各行过于相似，难以稳定求逆」。")
    L.append("上表用列间相关性刻画可区分性：**相关系数越低说明各列越可区分**，")
    L.append("据此可以定量确认遮挡物是否真的改善了逆问题的条件性。\n")

    L.append("## 3. V3 计算视场（复现论文 Fig. 3 的定义）\n")
    L.append("| 项目 | 数值 |")
    L.append("|---|---|")
    L.append(f"| 隐藏像素总数 | {v3['n_scene_pixels']} |")
    L.append(f"| 计算视场内 | {v3['n_computational_fov']}（{v3['fraction']:.1%}） |")
    L.append(f"| 从不被任何墙点看见 | {v3['n_never_visible']} |")
    L.append(f"| 始终可见（无编码信息） | {v3['n_always_visible']} |")
    L.append("")
    L.append("计算视场占比直接决定隐藏场景中「可恢复区域」的大小，")
    L.append("是评估本方案有效视场的第一手依据。\n")

    L.append("## 4. V4 可见性实现的交叉验证\n")
    L.append("| 项目 | 值 |")
    L.append("|---|---|")
    L.append(f"| 网格 | {v4['n_wall']} x {v4['n_scene']} |")
    L.append(f"| 步进采样数 L | {v4['L']} |")
    L.append(f"| 一致率 | {v4['agree']:.6f} |")
    L.append(f"| 不一致元素数 | {v4['n_disagree']} / {v4['total']} |")
    L.append("")
    L.append("闭式解与暴力步进采样在缩小网格上逐元素比对。若一致率高，说明闭式解正确，")
    L.append("可以放心替换论文的高开销采样实现。\n")

    L.append("## 5. V5 投影能量冒烟测试\n")
    if v5:
        L.append("| 候选位置 | 投影能量 | 相对真值 | 单次耗时 s |")
        L.append("|---|---|---|---|")
        for r in v5:
            L.append(f"| {r['name']} | {r['energy']:.8g} | {r['rel_to_true']:.6f} | "
                     f"{r['elapsed']:.2f} |")
        L.append("")
    else:
        L.append("（未执行，见 `--skip-v5`）\n")

    L.append("## 6. 结论与不确定性\n")
    L.append("")

    with io.open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(L))


# ---------------------------------------------------------------- 主流程


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-root", required=True)
    ap.add_argument("--out-dir", default="sim/results")
    ap.add_argument("--skip-v5", action="store_true")
    args = ap.parse_args()

    font = setup_font()
    os.makedirs(args.out_dir, exist_ok=True)
    cfg = os.path.join(args.data_root, "TestPosChair", "chair_test.mat")

    t0 = time.time()
    setup = F.load_setup(cfg)
    print("[setup] 几何配置：")
    for k, v in setup.summary().items():
        print(f"        {k:26s} {v}")

    d = io_mat.load_mat(cfg)
    meas = np.dstack([np.asarray(d[f"meas_{c}"], dtype=np.float64) for c in "rgb"])
    gt = np.asarray(d["ground_truth1"], dtype=np.float64)
    print(f"[data] meas={meas.shape}  ground_truth={gt.shape}")

    print("[V1] 半影结构对齐 ...")
    v1 = v1_alignment(setup, meas, gt)
    print(f"     最优方向约定 = {v1['best_orientation']}")
    print(f"     可见比例 r = {v1['visible_fraction_corr']:.4f}  "
          f"差分 r = {v1['visible_fraction_corr_diff']:.4f}")
    print(f"     完整前向 r = " +
          "  ".join(f"{nm}:{v1['forward_corr'][nm]:.4f}" for nm in "RGB"))

    print("[V2] 遮挡物对条件性的作用 ...")
    v2 = v2_conditioning(setup)
    for tag in ("有遮挡物", "无遮挡物"):
        print(f"     {tag}: 列间平均|r|={v2[tag]['mean_abs_col_corr']:.6f}  "
              f"s10/s1={v2[tag]['decay_ratio_s10_over_s1']:.4f}")

    print("[V3] 计算视场 ...")
    v3 = v3_computational_fov(setup)
    print(f"     计算视场 {v3['n_computational_fov']}/{v3['n_scene_pixels']} "
          f"({v3['fraction']:.1%})  从不被见 {v3['n_never_visible']}  "
          f"始终可见 {v3['n_always_visible']}")

    print("[V4] 可见性交叉验证 ...")
    v4 = v4_visibility_crosscheck(setup)
    print(f"     一致率 {v4['agree']:.6f}  不一致 {v4['n_disagree']}/{v4['total']}")

    v5 = []
    if not args.skip_v5:
        print("[V5] 投影能量冒烟测试（真值位置 + 邻域，G 通道，未差分）...")
        offs = [("真值位置", (0, 0, 0)),
                ("x +2cm", (0.02, 0, 0)), ("x -2cm", (-0.02, 0, 0)),
                ("z +2cm", (0, 0, 0.02)), ("z -2cm", (0, 0, -0.02)),
                ("y +5cm", (0, 0.05, 0)), ("y -5cm", (0, -0.05, 0))]
        v5 = v5_projection_smoke(setup, meas[:, :, 1], offs, differenced=False,
                                 kappa=0.75)
        for r in v5:
            print(f"     {r['name']:12s} E={r['energy']:.8g}  "
                  f"相对={r['rel_to_true']:.6f}  {r['elapsed']:.2f}s")

    if _HAVE_MPL:
        print("[fig] 出图 ...")
        fig_alignment(v1, os.path.join(args.out_dir, "fig_penumbra_alignment.png"))
        fig_conditioning(v2, os.path.join(args.out_dir, "fig_conditioning.png"))
        fig_cfo(v3, os.path.join(args.out_dir, "fig_computational_fov.png"))

    path = os.path.join(args.out_dir, "geometry_validation.md")
    write_report(path, setup, v1, v2, v3, v4, v5, font, time.time() - t0)
    print(f"[done] {path}")


if __name__ == "__main__":
    main()
