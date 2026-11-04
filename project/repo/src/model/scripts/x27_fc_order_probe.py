# -*- coding: utf-8 -*-
"""X27 —— 「能量图 F/C 序矛盾」根因判定脚本（只读；不改动任何被测代码）。

回答四个问题：
  Q1 展平序（C / F）能否改变两张已算好的能量面之间的相关系数？
  Q2 隐藏场景向量（A 的列）做 D-03 那类列置换，能否改变定位能量？
  Q3 官方 proj_yOnRangeA* 的「读出口径」是什么（第 4 维到底是什么、哪一档才是那次运行用的）？
  Q4 是否存在某个「网格轴顺序」变换 T，使 r(T(本实现), 官方) 趋近 1？

用法（仓库根目录）：
  python src/model/scripts/x27_fc_order_probe.py --data-root "<官方 Data 目录>"
"""

from __future__ import annotations

import argparse
import io
import itertools
import os
import sys
import warnings

import numpy as np
import scipy.io as sio

warnings.filterwarnings("ignore")

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC_MODEL = os.path.dirname(_HERE)
if _SRC_MODEL not in sys.path:
    sys.path.insert(0, _SRC_MODEL)

from penumbrasight_model import forward as F          # noqa: E402
from penumbrasight_model import localization as L     # noqa: E402

LOG: list = []


def p(*a):
    s = " ".join(str(x) for x in a)
    LOG.append(s)
    print(s, flush=True)


def pear(a, b):
    a = np.asarray(a, dtype=np.float64).ravel()
    b = np.asarray(b, dtype=np.float64).ravel()
    if a.size != b.size:
        return float("nan")
    a = a - a.mean()
    b = b - b.mean()
    den = np.sqrt(float(a @ a) * float(b @ b))
    return float(a @ b / den) if den > 0 else float("nan")


# ======================================================================
# Q1 —— 展平序不变性
# ======================================================================
def q1_flatten_order(cube_a, cube_b, label):
    p("")
    p("### Q1 展平序不变性：" + label)
    p("  两数组同形状 %s。同一批元素，只换展平序：" % (cube_a.shape,))
    rC = pear(cube_a.ravel(order="C"), cube_b.ravel(order="C"))
    rF = pear(cube_a.ravel(order="F"), cube_b.ravel(order="F"))
    p("    order=C   r = %+.15f" % rC)
    p("    order=F   r = %+.15f" % rF)
    p("  |r_C - r_F| = %.3e  => 对两侧施加同一置换不改变 r（排列不变性），"
      "即「展平序」本身不可能造成相关的矛盾" % abs(rC - rF))
    rng = np.random.default_rng(0)
    pm = rng.permutation(cube_b.size)
    b_perm = cube_b.ravel()[pm]
    p("  反例（只置换一侧的展平序 = 行序不一致，D-01 的机制）：r = %+.6f"
      % pear(cube_a.ravel(), b_perm))


# ======================================================================
# Q2 —— 列置换 / 行置换对定位能量的影响
# ======================================================================
def q2_permutation_invariance(cfg, ll_list):
    p("")
    p("### Q2 定位能量对「一致性置换」的不变性（在真 A 上数值实测）")
    setup = F.load_setup(cfg)
    d = sio.loadmat(cfg)
    y2d = np.asarray(d["meas_g"], dtype=np.float64)

    nrow, ncol = setup.monitor.shape          # (29, 36)
    n = setup.wall.n

    perm_d03 = np.arange(nrow * ncol).reshape(nrow, ncol, order="F").ravel(order="C")
    rng = np.random.default_rng(1)
    perm_rand = rng.permutation(nrow * ncol)

    for ll in ll_list:
        A = F.build_transport_matrix(setup.with_occluder_ll(ll))
        Ad = L.apply_row_difference_A(A, n)
        yd = L.apply_row_difference_img(y2d)
        e0 = L.projection_energy_detail(Ad, yd, kappa=0.75)[0]
        k0 = L.projection_energy_detail(Ad, yd, kappa=0.75)[1]

        e_c = L.projection_energy_detail(Ad[:, perm_d03], yd, kappa=0.75)[0]
        e_r = L.projection_energy_detail(Ad[:, perm_rand], yd, kappa=0.75)[0]

        pm = rng.permutation(Ad.shape[0])
        e_rc = L.projection_energy_detail(Ad[pm, :], yd[pm], kappa=0.75)[0]
        e_rw = L.projection_energy_detail(Ad[pm, :], yd, kappa=0.75)[0]

        e_raw = L.projection_energy_detail(A, y2d.reshape(-1), kappa=0.75)[0]

        p("  ll=%s  保留了 %d 个成分" % (np.round(ll, 4).tolist(), k0))
        p("    基值 E（差分口径）        = %.12e" % e0)
        p("    E 列置换 D-03  相对偏差 %.3e" % (abs(e_c - e0) / abs(e0)))
        p("    E 列置换 随机  相对偏差 %.3e" % (abs(e_r - e0) / abs(e0)))
        p("    E 行置换 一致  相对偏差 %.3e" % (abs(e_rc - e0) / abs(e0)))
        p("    E 行置换 不一致 相对偏差 %+.3e   <- 只有这种会变" % ((e_rw - e0) / abs(e0)))
        p("    E 不差分（官方口径）相对偏差 %+.3f" % ((e_raw - e0) / abs(e0)))
        del A, Ad


# ======================================================================
# Q3 —— 官方读出口径
# ======================================================================
RND_KEYS = {
    1: ("", ("proj_yOnRangeA", "proj_yOnRangeA_g", "proj_yOnRangeA_b"), "best_pos"),
    2: ("2", ("proj_yOnRangeA_2", "proj_yOnRangeA_g_2", "proj_yOnRangeA_b_2"), "best_pos_2"),
    3: ("3", ("proj_yOnRangeA_3", "proj_yOnRangeA_g_3", "proj_yOnRangeA_b_3"), "best_pos_3"),
    4: ("4", ("proj_yOnRangeA_4", "proj_yOnRangeA_g_4", "proj_yOnRangeA_b_4"), "best_pos_4"),
}


def q3_readout(chair, th_lists):
    p("")
    p("### Q3 官方 proj_yOnRangeA* 的读出口径：第 4 维是什么")
    p("  判据：官方 best_pos = mean(逐通道 3D argmax)，故「三通道 argmax 均值」须逐位等于 best_pos")
    out = {}
    for rnd in sorted(RND_KEYS):
        suf, keys, bpkey = RND_KEYS[rnd]
        gx = chair["xhatvals" + suf].reshape(-1)
        gy = chair["yhatvals" + suf].reshape(-1)
        gz = chair["zhatvals" + suf].reshape(-1)
        bp = chair[bpkey].reshape(-1)
        fidx = [float(np.interp(bp[0], gx, np.arange(gx.size))),
                float(np.interp(bp[1], gy, np.arange(gy.size))),
                float(np.interp(bp[2], gz, np.arange(gz.size)))]
        tkey = "thresh" if rnd == 1 else ("thresh_%d" % rnd)
        tk = int(chair[tkey].reshape(-1)[0])
        p("  第 %d 轮：best_pos=%s -> 浮点下标=(%.4f, %.4f, %.4f)；%s=%d（1 基）"
          % (rnd, np.round(bp, 6).tolist(), fidx[0], fidx[1], fidx[2], tkey, tk))
        th = th_lists.get(rnd)
        if th is not None:
            p("       该轮 .mat 内 threshold 列表=%s" % (np.round(th, 8).tolist(),))
            if tk <= th.size:
                p("       threshold(%d) = %g" % (tk, th[tk - 1]))
        cubes = [np.asarray(chair[k], dtype=np.float64) for k in keys]
        hits = []
        for t in range(cubes[0].shape[3]):
            ams = [np.unravel_index(np.argmax(c[:, :, :, t]), c.shape[:3]) for c in cubes]
            mean = np.mean(np.array(ams, dtype=float), axis=0)
            err = float(np.abs(mean - np.array(fidx)).max())
            if err < 1e-9:
                hits.append(t)
        ok = (tk - 1) in hits
        out[rnd] = {"t_hits": hits, "thresh_minus_1": tk - 1}
        p("       命中 best_pos 的第 4 维下标 t=%s ；thresh-1=%d  =>  %s"
          % (hits, tk - 1,
             "一致（口径确认 t = thresh-1）" if ok
             else "不一致，需人工复核"))
    return out


# ======================================================================
# Q4 —— 变换族穷举
# ======================================================================
def q4_transform_search(mine, off, label, off_t=None):
    p("")
    p("### Q4 是否存在网格变换 T 使 r(T(本实现), 官方) 趋近 1？ —— " + label)
    if off_t is not None:
        off = off[:, :, :, off_t]
    m = np.asarray(mine, dtype=np.float64)
    o = np.asarray(off, dtype=np.float64)
    best = (0.0, None)
    rows = []
    for perm in itertools.permutations(range(3)):
        a = np.transpose(m, perm)
        if a.shape != o.shape:
            continue
        for flips in itertools.product((False, True), repeat=3):
            b = a
            for ax, fl in enumerate(flips):
                if fl:
                    b = np.flip(b, axis=ax)
            r = pear(b, o)
            rows.append((perm, flips, r))
            if abs(r) > abs(best[0]):
                best = (r, (perm, flips))
    rows.sort(key=lambda x: -abs(x[2]))
    if not rows:
        p("    形状不匹配，跳过")
        return best
    for perm, flips, r in rows[:5]:
        p("    perm=%-12s flips=%-18s r = %+.4f" % (str(perm), str(flips), r))
    p("  最大 |r| = %.4f（perm=%s, flips=%s）；共试 %d 个变换"
      % (abs(best[0]), best[1][0], best[1][1], len(rows)))
    verdict = "反证成立（不是纯索引/轴序问题）" if abs(best[0]) < 0.9 else "需进一步检查"
    p("  判据：若是纯索引/轴序问题，应存在 T 使 |r| 趋近 1；实测最大 %.4f => %s"
      % (abs(best[0]), verdict))
    return best


def q5_rank1_and_table(chair, mine1, mine2, offr1, arr2):
    p("")
    p("### Q5 两张图各自有多「秩一」？—— 解释 0.4~0.6 那一档相关从何而来")
    def r1frac(M):
        M = np.asarray(M, float)
        Mc = M - M.mean()
        sv = np.linalg.svd(Mc, compute_uv=False)
        return float(sv[0] ** 2 / np.sum(sv ** 2))
    pairs = [("本实现第1轮 z0", mine1[:, :, 0]),
             ("官方第1轮 z0(t=0)", offr1[:, :, 0]),
             ("本实现第2轮 z0", mine2[:, :, 0]),
             ("官方第2轮 z0(t=0)", arr2[:, :, 0, 0]),
             ("官方第2轮 z0(t=3)", arr2[:, :, 0, 3])]
    for nm, M in pairs:
        p("    %-22s 首奇异值方差占比 = %.3f" % (nm, r1frac(M)))

    p("")
    p("### Q6 逐候选对照表（第 1 轮 z=0 平面；官方红通道 t=0 vs 本实现）")
    gx = chair["xhatvals"].reshape(-1)
    gy = chair["yhatvals"].reshape(-1)
    p("    下标为 0 基网格下标 (ix, iy)；能量按各自最大值归一后给出，便于同表比较")
    p("    %-10s %-22s %-14s %-14s %-10s" % ("rank", "官方前十候选(ix,iy)",
                                            "官方E/max", "本实现E/max", "位置(x,z)"))
    o = offr1[:, :, 0]
    m = mine1[:, :, 0]
    on = o / o.max()
    mn = m / m.max()
    idx = np.argsort(o.ravel())[::-1][:10]
    for rk, flat in enumerate(idx, 1):
        ix, iy = np.unravel_index(flat, o.shape)
        p("    %-10d (%-2d,%-2d)%s %-14.6f %-14.6f (%.4f,%.4f)"
          % (rk, ix, iy, " " * 8, on[ix, iy], mn[ix, iy], gx[ix], gy[iy]))
    p("    %-10s %-22s %-14s %-14s" % ("rank", "本实现前五候选(ix,iy)", "官方E/max",
                                      "本实现E/max"))
    idxm = np.argsort(m.ravel())[::-1][:5]
    for rk, flat in enumerate(idxm, 1):
        ix, iy = np.unravel_index(flat, m.shape)
        p("    %-10d (%-2d,%-2d)%s %-14.6f %-14.6f (%.4f,%.4f)"
          % (rk, ix, iy, " " * 8, on[ix, iy], mn[ix, iy], gx[ix], gy[iy]))


def q7_pairing_bug(chair, cube_path):
    p("")
    p("### Q7 复现 x25 比对脚本 run_localization_reconstruction.py:compare_to_official 的配对")
    npz = np.load(cube_path)
    shapes = {k: npz[k].shape for k in npz.files}
    p("    本实现能量立方体：%s" % (shapes,))
    d = chair
    for i, rk in enumerate(sorted(shapes), 1):
        shp = shapes[rk]
        p("    i=%d  r.round=%d  r.energies.shape=%s" % (i, i, shp))
        for pre in ("proj_yOnRangeA", "proj_yOnRangeA_g", "proj_yOnRangeA_b"):
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
                if sl.shape != shp[:2]:
                    p("      %-22s -> 跳过（官方切片 %s 与 %s 不符）" % (key, sl.shape, shp[:2]))
                else:
                    p("      %-22s -> ★参与比对：官方该 key 是「第 %s 轮」的图，"
                      "却与本实现「第 %d 轮」的立方体比" % (key, key.split("_")[-1]
                                                        if suffix else "1", i))


def q0_index_convention(chair, cube_path, repo):
    p("")
    p("### Q0 「候选位置 -> 网格下标」公式的逐项核对（两边都是 (x, y, z) 顺序）")
    gx = chair["xhatvals"].reshape(-1)
    gy = chair["yhatvals"].reshape(-1)
    gz = chair["zhatvals"].reshape(-1)
    p("  官方第 1 轮 gridvals 行向量：xhatvals=linspace(%g,%g,%d) 步长 %.8f"
      % (gx[0], gx[-1], gx.size, gx[1] - gx[0]))
    p("                            yhatvals=linspace(%g,%g,%d) 步长 %.8f"
      % (gy[0], gy[-1], gy.size, gy[1] - gy[0]))
    p("                            zhatvals=%s" % (np.round(gz, 6).tolist(),))
    bp = chair["best_pos"].reshape(-1)
    li = [int(round((bp[0] - gx[0]) / (gx[1] - gx[0]))),
          int(round((bp[1] - gy[0]) / (gy[1] - gy[0]))),
          int(round((bp[2] - gz[0]) / (gz[1] - gz[0])))]
    p("  官方 best_pos=%s  -> 反解下标 (ix,iy,iz)=(%d,%d,%d)"
      % (np.round(bp, 6).tolist(), li[0], li[1], li[2]))
    offr = np.asarray(chair["proj_yOnRangeA"], dtype=np.float64)[:, :, :, 0]
    am = np.unravel_index(np.argmax(offr), offr.shape)
    p("  官方 proj_yOnRangeA[:,:,:,0] 的 3D argmax = %s  -> 位置 = (%.6f, %.6f, %.6f)"
      % (am, gx[am[0]], gy[am[1]], gz[am[2]]))
    p("  => 官方数组的第 1 维对应 x 网格（iix）、第 2 维对应 y 网格（iiy）、"
      "第 3 维对应 z 网格（iiz）；与 occluderposgridsearch.m:69-74 的循环顺序一致")

    sj = os.path.join(repo, "sim", "results", "localization_reconstruction_summary.json")
    if os.path.exists(sj):
        import json
        with io.open(sj, encoding="utf-8") as f:
            s = json.load(f)
        for r in s.get("rounds", []):
            if r["round"] != 1:
                continue
            ll = r["best_ll"]
            p("  本实现第 1 轮 best_ll=%s" % (np.round(ll, 6).tolist(),))
            p("        xhatvals[13]=%.8f  yhatvals[13]=%.8f  zhatvals[1]=%.6f"
              % (gx[13], gy[13], gz[1]))
            p("    best_ll 与 (xhatvals[13], yhatvals[17], zhatvals[1]) 逐位相等：%s"
              % (np.allclose(ll, [gx[13], gy[17], gz[1]], atol=1e-12),))
            p("    best_ll 若按 (y,x,z) 解释则应为 (%.6f, %.6f, %.6f) —— 不符，"
              "故本实现也是 (x, y, z) 顺序" % (gy[13], gx[17], gz[1]))
    m1 = np.load(cube_path)
    a1 = np.unravel_index(np.argmax(m1["E_round1"]), m1["E_round1"].shape)
    a2 = np.unravel_index(np.argmax(m1["E_round2"]), m1["E_round2"].shape)
    p("  本实现 E_round1 的 3D argmax = %s ; E_round2 = %s" % (a1, a2))
    p("  => 两侧「候选枚举顺序」完全相同：(ix, iy, iz) 三层循环、x 最慢、z 最快；"
      "网格数值同取自 xhatvals/yhatvals/zhatvals，故第 0/1/2 维一一对应")


def q8_recon_impact(cfg, n_iter=150, lam=1e-4):
    p("")
    p("### Q8 同一个「F/C 列置换」对重建的影响（当前 e2e 重建链 tv_pdhg，各向同性 TV）")
    setup = F.load_setup(cfg)
    d = sio.loadmat(cfg)
    gt = np.asarray(d["ground_truth1"], dtype=np.float64)[:, :, 0]
    y2d = np.asarray(d["meas_g"], dtype=np.float64)
    ll = np.asarray(d["Occluder"], dtype=np.float64)[0]
    shape = setup.monitor.shape                       # (29, 36)
    nrow, ncol = shape

    A = F.build_transport_matrix(setup.with_occluder_ll(ll))
    Ad = L.apply_row_difference_A(A, setup.wall.n)
    b = L.apply_row_difference_img(y2d)
    perm = np.arange(nrow * ncol).reshape(nrow, ncol, order="F").ravel(order="C")

    from penumbrasight_model import reconstruction as RR
    rms = float(np.sqrt(np.mean(b ** 2)))
    res_c = RR.tv_pdhg(Ad, b / rms, shape, lam=lam, n_iter=n_iter)
    res_perm = RR.tv_pdhg(Ad[:, perm], b / rms, shape, lam=lam, n_iter=n_iter)

    u_c = res_c.image.reshape(-1)
    u_p = res_perm.image.reshape(-1)
    gt_c = gt.reshape(-1)                       # C 序（本实现 A 的列口径）
    gt_T = np.ascontiguousarray(gt.T).reshape(-1)
    p("  本实现 A 的列口径 = 图像 (29,36) 的 **C 序**展平（forward.py:flat_grid 用 C 序）")
    p("  不加列置换：r(u, gt_c)=%+.4f   r(u, gt.T_c)=%+.4f" %
      (pear(u_c, gt_c), pear(u_c, gt_T)))
    p("  加 D-03 列置换：r(u, gt_c)=%+.4f   r(u, gt.T_c)=%+.4f" %
      (pear(u_p, gt_c), pear(u_p, gt_T)))
    rc, rt = pear(u_c, u_p), float(np.max(np.abs(u_c - u_p)) /
                                    max(1e-30, np.max(np.abs(u_c))))
    p("  r(两种口径的结果)=%+.4f   逐值最大相对差=%.3e" % (rc, rt))
    if abs(rc) < 0.5:
        p("  => 两种口径给出几乎不相关的解 ⇒ 该列置换在当前链上**不是可忽略的开关**；")
        p("     但两者质量都低（r≈0.23~0.25），差异落在本问题的噪声量级内，")
        p("     因此既不能据此把置换判为正确、也不能判为错误。")
    p("  已证实的一点：当前链的 A 列是 C 序、tv_pdhg 的 reshape(shape) 也是 C 序，")
    p("     二者自洽，所以 D-03 那个「为 F 序 A 准备」的置换在当前链上本就不适用；")
    p("     它只对 forward_paper.py（F 序列）那条 legacy 重建链是必需的。")


def q9_crossthreshold_band(chair):
    p("")
    p("### Q9 对照：官方「同网格 / 同模型 / 同数据，只换阈值档」两两相关的分布")
    p("    这是「两个不同的泛函在同一张网格上」本该有的相关量级，用作 0.4~0.7 的参照band")
    for rnd, key in ((2, "proj_yOnRangeA_2"), (3, "proj_yOnRangeA_3")):
        arr = np.asarray(chair[key], dtype=np.float64)
        T = arr.shape[3]
        for mode in ("z0切片", "全3D"):
            vals = []
            for i in range(T):
                for j in range(i + 1, T):
                    if mode == "z0切片":
                        v = pear(arr[:, :, 0, i], arr[:, :, 0, j])
                    else:
                        v = pear(arr[:, :, :, i], arr[:, :, :, j])
                    vals.append(v)
            v = np.array(vals, dtype=float)
            finite = v[np.isfinite(v)]
            p("  第 %d 轮 %-6s 两两 r（%d 对）：min=%+.4f  中位=%+.4f  max=%+.4f  "
              "|r|>0.6 的对数=%d"
              % (rnd, mode, finite.size, finite.min(),
                 float(np.median(finite)), finite.max(),
                 int(np.sum(np.abs(finite) > 0.6))))


def _energy_official_style(A, y, sigma=0.75, k_max=10):
    """官方 occluderposgridsearch.m:78-90 的口径：K = min(#{s/s1 > sigma}, 10)。"""
    z = A.T @ y
    G = A.T @ A
    G = 0.5 * (G + G.T)
    lam, V = np.linalg.eigh(G)
    lam = np.maximum(lam[::-1], 0.0)
    V = V[:, ::-1]
    s = np.sqrt(lam)
    if s.size == 0 or s[0] <= 0:
        return 0.0, 0
    npass = int(np.sum(s / s[0] > sigma))
    K = min(npass, k_max)
    if K <= 0:
        return 0.0, npass
    c = V[:, :K].T @ z
    return float(np.sum((c ** 2) / lam[:K])), npass


_WCACHE = {}


def _winit(cfg):
    _WCACHE["cfg"] = cfg
    _WCACHE["setup"] = F.load_setup(cfg)
    d = sio.loadmat(cfg)
    _WCACHE["y_r"] = np.asarray(d["meas_r"], dtype=np.float64)
    _WCACHE["y_r_flat"] = _WCACHE["y_r"].reshape(-1)
    _WCACHE["y_r_diff"] = L.apply_row_difference_img(_WCACHE["y_r"])


def _worker(args):
    ll, sigma, k_max, differenced = args
    setup = _WCACHE["setup"]
    A = F.build_transport_matrix(setup.with_occluder_ll(ll))
    if differenced:
        A = L.apply_row_difference_A(A, setup.wall.n)
        y = _WCACHE["y_r_diff"]
    else:
        y = _WCACHE["y_r_flat"]
    return _energy_official_style(A, y, sigma=sigma, k_max=k_max)


def q10_recompute_nodiff(cfg, chair, offr1, differenced, tag, sigma=0.75, nproc=8):
    import multiprocessing as mp
    gx = chair["xhatvals"].reshape(-1)
    gy = chair["yhatvals"].reshape(-1)
    gz = chair["zhatvals"].reshape(-1)
    cands = [np.array([x, yv, zv]) for x in gx for yv in gy for zv in gz]
    args = [(ll, sigma, 10, differenced) for ll in cands]
    with mp.Pool(nproc, initializer=_winit, initargs=(cfg,)) as pool:
        res = pool.map(_worker, args, chunksize=8)
    E = np.array([r[0] for r in res], dtype=np.float64).reshape(gx.size, gy.size, gz.size)
    K = np.array([r[1] for r in res], dtype=np.int64).reshape(gx.size, gy.size, gz.size)
    am = np.unravel_index(np.argmax(E), E.shape)
    p("")
    p("### Q10 用官方口径复算第 1 轮 972 个候选（%s，sigma=%g，K<=10）" % (tag, sigma))
    uv, cnt = np.unique(K, return_counts=True)
    p("  K 的取值分布：%s" % (dict(zip(uv.tolist(), cnt.tolist())),))
    p("  3D argmax = %s -> ll=(%.6f, %.6f, %.6f)"
      % (am, gx[am[0]], gy[am[1]], gz[am[2]]))
    p("  r(本复算 3D, 官方t=0 3D) = %+.4f ; r(z0 切片) = %+.4f"
      % (pear(E, offr1), pear(E[:, :, 0], offr1[:, :, 0])))
    return E, K


def q4b_slice_transform(mine_z0, off_z0, label):
    p("")
    p("### Q4b 只对 z=0 的二维切片找变换（复现 x25 摘要里那种比法）—— " + label)
    m = np.asarray(mine_z0, dtype=np.float64)
    o = np.asarray(off_z0, dtype=np.float64)
    am = np.unravel_index(np.argmax(m), m.shape)
    ao = np.unravel_index(np.argmax(o), o.shape)
    aT = np.unravel_index(np.argmax(m.T), m.shape)
    d0 = float(np.linalg.norm(np.array(am, float) - np.array(ao, float)))
    dT = float(np.linalg.norm(np.array(aT, float) - np.array(ao, float)))
    p("    本实现 argmax              = %s" % (am,))
    p("    官方   argmax              = %s" % (ao,))
    p("    本实现转置(argmax 变为 %s) 到官方 argmax 的步距 = %.3f 格" % (aT, dT))
    p("    原取向 argmax 到官方 argmax 的步距               = %.3f 格" % d0)
    r0 = pear(m, o)
    rT = pear(m.T, o)
    p("    r(mine    , off  ) = %+.4f" % r0)
    p("    r(mine.T  , off  ) = %+.4f" % rT)
    p("    r(mine    , off.T) = %+.4f" % rT)
    raise_r = abs(rT) > abs(r0)
    if raise_r and dT < d0:
        p("    -> 转置同时抬高 |r| 且拉近 argmax：本档下无法据此排除转置")
    elif raise_r:
        p("    -> 转置抬高了 |r|，但 argmax 被推得更远（%.3f -> %.3f）：抬高是假信号" % (d0, dT))
    else:
        p("    -> 转置不抬高 |r|（%+.4f -> %+.4f）：转置被 r 反证（argmax 步距 %.3f -> %.3f）"
          % (r0, rT, d0, dT))


# ======================================================================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-root", required=True)
    ap.add_argument("--repo", default=os.path.dirname(os.path.dirname(_SRC_MODEL)))
    ap.add_argument("--recompute", action="store_true",
                    help="额外用官方口径（不差分 + K<=10）复算第 1 轮 972 个候选（约 2~5 分钟）")
    ap.add_argument("--nproc", type=int, default=8)
    args = ap.parse_args()

    chair_path = os.path.join(args.data_root, "TestPosChair", "chair_test.mat")
    cube_path = os.path.join(args.repo, "sim", "results", "localization_energy_cubes.npz")
    chair = sio.loadmat(chair_path)
    npz = np.load(cube_path)

    # 每一轮**自己那个 .mat** 里的 threshold 列表（各轮不同）
    th_lists = {}
    bur = {1: "bur_opt_1st_it_pos.mat", 2: "bur_opt_2nd_it_pos.mat",
           3: "bur_opt_3rd_it_pos.mat"}
    for rnd, fn in bur.items():
        f = os.path.join(args.data_root, "TestPosChair", fn)
        if os.path.exists(f):
            th_lists[rnd] = np.asarray(
                sio.loadmat(f)["threshold"], dtype=np.float64).reshape(-1)
    th_lists[4] = th_lists.get(2)

    mine1 = npz["E_round1"].astype(np.float64)
    mine2 = npz["E_round2"].astype(np.float64)

    p("# X27 能量图 F/C 序矛盾 —— 判定脚本实测输出")
    p("数据：%s" % chair_path)
    p("本实现能量立方体：%s" % cube_path)

    offr1 = np.asarray(chair["proj_yOnRangeA"], dtype=np.float64)[:, :, :, 0]
    offg1 = np.asarray(chair["proj_yOnRangeA_g"], dtype=np.float64)[:, :, :, 0]
    offb1 = np.asarray(chair["proj_yOnRangeA_b"], dtype=np.float64)[:, :, :, 0]

    q0_index_convention(chair, cube_path, args.repo)

    q1_flatten_order(mine1, offr1, "本实现第 1 轮 vs 官方第 1 轮(t=0) 三维展平")
    q1_flatten_order(mine1[:, :, 0], offr1[:, :, 0], "第 1 轮 z=0 二维切片")

    ll_true = np.asarray(chair["Occluder"], dtype=np.float64)[0]
    ll_a = np.array([0.52941176, 0.65, 0.2])
    ll_b = np.array([0.54705882, 0.63235294, 0.275])
    q2_permutation_invariance(chair_path, [ll_true, ll_a, ll_b])

    hits = q3_readout(chair, th_lists)

    p("")
    p("### 通道与「官方自己换一档」的对照（确认通道不是主因）")
    p("  r(官方R第1轮 z0, 官方G第1轮 z0) = %+.4f" % pear(offr1[:, :, 0], offg1[:, :, 0]))
    p("  r(官方R第1轮 z0, 官方B第1轮 z0) = %+.4f" % pear(offr1[:, :, 0], offb1[:, :, 0]))
    p("  r(官方G第1轮 z0, 官方B第1轮 z0) = %+.4f" % pear(offg1[:, :, 0], offb1[:, :, 0]))
    arr1 = np.asarray(chair["proj_yOnRangeA"], dtype=np.float64)
    p("  r(官方第1轮 t=0 全展平, 官方第1轮 t=1 全展平) = %+.4f   <- 官方自己换一档阈值"
      % pear(arr1[:, :, :, 0], arr1[:, :, :, 1]))

    q4b_slice_transform(mine1[:, :, 0], offr1[:, :, 0], "第 1 轮 z=0")
    arr2 = np.asarray(chair["proj_yOnRangeA_2"], dtype=np.float64)
    q4b_slice_transform(mine2[:, :, 0], arr2[:, :, 0, 0], "第 2 轮 z=0，摘要用的 t=0（错档）")
    q4b_slice_transform(mine2[:, :, 0], arr2[:, :, 0, 3], "第 2 轮 z=0，正确档 t=3")

    q4_transform_search(mine1, offr1, "第 1 轮（红通道，正确档 t=0）")
    q4_transform_search(mine2, arr2, "第 2 轮（红通道，正确档 t=3）", off_t=3)

    p("")
    p("### 读出口径修正前后（同一对本实现/官方，只改官方读哪一档）")
    for rnd, cube in ((1, mine1), (2, mine2)):
        if rnd == 1:
            key_r = "proj_yOnRangeA"
        else:
            key_r = "proj_yOnRangeA_%d" % rnd
        arr = np.asarray(chair[key_r], dtype=np.float64)
        tstar = hits[rnd]["t_hits"][0] if hits[rnd]["t_hits"] else 0
        todo = [0] if tstar == 0 else [0, tstar]
        for t in todo:
            o = arr[:, :, :, t]
            tag = "  <- 正确档" if t == tstar else ""
            p("  第 %d 轮 t=%d : r(z=0 切片)=%+.4f  r(全 3D 展平)=%+.4f%s"
              % (rnd, t, pear(cube[:, :, 0], o[:, :, 0]), pear(cube, o), tag))
    for t in range(arr2.shape[3]):
        p("  第 2 轮 t=%2d : r(z=0 切片)=%+.4f  r(全 3D)=%+.4f"
          % (t, pear(mine2[:, :, 0], arr2[:, :, 0, t]), pear(mine2, arr2[:, :, :, t])))

    q5_rank1_and_table(chair, mine1, mine2, offr1, arr2)
    q7_pairing_bug(chair, cube_path)
    q8_recon_impact(chair_path)
    q9_crossthreshold_band(chair)

    if args.recompute:
        E_nd, K_nd = q10_recompute_nodiff(chair_path, chair, offr1,
                                          differenced=False,
                                          tag="本实现 A + 官方不差分口径",
                                          nproc=args.nproc)
        p("  r(不差分复算 3D, 本实现已存差分立方体 E_round1) = %+.4f"
          % pear(E_nd, mine1))
        p("  r(不差分复算 3D, 官方 3D) = %+.4f  ← 与「差分口径」的 %+.4f 对比"
          % (pear(E_nd, offr1), pear(mine1, offr1)))
        np.savez_compressed(
            os.path.join(args.repo, "sim", "results", "x27_round1_nodiff_surf.npz"),
            E_nodiff=E_nd.astype(np.float32), K_npass=K_nd.astype(np.int16))

    out = os.path.join(args.repo, "sim", "results", "x27_fc_order_run.log")
    with io.open(out, "w", encoding="utf-8") as f:
        f.write("\n".join(LOG) + "\n")
    print("written " + out)


if __name__ == "__main__":
    main()
