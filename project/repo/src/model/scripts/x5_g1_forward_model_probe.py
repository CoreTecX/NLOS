# -*- coding: utf-8 -*-
"""X5 附带诊断：用**忠实定位前向模型**（`forward_loc`）复算整面，检验
「残余缺口的主因是两套前向模型不同」这条推测。

做法（三类面，**全部用官方算子口径**：不差分、`K = min(#{s/s1 > sigma}, 10)`、
返回 `norm` 而不是平方）：
  S_off    官方自己存的面 `proj_yOnRangeA{_k}[..., thresh_k-1]`
  S_faith  A 由 `forward_loc.build_A_from_plan` 建（论文定位链那套模型）
  S_py     A 由 `forward.py.build_transport_matrix` 建（本实现重建链那套模型）
同轮同网格比较 `r(S_faith, S_off)` / `r(S_py, S_off)` / `r(S_faith, S_py)`。

结论见 `sim/results/x5_recon_verification.md` §5；实测日志 `sim/results/x5_g1_probe.log`。

注意一个口径坑：官方 `occluderposgridsearch.m:55` 存的是
`norm(U(:,1:K)' * meas.r(:))`（norm，不是平方），而本实现立方体存的是平方能量
`Σ c²/λ`；本脚本必须返回 norm 才能与官方数组同量纲（否则比值会差一个平方，
曾实测到 1.2e8 倍）。

用法（在仓库根目录）：
  python src/model/scripts/x5_g1_forward_model_probe.py --data-root "<官方 Data 目录>"
"""
import argparse
import os
import sys
import time

import numpy as np
import scipy.io as sio

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC_MODEL = os.path.dirname(_HERE)
if _SRC_MODEL not in sys.path:
    sys.path.insert(0, _SRC_MODEL)

from penumbrasight_model import forward as F          # noqa: E402
from penumbrasight_model import forward_loc as FL     # noqa: E402

_ROUND_SUFFIX = {1: "", 2: "_2", 3: "_3", 4: "_4"}
_THRESH = {1: 1, 2: 4, 3: 6, 4: 1}      # thresh{k}（1 基），来自 chair_test.mat


def pear(a, b):
    x = np.asarray(a, dtype=np.float64).reshape(-1)
    y = np.asarray(b, dtype=np.float64).reshape(-1)
    x = x - x.mean()
    y = y - y.mean()
    den = np.sqrt(float(x @ x) * float(y @ y))
    return float(x @ y / den) if den > 0 else float("nan")


def energy_official_style(A, y, sigma, k_max=10):
    """官方口径，**返回 norm 不是平方**（与官方数组同量纲）。"""
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
    return float(np.sqrt(np.sum((c ** 2) / lam[:K]))), npass


# ------------------------------------------------------------------ worker
_W = {}


def _winit(kind, cfg, ds, N, mz):
    _W["kind"] = kind
    d = sio.loadmat(cfg)
    _W["y"] = np.asarray(d["meas_r"], dtype=np.float64).reshape(-1)
    if kind == "faith":
        _W["setup"] = FL.d11_loc_setup()
        _W["plan"] = FL.make_plan(_W["setup"], N, downsamp_factor=ds,
                                  mz_mirrored=mz)
    else:
        _W["setup"] = F.load_setup(cfg)


def _worker(args):
    ll, sigma = args
    if _W["kind"] == "faith":
        A = FL.build_A_from_plan(_W["plan"], ll)
    else:
        A = F.build_transport_matrix(_W["setup"].with_occluder_ll(ll))
    return energy_official_style(A, _W["y"], sigma)


def surface(cfg, kind, grid, ds, N, mz, sigma, nproc):
    import multiprocessing as mp
    gx, gy, gz = grid
    cands = [np.array([x, yv, zv]) for x in gx for yv in gy for zv in gz]
    args = [(ll, sigma) for ll in cands]
    t0 = time.time()
    with mp.Pool(nproc, initializer=_winit,
                 initargs=(kind, cfg, ds, N, mz)) as pool:
        res = pool.map(_worker, args, chunksize=4)
    E = np.array([r[0] for r in res], dtype=np.float64).reshape(
        gx.size, gy.size, gz.size)
    K = np.array([r[1] for r in res], dtype=np.int64).reshape(
        gx.size, gy.size, gz.size)
    uv, cnt = np.unique(K, return_counts=True)
    print(f"   [{kind}] 网格 {E.shape} 用时 {time.time()-t0:.1f}s  "
          f"K 取值={dict(zip(uv.tolist(), cnt.tolist()))}", flush=True)
    return E, K


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-root", required=True,
                    help="官方 Data 目录（其下含 TestPosChair/）")
    ap.add_argument("--nproc", type=int, default=6)
    ap.add_argument("--ds", type=int, default=3)
    ap.add_argument("--nmon-scan", default="1,2,3,10")
    ap.add_argument("--nmon", type=int, default=2)
    ap.add_argument("--skip-scan", action="store_true")
    args = ap.parse_args()

    cfg = os.path.join(args.data_root, "TestPosChair", "chair_test.mat")
    d = sio.loadmat(cfg)
    meas_r = np.asarray(d["meas_r"], dtype=np.float64)
    print(f"meas_r shape={meas_r.shape} size={meas_r.size}")
    grids = {}
    for tag, suf in (("r1", ""), ("r2", "2"), ("r3", "3"), ("r4", "4")):
        grids[tag] = tuple(
            np.asarray(d[f"{k}{suf}"], dtype=np.float64).reshape(-1)
            for k in ("xhatvals", "yhatvals", "zhatvals"))
    print("网格尺寸：" + "  ".join(
        f"{k}={tuple(g.size for g in v)}" for k, v in grids.items()))

    # 0) A_py 行数必须等于 meas_r
    setup_py = F.load_setup(cfg)
    A_probe = F.build_transport_matrix(setup_py.with_occluder_ll(
        np.array([0.5, 0.6, 0.2])))
    print(f"A_py shape={A_probe.shape}  (需与 meas_r.size={meas_r.size} 一致)")
    assert A_probe.shape[0] == meas_r.size
    del A_probe

    # 1) Ndiscr_mon 扫描：官方第 1 轮 12 个抽样点（σ=0.75）
    if not args.skip_scan:
        print("\n== Ndiscr_mon 扫描（第 1 轮 σ=0.75，取 12 个官方抽样位）==")
        off1 = np.asarray(d["proj_yOnRangeA"], dtype=np.float64)[:, :, :, 0]
        pts = [(0, 0, 0), (17, 17, 2), (8, 8, 1), (13, 13, 1), (5, 12, 0),
               (2, 15, 2), (15, 11, 1), (4, 5, 0), (1, 0, 0), (14, 11, 2),
               (9, 10, 2), (13, 11, 1)]
        gx, gy, gz = grids["r1"]
        lls = [np.array([gx[i], gy[j], gz[k]]) for (i, j, k) in pts]
        ref = np.array([off1[i, j, k] for (i, j, k) in pts])
        import multiprocessing as mp
        for N in [int(v) for v in args.nmon_scan.split(",")]:
            with mp.Pool(args.nproc, initializer=_winit,
                         initargs=("faith", cfg, args.ds, N, True)) as pool:
                res = pool.map(_worker, [(ll, 0.75) for ll in lls])
            got = np.array([r[0] for r in res])
            ratio = got / ref
            print(f"   Nmon={N:2d}: 比值 {np.round(ratio, 4).tolist()}  "
                  f"min={ratio.min():.4f} max={ratio.max():.4f} "
                  f"均值={ratio.mean():.4f} 标准差={ratio.std():.5f}", flush=True)

    # 2) 第 2 轮（σ=0.1）
    print(f"\n== 第 2 轮（t=3, σ=0.1）整面：Nmon={args.nmon} ==")
    off2 = np.asarray(d["proj_yOnRangeA_2"], dtype=np.float64)[:, :, :, 3]
    E_f2, _ = surface(cfg, "faith", grids["r2"], args.ds, args.nmon, True,
                      0.1, args.nproc)
    E_p2, _ = surface(cfg, "py", grids["r2"], args.ds, args.nmon, True,
                      0.1, args.nproc)
    print(f"   r(S_faith, S_off) 3D = {pear(E_f2, off2):+.4f}   "
          f"(z=官方argmax 层 {pear(E_f2[:, :, 5], off2[:, :, 5]):+.4f})")
    print(f"   r(S_py   , S_off) 3D = {pear(E_p2, off2):+.4f}   "
          f"(z=官方argmax 层 {pear(E_p2[:, :, 5], off2[:, :, 5]):+.4f})")
    print(f"   r(S_faith, S_py ) 3D = {pear(E_f2, E_p2):+.4f}")
    for nm, E in (("S_faith", E_f2), ("S_py", E_p2)):
        print(f"   {nm} argmax = {np.unravel_index(np.argmax(E), E.shape)}")
    print(f"   S_off argmax = {np.unravel_index(np.argmax(off2), off2.shape)}")

    # 3) 第 1 轮（σ=0.75）
    print(f"\n== 第 1 轮（t=0, σ=0.75）整面：Nmon={args.nmon} ==")
    off1 = np.asarray(d["proj_yOnRangeA"], dtype=np.float64)[:, :, :, 0]
    E_f1, _ = surface(cfg, "faith", grids["r1"], args.ds, args.nmon, True,
                      0.75, args.nproc)
    E_p1, _ = surface(cfg, "py", grids["r1"], args.ds, args.nmon, True,
                      0.75, args.nproc)
    print(f"   r(S_faith, S_off) 3D = {pear(E_f1, off1):+.4f}   "
          f"(z=0 切片 {pear(E_f1[:, :, 0], off1[:, :, 0]):+.4f})")
    print(f"   r(S_py   , S_off) 3D = {pear(E_p1, off1):+.4f}   "
          f"(z=0 切片 {pear(E_p1[:, :, 0], off1[:, :, 0]):+.4f})")
    print(f"   r(S_faith, S_py ) 3D = {pear(E_f1, E_p1):+.4f}")

    print("\n[done]")


if __name__ == "__main__":
    main()
