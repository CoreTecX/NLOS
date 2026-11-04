# -*- coding: utf-8 -*-
"""X36 —— 主机侧定点 ADMM（论文链重建）验证与缺口量化。

做什么
------
1. 复用 X35 的论文算子与口径（126x126 墙、29x36 隐藏图、F 序展平 + 行差），
   K 秩截断，真实测量 paper_meas R 通道。
2. 论文 λ 的尺度重参数化：s_f=128、s_d=2^18 -> λ_q16 = 20（与 X35 一致）。
3. 双精度参考：论文尺度 ADMM（M5 求解器）+ 核尺度的等价 ADMM。
4. 定点 ADMM：C++ 驱动（build/scripts/x36_admm_fixed.cpp）复用核的
   ps::lr_forward / lr_adjoint / TV 原语 / 舍入饱和原语，融合正规方程算子
   BᵀB = V S² Vᵀ，秩域中间量放宽。
5. 缺口量化：把「算子定点化」与「内层 CG 整数化」两条损失分开量，并做
   Q 格式宽度（FRAC）、ρ、内层迭代数的敏感性扫描。
6. 三组落盘对照（本轮补）：
   - 秩域中间量宽/窄：KIM(2^23-1) vs 2^30-1，双精度 CG（n_cg=15）与定点链（n_cg=30）；
   - Q6.18 状态量化：双精度 ADMM 状态逐步量化到 Q6.18（n_cg=15）；
   - lr 往返饱和：驱动 mode=1 走 lr_forward→lr_adjoint 拼 BᵀB，与融合算子同档对照。

只写 build/scripts 与 sim/results；不改 src/fpga/hls/**、src/fpga/rtl/**、
src/model/**。不跑 Vivado 实现/bitstream。

用法（仓库根执行）：
    python build/scripts/x36_admm_fixed.py
"""
from __future__ import annotations

import argparse
import json
import math
import os
import struct
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, os.pardir, os.pardir))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
import x35_paper_lambda_fixed as X35  # noqa: E402
import x20_hls_recon_image as X     # noqa: E402

VEC = os.path.join(REPO, "sim", "vectors")
RESULTS = os.path.join(REPO, "sim", "results")
WORK = os.path.join(REPO, "build", "x36_work")
ROWS, COLS = 29, 36
NH = ROWS * COLS
KIM = (1 << 23) - 1                 # ps_fixed.hpp: kImageCodeMax（Q6.18）
KMM = (1 << 17) - 1                 # ps_fixed.hpp: kMeasCodeMax（Q2.16）
WIDE = (1 << 30) - 1                # 秩域中间量放宽上限（驱动 kWideLim 同）
GF = 16                             # ps_fixed.hpp: kGainFrac
Q16 = 65536.0


def log(m):
    print(m, flush=True)


# ------------------------------------------------------------------ 定点仿真原语
# 与 src/fpga/hls/ps_fixed.hpp 逐行同义（round_half_even + 饱和）。

def msr(a, b, shift):
    """mul_shift_round：先乘，再按 shift 就近取整右移（与 ps_fixed.hpp 同）。"""
    a = np.asarray(a, dtype=np.int64); b = np.asarray(b, dtype=np.int64)
    p = a * b
    if shift == 0:
        return p
    if shift < 0:
        return p << (-shift)
    q = p >> shift
    rem = p - (q << shift)
    half = 1 << (shift - 1)
    return q + ((rem > half) | ((rem == half) & ((q & 1) != 0)))


def nar(v, lim, cnt=None):
    v = np.asarray(v, dtype=np.int64)
    o = np.clip(v, -lim, lim)
    if cnt is not None:
        cnt[0] += int(np.count_nonzero(o != v))
    return o


def ratio_q16_near(num, den):
    if den == 0:
        return 0
    return int(round(float(num) / float(den) * 65536.0))


def sm_err(u, ref):
    """标度不变误差 min_alpha ||alpha*u - ref|| / ||ref||（与 x15g 同口径）。"""
    u = np.asarray(u, float).ravel(); ref = np.asarray(ref, float).ravel()
    den = float(u @ u)
    if den <= 0:
        return float("inf")
    a = float(u @ ref) / den
    nr = float(np.linalg.norm(ref))
    return float(np.linalg.norm(a * u - ref) / nr) if nr > 0 else float("nan")


# ------------------------------------------------------------------ 参考解（双精度）

def admm_double_kernel(Bk, Vk, s_f, dm, s_d, lam_q16, rho_q16, n_outer=400, n_cg=30):
    """核尺度的双精度 ADMM（与论文尺度等价，见 X35 报告）。"""
    N = NH
    d_k = dm / s_d
    ATb = Bk.T @ d_k
    rho_k = rho_q16 / Q16
    th = (lam_q16 / Q16) / rho_k
    Lop = lambda u: -X35.div2(X35.grad2(u.reshape((ROWS, COLS)))).reshape(-1)  # noqa: E731
    op = lambda u: Bk.T @ (Bk @ u) + rho_k * Lop(u)                            # noqa: E731
    u = np.zeros(N)
    d = np.zeros((2, ROWS, COLS))
    q = np.zeros((2, ROWS, COLS))
    for _ in range(n_outer):
        u = X35.cg(op, ATb - rho_k * X35.div2(d - q).reshape(-1), u, n_cg)
        u = np.maximum(0.0, u)
        g = X35.grad2(u.reshape((ROWS, COLS)))
        s_ = g + q
        nrm = np.sqrt(s_[0] ** 2 + s_[1] ** 2)
        d = s_ * np.maximum(0.0, 1.0 - th / np.maximum(nrm, 1e-30))
        q = q + g - d
    return u


def admm_double_kernel_state_quant(Bk, s_f, dm, s_d, lam_q16, rho_q16, n_outer=400,
                                   n_cg=15, quant=True):
    """核尺度双精度 ADMM；quant=True 时把每个外层迭代后状态量化到 Q6.18 码字
    （round + 饱和到 ±KIM），用于隔离「状态位宽」这一项。
    返回 (解, |z|max 核单位)。
    """
    N = NH
    d_k = dm / s_d
    ATb = Bk.T @ d_k
    rho_k = rho_q16 / Q16
    th = (lam_q16 / Q16) / rho_k
    Lop = lambda u: -X35.div2(X35.grad2(u.reshape((ROWS, COLS)))).reshape(-1)  # noqa: E731
    op = lambda u: Bk.T @ (Bk @ u) + rho_k * Lop(u)                            # noqa: E731
    u = np.zeros(N)
    d = np.zeros((2, ROWS, COLS))
    q = np.zeros((2, ROWS, COLS))
    zmax = 0.0
    for _ in range(n_outer):
        u = X35.cg(op, ATb - rho_k * X35.div2(d - q).reshape(-1), u, n_cg)
        if quant:
            u = np.clip(np.rint(u * (2.0 ** 18)), -KIM, KIM).astype(np.int64) / (2.0 ** 18)
        u = np.maximum(0.0, u)
        zmax = max(zmax, float(np.max(np.abs(u))))
        g = X35.grad2(u.reshape((ROWS, COLS)))
        s_ = g + q
        nrm = np.sqrt(s_[0] ** 2 + s_[1] ** 2)
        d = s_ * np.maximum(0.0, 1.0 - th / np.maximum(nrm, 1e-30))
        q = q + g - d
    return u, zmax


def op_fixed_doublecg(Uk, S, Vk, s_max, s_f, s_d, UU, S_, V_, ue, ve, se, rho_q16,
                      lam_q16, n_outer=400, n_cg=30, wide=(1 << 30) - 1):
    """算子按定点算（融合 V S² Vᵀ），但 CG 用双精度：隔离「算子定点化」的损失。"""
    N = NH
    sig_k = S * s_max * (s_f / s_d)
    Bk = (Uk * sig_k[None, :]) @ Vk.T
    d_k = np.load(os.path.join(VEC, "paper_meas.npy"))
    dmv = X35.dvec(X35.rows_of(d_k[:, :, 0], "F"))
    dk = dmv / s_d
    ATb = Bk.T @ dk
    rho_k = rho_q16 / Q16
    th = (lam_q16 / Q16) / rho_k
    cnt = [0]

    def btb_code(p):
        t = nar(msr(V_ @ p, 1 << GF, 16 - ve), wide, cnt)
        t = nar(msr(t * S_, 1 << GF, 16 - se), wide, cnt)
        t = nar(msr(t * S_, 1 << GF, 16 - se), wide, cnt)
        return nar(msr(V_.T @ t, 1 << GF, 16 - ve), KIM, cnt)

    def op(p):
        pi = np.clip(np.rint(p * (2.0 ** 18)), -KIM, KIM).astype(np.int64)
        L = -X35.div2(X35.grad2(p.reshape((ROWS, COLS)))).reshape(-1)
        return btb_code(pi).astype(float) / (2.0 ** 18) + rho_k * L

    u = np.zeros(N); d = np.zeros((2, ROWS, COLS)); q = np.zeros((2, ROWS, COLS))
    for _ in range(n_outer):
        u = X35.cg(op, ATb - rho_k * X35.div2(d - q).reshape(-1), u, n_cg)
        u = np.maximum(0.0, u)
        g = X35.grad2(u.reshape((ROWS, COLS)))
        s_ = g + q
        nrm = np.sqrt(s_[0] ** 2 + s_[1] ** 2)
        d = s_ * np.maximum(0.0, 1.0 - th / np.maximum(nrm, 1e-30))
        q = q + g - d
    return u, cnt[0]


# ------------------------------------------------------------------ 定点 ADMM（经真核）

def compile_driver(exe):
    src = os.path.join(HERE, "x36_admm_fixed.cpp")
    cmd = ["g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Wno-unknown-pragmas",
           "-DPS_LR_MAX_RANK=96", "-DPS_LR_MAX_RANK_FP=96",
           "-Isrc/hardware", "-Isrc/software/include", "-Isrc/fpga/hls",
           "-Isrc/fpga/hls/pdhg_core", "-Isrc/fpga/hls/lr_operator", src,
           "src/fpga/hls/pdhg_core/pdhg_core_fixed.cpp",
           "src/fpga/hls/lr_operator/lr_operator_fixed.cpp",
           "src/fpga/hls/pdhg_core/pdhg_core.cpp",
           "src/fpga/hls/lr_operator/lr_operator.cpp", "-o", exe]
    r = subprocess.run(cmd, cwd=REPO, capture_output=True, encoding="utf-8",
                       errors="replace")
    if r.returncode != 0:
        log(r.stdout); log(r.stderr)
        raise SystemExit("driver compile failed")
    return exe


def write_case(path, K, M, N, n_rows, n_cols, n_outer, n_cg, rho_q16, lam_q16,
               exps, U_store, S_code, V_store, d_code, wide, mode=0):
    h = [K, M, N, n_rows, n_cols, n_outer, n_cg, rho_q16, lam_q16,
         exps[0], exps[1], exps[2], wide, mode, 0, 0]
    with open(path, "wb") as f:
        f.write(b"X36C")
        f.write(struct.pack("<16i", *h))
        f.write(U_store.astype("<i2").tobytes(order="C"))
        f.write(S_code.astype("<u2").tobytes(order="C"))
        f.write(V_store.astype("<i2").tobytes(order="C"))
        f.write(d_code.astype("<i4").tobytes(order="C"))


def read_result(path, n_outer, n_cg, N):
    with open(path, "rb") as f:
        assert f.read(4) == b"X36R"
        h = struct.unpack("<8i", f.read(32))
        res = np.frombuffer(f.read(8 * n_outer), "<f8").copy()
        obj = np.frombuffer(f.read(8 * n_outer), "<f8").copy()
        spl = np.frombuffer(f.read(8 * n_outer), "<f8").copy()
        zmx = np.frombuffer(f.read(8 * n_outer), "<f8").copy()
        cg = np.frombuffer(f.read(8 * n_cg), "<f8").copy()
        z = np.frombuffer(f.read(4 * N), "<i4").copy()
    return dict(N=h[0], n_outer=h[1], sat=h[2], mac_ovf=h[3], out_ovf=h[4],
                score_ovf=h[5], gain_clamp=h[6], cg_breaks=h[7],
                res_sq=res, obj=obj, split=spl, zmax=zmx, cg_res=cg, z=z)


def run_driver(exe, case, resf):
    r = subprocess.run([exe, case, resf], capture_output=True, encoding="utf-8",
                       errors="replace")
    if r.returncode != 0:
        log(r.stdout); log(r.stderr)
        raise SystemExit("driver failed")
    diag = {}
    for line in r.stdout.splitlines():
        if line.startswith("[A fixed]") or line.startswith("[flags]"):
            for tok in line.split():
                if "=" in tok:
                    k, v = tok.split("=", 1)
                    diag[k] = v
    return diag


# ------------------------------------------------------------------ main

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rank", type=int, default=96)
    ap.add_argument("--n-outer", type=int, default=400)
    ap.add_argument("--n-cg", type=int, default=30)
    ap.add_argument("--s-f", type=int, default=128)
    ap.add_argument("--out", default="")
    args = ap.parse_args()
    os.makedirs(WORK, exist_ok=True)
    t0 = time.time()

    log("=" * 88)
    log("X36  主机侧定点 ADMM（论文链重建）")
    log("=" * 88)

    # ---- 0. 算子 / 测量 / 真值 ----
    dA = X35.build_paper_operator()
    meas = np.load(os.path.join(VEC, "paper_meas.npy"))
    dm = X35.dvec(X35.rows_of(meas[:, :, 0], "F"))
    gt = np.load(os.path.join(VEC, "paper_gt.npy"))
    gtC = gt[:, :, 0].astype(float).reshape(-1, order="C")
    s_d = X35.choose_s_d(dm) if hasattr(X35, "choose_s_d") else float(2 ** int(math.ceil(math.log2(float(np.abs(dm).max()) / 1.5))))
    Uk, S, Vk, s_max, _Vc, _S = X35.top_singular(dA, args.rank)
    del dA
    s_f = float(args.s_f)
    lam_p = X35.REG_R / X35.CSCALE_R
    lam_k = lam_p * s_f / (s_d ** 2)
    lam_q16 = int(round(lam_k * Q16))
    V = np.ascontiguousarray(Vk.T)
    U_store, S_code, V_store, ue, ve, se, S_scaled = X.quantize_factors(
        Uk, S, V, s_max, s_f, s_d)
    D, d_code = X.quantize_measurement(dm, s_d)
    d_sq_code = float(np.sum(d_code.astype(np.float64) ** 2))
    sig_k = S * s_max * (s_f / s_d)
    rho_k = 0.1 * float(np.mean(sig_k ** 2))
    rho_q16 = int(round(rho_k * Q16))
    Bk = (Uk * sig_k[None, :]) @ Vk.T
    log("[0] K=%d  M=%d  N=%d  s_f=%.0f  s_d=2^%.0f  ||B_kernel||=%.4f"
        % (args.rank, Bk.shape[0], NH, s_f, math.log2(s_d), s_max * s_f / s_d))
    log("    lam_paper=%.6g  lam_solver=%.6g  lambda_q16=%d  rho_solver=%.6g  rho_q16=%d"
        % (lam_p, lam_k, lam_q16, rho_k, rho_q16))
    UU = U_store.astype(np.int64); S_ = S_code.astype(np.int64); V_ = V_store.astype(np.int64)

    # ---- 1. 参考：论文 λ 下的收敛求解器 ----
    log("[1] 参考解（同一算子、同一测量、论文 λ）")
    f_ls = Vk @ ((Uk.T @ dm) / (S * s_max))
    f_ref_paper, rho_p = X35.admm_tv(Uk, s_max, S, Vk, dm, lam_p, nonneg=True)
    log("    精确截断最小二乘      r(gt)=%+.4f" % X.pearson(f_ls, gtC))
    log("    论文尺度 ADMM(K=96)   r(gt)=%+.4f  rho_p=%.6g" % (X.pearson(f_ref_paper, gtC), rho_p))

    # ---- 2. 核尺度双精度参考（对每个内层迭代数各算一份，做同口径对照）----
    refs = {}
    for ncg in sorted({15, args.n_cg}):
        refs[ncg] = admm_double_kernel(Bk, Vk, s_f, dm, s_d, lam_q16, rho_q16,
                                       args.n_outer, ncg)
        log("[2] 核尺度双精度 ADMM n_cg=%2d  r(gt)=%+.4f" % (ncg, X.pearson(refs[ncg] * s_f, gtC)))
    f_ref_kern = refs[args.n_cg]
    log("    relL2(核尺度 n_cg=%d vs 论文尺度 n_cg=15)=%.4e"
        % (args.n_cg, X.rel_l2(f_ref_kern * s_f, f_ref_paper)))

    # ---- 3. 算子定点化损失（双精度 CG）----
    f_opfix, op_sat = op_fixed_doublecg(Uk, S, Vk, s_max, s_f, s_d, UU, S_, V_,
                                        ue, ve, se, rho_q16, lam_q16,
                                        args.n_outer, args.n_cg)
    log("[3] 定点算子 + 双精度 CG  n_cg=%d  r(gt)=%+.4f  （中间量饱和事件 %d）"
        % (args.n_cg, X.pearson(f_opfix * s_f, gtC), op_sat))

    # ---- 3b. 秩域中间量宽/窄对照（双精度 CG、同一算子、n_cg=15）----
    rw = {}
    for tag, wid in (("narrow_q6_18", KIM), ("wide_2p30", WIDE)):
        ff, sat = op_fixed_doublecg(Uk, S, Vk, s_max, s_f, s_d, UU, S_, V_,
                                    ue, ve, se, rho_q16, lam_q16, args.n_outer, 15, wid)
        rw[tag] = dict(limit=int(wid), n_cg=15, r_gt=X.pearson(ff * s_f, gtC),
                       sat_events=int(sat))
        log("    [秩域宽度/双精度CG] n_cg=15 %-12s (上限 %d)  r(gt)=%+.4f  饱和=%d"
            % (tag, wid, rw[tag]["r_gt"], rw[tag]["sat_events"]))

    # ---- 4. 定点 ADMM（C++ 驱动，复用核原语）----
    log("[4] 定点 ADMM（C++ 驱动，复用核的 lr_forward/lr_adjoint/TV 原语）")
    exe = compile_driver(os.path.join(WORK, "x36_driver.exe"))

    def run_fixed(ncg, wide=WIDE, mode=0):
        tag = "w%d_m%d" % (wide, mode)
        case = os.path.join(WORK, "case_ncg%d_%s.bin" % (ncg, tag))
        resf = os.path.join(WORK, "res_ncg%d_%s.bin" % (ncg, tag))
        write_case(case, args.rank, Bk.shape[0], NH, ROWS, COLS, args.n_outer, ncg,
                   rho_q16, lam_q16, (ue, ve, se), U_store, S_code, V_store, d_code,
                   wide, mode)
        diag = run_driver(exe, case, resf)
        R = read_result(resf, args.n_outer, ncg, NH)
        f_fix = R["z"].astype(float) / (2.0 ** 18) * s_f
        return f_fix, R, diag

    cg_sweep = []
    for ncg in (5, 10, 15, 20, 25, 30, 40, 60):
        ff, R, _ = run_fixed(ncg)
        cg_sweep.append(dict(n_cg=ncg, r_gt=X.pearson(ff, gtC),
                             rho_last=math.sqrt(R["res_sq"][-1] / d_sq_code),
                             sat=int(R["sat"])))
        log("    [内层扫描] n_cg=%2d -> r(gt)=%+.4f  rho_last=%.4f  饱和=%d"
            % (ncg, cg_sweep[-1]["r_gt"], cg_sweep[-1]["rho_last"], cg_sweep[-1]["sat"]))

    fixed = {}
    for ncg in sorted({15, args.n_cg}):
        f_fix, R, diag = run_fixed(ncg)
        rec = dict(n_cg=ncg, n_outer=args.n_outer,
                   r_gt=X.pearson(f_fix, gtC),
                   r_gt_dbl=X.pearson(refs[ncg] * s_f, gtC),
                   relL2_fix_vs_dblADMM=X.rel_l2(f_fix, refs[ncg] * s_f),
                   r_fix_vs_dblADMM=X.pearson(f_fix, refs[ncg] * s_f),
                   relL2_fix_vs_paperref=X.rel_l2(f_fix, f_ref_paper),
                   sm_err_fix_vs_dblADMM=sm_err(f_fix, refs[ncg] * s_f),
                   sm_err_fix_vs_gt=sm_err(f_fix, gtC),
                   rho_last=math.sqrt(R["res_sq"][-1] / d_sq_code) if d_sq_code > 0 else 0.0,
                   sat=R["sat"], mac_ovf=R["mac_ovf"], out_ovf=R["out_ovf"],
                   score_ovf=R["score_ovf"], gain_clamp=R["gain_clamp"],
                   cg_breaks=R["cg_breaks"],
                   res_sq_first=float(R["res_sq"][0]), res_sq_last=float(R["res_sq"][-1]),
                   obj_first=float(R["obj"][0]), obj_last=float(R["obj"][-1]),
                   split_first=float(R["split"][0]), split_last=float(R["split"][-1]),
                   zmax_first=float(R["zmax"][0]), zmax_last=float(R["zmax"][-1]),
                   cg_res_first=float(R["cg_res"][0]), cg_res_last=float(R["cg_res"][-1]),
                   res_sq_monotone_frac=float(np.mean(np.diff(R["res_sq"]) <= 1e-9)),
                   obj_monotone_frac=float(np.mean(np.diff(R["obj"]) <= 1e-9)))
        fixed[ncg] = rec
        log("    n_cg=%2d  r(gt)=%+.4f  relL2(fix vs 双精度ADMM)=%.4e  r(fix,dbl)=%.8f  sm_err=%.4e  rho_last=%.4f"
            % (ncg, rec["r_gt"], rec["relL2_fix_vs_dblADMM"], rec["r_fix_vs_dblADMM"],
               rec["sm_err_fix_vs_dblADMM"], rec["rho_last"]))
        log("            饱和=%d  out_ovf=%d  mac_ovf=%d  gain_clamp=%d  res_sq %.4g -> %.4g"
            % (rec["sat"], rec["out_ovf"], rec["mac_ovf"], rec["gain_clamp"],
               rec["res_sq_first"], rec["res_sq_last"]))
        if ncg == args.n_cg:
            hist = dict(res_sq=R["res_sq"].tolist(), obj=R["obj"].tolist(),
                        split=R["split"].tolist(), zmax=R["zmax"].tolist(),
                        cg_res=R["cg_res"].tolist())

    # ---- 5. 秩域中间量宽/窄对照（定点链、驱动执行、headline 档 n_cg=args.n_cg）----
    log("[5] 秩域中间量宽/窄（定点链、驱动执行）")
    rank_width_fixed = {}
    for tag, wid in (("wide_2p30", WIDE), ("narrow_q6_18", KIM)):
        ff, R, _ = run_fixed(args.n_cg, wide=wid)
        rank_width_fixed[tag] = dict(
            limit=int(wid), n_cg=args.n_cg, r_gt=X.pearson(ff, gtC),
            zmax_last=float(R["zmax"][-1]), sat=int(R["sat"]), out_ovf=int(R["out_ovf"]),
            cg_breaks=int(R["cg_breaks"]))
        log("    n_cg=%d %-12s (上限 %d)  r(gt)=%+.4f  |z|max=%.4f  饱和=%d  out_ovf=%d"
            % (args.n_cg, tag, wid, rank_width_fixed[tag]["r_gt"],
               rank_width_fixed[tag]["zmax_last"], rank_width_fixed[tag]["sat"],
               rank_width_fixed[tag]["out_ovf"]))

    # ---- 6. Q6.18 状态量化（双精度 ADMM、n_cg=15；隔离「状态位宽」）----
    log("[6] Q6.18 状态量化（双精度 ADMM）")
    f_pure, zmax_pure = admm_double_kernel_state_quant(Bk, s_f, dm, s_d, lam_q16,
                                                       rho_q16, args.n_outer, 15, quant=False)
    f_q618, zmax_q618 = admm_double_kernel_state_quant(Bk, s_f, dm, s_d, lam_q16,
                                                       rho_q16, args.n_outer, 15, quant=True)
    state_quant = dict(n_cg=15, n_outer=args.n_outer,
                       pure_double_r_gt=X.pearson(f_pure * s_f, gtC),
                       q6_18_state_r_gt=X.pearson(f_q618 * s_f, gtC),
                       pure_zmax=float(zmax_pure), q6_18_zmax=float(zmax_q618))
    log("    n_cg=15  纯双精度 r(gt)=%+.4f  Q6.18 状态 r(gt)=%+.4f  (|z|max %.3f / %.3f)"
        % (state_quant["pure_double_r_gt"], state_quant["q6_18_state_r_gt"],
           state_quant["pure_zmax"], state_quant["q6_18_zmax"]))

    # ---- 7. lr_forward→lr_adjoint 拼 BᵀB 的饱和对照（驱动执行，mode=1）----
    log("[7] lr_forward→lr_adjoint 拼 BᵀB（测量域 Q2.16 往返）对照")
    f_lr, Rlr, _ = run_fixed(args.n_cg, mode=1)
    lr_roundtrip = dict(
        n_cg=args.n_cg, r_gt=X.pearson(f_lr, gtC), zmax_last=float(Rlr["zmax"][-1]),
        sat=int(Rlr["sat"]), out_ovf=int(Rlr["out_ovf"]), mac_ovf=int(Rlr["mac_ovf"]),
        cg_breaks=int(Rlr["cg_breaks"]),
        fused_r_gt=fixed[args.n_cg]["r_gt"],
        fused_zmax_last=fixed[args.n_cg]["zmax_last"])
    log("    n_cg=%d  lr 往返 r(gt)=%+.4f  |z|max=%.4f  饱和=%d  out_ovf=%d  "
        "（融合算子同档 r(gt)=%+.4f）"
        % (args.n_cg, lr_roundtrip["r_gt"], lr_roundtrip["zmax_last"],
           lr_roundtrip["sat"], lr_roundtrip["out_ovf"], lr_roundtrip["fused_r_gt"]))

    out = dict(
        meta=dict(M=int(Bk.shape[0]), N=NH, rank=args.rank, shape=[ROWS, COLS],
                  s_d=s_d, s_f=s_f, norm_B_kernel=s_max * s_f / s_d, rho_q16=rho_q16,
                  rho_solver=rho_k, lambda_q16=lam_q16, lam_solver=lam_k,
                  operating_point="x35 s_f=128 + paper lambda reparametrised",
                  operator="fused normal-equation B^T B = V S^2 V^T, wide rank intermediate",
                  solver="ADMM + fixed inner CG + per-pixel TV shrink + nonneg",
                  note="host side only; reuses ps_fixed.hpp primitives via C++ driver; no HLS/RTL source touched",
                  elapsed_s=time.time() - t0),
        reference=dict(
            ls_r_gt=X.pearson(f_ls, gtC),
            paper_admm_r_gt=X.pearson(f_ref_paper, gtC),
            kernel_double_admm_r_gt=X.pearson(f_ref_kern * s_f, gtC),
            kernel_double_admm_relL2_vs_paper=X.rel_l2(f_ref_kern * s_f, f_ref_paper)),
        attribution=dict(
            fixed_operator_double_cg_r_gt=X.pearson(f_opfix * s_f, gtC),
            fixed_operator_double_cg_n_cg=args.n_cg,
            fixed_operator_saturation_events=int(op_sat)),
        rank_width=dict(
            note="same operator, double-precision CG, n_cg=15: narrow=KIM(2^23-1) vs wide=2^30-1",
            double_cg_narrow=rw["narrow_q6_18"],
            double_cg_wide=rw["wide_2p30"],
            fixed_wide=rank_width_fixed["wide_2p30"],
            fixed_narrow=rank_width_fixed["narrow_q6_18"]),
        state_quantization=state_quant,
        lr_roundtrip=lr_roundtrip,
        inner_cg_sweep=cg_sweep,
        fixed=fixed,
        history=hist)
    jp = args.out or os.path.join(RESULTS, "x36_admm_fixed.json")
    with open(jp, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    log("saved %s (%.1f s)" % (os.path.relpath(jp, REPO), time.time() - t0))
    return 0


if __name__ == "__main__":
    sys.exit(main())
