# -*- coding: utf-8 -*-
"""X35 - make the paper-chain regularisation strength representable in the
fixed-point reconstruction chain, and quantify the gap that remains.

What this script establishes (host side only, no HLS/RTL source touched)
-------------------------------------------------------------------------
1. Lambda dimensional audit. The paper regulariser, expressed in the units the
   fixed-point kernel actually integrates, is
       lambda_solver = (REG_R / CSCALE_R) * s_f / s_d^2
   With the X34 operating point (s_f = 1, s_d = 2^18) this is 2.328e-06, which is
   below one Q16.16 LSB (1.526e-05); lambda_q16 rounds to 0, so the chain runs
   with the TV term switched off.
2. Scale reparametrisation. The kernel already accepts an arbitrary power-of-two
   image scale s_f (it only changes the S code exponent s_exp and the f code
   range). Picking s_f by the same rule the B model uses (||B_kernel||_2 ~ 2)
   makes the paper lambda representable: at s_f = 128 the same physical lambda is
   lambda_q16 = 20 with overflow = 0.
3. Reference quality. With a converging solver (ADMM-CG, the repo's chosen M5
   solver) the paper lambda on the same operator gives r(gt) ~ +0.37, i.e. the
   quality the A model / ADMM chain already reaches.
4. Kernel quality. The frozen kernel is a fixed-step PDHG. On this operator it
   does not converge to the TV optimum in any feasible iteration count, at any
   scale and any representable lambda, so its r(gt) stays near zero / negative.

Reuses build/scripts/x20_fixed_vs_double.cpp (read only) as the fixed-vs-double
driver. Emits sim/results/x35_paper_lambda_fixed.json; the prose report lives in
sim/results/x35_paper_lambda_fixed.md.
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
import x20_hls_recon_image as X  # noqa: E402

VEC = os.path.join(REPO, "sim", "vectors")
RESULTS = os.path.join(REPO, "sim", "results")
WORK = os.path.join(REPO, "build", "x35_work")

N = 126
ROWS, COLS = 29, 36
NH = ROWS * COLS
SHAPE = (ROWS, COLS)
CSCALE_R = 312.5
REG_R = 1e7 * 5.0
Q16 = 65536.0
K_GRAD = 8.0          # upper bound on ||grad||^2 for the 2-component forward diff


def log(msg: str) -> None:
    print(msg, flush=True)


# ---------------------------------------------------------------- paper operator


def drows(M):
    o = np.empty_like(M)
    o[:-1, :] = M[:-1, :] - M[1:, :]
    o[-1, :] = M[-1, :]
    return o


def dvec(v):
    o = np.empty_like(v)
    o[:-1] = v[:-1] - v[1:]
    o[-1] = v[-1]
    return o


def rows_of(m2d, order):
    if order == "F":
        return np.ascontiguousarray(m2d.T).ravel(order="C")
    return np.ascontiguousarray(m2d).ravel(order="C")


def col_perm():
    return np.arange(NH).reshape(ROWS, COLS, order="F").ravel(order="C")


def build_paper_operator():
    """Same row/column convention as paper_recon_pdhg.py / x15g_model_error_tolerance.py."""
    A0 = np.load(os.path.join(VEC, "paper_A_va1.npy"))
    A3 = np.ascontiguousarray(A0.reshape(N, N, NH).transpose(1, 0, 2))
    dA = np.ascontiguousarray(drows(A3.reshape(N * N, NH))[:, col_perm()])
    del A0, A3
    return dA


def top_singular(dA, K):
    G = dA.T @ dA
    G = 0.5 * (G + G.T)
    lam_eig, Vc = np.linalg.eigh(G)
    order = np.argsort(lam_eig)[::-1]
    lam_eig = lam_eig[order]
    Vc = Vc[:, order]
    S = np.sqrt(np.maximum(lam_eig, 0.0))
    Vk = Vc[:, :K]
    s_max = float(S[0])
    Uk = (dA @ Vk) / np.maximum(S[:K][None, :], 1e-300)
    Uk = Uk / np.maximum(np.linalg.norm(Uk, axis=0, keepdims=True), 1e-300)
    return Uk, (S[:K] / s_max), Vk, s_max, Vc, S


# ---------------------------------------------------------------- host reference


def grad2(u2d):
    g = np.zeros((2,) + SHAPE)
    g[0, :-1, :] = u2d[1:, :] - u2d[:-1, :]
    g[1, :, :-1] = u2d[:, 1:] - u2d[:, :-1]
    return g


def div2(g):
    out = np.zeros(SHAPE)
    out[:-1, :] -= g[0, :-1, :]
    out[1:, :] += g[0, :-1, :]
    out[:, :-1] -= g[1, :, :-1]
    out[:, 1:] += g[1, :, :-1]
    return -out


def cg(op, b, x0, n_iter):
    x = x0.copy()
    r = b - op(x)
    p = r.copy()
    rs = float(r @ r)
    for _ in range(n_iter):
        Ap = op(p)
        den = float(p @ Ap)
        if not np.isfinite(den) or den <= 0:
            break
        a = rs / den
        x = x + a * p
        r = r - a * Ap
        rs_new = float(r @ r)
        p = r + (rs_new / rs) * p
        rs = rs_new
    return x


def admm_tv(Uk, s_max, S_norm, Vk, dm, lam, n_outer=400, n_cg=15, nonneg=True):
    """ADMM + fixed inner CG on min .5||dA f - dm||^2 + lam*TV(f), the M5 solver."""
    sig = S_norm * s_max
    Vst = Vk.T
    ATb = Vst.T @ (sig * (Uk.T @ dm))
    ATAop = lambda u: Vst.T @ (sig * (sig * (Vst @ u)))          # noqa: E731
    Lop = lambda u: -div2(grad2(u.reshape(SHAPE))).reshape(-1)   # noqa: E731
    rho = 0.1 * float(np.mean(sig ** 2))
    op = lambda u: ATAop(u) + rho * Lop(u)                       # noqa: E731
    u = np.zeros(NH)
    d = np.zeros((2,) + SHAPE)
    q = np.zeros((2,) + SHAPE)
    th = lam / rho
    for _ in range(n_outer):
        u = cg(op, ATb - rho * div2(d - q).reshape(-1), u, n_cg)
        if nonneg:
            u = np.maximum(0.0, u)
        g = grad2(u.reshape(SHAPE))
        s_ = g + q
        nrm = np.sqrt(s_[0] ** 2 + s_[1] ** 2)
        d = s_ * np.maximum(0.0, 1.0 - th / np.maximum(nrm, 1e-30))
        q = q + g - d
    return u, rho


# ---------------------------------------------------------------- kernel driver


def compile_driver(max_rank, exe):
    src = os.path.join(HERE, "x20_fixed_vs_double.cpp")
    cmd = ["g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Wno-unknown-pragmas",
           "-DPS_LR_MAX_RANK=%d" % max_rank, "-DPS_LR_MAX_RANK_FP=%d" % max_rank,
           "-Isrc/hardware", "-Isrc/software/include", "-Isrc/fpga/hls",
           "-Isrc/fpga/hls/pdhg_core", "-Isrc/fpga/hls/lr_operator", src,
           "src/fpga/hls/pdhg_core/pdhg_core_fixed.cpp",
           "src/fpga/hls/lr_operator/lr_operator_fixed.cpp",
           "src/fpga/hls/pdhg_core/pdhg_core.cpp",
           "src/fpga/hls/lr_operator/lr_operator.cpp", "-o", exe]
    r = subprocess.run(cmd, cwd=REPO, capture_output=True, encoding="utf-8",
                       errors="replace")
    if r.returncode != 0:
        log(r.stdout)
        log(r.stderr)
        raise SystemExit("driver compile failed")


def choose_s_d(dm):
    p = int(math.ceil(math.log2(max(float(np.abs(dm).max()) / 1.5, 1e-30))))
    return float(2 ** max(0, p))


def patch_iters(case, n_iter):
    with open(case, "r+b") as f:
        f.seek(4 + 5 * 4)
        f.write(struct.pack("<i", int(n_iter)))


def run_kernel(exe, case, resf, cfg, U_store, S_code, V_store, exps, d_code,
               Ud_ref, Sd_ref, Vd_ref, D, n_iter):
    c = dict(cfg)
    c["n_iter"] = n_iter
    X.write_case(case, c, U_store, S_code, V_store, exps, d_code,
                 Ud_ref, Sd_ref, Vd_ref, D)
    r = subprocess.run([exe, case, resf], capture_output=True, encoding="utf-8",
                       errors="replace")
    if r.returncode != 0:
        log(r.stdout)
        log(r.stderr)
        raise SystemExit("driver failed")
    diag = {}
    for line in r.stdout.splitlines():
        if line.startswith("[A fixed]"):
            for tok in line.split():
                if "=" in tok:
                    k, v = tok.split("=", 1)
                    diag[k] = v
    return X.read_result(resf), diag


# ---------------------------------------------------------------- main


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rank", type=int, default=96)
    ap.add_argument("--iters", default="16,256,1024,4096",
                    help="fixed-point iterations (kernel caps at 4096)")
    ap.add_argument("--s-f", type=int, default=128, help="corrected image scale (2^k)")
    ap.add_argument("--status-quo-sf", type=int, default=1)
    ap.add_argument("--out", default="")
    args = ap.parse_args()
    os.makedirs(WORK, exist_ok=True)
    t0 = time.time()

    log("=" * 84)
    log("X35  paper-chain lambda representation in the fixed-point chain")
    log("=" * 84)

    # ---- 0. lambda dimensional audit -------------------------------------
    s_d = None
    dA = build_paper_operator()
    meas = np.load(os.path.join(VEC, "paper_meas.npy"))
    dm = dvec(rows_of(meas[:, :, 0], "F"))
    gt = np.load(os.path.join(VEC, "paper_gt.npy"))
    gtC = gt[:, :, 0].astype(float).reshape(-1, order="C")
    ref = np.load(os.path.join(VEC, "paper_recon_pdhg.npy"))[:, :, 0]
    refC = ref.astype(float).reshape(-1, order="C") * CSCALE_R
    s_d = choose_s_d(dm)
    lam_paper = REG_R / CSCALE_R

    audit = {}
    log("[0] lambda dimensional audit (R channel; s_d = 2^%.0f = %.0f)"
        % (math.log2(s_d), s_d))
    for cn, cs, rg in (("R", 312.5, 5e7), ("G", 425.34722222, 25e7),
                       ("B", 496.54222222, 25e7)):
        lp = rg / cs
        ls_ = lp * 1.0 / (s_d ** 2)
        audit[cn] = dict(reg=rg, cscale=cs, lam_paper=lp, lam_solver_sf1=ls_,
                         lam_q16_sf1=int(round(ls_ * Q16)),
                         q16_lsb=1.0 / Q16, frac_bits_needed=-math.log2(ls_))
        log("    %s: REG/CS = %.6g   lambda_solver(s_f=1) = %.6g   "
            "lambda_q16 = %d   (need %.2f frac bits)"
            % (cn, lp, ls_, int(round(ls_ * Q16)), -math.log2(ls_)))
    log("    Q16.16 LSB = %.6g -> R and B round to 0; G rounds to 1 (79%% high)"
        % (1.0 / Q16))

    # ---- 1. operator + reference solve ----------------------------------
    log("[1] rank-%d operator; ADMM-CG reference at the paper lambda" % args.rank)
    Uk, S, Vk, s_max, _Vc, _S = top_singular(dA, args.rank)
    V = np.ascontiguousarray(Vk.T)
    f_ls = Vk @ ((Uk.T @ dm) / (S * s_max))
    f_ref_admm, rho = admm_tv(Uk, s_max, S, Vk, dm, lam_paper, nonneg=True)
    log("    s_max = %.4f   rho_ref = %.6g" % (s_max, rho))
    log("    exact truncated LS      : r(gt)=%+.4f  r(paper_ref)=%+.4f"
        % (X.pearson(f_ls, gtC), X.pearson(f_ls, refC)))
    log("    ADMM-CG at paper lambda : r(gt)=%+.4f  r(paper_ref)=%+.4f"
        % (X.pearson(f_ref_admm, gtC), X.pearson(f_ref_admm, refC)))

    # ADMM reference quality across ranks (reuses the Gram eigendecomposition)
    ref_by_rank = {}
    for K2 in (96, 192, 384):
        Vk2 = _Vc[:, :K2]
        s2 = _S[:K2]
        Uk2 = (dA @ Vk2) / np.maximum(s2[None, :], 1e-300)
        Uk2 = Uk2 / np.maximum(np.linalg.norm(Uk2, axis=0, keepdims=True), 1e-300)
        f2, _ = admm_tv(Uk2, s_max, s2 / s_max, Vk2, dm, lam_paper, nonneg=True)
        fl2, _ = admm_tv(Uk2, s_max, s2 / s_max, Vk2, dm, lam_paper, nonneg=False)
        ref_by_rank[K2] = dict(r_gt_nonneg=X.pearson(f2, gtC),
                               r_gt_free=X.pearson(fl2, gtC),
                               r_ref_nonneg=X.pearson(f2, refC),
                               ceiling_gt=X.pearson(Vk2 @ (Vk2.T @ gtC), gtC))
        log("    rank %3d: ADMM r(gt) nonneg=%+.4f free=%+.4f | row-space ceiling=%+.4f"
            % (K2, ref_by_rank[K2]["r_gt_nonneg"], ref_by_rank[K2]["r_gt_free"],
               ref_by_rank[K2]["ceiling_gt"]))
        del Uk2, f2, fl2

    # ---- 2. scale reparametrisation -------------------------------------
    # lam_solver = lam_paper * s_f / s_d^2 ; want lam_q16 = round(lam_solver*2^16) >= 8
    s_f_min = (8.0 / Q16) * (s_d ** 2) / lam_paper
    s_f_bal = math.sqrt(K_GRAD) * s_d / s_max       # ||B_kernel|| ~ sqrt(||grad||^2)
    log("[2] scale reparametrisation")
    log("    lambda_solver = lam_paper * s_f / s_d^2")
    log("    smallest s_f with lambda_q16 >= 8       : %.3f  -> 2^%d = %.0f"
        % (s_f_min, math.ceil(math.log2(max(s_f_min, 1.0))),
           2 ** math.ceil(math.log2(max(s_f_min, 1.0)))))
    log("    s_f that balances ||B||^2 with ||grad||^2: %.2f  -> 2^%d = %.0f"
        % (s_f_bal, int(round(math.log2(s_f_bal))), 2 ** int(round(math.log2(s_f_bal)))))

    # ---- 3. kernel runs -------------------------------------------------
    log("[3] frozen kernel (fixed-step PDHG) vs the ADMM reference")
    exe = os.path.join(WORK, "x35_driver.exe")
    compile_driver(max(args.rank, 96), exe)
    iters = [int(x) for x in args.iters.split(",")]
    D, d_code = X.quantize_measurement(dm, s_d)
    Ud_ref = np.ascontiguousarray(Uk.T)
    Vd_ref = np.ascontiguousarray(V)

    table = []
    for tag, s_f in (("status_quo", float(args.status_quo_sf)),
                     ("rescored", float(args.s_f))):
        U_store, S_code, V_store, u_exp, v_exp, s_exp, _ = X.quantize_factors(
            Uk, S, V, s_max, s_f, s_d)
        Sd_ref = S * (s_max * s_f / s_d)
        norm_B = s_max * s_f / s_d
        tau = math.sqrt(0.99) / math.sqrt(norm_B * norm_B + K_GRAD)
        tau_q16 = int(round(tau * Q16))
        lam_real = lam_paper * s_f / (s_d ** 2)
        lam_q16 = int(round(lam_real * Q16))
        log("    [%s] s_f=%.0f  ||B_kernel||=%.4f  |B|^2=%.4f  tau=%.4f  "
            "lambda_solver=%.6g  lambda_q16=%d"
            % (tag, s_f, norm_B, norm_B ** 2, tau, lam_real, lam_q16))
        cfg = dict(n_rows=ROWS, n_cols=COLS, min_iters=8, residual_every=4,
                   tv_mode=0, tau_q16=tau_q16, sigma_q16=tau_q16, theta_q16=65536,
                   lambda_q16=lam_q16, rho_stop_q8=0)
        for ni in iters:
            case = os.path.join(WORK, "case_%s_%d.bin" % (tag, ni))
            resf = os.path.join(WORK, "res_%s_%d.bin" % (tag, ni))
            R, diag = run_kernel(exe, case, resf, cfg, U_store, S_code, V_store,
                                 (u_exp, v_exp, s_exp), d_code, Ud_ref, Sd_ref,
                                 Vd_ref, D, ni)
            ff = R["fixed"] * s_f
            fr = R["dbl_ref"] * s_f
            rec = dict(tag=tag, s_f=s_f, n_iter=ni, n_used=R["n_used"],
                       overflow=R["overflow"], norm_B=norm_B, tau=tau,
                       lam_solver=lam_real, lambda_q16=lam_q16,
                       u_exp=u_exp, v_exp=v_exp, s_exp=s_exp,
                       S_code_max=int(S_code.max()), d_code_max=int(np.abs(d_code).max()),
                       fixed_vs_dblref=X.metrics(ff, fr),
                       fixed_vs_gt=dict(X.metrics(ff, gtC), pearson_ref=X.pearson(ff, refC)),
                       dblref_vs_gt=dict(X.metrics(fr, gtC), pearson_ref=X.pearson(fr, refC)),
                       dblref_vs_admm=X.metrics(fr, f_ref_admm),
                       fixed_vs_admm=X.metrics(ff, f_ref_admm),
                       res_sq=diag.get("res_sq"), rho_q8=diag.get("rho"))
            table.append(rec)
            log("        N=%4d ovf=%d | fixed vs dbl relL2=%.3e r=%.10f | "
                "r(gt) fix=%+.4f dbl=%+.4f | relL2(dbl,ADMM)=%.4f | res_sq=%s rho=%s"
                % (ni, R["overflow"], rec["fixed_vs_dblref"]["rel_l2"],
                   rec["fixed_vs_dblref"]["pearson"], rec["fixed_vs_gt"]["pearson"],
                   rec["dblref_vs_gt"]["pearson"],
                   rec["dblref_vs_admm"]["rel_l2"], rec["res_sq"], rec["rho_q8"]))
        del U_store, S_code, V_store

    # ---- 4. is lambda the binding constraint? sweep it at the corrected scale ----
    log("[4] lambda sweep at the corrected scale (s_f=%d, N=1024): if the "
        "reconstruction is limited by lambda, r(gt) must move with lambda_q16"
        % args.s_f)
    s_f = float(args.s_f)
    U_store, S_code, V_store, u_exp, v_exp, s_exp, _ = X.quantize_factors(
        Uk, S, V, s_max, s_f, s_d)
    Sd_ref = S * (s_max * s_f / s_d)
    norm_B = s_max * s_f / s_d
    tau = math.sqrt(0.99) / math.sqrt(norm_B * norm_B + K_GRAD)
    tau_q16 = int(round(tau * Q16))
    sweep = []
    for lam_q16 in (0, 20, 300, 3000):
        cfg = dict(n_rows=ROWS, n_cols=COLS, min_iters=8, residual_every=4,
                   tv_mode=0, tau_q16=tau_q16, sigma_q16=tau_q16, theta_q16=65536,
                   lambda_q16=lam_q16, rho_stop_q8=0)
        case = os.path.join(WORK, "case_sweep_%d.bin" % lam_q16)
        resf = os.path.join(WORK, "res_sweep_%d.bin" % lam_q16)
        R, _ = run_kernel(exe, case, resf, cfg, U_store, S_code, V_store,
                          (u_exp, v_exp, s_exp), d_code, Ud_ref, Sd_ref, Vd_ref,
                          D, 1024)
        ff = R["fixed"] * s_f
        fr = R["dbl_ref"] * s_f
        rec = dict(lambda_q16=lam_q16, lam_solver=lam_q16 / Q16,
                   r_gt_fix=X.pearson(ff, gtC), r_gt_dbl=X.pearson(fr, gtC),
                   r_ref_fix=X.pearson(ff, refC),
                   fixed_vs_dblref=X.metrics(ff, fr)["rel_l2"],
                   overflow=R["overflow"])
        sweep.append(rec)
        log("        lambda_q16=%5d (lambda=%9.3g): r(gt) fix=%+.4f dbl=%+.4f | "
            "relL2(fix,dbl)=%.3e ovf=%d"
            % (lam_q16, rec["lam_solver"], rec["r_gt_fix"], rec["r_gt_dbl"],
               rec["fixed_vs_dblref"], R["overflow"]))
    del U_store, S_code, V_store

    out = dict(
        meta=dict(M=int(dA.shape[0]), N=int(dA.shape[1]), rank=args.rank,
                  shape=list(SHAPE), s_d=s_d,
                  operator="fig4_column_c reconstruction chain (126x126 wall)",
                  measurement="paper_meas R channel, F-order flatten + row difference",
                  solver_kernel="ps::pdhg_solve_channel (fixed-step PDHG)",
                  solver_reference="ADMM + fixed inner CG (M5 decision, x15g)",
                  note="host-side only; no HLS/RTL source touched",
                  elapsed_s=time.time() - t0),
        lambda_audit=audit,
        lam_paper_R=lam_paper,
        reference=dict(rho=rho,
                       ls=dict(r_gt=X.pearson(f_ls, gtC), r_ref=X.pearson(f_ls, refC)),
                       admm_paper_lam=dict(r_gt=X.pearson(f_ref_admm, gtC),
                                           r_ref=X.pearson(f_ref_admm, refC))),
        reference_by_rank=ref_by_rank,
        s_f_min_for_q16=int(8),
        s_f_min=float(s_f_min),
        s_f_balanced=float(s_f_bal),
        table=table,
        lambda_sweep=sweep)
    jp = args.out or os.path.join(RESULTS, "x35_paper_lambda_fixed.json")
    with open(jp, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    log("saved %s (%.1f s)" % (os.path.relpath(jp, REPO), time.time() - t0))
    return 0


if __name__ == "__main__":
    sys.exit(main())
