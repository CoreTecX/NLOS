# -*- coding: utf-8 -*-
"""X32 —— 两件事的端到端复核（用真实 .psm，不现场重算算子）

(1) 复现 X20 的 psm 路径重建，但**绝对幅度 s_max 从 psm 候选头读**（不再读 manifest），
    核对与 X20 的两条路径结论是否一致（相对 L2 / 相关系数 / PSNR）。
(2) 用实时 HLS 定点核验证任务 1 的折算下发路径：把 s_max 与 s_scale 折成
    s_gain_q16 / s_exp 写进 S 级寄存器后，重建与「不折算（只用 s_scale）」相比
    幅度是否正确、与精确参考差多少。

复用 build/scripts/x20_hls_recon_image.py 的算子读取与 case/驱动编排（import 调其函数），
只把 s_max 的来源从 manifest 换成 psm 候选头，并另加折算支路的一支驱动
build/scripts/x32_s_fold_driver.cpp。

用法（仓库根执行）：
    python build/scripts/x32_psm_e2e_smax.py
中间产物落 build/x32_work/；结果写 sim/results/x32_smax_fold_and_psm_e2e.json。
"""

from __future__ import annotations

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
BUILD_SCRIPTS = HERE
if BUILD_SCRIPTS not in sys.path:
    sys.path.insert(0, BUILD_SCRIPTS)
MODEL = os.path.join(REPO, "src", "model")
if MODEL not in sys.path:
    sys.path.insert(0, MODEL)

import x20_hls_recon_image as x20          # noqa: E402  （复用其算子读取/编排）
from penumbrasight_model import psm        # noqa: E402

WORK = os.path.join(REPO, "build", "x32_work")
RESULTS = os.path.join(REPO, "sim", "results")
PSM_PATH = os.path.join(REPO, "sim", "models", "model_b.psm")
MANIFEST_PATH = os.path.join(REPO, "sim", "models", "model_b.manifest.json")
CAND = x20.CAND
SHAPE = x20.SHAPE
PEAK = x20.PEAK


def log(msg):
    print(msg, flush=True)


# ---------------------------------------------------------------- psm 记录

def read_psm_candidate(path, cand):
    """直接按 psm_format.h 的偏移读候选记录（含 +20 的 s_max），不做现场重算。"""
    with open(path, "rb") as f:
        hdr = psm.PsmHeader.unpack(f.read(psm.HEADER_BYTES))
        base = psm.HEADER_BYTES + cand * hdr.record_bytes
        f.seek(base)
        raw = f.read(hdr.record_bytes)
    rec = psm._unpack_record(hdr, raw)
    rec.validate(hdr)
    return hdr, rec


# ---------------------------------------------------------------- 折算（与 C 端同口径）

def round_half_even(x):
    return float(np.rint(x))


def fold_multiplier(m):
    """复刻 fix_core_driver.c 的 ps_fix_fold_multiplier：2 的幂 -> exp 支路（精确）；
    否则 Q16.16 支路，exp = kGainExpInvalid。"""
    GAIN_EXP_INVALID = 0x7FFFFFFF
    m = float(m)
    e = math.log2(m)
    if abs(e - round(e)) < 1e-12:                 # m 是 2 的幂
        return dict(s_gain_q16=0, s_exp=int(round(e)), multiplier=float(2.0 ** round(e)),
                    rel_err=0.0, exact=1)
    gain = int(round_half_even(m * 65536.0))
    mult = gain / 65536.0
    return dict(s_gain_q16=gain, s_exp=GAIN_EXP_INVALID, multiplier=mult,
                rel_err=abs(mult - m) / m, exact=0)


def fold_s_scale(s_scale, s_max):
    return fold_multiplier(float(s_scale) * float(s_max))


# ---------------------------------------------------------------- case 写出

def write_case_x20c(path, cfg, U_store, S_code, V_store, exps, d_code,
                    Ud_ref, Sd_ref, Vd_ref, d_real):
    hdr = [U_store.shape[0], U_store.shape[1], V_store.shape[1],
           cfg["n_rows"], cfg["n_cols"], cfg["n_iter"], cfg["min_iters"],
           cfg["residual_every"], cfg["tv_mode"], cfg["tau_q16"], cfg["sigma_q16"],
           cfg["theta_q16"], cfg["lambda_q16"], cfg["rho_stop_q8"], 0]
    with open(path, "wb") as f:
        f.write(b"X20C")
        f.write(struct.pack("<%di" % len(hdr), *hdr))
        f.write(struct.pack("<3i", *exps))
        f.write(U_store.astype("<i2").tobytes(order="C"))
        f.write(S_code.astype("<u2").tobytes(order="C"))
        f.write(V_store.astype("<i2").tobytes(order="C"))
        f.write(d_code.astype("<i4").tobytes(order="C"))
        f.write(np.ascontiguousarray(Ud_ref, "<f8").tobytes(order="C"))
        f.write(np.ascontiguousarray(Sd_ref, "<f8").tobytes(order="C"))
        f.write(np.ascontiguousarray(Vd_ref, "<f8").tobytes(order="C"))
        f.write(np.ascontiguousarray(d_real, "<f8").tobytes(order="C"))


def read_result_x32(path):
    with open(path, "rb") as f:
        magic = f.read(4)
        assert magic == b"X32R", magic
        hd = struct.unpack("<4i", f.read(16))
        n = hd[0]
        fx = np.frombuffer(f.read(8 * n), "<f8").copy()
        fb = np.frombuffer(f.read(8 * n), "<f8").copy()
        fc = np.frombuffer(f.read(8 * n), "<f8").copy()
    return dict(n_used=hd[2], overflow=hd[3], fixed=fx, dbl_same=fb, dbl_ref=fc)


def compile_fold_driver():
    exe = os.path.join(WORK, "x32_fold_driver.exe")
    try:
        if os.path.exists(exe):
            os.remove(exe)
    except OSError:
        pass
    src = os.path.join(HERE, "x32_s_fold_driver.cpp")
    cmd = ["g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Wno-unknown-pragmas",
           "-Isrc/hardware", "-Isrc/software/include", "-Isrc/fpga/hls",
           "-Isrc/fpga/hls/pdhg_core", "-Isrc/fpga/hls/lr_operator",
           src,
           "src/fpga/hls/pdhg_core/pdhg_core_fixed.cpp",
           "src/fpga/hls/lr_operator/lr_operator_fixed.cpp",
           "src/fpga/hls/pdhg_core/pdhg_core.cpp",
           "src/fpga/hls/lr_operator/lr_operator.cpp",
           "-o", exe]
    log("[fold] 编译折算支路驱动 ...")
    r = subprocess.run(cmd, cwd=REPO, capture_output=True,
                       encoding="utf-8", errors="replace")
    if r.returncode != 0:
        log(r.stdout); log(r.stderr)
        raise SystemExit("折算驱动编译失败")
    return exe


def main():
    os.makedirs(WORK, exist_ok=True)
    t0 = time.time()
    log("=" * 88)
    log("X32：真实 .psm 端到端重跑（s_max 取自 psm）+ 折算下发路径（s_gain_q16/s_exp）")
    log("=" * 88)

    if not os.path.exists(PSM_PATH):
        raise SystemExit("找不到 %s（.psm 不入库，需先按 sim/models/readme.md 生成）"
                         % PSM_PATH)

    # ---------- 读 psm（幅度只来自这里）----------
    hdr, rec = read_psm_candidate(PSM_PATH, CAND)
    s_max_psm = float(rec.s_max)
    log("[psm] %s：meas %dx%d hidden %dx%d candidates=%d rank_max=%d geom=%08X" %
        (os.path.relpath(PSM_PATH, REPO), hdr.meas_h, hdr.meas_w,
         hdr.hidden_h, hdr.hidden_w, hdr.candidates, hdr.rank_max, hdr.geometry_hash))
    log("      候选 %d：u_scale=2^%d s_scale=2^%d v_scale=2^%d  s_max(psm)=%.9f" %
        (CAND, int(round(math.log2(rec.u_scale))), int(round(math.log2(rec.s_scale))),
         int(round(math.log2(rec.v_scale))), s_max_psm))

    # ---------- (1) 复现 X20 psm 路径，但 s_max 从 psm 读 ----------
    fld = x20.build_field_operator()              # 现场算子只用缓存，不重算
    Up, Sp, Vp, psm_meta = x20.load_psm_operator()
    f2d, F = x20.build_target()
    driver_x20 = x20.compile_driver()

    log("[1/3] psm 路径重建（s_max 来自 psm，全程不读 manifest）...")
    fld_configs = [("main", {"n_iter": 1024, "lambda_q16": 66})]
    # psm 路径：只用主配置，足够与 X20 的 2.1957e-04 / 2.5546e-04 对照
    runs_psm, sols_psm, meta_psm = x20.run_source(
        "psm", Up, Sp, Vp, s_max_psm, F, f2d,
        [("main", {"n_iter": 1024, "lambda_q16": 66})], driver_x20, extra_meta=psm_meta)
    cross = x20.metrics(sols_psm["main"]["fixed"], sols_psm["main"]["dbl_ref"])
    rec_psm = runs_psm["main"]
    log("      psm 路径 main：定点 vs psm双精度 relL2=%.4e r=%.10f PSNR=%.2f dB" %
        (rec_psm["fixed_vs_dblref"]["rel_l2"], rec_psm["fixed_vs_dblref"]["pearson"],
         rec_psm["fixed_vs_dblref"]["psnr_db"]))
    log("      psm 定点 vs 隐藏真值 r=%.4f" % rec_psm["fixed_vs_gt"]["pearson"])

    # 跨路径（psm 定点 vs 现场重算双精度参考）——与 X20 的 2.5546e-04 对照
    runs_fld, sols_fld, meta_fld = x20.run_source(
        "fld", fld[0], fld[1], fld[2], fld[3], F, f2d,
        fld_configs, driver_x20)
    cross_path = x20.metrics(sols_psm["main"]["fixed"], sols_fld["main"]["dbl_ref"])
    log("      跨路径：psm 定点 vs 现场双精度参考 relL2=%.4e r=%.10f PSNR=%.2f dB" %
        (cross_path["rel_l2"], cross_path["pearson"], cross_path["psnr_db"]))

    # ---------- (2) 折算下发路径：核内用 s_gain_q16 支路 ----------
    log("[2/3] 折算：s_max 与 s_scale -> s_gain_q16 / s_exp ...")
    # 模型幅度折算（不含帧量程）：乘数 = s_scale*s_max
    fold_model = fold_s_scale(rec.s_scale, s_max_psm)
    log("      模型幅度：m = s_scale*s_max = %.9e -> s_gain_q16=%d s_exp=0x%08X 乘数=%.9e rel_err=%.4e" %
        (rec.s_scale * s_max_psm, fold_model["s_gain_q16"], fold_model["s_exp"] & 0xFFFFFFFF,
         fold_model["multiplier"], fold_model["rel_err"]))

    U_dq = rec.U.astype(np.float64).T * rec.u_scale              # (M, rank)
    V_dq = rec.V.astype(np.float64) * rec.v_scale                # (rank, N)
    S_code = rec.S.astype(np.float64)                            # Q2.14 归一化码字
    S_abs = S_code * rec.s_scale * s_max_psm                     # 精确绝对奇异值
    B_exact = (U_dq * S_abs[None, :]) @ V_dq                     # psm 的绝对算子
    d_unit = B_exact @ F
    s_f, s_d = x20.choose_scales(F, d_unit, s_max_psm)           # B 档 -> (1.0, 32.0)

    # 重建用的 S 乘数 = s_scale * s_max * (s_f/s_d)；与 X20 的码字预缩放同源。
    m_rec = rec.s_scale * s_max_psm * (s_f / s_d)
    fold = fold_multiplier(m_rec)
    log("      重建量程 s_f=%.0f s_d=%.0f：m_rec = m*(s_f/s_d) = %.9e -> s_gain_q16=%d s_exp=0x%08X rel_err=%.4e" %
        (s_f, s_d, m_rec, fold["s_gain_q16"], fold["s_exp"] & 0xFFFFFFFF, fold["rel_err"]))

    # 三种算子（相对同一参考 O_ref = psm 绝对算子 / s_d）
    O_ref = (U_dq * (S_code * (rec.s_scale * s_max_psm / s_d))[None, :]) @ V_dq   # 精确（含量程比）
    O_fold = (U_dq * (S_code * fold["multiplier"])[None, :]) @ V_dq              # 折算后（核实际用）
    O_naive = (U_dq * (S_code * rec.s_scale)[None, :]) @ V_dq                    # 缺 s_max（任务 1 之前）
    O_foldfull = (U_dq * (S_code * fold_model["multiplier"])[None, :]) @ V_dq    # 模型幅度折算，满量程
    op = {
        "fold_vs_ref": x20.metrics(O_fold, O_ref),
        "naive_vs_ref": x20.metrics(O_naive, O_ref),
        "foldmodel_vs_abs": x20.metrics(O_foldfull, B_exact),
        "spec_ref": float(np.linalg.norm(O_ref, 2)),
        "spec_abs": float(np.linalg.norm(B_exact, 2)),
    }
    log("      算子级：折算 vs 精确(含量程) relL2=%.4e r=%.10f | 缺 s_max vs 精确 relL2=%.4e | 模型幅度折算 vs 绝对算子 relL2=%.4e" %
        (op["fold_vs_ref"]["rel_l2"], op["fold_vs_ref"]["pearson"],
         op["naive_vs_ref"]["rel_l2"], op["foldmodel_vs_abs"]["rel_l2"]))

    norm_B = op["spec_ref"]
    tau = math.sqrt(0.99) / math.sqrt(norm_B * norm_B + 8.0)
    tau_q16 = int(round(tau * 65536.0))

    driver_fold = compile_fold_driver()
    U_store = np.ascontiguousarray(rec.U.astype(np.int16))          # (rank, M) 原始码字
    V_store = np.ascontiguousarray(rec.V.astype(np.int16))          # (rank, N)
    Ud_ref = np.ascontiguousarray(U_dq.T)                           # (rank, M)
    Vd_ref = np.ascontiguousarray(V_dq)
    Sd_ref = S_code * (rec.s_scale * s_max_psm / s_d)               # 精确（含量程比）

    d_real = d_unit / s_d
    d_code = np.clip(np.rint(d_real * 65536.0), -131071, 131071).astype(np.int32)
    base = dict(n_rows=SHAPE[0], n_cols=SHAPE[1], n_iter=1024, min_iters=8,
                residual_every=4, tv_mode=0, tau_q16=tau_q16, sigma_q16=tau_q16,
                theta_q16=65536, lambda_q16=66, rho_stop_q8=0)

    # 三种「码字 / 标量」组合，参考同为 O_ref：
    #   fold     : 模型幅度 s_scale*s_max 进标量，帧量程比 c=s_f/s_d 留码字（任务 1 目标形态）
    #   fold_alt : 模型幅度与帧量程比都进标量（单旋钮形态）
    #   naive    : 只用 s_scale，缺 s_max（任务 1 之前）
    u_exp_i = int(round(math.log2(rec.u_scale)))
    v_exp_i = int(round(math.log2(rec.v_scale)))
    s_codes = np.ascontiguousarray(
        np.rint(rec.S.astype(np.float64) * (s_f / s_d)).astype(np.uint16))

    results = {}
    for tag, S_arr, s_exp, s_gain, note in (
            ("fold", s_codes, fold["s_exp"], fold_model["s_gain_q16"],
             "模型幅度进标量、帧量程比留码字（任务 1 目标形态）"),
            ("fold_alt", rec.S, fold["s_exp"], fold["s_gain_q16"],
             "模型幅度与帧量程比都进标量（单旋钮形态）"),
            ("naive", s_codes, int(round(math.log2(rec.s_scale))), 0,
             "只用 s_scale，缺 s_max（任务 1 之前）")):
        case = os.path.join(WORK, "case_%s.bin" % tag)
        res = os.path.join(WORK, "res_%s.bin" % tag)
        write_case_x20c(case, base, U_store, S_arr, V_store,
                        (u_exp_i, v_exp_i, s_exp), d_code, Ud_ref, Sd_ref, Vd_ref, d_real)
        cmd = [driver_fold, case, res]
        if s_gain:
            cmd.append(str(s_gain))
        r = subprocess.run(cmd, cwd=REPO, capture_output=True,
                           encoding="utf-8", errors="replace")
        if r.returncode != 0:
            log(r.stdout); log(r.stderr)
            raise SystemExit("折算驱动运行失败：%s" % tag)
        for line in r.stdout.strip().splitlines():
            log("      [%s] %s" % (tag, line))
        R = read_result_x32(res)
        f_fix = R["fixed"]
        gt = F * 1.0
        results[tag] = dict(
            note=note, s_gain_q16=s_gain, s_exp=int(s_exp),
            fixed_vs_dblref=x20.metrics(f_fix, R["dbl_ref"]),
            fixed_vs_dblsame=x20.metrics(f_fix, R["dbl_same"]),
            fixed_vs_gt=x20.metrics(f_fix, gt),
            n_used=int(R["n_used"]), overflow=int(R["overflow"]),
        )
        m = results[tag]["fixed_vs_dblref"]
        log("      %-6s %s：定点 vs 精确双精度 relL2=%.4e r=%.8f PSNR=%.2f dB | vs 真值 r=%.4f" %
            (tag, note, m["rel_l2"], m["pearson"], m["psnr_db"],
             results[tag]["fixed_vs_gt"]["pearson"]))

    # ---------- (3) 「不依赖 manifest」的证据 ----------
    log("[3/3] manifest 对照（仅核对，主流程未读它）...")
    with open(MANIFEST_PATH, "r", encoding="utf-8") as f:
        man = json.load(f)
    s_max_manifest = float(man["candidates_meta"][CAND]["s_max"])
    # psm 里 s_max 是 float32，manifest 里是 float64：按 float32 精度比（相对 1e-6）。
    dmax = abs(s_max_psm - s_max_manifest)
    same = dmax <= 1e-6 * abs(s_max_manifest)
    log("      s_max(psm,float32)=%.9f  s_max(manifest,float64)=%.9f  |差|=%.3e  一致=%s" %
        (s_max_psm, s_max_manifest, dmax, same))

    # 对照：同一条 psm 路径，但 s_max 改用 manifest 的 float64 值 —— 用来归因
    # 「psm 里 s_max 是 float32」这一点对结果的影响（不是幅度来源变了）。
    log("[3b] 对照：同一 psm 路径、s_max 改用 manifest 的 float64 值 ...")
    runs_ctrl, _sols_ctrl, _mctrl = x20.run_source(
        "psmctrl", Up, Sp, Vp, s_max_manifest, F, f2d,
        [("main", {"n_iter": 1024, "lambda_q16": 66})], driver_x20, extra_meta=psm_meta)
    ctrl = runs_ctrl["main"]["fixed_vs_dblref"]
    log("      manifest s_max 路径：定点 vs psm双精度 relL2=%.4e r=%.10f PSNR=%.2f dB" %
        (ctrl["rel_l2"], ctrl["pearson"], ctrl["psnr_db"]))

    out = {
        "meta": dict(cand=CAND, rank=int(hdr.rank_max), shape=list(SHAPE),
                     psm=os.path.relpath(PSM_PATH, REPO),
                     geometry_hash=hdr.geometry_hash, payload_crc32=hdr.payload_crc32,
                     u_exp=int(round(math.log2(rec.u_scale))),
                     v_exp=int(round(math.log2(rec.v_scale))),
                     s_scale=rec.s_scale, s_max_psm=s_max_psm,
                     s_max_manifest=s_max_manifest, s_max_abs_diff=dmax,
                     s_max_identical=bool(same),
                     fold_model=fold_model, fold_recon=fold,
                     s_f=s_f, s_d=s_d, m_rec=m_rec, norm_B=norm_B, tau_q16=tau_q16,
                     elapsed_s=time.time() - t0,
                     note="主流程的绝对幅度只来自 psm 候选头 +20；manifest 仅用于第 3 步核对"),
        "x20_psm_path": dict(runs=rec_psm, cross=cross_path),
        "manifest_smax_control": ctrl,
        "operator": op,
        "fold_branch": results,
        "reference_x20": dict(
            psm_path_fixed_vs_dblref_relL2=2.1957e-4,
            psm_path_psnr_db=90.75,
            cross_path_relL2=2.5546e-4,
            cross_path_psnr_db=89.44,
            fixed_vs_gt_r=0.4335),
    }
    jp = os.path.join(RESULTS, "x32_smax_fold_and_psm_e2e.json")
    with open(jp, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    log("已保存 %s；总用时 %.1f s" % (os.path.relpath(jp, REPO), time.time() - t0))
    return 0


if __name__ == "__main__":
    sys.exit(main())
