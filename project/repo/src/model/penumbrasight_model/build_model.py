# -*- coding: utf-8 -*-
"""build_model.py —— model.psm 生成器（方案 §5.2 / §6.3 / §8.1，§17 必交件）。

链路：
    几何配置 -> 候选网格 -> 每个候选建差分传输矩阵 B_p = D_y A(p)
    -> 截断 SVD（走 Gram，避免对大矩阵做全 SVD）
    -> 按方案 §6.3 量化（U/V = Q1.15 + 每候选独立 scale，S = Q2.14）
    -> 写 model.psm（含 geometry_hash 与 payload CRC32）
    -> 回读校验（重新解析头部 + 全文件 CRC + 只读前 K 列路径）

为什么走 Gram 而不是直接 SVD
-------------------------------------------------------------------------------
B 是 (M, N) = (3906, 1044) 量级，直接 `scipy.linalg.svd` 要几秒到几十秒并且只
为了前 R=64 列而算满秩。而 BᵀB 只有 N×N = 1044²，特征分解后取前 R 个特征对，
再用 `U = B V / S` 回代即可 —— 这与仓库既有 `src/hardware/gram.hpp` 的口径一致，
也让 PC 侧的离线建模与板端 Gram 路径用同一套数学。

一个必须小心的数值点：`S = sqrt(λ)` 在 λ 很小或为负（浮点误差）时会出 NaN/0。
这里对 λ 做 0 截断，并且 **S 归一化时除以最大奇异值**（方案 §6.3：
「归一化奇异值 Q2.14，最大奇异值另存 Q16.16 增益」），所以模型文件里还要
记录 `s_max`。少了这一步，S 会溢出 Q2.14。

用法
-------------------------------------------------------------------------------
    python -m penumbrasight_model.build_model --help
    # 冒烟：3 个候选、秩 8，验证整条链路
    python -m penumbrasight_model.build_model --candidates 3 --rank 8 \
        --out ../../tests/vectors/model_smoke.psm
    # 全量（243 或 315 候选，R=64）：耗时按实测单候选时间外推，见 --dry-run
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from typing import Dict, List, Optional, Tuple

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from . import forward_loc as FL          # noqa: E402
from . import psm                        # noqa: E402
from . import quantize as Q              # noqa: E402

REPO = os.path.abspath(os.path.join(_HERE, os.pardir, os.pardir, os.pardir))

# 论文 D11 档的候选网格（与 x4k/x4l 完全一致，便于交叉核对）
PAPER_GRID = dict(lo=(0.33, 0.42, 0.13), hi=(0.62, 0.70, 0.30), n=(9, 7, 5))


def _candidate_grid(spec: Dict) -> List[Tuple[int, int, int]]:
    """先 z 后 y 后 x 的字典序遍历，保证两次运行得到同一个 model.psm。"""
    nx, ny, nz = spec["n"]
    lo, hi = spec["lo"], spec["hi"]
    gx = np.linspace(lo[0], hi[0], nx)
    gy = np.linspace(lo[1], hi[1], ny)
    gz = np.linspace(lo[2], hi[2], nz)
    out = []
    for ix in range(nx):
        for iy in range(ny):
            for iz in range(nz):
                out.append((ix, iy, iz))
    return [(ix, iy, iz, (float(gx[ix]), float(gy[iy]), float(gz[iz])))
            for ix, iy, iz in out]


def apply_row_diff(A: np.ndarray, npx: int, *, axis: int = 0) -> np.ndarray:
    """把 (npx*npx, N) 的 A 变成差分后的 ((npx-1)*npx, N)。

    - `axis=0`（默认）：竖直相邻**行**差分，对应方案 §6.3 的「垂直差分」。
    - `axis=1`：水平差分。X4k 实测指出「垂直对定位更好」不成立（方向优势随
      场景翻转），方案指定垂直是为了与重建链接口统一，故默认 axis=0。

    不物化差分算子：直接按图像布局做切片相减，省掉一个 (3906, 3906) 的稀疏矩阵。
    行序与 `forward_loc.row_to_wall_index` 一致（行 = z、列 = x），所以
    「相邻行差分」就是 i+1 行减 i 行。
    """
    N = A.shape[1]
    A3 = A.reshape(npx, npx, N)                      # [ix, iz, N]
    D = A3[:, 1:, :] - A3[:, :-1, :] if axis == 0 else A3[1:, :, :] - A3[:-1, :, :]
    return np.ascontiguousarray(D).reshape(-1, N)


def truncated_svd_via_gram(B: np.ndarray, rank: int, *, chunk: int = 0
                           ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    """由 B 的 Gram 矩阵取前 `rank` 个奇异三元组。

    返回 `(U, S, V, s_max)`：
        U: (M, rank)  orthonormal columns
        S: (rank,)    归一化到最大值为 1 的奇异值（方案 §6.3）
        V: (rank, N)  右奇异向量，rank-major —— **注意是 (rank, N) 而不是 (N, rank)**
        s_max: 最大奇异值（Q16.16 增益的原始值）

    V 取 (rank, N) 是因为 model.psm 的 V 是 rank-major 布局（psm.py 已冻结），
    直接给成这个形状可以省掉一次转置，也避免写盘时再转错。
    """
    M, N = B.shape
    rank = int(min(rank, N, M))
    G = B.T @ B                                  # (N, N)
    G = 0.5 * (G + G.T)
    # 只需要前 rank 个特征对；用 scipy 的 subset 加速，没有就回退到全谱
    try:
        from scipy.linalg import eigh as _eigh
        lam, Vc = _eigh(G, subset_by_index=[N - rank, N - 1], driver="evr")
    except Exception:
        lam, Vc = np.linalg.eigh(G)
        lam, Vc = lam[-rank:], Vc[:, -rank:]
    order = np.argsort(lam)[::-1]
    lam, Vc = np.maximum(lam[order], 0.0), Vc[:, order]     # λ 截断到 0
    S = np.sqrt(lam)
    s_max = float(S[0]) if S.size else 0.0
    Vc = Vc / np.maximum(np.linalg.norm(Vc, axis=0, keepdims=True), 1e-300)
    if s_max > 0:
        U = (B @ Vc) / S[None, :]
        U = U / np.maximum(np.linalg.norm(U, axis=0, keepdims=True), 1e-300)
    else:
        U = np.zeros((M, rank))
    S_norm = S / s_max if s_max > 0 else S
    return U, S_norm, Vc.T, s_max


def observable_mask_from_B(B: np.ndarray, *, rel_thresh: float = 0.1
                           ) -> Tuple[np.ndarray, Dict[str, float]]:
    """可观测性掩膜（方案 §6.6「像素可信度为预计算可观测性掩膜与全局可信度的乘积」）。

    **必须用差分后的 B，不能用未差分的 A** —— 这条是实测出来的，不是想当然：
    在论文 D11 档（ds=4、63x63 墙、1044 隐藏点）上量列范数的分布，

        A（未差分）列范数 min/max = 0.422   -> 阈值取到 0.3 掩膜仍全是 1
        B（差分后）列范数 min/max = 0.0842  -> 阈值 0.1 时 99.4%、0.3 时 62.8%

    也就是说用 A 的话，任何合理阈值都给出全 1 掩膜，这个机制等于没接线。
    物理上也该用 B：本系统能观测到一个隐藏像素，指的是它能在**差分测量**里
    造成可检出的变化，而不是它「发了光」。

    但也要如实记一笔：即便用 B，在论文这档几何下 0.1 阈值仍给出 99.4% 的 1，
    说明该几何下几乎所有隐藏像素都是可观测的。**所以这个掩膜在论文档上无法被
    有效验证**，它的实际区分度要等 B 档几何（墙 96 而非 63）和真实实验台数据
    才谈得上；在此之前不要拿它当可信度的有效门控来宣称效果。

    返回 (掩膜, 诊断dict)；诊断里带上分位数，便于换几何后复查。
    """
    col = np.linalg.norm(B, axis=0)
    if col.size == 0 or col.max() <= 0:
        return np.ones(B.shape[1], dtype=np.uint8), dict(
            col_min_over_max=1.0, mask_frac=1.0)
    ratio = col / col.max()
    nz = col[col > 0]
    diag = dict(
        col_min_over_max=float(nz.min() / col.max()) if nz.size else 0.0,
        col_median_over_max=float(np.median(ratio)),
        mask_frac=float((col >= rel_thresh * col.max()).mean()),
        nonzero_cols=int(nz.size), total_cols=int(col.size),
        rel_thresh=float(rel_thresh),
    )
    return (col >= rel_thresh * col.max()).astype(np.uint8), diag


def build_one_candidate(A_builder, occ_ll, rank: int, cache: Dict) -> psm.CandidateRecord:
    """建一个候选的 B_p，做截断 SVD 并量化成 psm.CandidateRecord。"""
    npx = cache["npx"]
    A = A_builder(occ_ll)
    B = apply_row_diff(A, npx, axis=cache["diff_axis"])
    U, S_norm, V, s_max = truncated_svd_via_gram(B, rank)

    # U/V 用**幂次 scale**（`quantize_symmetric_pow2`），不是连续自适应 scale。
    # 原因：硬件数据通路的增益是 Q16.16（`ps::scale_to_gain_q16`），分辨率
    # 1.53e-5，而真实的 U/V scale 落在 1e-6~1e-5 —— 连续 scale 会取整到 0，
    # 实测 8 个量里 6 个为 0。改成 2 的幂之后增益精确可表示，相对误差恒为 0。
    # 详见 docs/gain_representation.md 与 test_lr_operator.cpp 的 L7 段。
    fa_u = Q.quantize_symmetric_pow2(U, 16, int_bits=1, label="U Q1.15 2^exp")
    fa_v = Q.quantize_symmetric_pow2(V, 16, int_bits=1, label="V Q1.15 2^exp")
    # S 走 Q2.14 定点，scale 恒为 2^-14 —— 本身已是 2 的幂，无需改。
    fa_s = Q.quantize_singular_values(S_norm, 16)
    mask, mask_diag = observable_mask_from_B(
        B, rel_thresh=cache["mask_rel_thresh"])

    # 补零到 rank_max（SVD 实际给出的秩可能小于请求的 rank）
    def _pad2d(x: np.ndarray, rows: int, cols: int) -> np.ndarray:
        out = np.zeros((rows, cols), dtype=x.dtype)
        r = min(rows, x.shape[0])
        c = min(cols, x.shape[1])
        out[:r, :c] = x[:r, :c]
        return out

    rm = cache["rank_max"]

    # ---- U 落盘布局自检：专防「少一次转置」这类静默错误 ----
    # 依据：U 是正交列，做成 rank-major 落盘后 `codes·scale` 的 Gram 矩阵应≈单位阵。
    # 若只拷进左上角一块，前面若干行范数会远小于 1，对角明显偏小，这里当场拦下。
    # 为什么必须专门查这一项：形状对、CRC 对、u_lsb_error 也对（它在转置之前算），
    # 三个既有校验**都**拦不住这个错，它会一路混到重建出图才暴露。
    _u_rm = np.ascontiguousarray(fa_u.codes.T)            # (rank, M) —— 落盘布局
    _g = _u_rm.astype(np.float64) * float(fa_u.scale)
    _gram = _g @ _g.T
    _diag_dev = float(np.max(np.abs(np.diag(_gram) - 1.0)))
    _off_dev = float(np.max(np.abs(_gram - np.diag(np.diag(_gram)))))
    if _diag_dev > 0.05 or _off_dev > 0.05:
        raise RuntimeError(
            "U 落盘布局自检失败：对角偏差 %.4f、非对角偏差 %.4f（阈值 0.05）。"
            "最可能的原因是 U 少了一次转置 —— truncated_svd_via_gram 返回的是 "
            "(M, rank)，而 psm 要求 rank-major 的 (rank, M)。"
            % (_diag_dev, _off_dev))

    return psm.CandidateRecord(
        x_mm=int(round(occ_ll[0] * 1000.0)),
        wall_distance_mm=int(round((cache["D"] - occ_ll[1]) * 1000.0)),
        z_mm=int(round(occ_ll[2] * 1000.0)),
        u_scale=float(fa_u.scale), s_scale=float(fa_s.scale),
        v_scale=float(fa_v.scale),
        # ★ s_max 随候选头入 psm（2026-09-29）。S 落盘时已归一化（最大 1），
        #   绝对幅度 σ = S_code · s_scale · s_max，故 s_max 必须与 S 同记录存放，
        #   否则只读 psm 的板端拿到的是 B/s_max。
        s_max=float(s_max),
        # ★ U 必须转置成 rank-major。这里踩过一次：`truncated_svd_via_gram` 返回的
        #   U 是 (M, rank)（measurement-major），直接 _pad2d 到 (rank, M) 只会拷进
        #   左上角 rank×rank 那一小块、其余全零，而形状与 CRC 校验照样通过。
        #   V **不需要**转置：该函数已经返回 rank-major 的 (rank, N)。
        U=_pad2d(np.ascontiguousarray(fa_u.codes.T).astype(np.int16),
                 rm, cache["meas_len"]),
        S=_pad2d(fa_s.codes.astype(np.uint16)[:, None], rm, 1).ravel(),
        V=_pad2d(fa_v.codes.astype(np.int16), rm, cache["hidden_len"]),
        observable_mask=mask.astype(np.uint8),
    ), dict(s_max=s_max,
            u_lsb_error=fa_u.max_abs_lsb_error(U),
            v_lsb_error=fa_v.max_abs_lsb_error(V),
            s_lsb_error=fa_s.max_abs_lsb_error(S_norm),
            # 增益必须能转成指数（硬件的 `ps::scale_to_exp2`）。这三个数
            # 决定定点层能不能用：
            #   · scale 必须是 2 的幂（非 2 的幂 -> scale_to_exp2 返回 invalid）
            #   · exp 必须装进 int8（[-128,127]）
            # 真实 B 档实测 exp ∈ [-18,-16]，余量 111 个 2 的幂。
            # `pow2_ok` 为 False 时**不要**把模型拿去上板。
            u_exp=Q.scale_to_exponent(fa_u.scale),
            v_exp=Q.scale_to_exponent(fa_v.scale),
            s_exp=Q.scale_to_exponent(fa_s.scale),
            u_scale=float(fa_u.scale),
            v_scale=float(fa_v.scale),
            s_scale=float(fa_s.scale),
            exp_int8_ok=all(Q.gain_exponent_range_ok(e, width=8) for e in (
                Q.scale_to_exponent(fa_u.scale),
                Q.scale_to_exponent(fa_v.scale),
                Q.scale_to_exponent(fa_s.scale))),
            **mask_diag)


def _verify_u_layout(path: str) -> Optional[bool]:
    """收尾门禁：跑 build/scripts/verify_psm_u_layout.py 校验 U 落盘布局。

    为什么要单独查这一项：形状、payload CRC32、`u_lsb_error` 三条既有校验
    **全都拦不住**「U 少一次转置」——形状仍是 (rank_max, M)、CRC 算的是写出去的
    那份自洽字节、u_lsb_error 在转置之前算。只有专查「U 每列是否填满 + Gram≈单位阵」
    才拦得住。这里直接复用 build/scripts 下的那个工具（同一实现，避免两份判据走偏）。

    返回 True/False；工具脚本不存在时返回 None（调用方据此只提示、不判失败）。
    """
    import importlib.util
    tool = os.path.join(REPO, "build", "scripts", "verify_psm_u_layout.py")
    if not os.path.isfile(tool):
        return None
    spec = importlib.util.spec_from_file_location("verify_psm_u_layout", tool)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)                      # type: ignore[union-attr]
    from pathlib import Path as _Path
    return bool(mod.check(_Path(path)))


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="model.psm 生成器")
    ap.add_argument("--geometry", default="paper", choices=("paper", "b"),
                    help="paper = 论文 D11 档（已验证）；b = B 档交付档")
    ap.add_argument("--downsample", type=int, default=None,
                    help="2 的幂级额外缩减；默认论文档 4（得 63x63）、B 档 0（得 96x96）")
    ap.add_argument("--rank", type=int, default=64)
    ap.add_argument("--rank-max", type=int, default=64)
    ap.add_argument("--loc-rank", type=int, default=8)
    ap.add_argument("--nmon", type=int, default=2)
    ap.add_argument("--candidates", type=int, default=0,
                    help="只建前 N 个候选（0 = 全部）；冒烟测试用 3")
    ap.add_argument("--diff-axis", type=int, default=0, choices=(0, 1))
    ap.add_argument("--mask-rel-thresh", type=float, default=0.1)
    ap.add_argument("--mz-mirror", action=argparse.BooleanOptionalAction,
                    default=False,
                    help="视角因子 z 配对；X4l 实测 --no-mz-mirror 拟合更好，故默认关")
    ap.add_argument("--out", default="")
    ap.add_argument("--manifest", default="")
    ap.add_argument("--dry-run", action="store_true", help="只报网格规模与预估耗时")
    args = ap.parse_args(argv)

    if args.geometry == "b":
        st = FL.b_loc_setup()
        grid_spec, n_expected = FL.b_candidate_grid()
        # B 档的 96x96 是「分块后」的边长，ds=0 即得 96；
        # 论文档才是 ds=4（63x63）。见 FL.b_loc_setup 的文首说明。
        ds = 0 if args.downsample is None else args.downsample
        geom_yaml_name = "geometry_b.yaml"
        out_default = os.path.join(REPO, "configs", "model_b.psm")
    else:
        st = FL.d11_loc_setup()
        grid_spec, n_expected = PAPER_GRID, 315
        ds = 4 if args.downsample is None else args.downsample
        geom_yaml_name = "geometry_paper.yaml"
        out_default = os.path.join(REPO, "sim", "results", "model_paper.psm")

    npx = FL.num_pixels_for(st, ds)
    grid = _candidate_grid(grid_spec)
    n_total = len(grid)
    if n_expected != n_total:                      # 网格定义与预期不符，早报错
        print("!! 候选网格声明 %d 个，实际展开 %d 个 —— 检查 b_candidate_grid/PAPER_GRID"
              % (n_expected, n_total))
        return 2
    n_build = n_total if args.candidates <= 0 else min(args.candidates, n_total)

    plan = FL.make_plan(st, args.nmon, downsamp_factor=ds,
                        mz_mirrored=args.mz_mirror)
    meas_len = npx * (npx - 1) if args.diff_axis == 0 else (npx - 1) * npx
    hidden_len = st.num_blocks[0] * st.num_blocks[1]

    print("几何    : %s   ds=%d  墙网格 %dx%d  差分后 M=%d  隐藏 N=%d"
          % (args.geometry, ds, npx, npx, meas_len, hidden_len))
    print("候选    : 网格 %s 共 %d 个，本次建 %d 个"
          % (grid_spec["n"], n_total, n_build))
    print("模型    : rank=%d  rank_max=%d  loc_rank=%d  nmon=%d  差分轴=%s  z配对=%s"
          % (args.rank, args.rank_max, args.loc_rank, args.nmon,
             "垂直" if args.diff_axis == 0 else "水平",
             "mirrored" if args.mz_mirror else "direct"))

    if args.dry_run:
        print("\n--dry-run：不建模型。单候选实测耗时见上一次真实运行的 manifest。")
        print("预计全量耗时 = 单候选耗时 x %d" % n_total)
        return 0

    # 几何哈希：由**当前 geometry 对应的那份 YAML** 的冻结范围算出，
    # 表示「模型是按哪套几何冻出来的」。B 档与论文档共用同一套字段名，
    # 但数值不同 —— 换档必须换哈希，否则 §6.6 的灰屏保护会失效。
    #
    # 这里的文件名与提示串都取**当前档位**（`geom_yaml_name`），不硬编码
    # `geometry_b.yaml`：否则论文档缺 `geometry_paper.yaml` 时提示会印成
    # 「无法从 geometry_b.yaml 算哈希」，把人引到错误的文件上（X28 §五.3 / X29 项 4）。
    geom_yaml = os.path.join(REPO, "configs", geom_yaml_name)
    geom_hash = 0
    if os.path.isfile(geom_yaml):
        try:
            import yaml                      # 可选依赖
            with open(geom_yaml, "r", encoding="utf-8") as f:
                geom_hash = psm.build_geometry_hash(yaml.safe_load(f))
        except Exception as exc:             # pragma: no cover
            print("  !! 无法从 %s 算哈希：%s；几何哈希退化为 0"
                  % (geom_yaml_name, exc))
    else:
        print("  !! 缺 %s，几何哈希退化为 0（该模型将无法通过 §6.6 的灰屏保护）"
              % os.path.relpath(geom_yaml, REPO))

    header = psm.PsmHeader(
        meas_h=(npx - 1) if args.diff_axis == 0 else npx,
        meas_w=npx if args.diff_axis == 0 else (npx - 1),
        hidden_h=st.num_blocks[0], hidden_w=st.num_blocks[1],
        candidates=n_build, rank_max=args.rank_max,
        rank_default=args.rank, loc_rank=args.loc_rank,
        geometry_hash=geom_hash,
    )
    header.validate()

    cache = dict(npx=npx, D=float(st.D), meas_len=meas_len, hidden_len=hidden_len,
                 rank_max=args.rank_max, diff_axis=args.diff_axis,
                 mask_rel_thresh=args.mask_rel_thresh)

    def _builder(ll):
        return FL.build_A_from_plan(plan, ll)

    recs: List[psm.CandidateRecord] = []
    meta: List[Dict] = []
    t0 = time.time()
    for k in range(n_build):
        ix, iy, iz, ll = grid[k]
        rec, info = build_one_candidate(_builder, ll, args.rank, cache)
        recs.append(rec)
        info.update(ix=ix, iy=iy, iz=iz, xyz=ll)
        meta.append(info)
        if (k + 1) % 5 == 0 or k + 1 == n_build:
            dt = time.time() - t0
            print("  %4d/%d  %6.1f s（%.2f s/候选）  最近 s_max=%.4g  掩膜占比=%.3f"
                  % (k + 1, n_build, dt, dt / (k + 1), info["s_max"],
                     info["mask_frac"]))
    per_cand = (time.time() - t0) / max(n_build, 1)

    out = args.out or out_default
    out = os.path.abspath(out)
    summary = psm.write_model(out, header, recs)
    summary["per_candidate_seconds"] = per_cand
    summary["extrapolated_full_seconds"] = per_cand * n_total
    summary["candidates_built"] = n_build
    summary["candidates_total"] = n_total
    summary["rank"] = args.rank
    summary["geom_yaml"] = os.path.relpath(geom_yaml, REPO) if os.path.isfile(geom_yaml) else None
    summary["candidates_meta"] = meta
    # 测量归一化口径（2026-09-29 入库）：SharedExponentFrame 的每帧共享指数是
    # 实拍数据当场算出的运行时量，与静态模型无关，故不进 psm 头；但「口径」要
    # 随模型记录，方便离线复算 d 时用对除数。逐帧指数本身记在 experiment
    # manifest（experiments/manifests/schema.json 的 measurement_normalization）。
    summary["measurement_normalization"] = Q.measurement_normalization_convention()
    summary.pop("verify", None)

    man = args.manifest or os.path.splitext(out)[0] + ".manifest.json"
    with open(man, "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)

    print("\n写出    : %s（%.2f MB）" % (out, summary["bytes"] / 1e6))
    v = psm.verify_model(out, expect_hash=header.geometry_hash)
    print("校验    : CRC32 %08X  payload %d B  几何哈希 %08X  -> %s"
          % (v["crc32"], v["payload_bytes"], v["geometry_hash"],
             "通过" if v["ok_crc32"] and v["ok_payload_bytes"]
             and v["ok_geometry_hash"] else "失败"))
    # 只读前 K 列路径（板端定位真正走的读法）
    U_k = psm.read_loc_ranks(out, min(1, n_build - 1), min(args.loc_rank, args.rank))
    print("只读前K : shape=%s  首元素=%.6g（这条路径不通，板端定位就读不了模型）"
          % (U_k.shape, U_k[0, 0]))

    # ---- 收尾门禁：U 落盘布局 ----
    # 生成即校验，退出码非零即失败，避免「写出去的包布局不合法」悄悄入库。
    layout_ok = _verify_u_layout(out)
    if layout_ok is None:
        print("U布局   : 跳过（未找到 build/scripts/verify_psm_u_layout.py）")
    else:
        print("U布局   : %s（rank-major + 每列非零 + Gram≈单位阵）"
              % ("OK" if layout_ok else "FAIL"))
        if not layout_ok:
            print("!! U 落盘布局不合法，本包不可交付")
            return 3

    print("耗时    : %.2f s/候选；全部 %d 个候选预计 %.1f min"
          % (per_cand, n_total, per_cand * n_total / 60.0))
    print("manifest: %s" % man)
    return 0


if __name__ == "__main__":
    sys.exit(main())

