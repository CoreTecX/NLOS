# -*- coding: utf-8 -*-
"""重建链与定点量化单元测试（对应方案 §12 Task 3/4）。

覆盖两件互相咬合的事：

  1. **量化与共享指数口径**（`penumbrasight_model.quantize`）：舍入/饱和、
     Q1.15（U/V 系数）、Q2.14（归一化奇异值）、2 的幂 scale 与指数往返、
     每帧共享指数、以及位宽链在 packed / fullscale 两种口径下的自洽性。
     重建链的定点核吃的是这套换算，口径写错在图上只表现为「整体偏暗/出条纹」。
  2. **重建算子的代数性质**（`penumbrasight_model.reconstruction`）：
     grad/div 的伴随关系、L2 球投影、算子范数估计、以及小尺寸 PDHG 的行为
     （非负约束、恒等算子下能回收测量、形状不符被拦下）与质量度量。

全部用合成小数据，不读官方数据集，也不跑大尺寸求解；跑得很快。
可直接运行（不依赖 pytest）：
    python src/model/tests/test_reconstruction.py
"""

from __future__ import annotations

import os
import sys
import traceback

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC_MODEL = os.path.dirname(_HERE)
if _SRC_MODEL not in sys.path:
    sys.path.insert(0, _SRC_MODEL)

from penumbrasight_model import quantize as Q                 # noqa: E402
from penumbrasight_model import reconstruction as R           # noqa: E402


# ---------------------------------------------------------------- 舍入/饱和


def test_round_half_even_and_saturate():
    x = np.array([0.5, 1.5, 2.5, 3.5, -0.5, -1.5, -2.5])
    got = Q.round_half_even(x)
    assert np.array_equal(got, np.array([0.0, 2.0, 2.0, 4.0, -0.0, -2.0, -2.0])), got

    sat = Q.saturate(np.array([1e9, -1e9, 5.0]), -32768, 32767)
    assert np.array_equal(sat, np.array([32767.0, -32768.0, 5.0])), sat


# ---------------------------------------------------------------- Q1.15


def test_quantize_symmetric_q1_15():
    rng = np.random.default_rng(3)
    x = rng.standard_normal(4096) * 0.3
    fa = Q.quantize_symmetric(x, 16)
    assert fa.q_format == "Q1.15", fa.q_format
    assert fa.codes.dtype == np.int16
    assert int(fa.codes.min()) >= -32767 and int(fa.codes.max()) <= 32767
    assert fa.max_abs_lsb_error(x) <= 1.0, fa.max_abs_lsb_error(x)
    # 超出动态范围时饱和（不是回绕）
    big = Q.quantize_symmetric(np.array([1e9, -1e9]), 16, scale=2 ** -15)
    assert np.array_equal(big.codes, np.array([32767, -32767], dtype=np.int16))


# ---------------------------------------------------------------- 2 的幂 scale


def test_quantize_pow2_and_exponent_roundtrip():
    rng = np.random.default_rng(4)
    x = rng.standard_normal(512) * 0.2
    fa = Q.quantize_symmetric_pow2(x, 16)
    e = fa.scale_pow2
    assert e is not None, "2 的幂量化后 scale 必须是 2 的幂：%r" % (fa.scale,)
    assert Q.scale_to_exponent(fa.scale) == e
    assert Q.exponent_to_scale(e) == fa.scale
    assert fa.max_abs_lsb_error(x) <= 1.0

    try:
        Q.scale_to_exponent(1.5)
    except ValueError:
        pass
    else:
        raise AssertionError("非 2 的幂 scale 未被 scale_to_exponent 拒绝")

    assert Q.gain_exponent_range_ok(-17)
    assert not Q.gain_exponent_range_ok(200)


# ---------------------------------------------------------------- Q2.14 / 共享指数


def test_quantize_singular_values_q2_14():
    rng = np.random.default_rng(5)
    s = np.sort(rng.random(64))[::-1]
    s = s / s[0]
    fs = Q.quantize_singular_values(s)
    assert fs.q_format == "Q2.14", fs.q_format
    assert abs(fs.scale - 2 ** -14) < 1e-18, fs.scale
    assert int(fs.codes.min()) >= 0 and int(fs.codes.max()) <= 65535
    assert fs.max_abs_lsb_error(s) <= 1.0


def test_shared_exponent_preserves_channel_ratio():
    rng = np.random.default_rng(6)
    y = np.stack([rng.random(1000) * 4.0 + 0.1,
                  rng.random(1000) * 1.0 + 0.1,
                  rng.random(1000) * 2.0 + 0.1])
    fa, frame = Q.shared_exponent(y)
    assert fa.q_format == "Q2.16", fa.q_format
    m = float(np.max(np.abs(y)))
    assert frame.exponent == int(np.ceil(np.log2(m))) + 1, frame.exponent
    hi = (1 << (frame.mantissa_bits - 1)) - 1
    assert int(fa.codes.min()) >= -hi and int(fa.codes.max()) <= hi
    # 三通道共用指数 -> 通道间相对幅度原样保留
    r_src = float(y[0].mean() / y[2].mean())
    r_q = float(fa.dequantize()[0].mean() / fa.dequantize()[2].mean())
    assert abs(r_src - r_q) / r_src < 1e-4, (r_src, r_q)

    # 全零输入：指数取 0，不产生 -inf
    fz, fz_frame = Q.shared_exponent(np.zeros(8))
    assert fz_frame.exponent == 0 and np.all(fz.codes == 0)


# ---------------------------------------------------------------- 位宽链


def test_fixed_point_chain_packed_vs_fullscale():
    assert Q.FixedPointChain().value_packing == "packed", "默认口径应为 packed"
    assert Q.FixedPointChain.from_yaml({}).value_packing == "packed"

    ch = Q.FixedPointChain(value_packing="packed", block=10, frames=4)
    assert ch.block_sum_bits() == 19, ch.block_sum_bits()   # 12 + ceil(log2(100))
    for frames in (1, 4, 8):
        Q.FixedPointChain(value_packing="packed", block=10,
                          frames=frames).check()             # 不抛异常

    rep = Q.FixedPointChain(value_packing="fullscale", block=10,
                            frames=4).check(raise_on_shortfall=False)
    assert rep["ok_diff"] is False, "fullscale 下 24 bit 差分通路应判为不够宽"
    assert rep["ok"] is False


# ---------------------------------------------------------------- 重建算子


def test_gradient_divergence_adjoint_identity():
    rng = np.random.default_rng(7)
    f = rng.standard_normal((4, 3))
    g = rng.standard_normal((2, 4, 3))
    lhs = float(np.sum(R.gradient(f) * g))
    rhs = float(np.sum(f * R.divergence(g)))
    assert abs(lhs + rhs) < 1e-12, (lhs, rhs)   # <grad f, g> = <f, -div g>


def test_project_l2_ball():
    inside = np.zeros((2, 2, 2))
    inside[0] = 0.1
    out = R.project_l2_ball(inside.copy(), radius=1.0)
    assert np.allclose(out, inside), "球内向量不应被改动"

    q = np.zeros((2, 2, 2))
    q[0, 0, 0] = 3.0
    q[1, 0, 0] = 4.0
    out2 = R.project_l2_ball(q, radius=1.0)
    assert abs(np.hypot(out2[0, 0, 0], out2[1, 0, 0]) - 1.0) < 1e-12

    zero = R.project_l2_ball(np.zeros((2, 2, 2)), radius=1.0)
    assert np.all(zero == 0.0), "零向量投影不应产生 NaN"


def test_operator_norm_sq_positive():
    rng = np.random.default_rng(8)
    A = rng.standard_normal((6, 6))
    L2 = R.operator_norm_sq(A, (2, 3), n_iter=40)
    assert np.isfinite(L2) and L2 > 0.0, L2


def test_tv_pdhg_identity_recovers_and_is_nonneg():
    A = np.eye(4)
    b = np.array([1.0, 2.0, 3.0, 4.0])
    res = R.tv_pdhg(A, b, (2, 2), lam=0.0, n_iter=2000, record_every=200)
    assert res.image.shape == (2, 2)
    f = res.image.reshape(-1)
    assert np.all(f >= 0.0), "非负约束应生效"
    assert np.max(np.abs(f - b)) < 0.05, f
    assert len(res.objective) > 0 and res.objective[-1]["data"] < 0.05

    # 负测量：解被截到 0，其余分量仍接近测量
    res2 = R.tv_pdhg(A, np.array([-1.0, 2.0, 3.0, 4.0]), (2, 2),
                     lam=0.0, n_iter=2000)
    f2 = res2.image.reshape(-1)
    assert f2[0] < 0.05 and np.max(np.abs(f2[1:] - [2.0, 3.0, 4.0])) < 0.05, f2


def test_positive_tv_has_analytic_minimum():
    # This test isolates the TV adjoint sign; lambda=0 cannot detect it.
    A = np.eye(4)
    b = np.array([0., 1., 0., 1.])
    out = R.tv_pdhg(A, b, (2, 2), lam=0.1, n_iter=1000, L2=5.1)
    assert np.max(np.abs(out.image - [[.1, .9], [.1, .9]])) < 1e-8
    assert abs(out.objective[-1]["obj"] - .18) < 1e-10


def test_norm_matches_explicit_gradient_matrix():
    for shape in [(2, 2), (2, 3), (1, 4)]:
        n = int(np.prod(shape))
        G = np.column_stack([R.gradient(np.eye(n)[i].reshape(shape)).ravel()
                             for i in range(n)])
        A = np.eye(n)
        expected = np.linalg.eigvalsh(A.T @ A + G.T @ G)[-1]
        actual = R.operator_norm_sq(A, shape, n_iter=200)
        assert abs(actual - expected) < 1e-8, (actual, expected)


def test_tv_pdhg_rejects_shape_mismatch():
    A = np.eye(4)
    try:
        R.tv_pdhg(A, np.zeros(4), (3, 3), n_iter=5)
    except ValueError:
        return
    raise AssertionError("A 列数与 shape 不符未被拦下")


def test_quality_metrics():
    ref = np.arange(6, dtype=float).reshape(2, 3) + 1.0
    a = 2.0 * ref
    matched, alpha = R.scale_matched(a, ref)
    # alpha = a·ref / a·a；a = 2*ref 时 alpha = 0.5，标度后回到 ref
    assert abs(alpha - 0.5) < 1e-12 and np.allclose(matched, ref)
    assert R.psnr_scale_matched(a, a) == float("inf")
    assert abs(R.pearson(a, 3.0 * ref + 5.0) - 1.0) < 1e-12


# ---------------------------------------------------------------- 运行器


def test_final_objective_without_periodic_recording():
    result = R.tv_pdhg(np.eye(4), np.ones(4), (2, 2), lam=0, n_iter=100, record_every=0)
    assert len(result.objective) == 1
    assert result.as_dict()["final_objective"]["iter"] == 100


def main() -> int:
    tests = [(n, f) for n, f in sorted(globals().items())
             if n.startswith("test_") and callable(f)]
    passed, failed = 0, []
    for name, fn in tests:
        try:
            fn()
            passed += 1
            print(f"  PASS  {name}")
        except Exception as exc:  # noqa: BLE001
            failed.append((name, exc))
            print(f"  FAIL  {name}")
            traceback.print_exc()
    print()
    print("=" * 70)
    print(f"test_reconstruction 通过 {passed} / {len(tests)}")
    if failed:
        print("失败项：")
        for name, exc in failed:
            print(f"  - {name}: {exc}")
    print("=" * 70)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
