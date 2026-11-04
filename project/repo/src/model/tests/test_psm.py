# -*- coding: utf-8 -*-
"""model.psm 读写与格式契约单元测试（对应方案 §12 Task 3）。

覆盖 `penumbrasight_model.psm`：头部结构体、单候选记录的 4096 字节对齐、
几何哈希、write/read 往返、只读前 K 列路径、以及 篡改检测。

用**自造的小记录**（rank 4 / M 12 / N 4），不读 `tests/vectors/` 的大 fixture，
因此运行很快；大 fixture 的字节级校验由 `tests/vectors/generate_vectors.py`
与 `build/scripts/verify_psm_u_layout.py` 负责。

可直接运行（不依赖 pytest）：
    python src/model/tests/test_psm.py
"""

from __future__ import annotations

import os
import struct
import sys
import tempfile
import traceback

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC_MODEL = os.path.dirname(_HERE)
if _SRC_MODEL not in sys.path:
    sys.path.insert(0, _SRC_MODEL)

from penumbrasight_model import psm                    # noqa: E402

ALIGN = 4096


def _tiny_header(**kw):
    base = dict(meas_h=4, meas_w=3, hidden_h=2, hidden_w=2, candidates=3,
                rank_max=4, rank_default=2, loc_rank=2, geometry_hash=0xAABBCCDD)
    base.update(kw)
    return psm.PsmHeader(**base)


def _tiny_records(header, n=None, seed=0):
    rng = np.random.default_rng(seed)
    n = header.candidates if n is None else n
    r, m, h = header.rank_max, header.meas_len, header.hidden_len
    out = []
    for i in range(n):
        out.append(psm.CandidateRecord(
            x_mm=10 * i, wall_distance_mm=-20 + 10 * i, z_mm=20 * i,
            u_scale=1.0 / 4096, s_scale=1.0 / 16384, v_scale=1.0 / 4096,
            U=rng.integers(-32768, 32767, size=(r, m), dtype=np.int16),
            S=rng.integers(0, 65535, size=(r,), dtype=np.uint16),
            V=rng.integers(-32768, 32767, size=(r, h), dtype=np.int16),
            observable_mask=np.array([1, 1, 0, 1], dtype=np.uint8),
            s_max=40.0 + 5.0 * i))
    return out


# ---------------------------------------------------------------- 结构与常量


def test_header_and_candidate_header_sizes():
    assert psm.HEADER_BYTES == 64
    assert psm._HEADER_STRUCT.size == 64
    assert psm._CAND_HDR_STRUCT.size == 24   # x/y/z/reserved(4×int16) + 4×float32（含 s_max）


def test_header_pack_unpack_roundtrip():
    h = _tiny_header()
    h2 = psm.PsmHeader.unpack(h.pack())
    for f in ("meas_h", "meas_w", "hidden_h", "hidden_w", "candidates",
              "rank_max", "rank_default", "loc_rank", "geometry_hash"):
        assert getattr(h, f) == getattr(h2, f), f
    assert h2.payload_bytes == 0 and h2.payload_crc32 == 0


def test_header_unpack_rejects_bad_magic_and_version():
    h = _tiny_header()
    raw = bytearray(h.pack())
    raw[0:4] = b"XXXX"
    try:
        psm.PsmHeader.unpack(bytes(raw))
    except ValueError:
        pass
    else:
        raise AssertionError("错误魔数未报错")
    raw = bytearray(h.pack())
    struct.pack_into("<H", raw, 4, 999)          # version
    try:
        psm.PsmHeader.unpack(bytes(raw))
    except ValueError:
        return
    raise AssertionError("不支持的版本未报错")


def test_header_validate_rank_ordering():
    # 合法约束是 0 < loc_rank <= rank_default <= rank_max。
    # 这里原先写成 loc_rank=3 > rank_default=2 却注释「合法」：validate() 会照章抛
    # ValueError，这一行自己就把测试打挂了；而下面 bad 列表里又拿**同一组值**当
    # 「非法样例」，前后自相矛盾。改成顺序正确的三元组。
    _tiny_header(loc_rank=2, rank_default=3, rank_max=4).validate()  # 合法
    for bad in (dict(loc_rank=0, rank_default=2, rank_max=4),
                dict(loc_rank=3, rank_default=2, rank_max=4),
                dict(loc_rank=2, rank_default=5, rank_max=4)):
        try:
            psm.PsmHeader(**{**dict(meas_h=4, meas_w=3, hidden_h=2, hidden_w=2,
                                    candidates=1, geometry_hash=0), **bad}).validate()
        except ValueError:
            continue
        raise AssertionError(f"非法秩关系未被拦下：{bad}")


def test_record_bytes_and_offsets_are_4096_aligned():
    """核对 psm.py 文首记的 R=64 / M=9120 / N=432 尺寸表。"""
    h = psm.PsmHeader(meas_h=95, meas_w=96, hidden_h=18, hidden_w=24,
                      candidates=1, rank_max=64, rank_default=64,
                      loc_rank=8, geometry_hash=0)
    off = h.array_offsets()
    assert h.meas_len == 9120 and h.hidden_len == 432
    assert off["U"] == ALIGN
    for name, o in off.items():
        assert o % ALIGN == 0, (name, o)
    assert h.record_bytes == 1236992
    assert h.record_bytes % ALIGN == 0


def test_parity_lines_shape():
    """与 C 端 `test_psm_format.cpp` 比对用的规范行（前缀比对）。"""
    lines = psm.parity_lines()
    assert lines[0].startswith("b_r64 rank=64 M=9120 N=432"), lines[0]
    assert any(ln.startswith("sizeof_header=64") for ln in lines)
    assert any(ln.startswith("off_magic=0") for ln in lines)


# ---------------------------------------------------------------- 几何哈希


def _geom(ci=0.0):
    return {
        "planes": {"wall": 1.03},
        "wall": {"n_points": 126},
        "hidden": {"rows": 29, "cols": 36},
        "occluder": {"size": [0.075, 0.075],
                     "nominal_center": {"xyz": [ci, 0.594, -0.0025]}},
        "forward_model": {"view_factor_z_pairing": True, "stick": 1},
    }


def test_geometry_hash_deterministic_and_sensitive():
    assert psm.build_geometry_hash(_geom()) == psm.build_geometry_hash(_geom())
    assert psm.build_geometry_hash(_geom()) != psm.build_geometry_hash(_geom(ci=0.01))
    g2 = _geom()
    g2["wall"]["n_points"] = 96
    assert psm.build_geometry_hash(_geom()) != psm.build_geometry_hash(g2)


# ---------------------------------------------------------------- 往返与读取


def test_write_read_roundtrip_bitwise():
    h = _tiny_header()
    recs = _tiny_records(h)
    d = tempfile.mkdtemp(prefix="psm_test_")
    path = os.path.join(d, "tiny.psm")
    try:
        summary = psm.write_model(path, h, recs)
        assert summary["bytes"] == psm.HEADER_BYTES + h.record_bytes * h.candidates
        assert summary["candidates"] == h.candidates
        h2, r2 = psm.read_model(path)
        assert h2.geometry_hash == h.geometry_hash
        assert h2.payload_crc32 == summary["crc32"]
        for a, b in zip(recs, r2):
            assert a.x_mm == b.x_mm and a.z_mm == b.z_mm
            assert a.wall_distance_mm == b.wall_distance_mm
            assert a.s_max == b.s_max, "s_max 往返不一致"
            assert np.array_equal(a.U, b.U), "U 往返不一致"
            assert np.array_equal(a.S, b.S), "S 往返不一致"
            assert np.array_equal(a.V, b.V), "V 往返不一致"
            assert np.array_equal(a.observable_mask, b.observable_mask)
            assert np.allclose(a.S_absolute(), b.S_absolute())
    finally:
        for f in os.listdir(d):
            os.remove(os.path.join(d, f))
        os.rmdir(d)


def test_read_loc_ranks_matches_first_k():
    h = _tiny_header()
    recs = _tiny_records(h)
    d = tempfile.mkdtemp(prefix="psm_test_")
    path = os.path.join(d, "tiny.psm")
    try:
        psm.write_model(path, h, recs)
        U = psm.read_loc_ranks(path, 1, 2)
        assert U.shape == (2, h.meas_len)
        assert np.allclose(U, recs[1].U_float()[:2])
        try:
            psm.read_loc_ranks(path, 0, 99)
        except ValueError:
            return
        raise AssertionError("k 越界未报错")
    finally:
        for f in os.listdir(d):
            os.remove(os.path.join(d, f))
        os.rmdir(d)


def test_verify_model_detects_payload_tamper():
    h = _tiny_header()
    recs = _tiny_records(h)
    d = tempfile.mkdtemp(prefix="psm_test_")
    path = os.path.join(d, "tiny.psm")
    try:
        psm.write_model(path, h, recs)
        psm.verify_model(path)                        # 未篡改时通过
        with open(path, "r+b") as f:
            f.seek(psm.HEADER_BYTES + 5)              # 动一个 payload 字节
            b = f.read(1)
            f.seek(psm.HEADER_BYTES + 5)
            f.write(bytes([b[0] ^ 0xFF]))
        try:
            psm.verify_model(path)
        except ValueError:
            return
        raise AssertionError("payload 被篡改但 CRC 校验未报错")
    finally:
        for f in os.listdir(d):
            os.remove(os.path.join(d, f))
        os.rmdir(d)


def test_candidate_validate_rejects_wrong_shapes():
    h = _tiny_header()
    good = _tiny_records(h, n=1)[0]
    good.validate(h)                                  # 合法
    bad = psm.CandidateRecord(**{**good.__dict__,
                                 "U": np.zeros((h.rank_max, h.meas_len + 1),
                                               dtype=np.int16)})
    try:
        bad.validate(h)
    except ValueError:
        return
    raise AssertionError("U 形状不符未报错")


# ---------------------------------------------------------------- 运行器


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
    print(f"test_psm 通过 {passed} / {len(tests)}")
    if failed:
        print("失败项：")
        for name, exc in failed:
            print(f"  - {name}: {exc}")
    print("=" * 70)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
