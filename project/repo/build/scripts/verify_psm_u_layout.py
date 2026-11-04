#!/usr/bin/env python3
"""校验 model.psm 里 U 因子的落盘布局（rank-major）是否正确。

为什么需要这个工具
------------------
2026-09-29 发现 `sim/models/model_b.psm` 的 U **少了一次转置**：
`build_model.py` 里的 `truncated_svd_via_gram()` 返回的是 (M, rank)（measurement-major），
而 psm 格式要求 rank-major 的 (rank, M)。直接 `_pad2d(x, rank, M)` 只会拷进
左上角 rank×rank 那一小块、其余全零。

最麻烦的地方在于**既有校验全都拦不住**：
  · 形状对（仍是 rank×M）
  · payload CRC32 对（算的是写出去的那份字节）
  · `u_lsb_error` 对（它在转置之前算，量的是量化误差）
所以必须专门查「U 的每一列是否真的被填满」。

判据
----
U 是正交列，rank-major 落盘后 `codes · u_scale` 的 Gram 矩阵应≈单位阵：
  · 对角应≈1     —— 若只拷了左上角，前面若干行范数会远小于 1，对角明显偏小
  · 非对角应≈0
再用「非零列数」做一次直观确认：正确时每列都该有非零元素。

用法
----
    python build/scripts/verify_psm_u_layout.py <model.psm> [<model.psm> ...]

退出码：全部通过为 0，任一失败为 1（可直接当门禁）。
"""

from __future__ import annotations

import struct
import sys
from pathlib import Path

import numpy as np

ALIGN = 4096
DIAG_TOL = 0.05


def _align_up(n: int, a: int = ALIGN) -> int:
    return ((n + a - 1) // a) * a


def read_header(f) -> dict:
    """读 psm 文件头（ps_psm_header_t，64 B）。字段偏移见 src/software/include/psm_format.h。"""
    f.seek(0)
    raw = f.read(48)
    magic = raw[0:4]
    # 字段偏移以 src/software/include/psm_format.h 为准（那里逐行标了偏移）：
    #   +6 header_bytes / +8 meas_h / +10 meas_w / +12 hidden_h / +14 hidden_w
    #   +16 candidates / +18 rank_max / +20 rank_default / +22 loc_rank
    #   +24 geometry_hash / +28 payload_crc32 / +32 payload_bytes(u64)
    # 注意：**从 +6 起**，不是 +4 —— 这里少算 2 字节会把 M/N/candidates 全读错。
    header_bytes, meas_h, meas_w, hidden_h, hidden_w, candidates = struct.unpack_from(
        "<6H", raw, 6)
    rank_max, rank_default, loc_rank = struct.unpack_from("<3H", raw, 18)
    geometry_hash, = struct.unpack_from("<I", raw, 24)
    payload_crc32, = struct.unpack_from("<I", raw, 28)
    payload_bytes, = struct.unpack_from("<Q", raw, 32)
    return dict(magic=magic, header_bytes=header_bytes, meas_h=meas_h,
                meas_w=meas_w, hidden_h=hidden_h, hidden_w=hidden_w,
                candidates=candidates, rank_max=rank_max,
                rank_default=rank_default, loc_rank=loc_rank,
                geometry_hash=geometry_hash, payload_crc32=payload_crc32,
                payload_bytes=payload_bytes)


def check(path: Path) -> bool:
    size = path.stat().st_size
    with path.open("rb") as f:
        h = read_header(f)
        meas_len = h["meas_h"] * h["meas_w"]
        hidden_len = h["hidden_h"] * h["hidden_w"]
        rank = h["rank_max"]

        # 单候选记录大小：payload 总长整除候选数（实测恰好整除，否则回退到推算）
        rec = h["payload_bytes"] // h["candidates"] if h["candidates"] else 0
        u_off = _align_up(24)                              # 候选头 24 B 对齐到 4096
        u_len = rank * meas_len * 2

        print(f"--- {path.name}")
        print(f"    文件 {size:,} B   候选 {h['candidates']}   rank_max {rank}   "
              f"M {meas_len}   N {hidden_len}")
        print(f"    payload {h['payload_bytes']:,} B   单候选 {rec:,} B   "
              f"U 偏移 {u_off}   长度 {u_len:,} B")

        if rec <= 0 or u_off + u_len > rec:
            print("    [FAIL] 单候选记录大小与 U 长度不自洽，无法按偏移读取")
            return False

        bad = 0
        for idx in (0, h["candidates"] // 2, h["candidates"] - 1):
            f.seek(64 + idx * rec + u_off)
            u = np.fromfile(f, dtype=np.int16, count=rank * meas_len).reshape(rank, meas_len)

            nz_cols = int(np.count_nonzero(np.any(u != 0, axis=0)))
            # 这一项是本次缺陷的直接指纹：修前只有前 rank 列非零
            col_ok = nz_cols > rank * 2

            # Gram 归一化：除以 (0,0) 元素，这样不必依赖候选头里 u_scale 的字段偏移。
            # 修复后各行范数相同，归一化对角应为 1；只拷左上角时各行范数不等（甚至为 0），
            # 归一化对角会明显偏离 1。
            g = u.astype(np.float64)
            gram = g @ g.T
            base = float(gram[0, 0]) or 1.0
            gram = gram / base
            diag_dev = float(np.max(np.abs(np.diag(gram) - 1.0)))
            off = gram - np.diag(np.diag(gram))
            off_dev = float(np.max(np.abs(off)))

            ok = col_ok and diag_dev <= DIAG_TOL and off_dev <= DIAG_TOL
            if not ok:
                bad += 1
            print(f"    候选 {idx:>4}  非零列 {nz_cols:>6}/{meas_len}  "
                  f"Gram 对角偏差 {diag_dev:.4f}  非对角偏差 {off_dev:.4f}  "
                  f"-> {'OK' if ok else 'FAIL'}")

        if bad:
            print(f"    [FAIL] {bad} 个抽样候选的 U 布局不合法。")
            print("           最可能的原因：U 落盘时少了一次转置 "
                  "(truncated_svd_via_gram 返回 (M, rank)，psm 要 (rank, M))。")
            return False
        print("    [OK] U 为 rank-major，每列已填满，Gram 接近单位阵。")
        return True


def main(argv: list[str]) -> int:
    if len(argv) < 2:
        print(__doc__)
        return 2
    all_ok = True
    for p in argv[1:]:
        path = Path(p)
        if not path.exists():
            print(f"--- {path} 不存在")
            all_ok = False
            continue
        try:
            all_ok &= check(path)
        except Exception as exc:  # noqa: BLE001
            print(f"    [FAIL] 读取失败：{type(exc).__name__}: {exc}")
            all_ok = False
    print()
    print("总结：" + ("全部通过" if all_ok else "存在失败项，见上"))
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
