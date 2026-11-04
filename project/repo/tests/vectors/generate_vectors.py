#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tests/vectors/generate_vectors.py —— 本目录两个 .psm fixture 的唯一生成入口。

本目录（`tests/vectors/`）只放**板端一致性测试用的小型 fixture**，且是仓库中
唯一允许入库 `.psm` 的地方（根 `.gitignore` 里 `*.psm` 一律忽略，只对本目录开例外）。
这里的两份 fixture 都由 `penumbrasight_model.build_model` 生成，参数固定在下方 `SPECS`：

    model_b_r8_c3.psm      --geometry b     --candidates 3 --rank 8 --rank-max 8 --loc-rank 8
    model_paper_r8_c3.psm  --geometry paper --candidates 3 --rank 8 --rank-max 8 --loc-rank 8

为什么单独写一个入口
-------------------------------------------------------------------------
1) 收敛：此前生成 fixture 靠散落的命令行（记在 `tests/vectors/readme.md` 与
   `sim/results/x29_config_consistency.md`），换个 shell 就容易漏参数。这里把参数
   固化成数据，一条命令复现两个包。
2) 逐字节可复现：`build_model.py` 的候选网格、SVD 与量化都是确定性的，同一台机器上
   重跑必须得到**逐字节相同**的 `.psm`。本脚本生成后直接比 SHA256，不一致就非零退出。
3) 与 U 布局门禁绑定：生成后会调 `build/scripts/verify_psm_u_layout.py` 的同一实现，
   把「U 少一次转置」这类形状/CRC 都拦不住的错误当场暴露。

它**不做**什么
-------------------------------------------------------------------------
本目录只有 `.psm` fixture；`sim/vectors/` 下的算法向量（`v1..v4.npz` 等）由
`src/model/scripts/make_vectors.py` 生成，依赖仓库外的官方数据集，不在本脚本职责内。

用法
-------------------------------------------------------------------------
    # 重新生成两个 fixture（会覆盖 tests/vectors/ 下的 .psm 与 .manifest.json）
    python tests/vectors/generate_vectors.py

    # 只校验现有 fixture 是否逐字节等于预期（不生成、不写盘）
    python tests/vectors/generate_vectors.py --check

    # 生成后接受新的 SHA256（生成器/依赖升级导致字节变化时用），只打印不比对
    python tests/vectors/generate_vectors.py --update

退出码：全部通过 0，任一失败 1。

注意：`.manifest.json` 里含 `per_candidate_seconds` / `extrapolated_full_seconds`
两个**计时字段**，因此 manifest 不参与逐字节比对；逐字节承诺只对 `.psm` 成立。
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import os
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_REPO = _HERE.parent.parent
_SRC_MODEL = _REPO / "src" / "model"
if str(_SRC_MODEL) not in sys.path:
    sys.path.insert(0, str(_SRC_MODEL))

from penumbrasight_model import build_model as BM      # noqa: E402
from penumbrasight_model import psm                    # noqa: E402

# 生成参数（与 tests/vectors/readme.md 记录的原始命令一致）
SPECS = [
    dict(name="model_b_r8_c3", geometry="b",
         candidates=3, rank=8, rank_max=8, loc_rank=8),
    dict(name="model_paper_r8_c3", geometry="paper",
         candidates=3, rank=8, rank_max=8, loc_rank=8),
]

# 逐字节基线（2026-09-29 候选头 20->24 字节（加 s_max）后重生成实测；--update 可刷新）
EXPECTED_SHA256 = {
    "model_b_r8_c3":
        "5147872896767B2394C521835D1CC24A169154492971A2126818174155E4A382",
    "model_paper_r8_c3":
        "1C7119BD24D1726925198D097560BE217F8173D0E498D1D8519F9252FE59156C",
}


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest().upper()


def _load_layout_tool():
    """复用 build/scripts/verify_psm_u_layout.py（同一判据，不另写一份）。"""
    tool = _REPO / "build" / "scripts" / "verify_psm_u_layout.py"
    if not tool.is_file():
        return None
    spec = importlib.util.spec_from_file_location("verify_psm_u_layout", tool)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)                       # type: ignore[union-attr]
    return mod


def _cmd_for(spec: dict, out: Path, man: Path) -> list:
    return [
        "--geometry", spec["geometry"],
        "--candidates", str(spec["candidates"]),
        "--rank", str(spec["rank"]),
        "--rank-max", str(spec["rank_max"]),
        "--loc-rank", str(spec["loc_rank"]),
        "--out", str(out),
        "--manifest", str(man),
    ]


def main() -> int:
    ap = argparse.ArgumentParser(description="tests/vectors 的 .psm fixture 生成器")
    ap.add_argument("--check", action="store_true",
                    help="只校验现有 fixture，不生成、不写盘")
    ap.add_argument("--update", action="store_true",
                    help="生成后只打印 SHA256，不与基线比对")
    args = ap.parse_args()

    tool = _load_layout_tool()
    if tool is None:
        print("!! 找不到 build/scripts/verify_psm_u_layout.py，U 布局门禁不可用")
        return 1

    all_ok = True
    for spec in SPECS:
        name = spec["name"]
        out = _HERE / f"{name}.psm"
        man = _HERE / f"{name}.manifest.json"
        print("=" * 74)
        print(f"[{name}]  geometry={spec['geometry']}  rank={spec['rank']}  "
              f"candidates={spec['candidates']}")

        if not args.check:
            rc = BM.main(_cmd_for(spec, out, man))
            if rc != 0:
                print(f"  [FAIL] 生成返回非零退出码 {rc}")
                all_ok = False
                continue

        if not out.is_file():
            print(f"  [FAIL] 缺文件 {out}")
            all_ok = False
            continue

        # 1) payload CRC / 几何哈希（psm 自带回读校验）
        v = psm.verify_model(str(out))
        print(f"  verify_model : CRC32 {v['crc32']:08X}  payload {v['payload_bytes']} B  "
              f"几何哈希 {v['geometry_hash']:08X}  -> OK")

        # 2) U 落盘布局（本脚本存在的直接理由）
        layout_ok = bool(tool.check(out))
        if not layout_ok:
            print("  [FAIL] U 布局校验失败")
            all_ok = False

        # 3) 逐字节比对
        digest = _sha256(out)
        exp = EXPECTED_SHA256.get(name)
        if args.update or exp is None:
            print(f"  SHA256       : {digest}（未比对）")
        elif exp == digest:
            print(f"  SHA256       : {digest}  -> 与基线逐字节一致")
        else:
            print(f"  [FAIL] SHA256 : {digest}")
            print(f"         基线    : {exp}")
            all_ok = False

        if not layout_ok:
            all_ok = False

    print("=" * 74)
    print("总结：" + ("全部通过" if all_ok else "存在失败项，见上"))
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
