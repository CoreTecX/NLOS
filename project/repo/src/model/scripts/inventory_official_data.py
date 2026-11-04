# -*- coding: utf-8 -*-
"""T0-1 官方数据资产盘点与契约提取。

只读地扫描官方数据集目录下的全部 .mat 测量文件与标注文本文件，
提取形状、dtype、数值范围、元数据与校验和，产出：

  data/inventory/data_inventory.csv      每个文件一行
  data/inventory/data_inventory_summary.md  聚合结论（机器生成）
  data/inventory/text_meta.json          标注文本文件原文归档

设计约束：
- 不修改、不移动、不复制原始数据。
- 大体积原始数据不入库；本脚本只产出清单与统计量。
- 单文件逐个处理，内存占用与文件数无关。

用法：
  python inventory_official_data.py --data-root <官方 Data 目录> --out-dir data/inventory
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import os
import re
import sys
import time
from collections import Counter, defaultdict

import numpy as np
import scipy.io as sio

# ---------------------------------------------------------------- 工具函数


def sha256_file(path: str, chunk: int = 1 << 20) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while True:
            b = f.read(chunk)
            if not b:
                break
            h.update(b)
    return h.hexdigest()


def mat_header_info(path: str) -> dict:
    """从文件头 128 字节读取 MATLAB 版本描述（不加载数据）。"""
    with open(path, "rb") as f:
        head = f.read(128)
    desc = head[0:116].decode("latin-1", errors="replace").strip()
    endian = head[126:128]
    return {
        "header_desc": desc,
        "endian": "little" if endian == b"IM" else "big",
    }


def scalarize(v):
    """把 MATLAB 标量/数组压成 Python 标量，便于写入 CSV。"""
    if v is None:
        return None
    arr = np.asarray(v)
    if arr.size == 0:
        return None
    if arr.size == 1:
        return arr.reshape(-1)[0].item()
    return None


def numeric_stats(arr: np.ndarray) -> dict:
    """对图像数组做数值范围与量化特征统计。"""
    a = np.asarray(arr)
    if a.dtype.kind not in "fiu":
        return {}
    f = a.astype(np.float64, copy=False)
    finite = np.isfinite(f)
    n_finite = int(finite.sum())
    n_total = int(f.size)
    out = {
        "min": float(f[finite].min()) if n_finite else None,
        "max": float(f[finite].max()) if n_finite else None,
        "mean": float(f[finite].mean()) if n_finite else None,
        "p999": float(np.percentile(f[finite], 99.9)) if n_finite else None,
        "nan_count": int(n_total - n_finite),
    }
    # 是否全部为整数（判断是否位深化后的原始值）
    if n_finite:
        sample = f[finite]
        if sample.size > 2_000_000:  # 大图抽样，避免耗时
            idx = np.linspace(0, sample.size - 1, 2_000_000).astype(np.int64)
            sample = sample[idx]
        out["integer_ratio"] = float(np.mean(sample == np.round(sample)))
    else:
        out["integer_ratio"] = None
    # 推断位深：若为整数，看最大值需要多少位
    if out["max"] is not None and out["integer_ratio"] == 1.0:
        out["inferred_bits"] = int(np.ceil(np.log2(out["max"] + 1))) if out["max"] > 0 else 0
    else:
        out["inferred_bits"] = None
    # 饱和线索：是否出现量程上沿的重复值
    if out["integer_ratio"] == 1.0 and out["max"] is not None and out["max"] > 0:
        out["frac_at_max"] = float(np.mean(sample == out["max"]))
    else:
        out["frac_at_max"] = None
    return out


# ---------------------------------------------------------------- 主流程


def collect_mat_files(root: str):
    files = []
    for dirpath, _dirnames, filenames in os.walk(root):
        for fn in sorted(filenames):
            if fn.lower().endswith(".mat"):
                files.append(os.path.join(dirpath, fn))
    return sorted(files)


def collect_text_files(root: str):
    files = []
    for dirpath, _dirnames, filenames in os.walk(root):
        for fn in sorted(filenames):
            if fn.lower().endswith(".txt"):
                files.append(os.path.join(dirpath, fn))
    return sorted(files)


def process_mat(path: str, root: str) -> dict:
    rel = os.path.relpath(path, root)
    parts = rel.split(os.sep)
    group = parts[0] if len(parts) > 1 else "(root)"
    subgroup = parts[1] if len(parts) > 2 else ""

    row = {
        "relpath": rel,
        "group": group,
        "subgroup": subgroup,
        "filename": os.path.basename(path),
        "bytes": os.path.getsize(path),
        "mb": round(os.path.getsize(path) / (1 << 20), 3),
        "sha256": "",
        "mat_header": "",
        "endian": "",
        "varnames": "",
        "image_shape": "",
        "image_dtype": "",
        "min": None,
        "max": None,
        "mean": None,
        "p999": None,
        "integer_ratio": None,
        "inferred_bits": None,
        "frac_at_max": None,
        "nan_count": None,
        "pixel_size": None,
        "shutter_speed": None,
        "status": "",
        "error": "",
    }

    try:
        hdr = mat_header_info(path)
        row["mat_header"] = hdr["header_desc"]
        row["endian"] = hdr["endian"]
    except Exception as exc:  # noqa: BLE001
        row["error"] = f"header:{exc}"

    try:
        row["sha256"] = sha256_file(path)
    except Exception as exc:  # noqa: BLE001
        row["error"] += f" sha:{exc}"

    try:
        d = sio.loadmat(path)
        names = [k for k in d.keys() if not k.startswith("__")]
        row["varnames"] = ";".join(sorted(names))

        img_key = None
        for cand in ("image", "img", "I", "data"):
            if cand in d:
                img_key = cand
                break
        if img_key is None and names:
            # 退化为：取尺寸最大的数值数组
            best, best_size = None, -1
            for n in names:
                a = np.asarray(d[n])
                if a.dtype.kind in "fiu" and a.size > best_size:
                    best, best_size = n, a.size
            img_key = best

        if img_key is not None:
            arr = np.asarray(d[img_key])
            row["image_shape"] = "x".join(str(s) for s in arr.shape)
            row["image_dtype"] = str(arr.dtype)
            row.update({k: v for k, v in numeric_stats(arr).items() if k in row})

        for meta in ("pixel_size", "shutter_speed", "exposure", "gain"):
            if meta in d:
                row[meta] = scalarize(d[meta])

        row["status"] = "ok"
    except NotImplementedError:
        row["status"] = "unsupported_format"
        row["error"] += " loadmat:NotImplementedError(疑似 v7.3/HDF5)"
    except Exception as exc:  # noqa: BLE001
        row["status"] = "failed"
        row["error"] += f" loadmat:{exc}"

    return row


def process_text(path: str, root: str) -> dict:
    rel = os.path.relpath(path, root)
    try:
        with io.open(path, "r", encoding="utf-8", errors="replace") as f:
            content = f.read()
    except Exception as exc:  # noqa: BLE001
        content = f"<<read error: {exc}>>"
    return {"relpath": rel, "bytes": os.path.getsize(path), "content": content}


def summarise(rows, out_path, data_root, elapsed):
    """生成机器可读的聚合结论。"""
    ok = [r for r in rows if r["status"] == "ok"]
    by_group = defaultdict(list)
    for r in ok:
        by_group[r["group"]].append(r)

    shapes = Counter(r["image_shape"] for r in ok)
    dtypes = Counter(r["image_dtype"] for r in ok)
    bits = Counter(str(r["inferred_bits"]) for r in ok)
    ints = Counter(str(r["integer_ratio"]) for r in ok)
    hdrs = Counter(r["mat_header"] for r in rows)

    mins = [r["min"] for r in ok if r["min"] is not None]
    maxs = [r["max"] for r in ok if r["max"] is not None]
    p999s = [r["p999"] for r in ok if r["p999"] is not None]

    L = []
    L.append("# 官方数据资产盘点（机器生成汇总）\n")
    L.append(f"- 数据根目录：`{data_root}`")
    L.append(f"- 扫描时间：{time.strftime('%Y-%m-%d %H:%M:%S')}")
    L.append(f"- 耗时：{elapsed:.1f} s")
    L.append(f"- 文件总数：{len(rows)}（成功解析 {len(ok)}，失败/不支持 {len(rows) - len(ok)}）")
    L.append(f"- 总体积：{sum(r['bytes'] for r in rows) / (1 << 30):.3f} GB\n")

    L.append("## 1. MATLAB 格式\n")
    for k, v in hdrs.most_common():
        L.append(f"- `{k}`：{v} 个文件")
    L.append("")

    L.append("## 2. 图像数组形状\n")
    for k, v in shapes.most_common():
        L.append(f"- `{k}`：{v} 个文件")
    L.append("")

    L.append("## 3. 图像 dtype\n")
    for k, v in dtypes.most_common():
        L.append(f"- `{k}`：{v} 个文件")
    L.append("")

    L.append("## 4. 是否全为整数（整数化判定）\n")
    for k, v in ints.most_common():
        L.append(f"- integer_ratio = `{k}`：{v} 个文件")
    L.append("")

    L.append("## 5. 推断位深（仅对整数化数据）\n")
    for k, v in bits.most_common():
        L.append(f"- `{k}` bit：{v} 个文件")
    L.append("")

    if mins:
        L.append("## 6. 数值范围（全体文件）\n")
        L.append(f"- 全局 min：{min(mins):.6g}")
        L.append(f"- 全局 max：{max(maxs):.6g}")
        L.append(f"- 全局 p99.9 最大值：{max(p999s):.6g}")
        L.append(f"- 单文件 max 的最大值：{max(maxs):.6g}")
        L.append("")

    L.append("## 7. 按目录分组的文件数\n")
    for g in sorted(by_group):
        gs = by_group[g]
        shp = Counter(r["image_shape"] for r in gs)
        L.append(f"- `{g}`：{len(gs)} 个文件，形状 {dict(shp)}，"
                 f"总体积 {sum(r['bytes'] for r in gs) / (1 << 20):.1f} MB")
    L.append("")

    L.append("## 8. 元数据字段出现情况\n")
    for field in ("pixel_size", "shutter_speed"):
        have = [(r["relpath"], r[field]) for r in ok if r[field] is not None]
        vals = sorted({v for _, v in have})
        L.append(f"- `{field}`：{len(have)} 个文件携带，取值集合 = {vals}")
    L.append("")

    L.append("## 9. 全部变量名清单（按出现次数）\n")
    vc = Counter()
    for r in ok:
        for n in filter(None, r["varnames"].split(";")):
            vc[n] += 1
    for k, v in vc.most_common():
        L.append(f"- `{k}`：出现于 {v} 个文件")
    L.append("")

    failed = [r for r in rows if r["status"] != "ok"]
    if failed:
        L.append("## 10. 未成功解析的文件\n")
        for r in failed[:50]:
            L.append(f"- `{r['relpath']}` — {r['status']} — {r['error']}")
        if len(failed) > 50:
            L.append(f"- …其余 {len(failed) - 50} 个同类文件省略")
        L.append("")

    with io.open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(L))
    return "\n".join(L)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-root", required=True)
    ap.add_argument("--out-dir", default="data/inventory")
    ap.add_argument("--limit", type=int, default=0, help="仅处理前 N 个 .mat（调试用）")
    ap.add_argument("--skip-sha", action="store_true", help="跳过 SHA256（加速）")
    args = ap.parse_args()

    root = args.data_root
    if not os.path.isdir(root):
        print(f"[FATAL] 数据目录不存在：{root}")
        sys.exit(1)
    os.makedirs(args.out_dir, exist_ok=True)

    t0 = time.time()
    mats = collect_mat_files(root)
    txts = collect_text_files(root)
    if args.limit:
        mats = mats[: args.limit]
    print(f"[info] 发现 .mat {len(mats)} 个，.txt {len(txts)} 个")

    rows = []
    for i, p in enumerate(mats, 1):
        row = process_mat(p, root)
        if args.skip_sha:
            row["sha256"] = ""
        rows.append(row)
        flag = "ok " if row["status"] == "ok" else "ERR"
        print(f"[{i:>3}/{len(mats)}] {flag} {row['relpath']} "
              f"shape={row['image_shape']} dtype={row['image_dtype']} "
              f"max={row['max']} bits={row['inferred_bits']}")

    # CSV
    csv_path = os.path.join(args.out_dir, "data_inventory.csv")
    fields = list(rows[0].keys()) if rows else []

    def _clean(v):
        """把字段统一成可安全写入 CSV 的字符串，去掉换行与控制字符。"""
        if v is None:
            return ""
        s = v if isinstance(v, str) else repr(v) if isinstance(v, float) and not np.isfinite(v) else str(v)
        return re.sub(r"[\r\n\t\x00-\x1f]", " ", s)

    with io.open(csv_path, "w", encoding="utf-8-sig", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields, escapechar="\\",
                           quoting=csv.QUOTE_ALL, lineterminator="\r\n")
        w.writeheader()
        for r in rows:
            w.writerow({k: _clean(v) for k, v in r.items()})
    print(f"[ok] 写出 {csv_path}")

    # 文本标注归档
    txt_meta = [process_text(p, root) for p in txts]
    json_path = os.path.join(args.out_dir, "text_meta.json")
    with io.open(json_path, "w", encoding="utf-8") as f:
        json.dump(txt_meta, f, ensure_ascii=False, indent=2)
    print(f"[ok] 写出 {json_path}（{len(txt_meta)} 个文本文件）")

    # 汇总
    elapsed = time.time() - t0
    md_path = os.path.join(args.out_dir, "data_inventory_summary.md")
    summarise(rows, md_path, root, elapsed)
    print(f"[ok] 写出 {md_path}")
    print(f"[done] 耗时 {elapsed:.1f} s")


if __name__ == "__main__":
    main()
