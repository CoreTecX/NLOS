# -*- coding: utf-8 -*-
"""model.psm —— 低秩模型包读写（方案 §8.1）。

这是 PC 侧（`build_model.py`）与板端（`software/model_loader`、`hls/lr_operator`、
`hls/projection_engine`）之间**唯一**的数据接口。格式一旦落盘就被 FPGA 读取，
因此本文件的每一个偏移都是契约，不允许「差不多」。

格式（全部小端）
----------------
头部固定 64 字节：

    struct PsmHeader {             偏移   类型
        char     magic[4];          0     "PSM1"
        uint16_t version;           4     1
        uint16_t header_bytes;      6     64
        uint16_t meas_h;            8     95    差分后测量行数
        uint16_t meas_w;           10     96    差分后测量列数
        uint16_t hidden_h;         12     18    隐藏图像行数（z）
        uint16_t hidden_w;         14     24    隐藏图像列数（x）
        uint16_t candidates;       16     243
        uint16_t rank_max;         18     64（R=96 模型写 96）
        uint16_t rank_default;     20     64
        uint16_t loc_rank;         22     8     K
        uint32_t geometry_hash;    24
        uint32_t payload_crc32;    28
        uint64_t payload_bytes;    32
        uint8_t  reserved[24];     40    全 0
    };

每个候选一条记录，记录内**四个数组各自按 4096 字节对齐**，记录本身也按
4096 对齐，因此任一条记录的任一数组的绝对偏移都是 4096 的整数倍：

    记录内偏移      内容                     长度
    0               候选头（24 字节）         x/y/z/reserved:int16 + 3×float32 scale + float32 s_max
    4096            U  int16[R*M]             rank-major：rank 最外层
    ...             S  uint16[R]              **归一化**（最大 1）Q2.14
    ...             V  int16[R*N]             rank-major
    ...             observable_mask uint8[N]
    ...             补齐到 4096 整数倍

S 的归一化与 s_max（2026-09-29 起入 psm）
----------------------------------------
方案 §6.3 要求「归一化奇异值 Q2.14，最大奇异值另存 Q16.16 增益」。落盘时 S 除
以其最大值，故码字域里 max(S) = 1；绝对幅度靠候选头 +20 的 `s_max` 还原：

    S_abs[k] = S_code[k] * s_scale * s_max

此前 `s_max` 只在 manifest.json，只读 psm 的板端拿到的是 B/s_max，重建必须由
调用方从旁边补系数 —— 那是接口缺口（见 x28b 报告 §7）。现在每个候选记录自带宽
度。候选头由 20 变 24 字节**不改变数组偏移**：U 起点仍是 align_up(24, 4096)
== 4096，记录长度与 payload 长度都不变，只有 payload 字节内容（进而 CRC32）变。

U 与 V 都是 **rank-major**（rank 为最外层、测量/图像索引连续），这样定位只需
顺序读取前 `loc_rank` 个 rank，不必跳读——方案 §5.3 的「定位扫描只读前 K=8 列，
单次读取约 35.5 MB 模型数据」就建立在这条布局上。

尺寸核对（R=64, M=9120, N=432）
-------------------------------
    候选头      20            -> 4096
    U           1,167,360     (285 × 4096，正好整块)
    S           128           -> 4096
    V           55,296        -> 57,344
    mask        432           -> 4096
    ------------------------------------
    单候选      1,236,992 B   = 302 × 4096
    243 候选    300,589,056 B ≈ 286.7 MiB

与方案 §5.2 的「243 个候选总计约 297 MB」一致（差在 MiB/MB 换算）。

一条值得记住的约定：**rank 是最外层**。若哪天为了别的目的改成 measurement-major，
定位的「只读前 K 列」立刻退化成 9120 次跨步读取，§5.3 的 35.5 MB 假设作废。
"""

from __future__ import annotations

import dataclasses
import os
import struct
import zlib
from typing import BinaryIO, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

# ------------------------------------------------------------------ 常量

MAGIC = b"PSM1"
VERSION = 1
HEADER_BYTES = 64
ARRAY_ALIGN = 4096

#: '<4s' + 10 个 H + 'II' + 'Q' + '24s'
#: 4 + 20 + 8 + 8 + 24 = 64。struct 默认不加对齐填充，故与 C 端
#: `#pragma pack(1)` 的结构体逐字节对应（见 software/include/psm_format.h）。
_HEADER_STRUCT = struct.Struct("<4sHHHHHHHHHHIIQ24s")
assert _HEADER_STRUCT.size == HEADER_BYTES, _HEADER_STRUCT.size

#: 候选头：x_mm, wall_distance_mm, z_mm, reserved (4 × int16) + 4 × float32
#: （u_scale / s_scale / v_scale / s_max）。4 个 int16 + 4 个 float32 = 24 字节。
#: 与 psm_format.h 的 ps_psm_candidate_hdr_t 逐字节对应。
_CAND_HDR_STRUCT = struct.Struct("<hhhhffff")
assert _CAND_HDR_STRUCT.size == 24, _CAND_HDR_STRUCT.size


def _align_up(n: int, a: int = ARRAY_ALIGN) -> int:
    return ((n + a - 1) // a) * a


# ------------------------------------------------------------------ 头部

@dataclasses.dataclass(frozen=True)
class PsmHeader:
    """model.psm 头部。字段名与 §8.1 的 C 结构体一一对应。"""

    meas_h: int
    meas_w: int
    hidden_h: int
    hidden_w: int
    candidates: int
    rank_max: int
    rank_default: int
    loc_rank: int
    geometry_hash: int
    payload_crc32: int = 0
    payload_bytes: int = 0
    version: int = VERSION
    header_bytes: int = HEADER_BYTES

    # -------------------------------------------------- 派生量

    @property
    def meas_len(self) -> int:
        """M：差分后测量总数。"""
        return self.meas_h * self.meas_w

    @property
    def hidden_len(self) -> int:
        """N：隐藏图像像素总数。"""
        return self.hidden_h * self.hidden_w

    @property
    def record_bytes(self) -> int:
        """单条候选记录的总字节数（含内部对齐与尾部补齐）。"""
        r = self.rank_max
        off = _CAND_HDR_STRUCT.size
        off = _align_up(off) + r * self.meas_len * 2      # U
        off = _align_up(off) + r * 2                      # S
        off = _align_up(off) + r * self.hidden_len * 2    # V
        off = _align_up(off) + self.hidden_len            # mask
        return _align_up(off)

    def array_offsets(self) -> Dict[str, int]:
        """记录内四个数组的相对偏移。板端 model_loader 用它拼 DMA 描述符。"""
        off = _align_up(_CAND_HDR_STRUCT.size)
        out = {"U": off}
        off = _align_up(off + self.rank_max * self.meas_len * 2)
        out["S"] = off
        off = _align_up(off + self.rank_max * 2)
        out["V"] = off
        off = _align_up(off + self.rank_max * self.hidden_len * 2)
        out["observable_mask"] = off
        return out

    # -------------------------------------------------- 序列化

    def pack(self, *, crc32: Optional[int] = None,
             payload_bytes: Optional[int] = None) -> bytes:
        return _HEADER_STRUCT.pack(
            MAGIC, self.version, self.header_bytes,
            self.meas_h, self.meas_w, self.hidden_h, self.hidden_w,
            self.candidates, self.rank_max, self.rank_default, self.loc_rank,
            self.geometry_hash,
            self.payload_crc32 if crc32 is None else crc32,
            self.payload_bytes if payload_bytes is None else payload_bytes,
            b"\x00" * 24,
        )

    @classmethod
    def unpack(cls, buf: bytes) -> "PsmHeader":
        if len(buf) < HEADER_BYTES:
            raise ValueError("头部不足 %d 字节，得到 %d" % (HEADER_BYTES, len(buf)))
        (magic, ver, hb, mh, mw, hh, hw, cand, rmax, rdef, k, gh,
         crc, pbytes, _res) = _HEADER_STRUCT.unpack(buf[:HEADER_BYTES])
        if magic != MAGIC:
            raise ValueError("魔数不是 %r，得到 %r" % (MAGIC, magic))
        if ver != VERSION:
            raise ValueError("版本不支持：%d" % ver)
        if hb != HEADER_BYTES:
            raise ValueError("header_bytes 不是 %d，得到 %d" % (HEADER_BYTES, hb))
        return cls(meas_h=mh, meas_w=mw, hidden_h=hh, hidden_w=hw,
                   candidates=cand, rank_max=rmax, rank_default=rdef, loc_rank=k,
                   geometry_hash=gh, payload_crc32=crc, payload_bytes=pbytes,
                   version=ver, header_bytes=hb)

    def validate(self, *, expect: Optional[Dict[str, int]] = None) -> None:
        """把「形状自洽」做成可执行检查。expect 里给了期望值就顺便比对。"""
        if self.meas_h <= 0 or self.meas_w <= 0:
            raise ValueError("测量尺寸非法")
        if self.hidden_h <= 0 or self.hidden_w <= 0:
            raise ValueError("隐藏图像尺寸非法")
        if self.candidates <= 0:
            raise ValueError("候选数为 0")
        if not (0 < self.loc_rank <= self.rank_default <= self.rank_max):
            raise ValueError(
                "秩必须满足 0 < loc_rank(%d) <= rank_default(%d) <= rank_max(%d)"
                % (self.loc_rank, self.rank_default, self.rank_max))
        if expect:
            for k, v in expect.items():
                got = getattr(self, k)
                if got != v:
                    raise ValueError("%s 期望 %s，得到 %s" % (k, v, got))


# ------------------------------------------------------------------ 候选记录

@dataclasses.dataclass
class CandidateRecord:
    """一个候选位置的截断 SVD。

    U: int16[rank, M]、S: uint16[rank]、V: int16[rank, N]，**rank 为最外层**。
    三个 scale 是 float32：U/V 是 Q1.15 定点系数，S 是 Q2.14 归一化奇异值，
    各自乘自己的 scale 还原到浮点。

    `s_max` 是该候选的最大奇异值（未归一化），2026-09-29 起随候选头入 psm。
    S 在码字域里最大值恒为 1，绝对幅度 = S_code * s_scale * s_max（见 S_absolute）。
    合成记录（自检 / 小 fixture）不关心绝对幅度时留默认 1.0。
    """

    x_mm: int
    wall_distance_mm: int
    z_mm: int
    u_scale: float
    s_scale: float
    v_scale: float
    U: np.ndarray
    S: np.ndarray
    V: np.ndarray
    observable_mask: np.ndarray
    s_max: float = 1.0

    def validate(self, header: PsmHeader) -> "CandidateRecord":
        r, m, n = header.rank_max, header.meas_len, header.hidden_len
        if self.U.shape != (r, m):
            raise ValueError("U 形状应为 (%d, %d)，得到 %s" % (r, m, self.U.shape))
        if self.S.shape != (r,):
            raise ValueError("S 形状应为 (%d,)，得到 %s" % (r, self.S.shape))
        if self.V.shape != (r, n):
            raise ValueError("V 形状应为 (%d, %d)，得到 %s" % (r, n, self.V.shape))
        if self.observable_mask.shape != (n,):
            raise ValueError("mask 形状应为 (%d,)，得到 %s"
                             % (n, self.observable_mask.shape))
        for name, arr in (("U", self.U), ("S", self.S), ("V", self.V)):
            if arr.dtype != (np.uint16 if name == "S" else np.int16):
                raise ValueError("%s dtype 应为 %s，得到 %s"
                                 % (name, "uint16" if name == "S" else "int16",
                                    arr.dtype))
        if self.observable_mask.dtype != np.uint8:
            raise ValueError("mask dtype 应为 uint8")
        return self

    # -------------------------------------------------- 量化往返

    def U_float(self) -> np.ndarray:
        return self.U.astype(np.float64) * self.u_scale

    def S_float(self) -> np.ndarray:
        """归一化奇异值（最大 1）：S_code * s_scale，**未乘 s_max**。

        定位只需要方向，用这个就够了；重建要绝对幅度，用 S_absolute()。
        """
        return self.S.astype(np.float64) * self.s_scale

    def S_absolute(self) -> np.ndarray:
        """绝对奇异值 σ：S_code * s_scale * s_max。"""
        return self.S_float() * self.s_max

    def V_float(self) -> np.ndarray:
        return self.V.astype(np.float64) * self.v_scale

    def reconstruct(self) -> np.ndarray:
        """B ≈ U diag(σ) Vᵀ，返回 float64 的 (M, N)。仅供调试，不要在线用。

        用绝对奇异值 σ = S_code·s_scale·s_max，故返回值是**真实幅度**的 B，
        不是 B/s_max。合成记录 s_max 取默认 1.0 时两者一致。
        """
        U, s, V = self.U_float(), self.S_absolute(), self.V_float()
        return (U * s[None, :]) @ V.T


# ------------------------------------------------------------------ 几何哈希

def build_geometry_hash(geom: Dict, *, crc: bool = True) -> int:
    """由几何配置算 geometry_hash。

    规范串按**固定字段顺序**拼接，浮点统一用 `%.9g`（够表达 0.004583333 这类
    由除法定出来的值，又不至于把二进制尾数也带进去）。改任何一个参与字段都会
    换哈希，从而触发方案 §6.6 的「模型几何哈希不一致 -> 强制灰屏」。

    范围见 configs/geometry_b.yaml 的 geometry_hash_scope。
    """
    fm = geom.get("forward_model", {})
    parts: List[str] = []
    for key in ("planes", "wall", "hidden"):
        blk = geom.get(key, {})
        parts.append(key)
        for k in sorted(blk):
            parts.append("%s=%s" % (k, _fmt(blk[k])))
    parts.append("occluder_size=%s" % _fmt(geom.get("occluder", {}).get("size")))
    parts.append("occluder_center=%s"
                 % _fmt(geom.get("occluder", {}).get("nominal_center", {}).get("xyz")))
    for k in ("view_factor_z_pairing", "block_monitor_x_order", "stick",
              "view_angle", "monitor_discr_n", "radiation_term"):
        parts.append("%s=%s" % (k, _fmt(fm.get(k))))
    spec = "|".join(parts).encode("ascii")
    h = zlib.crc32(spec) & 0xFFFFFFFF
    return h


def _fmt(v) -> str:
    if isinstance(v, (list, tuple)):
        return "[" + ",".join(_fmt(x) for x in v) + "]"
    if isinstance(v, dict):
        return "{" + ",".join("%s:%s" % (k, _fmt(v[k])) for k in sorted(v)) + "}"
    if isinstance(v, bool):
        return "T" if v else "F"
    if isinstance(v, float):
        return "%.9g" % v
    return str(v)


# ------------------------------------------------------------------ 写

def write_model(path: str, header: PsmHeader,
                candidates: Iterable[CandidateRecord],
                *, verify: bool = True) -> Dict[str, object]:
    """流式写 model.psm，边写边算 CRC32。返回一份可写进 manifest 的摘要。

    刻意不把整份 payload 先拼进内存：243 候选约 300 MB，拼起来会让
    build_model.py 的内存占用翻倍，而且没有任何好处。
    """
    header.validate()
    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    crc = 0
    n_written = 0
    n_cand = 0
    per_cand: List[Dict[str, object]] = []

    with open(path, "wb") as f:
        f.write(header.pack(crc32=0, payload_bytes=0))   # 占位，最后回填
        for rec in candidates:
            rec.validate(header)
            blob = _pack_record(header, rec)
            f.write(blob)
            crc = zlib.crc32(blob, crc)
            n_written += len(blob)
            n_cand += 1
            per_cand.append(dict(x_mm=rec.x_mm, wall_distance_mm=rec.wall_distance_mm,
                                 z_mm=rec.z_mm,
                                 u_scale=rec.u_scale, s_scale=rec.s_scale,
                                 v_scale=rec.v_scale, s_max=float(rec.s_max),
                                 observable_frac=float(rec.observable_mask.mean())))
        if n_cand != header.candidates:
            raise ValueError("候选数不符：头部写 %d，实写 %d"
                             % (header.candidates, n_cand))
        f.seek(0)
        f.write(header.pack(crc32=crc & 0xFFFFFFFF, payload_bytes=n_written))

    summary = dict(path=os.path.abspath(path), bytes=os.path.getsize(path),
                   payload_bytes=n_written, crc32=crc & 0xFFFFFFFF,
                   geometry_hash=header.geometry_hash,
                   candidates=n_cand, candidates_meta=per_cand)
    if verify:
        chk = verify_model(path, expect_hash=header.geometry_hash)
        summary["verify"] = chk
    return summary


def _pack_record(header: PsmHeader, rec: CandidateRecord) -> bytes:
    """按对齐规则拼一条记录。对齐是契约，故这里显式补齐而不是靠调用方。"""
    off = header.array_offsets()
    buf = bytearray(header.record_bytes)
    _CAND_HDR_STRUCT.pack_into(buf, 0, rec.x_mm, rec.wall_distance_mm, rec.z_mm, 0,
                               rec.u_scale, rec.s_scale, rec.v_scale,
                               float(rec.s_max))
    regions = (
        ("U", np.ascontiguousarray(rec.U, dtype="<i2")),
        ("S", np.ascontiguousarray(rec.S, dtype="<u2")),
        ("V", np.ascontiguousarray(rec.V, dtype="<i2")),
        ("observable_mask", np.ascontiguousarray(rec.observable_mask, dtype=np.uint8)),
    )
    for name, arr in regions:
        raw = arr.tobytes(order="C")
        start = off[name]
        end = start + len(raw)
        if end > len(buf):
            raise ValueError("记录空间不足：%s 需要到 %d，记录只有 %d"
                             % (name, end, len(buf)))
        buf[start:end] = raw
    return bytes(buf)


# ------------------------------------------------------------------ 读

def read_model(path: str, *, load_records: bool = True,
               max_candidates: Optional[int] = None):
    """读 model.psm。`load_records=False` 时只解析头部（板端启动路径用这个）。"""
    with open(path, "rb") as f:
        header = PsmHeader.unpack(f.read(HEADER_BYTES))
        if not load_records:
            return header, []
        recs: List[CandidateRecord] = []
        n = header.candidates if max_candidates is None else \
            min(header.candidates, max_candidates)
        f.seek(HEADER_BYTES)
        for i in range(n):
            raw = f.read(header.record_bytes)
            if len(raw) != header.record_bytes:
                raise ValueError("候选 %d 读不满：期望 %d 字节，得到 %d"
                                 % (i, header.record_bytes, len(raw)))
            recs.append(_unpack_record(header, raw))
    return header, recs


def _unpack_record(header: PsmHeader, raw: bytes) -> CandidateRecord:
    off = header.array_offsets()
    x_mm, wall_d, z_mm, _res, us, ss, vs, smax = _CAND_HDR_STRUCT.unpack_from(raw, 0)
    r, m, n = header.rank_max, header.meas_len, header.hidden_len
    U = np.frombuffer(raw, dtype="<i2", count=r * m, offset=off["U"]).reshape(r, m)
    S = np.frombuffer(raw, dtype="<u2", count=r, offset=off["S"])
    V = np.frombuffer(raw, dtype="<i2", count=r * n, offset=off["V"]).reshape(r, n)
    mask = np.frombuffer(raw, dtype=np.uint8, count=n, offset=off["observable_mask"])
    return CandidateRecord(x_mm=x_mm, wall_distance_mm=wall_d, z_mm=z_mm,
                           u_scale=us, s_scale=ss, v_scale=vs, s_max=smax,
                           U=U.copy(), S=S.copy(), V=V.copy(),
                           observable_mask=mask.copy())


def read_loc_ranks(path: str, candidate_index: int, k: int) -> np.ndarray:
    """只读某个候选的前 k 个 rank 的 U 系数 -> (k, M) float64。

    这是定位引擎真正会走的路径：U 是 rank-major，前 k 个 rank 在文件里是
    **连续**的 k*M*2 字节，所以一次 pread 就够。方案 §5.3 的「只读前 K=8 列、
    单次约 35.5 MB」正是这个读法的产物。写个专门的函数是为了让板端实现有
    逐字节可对照的参照。
    """
    with open(path, "rb") as f:
        header = PsmHeader.unpack(f.read(HEADER_BYTES))
        if not (1 <= k <= header.rank_max):
            raise ValueError("k 必须在 1..%d，得到 %d" % (header.rank_max, k))
        if not (0 <= candidate_index < header.candidates):
            raise ValueError("候选下标越界：%d" % candidate_index)
        base = HEADER_BYTES + candidate_index * header.record_bytes
        f.seek(base)
        hdr = f.read(_CAND_HDR_STRUCT.size)
        _x, _y, _z, _r, u_scale, _ss, _vs, _smax = _CAND_HDR_STRUCT.unpack(hdr)
        off = header.array_offsets()
        f.seek(base + off["U"])
        raw = f.read(k * header.meas_len * 2)
    U = np.frombuffer(raw, dtype="<i2").reshape(k, header.meas_len)
    return U.astype(np.float64) * u_scale


def verify_model(path: str, *, expect_hash: Optional[int] = None) -> Dict[str, object]:
    """校验 CRC32、payload 长度、以及（可选）几何哈希。"""
    with open(path, "rb") as f:
        header = PsmHeader.unpack(f.read(HEADER_BYTES))
        f.seek(HEADER_BYTES)
        crc = 0
        total = 0
        while True:
            chunk = f.read(1 << 20)
            if not chunk:
                break
            crc = zlib.crc32(chunk, crc)
            total += len(chunk)
    ok_crc = (crc & 0xFFFFFFFF) == header.payload_crc32
    ok_len = total == header.payload_bytes
    ok_hash = True if expect_hash is None else header.geometry_hash == expect_hash
    res = dict(ok_crc32=ok_crc, ok_payload_bytes=ok_len, ok_geometry_hash=ok_hash,
               crc32=crc & 0xFFFFFFFF, payload_bytes=total,
               geometry_hash=header.geometry_hash,
               expected_hash=expect_hash)
    if not (ok_crc and ok_len and ok_hash):
        raise ValueError("model.psm 校验失败：%s" % res)
    return res


def summarize(path: str) -> str:
    """给 manifest / 日志用的一行人话摘要。"""
    header, _ = read_model(path, load_records=False)
    off = header.array_offsets()
    return ("%s: %dx%d -> %dx%d, %d 候选, rank max/default/loc = %d/%d/%d, "
            "单候选 %d B, payload %d B, crc32 %08X, geom %08X"
            % (os.path.basename(path), header.meas_h, header.meas_w,
               header.hidden_h, header.hidden_w, header.candidates,
               header.rank_max, header.rank_default, header.loc_rank,
               header.record_bytes, header.payload_bytes,
               header.payload_crc32, header.geometry_hash))


def parity_lines() -> List[str]:
    """与 `src/software/test_psm_format.cpp` **逐字符相同**的规范行（前 6 行）。

    两侧各自算、然后 diff，是保证「PC 生成 / 板端读取」不错位的唯一可靠办法：
    偏移算错不会崩，只会让板上读到看着合理但整体错位的数据。

    C 端在此之后还会多打三行 `check_header_*` —— 那是 C 校验器自身的行为检查
    （篡改哈希/破坏秩关系时是否报出正确错误码），不是跨语言一致性，Python 侧
    无从计算，故本函数只输出前 6 行；比对脚本按前缀比。
    """
    cases = [
        ("b_r64", 64, 95, 96, 18, 24),
        ("r96", 96, 95, 96, 18, 24),
        ("paper", 64, 62, 63, 29, 36),
        ("tiny", 4, 4, 3, 2, 2),
    ]
    out = []
    for name, rank, mh, mw, hh, hw in cases:
        h = PsmHeader(meas_h=mh, meas_w=mw, hidden_h=hh, hidden_w=hw,
                      candidates=1, rank_max=rank, rank_default=rank,
                      loc_rank=1, geometry_hash=0)
        off = h.array_offsets()
        out.append("%s rank=%d M=%d N=%d U=%d S=%d V=%d MASK=%d REC=%d"
                   % (name, rank, h.meas_len, h.hidden_len, off["U"], off["S"],
                      off["V"], off["observable_mask"], h.record_bytes))
    out.append("sizeof_header=%d sizeof_cand_hdr=%d"
               % (HEADER_BYTES, _CAND_HDR_STRUCT.size))
    out.append("off_magic=0 off_geom=24 off_crc=28 off_payload=32")
    return out


# ------------------------------------------------------------------ 自检
if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--parity":
        for line in parity_lines():
            print(line)
        sys.exit(0)
    if len(sys.argv) > 1:
        print(summarize(sys.argv[1]))
        print(verify_model(sys.argv[1]))
        sys.exit(0)

    # 不依赖任何数据的小自检：手工造 3 个候选，落盘再读回，逐位比对。
    hdr = PsmHeader(meas_h=4, meas_w=3, hidden_h=2, hidden_w=2, candidates=3,
                    rank_max=4, rank_default=2, loc_rank=2, geometry_hash=0xDEADBEEF)
    hdr.validate()
    rng = np.random.default_rng(0)
    recs = []
    for i in range(3):
        recs.append(CandidateRecord(
            x_mm=10 * i, wall_distance_mm=-20 + 10 * i, z_mm=20 * i,
            u_scale=1.0 / 4096, s_scale=1.0 / 16384, v_scale=1.0 / 4096,
            U=rng.integers(-32768, 32767, size=(4, 12), dtype=np.int16),
            S=rng.integers(0, 65535, size=(4,), dtype=np.uint16),
            V=rng.integers(-32768, 32767, size=(4, 4), dtype=np.int16),
            observable_mask=np.array([1, 1, 0, 1], dtype=np.uint8)))
    out = os.path.join(os.environ.get("TEMP", "."), "psm_selftest.psm")
    s = write_model(out, hdr, recs)
    print(summarize(out))
    h2, r2 = read_model(out)
    assert h2.geometry_hash == hdr.geometry_hash
    for a, b in zip(recs, r2):
        assert a.x_mm == b.x_mm and a.z_mm == b.z_mm
        assert np.array_equal(a.U, b.U), "U 往返不一致"
        assert np.array_equal(a.S, b.S), "S 往返不一致"
        assert np.array_equal(a.V, b.V), "V 往返不一致"
        assert np.array_equal(a.observable_mask, b.observable_mask)
        assert np.allclose(a.U_float(), b.U_float())
    U2 = read_loc_ranks(out, 1, 2)
    assert U2.shape == (2, 12) and np.allclose(U2, recs[1].U_float()[:2])
    print("自检通过：头部 %d B，单候选记录 %d B（对齐到 %d 的整数倍），"
          "U/S/V/mask 往返逐位一致，只读前 K 列路径一致"
          % (HEADER_BYTES, hdr.record_bytes, ARRAY_ALIGN))
