# -*- coding: utf-8 -*-
"""量化工具链（方案 §6.3 定点格式 / §6.5 求解器参数 / §9.2 定点误差指标）。

这个模块是「PC 浮点基线 → FPGA 定点改写」之间**唯一**的换算入口。方案 §6.3
末段定了一条硬要求：

> 每个量化点使用「向最近值舍入、饱和」而不是截断回绕。模型文件保存 scale 和
> 指数，bit-exact Python 模型必须与 RTL 逐向量相等或误差不超过 1 LSB。

因此本文件刻意只做三件事，每件事都能被逐位复算：
  1. **定标**：把浮点数组映射成定点码字 + 一个 scale（`quantize_*`）
  2. **回放**：把码字乘回 scale（`dequantize`），供 Python 侧与 RTL 对照
  3. **核算**：位宽链、共享指数、算子的谱范数（`FixedPointChain` / `shared_exponent`）

位宽链的两种口径（见 configs/fixed_point_b.yaml）
-------------------------------------------------
方案 §6.3 假设「RAW 12 bit 存于 16 bit」，块和取 `12 + ceil(log2(100)) = 19`（写 20）。
仓库数据契约实测官方数据按 **16 bit 满量程**参与累加，块和应取 `16 + 7 = 23`。
两者差 4 bit，取决于相机把 12 bit 放进 16 bit 容器的方式（右对齐 / 左移 4 位）。
本模块**同时给出两条链**，由 `value_packing` 选择。

**默认 = `packed`（右对齐 12 bit）**，与 `configs/fixed_point_b.yaml: value_packing`
以及 `configs/camera_b.yaml` 的待实测项口径**必须一致**（此前本类默认 `fullscale`，
导致 `FixedPointChain()` 裸构造与 `FixedPointChain.from_yaml(cfg)` 得到两条不同的链）。
选 `packed` 而不是 `fullscale` 的理由是**风险不对称**：
  * 按 packed 建而实物是 fullscale -> **错**（差分通路被削 84%~92%，静默削顶）；
  * 按 fullscale 建而实物是 packed -> 只**浪费 4 bit 位宽**，不产生正确性问题。
所以在首帧实拍定案之前，保底策略是「按 packed 建通路 + 把 fullscale 的加宽预案记在案」
（`configs/fixed_point_b.yaml: if_fullscale_then_widen`）。
`fullscale` 仍作为**显式可选项**保留（传 `value_packing="fullscale"`，或 YAML 里显式写
`value_packing: fullscale`）；一旦切换，`check()` 会报出差分通路**不够宽**（26~28 bit vs 声明 24）。

一个容易踩的坑：`saturate` 与 `wrap` 的差别在单次观测上只有个别样点不同，但在
16 次迭代的重建里会通过 `A` 的乘加放大。故所有量化点一律 `np.rint` + `clip`，
**不允许**用 `astype` 当量化（那是截断，且会对负数朝零截断而引入符号偏置）。
"""

from __future__ import annotations

import dataclasses
import math
import os
from typing import Dict, Optional, Sequence, Tuple

import numpy as np


# ==================================================================== 舍入

def round_half_even(x: np.ndarray) -> np.ndarray:
    """向最近值舍入，半值取偶。等价于 RTL 里的 round-half-to-even。

    用 `np.rint` 而不是 `np.round`：两者在这个语义下相同，但 `np.rint` 更直接
    地表意「round to nearest integer」，且不会因为 ndigits 参数被误用改语义。
    """
    return np.rint(np.asarray(x, dtype=np.float64))


def saturate(codes: np.ndarray, lo: int, hi: int) -> np.ndarray:
    """饱和到 [lo, hi]。**禁止回绕**（方案 §6.3）。"""
    return np.clip(codes, lo, hi)


# ==================================================================== 对称定点

@dataclasses.dataclass(frozen=True)
class FixedArray:
    """一份定点数据：码字 + 实际步长 scale + **名义** Q 格式。

    `nominal_int_bits` 是**声明**的格式（方案 §6.3 那一列），不是从 scale 反推的。
    这一点很关键：U/V 是「Q1.15 + 每候选独立 scale」，scale 是自适应的，从它反推
    小数位会得出乱七八糟的结果（实测把 Q1.15 反推成 Q1.15 纯属数值巧合，换个数据
    就变成 Q0.15 之类）。而 RTL 是按**声明的**格式搭的，Python 侧必须报同一个格式，
    否则「bit-exact 与 RTL 对照」这条判据没有共同的记法。
    """

    codes: np.ndarray
    scale: float
    bits: int
    signed: bool
    nominal_int_bits: int
    label: str = ""

    @property
    def nominal_frac_bits(self) -> int:
        return self.bits - self.nominal_int_bits

    @property
    def q_format(self) -> str:
        """形如 Q1.15 / Q2.14 / Q2.16 的名义记号。"""
        return "Q%d.%d" % (self.nominal_int_bits, self.nominal_frac_bits)

    @property
    def scale_pow2(self) -> Optional[int]:
        """实际 scale 若是 2 的幂则返回其指数，否则 None（自适应 scale 通常是 None）。"""
        if self.scale <= 0:
            return None
        e = math.log2(self.scale)
        return int(round(e)) if abs(e - round(e)) < 1e-9 else None

    def dequantize(self) -> np.ndarray:
        return self.codes.astype(np.float64) * self.scale

    def max_abs_lsb_error(self, x_ref: np.ndarray) -> float:
        """与参考浮点的最大偏差，单位 LSB。判据：<= 1（方案 §6.3）。"""
        if self.scale == 0.0:
            return 0.0
        return float(np.max(np.abs(self.dequantize() - np.asarray(x_ref))) / self.scale)


def _nominal_frac_from_int(bits: int, int_bits: int) -> int:
    f = bits - int_bits
    if f < 0:
        raise ValueError("名义整数位 %d 超过总位宽 %d" % (int_bits, bits))
    return f


def quantize_symmetric(x: np.ndarray, bits: int,
                       *, scale: Optional[float] = None,
                       int_bits: int = 1,
                       label: str = "") -> FixedArray:
    """有符号对称量化。方案 §6.3 的 **Q1.15** 就是 `bits=16, int_bits=1`。

    `scale = None` 时按 `max|x|` 自适应：`scale = max|x| / (2^(bits-1) - 1)`。
    上限取 `2^(bits-1) - 1` 而不是 `2^(bits-1)`，是为了让 `-max` 与 `+max` 都有
    码字（否则最小负码字没有对应正码字，U/V 的点积会多出一个非对称偏置）。

    U/V 系数用这个函数 + **每候选独立 scale**（方案 §6.3）：`int_bits=1` 给出
    名义 Q1.15，而 scale 随候选自适应，scale 以 float32 写进 model.psm 候选头。
    """
    x = np.asarray(x, dtype=np.float64)
    hi = (1 << (bits - 1)) - 1
    if scale is None:
        m = float(np.max(np.abs(x))) if x.size else 0.0
        scale = (m / hi) if m > 0 else 1.0
    codes = saturate(round_half_even(x / scale), -hi, hi)
    return FixedArray(codes.astype(np.int16 if bits <= 16 else np.int32),
                      scale, bits, True, int_bits, label)


def quantize_symmetric_pow2(x: np.ndarray, bits: int, *,
                            int_bits: int = 1,
                            label: str = "") -> FixedArray:
    """有符号对称量化，但 scale **强制取 2 的幂**（增益精确可表示）。

    为什么需要它
    ---------------------------------------------------------------------------
    U/V 用「每候选独立 scale」（方案 §6.3），而定点层的增益字段是 Q16.16。
    Q16.16 的分辨率是 1.53e-5，而真实 B 档模型的 scale 落在 1e-6~1e-5 量级：
    实测 `gain_q16 = 0.2~0.8`，**全部取整到 0**（6/6），相对误差上界
    `0.5/gain_q16` 直接爆掉。判据（见 `test_lr_operator.cpp` 的 L7 段）要求
    `gain_q16 >= 500` 才能把增益相对误差压在 1e-3 —— 差 2500 倍。

    把 scale 强制成 2 的幂之后，增益**精确可表示**：写成「尾数 = 1，指数 = e」
    的形式，在任何带指数的表示下都无舍入误差，相对误差恒为 0。

    代价：码字动态范围使用率下降。`2^ceil` 相对连续的 `max/hi` 最多浪费一倍，
    即最多少用 1 bit 码字精度。这个代价换来「增益误差恒为 0」。
    `test_lr_operator.cpp` 的 L7 段已独立验过：幂次 scale 下总误差 <= 1 LSB。

    注意：**这只是 PC 侧的能力**。要真正生效，接收端（`psm.py` 写盘、
    `psm_format.h`/`model_loader` 读盘、`ps_fixed.hpp` 的 gain 解算）必须同步
    把增益字段的语义从「Q16.16 小数」改成「指数」。在此之前的模型一律仍按
    旧语义解释 —— 详见 `docs/gain_representation.md`。
    """
    x = np.asarray(x, dtype=np.float64)
    hi = (1 << (bits - 1)) - 1
    if x.size == 0:
        scale = 1.0
    else:
        m = float(np.max(np.abs(x)))
        if m <= 0:
            scale = 1.0
        else:
            # 连续最优 scale 再**向上**取到 2 的幂；向上保证不溢出。
            ideal = m / hi
            scale = float(math.ldexp(1.0, int(math.ceil(math.log2(ideal)))))
    codes = saturate(round_half_even(x / scale), -hi, hi)
    return FixedArray(codes.astype(np.int16 if bits <= 16 else np.int32),
                      scale, bits, True, int_bits, label)


def scale_to_exponent(scale: float) -> int:
    """把 2 的幂 scale 转成指数。

    **不是 2 的幂就抛错**，而不是四舍五入 —— 这样调用方不会静默写进一个
    被悄悄改过的增益。这正是旧表示出问题的方式：Q16.16 把 1e-6 悄悄变成 0。
    """
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError("scale 必须是有限正数，得到 %r" % (scale,))
    e = math.log2(scale)
    if abs(e - round(e)) > 1e-9:
        raise ValueError("scale 不是 2 的幂：%r —— 幂次量化没有生效？" % (scale,))
    return int(round(e))


def exponent_to_scale(e: int) -> float:
    """`scale_to_exponent` 的逆。"""
    return float(math.ldexp(1.0, int(e)))


def gain_exponent_range_ok(e: int, *, width: int = 8) -> bool:
    """指数能否装进一个 `width` 位**有符号**整数字段。

    供 `build_model.py` 在写盘前自查：真实 B 档的 u/v 指数约 -17/-16，
    8 位有符号（[-128,127]）绰绰有余，余量 111 个 2 的幂。
    """
    lo, hi = -(1 << (width - 1)), (1 << (width - 1)) - 1
    return lo <= int(e) <= hi


def quantize_unsigned(x: np.ndarray, bits: int,
                      *, scale: Optional[float] = None,
                      int_bits: int = 2,
                      fixed_frac_bits: Optional[int] = None,
                      label: str = "") -> FixedArray:
    """无符号量化。方案 §6.3 的 **Q2.14** 就是 `bits=16, int_bits=2, fixed_frac_bits=14`。

    用于归一化奇异值 S（最大值另存 Q16.16 增益）。

    `fixed_frac_bits` 给了就**钉死** scale = 2^-frac（RTL 按声明格式搭，需要钉死）；
    没给则按 `max|x|` 自适应。

    注意方案选 Q2.14 而不是 Q1.15 是**故意留 2 个整数位**：S 已归一化到 [0,1]，
    用 Q1.15 本可多拿 1 bit 精度，但留出的整数头寸可以吸收「归一化时用错了参考
    最大值」这类误差而只表现为精度下降、不会溢出。本函数默认照方案走。
    """
    x = np.asarray(x, dtype=np.float64)
    hi = (1 << bits) - 1
    if fixed_frac_bits is not None:
        scale = math.ldexp(1.0, -fixed_frac_bits)
    elif scale is None:
        m = float(np.max(x)) if x.size else 0.0
        scale = (m / hi) if m > 0 else 1.0
    codes = saturate(round_half_even(x / scale), 0, hi)
    return FixedArray(codes.astype(np.uint16 if bits <= 16 else np.uint32),
                      scale, bits, False, int_bits, label)


def quantize_singular_values(s_norm: np.ndarray, bits: int = 16) -> FixedArray:
    """归一化奇异值的专用入口：按方案 §6.3 的 **Q2.14 unsigned 16 bit** 钉死格式。"""
    return quantize_unsigned(s_norm, bits, int_bits=2, fixed_frac_bits=14,
                             label="singular Q2.14")


# ==================================================================== 共享指数

@dataclasses.dataclass(frozen=True)
class SharedExponentFrame:
    """归一化测量：Q2.16 signed 18 bit + 每帧共享指数（方案 §6.3）。

    口径（2026-09-29 入库，定义与用法见 measurement_normalization_convention）
    ---------------------------------------------------------------------------
    每帧（三通道一起）算一个整数指数 `exponent`：

        exponent = ceil(log2(max|x|)) + 1        （max|x| = 0 时取 0）

    编码：`code = round_half_even(x / scale)`，`scale = 2^(exponent - frac_bits)`，
    饱和到 18 bit signed。解码（**板端/离线复算要用的除数**）：

        x = code * scale = code * 2^(exponent - frac_bits) = code / 2^(frac_bits - exponent)

    即除数是 `2^(frac_bits - exponent)`（也叫 `divisor`）。三通道共用同一个指数，
    通道间相对幅度因此被原样保留（方案 §6.4 的联合分数 s_p 是三通道求和）。
    """

    exponent: int
    mantissa_bits: int = 18
    frac_bits: int = 16

    @property
    def scale(self) -> float:
        return math.ldexp(1.0, -self.frac_bits + self.exponent)

    @property
    def divisor(self) -> float:
        """解码除数：x = code / divisor。= 2^(frac_bits - exponent)。"""
        return math.ldexp(1.0, self.frac_bits - self.exponent)

    @property
    def code_max(self) -> int:
        return (1 << (self.mantissa_bits - 1)) - 1

    def as_manifest(self) -> Dict[str, object]:
        """写进 manifest 的一帧口径（整数指数 + 静态格式 + 派生的除数）。"""
        return {
            "exponent": int(self.exponent),
            "mantissa_bits": int(self.mantissa_bits),
            "frac_bits": int(self.frac_bits),
            "scale": self.scale,
            "divisor": self.divisor,
        }


def measurement_normalization_convention() -> Dict[str, object]:
    """测量归一化的**静态口径**（与具体帧/数据无关的那部分）。

    为什么需要它入库：`shared_exponent()` 的每帧指数在 psm 与 manifest 里原先
    都没有字段，实拍路径因此**没有确定的除数** —— 拿到一帧 Q2.16 meas_norm 码字
    却不知道该除多少才能还原。指数本身是逐帧的运行时量，记在 experiment manifest
    的 `acquisition.measurement_normalization.frame_exponents`；本函数给的是
    「每帧指数怎么算、码字怎么解」的冻结口径，随 model manifest 一起入库，
    使离线复算与板端解算有同一套记法。
    """
    return {
        "scheme": "per_frame_shared_exponent",
        "format": "Q2.16 signed",
        "mantissa_bits": 18,
        "frac_bits": 16,
        "exponent_formula": "exponent = ceil(log2(max|x|)) + 1 (0 if max|x| == 0)",
        "encode": ("code = round_half_even(x / 2^(exponent - frac_bits)); "
                   "saturate to signed 18-bit"),
        "decode": ("x = code * 2^(exponent - frac_bits) "
                   "= code / 2^(frac_bits - exponent)"),
        "channels_share_exponent": True,
        "per_frame_values_recorded_in":
            "experiments/manifests 的 acquisition.measurement_normalization.frame_exponents",
    }



def shared_exponent(x: np.ndarray, *, mantissa_bits: int = 18,
                    frac_bits: int = 16) -> Tuple[FixedArray, SharedExponentFrame]:
    """按**每帧共享指数**把浮点测量压成 Q2.16。

    共享指数的意义：三个通道（R/G/B）共用同一个指数，于是通道间的相对幅度被
    原样保留 —— 若各通道各自归一化，RGB 联合分数 s_p 里三通道的相对权重就被
    悄悄改掉了（方案 §6.4 的 s_p 是三通道求和）。

    指数取 `ceil(log2(max|x|)) + 1`，留 1 bit 余量；mantissa 用 `(N-1)` 位存
    符号 + `frac_bits` 位小数，整数位 = `N - 1 - frac_bits`。
    """
    x = np.asarray(x, dtype=np.float64)
    m = float(np.max(np.abs(x))) if x.size else 0.0
    exp = 0 if m == 0 else (math.ceil(math.log2(m)) + 1)
    # 目标 scale：2^(exp - frac_bits)；等价于把整数位宽留给 2^exp 量级
    scale = math.ldexp(1.0, exp - frac_bits)
    hi = (1 << (mantissa_bits - 1)) - 1
    codes = saturate(round_half_even(x / scale), -hi, hi)
    # 名义格式：方案 §6.3 的 Q2.16 —— 整数位 2（含符号），故 mantissa_bits - frac_bits = 2
    fa = FixedArray(codes.astype(np.int32), scale, mantissa_bits, True,
                    mantissa_bits - frac_bits, "meas_norm")
    return fa, SharedExponentFrame(exponent=exp, mantissa_bits=mantissa_bits,
                                   frac_bits=frac_bits)


# ==================================================================== 位宽链

@dataclasses.dataclass(frozen=True)
class FixedPointChain:
    """整条位宽链。所有派生量都由字段算出，不允许手填（与 types.py 同一纪律）。"""

    raw_bits: int = 16
    effective_bits: int = 12          # 相机的有效位深（camera_b.yaml）
    block: int = 10
    frames: int = 4
    # 默认 packed：与 configs/fixed_point_b.yaml / camera_b.yaml 的待实测口径对齐，
    # 且风险不对称下它是保底方向（理由见模块文首）。可显式传 "fullscale"。
    value_packing: str = "packed"      # packed(右对齐) | fullscale(左移4位)

    diff_bits: int = 24
    coeff_bits: int = 16
    mac_acc_bits: int = 48
    image_state_bits: int = 24
    score_bits: int = 64
    loc_rank: int = 8
    recon_rank: int = 64

    # -------------------------------------------------- 派生

    def _src_bits(self) -> int:
        """参与累加的源位宽：右对齐时用有效位深，左移/满量程时用容器位宽。"""
        if self.value_packing == "packed":
            return self.effective_bits
        if self.value_packing == "fullscale":
            return self.raw_bits
        raise ValueError("value_packing 只能是 packed 或 fullscale，得到 %r"
                         % self.value_packing)

    def block_sum_bits(self) -> int:
        """块和位宽 = 源位宽 + ceil(log2(block^2))。"""
        return self._src_bits() + math.ceil(math.log2(self.block * self.block))

    def frame_acc_bits(self) -> int:
        """帧累积位宽 = 块和位宽 + ceil(log2(frames))。"""
        return self.block_sum_bits() + math.ceil(math.log2(max(self.frames, 1)))

    def diff_bits_needed(self) -> int:
        """差分需要「帧累积位宽 + 1 位符号」。"""
        return self.frame_acc_bits() + 1

    def mac_acc_bits_needed(self) -> int:
        """MAC 累加器需要 coeff_bits + 最大乘数位宽 + ceil(log2(累加长度))。"""
        return (self.coeff_bits * 2
                + math.ceil(math.log2(max(self.block * self.block, 1))))

    def scale_bits(self, *, int_bits: int, frac_bits: int) -> int:
        return int_bits + frac_bits

    # -------------------------------------------------- 校验

    def check(self, *, raise_on_shortfall: bool = True) -> Dict[str, object]:
        """核对「声明位宽 >= 需要位宽」。返回报告，可选抛异常。"""
        rep = {
            "value_packing": self.value_packing,
            "src_bits": self._src_bits(),
            "block_sum_bits": self.block_sum_bits(),
            "block_sum_declared": 20 if self.value_packing == "packed" else 23,
            "frame_acc_bits": self.frame_acc_bits(),
            "diff_bits_needed": self.diff_bits_needed(),
            "diff_bits_declared": self.diff_bits,
            "mac_acc_bits_needed": self.mac_acc_bits_needed(),
            "mac_acc_bits_declared": self.mac_acc_bits,
        }
        rep["ok_block_sum"] = rep["block_sum_declared"] >= rep["block_sum_bits"]
        rep["ok_diff"] = self.diff_bits >= rep["diff_bits_needed"]
        rep["ok_mac_acc"] = self.mac_acc_bits >= rep["mac_acc_bits_needed"]
        rep["ok"] = all(rep[k] for k in ("ok_block_sum", "ok_diff", "ok_mac_acc"))
        if raise_on_shortfall and not rep["ok"]:
            raise ValueError("位宽链不自洽：%s" % rep)
        return rep

    def describe(self) -> str:
        r = self.check(raise_on_shortfall=False)
        return ("value_packing=%s  源 %d bit -> 块和 %d（声明 %d）-> %d 帧累积 %d "
                "-> 差分 %d（声明 %d）；MAC 需要 %d（声明 %d）"
                % (self.value_packing, r["src_bits"], r["block_sum_bits"],
                   r["block_sum_declared"], self.frames, r["frame_acc_bits"],
                   r["diff_bits_needed"], r["diff_bits_declared"],
                   r["mac_acc_bits_needed"], r["mac_acc_bits_declared"]))

    @classmethod
    def from_yaml(cls, cfg: Dict) -> "FixedPointChain":
        """从 configs/fixed_point_b.yaml 的 dict 读。缺字段用默认值。

        `value_packing` 的兜底默认与数据类字段一致（`packed`），
        避免「YAML 缺这一行」时静默退回另一条链。
        """
        bc = cfg.get("bit_chain", {})
        return cls(
            raw_bits=int(bc.get("raw_in", {}).get("bits", 16)),
            block=int(bc.get("block_sum", {}).get("block", 10)),
            frames=int(bc.get("frame_acc", {}).get("frames_default", 4)),
            value_packing=str(cfg.get("value_packing", "packed")),
            diff_bits=int(bc.get("diff", {}).get("bits", 24)),
            coeff_bits=int(bc.get("coeff", {}).get("bits", 16)),
            mac_acc_bits=int(bc.get("mac_acc", {}).get("bits", 48)),
            image_state_bits=int(bc.get("image_state", {}).get("bits", 24)),
            score_bits=int(bc.get("score", {}).get("bits", 64)),
        )


# ==================================================================== 谱范数

def power_iteration_norm(B_apply, Bt_apply, n_in: int, *, iters: int = 200,
                         tol: float = 1e-10, seed: int = 0
                         ) -> Tuple[float, int, float]:
    """幂迭代估计 ||B||_2（方案 §6.5：PC 端据此导出安全的 tau、sigma）。

    只要求**算子**而不是矩阵，因此可以吃低秩形式 `B f = U(S∘(Vᵀf))`，不必把
    9120x432 的稠密矩阵物化。

    返回 `(norm, 实际迭代数, 最后一步的相对变化)`。相对变化没到 tol 也返回，
    由调用方决定要不要加迭代 —— 悄悄返回一个没收敛的值会让 tau 偏小、PDHG
    发散，而发散在图像上表现为「一片灰」，很难回溯到是范数没估准。
    """
    rng = np.random.default_rng(seed)
    v = rng.standard_normal(n_in)
    nv = float(np.linalg.norm(v))
    if nv == 0:
        return 0.0, 0, 0.0
    v /= nv
    lam_prev = 0.0
    delta = float("inf")
    used = 0
    for i in range(1, iters + 1):
        w = Bt_apply(B_apply(v))          # BᵀB v
        nw = float(np.linalg.norm(w))
        if nw == 0:
            return 0.0, i, 0.0
        v = w / nw
        lam = nw                            # ||BᵀB v|| / ||v|| = 瑞利商上界估计
        delta = abs(lam - lam_prev) / max(abs(lam), 1e-300)
        lam_prev = lam
        used = i
        if delta < tol:
            break
    return math.sqrt(max(lam_prev, 0.0)), used, delta


def pdhg_step_sizes(norm_B: float, *, safety: float = 0.99) -> Tuple[float, float]:
    """由 ||B||_2 导出 PDHG 的安全步长（方案 §6.5）。

    原始-对偶的经典条件是 `tau * sigma * ||B||^2 < 1`，取等号再乘 safety。
    这里取对称形式 `tau = sigma = sqrt(safety) / ||B||`，使两边的收敛速度一致；
    `safety = 0.99` 留 1% 余量吸收范数估计误差与定点舍入。
    """
    if norm_B <= 0:
        raise ValueError("||B|| 必须为正，得到 %r" % norm_B)
    s = math.sqrt(safety) / norm_B
    return s, s


# ==================================================================== 指标

def psnr(a: np.ndarray, b: np.ndarray, *, peak: Optional[float] = None) -> float:
    """峰值信噪比。peak 缺省取参考图的值域。"""
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    mse = float(np.mean((a - b) ** 2))
    if mse <= 0:
        return float("inf")
    if peak is None:
        peak = float(np.max(a) - np.min(a)) or 1.0
    return 10.0 * math.log10((peak * peak) / mse)


def ssim_global(a: np.ndarray, b: np.ndarray) -> float:
    """全局 SSIM（单窗口），用于 §9.2 的「SSIM 下降 <= 0.01/0.02」判据。

    只做全局单窗口：重建图是 24x18 的小图，滑窗 SSIM 的窗口尺寸会逼近图幅，
    退化成同一个量。这里显式说明，免得报告里被读成标准滑窗 SSIM。
    """
    a = np.asarray(a, dtype=np.float64).ravel()
    b = np.asarray(b, dtype=np.float64).ravel()
    ma, mb = a.mean(), b.mean()
    va, vb = a.var(), b.var()
    cov = float(np.mean((a - ma) * (b - mb)))
    c1 = (0.01 * 1.0) ** 2
    c2 = (0.03 * 1.0) ** 2
    num = (2 * ma * mb + c1) * (2 * cov + c2)
    den = (ma * ma + mb * mb + c1) * (va + vb + c2)
    return float(num / den) if den else 0.0


def check_acceptance(f_fixed: np.ndarray, f_float: np.ndarray,
                     ground_truth: Optional[np.ndarray] = None,
                     *, peak: Optional[float] = None,
                     psnr_drop_max: float = 0.5,
                     ssim_drop_max: float = 0.01) -> Dict[str, object]:
    """方案 §9.2 的定点误差判据：相对**同算法**浮点，PSNR 下降 <= 0.5 dB、SSIM 下降 <= 0.01。

    两个容易搞错的地方，这里显式处理：

    1. **「下降」需要一个共同的参考物。** 有真值时参考物就是真值：
       `drop = PSNR(浮点, 真值) - PSNR(定点, 真值)`。
       直接把浮点与定点互比 PSNR 得不到 0.5 dB 这个判据（同一张图自比是无穷大），
       所以 `ground_truth=None` 时退化成**代理量**并在返回值里标 `proxy=True`：
       以 `peak` 为满量程测「定点相对浮点漂移了多少」，仅用于盲测集没有真值的场合，
       不能拿它去宣称满足 §9.2。

    2. **「同算法」**：对照物必须是同一个求解器的浮点跑（PDHG 对 PDHG）。拿 FISTA
       浮点当基线会把求解器差异错算成量化损失（方案 §6.5 末条）。

    另外注意 `ssim_global` 是**全局单窗口** SSIM，不是滑窗 SSIM；24x18 的小图下滑窗
    窗口尺寸逼近图幅而退化，报告里要写明口径。
    """
    f_fixed = np.asarray(f_fixed, dtype=np.float64)
    f_float = np.asarray(f_float, dtype=np.float64)
    rep: Dict[str, object] = {}
    if ground_truth is not None:
        gt = np.asarray(ground_truth, dtype=np.float64)
        p_float = psnr(f_float, gt, peak=peak)
        p_fixed = psnr(f_fixed, gt, peak=peak)
        s_float = ssim_global(gt, f_float)
        s_fixed = ssim_global(gt, f_fixed)
        rep["proxy"] = False
    else:
        # 代理口径：以 peak 为满量程，量定点相对浮点的漂移
        peak_v = peak if peak is not None else (float(np.max(f_float)) or 1.0)
        p_float = float("inf")
        p_fixed = psnr(f_float, f_fixed, peak=peak_v)
        s_float, s_fixed = 1.0, ssim_global(f_float, f_fixed)
        rep["proxy"] = True
    rep.update(psnr_float=p_float, psnr_fixed=p_fixed,
               ssim_float=s_float, ssim_fixed=s_fixed)
    if rep["proxy"]:
        rep.update(psnr_drop_db=None, ssim_drop=1.0 - s_fixed,
                   ok_psnr=False, ok_ssim=(1.0 - s_fixed) <= ssim_drop_max,
                   ok=False,
                   note="无真值，PSNR 判据不可判（proxy 口径只给漂移量，不得宣称满足 §9.2）")
        return rep
    drop_p = p_float - p_fixed
    drop_s = s_float - s_fixed
    rep.update(psnr_drop_db=drop_p, ssim_drop=drop_s,
               ok_psnr=drop_p <= psnr_drop_max, ok_ssim=drop_s <= ssim_drop_max,
               ok=(drop_p <= psnr_drop_max and drop_s <= ssim_drop_max))
    return rep


# ==================================================================== 自检
if __name__ == "__main__":
    rng = np.random.default_rng(1)

    print("== 位宽链 ==")
    for packing in ("packed", "fullscale"):
        ch = FixedPointChain(value_packing=packing, block=10, frames=4)
        print("  %-10s %s" % (packing, ch.describe()))
        r = ch.check(raise_on_shortfall=False)
        if not r["ok"]:
            need = r["diff_bits_needed"]
            print("             ^ 不自洽：差分通路需要 %d bit，声明只有 %d bit。"
                  "改 value_packing 就必须同时加宽差分通路（这里是 +%d bit），"
                  "否则块和/累积会静默饱和。" % (need, r["diff_bits_declared"],
                                                need - r["diff_bits_declared"]))
    # 方案 §6.3 声明的那条链：12 bit 源、8 帧累积 23 bit、差分 24 bit —— 只在
    # packed（右对齐）口径下自洽。用 8 帧再确认一次。
    ch8 = FixedPointChain(value_packing="packed", block=10, frames=8)
    print("  8 帧 packed " + ch8.describe())
    ch8.check()                       # 8 帧时差分需 23 bit，声明 24 bit，自洽
    for frames in (1, 4, 8):
        FixedPointChain(value_packing="packed", block=10, frames=frames).check()
    print("  -> packed 口径在 1/4/8 帧下全部自洽；fullscale 口径需把差分通路加宽到 26 bit")

    # 口径默认值一致性（X29 修）：裸构造、from_yaml 兜底、YAML 实际值三处必须相同。
    # 此前 dataclass 默认 fullscale、而 configs/fixed_point_b.yaml 写 packed，
    # 于是 FixedPointChain() 与 FixedPointChain.from_yaml(cfg) 会得到两条不同的链。
    assert FixedPointChain().value_packing == "packed", "默认口径应为 packed"
    assert FixedPointChain.from_yaml({}).value_packing == "packed", "from_yaml 兜底应为 packed"
    _cfg_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        os.pardir, os.pardir, os.pardir, "configs", "fixed_point_b.yaml")
    try:
        import yaml                                    # 可选依赖
        with open(_cfg_path, "r", encoding="utf-8") as _f:
            _cfg = yaml.safe_load(_f)
        _from_yaml = FixedPointChain.from_yaml(_cfg).value_packing
        assert _from_yaml == FixedPointChain().value_packing, (
            "configs/fixed_point_b.yaml 的 value_packing=%r 与 dataclass 默认 %r 不一致"
            % (_from_yaml, FixedPointChain().value_packing))
        print("  口径默认一致性：dataclass / from_yaml / configs 三处均为 %r"
              % FixedPointChain().value_packing)
    except ImportError:
        print("  (无 PyYAML，跳过 YAML 侧一致性检查；dataclass 与 from_yaml 兜底已一致)")

    print("\n== 对称量化 Q1.15（U/V 系数）==")
    x = rng.standard_normal(4096) * 0.3
    fa = quantize_symmetric(x, 16)
    print("  %s  scale=%.3e  max LSB 误差=%.4f  -> %s"
          % (fa.q_format, fa.scale, fa.max_abs_lsb_error(x),
             "达标(<=1)" if fa.max_abs_lsb_error(x) <= 1.0 else "超标"))
    # 极端情形：正好命中半格的值必须走偶数舍入
    half = np.array([0.5, 1.5, 2.5, 3.5]) * 2 ** -15
    fh = quantize_symmetric(half, 16, scale=2 ** -15)
    print("  半格值 %s -> 码字 %s（偶数舍入）" % (half * 2 ** 15, fh.codes))

    print("\n== 无符号量化 Q2.14（归一化奇异值，格式钉死）==")
    s = np.sort(rng.random(64))[::-1]
    s = s / s[0]
    fs = quantize_singular_values(s)
    print("  %s  scale=2^%s  码字范围=[%d, %d]  max LSB 误差=%.4f"
          % (fs.q_format, fs.scale_pow2, fs.codes.min(), fs.codes.max(),
             fs.max_abs_lsb_error(s)))
    assert fs.q_format == "Q2.14", fs.q_format
    print("  自适应对照（不钉死格式）：%s" % quantize_unsigned(s, 16).q_format)

    print("\n== 共享指数 Q2.16（归一化测量，三通道共指数）==")
    y = np.stack([rng.random(1000) * 4.0 + 0.1,
                  rng.random(1000) * 4.0 + 0.1,
                  rng.random(1000) * 4.0 + 0.1])
    fy, frame = shared_exponent(y)
    print("  %s  指数=%d  scale=2^%d  码字范围=[%d, %d]"
          % (fy.q_format, frame.exponent, fy.scale_pow2,
             fy.codes.min(), fy.codes.max()))
    rel = np.abs(fy.dequantize() - y).max() / np.abs(y).max()
    # 名义小数位 16，但 exp 取 ceil(log2(max))+1 会吃掉约 1 bit 头寸，
    # 故实际相对精度约 2^-15 而不是 2^-17。这里按 2^-14 设闸口，留 1 bit 余量。
    print("  最大相对偏差 %.3e（名义 2^-16=%.2e，因 1 bit 头寸实际约 2^-15=%.2e）"
          % (rel, 2 ** -16, 2 ** -15))
    assert rel < 2 ** -14, "共享指数精度不足"
    # 三通道共用指数：相对幅度必须原样保留
    ratio_src = float(y[0].mean() / y[2].mean())
    ratio_q = float(fy.dequantize()[0].mean() / fy.dequantize()[2].mean())
    print("  通道 0/2 的幅度比 原始 %.6f -> 量化后 %.6f（差 %.1e，共指数不改变相对幅度）"
          % (ratio_src, ratio_q, abs(ratio_src - ratio_q) / ratio_src))
    assert abs(ratio_src - ratio_q) / ratio_src < 1e-4
    # 口径入库（2026-09-29）：解码除数 = 2^(frac_bits - exponent)，且与 dequantize 一致
    conv = measurement_normalization_convention()
    assert conv["frac_bits"] == frame.frac_bits == 16, conv
    assert conv["mantissa_bits"] == frame.mantissa_bits == 18, conv
    assert abs(frame.divisor - 2.0 ** (frame.frac_bits - frame.exponent)) < 1e-12
    assert np.allclose(fy.dequantize(), fy.codes.astype(np.float64) / frame.divisor)
    mf = frame.as_manifest()
    assert mf["divisor"] == frame.divisor and mf["exponent"] == frame.exponent
    print("  口径：exponent=%d，解码除数 2^(16-%d)=%.1f；dequantize == code/divisor 一致"
          % (frame.exponent, frame.exponent, frame.divisor))

    print("\n== 饱和而非回绕 ==")
    big = np.array([0.0, 1e9, -1e9])
    fb = quantize_symmetric(big, 16, scale=2 ** -15)
    print("  [0, 1e9, -1e9] -> %s（顶到 ±32767，不是回绕）" % fb.codes)

    print("\n== PDHG 步长（幂迭代估计 ||B||_2）==")
    A = rng.standard_normal((200, 60))
    nrm, used, delta = power_iteration_norm(
        lambda f: A @ f, lambda r: A.T @ r, 60)
    exact = float(np.linalg.svd(A, compute_uv=False)[0])
    print("  幂迭代 %.6f（%d 次，末次相对变化 %.2e）  精确 %.6f  相对误差 %.2e"
          % (nrm, used, delta, exact, abs(nrm - exact) / exact))
    tau, sigma = pdhg_step_sizes(nrm)
    print("  tau = sigma = %.6e   tau*sigma*||B||^2 = %.6f（须 < 1）"
          % (tau, tau * sigma * nrm * nrm))
    assert tau * sigma * nrm * nrm < 1.0
    # 幂迭代**低估**范数（瑞利商从下方收敛），低估会让 tau 偏大、PDHG 可能发散。
    # 这里量一下低估多少，确认 safety=0.99 的 1% 余量足够覆盖。
    print("  幂迭代低估比例 %.3e（正因它是低估，safety=0.99 的 1%% 余量才必须留）"
          % ((exact - nrm) / exact))
    assert (exact - nrm) / exact < 0.01, "范数低估超过 1% 余量，safety 要调小"

    print("\n== §9.2 判据走一遍 ==")
    gt = (rng.random((18, 24)) > 0.6) * 1.0            # 稀疏方块真值
    f_ref = gt + rng.standard_normal((18, 24)) * 0.02  # 浮点基线
    f_q = gt + rng.standard_normal((18, 24)) * 0.021   # 定点（噪声略大）
    rep = check_acceptance(f_q, f_ref, gt, peak=1.0)
    print("  PSNR 浮点 %.2f -> 定点 %.2f（降 %.3f dB，判据 <=0.5）"
          % (rep["psnr_float"], rep["psnr_fixed"], rep["psnr_drop_db"]))
    print("  SSIM 浮点 %.4f -> 定点 %.4f（降 %.4f，判据 <=0.01）-> %s"
          % (rep["ssim_float"], rep["ssim_fixed"], rep["ssim_drop"],
             "达标" if rep["ok"] else "超标"))
    rep2 = check_acceptance(f_q, f_ref, None, peak=1.0)
    print("  无真值时：proxy=%s，PSNR 判据 %s（不得宣称满足 §9.2）"
          % (rep2["proxy"], "不可判" if rep2["psnr_drop_db"] is None else "可判"))
    assert rep2["proxy"] and rep2["psnr_drop_db"] is None and not rep2["ok"]
    print("\n全部自检通过。")
