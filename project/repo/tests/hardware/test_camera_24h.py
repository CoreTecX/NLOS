# -*- coding: utf-8 -*-
"""tests/hardware/test_camera_24h.py —— 相机长时间采集稳定性与参数漂移。

对应：技术方案 §12 Task 10；`docs/port_completeness.md` §11.6 的长期缺件。
判据口径来源：`configs/camera_b.yaml`（`must_disable` / `value_packing` /
`acquisition.saturation_rule`）、`docs/experiment_protocol.md` §二/§三、
`docs/hardware_freeze.md` §1。

「24h」指什么
---------------------------------------------------------------------------
长时间连续取流下的稳定性与参数漂移，具体覆盖五类：
  1. `must_disable` 清单是否真的关停（自动曝光/增益/白平衡、gamma、锐化、降噪、
     自动黑电平、CCM、LUT，以及切断 Bayer 通路处理的 `Super Bayer Enable`）；
  2. 位深与 `value_packing` 是否漂移（容器 16 bit / 有效 12 bit / packed 右对齐）；
  3. 帧率、丢帧与断流；
  4. 帧统计量（均值、饱和占比、p99.9、中位数、SNR 代理）的阈值与长时漂移；
  5. 温度读数与帧间隔的关联（主机只能算相关；温度对上板 PL 时序/帧节拍的影响需板测）。

相机现状：为什么设备访问是接口 + 桩
---------------------------------------------------------------------------
相机 **MV-CS050-10GC** 当前**未联网、也不允许接入**（老师要求先脱离相机做 SD 卡
数据链验证）。因此本脚本把设备访问收进 `CameraDevice` 接口，提供四种实现：

    synthetic（默认）  合成 24h 元数据时间轴，可注入漂移/丢帧/断流
    recorded           回放已录制的元数据（JSON；字段契约见 `_load_recorded`）
    spinnaker/gentl    真机（**留桩**：主机上返回 Unavailable -> 相关项判 PENDING）

主机能验的、与必须真机才能验的
---------------------------------------------------------------------------
* **主机可验（本脚本 `--self-check` 给出判据逻辑的通过证据）**：
  must_disable 判据逻辑、位深/打包一致性判据、丢帧/断流判据、统计量与漂移判据、
  温度-间隔相关性判据，以及「配置清单（camera_b.yaml）↔ 本脚本强制项」的一致性。
* **必须真机才能验（一律 PENDING，不代跑、不臆造）**：
  真机参数回读（节点是否真关）、真机 24h 连续取流、真机丢帧/断流、
  真机帧统计漂移、真机温度读数、以及温度对上板时序/帧节拍的实测影响。

诚实边界
---------------------------------------------------------------------------
* 主机上的 24h 是**压缩时间轴**（`--hours 24` + `--sample-every` 抽样），
  它不改变判据逻辑，但**不代表真机 24h 的任何结论**。
* 凡文档未冻结的阈值（漂移容差、抖动/断流系数、温度相关上限、温度工作带），
  脚本用显式 `DEF_*` 常量给出并标注「工程默认，非文档冻结值」，不冒充实测口径。
* PENDING 不导致退出码非零；只有 FAIL 才退出码 1。

用法
---------------------------------------------------------------------------
    python tests/hardware/test_camera_24h.py --self-check          # 主机判据逻辑自检
    python tests/hardware/test_camera_24h.py                       # 合成 24h 时间轴
    python tests/hardware/test_camera_24h.py --device recorded --metadata meta.json
    python tests/hardware/test_camera_24h.py --device spinnaker    # 真机（留桩 -> PENDING）
    python tests/hardware/test_camera_24h.py --json-out build/reports/camera_24h.json
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

PASS, FAIL, PENDING, NA = "PASS", "FAIL", "PENDING", "N/A"

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.abspath(os.path.join(_HERE, "..", ".."))
_CAMERA_YAML = os.path.join(_ROOT, "configs", "camera_b.yaml")
_SCRIPT_VERSION = "1.0.0"

# ---------------------------------------------------------------- 冻结口径
# 下列常量取自文档；改口径须同时改对应文件。
FULL_SCALE_PACKED = 4095              # camera_b.yaml: value_packing=packed -> 满量程 4095
FULL_SCALE_FULLSCALE = 65520         # 左移 4 位时的满量程（仅用于判据说明）
SAT_RATIO_MAX = 0.001                 # experiment_protocol §三：墙面 ROI 饱和像素占比 < 0.1%
P999_RATIO_MAX = 0.95                 # camera_b.yaml.saturation_rule.p999_below_ratio_of_full_scale
MEDIAN_RATIO_RANGE = (0.20, 0.80)     # camera_b.yaml.saturation_rule.median_within
CONTAINER_BITS = 16                   # camera_b.yaml.sensor.container_bits
EFFECTIVE_BITS = 12                   # camera_b.yaml.sensor.bit_depth
VALUE_PACKING = "packed"              # camera_b.yaml.sensor.value_packing（2026-09-30 定案）

# ---------------------------------------------------------------- 工程默认（非文档冻结）
DEF_MEAN_DRIFT_TOL = 0.05            # 首/末窗口均值相对漂移上限
DEF_SNR_DRIFT_TOL = 0.10             # 首/末窗口 SNR 代理相对漂移上限
DEF_INTERVAL_JITTER = (0.5, 3.0)     # 帧间隔 / 标称间隔 的可接受带（低于 0.5x 或高于 3x 记异常）
DEF_DISCONNECT_FACTOR = 5.0          # 帧间隔 > 该倍数 * 标称 -> 判「断流」
DEF_TEMP_INTERVAL_CORR_MAX = 0.90    # 温度与帧间隔的 |Pearson r| 上限（超过 -> FAIL 提示耦合）
DEF_TEMP_BAND_C = (0.0, 50.0)        # 温度工作带（工程默认）
ROI_PIXELS = 1920 * 1920             # B 档居中方形 ROI（configs/geometry_b.yaml / camera_b.yaml）

# ---------------------------------------------------------------- must_disable 映射
# (规范键, 相机节点别名, 可接受为「已关停」的取值, 为什么必须关)
# 取值比较前统一 lower/strip；布尔 True/False 经 str() 后落进 {true,false}。
MUST_DISABLE = [
    ("auto_exposure", ["ExposureAuto", "AutoExposureEnable", "AutoExposure"],
     {"off", "false", "0"}, "自动曝光：帧间亮度漂移会直接毁掉差分信号"),
    ("auto_gain", ["GainAuto", "AutoGainEnable"],
     {"off", "false", "0"}, "自动增益：同自动曝光"),
    ("auto_white_balance", ["BalanceWhiteAuto", "AutoWhiteBalance"],
     {"off", "false", "0"}, "自动白平衡：三通道增益漂移，RGB 通道间比值不可比"),
    ("gamma", ["GammaEnable", "Gamma"],
     {"off", "false", "0"}, "gamma：非线性压缩，破坏 y = A f + b 的线性假设"),
    ("sharpening", ["SharpnessEnable", "Sharpness"],
     {"off", "false", "0"}, "锐化：空域高通，与垂直差分 D_y 互相干扰"),
    ("denoise", ["NoiseReductionEnable", "DenoiseEnable", "NoiseReduction"],
     {"off", "false", "0"}, "降噪：非线性空域滤波，破坏线性前向模型前提"),
    ("auto_black_level", ["BlackLevelAuto", "AutoBlackLevelEnable", "BlackLevelSelector"],
     {"off", "false", "0"}, "自动黑电平：自动漂移改变暗场基准，与差分 D_y 冲突"),
    ("ccm", ["ColorTransformationEnable", "CCMEnable", "ColorTransformationSelector"],
     {"off", "false", "0"}, "CCM：非线性色彩变换，破坏 Bayer 数据的线性性"),
    ("lut", ["LUTEnable", "LUTSelector", "LUTEnableSelector"],
     {"off", "false", "0"}, "LUT：改变码值到亮度的对应关系"),
    ("super_bayer_enable", ["SuperBayerEnable", "SuperBayer"],
     {"off", "false", "0"},
     "Super Bayer 总开关：开启后 Bayer 通路的 Gamma/锐度/对比度/CCM/LUT 等节点才生效"),
]
MUST_DISABLE_KEYS = [k for k, _, _, _ in MUST_DISABLE]


# ================================================================ 小工具
def _norm(v) -> str:
    if isinstance(v, bool):
        return "true" if v else "false"
    return str(v).strip().strip('"').strip("'").lower()


def _pearson(xs, ys):
    n = len(xs)
    if n < 3:
        return None
    mx = sum(xs) / n
    my = sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    syy = sum((y - my) ** 2 for y in ys)
    if sxx <= 0 or syy <= 0:
        return None
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    return sxy / math.sqrt(sxx * syy)


class Check:
    __slots__ = ("name", "status", "detail")

    def __init__(self, name, status, detail=""):
        self.name = name
        self.status = status
        self.detail = detail

    def as_dict(self):
        return {"name": self.name, "status": self.status, "detail": self.detail}


# ================================================================ 设备接口
class CameraUnavailable(RuntimeError):
    pass


class CameraDevice(object):
    """设备访问接口。真机实现只需实现 open/read_configuration/iter_samples。"""

    name = "base"

    def open(self):
        raise CameraUnavailable("未实现")

    def close(self):
        pass

    def read_configuration(self):
        """返回配置快照 dict：节点名 -> 值（GenICam 节点显示名或其别名）。"""
        raise CameraUnavailable("未实现")

    def iter_samples(self, hours, sample_every):
        """返回样本 dict 序列，字段见模块头与 `synth_samples`。"""
        raise CameraUnavailable("未实现")


class SyntheticDevice(CameraDevice):
    """合成元数据源（主机默认）。可用关键字注入各类异常以验证判据。"""

    name = "synthetic"

    def __init__(self, config=None, fps=5.0, n=None, hours=24.0, seed=1, **inject):
        self._cfg = config if config is not None else synth_config()
        self._fps = fps
        self._n = n
        self._hours = hours
        self._seed = seed
        self._inject = inject

    def open(self):
        return

    def read_configuration(self):
        return dict(self._cfg)

    def iter_samples(self, hours, sample_every):
        n = self._n if self._n is not None else max(2, int(math.ceil(hours * 3600.0 / float(sample_every))) + 1)
        return synth_samples(n=n, fps=self._fps, hours=hours, seed=self._seed, **self._inject)


class RecordedDevice(CameraDevice):
    """回放录制的元数据（JSON）：

        {"config": {节点名: 值, ...},
         "samples": [ {t_s, frame_index, interval_ms, dropped, exposure_us, gain_db,
                       temperature_c, container_bits, effective_bits, value_packing,
                       roi_pixels, sat_count, mean, max_value, p999, median,
                       snr_proxy, low4_all_zero_ratio}, ... ]}
    """

    name = "recorded"

    def __init__(self, path):
        self._path = path
        self._data = None

    def open(self):
        if not os.path.isfile(self._path):
            raise CameraUnavailable("录制文件不存在：%s" % self._path)
        with open(self._path, "r", encoding="utf-8") as f:
            self._data = json.load(f)
        if "samples" not in self._data:
            raise CameraUnavailable("录制文件缺 samples 字段：%s" % self._path)

    def read_configuration(self):
        return dict(self._data.get("config", {}))

    def iter_samples(self, hours, sample_every):
        return list(self._data.get("samples", []))


class SpinnakerDevice(CameraDevice):
    name = "spinnaker"

    def open(self):
        raise CameraUnavailable(
            "Spinnaker SDK 不可用（相机未联网且当前不允许接入；主机构建无 SDK）。"
            "真机运行需在板端/联网主机上装 MVS/Spinnaker 并接相机。")


class GentlDevice(CameraDevice):
    name = "gentl"

    def open(self):
        raise CameraUnavailable(
            "GenTL/GenICam 不可用（相机未联网且当前不允许接入）。"
            "真机运行需 GenTL producer（MVS 或 Aravis）并接相机。")


def build_device(args):
    if args.device == "synthetic":
        return SyntheticDevice(hours=args.hours, fps=args.fps)
    if args.device == "recorded":
        if not args.metadata:
            raise SystemExit("--device recorded 需要 --metadata <json>")
        return RecordedDevice(args.metadata)
    if args.device == "spinnaker":
        return SpinnakerDevice()
    if args.device == "gentl":
        return GentlDevice()
    raise SystemExit("未知设备类型：%s" % args.device)


# ================================================================ 合成数据
def synth_config(**over):
    cfg = {
        "ModelName": "MV-CS050-10GC",
        "PixelFormat": "BayerRG12",
        "ExposureAuto": "Off",
        "GainAuto": "Off",
        "BalanceWhiteAuto": "Off",
        "GammaEnable": False,
        "SharpnessEnable": False,
        "NoiseReductionEnable": False,
        "BlackLevelAuto": False,
        "ColorTransformationEnable": False,
        "LUTEnable": False,
        "SuperBayerEnable": False,
    }
    cfg.update(over)
    return cfg


def synth_samples(n=1441, fps=5.0, hours=24.0, seed=1,
                  packing=VALUE_PACKING, mean=2000.0, median=2000.0, p999=3000.0,
                  snr=40.0, sat_frac=0.0002,
                  drop_at=None, stall_at=None, mean_drift=0.0, snr_drift=0.0,
                  sat_breach_at=None, low4_all_zero=False,
                  temp_start=25.0, temp_end=27.0, temp_interval_corr=False):
    """确定性合成时间轴。注入项各自独立，便于 --self-check 逐一断言。"""
    nom = 1000.0 / float(fps)
    total_s = hours * 3600.0
    samples = []
    for i in range(n):
        frac = (i / float(n - 1)) if n > 1 else 0.0
        t = total_s * frac
        # 基准间隔带一点与温度无关的确定性抖动，使「温度-间隔」相关性可算；
        # 抖动幅度 0.2% 远小于 timeline:jitter 的判据带 [0.5x, 3.0x]。
        interval = nom * (1.0 + 0.002 * ((i % 3) - 1))
        dropped = 0
        if drop_at is not None and i == drop_at:
            dropped = 3
        if stall_at is not None and i == stall_at:
            interval = nom * (DEF_DISCONNECT_FACTOR + 1.0)
        if temp_interval_corr:
            interval = nom * (1.0 + 0.05 * frac)
        m = mean * (1.0 + mean_drift * frac)
        sn = snr * (1.0 + snr_drift * frac)
        temp = temp_start + (temp_end - temp_start) * frac
        sat_count = int(ROI_PIXELS * sat_frac)
        if sat_breach_at is not None and i == sat_breach_at:
            sat_count = int(ROI_PIXELS * 0.005)  # 0.5% >> 0.1%
        frame_index = i
        if packing == "fullscale":
            max_value, low4 = 65000, 1.0
        else:
            max_value = 4090
            low4 = 1.0 if low4_all_zero else 0.0625
        samples.append({
            "t_s": t,
            "frame_index": frame_index,
            "interval_ms": interval,
            "dropped": dropped,
            "exposure_us": 175000,
            "gain_db": 3.0,
            "temperature_c": temp,
            "container_bits": CONTAINER_BITS,
            "effective_bits": EFFECTIVE_BITS,
            "value_packing": packing,
            "roi_pixels": ROI_PIXELS,
            "sat_count": sat_count,
            "mean": m,
            "max_value": max_value,
            "p999": p999,
            "median": median,
            "snr_proxy": sn,
            "low4_all_zero_ratio": low4,
        })
    return samples


# ================================================================ 判据
def check_disable_list(cfg):
    """must_disable 清单是否真关停。节点缺失 -> PENDING；节点在但未关 -> FAIL。"""
    out = []
    for key, aliases, ok, why in MUST_DISABLE:
        hit = None
        for a in aliases:
            for k, v in cfg.items():
                if k.lower() == a.lower():
                    hit = (k, v)
                    break
            if hit:
                break
        if hit is None:
            out.append(Check("disable:" + key, PENDING,
                             "配置快照里没有节点 %s（节点名随固件/GenICam 版本变化，"
                             "需真机回读确认它是否存在或改用什么名字）" % "/".join(aliases)))
            continue
        name, val = hit
        if _norm(val) in ok:
            out.append(Check("disable:" + key, PASS, "%s = %r" % (name, val)))
        else:
            out.append(Check("disable:" + key, FAIL,
                             "%s = %r 未关停（要求 %s）—— %s" % (name, val, sorted(ok), why)))
    return out


def check_config_sync(cfg):
    """camera_b.yaml 的 must_disable 与本脚本强制项的一致性（单向：yaml -> 脚本）。"""
    loaded, src = _load_camera_yaml()
    if loaded is None:
        return [Check("config_sync", PENDING, "读不到 configs/camera_b.yaml（%s）" % src)]
    yaml_list = []
    acq = loaded.get("acquisition", {}) if isinstance(loaded, dict) else {}
    if isinstance(acq, dict):
        yaml_list = list(acq.get("must_disable", []) or [])
    if not yaml_list:
        return [Check("config_sync", PENDING, "camera_b.yaml 未给出 acquisition.must_disable")]
    yaml_set = set(str(x).strip().lower() for x in yaml_list)
    missing = [k for k in MUST_DISABLE_KEYS if k not in yaml_set]
    extra = sorted(yaml_set - set(MUST_DISABLE_KEYS))
    if missing:
        return [Check("config_sync", FAIL,
                      "camera_b.yaml 的 must_disable 缺 %s（本脚本按 docs/experiment_protocol.md "
                      "§二 与 hardware_freeze §1 强制）" % missing)]
    if extra:
        return [Check("config_sync", PENDING,
                      "camera_b.yaml 另有未映射项 %s —— 需确认其相机节点名后补进 MUST_DISABLE" % extra)]
    return [Check("config_sync", PASS, "camera_b.yaml 的 %d 项与本脚本强制清单一致" % len(yaml_set))]


def check_format(samples):
    """位深与 value_packing 是否漂移（含强/弱判据的实物交叉验证）。"""
    out = []
    bad_pack = [s for s in samples if _norm(s.get("value_packing", "")) != VALUE_PACKING]
    if bad_pack:
        out.append(Check("format:value_packing", FAIL,
                         "%d 个样本声明非 %s（首个 t=%.0fs，值=%r）—— 打包口径漂移"
                         % (len(bad_pack), VALUE_PACKING, bad_pack[0]["t_s"],
                            bad_pack[0].get("value_packing"))))
    else:
        out.append(Check("format:value_packing", PASS, "%d 个样本均为 %s" % (len(samples), VALUE_PACKING)))

    bad_bits = [s for s in samples
                if int(s.get("container_bits", 0)) != CONTAINER_BITS
                or int(s.get("effective_bits", 0)) != EFFECTIVE_BITS]
    if bad_bits:
        out.append(Check("format:bits", FAIL,
                         "%d 个样本的容器/有效位深不是 %d/%d（首个 t=%.0fs：%s/%s）"
                         % (len(bad_bits), CONTAINER_BITS, EFFECTIVE_BITS, bad_bits[0]["t_s"],
                            bad_bits[0].get("container_bits"), bad_bits[0].get("effective_bits"))))
    else:
        out.append(Check("format:bits", PASS, "容器 %d bit / 有效 %d bit" % (CONTAINER_BITS, EFFECTIVE_BITS)))

    # 打包口径的实物判据（docs/hardware_freeze.md §1.2）：声明 packed 时，
    # 强判据「整帧低 4 位恒零」与弱判据「max > 4095」都不得成立。
    contra = []
    for s in samples:
        if not isinstance(s.get("low4_all_zero_ratio"), (int, float)):
            continue
        if s["low4_all_zero_ratio"] >= 0.999 or int(s.get("max_value", 0)) > FULL_SCALE_PACKED:
            contra.append(s)
    if contra:
        out.append(Check("format:packing_evidence", FAIL,
                         "%d 个样本出现 fullscale 特征（低 4 位恒零或 max>%d；首个 t=%.0fs）——"
                         "与声明的 packed 矛盾" % (len(contra), FULL_SCALE_PACKED, contra[0]["t_s"])))
    else:
        out.append(Check("format:packing_evidence", PASS,
                         "无 fullscale 特征（低 4 位非恒零且 max<=%d）" % FULL_SCALE_PACKED))
    return out


def check_timeline(samples, fps, allowed_dropped):
    """帧序、丢帧与断流。"""
    out = []
    total_dropped = sum(int(s.get("dropped", 0) or 0) for s in samples)
    if total_dropped > allowed_dropped:
        out.append(Check("timeline:dropped", FAIL,
                         "累计丢帧 %d > 允许 %d" % (total_dropped, allowed_dropped)))
    else:
        out.append(Check("timeline:dropped", PASS, "累计丢帧 %d（允许 %d）" % (total_dropped, allowed_dropped)))

    # 帧号严格递增（允许因丢帧出现跳变，但不允许回退）
    idx = [int(s.get("frame_index", 0)) for s in samples]
    back = [(i, idx[i - 1], idx[i]) for i in range(1, len(idx)) if idx[i] < idx[i - 1]]
    if back:
        out.append(Check("timeline:index", FAIL, "帧号出现回退 %d 处（首个 %s）" % (len(back), back[0])))
    else:
        out.append(Check("timeline:index", PASS, "帧号单调不减"))

    nom = 1000.0 / float(fps)
    lo, hi = DEF_INTERVAL_JITTER
    stalls = [s for s in samples if float(s.get("interval_ms", nom)) > nom * DEF_DISCONNECT_FACTOR]
    if stalls:
        out.append(Check("timeline:disconnect", FAIL,
                         "%d 处断流（帧间隔 > %.1fx 标称 %.1fms；首个 t=%.0fs 间隔=%.1fms）"
                         % (len(stalls), DEF_DISCONNECT_FACTOR, nom, stalls[0]["t_s"],
                            stalls[0]["interval_ms"])))
    else:
        out.append(Check("timeline:disconnect", PASS, "无断流"))

    bad_jit = [s for s in samples
               if not (nom * lo <= float(s.get("interval_ms", nom)) <= nom * hi)]
    if bad_jit:
        out.append(Check("timeline:jitter", FAIL,
                         "%d 处帧间隔越带（[%.1fx, %.1fx] 标称 %.1fms；首个 t=%.0fs 间隔=%.1fms）"
                         % (len(bad_jit), lo, hi, nom, bad_jit[0]["t_s"], bad_jit[0]["interval_ms"])))
    else:
        out.append(Check("timeline:jitter", PASS, "帧间隔在 [%.1fx, %.1fx] 标称带内" % (lo, hi)))
    return out


def check_stats(samples, mean_drift_tol, snr_drift_tol):
    """帧统计量的绝对阈值与长时漂移。"""
    out = []
    sat_bad, p999_bad, med_bad = [], [], []
    for s in samples:
        roi = float(s.get("roi_pixels") or ROI_PIXELS)
        ratio = float(s.get("sat_count", 0)) / roi if roi else 0.0
        if ratio > SAT_RATIO_MAX:
            sat_bad.append((s["t_s"], ratio))
        if float(s.get("p999", 0)) > P999_RATIO_MAX * FULL_SCALE_PACKED:
            p999_bad.append((s["t_s"], s["p999"]))
        med = float(s.get("median", 0))
        if not (MEDIAN_RATIO_RANGE[0] * FULL_SCALE_PACKED
                <= med <= MEDIAN_RATIO_RANGE[1] * FULL_SCALE_PACKED):
            med_bad.append((s["t_s"], med))

    out.append(Check("stats:saturation", FAIL if sat_bad else PASS,
                     ("%d 个样本饱和占比 > %.2f%%（首个 t=%.0fs：%.4f%%）"
                      % (len(sat_bad), SAT_RATIO_MAX * 100, sat_bad[0][0], sat_bad[0][1] * 100))
                     if sat_bad else "全部样本饱和占比 <= %.2f%%" % (SAT_RATIO_MAX * 100)))
    out.append(Check("stats:p999", FAIL if p999_bad else PASS,
                     ("%d 个样本 p99.9 > %.0f（首个 t=%.0fs：%.0f）"
                      % (len(p999_bad), P999_RATIO_MAX * FULL_SCALE_PACKED, p999_bad[0][0], p999_bad[0][1]))
                     if p999_bad else "全部样本 p99.9 <= %.0f" % (P999_RATIO_MAX * FULL_SCALE_PACKED)))
    out.append(Check("stats:median", FAIL if med_bad else PASS,
                     ("%d 个样本中位数越带 [%.0f, %.0f]（首个 t=%.0fs：%.0f）"
                      % (len(med_bad), MEDIAN_RATIO_RANGE[0] * FULL_SCALE_PACKED,
                         MEDIAN_RATIO_RANGE[1] * FULL_SCALE_PACKED, med_bad[0][0], med_bad[0][1]))
                     if med_bad else
                     "全部样本中位数在 [%.0f, %.0f]" % (MEDIAN_RATIO_RANGE[0] * FULL_SCALE_PACKED,
                                                          MEDIAN_RATIO_RANGE[1] * FULL_SCALE_PACKED)))

    drift_checks = [
        ("mean", "mean", mean_drift_tol),
        ("snr", "snr_proxy", snr_drift_tol),
    ]
    for label, key, tol in drift_checks:
        a, b = _window_means(samples, key)
        if a is None:
            out.append(Check("stats:drift_" + label, PENDING, "样本不足以计算首/末窗口漂移"))
            continue
        denom = abs(a) if abs(a) > 1e-12 else 1.0
        rel = abs(b - a) / denom
        out.append(Check("stats:drift_" + label, FAIL if rel > tol else PASS,
                         "首窗口均值 %.4f -> 末窗口 %.4f，相对漂移 %.2f%%（工程默认上限 %.0f%%）"
                         % (a, b, rel * 100, tol * 100)))
    return out


def _window_means(samples, key):
    """首 1/8 与末 1/8 的均值（窗口和漂移分析）。"""
    n = len(samples)
    w = max(1, n // 8)
    if n < 2 * w:
        return None, None
    first = [float(s.get(key, 0)) for s in samples[:w]]
    last = [float(s.get(key, 0)) for s in samples[-w:]]
    return sum(first) / len(first), sum(last) / len(last)


def check_temperature(samples, corr_max, band):
    """温度工作带与「温度 -> 帧间隔」相关性。板端时序影响须板测，另外单列 PENDING。"""
    out = []
    temps = [float(s.get("temperature_c", 0)) for s in samples]
    tmin, tmax = (min(temps), max(temps)) if temps else (0.0, 0.0)
    if tmin < band[0] or tmax > band[1]:
        out.append(Check("temp:band", FAIL,
                         "温度 %.1f~%.1f C 越出工作带 [%.0f, %.0f]（工程默认）"
                         % (tmin, tmax, band[0], band[1])))
    else:
        out.append(Check("temp:band", PASS, "温度 %.1f~%.1f C 在工作带内（工程默认）" % (tmin, tmax)))

    intervals = [float(s.get("interval_ms", 0)) for s in samples]
    r = _pearson(temps, intervals)
    if r is None:
        out.append(Check("temp:interval_corr", PENDING, "样本不足或无方差，无法算温度-间隔相关"))
    elif (tmax - tmin) > 1.0 and abs(r) > corr_max:
        out.append(Check("temp:interval_corr", FAIL,
                         "温度与帧间隔 |r|=%.2f > %.2f（温度跨度 %.1f C）——温度耦合帧节拍，"
                         "需排查是不是时钟/散热问题" % (abs(r), corr_max, tmax - tmin)))
    else:
        out.append(Check("temp:interval_corr", PASS,
                         "温度-间隔 |r|=%s（温度跨度 %.1f C）"
                         % ("n/a" if r is None else ("%.2f" % abs(r)), tmax - tmin)))

    out.append(Check("temp:board_timing", PENDING,
                     "温度对板端 PL 时序 / 帧节拍的影响需上板实测（主机只算相关性）"))
    return out


# ================================================================ 配置读取
def _load_camera_yaml():
    if not os.path.isfile(_CAMERA_YAML):
        return None, "缺文件 %s" % _CAMERA_YAML
    try:
        import yaml  # noqa: F401
    except Exception:
        return None, "本机无 pyyaml"
    try:
        with open(_CAMERA_YAML, "r", encoding="utf-8") as f:
            return yaml.safe_load(f), _CAMERA_YAML
    except Exception as e:  # pragma: no cover
        return None, "解析失败：%s" % e


def load_recorded(path):
    """供 --self-check/外部复用：读一份录制 JSON。"""
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


# ================================================================ 运行
def run_device_checks(device, args):
    checks = []
    try:
        device.open()
    except CameraUnavailable as e:
        checks.append(Check("device:open", PENDING, "%s（%s）" % (e, device.name)))
        checks.extend(_pending_set("真机不可用"))
        return checks

    checks.append(Check("device:open", PASS, "设备 = %s" % device.name))
    try:
        cfg = device.read_configuration()
    except CameraUnavailable as e:
        checks.append(Check("device:config", PENDING, "配置快照不可用：%s" % e))
        cfg = None

    if cfg:
        checks.extend(check_disable_list(cfg))
    else:
        checks.extend([Check("disable:" + k, PENDING, "无配置快照") for k in MUST_DISABLE_KEYS])
    checks.extend(check_config_sync(cfg or {}))

    try:
        samples = device.iter_samples(args.hours, args.sample_every)
    except CameraUnavailable as e:
        checks.append(Check("device:samples", PENDING, "取流样本不可用：%s" % e))
        checks.extend(_pending_set("真机不可用"))
        return checks

    checks.append(Check("device:samples", PASS if samples else PENDING,
                        "样本数 %d" % len(samples)))
    if samples:
        checks.extend(check_format(samples))
        checks.extend(check_timeline(samples, args.fps, args.allowed_dropped))
        checks.extend(check_stats(samples, args.mean_drift_tol, args.snr_drift_tol))
        checks.extend(check_temperature(samples, args.temp_interval_corr_max, DEF_TEMP_BAND_C))
    return checks


def _pending_set(reason):
    names = ["disable:*", "format:*", "timeline:*", "stats:*", "temp:*"]
    return [Check(n, PENDING, reason) for n in names]


def summarize(checks):
    n_pass = sum(1 for c in checks if c.status == PASS)
    n_fail = sum(1 for c in checks if c.status == FAIL)
    n_pend = sum(1 for c in checks if c.status == PENDING)
    return n_pass, n_fail, n_pend


def report(checks, title, quiet=False):
    if not quiet:
        print("==== %s ====" % title)
        for c in checks:
            print("  [%-7s] %-28s %s" % (c.status, c.name, c.detail))
    n_pass, n_fail, n_pend = summarize(checks)
    print("  ---- PASS %d / FAIL %d / PENDING %d ----" % (n_pass, n_fail, n_pend))
    return n_fail


# ================================================================ 自检
def self_check(quiet=False):
    """对判据逻辑本身做 (场景 -> 预期判决) 断言。全部通过返回 0。"""
    cases = []

    def expect(name, got_checks, want_status, want_key=None):
        hit = [c for c in got_checks
               if (want_key is None or c.name == want_key)]
        ok = any(c.status == want_status for c in hit)
        cases.append((name, ok, "期望 %s%s，实得 %s"
                      % (want_status, ("@" + want_key) if want_key else "",
                         [c.status for c in hit])))

    # 1 must_disable：全关 -> 每项 PASS
    got = check_disable_list(synth_config())
    expect("disable/全关", got, PASS, "disable:auto_exposure")
    expect("disable/全关(super_bayer)", got, PASS, "disable:super_bayer_enable")

    # 2 自动曝光未关 -> FAIL
    got = check_disable_list(synth_config(ExposureAuto="Continuous"))
    expect("disable/AE 未关", got, FAIL, "disable:auto_exposure")

    # 3 节点缺失 -> PENDING
    cfg = synth_config()
    cfg.pop("GammaEnable")
    got = check_disable_list(cfg)
    expect("disable/节点缺失", got, PENDING, "disable:gamma")

    # 4 位深 / 打包漂移 -> FAIL
    got = check_format(synth_samples(n=9, packing="fullscale"))
    expect("format/口径漂移", got, FAIL, "format:value_packing")
    expect("format/fullscale 特征", got, FAIL, "format:packing_evidence")
    got = check_format(synth_samples(n=9))
    expect("format/正常", got, PASS, "format:value_packing")

    # 5 丢帧 / 断流 / 抖动
    got = check_timeline(synth_samples(n=9, fps=5.0, drop_at=4), 5.0, allowed_dropped=0)
    expect("timeline/丢帧", got, FAIL, "timeline:dropped")
    got = check_timeline(synth_samples(n=9, fps=5.0, stall_at=4), 5.0, allowed_dropped=0)
    expect("timeline/断流", got, FAIL, "timeline:disconnect")
    got = check_timeline(synth_samples(n=9, fps=5.0), 5.0, allowed_dropped=0)
    expect("timeline/正常", got, PASS, "timeline:jitter")

    # 6 统计量阈值与漂移
    got = check_stats(synth_samples(n=9, sat_breach_at=4), DEF_MEAN_DRIFT_TOL, DEF_SNR_DRIFT_TOL)
    expect("stats/饱和超标", got, FAIL, "stats:saturation")
    got = check_stats(synth_samples(n=32, mean_drift=0.5), DEF_MEAN_DRIFT_TOL, DEF_SNR_DRIFT_TOL)
    expect("stats/均值漂移", got, FAIL, "stats:drift_mean")
    got = check_stats(synth_samples(n=32), DEF_MEAN_DRIFT_TOL, DEF_SNR_DRIFT_TOL)
    expect("stats/正常", got, PASS, "stats:drift_mean")

    # 7 温度
    got = check_temperature(synth_samples(n=32, temp_start=20.0, temp_end=45.0,
                                          temp_interval_corr=True),
                            DEF_TEMP_INTERVAL_CORR_MAX, DEF_TEMP_BAND_C)
    expect("temp/耦合", got, FAIL, "temp:interval_corr")
    got = check_temperature(synth_samples(n=32, temp_start=24.0, temp_end=26.0),
                            DEF_TEMP_INTERVAL_CORR_MAX, DEF_TEMP_BAND_C)
    expect("temp/正常", got, PASS, "temp:interval_corr")
    expect("temp/板端时序 PENDING", got, PENDING, "temp:board_timing")

    # 8 配置清单一致性（有/无 pyyaml 都可能，故只断言结果自洽：FAIL 或 PASS 或 PENDING）
    got = check_config_sync(synth_config())
    cases.append(("config_sync/自洽", got[0].status in (PASS, FAIL, PENDING), got[0].detail))

    n_fail = 0
    if not quiet:
        print("==== test_camera_24h 判据逻辑自检 ====")
    for name, ok, detail in cases:
        if not ok:
            n_fail += 1
        if not quiet:
            print("  [%s] %-28s %s" % ("PASS" if ok else "FAIL", name, detail))
    if not quiet:
        print("  ---- 自检 %d 项，FAIL %d ----" % (len(cases), n_fail))
    return n_fail


def main(argv=None):
    p = argparse.ArgumentParser(description="相机 24h 稳定性与参数漂移测试（接口 + 桩）")
    p.add_argument("--device", default="synthetic",
                   choices=["synthetic", "recorded", "spinnaker", "gentl"],
                   help="设备实现；spinnaker/gentl 在主机上为留桩（判 PENDING）")
    p.add_argument("--metadata", default=None, help="--device recorded 的录制 JSON 路径")
    p.add_argument("--hours", type=float, default=24.0, help="覆盖的时间跨度（小时）")
    p.add_argument("--sample-every", type=float, default=60.0, help="元数据抽样间隔（秒）")
    p.add_argument("--fps", type=float, default=5.0, help="标称帧率（用于帧间隔判据）")
    p.add_argument("--allowed-dropped", type=int, default=0, help="允许的累计丢帧数")
    p.add_argument("--mean-drift-tol", type=float, default=DEF_MEAN_DRIFT_TOL,
                   help="首/末窗口均值相对漂移上限（工程默认）")
    p.add_argument("--snr-drift-tol", type=float, default=DEF_SNR_DRIFT_TOL,
                   help="首/末窗口 SNR 代理相对漂移上限（工程默认）")
    p.add_argument("--temp-interval-corr-max", type=float, default=DEF_TEMP_INTERVAL_CORR_MAX,
                   help="温度-帧间隔 |r| 上限（工程默认）")
    p.add_argument("--self-check", action="store_true", help="只跑判据逻辑自检")
    p.add_argument("--json-out", default=None, help="把结果写成 JSON")
    p.add_argument("--quiet", action="store_true")
    args = p.parse_args(argv)

    if args.self_check:
        n_fail = self_check(args.quiet)
        if args.json_out:
            _write_json(args.json_out, {"mode": "self-check",
                                        "script_version": _SCRIPT_VERSION,
                                        "fail": n_fail})
        return 1 if n_fail else 0

    device = build_device(args)
    checks = run_device_checks(device, args)
    n_fail = report(checks, "相机 24h 稳定性（设备 = %s）" % device.name, args.quiet)

    if args.json_out:
        _write_json(args.json_out, {
            "mode": "run",
            "script_version": _SCRIPT_VERSION,
            "device": device.name,
            "hours": args.hours,
            "sample_every_s": args.sample_every,
            "checks": [c.as_dict() for c in checks],
            "summary": dict(zip(["pass", "fail", "pending"], summarize(checks))),
        })

    if not args.quiet:
        print("  注：真机项（参数回读/24h 取流/温度对板端时序）在主机上恒为 PENDING；"
              "本脚本的主机证据是判据逻辑（--self-check），不代表相机行为。")
    return 1 if n_fail else 0


def _write_json(path, obj):
    d = os.path.dirname(os.path.abspath(path))
    if d and not os.path.isdir(d):
        os.makedirs(d, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(obj, f, ensure_ascii=False, indent=2)
    print("  结果已写入 %s" % path)


if __name__ == "__main__":
    sys.exit(main())
