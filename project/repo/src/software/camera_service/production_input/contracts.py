"""Checked immutable input contracts for the offline full-model pipeline."""
from dataclasses import dataclass
from pathlib import Path
import hashlib
import json
import struct
import time
import uuid
import zlib

import numpy as np

HEADER = struct.Struct('<8sIIIIQII16s8s')
MODEL_BYTES = 59332680
PFY_BYTES = 381112
M = 126 * 126
N = 29 * 36


class InputError(ValueError):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


def require(condition, code, message):
    if not condition:
        raise InputError(code, message)


def sha(data):
    return hashlib.sha256(data).hexdigest()


def read_json(path):
    def invalid(value):
        raise InputError('NONFINITE_JSON', value)
    return json.loads(Path(path).read_text('utf-8-sig'), parse_constant=invalid)


def write_json(path, value):
    with Path(path).open('x', encoding='utf8', newline='\n') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')


def resolve(base, path):
    return (Path(base) / path).resolve()


def pinned(base, record, max_bytes=128 * 1024 * 1024):
    path = resolve(base, record['path'])
    require(type(record['bytes']) is int and 0 < record['bytes'] <= max_bytes,
            'FILE_SIZE', 'Invalid or unsupported registered length')
    require(path.stat().st_size == record['bytes'], 'FILE_SIZE', str(path))
    data = path.read_bytes()
    require(len(data) == record['bytes'] and sha(data) == record['sha256'], 'FILE_HASH', str(path))
    return data


def blob(data, kind, shape, expected_bytes, model_id=None):
    require(len(data) == expected_bytes and len(data) >= HEADER.size, 'BLOB_SIZE', 'Wrong binary length')
    magic, version, k, h, w, size, crc, reserved, mid, tail = HEADER.unpack_from(data)
    require((magic, version, k, h, w) == (b'PSFULL1\0', 1, kind, *shape), 'BLOB_HEADER', 'Wrong binary contract')
    require(size == len(data)-64 and reserved == 0 and tail == bytes(8), 'BLOB_HEADER', 'Length/reserved mismatch')
    require(zlib.crc32(memoryview(data)[64:]) == crc, 'BLOB_CRC', 'Payload CRC mismatch')
    require(model_id is None or mid.hex() == model_id, 'MODEL_ID', 'Input/model identity mismatch')
    return mid.hex(), memoryview(data)[64:]


def encode_pfy(rgb, scales, model_id):
    rgb = np.asarray(rgb, dtype=np.float64)
    scales = np.asarray(scales, dtype=np.float64)
    require(rgb.shape == (126, 126, 3) and scales.shape == (3,), 'MEASUREMENT_SHAPE', 'Expected full 126x126x3 RGB')
    validate_measurement(rgb, scales)
    payload = scales.astype('<f8').tobytes() + rgb.reshape(-1, 3, order='F').astype('<f8').tobytes()
    return HEADER.pack(b'PSFULL1\0', 1, 2, 126, 126, len(payload), zlib.crc32(payload), 0,
                       bytes.fromhex(model_id), bytes(8)) + payload


def validate_measurement(rgb, scales):
    require(np.isfinite(rgb).all() and np.isfinite(scales).all(), 'NONFINITE', 'Nonfinite measurement')
    require(np.all(rgb >= 0) and np.all(scales > 0), 'MEASUREMENT_RANGE', 'Negative measurement or nonpositive scales')
    cc = scales / scales.max()
    require(np.all(cc > 0), 'SCALE_UNDERFLOW', 'RGB scale underflow')
    flat = rgb.reshape(-1, 3, order='F')
    author = np.concatenate((flat[:-1]-flat[1:], flat[-1:]), axis=0)
    ds = float(np.max(np.abs(author)))
    with np.errstate(over='ignore', invalid='ignore'):
        energy = float(np.sum(rgb * rgb))
        diff_energy = float((np.sum(np.diff(rgb, axis=0)**2)+np.sum(np.diff(rgb, axis=1)**2))*.5)
    require(np.isfinite(energy) and np.isfinite(diff_energy) and np.isfinite(ds), 'MEASUREMENT_RANGE', 'Measurement overflow')
    require(ds > 0 and np.sqrt(diff_energy) > 1e-12*max(1., np.sqrt(energy)),
            'NO_SPATIAL_VARIATION', 'No useful spatial variation for the complete solver')
    return ds


def decode_pfy(data, model_id=None):
    mid, payload = blob(data, 2, (126, 126), PFY_BYTES, model_id)
    values = np.frombuffer(payload, dtype='<f8')
    scales = values[:3]
    # Each MATLAB-ordered pixel carries an interleaved RGB triplet on disk.
    rgb = values[3:].reshape(M, 3).reshape(126, 126, 3, order='F')
    validate_measurement(rgb, scales)
    return mid, scales, rgb


def u64(value):
    require(value is None or (type(value) is int and 0 <= value < 2**64), 'TIME_ID', 'Expected unsigned integer or null')
    return None if value is None else str(value)


@dataclass(frozen=True)
class Frame:
    kind: str
    data: bytes
    envelope_json: str

    @property
    def envelope(self):
        return json.loads(self.envelope_json)


class FileSource:
    """One loaded immutable frame at a time; no camera SDK or background thread."""
    def __init__(self, session_id=None):
        self.session_id = session_id or str(uuid.uuid4())
        self.index = 0

    def load(self, descriptor, base):
        require(descriptor['schema_version'] == 1, 'SCHEMA', 'Unsupported input schema')
        kind = descriptor['kind']
        require(kind in ('PFY', 'RAW'), 'FORMAT', 'Only PFY and unpacked BayerRG12 are supported')
        require(descriptor['complete'] is True, 'INCOMPLETE_FRAME', 'Incomplete frame')
        data = pinned(base, descriptor['file'])
        capture = descriptor.get('capture', {})
        if kind == 'PFY':
            mid, _, _ = decode_pfy(data, descriptor['model_id'])
            fmt = {'version': 1, 'name': 'PSFULL1_PFY', 'shape': [126,126,3], 'model_id': mid}
        else:
            fmt = descriptor['format']
            self.decode_raw(data, fmt)
            require(descriptor.get('origin') in ('HISTORICAL_REAL_CAMERA_SINGLE_FRAME', 'SYNTHETIC'),
                    'SOURCE_KIND', 'Register whether RAW is historical camera data or synthetic')
            if descriptor['origin'] != 'SYNTHETIC':
                meta = json.loads(pinned(base, descriptor['metadata']))
                require(meta['complete'] is True and meta['sha256'] == sha(data) and meta['bytes'] == len(data),
                        'RAW_SIDECAR', 'RAW metadata identity/integrity mismatch')
                require((meta['width'], meta['height'], meta['pixel_format'], meta['raw_byte_order']) ==
                        (fmt['width'], fmt['height'], fmt['pixel_format'], fmt['dtype']),
                        'RAW_SIDECAR', 'RAW format differs from capture metadata')
                for field, key in [('original_sensor_frame_id','number'),('original_block_id','block_id'),('camera_timestamp_ticks','timestamp_ticks')]:
                    require(capture.get(field) == meta.get(key), 'RAW_SIDECAR', 'Original sensor identity differs')
        for name in ('exposure_us', 'camera_tick_frequency_hz'):
            value = capture.get(name)
            require(value is None or (type(value) in (int,float) and np.isfinite(value) and value > 0), 'CAPTURE_METADATA', name)
        self.index += 1
        envelope = dict(schema_version=1, source_kind='SYNTHETIC' if descriptor.get('origin') == 'SYNTHETIC' else 'REPLAY_SINGLE',
            source_session_id=self.session_id, frame_uid=f'{self.session_id}:{self.index}', replay_index=str(self.index),
            original_capture_session_id=capture.get('session_id'),
            original_sensor_frame_id=u64(capture.get('original_sensor_frame_id')),
            original_block_id=u64(capture.get('original_block_id')),
            camera_timestamp_ticks=u64(capture.get('camera_timestamp_ticks')),
            camera_tick_frequency_hz=capture.get('camera_tick_frequency_hz'), camera_clock_mapping=None,
            clock_domain='HOST_MONOTONIC:'+self.session_id, t_frame_complete_mono_ns=str(time.monotonic_ns()),
            format=fmt, exposure_us=capture.get('exposure_us'), gain_raw=capture.get('gain_raw'),
            gain_units=capture.get('gain_units'), complete=True, raw_sha256=sha(data), hash_state='VERIFIED',
            source_path=str(resolve(base, descriptor['file']['path'])), sample_id=descriptor['sample_id'],
            unknown_time_reason='Capture clock mapping unavailable; host replay time is not camera time',
            is_true_motion_sequence=False, hardware_accessed=False)
        if kind == 'RAW' and descriptor.get('origin') != 'SYNTHETIC':
            envelope['camera_identity'] = descriptor.get('camera_identity')
            envelope['sensor_test_pattern_off'] = meta.get('sensor_test_pattern_off') is True
        return Frame(kind, data, json.dumps(envelope, allow_nan=False))

    @staticmethod
    def decode_raw(data, fmt):
        require((fmt.get('pixel_format'),fmt.get('dtype'),fmt.get('bayer_pattern'),fmt.get('valid_bits')) ==
                ('BayerRG12','<u2','RGGB',12), 'RAW_FORMAT', 'Only unpacked low-12-bit RGGB uint16 is supported')
        h, w = fmt['height'], fmt['width']
        require(type(h) is int and type(w) is int and 0 < h <= 8192 and 0 < w <= 8192 and h%2 == w%2 == 0,
                'RAW_DIMENSIONS', 'Positive even bounded RAW dimensions required')
        require(fmt['row_stride_bytes'] == w*2 and len(data) == h*w*2, 'RAW_SIZE', 'No packed/padded/truncated RAW supported')
        raw = np.frombuffer(data, '<u2').reshape(h,w)
        require(raw.max() <= 4095, 'RAW_BITS', 'Values exceed low 12 bits')
        return raw


def load_model(descriptor, base):
    data = pinned(base, descriptor['model'])
    mid, _ = blob(data, 1, (29,36), MODEL_BYTES, descriptor['model_id'])
    return data, mid
