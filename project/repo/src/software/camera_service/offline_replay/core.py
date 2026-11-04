"""Host-only replay of the two pinned L02 captures; no camera or reconstruction."""
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from io import BytesIO
import hashlib
import importlib.util
import json
import time

import numpy as np
from PIL import Image, ImageDraw


class ReplayError(ValueError):
    pass


def require(ok, message):
    if not ok:
        raise ReplayError(message)


def sha(data):
    return hashlib.sha256(data).hexdigest()


def now():
    return datetime.now().astimezone().isoformat()


def read_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def new_json(path, value):
    with Path(path).open('x', encoding='utf8', newline='\n') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')


def within(path, root):
    path, root = Path(path).resolve(), Path(root).resolve()
    return path == root or root in path.parents


def workspace_path(workspace, relative):
    require(isinstance(relative, str) and not Path(relative).is_absolute(), 'Expected a workspace-relative path')
    path = (workspace / relative).resolve()
    require(within(path, workspace), 'Input path escapes workspace')
    return path


def pinned_bytes(workspace, record):
    path = workspace_path(workspace, record['workspace_path'])
    require(path.is_file(), 'Missing input: ' + str(path))
    if 'bytes' in record:
        require(path.stat().st_size == record['bytes'], 'Input length changed: ' + str(path))
    data = path.read_bytes()
    require(sha(data) == record['sha256'], 'Input hash changed: ' + str(path))
    return data


@dataclass(frozen=True)
class Frame:
    sample_id: str
    raw: bytes
    pixels: np.ndarray
    preview_rgb: np.ndarray
    metadata_bytes: bytes
    identity: dict
    validation_ms: float


class Catalog:
    """Immutable input cache, bounded by the two registered single-frame samples."""
    def __init__(self, workspace, lock_path=None):
        self.workspace = Path(workspace).resolve()
        lock_path = Path(lock_path) if lock_path else Path(__file__).with_name('input_lock.json')
        self.lock = read_json(lock_path)
        require(self.lock.get('schema_version') == 1, 'Unknown input-lock version')
        self.manifest = json.loads(pinned_bytes(self.workspace, self.lock['manifest']))
        contract = json.loads(pinned_bytes(self.workspace, self.lock['contract']))
        require(self.manifest.get('schema_version') == contract.get('schema_version') == 1, 'Unknown manifest version')
        self.rows = {row['sample_id']: row for row in self.manifest['samples']}
        self.contracts = {row['id']: row for row in contract['samples']}
        require(len(self.rows) == len(self.manifest['samples']) == 2 and self.rows.keys() == self.contracts.keys(), 'Expected the two L02 samples')
        self.default = self.manifest['default_sample_id']
        require(self.default in self.rows, 'Unknown default sample')
        # Execute only the locally pinned L02 validator source; no capture SDK imports.
        code = pinned_bytes(self.workspace, self.lock['validator'])
        self.validator = importlib.util.module_from_spec(importlib.util.spec_from_loader('_l02_raw_validator', loader=None))
        exec(compile(code, self.lock['validator']['workspace_path'], 'exec'), self.validator.__dict__)
        self.cache = {}

    def load(self, sample_id):
        require(sample_id in self.rows, 'Unknown sample: ' + str(sample_id))
        if sample_id in self.cache:
            return self.cache[sample_id]
        start = time.perf_counter()
        row, contract = self.rows[sample_id], self.contracts[sample_id]
        require(row['allow_offline_single_frame'] and not row['allow_true_temporal_sequence'] and not row['allow_direct_nlos_reconstruction'], 'Unsupported input scope')
        files = {role: pinned_bytes(self.workspace, record) for role, record in contract['files'].items()}
        meta, active, state = [json.loads(files[key]) for key in ['metadata', 'camera_active', 'state']]
        pixels, _ = self.validator.decode_checked(files['raw'], meta, active, state)
        require(row['raw'] == contract['files']['raw'], 'Manifest RAW identity differs')
        require((row['format']['width'], row['format']['height']) == (meta['width'], meta['height']), 'Manifest layout differs')
        stride = contract['archival_preview_sensor_stride']
        require(stride in (2, 4) and meta['width'] % stride == meta['height'] % stride == 0, 'Unsupported preview stride')
        rgb = np.stack((pixels[0::stride, 0::stride],
                        (pixels[0::stride, 1::stride].astype(np.uint32) + pixels[1::stride, 0::stride]) * .5,
                        pixels[1::stride, 1::stride]), axis=2)
        low, high = np.percentile(rgb, [.5, 99.5])
        scaled = np.clip((rgb - low) * 255 / max(high - low, 64.), 0, 255).astype(np.uint8)
        # Backed by immutable bytes, so display cannot write through to RAW or cache.
        preview = np.frombuffer(scaled.tobytes(), np.uint8).reshape(scaled.shape)
        with Image.open(BytesIO(files['lossless_png'])) as image:
            require(np.array_equal(pixels, np.asarray(image)), 'Lossless reference differs')
        with Image.open(BytesIO(files['archival_preview'])) as image:
            require(np.array_equal(preview, np.asarray(image.convert('RGB'))), 'Historical preview differs')
        identity = {'sample_id': sample_id, 'source_raw': row['raw'], 'format': row['format'],
                    'original_capture': row['capture'], 'geometry': row['geometry'],
                    'historical_session_state': row['historical_session_state'],
                    'warnings': row['warnings'], 'source_type': row['source_type'],
                    'is_replay': True, 'is_reconstruction': False, 'motion_sequence': False,
                    'display_only_stretch': [float(low), float(high)], 'preview_sensor_stride': stride}
        frame = Frame(sample_id, files['raw'], pixels, preview, files['metadata'], identity,
                      (time.perf_counter() - start) * 1000)
        self.cache[sample_id] = frame
        return frame


class Recorder:
    def __init__(self, output, catalog, options):
        self.output = Path(output).resolve()
        require(not self.output.exists(), 'Output already exists; choose a new directory')
        if within(self.output, catalog.workspace):
            relative = self.output.relative_to(catalog.workspace)
            require(len(relative.parts) >= 2 and relative.parts[0] == '99_临时工作区',
                    'Workspace outputs must be new subdirectories of 99_临时工作区')
        for contract in catalog.contracts.values():
            for record in contract['files'].values():
                source = workspace_path(catalog.workspace, record['workspace_path'])
                require(not within(self.output, source.parent), 'Output overlaps input directory')
        self.output.mkdir(parents=True, exist_ok=False)
        self.stream = (self.output / 'events.jsonl').open('x', encoding='utf8', newline='\n')
        self.started = time.perf_counter()
        self.closed = False
        self.saves = 0
        self.save_attempts = 0
        self.options = options
        new_json(self.output / 'session.json', {'schema_version': 1, 'started_at': now(), 'options': options,
                 'workspace': str(catalog.workspace), 'input_lock': catalog.lock, 'mode': 'OFFLINE_SINGLE_FRAME_REPLAY',
                 'clock_scope': 'Host wall/monotonic times only; never camera timing or board performance',
                 'hardware_accessed': False, 'algorithm_executed': False})
        self.event('SESSION_STARTED', options=options)

    def event(self, kind, **data):
        require(not self.closed, 'Session is closed')
        row = {'event': kind, 'recorded_at': now(), 'host_elapsed_ms': (time.perf_counter() - self.started) * 1000, **data}
        self.stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + '\n')
        self.stream.flush()

    def save(self, frame, index):
        require(index > 0, 'Display or emit a frame before saving')
        self.save_attempts += 1
        target = self.output / 'saved' / ('snapshot_%04d' % self.save_attempts)
        target.mkdir(parents=True, exist_ok=False)
        # snapshot.json is written last: an interrupted write is never a complete snapshot.
        with (target / 'frame.raw').open('xb') as stream:
            stream.write(frame.raw)
        with (target / 'raw_metadata.json').open('xb') as stream:
            stream.write(frame.metadata_bytes)
        with (target / 'raw_bayer16.png').open('xb') as stream:
            Image.fromarray(frame.pixels).save(stream, format='PNG')
        image = Image.fromarray(frame.preview_rgb)
        banner = Image.new('RGB', (max(image.width, 600), image.height + 54), '#142738')
        banner.paste(image, (0, 54))
        draw = ImageDraw.Draw(banner)
        draw.text((12, 8), 'REPLAY | Historical single frame | NOT A RECONSTRUCTION', fill='white')
        draw.text((12, 29), '%s | Replay index %d' % (frame.sample_id, index), fill='#a6d8ee')
        with (target / 'preview.png').open('xb') as stream:
            banner.save(stream, format='PNG')
        files = {p.name: {'bytes': p.stat().st_size, 'sha256': sha(p.read_bytes())} for p in target.iterdir()}
        new_json(target / 'snapshot.json', {'status': 'COMPLETE', 'saved_at': now(), 'replay_index': index,
                 'identity': frame.identity, 'display_only': True, 'files': files})
        self.saves += 1
        self.event('SAVED', sample_id=frame.sample_id, replay_index=index, path=target.relative_to(self.output).as_posix())
        return target

    def finish(self, state, reason, emitted, displayed, error=None):
        if self.closed:
            return
        summary = {'status': state, 'reason': reason, 'finished_at': now(), 'frames_emitted': emitted,
                   'frames_displayed': displayed, 'snapshots_completed': self.saves,
                   'elapsed_host_ms': (time.perf_counter() - self.started) * 1000,
                   'hardware_accessed': False, 'algorithm_executed': False, 'error': error}
        self.event('SESSION_FINISHED', **summary)
        self.stream.close()
        self.closed = True
        new_json(self.output / 'summary.json', summary)


class Session:
    def __init__(self, catalog, recorder, frame):
        self.catalog, self.recorder, self.frame = catalog, recorder, frame
        self.index, self.displayed = 0, 0
        self.state, self.error, self.last_index = 'READY', None, None
        self.recorder.event('SAMPLE_SELECTED', identity=frame.identity, validation_host_ms=frame.validation_ms)

    def select(self, sample_id):
        require(self.state != 'CLOSED', 'Session is closed')
        self.stop('sample_change')
        self.frame, self.last_index = None, None
        try:
            self.frame = self.catalog.load(sample_id)
            self.state, self.error = 'READY', None
            self.recorder.event('SAMPLE_SELECTED', identity=self.frame.identity, validation_host_ms=self.frame.validation_ms)
        except Exception as exc:
            self.fail(exc)
            raise

    def start(self):
        require(self.frame is not None and self.state not in ('CLOSED', 'ERROR'), 'No valid input available')
        if self.state != 'RUNNING':
            self.state = 'RUNNING'
            self.recorder.event('PLAYBACK_STARTED', sample_id=self.frame.sample_id)

    def emit(self, displayed=False):
        require(self.frame is not None and self.state not in ('CLOSED', 'ERROR'), 'No valid input available')
        self.index += 1
        self.last_index = self.index
        self.displayed += int(displayed)
        self.recorder.event('FRAME', sample_id=self.frame.sample_id, replay_index=self.index,
                            displayed=displayed, original_frame_number=self.frame.identity['original_capture']['frame_number'],
                            original_timestamp_ticks=self.frame.identity['original_capture']['camera_timestamp_ticks'],
                            source_raw_sha256=self.frame.identity['source_raw']['sha256'], is_replay=True)
        return self.frame

    def save(self):
        require(self.frame is not None and self.last_index is not None and self.state not in ('CLOSED','ERROR'), 'No current frame to save')
        return self.recorder.save(self.frame, self.last_index)

    def stop(self, reason='user_stop'):
        if self.state not in ('CLOSED', 'ERROR'):
            self.state = 'STOPPED'
            self.recorder.event('PLAYBACK_STOPPED', reason=reason, replay_index=self.index)

    def fail(self, error):
        self.state, self.error = 'ERROR', str(error)
        self.recorder.event('ERROR', error=self.error)

    def close(self, reason='closed'):
        if self.state == 'CLOSED':
            return
        status = 'ERROR' if self.error else 'COMPLETE'
        self.recorder.finish(status, reason, self.index, self.displayed, self.error)
        self.state = 'CLOSED'
