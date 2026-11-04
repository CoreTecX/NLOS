"""Exercise the actual Tk widgets and frozen CPU backend without hardware I/O."""
from pathlib import Path
import argparse, hashlib, json, os, sys, time, traceback

REPO = Path(__file__).resolve().parents[2]
for key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[key] = '1'
sys.path.insert(0, str(REPO / 'src/software/ui/offline_console'))
from app import make_root, ConsoleWindow
from contracts import read_json, write_json


def verify(output):
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    root = make_root()
    # Tk still creates real widgets and PhotoImages; physical display is not measured.
    root.withdraw()
    cfg = read_json(REPO / 'configs/offline_engineering.json')
    window = ConsoleWindow(root, cfg['workspace_hint'], output / 'runs')
    report = dict(scope='WINDOWS_TK_FROZEN_CPU_D11_SYNTHETIC_TIME',
                  hardware_accessed=False, camera_connected=False,
                  physical_display_latency_measured=False, passed=False,
                  python=sys.version, checks={})

    def until(predicate, timeout=180):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            root.update()
            if predicate():
                return
            if window.engine and window.engine.finished.is_set() and window.engine.failed:
                raise RuntimeError('Engine failed: ' + str(window.engine.error))
            time.sleep(.01)
        raise TimeoutError('GUI acceptance deadline')

    try:
        window.source_choice.set(next(k for k,v in window.SOURCES.items() if v == 'd11'))
        window.start()
        until(lambda: window.engine.processed >= 6 and window.last_snapshot.get('fast', {}).get('valid'))
        engine = window.engine
        report['checks']['preview_photo_created'] = 'preview' in window.photos
        window.command('IMAGE')
        until(lambda: engine.controller.future is not None)
        future = engine.controller.future
        before = engine.processed
        window.command('MODE', strategy='IMAGE')
        window.command('MODE', strategy='FAST')
        until(lambda: engine.controller.policy_epoch >= 2)
        assert engine.controller.future is future, 'Mode switch replaced running solve'
        report['checks']['mode_switch_preserves_solve'] = True
        until(lambda: engine.latest_result is not None and 'image' in window.photos)
        result = engine.latest_result
        assert result['valid_numeric'] and result['output']['reference_acceptance'] is True
        assert result['publish_allowed']
        assert engine.processed > before + 2, 'FAST did not continue while solving'
        report['checks']['fast_continues_during_complete_solve'] = True
        report['fast_updates_during_solve'] = engine.processed - before
        report['checks']['full_cpu_reference_and_state_accepted'] = True
        report['checks']['reconstruction_photo_created'] = bool(window.image_widget.cget('image'))
        report['result'] = result
        report['image_caption'] = window.image_caption.get()
        assert window.presenter.history is not None
        window.show_history()
        assert window.history_view and '历史结果' in window.image_caption.get()
        report['checks']['history_label_and_image'] = True
        window.command('SAVE')
        until(lambda: engine.saver.saved == 1)
        snapshots = list((engine.output / 'saved').glob('*/snapshot.json'))
        assert len(snapshots) == 1
        snap = read_json(snapshots[0])
        for name, rec in snap['files'].items():
            data = (snapshots[0].parent / name).read_bytes()
            assert len(data) == rec['bytes'] and hashlib.sha256(data).hexdigest() == rec['sha256']
        assert 'image_reconstruction_native.png' in snap['files']
        assert snap['metadata']['reconstruction']['request_id'] == result['request_id']
        report['checks']['snapshot_content_hashes_and_provenance'] = True
        # Inspect actual rendered PhotoImage dimensions, independently of result metadata.
        report['tk_images'] = {k: dict(width=v.width(), height=v.height()) for k,v in window.photos.items()}
        window.stop()
        until(engine.finished.is_set, timeout=30)
        until(lambda: window.last_snapshot.get('state', {}).get('phase') == 'CLOSED', timeout=10)
        assert not engine.failed and engine.summary['closed']
        assert engine.summary['save']['drained'] and engine.summary['telemetry']['audit_complete']
        assert not window.last_snapshot['fast']['valid']
        report['checks']['stop_drains_and_invalidates_fast'] = True
        report['checks']['logs_complete'] = True
        report['console_summary'] = str(engine.output / 'console_summary.json')
        report['passed'] = all(report['checks'].values())
        assert report['passed']
    except Exception:
        report['error'] = traceback.format_exc()
        raise
    finally:
        if window.engine and not window.engine.finished.is_set():
            window.stop()
            until(window.engine.finished.is_set, timeout=180)
        write_json(output / 'acceptance.json', report)
        window.close()
    print(json.dumps(dict(passed=report['passed'], checks=report['checks'], output=str(output))))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', required=True, type=Path)
    verify(p.parse_args().output)
