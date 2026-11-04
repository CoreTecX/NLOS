"""PenumbraSight host offline RAW replay entry. No hardware access."""
from datetime import datetime
from pathlib import Path
import argparse
import json
import sys
import threading
import time
import uuid

from core import Catalog, Recorder, Session, ReplayError


def default_workspace():
    for parent in Path(__file__).resolve().parents:
        if (parent/'02_代码仓库').is_dir() and (parent/'03_任务产出').is_dir():
            return parent
    return None


def run_headless(session, mode, frames, interval_ms, save=False, stop_file=None):
    limit = 1 if mode == 'once' else (10 if frames is None else frames)
    stop_file = Path(stop_file) if stop_file else session.recorder.output / 'STOP'
    emitted, reason, deadline = 0, 'frame_limit', time.perf_counter()
    wait = threading.Event()
    session.start()
    try:
        while not limit or emitted < limit:
            if stop_file.exists(): reason = 'stop_file'; break
            remaining = deadline - time.perf_counter()
            if remaining > 0:
                wait.wait(min(remaining,.05)); continue
            session.emit(displayed=False); emitted += 1
            if save and emitted == 1: session.save()
            deadline = time.perf_counter() + interval_ms / 1000
    except KeyboardInterrupt:
        reason = 'keyboard_interrupt'
    session.stop(reason)
    session.close(reason)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, default=default_workspace())
    parser.add_argument('--sample', default=None, help='Default: raw_preview_612x512')
    parser.add_argument('--output', type=Path, help='New directory; defaults to workspace/99_临时工作区/L03_Offline_Replay_20261019/run_*')
    parser.add_argument('--mode', choices=['once','loop'], default='once')
    parser.add_argument('--interval-ms', type=int, default=200, help='Host replay interval, not camera FPS (10..60000)')
    parser.add_argument('--frames', type=int, help='Loop limit; headless default 10, GUI default unlimited; 0 means until stopped')
    parser.add_argument('--headless', action='store_true', help='Emit and log without creating a display')
    parser.add_argument('--save', action='store_true', help='Save the first emitted frame once')
    parser.add_argument('--stop-file', type=Path, help='Stop on presence of this file; default OUTPUT/STOP')
    args = parser.parse_args(argv)
    if args.workspace is None: parser.error('--workspace is required outside the formal repository')
    if not 10 <= args.interval_ms <= 60000: parser.error('--interval-ms must be 10..60000')
    if args.frames is not None and args.frames < 0: parser.error('--frames must be nonnegative')
    session = None; root = None
    try:
        catalog = Catalog(args.workspace)
        frame = catalog.load(args.sample or catalog.default)
        if not args.headless:
            from gui import make_root, ReplayWindow
            root = make_root(); root.withdraw()
        output = args.output or catalog.workspace/'99_临时工作区/L03_Offline_Replay_20261019'/('run_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8])
        options = {'sample':frame.sample_id,'mode':args.mode,'interval_ms':args.interval_ms,'frames':args.frames,
                   'headless':args.headless,'save_first_frame':args.save}
        recorder = Recorder(output,catalog,options)
        session = Session(catalog,recorder,frame)
        print(json.dumps({'event':'READY','output':str(recorder.output),'sample':frame.sample_id,'is_replay':True},ensure_ascii=False),flush=True)
        if args.headless:
            run_headless(session,args.mode,args.frames,args.interval_ms,args.save,args.stop_file)
        else:
            window = ReplayWindow(root,session,args.interval_ms,args.frames or 0,args.stop_file)
            if args.mode == 'once': window.once()
            else: window.play()
            if args.save and session.last_index is not None: window.save()
            root.deiconify()
            try: root.mainloop()
            except KeyboardInterrupt: window.close()
        return 2 if session.error else 0
    except Exception as error:
        if session is not None and session.state != 'CLOSED':
            session.fail(error); session.close('error')
        print(json.dumps({'event':'ERROR','error':str(error),'hardware_accessed':False},ensure_ascii=False),file=sys.stderr,flush=True)
        return 2
    finally:
        if root is not None:
            try: root.destroy()
            except Exception: pass


if __name__ == '__main__':
    sys.exit(main())
