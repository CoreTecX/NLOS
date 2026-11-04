"""Acceptance tests on real L02 inputs, with corruption restricted to test copies."""
from pathlib import Path
from unittest import mock
import argparse
import json
import shutil
import subprocess
import sys
import time
import unittest

import numpy as np
from PIL import Image

from core import Catalog, Recorder, Session, ReplayError, read_json, sha, workspace_path
from replay import run_headless
from gui import make_root, ReplayWindow

WORKSPACE = None
OUTPUT = None
PREVIEW = 'raw_preview_612x512'
FULL = 'raw_full_2448x2048'


class ReplayAcceptance(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.catalog = Catalog(WORKSPACE)
        cls.preview = cls.catalog.load(PREVIEW)
        cls.full = cls.catalog.load(FULL)

    def setUp(self):
        self.folder = OUTPUT/self._testMethodName
        self.folder.mkdir(parents=True, exist_ok=False)
        self.sessions = []

    def tearDown(self):
        for session in self.sessions:
            session.close('test_cleanup')

    def session(self):
        session = Session(self.catalog,Recorder(self.folder/'run',self.catalog,{'test':self._testMethodName}),self.preview)
        self.sessions.append(session)
        return session

    def clone(self):
        root = self.folder/'input_copy'
        records = list(self.catalog.lock[key] for key in ['manifest','contract','validator'])
        records += list(self.catalog.contracts[PREVIEW]['files'].values())
        for record in records:
            rel = record['workspace_path']; target = root/rel
            target.parent.mkdir(parents=True,exist_ok=True)
            shutil.copy2(WORKSPACE/rel,target)
        return root

    def cli(self, root, *extra):
        result = subprocess.run([sys.executable,'-B',str(Path(__file__).with_name('replay.py')),
                                 '--workspace',str(root),'--headless','--output',str(self.folder/'cli'),*extra],
                                capture_output=True,text=True,encoding='utf8',timeout=20)
        (self.folder/'stdout.log').write_text(result.stdout,'utf8')
        (self.folder/'stderr.log').write_text(result.stderr,'utf8')
        return result

    def window(self):
        root=make_root();root.withdraw()
        self.addCleanup(lambda: root.destroy() if root.winfo_exists() else None)
        return root,ReplayWindow(root,self.session(),interval_ms=10)

    def pump(self,root,seconds):
        until=time.perf_counter()+seconds
        while time.perf_counter()<until:
            root.update();time.sleep(.003)

    def test_preview_raw_readonly_and_historical_rgb_exact(self):
        frame=self.preview
        self.assertEqual(frame.pixels.shape,(512,612))
        self.assertFalse(frame.pixels.flags.writeable)
        self.assertFalse(frame.preview_rgb.flags.writeable)
        self.assertEqual(sha(frame.raw),self.catalog.rows[PREVIEW]['raw']['sha256'])
        with Image.open(WORKSPACE/self.catalog.rows[PREVIEW]['archival_preview']['source']['workspace_path']) as reference:
            np.testing.assert_array_equal(frame.preview_rgb,np.asarray(reference.convert('RGB')))
        with self.assertRaises(ValueError):frame.pixels[0,0]=123

    def test_full_frame_layout_and_unknown_binning_preserved(self):
        self.assertEqual(self.full.pixels.shape,(2048,2448))
        self.assertEqual(self.full.preview_rgb.shape,(512,612,3))
        self.assertIsNone(self.full.identity['original_capture']['binning'])
        self.assertIsNone(self.full.identity['original_capture']['timestamp_tick_frequency_hz'])
        self.assertEqual(self.full.identity['historical_session_state'],'ERROR')

    def test_cached_replay_does_not_reread_input(self):
        session=self.session()
        with mock.patch('core.pinned_bytes',side_effect=AssertionError('Unexpected file read during cached replay')):
            self.assertIs(self.catalog.load(PREVIEW),self.preview)
            for _ in range(20):self.assertIs(session.emit(),self.preview)
        self.assertEqual(session.index,20)

    def test_headless_loop_logs_replay_index_without_rewriting_source_identity(self):
        session=self.session()
        run_headless(session,'loop',3,10)
        events=[json.loads(line) for line in (session.recorder.output/'events.jsonl').read_text('utf8').splitlines()]
        frames=[row for row in events if row['event']=='FRAME']
        self.assertEqual([f['replay_index'] for f in frames],[1,2,3])
        self.assertEqual({f['original_frame_number'] for f in frames},{1})
        self.assertEqual({f['original_timestamp_ticks'] for f in frames},{self.preview.identity['original_capture']['camera_timestamp_ticks']})
        self.assertTrue(all(f['is_replay'] and not f['displayed'] for f in frames))
        self.assertEqual(read_json(session.recorder.output/'summary.json')['frames_emitted'],3)

    def test_save_exact_raw_metadata_and_lossless_png(self):
        session=self.session();session.emit();saved=session.save()
        self.assertEqual((saved/'frame.raw').read_bytes(),self.preview.raw)
        self.assertEqual((saved/'raw_metadata.json').read_bytes(),self.preview.metadata_bytes)
        with Image.open(saved/'raw_bayer16.png') as image:
            np.testing.assert_array_equal(np.asarray(image),self.preview.pixels)
        snapshot=read_json(saved/'snapshot.json')
        self.assertEqual(snapshot['status'],'COMPLETE')
        self.assertEqual(snapshot['identity']['original_capture'],self.preview.identity['original_capture'])
        with Image.open(saved/'preview.png') as image:
            self.assertEqual(image.height,self.preview.preview_rgb.shape[0]+54)
            self.assertEqual(image.getpixel((0,0)),(20,39,56))
        for name,record in snapshot['files'].items():self.assertEqual(sha((saved/name).read_bytes()),record['sha256'])

    def test_repeated_saves_use_new_paths(self):
        session=self.session();session.emit()
        first=session.save();raw=(first/'frame.raw').read_bytes();second=session.save()
        self.assertNotEqual(first,second)
        self.assertEqual((first/'frame.raw').read_bytes(),raw)
        self.assertEqual(session.recorder.saves,2)

    def test_incomplete_save_not_counted_as_complete(self):
        session=self.session();session.emit()
        with mock.patch('core.Image.Image.save',side_effect=OSError('test disk write failure')):
            with self.assertRaises(OSError):session.save()
        self.assertEqual(session.recorder.saves,0)
        self.assertFalse((session.recorder.output/'saved/snapshot_0001/snapshot.json').exists())
        self.assertEqual(session.save().name,'snapshot_0002')

    def test_switch_clears_last_frame_before_save(self):
        session=self.session();session.emit();session.select(FULL)
        with self.assertRaises(ReplayError):session.save()
        session.emit();self.assertEqual(session.save().joinpath('frame.raw').read_bytes(),self.full.raw)

    def test_failed_switch_invalidates_old_frame(self):
        session=self.session();session.emit()
        with self.assertRaises(ReplayError):session.select('not_registered')
        self.assertIsNone(session.frame)
        with self.assertRaises(ReplayError):session.save()
        session.close()
        self.assertEqual(read_json(session.recorder.output/'summary.json')['status'],'ERROR')

    def test_existing_output_is_never_overwritten(self):
        target=self.folder/'existing';target.mkdir();(target/'marker').write_text('keep','utf8')
        with self.assertRaises(ReplayError):Recorder(target,self.catalog,{})
        self.assertEqual(list(target.iterdir()),[target/'marker'])

    def test_source_and_repository_output_paths_are_rejected(self):
        source=WORKSPACE/self.catalog.rows[PREVIEW]['raw']['workspace_path']
        for target in [source.parent/'l03_forbidden',WORKSPACE/'02_代码仓库/penumbrasight/l03_forbidden',WORKSPACE/'03_任务产出/l03_forbidden']:
            with self.assertRaises(ReplayError):Recorder(target,self.catalog,{})
            self.assertFalse(target.exists())

    def test_input_path_escape_rejected(self):
        with self.assertRaises(ReplayError):workspace_path(WORKSPACE,'../outside.raw')
        with self.assertRaises(ReplayError):workspace_path(WORKSPACE,str(WORKSPACE/'x.raw'))

    def test_cli_corrupted_raw_fails_before_output(self):
        root=self.clone();path=root/self.catalog.rows[PREVIEW]['raw']['workspace_path']
        raw=bytearray(path.read_bytes());raw[0]^=1;path.write_bytes(raw)
        result=self.cli(root)
        self.assertEqual(result.returncode,2);self.assertIn('Input hash changed',result.stderr)
        self.assertFalse((self.folder/'cli').exists())

    def test_cli_truncated_raw_fails_before_output(self):
        root=self.clone();path=root/self.catalog.rows[PREVIEW]['raw']['workspace_path']
        path.write_bytes(path.read_bytes()[:-2])
        result=self.cli(root)
        self.assertEqual(result.returncode,2);self.assertIn('Input length changed',result.stderr)
        self.assertFalse((self.folder/'cli').exists())

    def test_cli_metadata_change_fails_before_output(self):
        root=self.clone();path=root/self.catalog.rows[PREVIEW]['metadata']['workspace_path']
        path.write_bytes(path.read_bytes().replace(b'512',b'513',1))
        result=self.cli(root)
        self.assertEqual(result.returncode,2);self.assertIn('Input hash changed',result.stderr)
        self.assertFalse((self.folder/'cli').exists())

    def test_manifest_pin_prevents_relaxing_the_contract(self):
        root=self.clone();path=root/self.catalog.lock['manifest']['workspace_path']
        path.write_text(path.read_text('utf8').replace('false','true',1),'utf8')
        with self.assertRaises(ReplayError):Catalog(root)

    def test_cli_unknown_sample_rejected(self):
        result=self.cli(WORKSPACE,'--sample','not_registered')
        self.assertEqual(result.returncode,2);self.assertFalse((self.folder/'cli').exists())

    def test_cli_once_saves_and_exits(self):
        result=self.cli(WORKSPACE,'--save')
        self.assertEqual(result.returncode,0,result.stderr)
        summary=read_json(self.folder/'cli/summary.json')
        self.assertEqual((summary['frames_emitted'],summary['frames_displayed'],summary['snapshots_completed']),(1,0,1))

    def test_stop_file_interrupts_a_long_interval(self):
        target=self.folder/'process'
        process=subprocess.Popen([sys.executable,'-B',str(Path(__file__).with_name('replay.py')),
                     '--workspace',str(WORKSPACE),'--headless','--mode','loop','--frames','0','--interval-ms','60000','--output',str(target)],
                     stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,encoding='utf8')
        try:
            until=time.monotonic()+15
            while time.monotonic()<until:
                log=target/'events.jsonl'
                if log.exists() and '"event": "FRAME"' in log.read_text('utf8'):break
                self.assertIsNone(process.poll());time.sleep(.02)
            else:self.fail('Process did not emit a frame')
            start=time.perf_counter();(target/'STOP').touch()
            stdout,stderr=process.communicate(timeout=3)
            elapsed=time.perf_counter()-start
            (self.folder/'stop_latency.json').write_text(json.dumps({'host_stop_seconds':elapsed,'scope':'One host-process test, not a board metric'}),'utf8')
            self.assertEqual(process.returncode,0,stderr)
            self.assertLess(elapsed,3)
            summary=read_json(target/'summary.json')
            self.assertEqual((summary['reason'],summary['frames_emitted']),('stop_file',1))
        finally:
            if process.poll() is None:process.terminate();process.communicate(timeout=5)

    def test_gui_buttons_render_save_stop_and_resume(self):
        root,window=self.window()
        window.buttons['单次显示'].invoke();root.update()
        self.assertIsNotNone(window.photo)
        self.assertEqual((window.photo.width(),window.photo.height()),(306,256))
        self.assertEqual(window.photo._PhotoImage__photo.get(10,10),tuple(int(x) for x in self.preview.preview_rgb[10,10]))
        window.buttons['保存当前帧'].invoke()
        self.assertEqual(window.session.recorder.saves,1)
        window.buttons['循环播放'].invoke();self.pump(root,.08)
        window.buttons['停止'].invoke();count=window.session.index
        self.pump(root,.06);self.assertEqual(window.session.index,count)
        self.assertIsNone(window.timer)
        window.buttons['循环播放'].invoke();self.pump(root,.04);window.stop()
        self.assertGreater(window.session.index,count)

    def test_gui_switches_both_samples_and_keeps_identity(self):
        root,window=self.window();window.once()
        window.choice.set(FULL);window.select();root.update()
        self.assertEqual(window.session.frame.sample_id,FULL)
        self.assertEqual((window.photo.width(),window.photo.height()),(612,512))
        self.assertIn('2448×2048',window.details.get())
        window.choice.set(PREVIEW);window.select();root.update()
        self.assertEqual(window.session.frame.identity['original_capture']['frame_number'],1)
        self.assertEqual(window.session.index,3)

    def test_gui_save_failure_stops_timer_and_logs_error(self):
        root,window=self.window();window.play()
        with mock.patch.object(window.session.recorder,'save',side_effect=OSError('test disk full')):
            window.buttons['保存当前帧'].invoke()
        count=window.session.index;self.pump(root,.05)
        self.assertFalse(window.running);self.assertIsNone(window.timer)
        self.assertEqual(window.session.index,count)
        self.assertEqual(window.session.state,'ERROR')
        self.assertEqual(str(window.buttons['保存当前帧']['state']),'disabled')
        self.assertIsNone(window.photo)

    def test_gui_frame_limit_and_stop_file_leave_no_timer(self):
        root,window=self.window();window.limit=3;window.play();self.pump(root,.12)
        self.assertEqual(window.session.index,3);self.assertFalse(window.running)
        self.assertIsNone(window.timer)
        window.stop_file.touch();window.play();self.pump(root,.03)
        self.assertEqual(window.session.index,3);self.assertFalse(window.running)

    def test_gui_resizing_fits_full_image_and_does_not_emit_frames(self):
        root,window=self.window()
        window.choice.set(FULL);window.select()
        root.deiconify();root.geometry('760x650+30+30');self.pump(root,.06)
        count=window.session.index
        self.assertLessEqual(window.photo.width(),window.image.winfo_width())
        self.assertLessEqual(window.photo.height(),window.image.winfo_height())
        self.assertEqual(count,1)
        for button in window.buttons.values():
            self.assertTrue(button.winfo_viewable())
            self.assertLessEqual(button.winfo_rooty()+button.winfo_height(),root.winfo_rooty()+root.winfo_height())


def main():
    global WORKSPACE,OUTPUT
    parser=argparse.ArgumentParser()
    parser.add_argument('--workspace',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    WORKSPACE=args.workspace.resolve();OUTPUT=args.output.resolve()
    OUTPUT.mkdir(parents=True,exist_ok=False)
    suite=unittest.defaultTestLoader.loadTestsFromTestCase(ReplayAcceptance)
    with (OUTPUT/'tests.log').open('x',encoding='utf8') as log:
        result=unittest.TextTestRunner(stream=log,verbosity=2).run(suite)
    summary={'passed':result.wasSuccessful(),'test_cases':result.testsRun,'failures':len(result.failures),'errors':len(result.errors),
             'hardware_accessed':False,'gui_scope':'Actual Tcl/Tk widgets and PhotoImage; mapped window for resize check; no screen pixel capture',
             'results':(OUTPUT/'tests.log').read_text('utf8')}
    (OUTPUT/'tests.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),'utf8')
    print(summary['results']);print(json.dumps({k:v for k,v in summary.items() if k!='results'},ensure_ascii=False))
    return 0 if result.wasSuccessful() else 1


if __name__=='__main__':sys.exit(main())
