import unittest,threading,queue,hashlib
from support_console import *
from saver import SaveSnapshot,SnapshotSaver
from logger import BoundedLog
from presentation import DisplayMailbox,Presenter
from runtime import ConsoleEngine


def wait_until(predicate,seconds=5):
    stop=time.monotonic()+seconds
    while time.monotonic()<stop:
        if predicate():return
        time.sleep(.01)
    raise AssertionError('Timed out waiting for condition')


class SaveLogTests(unittest.TestCase):
    def setUp(self):self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
    def tearDown(self):self.temp.cleanup()
    def snapshot(self):
        return SaveSnapshot(json.dumps(dict(preview_shape=[2,2,3],frame_uid='fixed')), (('input.raw',bytes(range(12))),('preview.rgb',bytes(range(12)))))
    def test_immutable_copied_snapshot_saved_with_hashes(self):
        rows=[];s=SnapshotSaver(self.root,rows.append);s.submit(self.snapshot());s.close(wait=True)
        self.assertEqual(s.saved,1);folder=Path(rows[0]['path']);saved=read_json(folder/'snapshot.json')
        for name,r in saved['files'].items():self.assertEqual(sha((folder/name).read_bytes()),r['sha256'])
        self.assertTrue((folder/'preview.png').is_file())
    def test_one_running_one_pending_busy(self):
        gate=threading.Event();started=threading.Event()
        def writer(root,rid,snapshot):started.set();gate.wait(3);return str(root/rid)
        s=SnapshotSaver(self.root,writer=writer)
        try:
            self.assertEqual(s.submit(self.snapshot())['outcome'],'QUEUED');self.assertTrue(started.wait(1))
            self.assertEqual(s.submit(self.snapshot())['outcome'],'QUEUED');self.assertEqual(s.submit(self.snapshot())['outcome'],'BUSY')
            self.assertEqual(s.max_active,2);s.close();self.assertFalse(s.drained)
        finally:gate.set();s.close(wait=True)
    def test_disk_full_reports_failure(self):
        rows=[]
        def fail(*args):raise OSError('simulated disk full')
        s=SnapshotSaver(self.root,rows.append,fail);s.submit(self.snapshot());s.close(wait=True)
        self.assertEqual(s.failed,1);self.assertEqual(s.saved,0);self.assertEqual(rows[0]['outcome'],'FAILED')
    def test_no_snapshot_overwrite(self):
        from saver import write_snapshot
        write_snapshot(self.root,'fixed',self.snapshot())
        with self.assertRaises(FileExistsError):write_snapshot(self.root,'fixed',self.snapshot())
    def test_closed_saver_refuses(self):
        s=SnapshotSaver(self.root);s.close(wait=True);self.assertEqual(s.submit(self.snapshot())['outcome'],'CLOSED')
    def test_path_and_mutable_bytes_rejected(self):
        for files in [(('../outside',b'x'),),(('x',bytearray(b'x')),),(('x',b'x'),('x',b'y'))]:
            with self.assertRaises(ValueError):SaveSnapshot('{}',files)
    def test_log_overflow_is_explicit(self):
        gate=threading.Event();started=threading.Event()
        def sink(line):started.set();gate.wait(3)
        log=BoundedLog(self.root/'log',capacity=2,sink=sink)
        try:
            log.emit(dict(event='first'));started.wait(1)
            for i in range(10):log.emit(dict(event='test',i=i))
            self.assertGreater(log.dropped,0);self.assertFalse(log.status()['audit_complete'])
        finally:gate.set();log.close(wait=True)
    def test_log_io_failure_recorded(self):
        def fail(line):raise OSError('disk unavailable')
        log=BoundedLog(self.root/'log',sink=fail);log.emit(dict(event='x'));log.close(wait=True)
        self.assertEqual(log.failed,1);self.assertFalse(log.status()['audit_complete'])
    def test_log_all_records_flushed(self):
        log=BoundedLog(self.root/'log')
        for i in range(20):log.emit(dict(i=i))
        log.close(wait=True);rows=[json.loads(s) for s in (self.root/'log').read_text('utf8').splitlines()]
        self.assertEqual([r['i'] for r in rows],list(range(20)));self.assertTrue(log.status()['audit_complete'])


class PresentationTests(unittest.TestCase):
    def setUp(self):
        self.p=Presenter('run');self.ctx=context().data
        self.p.apply('state',dict(run_id='run',update_id=1,context=self.ctx,validity='VALID',scene_epoch='0'))
    def image(self,update=3,sequence=1,**kwargs):
        result=dict(context=self.ctx,sensor_sequence=str(sequence),publish_allowed=True,valid_numeric=True,
            measurement=dict(frame_complete_ns='1000000000',k=4),freshness='CURRENT',scene_epoch='0',**kwargs)
        return dict(run_id='run',update_id=update,result=result)
    def test_separate_mailbox_channels(self):
        box=DisplayMailbox();box.publish('image',dict(id=1))
        for i in range(20):box.publish('fast',dict(id=i))
        slots,_=box.take();self.assertEqual(slots['image']['id'],1);self.assertEqual(slots['fast']['id'],19);self.assertEqual(box.replaced['fast'],19)
    def test_event_overflow_counted(self):
        box=DisplayMailbox(2)
        for i in range(4):box.event(dict(i=i))
        self.assertEqual(box.dropped_events,2)
    def test_old_run_rejected(self):self.assertFalse(self.p.apply('fast',dict(run_id='old',update_id=10)))
    def test_fast_expiration_shows_unknown(self):
        self.p.apply('fast',dict(run_id='run',update_id=2,context=self.ctx,frame_complete_ns='1000000000',event=dict(valid=True,presence='present',direction='right')))
        self.assertEqual(self.p.view(1_100_000_000)['fast']['presence'],'present')
        self.assertEqual(self.p.view(2_000_000_000)['fast']['presence'],'unknown')
    def test_image_age_continues_without_updates(self):
        self.p.apply('image',self.image());a=self.p.view(1_100_000_000);b=self.p.view(4_000_000_000)
        self.assertEqual(a['image']['age_ms'],100);self.assertEqual(b['image']['age_ms'],3000);self.assertEqual(b['image']['label'],'HISTORICAL_AGE')
    def test_stale_context_cannot_replace_current(self):
        self.p.apply('image',self.image());old=self.p.image
        row=self.image(4,2);row['result']['context']=context().changed(model_epoch=1).data
        self.assertFalse(self.p.apply('image',row));self.assertIs(self.p.image,old);self.assertIs(self.p.history,row)
    def test_older_result_cannot_replace_newer(self):
        self.p.apply('image',self.image(3,5));self.assertFalse(self.p.apply('image',self.image(4,4)));self.assertEqual(self.p.image['result']['sensor_sequence'],'5')
    def test_scene_change_marks_image(self):
        self.p.apply('image',self.image());self.p.apply('state',dict(run_id='run',update_id=5,context=self.ctx,validity='VALID',scene_epoch='1'))
        self.assertEqual(self.p.view(1_100_000_000)['image']['label'],'SCENE_CHANGED')
    def test_invalid_state_does_not_claim_absent(self):
        self.p.apply('fast',dict(run_id='run',update_id=2,context=self.ctx,frame_complete_ns='1000000000',event=dict(valid=True,presence='absent',direction='unknown')))
        self.p.apply('state',dict(run_id='run',update_id=5,context=self.ctx,validity='INVALID',scene_epoch='1'))
        self.assertEqual(self.p.view(1_100_000_000)['fast']['presence'],'unknown')
    def test_result_without_permission_is_history_only(self):
        row=self.image();row['result']['publish_allowed']=False;self.assertFalse(self.p.apply('image',row));self.assertIsNone(self.p.image)


class EngineTests(unittest.TestCase):
    def setUp(self):self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name);self.engine=None
    def tearDown(self):
        if self.engine:self.engine.submit('STOP');self.engine.finished.wait(8)
        self.temp.cleanup()
    def create(self,backend=None):
        def factory(*args):return backend or valid_backend
        self.engine=ConsoleEngine(WORKSPACE,self.root/'run','dynamic',backend_factory=factory);self.engine.start()
        wait_until(lambda:self.engine.processed>=5);return self.engine
    def test_control_commands_have_acks(self):
        e=self.create();e.submit('K',value=4);e.submit('MODE',strategy='FAST')
        wait_until(lambda:e.session.temporal.burst.requested_k==4)
        _,events=e.mailbox.take();self.assertTrue(any(x.get('kind')=='COMMAND_ACK' and x['outcome']=='APPLIED' for x in events))
    def test_stop_drains_running_worker(self):
        started=threading.Event();release=threading.Event()
        def backend(req):started.set();release.wait(4);return valid_backend(req)
        e=self.create(backend)
        try:
            e.submit('IMAGE');self.assertTrue(started.wait(2));e.submit('STOP');time.sleep(.1);self.assertFalse(e.finished.is_set())
        finally:release.set()
        wait_until(e.finished.is_set);self.assertEqual(e.latest_result['freshness'],'STALE_CONTEXT');self.assertFalse(e.latest_result['publish_allowed'])
    def test_save_does_not_stop_fast(self):
        e=self.create();before=e.processed;e.submit('SAVE');wait_until(lambda:e.saver.saved==1);wait_until(lambda:e.processed>before)
        saved=list((e.output/'saved').glob('*/snapshot.json'));self.assertEqual(len(saved),1)
    def test_failed_start_does_not_write_existing_directory(self):
        (self.root/'run').mkdir();(self.root/'run/keep.txt').write_text('keep','utf8')
        e=ConsoleEngine(WORKSPACE,self.root/'run');self.engine=e;e.start();wait_until(e.finished.is_set)
        self.assertTrue(e.failed);self.assertFalse((self.root/'run/console_summary.json').exists());self.assertEqual((self.root/'run/keep.txt').read_text(),'keep')
    def test_stop_bypasses_full_queue(self):
        e=ConsoleEngine(WORKSPACE,self.root/'unstarted')
        for _ in range(20):e.submit('SAVE')
        self.assertGreater(e.busy_commands,0);self.assertEqual(e.submit('STOP')['outcome'],'STOP_REQUESTED');self.assertTrue(e.stop_event.is_set())

if __name__=='__main__':unittest.main(verbosity=2)
