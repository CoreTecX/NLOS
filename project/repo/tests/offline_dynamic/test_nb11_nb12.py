"""Data ownership, bounded latency queues and complete-solver cache contracts."""
from support import *
import unittest,tempfile,threading,warnings
from dataclasses import replace
from unittest.mock import patch
sys.path.insert(0,str(REPO/'tests/offline_console'))
sys.path.insert(0,str(REPO/'src/software/full_model/offline_pipeline'))
from support_console import CONSOLE
from runtime import ConsoleEngine
from resources import VerifiedResources
from contracts import InputError,pinned,FileSource
from capture_bridge import CapturedFrameSource
from controller import Controller,Request
from detector import FastDetector
from backend import FrozenCpuBackend
from saver import SnapshotSaver,SaveSnapshot
from run_batch import run_batch
from native_backend import prepare_native
from inputs import release_info


class MessageTests(unittest.TestCase):
    def test_rgb_and_scales_are_immutable_shared_views(self):
        m=measurement(1,60);self.assertTrue(np.shares_memory(m.rgb,m.rgb))
        self.assertFalse(m.rgb.flags.writeable);self.assertFalse(m.scales.flags.writeable)
        with self.assertRaises(ValueError):m.rgb[0,0,0]=0
        with self.assertRaises(ValueError):m.rgb.setflags(write=True)
        np.testing.assert_array_equal(m.rgb,decode_pfy(m.pfy)[2])
        self.assertEqual(m.sha256,sha(m.pfy))
        # Add an axis without flattening the non-contiguous channel layout.
        with warnings.catch_warnings():
            warnings.simplefilter('ignore',DeprecationWarning)
            view=m.rgb;view.shape=(1,126,126,3)
            scales=m.scales;scales.shape=(1,3)
        self.assertEqual(m.rgb.shape,(126,126,3))
        self.assertEqual(m.scales.shape,(3,))
    def test_replacement_revalidates_and_changes_identity(self):
        m=measurement(1);other=replace(m,pfy=measurement(2,60).pfy)
        self.assertNotEqual(m.sha256,other.sha256);self.assertFalse(np.array_equal(m.rgb,other.rgb))
        damaged=bytearray(m.pfy);damaged[-1]^=1
        with self.assertRaises(InputError):replace(m,pfy=bytes(damaged))
    def test_context_dictionary_cannot_modify_cached_key(self):
        c=context();key=c.key;data=c.data;data['model_epoch']='99'
        self.assertEqual(c.key,key);self.assertNotEqual(c.changed(model_epoch=1).key,key)
    def test_cached_identity_has_fresh_member_dictionary(self):
        m=measurement(1);i=m.identity;i['context']['model_epoch']='99'
        self.assertEqual(m.identity['context']['model_epoch'],'0')


class CacheTests(unittest.TestCase):
    def setUp(self):self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
    def tearDown(self):self.temp.cleanup()
    def rec(self,name,data):
        (self.root/name).write_bytes(data);return dict(path=name,bytes=len(data),sha256=sha(data))
    def test_snapshot_survives_source_change_without_relabeling(self):
        c=VerifiedResources(12,2);a=self.rec('a',b'aaaa');self.assertEqual(c.get(self.root,a),b'aaaa')
        b=self.rec('a',b'bbbb');self.assertEqual(c.get(self.root,a),b'aaaa');self.assertEqual(c.get(self.root,b),b'bbbb')
        self.assertEqual(c.hits,1);self.assertEqual(c.misses,2)
    def test_cold_corruption_and_wrong_model_id_rejected(self):
        c=VerifiedResources();r=self.rec('x',b'good');(self.root/'x').write_bytes(b'evil')
        with self.assertRaises(InputError):c.get(self.root,r)
        with self.assertRaises(InputError):c.model(dict(D11,model_id='0'*32),WORKSPACE)
        self.assertEqual(c.bytes,0)
    def test_lru_memory_budget_and_eviction(self):
        c=VerifiedResources(8,2);rows=[self.rec(str(i),bytes([i])*4) for i in range(3)]
        for r in rows:c.get(self.root,r)
        self.assertEqual((c.bytes,c.evictions),(8,1));self.assertLessEqual(c.peak_bytes,8)
        (self.root/'0').write_bytes(b'xxxx')
        with self.assertRaises(InputError):c.get(self.root,rows[0])
    def test_zero_budget_is_verified_uncached_fallback(self):
        c=VerifiedResources(0);r=self.rec('x',b'123');c.get(self.root,r);c.get(self.root,r)
        self.assertEqual((c.bytes,c.hits,c.misses),(0,0,2))
    def test_clear_releases_all_snapshot_bytes(self):
        c=VerifiedResources(8);c.get(self.root,self.rec('x',b'123'));c.clear()
        self.assertEqual((c.bytes,len(c.entries)),(0,0))
    def test_cache_policy_rejects_unbounded_or_negative_limits(self):
        for n in (-1,True,1024**3):
            with self.assertRaises(InputError):VerifiedResources(n)

    def test_cached_aarch64_launch_keeps_static_cpu_contract(self):
        release,manifest=release_info(CONFIG,WORKSPACE);cache=VerifiedResources()
        cfg=dict(CONFIG,runtime_target='linux_aarch64',backend_mode='cpu')
        with patch('native_backend.platform.system',return_value='Linux'),patch('native_backend.platform.machine',return_value='aarch64'):
            for name in ('one','two'):
                folder=self.root/name;folder.mkdir();plan=prepare_native(cfg,WORKSPACE,release,manifest,folder,resources=cache)
                self.assertEqual(plan['command'][-2:],['cpu','20000']);self.assertTrue(plan['identity']['static'])
                self.assertEqual(plan['libraries'],{});self.assertFalse(plan['hardware_accessed'])
                self.assertEqual(sha((folder/'solver').read_bytes()),manifest['files']['bin/ps_full_aarch64']['sha256'])
        self.assertEqual(cache.hits,1)

    def test_cached_pl_still_requires_current_boot_receipt(self):
        release,manifest=release_info(CONFIG,WORKSPACE);cache=VerifiedResources();folder=self.root/'pl';folder.mkdir()
        with patch('native_backend.platform.system',return_value='Linux'),patch('native_backend.platform.machine',return_value='aarch64'):
            with self.assertRaises(InputError) as e:prepare_native(dict(CONFIG,runtime_target='linux_aarch64',backend_mode='pl'),WORKSPACE,release,manifest,folder,resources=cache)
        self.assertEqual(e.exception.code,'PL_NOT_AUTHORIZED');self.assertEqual(cache.bytes,0)


class CaptureQueueTests(unittest.TestCase):
    def source(self,mode=None):
        clock=Clock();fmt=dict(width=2,height=2,pixel_format='BayerRG12',dtype='<u2',bayer_pattern='RGGB',valid_bits=12,
                              row_stride_bytes=4,roi_sensor_offset=[0,0],binning=[1,1])
        s=CapturedFrameSource('simulated',dict(model='TEST',serial='TEST',mac='TEST'),fmt,dict(exposure_us=5000,gain_raw=64),
                              delivery_mode=mode,clock=clock)
        ctx=dict(boot_id=s.session_id,format=fmt,capture=s.capture,sensor_test_pattern_off=True)
        return s,ctx,clock
    def offer(self,s,ctx,clock,i):
        now=clock.step(10)/1e9
        return s.offer(bytes(8),dict(number=i,block_id=i,timestamp_ticks=i,complete=True,packet_count=1,
                                    leader_monotonic=now-.001,complete_monotonic=now),ctx)
    def test_live_defaults_to_latest_and_records_coalescing(self):
        s,c,t=self.source();self.offer(s,c,t,1);self.offer(s,c,t,2);t.step(5)
        frame,*_=s.next();self.assertEqual(frame.envelope['original_sensor_frame_id'],'2')
        self.assertEqual(frame.envelope['delivery']['queue_wait_ms'],5)
        self.assertEqual(s.status()['consumer_coalesced'],1);self.assertEqual(s.status()['pending'],0)
    def test_producer_overload_has_complete_accounting(self):
        s,c,t=self.source()
        for i in range(1,101):self.offer(s,c,t,i)
        f,*_=s.next();self.assertEqual(f.envelope['original_sensor_frame_id'],'100')
        st=s.status();self.assertEqual(st['enqueued'],st['dequeued']+st['dropped']+st['pending'])
        self.assertEqual(st['dropped'],99);self.assertLessEqual(st['max_pending'],2)
    def test_lossless_backpressure_requires_same_frame_retry(self):
        s,c,t=self.source('lossless');self.offer(s,c,t,1);self.offer(s,c,t,2)
        self.assertEqual(self.offer(s,c,t,3)['outcome'],'BUSY_RETRY_SAME_FRAME')
        ids=[s.next()[0].envelope['original_sensor_frame_id']]
        self.assertEqual(self.offer(s,c,t,3)['outcome'],'QUEUED')
        ids.extend(s.next()[0].envelope['original_sensor_frame_id'] for _ in range(2))
        self.assertEqual(ids,['1','2','3']);self.assertEqual(s.status()['dropped'],0)
    def test_lossless_invalid_frame_receives_explicit_error(self):
        s,c,t=self.source('lossless');self.offer(s,c,t,2);self.offer(s,c,t,1);s.next()
        with self.assertRaises(InputError):s.next()
        self.assertEqual((s.dequeued,s.accepted,s.rejected),(2,1,1))
    def test_queue_rejects_oversize_before_copy(self):
        s,c,t=self.source()
        with self.assertRaises(InputError):s.offer(bytes(10),{},c)
        self.assertEqual(s.enqueued,0)
    def test_offered_bytearray_is_owned_immutably(self):
        s,c,t=self.source();raw=bytearray(8)
        s.offer(raw,dict(number=1,block_id=1,timestamp_ticks=1,complete=True,packet_count=1,leader_monotonic=0,complete_monotonic=.1),c)
        raw[:]=bytes([255])*8;self.assertEqual(s.next()[0].data,bytes(8))


class BatchTests(unittest.TestCase):
    def setUp(self):self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
    def tearDown(self):self.temp.cleanup()
    def manifest(self):
        rows=[]
        for i in range(3):
            raw=json.dumps(dict(index=i)).encode();p=self.root/('%d.json'%i);p.write_bytes(raw)
            rows.append(dict(id='case%d'%i,input=dict(path=p.name,bytes=len(raw),sha256=sha(raw))))
        p=self.root/'batch.json';p.write_text(json.dumps(dict(schema_version=1,items=rows)),'utf8');return p,rows
    def test_all_items_in_order_including_failed_and_blocked(self):
        p,rows=self.manifest();seen=[]
        def execute(args,resources=None,input_snapshot=None):
            i=input_snapshot[0]['index'];seen.append(i)
            return dict(status=['COMPLETE','FAILED','BLOCKED'][i])
        with patch('run_batch.execute',side_effect=execute):r=run_batch(p,REPO/'configs/offline_engineering.json',WORKSPACE,self.root/'out')
        self.assertEqual(seen,[0,1,2]);self.assertTrue(r['all_accounted']);self.assertFalse(r['all_complete'])
        ledger=[json.loads(x) for x in (self.root/'out/ledger.jsonl').read_text('utf8').splitlines()]
        self.assertEqual([x['status'] for x in ledger if x['status']!='STARTED'],['COMPLETE','FAILED','BLOCKED'])
    def test_bad_descriptor_recorded_without_stopping_following_items(self):
        p,rows=self.manifest();(self.root/'1.json').write_bytes(b'corrupt')
        with patch('run_batch.execute',return_value=dict(status='COMPLETE')) as run:
            r=run_batch(p,REPO/'configs/offline_engineering.json',WORKSPACE,self.root/'out')
        self.assertEqual(run.call_count,2);self.assertEqual(r['counts'],dict(COMPLETE=2,FAILED=1,BLOCKED=0))
    def test_duplicate_batch_identity_rejected_before_output(self):
        p,rows=self.manifest();rows[1]['id']=rows[0]['id'];p.write_text(json.dumps(dict(schema_version=1,items=rows)),'utf8')
        with self.assertRaises(InputError):run_batch(p,REPO/'configs/offline_engineering.json',WORKSPACE,self.root/'out')
        self.assertFalse((self.root/'out').exists())


class WorkerLaneTests(unittest.TestCase):
    def test_same_model_keeps_each_registered_measurement_reference(self):
        with tempfile.TemporaryDirectory() as td:
            first=copy.deepcopy(D11);second=copy.deepcopy(D11)
            first['file']['sha256']=measurement(1).sha256
            second['file']['sha256']=measurement(2,60).sha256
            first['golden']=dict(path='first-reference');second['golden']=dict(path='second-reference')
            backend=FrozenCpuBackend(REPO,REPO/'configs/offline_engineering.json',WORKSPACE,Path(td)/'jobs',[first,second]);selected=[]
            def pipeline(args,resources=None):
                selected.append(read_json(args.input)['golden']['path']);return dict(status='FAILED')
            backend.pipeline.execute=pipeline
            try:
                for i,m in enumerate((measurement(1),measurement(2,60))):
                    backend(Request('request'+str(i),m,'MANUAL',0,1000,0,0))
                self.assertEqual(selected,['first-reference','second-reference'])
            finally:backend.close()

    def test_result_handoff_does_not_read_files_on_control_owner(self):
        with tempfile.TemporaryDirectory() as td:
            e=ConsoleEngine(WORKSPACE,Path(td)/'unused');ctx=context();clock=Clock()
            c=Controller(ctx,valid_backend,POLICY,clock);e.controller=c
            class Backend:
                def take_artifacts(self,rid):return (('image_reconstruction_native.png',b'png'),)
            e.backend=Backend();r=dict(request_id='a',valid_numeric=True,publish_allowed=True,freshness='CURRENT',
                                     output=dict(output_directory=td),context=ctx.data,sensor_sequence='1')
            c.results.append(r)
            try:
                with patch.object(Path,'read_bytes',side_effect=AssertionError('Control thread disk read')):e._results()
                self.assertEqual(e.latest_image_files,(('image_reconstruction_native.png',b'png'),))
            finally:c.close()
    def test_slow_saver_keeps_fast_running_with_bounded_queue(self):
        gate=threading.Event();started=threading.Event();rows=[]
        def writer(*args):started.set();gate.wait(4);return 'test'
        with tempfile.TemporaryDirectory() as td:
            saver=SnapshotSaver(td,rows.append,writer);snap=SaveSnapshot('{}',(('data',b'0'),))
            clock=Clock();ctrl=Controller(context(),valid_backend,POLICY,clock);det=FastDetector(REFERENCE,STABLE,FAST)
            try:
                saver.submit(snap);self.assertTrue(started.wait(1));saver.submit(snap)
                self.assertEqual(saver.submit(snap)['outcome'],'BUSY')
                for i in range(30):
                    now=clock.step();event=ctrl.ingest(measurement(i,now=now),det)
                self.assertTrue(event['valid']);self.assertLessEqual(saver.max_active,2)
            finally:gate.set();saver.close(True);ctrl.close()
            self.assertTrue(all(r['queue_wait_ms']>=0 and r['write_ms']>=0 for r in rows))


if __name__=='__main__':unittest.main(verbosity=2)
