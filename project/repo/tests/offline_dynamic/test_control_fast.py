import unittest,threading,time,copy
from dataclasses import replace
from support import *
from contracts import InputError
from detector import FastDetector
from controller import Controller,RecoveryGate
from messages import Audit,LeasePool,LatestSlot


class FastTests(unittest.TestCase):
    def detector(self,**kw):return FastDetector(REFERENCE,STABLE,dict(FAST,**kw))

    def test_presence_stationary_and_departure(self):
        d=self.detector();events=[]
        for i in range(30):events.append(d.process(measurement(i,60 if 4<=i<24 else None),now_ns=i*100_000_000))
        self.assertEqual([e['transition'] for e in events].count('APPEARED'),1)
        self.assertEqual([e['transition'] for e in events].count('DISAPPEARED'),1)
        self.assertTrue(all(e['presence']=='present' for e in events[6:24]))

    def test_direction_and_static_unknown(self):
        d=self.detector()
        events=[d.process(measurement(i,35+i*2),now_ns=i*100_000_000) for i in range(15)]
        self.assertTrue(all(e['direction']=='right' for e in events[3:]))
        for i in range(15,20):e=d.process(measurement(i,63),now_ns=i*100_000_000)
        self.assertEqual(e['direction'],'unknown')

    def test_direction_requires_registered_mapping(self):
        d=self.detector(direction_polarity=None)
        for i in range(8):e=d.process(measurement(i,40+i*2),now_ns=i*100_000_000)
        self.assertEqual(e['direction'],'unknown')

    def test_invalidation_is_not_disappearance(self):
        d=self.detector()
        for i in range(5):d.process(measurement(i,60),now_ns=i*100_000_000)
        e=d.process(measurement(5,60),valid=False,now_ns=500_000_000)
        self.assertEqual((e['presence'],e['transition']),('unknown','NONE'))
        d.process(measurement(6,60),now_ns=600_000_000)
        e=d.process(measurement(7,60),now_ns=700_000_000)
        self.assertEqual(e['transition'],'RESTORED')

    def test_light_negative_and_out_of_range(self):
        d=self.detector()
        for i in range(10):e=d.process(measurement(i,light=.5),now_ns=i*100_000_000)
        self.assertEqual(e['presence'],'absent')
        e=d.process(measurement(10,light=2.),now_ns=1_000_000_000)
        self.assertEqual(e['reason'],'ILLUMINATION_OUT_OF_RANGE')

    def test_duplicate_replay_not_new_update(self):
        d=self.detector();m=measurement(1)
        d.process(m,now_ns=m.frame_complete_ns)
        e=d.process(replace(m,frame_uid='different-replay-index'),now_ns=m.frame_complete_ns)
        self.assertEqual(e['reason'],'DUPLICATE_OR_OUT_OF_ORDER');self.assertEqual(d.processed,1)

    def test_expired_frame(self):
        e=self.detector().process(measurement(1),now_ns=1_000_000_000)
        self.assertEqual(e['reason'],'EXPIRED')

    def test_real_input_not_synthetic_acceptance(self):
        e=self.detector().process(replace(measurement(1),source_kind='CAMERA'),now_ns=100_000_000)
        self.assertEqual(e['reason'],'UNCALIBRATED_REAL_INPUT')

    def test_reference_remains_fixed(self):
        d=self.detector();h=sha(d.reference.tobytes())
        for i in range(30):d.process(measurement(i,60),now_ns=i*100_000_000)
        self.assertEqual(h,sha(d.reference.tobytes()))

    def test_capture_gap_unknown(self):
        d=self.detector();d.process(measurement(1),now_ns=100_000_000)
        e=d.process(measurement(20),now_ns=2_000_000_000)
        self.assertEqual(e['reason'],'CAPTURE_GAP_OR_CLOCK')


class ControlTests(unittest.TestCase):
    def setUp(self):
        self.clock=Clock();self.release=threading.Event();self.started=threading.Event();self.calls=[]
        def backend(req):
            self.calls.append(req);self.started.set()
            if not self.release.wait(5):raise RuntimeError('Test worker release timeout')
            return valid_backend(req)
        self.ctrl=Controller(context(),backend,POLICY,self.clock)
        self.det=FastDetector(REFERENCE,STABLE,FAST)
        self.m,_=warm(self.ctrl,self.det,self.clock)

    def tearDown(self):self.release.set();self.ctrl.close()

    def start(self):
        ack=self.ctrl.request_image(self.m,request_id='first');self.assertEqual(ack['outcome'],'RUNNING')
        self.assertTrue(self.started.wait(1))

    def finish(self):
        self.release.set();self.ctrl.future.result(timeout=2);self.ctrl.poll()

    def test_missing_registration_no_auto(self):
        self.ctrl.policy['auto_registered']=False
        self.assertEqual(self.ctrl.command('a','u',1,'AUTO')['outcome'],'AUTO_NOT_REGISTERED')

    def test_running_solve_survives_mode_switch(self):
        self.start();future=self.ctrl.future
        self.ctrl.command('c','u',1,'MANUAL','IMAGE');self.ctrl.poll()
        self.assertIs(self.ctrl.future,future);self.assertEqual(self.ctrl.strategy,'IMAGE')
        for i in range(4,8):
            now=self.clock.step();e=self.ctrl.ingest(measurement(i,ctx=self.ctrl.context,now=now),self.det)
        self.assertTrue(e['valid']);self.assertEqual(len(self.calls),1)

    def test_manual_overrides_auto(self):
        self.ctrl.command('a','u',1,'AUTO');self.ctrl.poll()
        self.ctrl.command('b','u',2,'MANUAL','FAST');self.ctrl.poll()
        warm(self.ctrl,self.det,self.clock,count=8,start=4)
        self.assertEqual((self.ctrl.control,self.ctrl.strategy),('MANUAL','FAST'))

    def test_auto_stability_and_dwell(self):
        self.ctrl.command('a','u',1,'AUTO');self.ctrl.poll()
        warm(self.ctrl,self.det,self.clock,count=2,start=4)
        self.assertEqual(self.ctrl.strategy,'FAST')
        warm(self.ctrl,self.det,self.clock,count=4,start=6)
        self.assertEqual(self.ctrl.strategy,'IMAGE')

    def test_auto_not_stable_without_new_frames(self):
        self.ctrl.command('a','u',1,'AUTO');self.ctrl.poll();self.clock.step(1000);self.ctrl.poll()
        self.assertEqual(self.ctrl.strategy,'FAST');self.assertNotEqual(self.ctrl.validity,'VALID')

    def test_single_pending_and_manual_busy(self):
        self.start();a=self.ctrl.request_image(self.m,'AUTO','a')
        old_deadline=self.ctrl.pending.deadline_ns;self.clock.step(50)
        b=self.ctrl.request_image(self.m,'AUTO','b')
        self.assertEqual(self.ctrl.pending.deadline_ns,old_deadline)
        self.ctrl.request_image(self.m,'MANUAL','manual')
        self.assertEqual(self.ctrl.request_image(self.m,'MANUAL','manual2')['outcome'],'BUSY')
        self.assertEqual(self.ctrl.pending.id,'manual');self.assertEqual(len(self.calls),1)

    def test_pending_expiration(self):
        self.start();self.ctrl.request_image(self.m,'AUTO','p');self.clock.step(2100);self.ctrl.poll()
        self.assertIsNone(self.ctrl.pending)

    def test_old_snapshot_cannot_be_submitted_as_fresh(self):
        self.clock.step(600)
        self.assertEqual(self.ctrl.request_image(self.m,request_id='old')['outcome'],'EXPIRED_MEASUREMENT')

    def test_policy_epoch_does_not_invalidate_result(self):
        self.start();self.ctrl.command('a','u',1,'MANUAL','FAST');self.ctrl.poll();self.finish()
        self.assertTrue(self.ctrl.last_result['publish_allowed'])

    def test_old_context_late_result_suppressed(self):
        self.start();self.ctrl.invalidate('SCENE_GEOMETRY');self.finish()
        self.assertIsNone(self.ctrl.last_result);self.assertEqual(self.ctrl.results[-1]['freshness'],'STALE_CONTEXT')

    def test_soft_cancel_drains_without_starting_second(self):
        self.start();f=self.ctrl.future;self.ctrl.cancel_image('first')
        self.assertIs(self.ctrl.future,f);self.assertEqual(self.ctrl.image_state,'DRAINING');self.finish()
        self.assertEqual(self.ctrl.results[-1]['freshness'],'CANCELLED')

    def test_invalid_candidate_old_geometry_valid(self):
        before=self.ctrl.context
        self.assertEqual(self.ctrl.propose_model(before,False,True),'REJECTED_CANDIDATE')
        self.assertEqual(self.ctrl.validity,'VALID')

    def test_invalid_candidate_old_geometry_invalid(self):
        self.ctrl.propose_model(self.ctrl.context,False,False)
        self.assertNotEqual(self.ctrl.validity,'VALID')

    def test_commit_waits_for_worker_then_three_frames(self):
        self.start();candidate=self.ctrl.context.changed(model_id='b'*32,model_hash='c'*64)
        self.assertEqual(self.ctrl.propose_model(candidate,True),'DRAINING')
        self.assertNotEqual(self.ctrl.context.data['model_id'],candidate.data['model_id']);self.finish()
        self.assertEqual(self.ctrl.context.data['model_id'],candidate.data['model_id'])
        self.assertEqual(self.ctrl.validity,'INITIALIZING')
        self.det=FastDetector(REFERENCE,STABLE,FAST)  # New registered reference after a model change.
        warm(self.ctrl,self.det,self.clock,count=2,start=4);self.assertNotEqual(self.ctrl.validity,'VALID')
        warm(self.ctrl,self.det,self.clock,count=2,start=6);self.assertEqual(self.ctrl.validity,'VALID')

    def test_failed_commit_does_not_fall_back(self):
        self.ctrl.loader=lambda c:(_ for _ in ()).throw(ValueError('bad model'))
        self.ctrl.propose_model(self.ctrl.context,True)
        self.assertEqual(self.ctrl.validity,'FAULTED')

    def test_retire_generation_once_per_invalid_period(self):
        self.ctrl.invalidate('a');one=self.ctrl.context.data['compat_generation'];self.ctrl.invalidate('b')
        self.assertEqual(one,self.ctrl.context.data['compat_generation'])

    def test_stop_bypasses_full_control_queue(self):
        for i in range(16):self.ctrl.command(str(i),'u',i,'MANUAL','FAST')
        self.assertEqual(self.ctrl.command('full','u',17,'MANUAL','FAST')['outcome'],'BUSY_QUEUE_FULL')
        self.assertEqual(self.ctrl.command('stop','u',18,'STOP')['outcome'],'APPLIED');self.assertTrue(self.ctrl.stopped)

    def test_command_dedup_and_old_sequence(self):
        a=self.ctrl.command('a','u',1,'MANUAL','FAST');b=self.ctrl.command('a','u',1,'MANUAL','IMAGE')
        self.assertEqual(a,b);self.ctrl.poll()
        self.assertEqual(self.ctrl.command('b','u',0,'MANUAL','IMAGE')['outcome'],'EXPIRED_SEQUENCE')

    def test_control_thread_owner(self):
        errors=[]
        def foreign():
            try:self.ctrl.poll()
            except InputError as e:errors.append(e.code)
        t=threading.Thread(target=foreign);t.start();t.join();self.assertEqual(errors,['OWNER'])

    def test_soft_timeout_no_reset(self):
        self.start();self.ctrl.policy['soft_timeout_ms']=10;self.clock.step(20);self.ctrl.poll()
        self.assertEqual(self.ctrl.image_state,'DRAINING');self.assertFalse(self.ctrl.future.done())

    def test_drain_timeout_faults_and_blocks_admission(self):
        self.start();self.ctrl.cancel_image('first');self.ctrl.policy['drain_timeout_ms']=10;self.clock.step(20);self.ctrl.poll()
        self.assertEqual(self.ctrl.validity,'FAULTED');self.assertEqual(self.ctrl.request_image(self.m)['outcome'],'STOPPED_OR_FAULTED')

    def test_audit_overflow_not_silent(self):
        audit=Audit(2)
        for i in range(4):audit.add('test',i=i)
        self.assertEqual(audit.dropped,2);self.assertFalse(audit.complete)

    def test_lease_generation_and_refcounts(self):
        p=LeasePool(1,8);token=p.put(b'123');p.retain(token);p.release(token)
        self.assertIsNone(p.put(b'4'));self.assertEqual(p.get(token),b'123');p.release(token)
        new=p.put(b'5');self.assertNotEqual(new,token)
        with self.assertRaises(InputError):p.get(token)

    def test_latest_frame_slot_records_replacement(self):
        slot=LatestSlot();slot.put('a');self.assertEqual(slot.put('b'),'a');self.assertEqual(slot.replaced,1);self.assertEqual(slot.take(),'b')

    def test_three_distinct_frames_required(self):
        gate=RecoveryGate();m=measurement(1)
        for _ in range(3):self.assertFalse(gate.observe(m,True,m.frame_complete_ns))
        for i in range(2,5):ready=gate.observe(measurement(i),True,i*100_000_000)
        self.assertTrue(ready)

    def test_legacy_context_refused(self):
        d=context().data;d['required_format_version']='PSMB96'
        with self.assertRaises(InputError):Context.create(**d)


if __name__=='__main__':unittest.main(verbosity=2)
