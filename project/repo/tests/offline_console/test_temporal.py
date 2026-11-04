import unittest,math
from support_console import *
from detector import FastDetector
from controller import Controller


class QualityTests(unittest.TestCase):
    def setUp(self):self.q=QualityEstimator(REFERENCE,STABLE,CONSOLE['quality'])
    def test_strong_signal(self):
        r=self.q.evaluate(registered(1,60,noise=.01));self.assertEqual(r['quality'],'USABLE');self.assertIsNone(r['probability'])
    def test_low_signal_diagnostics(self):
        r=self.q.evaluate(registered(1,60,amplitude=.1,noise=.06,seed=582))
        self.assertIn(r['quality'],('LOW_SIGNAL','UNOBSERVABLE'));self.assertTrue(r['valid'])
    def test_no_target_is_not_invalid(self):
        r=self.q.evaluate(registered(1));self.assertEqual(r['quality'],'UNOBSERVABLE');self.assertTrue(r['valid']);self.assertIsNone(r['snr_db'])
    def test_invalid_is_not_absent(self):
        r=self.q.evaluate(registered(1),False);self.assertEqual(r['quality'],'INVALID');self.assertFalse(r['valid'])
    def test_unknown_real_scope(self):
        r=self.q.evaluate(dataclasses.replace(registered(1),source_kind='CAMERA'));self.assertEqual(r['reason'],'REAL_QUALITY_UNREGISTERED')
    def test_illumination_gate(self):self.assertFalse(self.q.evaluate(registered(1,light=2))['valid'])
    def test_noise_gate(self):self.assertFalse(self.q.evaluate(registered(1,noise=2,seed=723))['valid'])
    def test_calibration_unknown_never_probability(self):
        r=reconstruction_quality(dict(valid_numeric=True,publish_allowed=False,freshness='STALE_CONTEXT',output=dict(reference_acceptance=None)))
        self.assertEqual(r['reference_state'],'NOT_AVAILABLE');self.assertFalse(r['physical_quality_accepted']);self.assertIsNone(r['probability'])


class BurstTests(unittest.TestCase):
    def fill(self,b,k=8,**kw):
        for i in range(k):
            m=registered(i,60,noise=.04,seed=110+i,**kw);b.observe(m,EVENT,GOOD)
        return m
    def test_k1_byte_identity(self):
        b=accumulator(1);m=self.fill(b,1);out,d=b.select(m);self.assertIs(out,m);self.assertEqual(out.pfy,m.pfy)
    def test_k2_k4_k8_scalar_oracle(self):
        for k in (2,4,8):
            b=accumulator(k);last=self.fill(b,k);out,d=b.select(last)
            arrays=[m.rgb for m in b.frames]
            for y,x,c in ((0,0,0),(125,125,2),(32,71,1),(63,62,0)):
                self.assertAlmostEqual(out.rgb[y,x,c],math.fsum(float(a[y,x,c]) for a in arrays)/k,places=12)
            self.assertEqual(out.k,k);self.assertEqual(d['wait_ms'],(k-1)*100);self.assertEqual(len(out.identity['members']),k)
    def test_missing_frame_clears_history(self):
        b=accumulator(4);self.fill(b,2);m=registered(3,60);b.observe(m,EVENT,GOOD);self.assertEqual(len(b.frames),1);self.assertEqual(b.last_reason,'MISSING_FRAME')
    def test_duplicate_is_not_new_capture(self):
        b=accumulator(2);m=self.fill(b,1);b.observe(dataclasses.replace(m,frame_uid='new-replay-index'),EVENT,GOOD)
        self.assertEqual(len(b.frames),0);self.assertEqual(b.rejected,1)
    def test_out_of_order_refused(self):
        b=accumulator(2);self.fill(b,3);b.observe(registered(1,60),EVENT,GOOD);self.assertEqual(len(b.frames),0)
    def test_context_change_clears(self):
        b=accumulator(4);self.fill(b,2);m=registered(2,60,ctx=context().changed(radiometry_epoch=1));b.observe(m,EVENT,GOOD)
        self.assertEqual(len(b.frames),1);self.assertEqual(b.last_reason,'COMPATIBILITY_CHANGED')
    def test_exposure_signature_change_clears(self):
        b=accumulator(4);self.fill(b,2);b.observe(dataclasses.replace(registered(2,60),capture_signature='different-exposure'),EVENT,GOOD)
        self.assertEqual(len(b.frames),1)
    def test_pfy_rgb_scales_change_clears(self):
        b=accumulator(4);self.fill(b,2);m=registered(2,60);m=dataclasses.replace(m,pfy=encode_pfy(m.rgb,[1,2,1],m.context.data['model_id']))
        b.observe(m,EVENT,GOOD);self.assertEqual(len(b.frames),1)
    def test_source_session_change_clears(self):
        b=accumulator(4);self.fill(b,2);m=registered(0,60,ctx=context('new_session'));b.observe(m,EVENT,GOOD);self.assertEqual(len(b.frames),1)
    def test_gap_and_clock_clear(self):
        for t in (1,900_000_000):
            b=accumulator(4);self.fill(b,2);m=dataclasses.replace(registered(2,60),sample_time_ns=t);b.observe(m,EVENT,GOOD);self.assertEqual(len(b.frames),1)
    def test_span_bound(self):
        b=BurstAccumulator(dict(CONSOLE['burst'],k=8,max_span_ms=400));m=self.fill(b,8);self.assertLessEqual(len(b.frames),5);self.assertIsNone(b.select(m)[0])
    def test_history_bound(self):
        b=accumulator(8);self.fill(b,40);self.assertEqual(len(b.frames),8)
    def test_motion_forces_single(self):
        b=accumulator(8);self.fill(b,8);m=registered(8,90,amplitude=4);b.observe(m,EVENT,GOOD);out,d=b.select(m)
        self.assertEqual(out.k,1);self.assertEqual(d['reason'],'MOTION_FORCED_SINGLE')
    def test_auto_low_snr_chooses_eight(self):
        b=accumulator('AUTO');m=self.fill(b,8);out,d=b.select(m);self.assertEqual(out.k,8)
    def test_auto_high_snr_chooses_one(self):
        b=accumulator('AUTO');m=registered(1,60);b.observe(m,EVENT,dict(GOOD,snr_db=20));self.assertEqual(b.select(m)[0].k,1)
    def test_unknown_snr_does_not_invent_gain(self):
        b=accumulator('AUTO');m=registered(1);b.observe(m,EVENT,dict(GOOD,snr_db=None));self.assertEqual(b.select(m)[0].k,1)
    def test_single_real_frame_never_burst(self):
        b=accumulator(8);m=dataclasses.replace(registered(1),source_kind='REPLAY_SINGLE');b.observe(m,EVENT,GOOD)
        self.assertEqual(b.select(m)[0].k,1)
    def test_unknown_sequence_never_burst(self):
        b=accumulator(8);m=measurement(1);b.observe(m,EVENT,GOOD);self.assertEqual(b.select(m)[0].k,1)
    def test_invalid_resets_and_waits(self):
        b=accumulator(8);m=self.fill(b,8);b.observe(registered(8),dict(valid=False),GOOD,False);self.assertIsNone(b.select(m)[0])
    def test_k_change_clears(self):
        b=accumulator(8);self.fill(b,8);b.set_k(2);self.assertEqual(len(b.frames),0)
    def test_reject_unknown_k(self):
        for k in (0,3,16,True):
            with self.assertRaises(InputError):accumulator(k)
    def test_fast_rejects_aggregate(self):
        b=accumulator(2);m=self.fill(b,2);out,_=b.select(m)
        event=FastDetector(REFERENCE,STABLE,FAST).process(out,now_ns=m.frame_complete_ns)
        self.assertEqual(event['reason'],'FAST_REQUIRES_SINGLE_FRAME')
    def test_controller_fast_continues_when_burst_waiting(self):
        clock=Clock();ctrl=Controller(context(),valid_backend,POLICY,clock);det=FastDetector(REFERENCE,STABLE,FAST)
        pipe=QualityPipeline(REFERENCE,STABLE,CONSOLE['quality'],dict(CONSOLE['burst'],k=8))
        ctrl.image_selector=pipe.select;ctrl.frame_observer=pipe.observe;ctrl.on_invalidate=pipe.reset
        try:
            for i in range(5):
                now=clock.step();m=registered(i,now=now);event=ctrl.ingest(m,det)
            self.assertTrue(event['valid']);self.assertEqual(event['k_fast'],1)
            self.assertEqual(ctrl.request_image(m,request_id='waiting')['outcome'],'WAITING_BURST');self.assertIsNone(ctrl.future)
            for i in range(5,12):
                now=clock.step();m=registered(i,now=now);event=ctrl.ingest(m,det)
            self.assertEqual(ctrl.request_image(m,request_id='ready')['outcome'],'RUNNING')
            ctrl.future.result();ctrl.poll();self.assertEqual(ctrl.results[-1]['measurement']['k'],8)
        finally:ctrl.close()

if __name__=='__main__':unittest.main(verbosity=2)
