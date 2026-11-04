"""NB08-NB10 host contracts; synthetic scenes are never physical evidence."""
import unittest,tempfile,threading,json
from dataclasses import replace
from unittest.mock import patch
from support import *
from contracts import InputError,FileSource
from detector import FastDetector
from controller import Controller
from session import OfflineSession,MotionAdapter
from capture_bridge import CapturedFrameSource
from registration import CORE_DOMAIN,validated_domain,control_errors,Registration
from rectify import rectify_raw
from motion_fixture import create_fixture,frame_for,generate,similarity,ROI
from raw_measurement import calibration_key,convert_raw


class FastRobustness(unittest.TestCase):
    def test_requires_nonempty_sensing_region(self):
        with self.assertRaises(InputError):FastDetector(REFERENCE,np.ones_like(STABLE),FAST)

    def test_invalid_policy_rejected(self):
        for change in ({'confirm_frames':1.5},{'confirm_frames':True},{'noise_floor':float('nan')},
                       {'allow_additive_correction':'yes'},{'max_stable_residual':float('inf')}):
            with self.subTest(change=change),self.assertRaises(InputError):FastDetector(REFERENCE,STABLE,dict(FAST,**change))

    def test_nonuniform_illumination_is_unknown(self):
        det=FastDetector(REFERENCE,STABLE,FAST)
        rgb=REFERENCE+((xx-63)*.015)[:,:,None]
        m=measurement(0,pfy=encode_pfy(rgb,[1,1,1],context().data['model_id']))
        event=det.process(m,now_ns=0)
        self.assertEqual(event['reason'],'STABLE_REFERENCE_CHANGED')
        self.assertEqual(event['transition'],'NONE');self.assertIsNone(event['confidence_probability'])

    def test_partial_reference_occlusion_is_unknown(self):
        rgb=REFERENCE.copy();rgb[:12,:]+=1
        m=measurement(0,pfy=encode_pfy(rgb,[1,1,1],context().data['model_id']))
        event=FastDetector(REFERENCE,STABLE,FAST).process(m,now_ns=0)
        self.assertEqual(event['reason'],'STABLE_REFERENCE_CHANGED')

    def test_existing_console_noise_remains_valid_without_false_target(self):
        det=FastDetector(REFERENCE,STABLE,FAST)
        for i in range(100):
            e=det.process(measurement(i,noise=.02,seed=20261019+i),now_ns=i*100_000_000)
            self.assertIsNone(e['reason']);self.assertNotEqual(e['transition'],'APPEARED')
        self.assertEqual(e['presence'],'absent')

    def test_validity_deadline_is_capture_based(self):
        det=FastDetector(REFERENCE,STABLE,FAST)
        for i in range(3):event=det.process(measurement(i),now_ns=i*100_000_000+150_000_000)
        self.assertTrue(event['valid']);self.assertEqual(int(event['expires_ns']),450_000_000)

    def test_low_contrast_below_floor_has_no_claim(self):
        det=FastDetector(REFERENCE,STABLE,FAST)
        events=[det.process(measurement(i,60,amplitude=.02),now_ns=i*100_000_000) for i in range(8)]
        self.assertFalse(any(e['transition']=='APPEARED' for e in events))
        self.assertIsNone(events[-1]['confidence_probability'])

    def test_one_frame_occlusion_does_not_chatter(self):
        det=FastDetector(REFERENCE,STABLE,FAST)
        for i in range(4):det.process(measurement(i,60),now_ns=i*100_000_000)
        e=det.process(measurement(4),now_ns=400_000_000)
        self.assertEqual((e['presence'],e['transition']),('present','NONE'))
        e=det.process(measurement(5,60),now_ns=500_000_000)
        self.assertEqual((e['presence'],e['transition']),('present','NONE'))

    def test_unknown_then_restored_is_not_new_target(self):
        det=FastDetector(REFERENCE,STABLE,FAST)
        for i in range(4):det.process(measurement(i,60),now_ns=i*100_000_000)
        e=det.process(measurement(4,60,light=2),now_ns=400_000_000)
        self.assertEqual(e['presence'],'unknown')
        for i in (5,6):e=det.process(measurement(i,60),now_ns=i*100_000_000)
        self.assertEqual(e['transition'],'RESTORED')


class SchedulerSafety(unittest.TestCase):
    def setUp(self):
        self.clock=Clock();self.release=threading.Event();self.calls=[]
        def backend(req):
            self.calls.append(req)
            if not self.release.wait(5):raise RuntimeError('test barrier timeout')
            return valid_backend(req)
        self.ctrl=Controller(context(),backend,POLICY,self.clock)
        self.det=FastDetector(REFERENCE,STABLE,FAST)
        self.m,_=warm(self.ctrl,self.det,self.clock)

    def tearDown(self):self.release.set();self.ctrl.close()
    def start(self):self.assertEqual(self.ctrl.request_image(self.m,request_id='first')['outcome'],'RUNNING')
    def finish(self):self.release.set();self.ctrl.future.result(2);self.ctrl.poll()
    def feed(self,i,x=None):
        now=self.clock.step();return self.ctrl.ingest(measurement(i,x,ctx=self.ctrl.context,now=now),self.det)

    def test_scene_change_cannot_publish_old_reconstruction(self):
        self.start();self.ctrl.request_image(self.m,'AUTO','waiting')
        for i in range(4,7):self.feed(i,60)
        self.assertIsNone(self.ctrl.pending);self.finish()
        result=self.ctrl.results[-1]
        self.assertTrue(result['valid_numeric']);self.assertEqual(result['freshness'],'SCENE_CHANGED')
        self.assertFalse(result['publish_allowed']);self.assertIsNone(self.ctrl.last_result)

    def test_pending_capture_expires_despite_fresh_input(self):
        self.start();self.ctrl.request_image(self.m,'AUTO','waiting')
        for i in range(4,11):self.feed(i)
        self.assertEqual(self.ctrl.validity,'VALID');self.assertIsNone(self.ctrl.pending)
        self.finish();self.assertEqual(len(self.calls),1)

    def test_poll_checks_capture_loss_before_finished_result(self):
        self.start();self.clock.step(600);self.finish()
        self.assertFalse(self.ctrl.results[-1]['publish_allowed'])
        self.assertEqual(self.ctrl.results[-1]['freshness'],'STALE_CONTEXT')

    def test_manual_fast_discards_auto_pending_but_drains_running(self):
        self.start();future=self.ctrl.future;self.ctrl.request_image(self.m,'AUTO','waiting')
        self.ctrl.command('fast','operator',1,'MANUAL','FAST');self.ctrl.poll()
        self.assertIsNone(self.ctrl.pending);self.assertIs(self.ctrl.future,future)
        self.finish();self.assertTrue(self.ctrl.last_result['publish_allowed'])

    def test_explicit_snapshot_survives_manual_fast(self):
        self.start();self.ctrl.request_image(self.m,'MANUAL','explicit')
        self.ctrl.command('fast','operator',1,'MANUAL','FAST');self.ctrl.poll()
        self.assertEqual(self.ctrl.pending.id,'explicit')

    def test_cancel_drain_keeps_fast_responsive(self):
        self.start();self.ctrl.cancel_image('first')
        for i in range(4,10):self.assertTrue(self.feed(i)['valid'])
        self.assertFalse(self.ctrl.future.done());self.finish()
        self.assertFalse(self.ctrl.results[-1]['publish_allowed'])

    def test_latest_pending_has_correct_identity(self):
        self.start();self.ctrl.request_image(self.m,'AUTO','old')
        now=self.clock.step();m=measurement(4,ctx=self.ctrl.context,now=now)
        self.ctrl.ingest(m,self.det);self.ctrl.request_image(m,'AUTO','new')
        self.finish();self.ctrl.future.result(2);self.ctrl.poll()
        self.assertEqual(self.ctrl.last_result['request_id'],'new')
        self.assertEqual(self.ctrl.last_result['frame_uid'],m.frame_uid)
        self.assertEqual([r.id for r in self.calls],['first','new'])

    def test_auto_motion_then_stability_hysteresis(self):
        self.ctrl.command('auto','operator',1,'AUTO');self.ctrl.poll()
        for i in range(4,11):self.feed(i)
        self.assertEqual(self.ctrl.strategy,'IMAGE')
        for i in range(11,16):self.feed(i,45+2*(i-11))
        self.assertEqual(self.ctrl.strategy,'FAST')
        for i in range(16,25):self.feed(i,53)
        self.assertEqual(self.ctrl.strategy,'IMAGE')

    def test_stop_prevents_second_solve(self):
        self.start();self.ctrl.request_image(self.m,'MANUAL','waiting');self.ctrl.stop(all_work=True)
        self.finish();self.assertEqual(len(self.calls),1);self.assertIsNone(self.ctrl.pending)
        self.assertFalse(self.ctrl.results[-1]['publish_allowed'])

    def test_slow_accumulated_motion_retires_snapshot(self):
        for i in range(4,8):self.feed(i,55)
        self.m=self.ctrl.latest_measurement;self.start();epoch=self.ctrl.scene_epoch
        for i in range(8,28):
            e=self.feed(i,55+.2*(i-7));self.assertLess(e['activity'],POLICY['activity_high'])
        self.assertGreater(self.ctrl.scene_epoch,epoch);self.finish()
        self.assertEqual(self.ctrl.results[-1]['freshness'],'SCENE_CHANGED')
        self.assertFalse(self.ctrl.results[-1]['publish_allowed'])

    def test_accumulated_motion_in_auto_returns_to_fast(self):
        for i in range(4,8):self.feed(i,55)
        self.ctrl.command('auto','operator',1,'AUTO');self.ctrl.poll()
        for i in range(8,16):self.feed(i,55)
        self.assertEqual(self.ctrl.strategy,'IMAGE')
        observed=[]
        for i in range(16,36):
            e=self.feed(i,55+.2*(i-15))
            if e.get('scene_changed'):observed.append(self.ctrl.strategy)
        self.assertTrue(observed);self.assertIn('FAST',observed)

    def test_nan_policy_rejected(self):
        for key in ('pending_ttl_ms','activity_high'):
            with self.assertRaises(InputError):Controller(context(),valid_backend,dict(POLICY,**{key:float('nan')}))


class MotionBoundaries(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp=tempfile.TemporaryDirectory();cls.root=Path(cls.temp.name)/'motion'
        cls.raw,cls.desc,cls.cal,cls.anchor,cls.contract=create_fixture(cls.root)
        cls.source=FileSource();cls.i=0
    @classmethod
    def tearDownClass(cls):cls.temp.cleanup()
    def frame(self,raw=None):
        type(self).i+=1
        return frame_for(self.root,self.source,self.desc,self.raw if raw is None else raw,self.i)
    def reg(self,f,raw):return self.anchor.register(raw,f.envelope['frame_uid'],f.envelope['source_session_id'],calibration_key(self.cal))
    def rect(self,f,d,r,c):return rectify_raw(f,d,self.cal,self.root,D11['model']['sha256'],r,c)

    def test_narrow_model_domain_rejects_inside_generic_core(self):
        raw=generate(similarity(8));f,d=self.frame(raw);r=self.reg(f,raw)
        c=dict(self.contract,validated_domain=dict(CORE_DOMAIN,max_corner_displacement_roi_diagonal=.005))
        with self.assertRaises(InputError) as e:self.rect(f,d,r,c)
        self.assertEqual(e.exception.code,'OUTSIDE_MODEL_DOMAIN')

    def test_narrow_control_error_domain(self):
        f,d=self.frame();r=self.reg(f,self.raw)
        diag=dict(r.diagnostics,independent_control_p95_grid_pixels=.2)
        r=replace(r,diagnostics_json=json.dumps(diag))
        with self.assertRaises(InputError) as e:self.rect(f,d,r,dict(self.contract,validated_domain=dict(CORE_DOMAIN,max_control_p95_grid_pixels=.1)))
        self.assertEqual(e.exception.code,'CONTROL_ERROR')

    def test_domain_cannot_silently_expand(self):
        for change in ({'max_abs_rotation_degrees':4},{'min_overlap':.8},{'max_abs_scale_delta':float('nan')}):
            with self.assertRaises(InputError):validated_domain(dict(CORE_DOMAIN,**change))
        with self.assertRaises(InputError):validated_domain({})

    def test_rectangular_roi_control_errors_use_both_axes(self):
        a=np.array([[0.,0.],[10.,10.]])
        errors=control_errors(np.eye(3),a,a+[[0,1],[1,0]],[0,0,504,252])
        np.testing.assert_allclose(errors,[1.,.5],rtol=0,atol=1e-14)

    def test_perspective_change_rejected_by_independent_controls(self):
        center=np.array([[1,0,378],[0,1,378],[0,0,1.]])
        h=center@np.array([[1,0,0],[0,1,0],[.00015,0,1.]])@np.linalg.inv(center)
        raw=generate(h);f,_=self.frame(raw)
        with self.assertRaises(InputError) as e:self.reg(f,raw)
        self.assertIn(e.exception.code,('CONTROL_ERROR','TRACK_FAILED','CONTROL_POINTS','OUTSIDE_CAM_CORE'))

    def test_reference_loss_rejected(self):
        raw=np.full_like(self.raw,900);f,_=self.frame(raw)
        with self.assertRaises(InputError):self.reg(f,raw)

    def test_nonfinite_tracking_rejected(self):
        raw=self.raw.astype(float);raw[0,0]=float('nan');f,_=self.frame()
        with self.assertRaises(InputError) as e:self.reg(f,raw)
        self.assertEqual(e.exception.code,'TRACKING_FORMAT')

    def test_invalid_tracking_backend_is_controlled_error(self):
        f,_=self.frame()
        with patch('registration.cv2.calcOpticalFlowPyrLK',return_value=(None,None,None)),self.assertRaises(InputError) as e:self.reg(f,self.raw)
        self.assertEqual(e.exception.code,'TRACK_FAILED')

    def test_real_domain_missing_rejected_before_physical_claim(self):
        f,d=self.frame();r=self.reg(f,self.raw)
        cal=dict(self.cal,scope='REAL_CALIBRATED');c=dict(self.contract,scope='REAL_CALIBRATED',reference_calibration_key=calibration_key(cal))
        r=replace(r,calibration_key=calibration_key(cal))
        with self.assertRaises(InputError) as e:rectify_raw(f,d,cal,self.root,D11['model']['sha256'],r,c)
        self.assertEqual(e.exception.code,'MOTION_DOMAIN')

    def test_reference_loss_and_three_distinct_frame_recovery(self):
        ctx=context(session='synthetic_motion').changed(calibration_id=self.cal['calibration_id'],roi_id=self.cal['roi_id'],operator_id=self.cal['operator_id'])
        ctrl=Controller(ctx,valid_backend,dict(POLICY,capture_gap_ms=5000))
        mask=np.zeros((378,378),bool);mask[15:42,15:-15]=True;mask[-42:-15,15:-15]=True
        mask[15:-15,15:42]=True;mask[15:-15,-42:-15]=True
        motion=MotionAdapter(self.raw,mask,self.cal,self.contract,dict(tracking_low=450,tracking_high=1450),self.root)
        f,d=self.frame();pfy,_=motion.convert(f,d,self.root)
        binding=dict(context=ctx.data,reference=decode_pfy(pfy)[2],stable_mask=STABLE,
                     fast_policy=dict(FAST,radiometric_normalizer=1000,ttl_ms=5000),motion=motion)
        session=OfflineSession(ctrl,{D11['model']['sha256']:binding},WORKSPACE)
        def feed(raw):
            f,d=self.frame(raw);return session.ingest_frame(f,d,self.root,self.i*100_000_000)
        try:
            for _ in range(4):out=feed(self.raw)
            self.assertEqual(ctrl.validity,'VALID')
            out=feed(np.full_like(self.raw,900));self.assertEqual(out['status'],'FRAME_REJECTED')
            self.assertIsNone(session.last_measurement)
            generation=ctrl.context.data['compat_generation']
            for _ in range(2):
                out=feed(self.raw);self.assertFalse(out['fast']['valid']);self.assertNotEqual(ctrl.validity,'VALID')
            out=feed(self.raw);self.assertEqual(ctrl.validity,'VALID')
            out=feed(self.raw);self.assertTrue(out['fast']['valid'])
            self.assertEqual(ctrl.context.data['compat_generation'],generation)
        finally:ctrl.close()


class LiveBridgeIntegration(unittest.TestCase):
    def test_transport_kind_reaches_measurement_then_real_policy_gate(self):
        # Simulated receiver bytes and mocked calibration conversion only.
        # No real calibration, camera acquisition or hardware validation is asserted.
        with tempfile.TemporaryDirectory() as td:
            root=Path(td)/'fixture';raw,desc,cal,_,_=create_fixture(root)
            source=CapturedFrameSource('simulated-receiver',dict(model='TEST',serial='TEST',mac='TEST'),
                desc['format'],desc['capture'],binding={k:desc[k] for k in ('model','model_id','calibration')},base=root)
            rc=dict(boot_id=source.session_id,format=source.fmt,capture=source.capture,sensor_test_pattern_off=True)
            clock=Clock();ctx=context(session=source.session_id);ctrl=Controller(ctx,valid_backend,POLICY,clock)
            session=OfflineSession(ctrl,{D11['model']['sha256']:dict(context=ctx.data,reference=REFERENCE,stable_mask=STABLE,fast_policy=FAST)},WORKSPACE)
            try:
                for i in range(1,4):
                    now=clock.step();meta=dict(number=i,block_id=i,timestamp_ticks=i,complete=True,packet_count=100,
                                               leader_monotonic=now/1e9-.01,complete_monotonic=now/1e9)
                    f,d,b,t=source.accept(raw.tobytes(),meta,rc)
                    self.assertEqual(f.envelope['source_kind'],'CAMERA')
                    with patch('session.convert_raw',return_value=(measurement(i).pfy,{})):
                        out=session.ingest_frame(f,d,b,t)
                    self.assertEqual(out['status'],'MEASUREMENT_ACCEPTED')
                self.assertEqual(out['fast']['reason'],'UNCALIBRATED_REAL_INPUT')
                # Unmocked path must reject synthetic calibration on camera data.
                out=session.ingest_frame(f,d,b,t)
                self.assertEqual(out['reason'],'CALIBRATION_SCOPE')
            finally:ctrl.close()


if __name__=='__main__':unittest.main(verbosity=2)
