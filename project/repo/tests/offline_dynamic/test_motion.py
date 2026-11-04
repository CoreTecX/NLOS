import unittest,tempfile,dataclasses,time
from contextlib import contextmanager
from motion_fixture import *
from contracts import InputError
from rectify import rectify_raw,bilinear_checked
from registration import core_scope,Registration
from raw_measurement import convert_raw


class MotionTests(unittest.TestCase):
    @contextmanager
    def assertCode(self,code):
        with self.assertRaises(InputError) as raised:yield
        self.assertEqual(raised.exception.code,code)

    @classmethod
    def setUpClass(cls):
        cls.temp=tempfile.TemporaryDirectory();cls.root=Path(cls.temp.name)/'fixture'
        cls.raw,cls.desc,cls.cal,cls.anchor,cls.contract=create_fixture(cls.root)
        cls.source=FileSource('motion-tests');cls.i=0

    @classmethod
    def tearDownClass(cls):cls.temp.cleanup()

    def frame(self,raw=None):
        type(self).i+=1
        return frame_for(self.root,self.source,self.desc,self.raw if raw is None else raw,self.i)

    def reg(self,f,raw):return self.anchor.register(raw,f.envelope['frame_uid'],f.envelope['source_session_id'],calibration_key(self.cal))

    def rect(self,f,d,r,contract=None):
        return rectify_raw(f,d,self.cal,self.root,D11['model']['sha256'],r,self.contract if contract is None else contract)

    def test_identity_matches_static_area(self):
        f,d=self.frame();r=self.reg(f,self.raw)
        a,_=self.rect(f,d,r);b,*_=convert_raw(f,d,self.cal,self.root,D11['model']['sha256'])
        self.assertTrue(np.allclose(decode_pfy(a)[2],decode_pfy(b)[2],rtol=0,atol=1e-10))

    def test_twenty_four_independent_transforms(self):
        rng=np.random.default_rng(719326);rows=[]
        ref_f,ref_d=self.frame();ref=decode_pfy(convert_raw(ref_f,ref_d,self.cal,self.root,D11['model']['sha256'])[0])[2]
        for i in range(24):
            h=similarity(*rng.uniform(-10,10,2),angle=rng.uniform(-2.4,2.4),scale=rng.uniform(.985,1.015))
            self.assertTrue(core_scope(h,ROI)['passed'])
            raw=generate(h);f,d=self.frame(raw);start=time.perf_counter();r=self.reg(f,raw);p,_=self.rect(f,d,r)
            controls=np.array([[x,y] for x in np.linspace(130,626,11) for y in np.linspace(130,626,11)])
            err=np.linalg.norm(transform_points(np.linalg.inv(r.current_to_reference),controls)-transform_points(h,controls),axis=1)/4
            rel=float(np.linalg.norm(decode_pfy(p)[2]-ref)/np.linalg.norm(ref))
            self.assertLess(np.percentile(err,95),.5);self.assertLess(rel,.01)
            rows.append(dict(transform=h.tolist(),true_control_p95_grid_pixels=float(np.percentile(err,95)),
                             relative_measurement_error=rel,seconds=time.perf_counter()-start,diagnostics=r.diagnostics))
        evidence=Path(__file__).resolve().parents[3]/'evidence'
        if evidence.is_dir():write_json(evidence/'motion_metrics.json',dict(scope='SYNTHETIC_ANALYTIC_WALL',transforms=rows,
            accepted=24,physical_motion_accuracy_measured=False,physical_forward_equivalence_measured=False))

    def test_scalar_same_color_sampling_oracle(self):
        h=similarity(4.7,-3.2,1.3,1.006);raw=generate(h);f,d=self.frame(raw);r=self.reg(f,raw)
        rgb=decode_pfy(self.rect(f,d,r)[0])[2];M=np.linalg.inv(r.current_to_reference)
        # Independently enumerate sensor centers and four neighbors; no vector helper.
        for gy in (0,1,15,63,95,125):
            for gx in (0,2,27,64,100,125):
                vals=[]
                for oy,ox in ((0,0),(0,1),(1,0),(1,1)):
                    total=0.
                    for ay in (0,1):
                        for ax in (0,1):
                            p=M@np.array([126+4*gx+2*ax+ox,126+4*gy+2*ay+oy,1.])
                            sx=(p[0]/p[2]-ox)/2;sy=(p[1]/p[2]-oy)/2;ix=int(np.floor(sx));iy=int(np.floor(sy))
                            for dy in (0,1):
                                for dx in (0,1):
                                    weight=(sx-ix if dx else 1-sx+ix)*(sy-iy if dy else 1-sy+iy)
                                    total+=(float(raw[2*(iy+dy)+oy,2*(ix+dx)+ox])-4)*weight
                    vals.append(total/4)
                np.testing.assert_allclose(rgb[gy,gx],[vals[0],(vals[1]+vals[2])/2,vals[3]],rtol=0,atol=1e-9)

    def test_target_does_not_fit_anchor(self):
        h=similarity(7,-5,1.2,1.0);a=generate(h,target=0);b=generate(h,target=2)
        fa,_=self.frame(a);fb,_=self.frame(b);ra=self.reg(fa,a);rb=self.reg(fb,b)
        controls=np.array([[200,200],[600,200],[600,600],[200,600]])
        delta=np.linalg.norm(transform_points(ra.current_to_reference,controls)-transform_points(rb.current_to_reference,controls),axis=1)/4
        self.assertLess(delta.max(),.1)

    def test_reject_reference_roi_features(self):
        with self.assertCode('ANCHOR_MASK'):
            FixedAnchor(self.raw,np.ones((378,378),bool),ROI,calibration_key(self.cal),dict(tracking_low=0,tracking_high=4095))
    def test_weak_reference(self):
        with self.assertCode('WEAK_REFERENCE'):
            mask=np.zeros((378,378),bool);mask[15:40,15:360]=True
            FixedAnchor(np.full((756,756),900),mask,ROI,calibration_key(self.cal),dict(tracking_low=0,tracking_high=4095))
    def test_calibration_mismatch(self):
        with self.assertCode('ANCHOR_CONTEXT'):self.anchor.register(self.raw,'x','y','wrong')
    def test_transform_frame_mismatch(self):
        f,d=self.frame();r=self.reg(f,self.raw)
        with self.assertCode('TRANSFORM_FRAME'):self.rect(f,d,dataclasses.replace(r,frame_uid='wrong'))
    def test_contract_model_mismatch(self):
        f,d=self.frame();r=self.reg(f,self.raw);c=dict(self.contract,model_sha256='0'*64)
        with self.assertCode('MOTION_MODEL'):self.rect(f,d,r,c)
    def test_unregistered_optics(self):
        f,d=self.frame();r=self.reg(f,self.raw)
        with self.assertCode('MOTION_MODEL'):self.rect(f,d,r,dict(self.contract,distortion_model='unknown'))
    def test_scope_bounds_joint(self):
        for h in (similarity(40,0),similarity(angle=3.01),similarity(scale=1.051),similarity(-40,-40)):
            self.assertFalse(core_scope(h,ROI)['passed'])
        self.assertTrue(core_scope(similarity(2,2,1,1.01),ROI)['passed'])
    def test_no_edge_padding(self):
        with self.assertCode('INVALID_SUPPORT'):bilinear_checked(np.ones((4,4)),np.ones((4,4),bool),np.array([3.]),np.array([1.]))
    def test_invalid_neighbor_even_zero_weight(self):
        v=np.ones((4,4),bool);v[2,2]=False
        with self.assertCode('INVALID_SUPPORT'):bilinear_checked(np.ones((4,4)),v,np.array([1.]),np.array([1.]))
    def test_nonfinite_support(self):
        with self.assertCode('INVALID_SUPPORT'):bilinear_checked(np.ones((4,4)),np.ones((4,4),bool),np.array([np.nan]),np.array([1.]))
    def test_saturated_support(self):
        raw=self.raw.copy();raw[300:304,300:304]=4095;f,d=self.frame(raw);r=self.reg(f,raw)
        with self.assertRaises(Exception):self.rect(f,d,r)
    def test_negative_corrected_support(self):
        raw=self.raw.copy();raw[300:304,300:304]=0;f,d=self.frame(raw);r=self.reg(f,raw)
        with self.assertRaises(Exception):self.rect(f,d,r)
    def test_reference_is_immutable(self):
        with self.assertRaises(ValueError):self.anchor.reference[0,0]=1
    def test_projective_or_shear_registration_rejected(self):
        f,d=self.frame();r=self.reg(f,self.raw)
        for h in (np.array([[1,.02,0],[0,1,0],[0,0,1]]),np.array([[1,0,0],[0,1,0],[.001,0,1]])):
            with self.assertCode('DEGENERATE_TRANSFORM'):dataclasses.replace(r,matrix=tuple(h.ravel()))
    def test_manual_outside_core_registration_rejected(self):
        f,d=self.frame();r=self.reg(f,self.raw)
        r=dataclasses.replace(r,matrix=tuple(np.linalg.inv(similarity(45)).ravel()))
        with self.assertCode('OUTSIDE_CAM_CORE'):self.rect(f,d,r)

if __name__=='__main__':unittest.main(verbosity=2)
