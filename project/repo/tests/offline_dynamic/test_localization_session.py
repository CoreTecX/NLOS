import unittest,tempfile,io,contextlib,threading,dataclasses
from support import *
from contracts import pinned,blob,MODEL_BYTES,InputError,FileSource
from bank import Candidate,ModelBank,build_candidate,measurement_feature,model_feature_matrix
from storage import save_bank,load_bank
from session import OfflineSession,MotionAdapter
from controller import Controller
from detector import FastDetector
from motion_fixture import create_fixture,generate,similarity,frame_for,calibration_key


def synthetic_bank(count=3):
    rows=np.arange(64);Q,_=np.linalg.qr(np.random.default_rng(4781).normal(size=(64,64)))
    cs=[Candidate(str(i),Q[:,i*4:i*4+2],Q[:,i*4:i*4+4],D11['model'],D11['model_id'],[i*.01,0.,0.]) for i in range(count)]
    return ModelBank(rows,[1/3]*3,cs,dict(top_k=2,min_fit=.8,min_margin=.05),'SYNTHETIC_TEST_ONLY'),Q


def feature_pfy(y):
    a=REFERENCE.copy();a[1,:, :]=a[0,:,:]
    a[1,:len(y),:]+=np.asarray(y)[:,None]
    return encode_pfy(a,[1,1,1],D11['model_id'])


class BankTests(unittest.TestCase):
    def setUp(self):self.bank,self.q=synthetic_bank()
    def test_known_positions_120_independent_vectors(self):
        rng=np.random.default_rng(532073);correct=0;rows=[]
        for i in range(120):
            k=i%3;weights=rng.normal(size=4);weights/=np.linalg.norm(weights)
            y=self.bank.candidates[k].full@weights+rng.normal(0,.005,64)
            result=self.bank.locate(feature_pfy(y),'SYNTHETIC')
            self.assertTrue(result['valid']);self.assertEqual(result['candidate'],str(k));correct+=1
            rows.append(dict(expected=str(k),selected=result['candidate'],fit=result['fit'],margin=result['margin']))
        evidence=Path(__file__).resolve().parents[3]/'evidence'
        if evidence.is_dir():(evidence/'localization_synthetic.json').write_text(json.dumps(dict(scope='SYNTHETIC_SUBSPACE_DICTIONARY',correct=correct,total=120,
            physical_mm_accuracy_measured=False,grid_position_error_m=0,events=rows),indent=2),'utf8')
    def test_ambiguous(self):
        y=(self.q[:,0]+self.q[:,4])/np.sqrt(2)
        bank=ModelBank(self.bank.rows,[1/3]*3,self.bank.candidates,dict(top_k=2,min_fit=.4,min_margin=.05),'SYNTHETIC_TEST_ONLY')
        self.assertEqual(bank.locate(feature_pfy(y),'SYNTHETIC')['reason'],'AMBIGUOUS')
    def test_out_of_library(self):self.assertEqual(self.bank.locate(feature_pfy(self.q[:,30]),'SYNTHETIC')['reason'],'OUT_OF_LIBRARY')
    def test_unobservable(self):self.assertEqual(self.bank.locate(feature_pfy(np.zeros(64)),'SYNTHETIC')['reason'],'UNOBSERVABLE')
    def test_bad_geometry(self):self.assertEqual(self.bank.locate(feature_pfy(self.q[:,0]),'SYNTHETIC',False)['reason'],'WALL_GEOMETRY_INVALID')
    def test_real_refused(self):self.assertEqual(self.bank.locate(feature_pfy(self.q[:,0]),'CAMERA')['reason'],'PHYSICAL_LIBRARY_NOT_CALIBRATED')
    def test_omitted_coarse_winner_is_certified(self):
        # True winner lives entirely outside its coarse rank; top-K pruning alone loses it.
        y=self.q[:,11]+.1*self.q[:,0]+.1*self.q[:,4]
        r=self.bank.locate(feature_pfy(y),'SYNTHETIC')
        self.assertEqual(r['candidate'],'2');self.assertTrue(r['exhaustive_certificate']);self.assertEqual(r['evaluated_candidates'],3)
    def test_cached_basis_immutable(self):
        with self.assertRaises(ValueError):self.bank.candidates[0].full[0,0]=1
    def test_wrong_rows(self):
        with self.assertRaises(InputError):ModelBank(np.zeros(64,int),[1/3]*3,self.bank.candidates,self.bank.policy,self.bank.scope)
    def test_nonorthonormal(self):
        c=dataclasses.replace(self.bank.candidates[0],full=np.ones((64,3)))
        with self.assertRaises(InputError):ModelBank(self.bank.rows,[1/3]*3,[c,self.bank.candidates[1]],self.bank.policy,self.bank.scope)
    def test_serialized_bank_verified_roundtrip_and_tamper(self):
        with tempfile.TemporaryDirectory() as t:
            p=Path(t)/'bank';save_bank(self.bank,p);b=load_bank(p/'bank.json',WORKSPACE)
            self.assertEqual(b.bank_id,self.bank.bank_id)
            with (p/'basis.npz').open('ab') as f:f.write(b'x')
            with self.assertRaises(InputError):load_bank(p/'bank.json',WORKSPACE)
    def test_model_hash_rejected(self):
        c=dataclasses.replace(self.bank.candidates[0],model=dict(D11['model'],sha256='0'*64))
        b=ModelBank(self.bank.rows,[1/3]*3,[c,self.bank.candidates[1]],self.bank.policy,self.bank.scope)
        with self.assertRaises(InputError):b.verify_candidate(c.id,WORKSPACE)
    def test_actual_pfm_projection_matches_legacy(self):
        sys.path.insert(0,str(WORKSPACE/'02_代码仓库/penumbrasight/src/model'))
        from penumbrasight_model.localization import projection_energy_detail
        rows=np.sort(np.random.default_rng(3206).choice(15750,96,replace=False));reports=[]
        for name in ('D11_rgb','Chair_mushroom'):
            d,_,_=historical_case(CONFIG,WORKSPACE,name)
            c,A=build_candidate(name,d['model'],WORKSPACE,rows)
            data=pinned(WORKSPACE,d['model']);_,payload=blob(data,1,(29,36),MODEL_BYTES)
            norm=np.frombuffer(payload,'<f8',1)[0];scales=np.frombuffer(payload,'<f8',1044,8)
            q=np.frombuffer(payload,'<i2',1044*15876,8+1044*8).reshape(1044,15876)
            # Independent scalar MATLAB column mapping, sampled across basis columns.
            for j in (0,37,1043):
                for i in (0,35,95):
                    y,x=divmod(int(rows[i]),126)
                    expected=(int(q[j,y+1+126*x])-int(q[j,y+126*x]))*scales[j]/(np.sqrt(2)*norm)
                    self.assertAlmostEqual(A[i,j],expected,places=14)
            y=np.random.default_rng(53220).normal(size=96)
            old,rank,s1=projection_energy_detail(A,y,.25);cached=float(np.sum((c.full.T@y)**2))
            self.assertAlmostEqual(old,cached,places=9);self.assertEqual(rank,c.full.shape[1])
            reports.append(dict(case=name,legacy_energy=old,cached_energy=cached,abs_error=abs(old-cached),rank=rank,
                                model_sha256=d['model']['sha256'],derivative_rows=96))
        e=Path(__file__).resolve().parents[3]/'evidence'
        if e.is_dir():(e/'localization_legacy_oracle.json').write_text(json.dumps(reports,indent=2),'utf8')


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        self.ctx=context();self.ctrl=Controller(self.ctx,valid_backend,POLICY)
        self.binding=dict(context=self.ctx.data,reference=REFERENCE,stable_mask=STABLE,fast_policy=FAST)
        self.session=OfflineSession(self.ctrl,{D11['model']['sha256']:self.binding},WORKSPACE)
    def tearDown(self):self.ctrl.close();self.temp.cleanup()
    def descriptor(self,i):
        p=self.root/f'{i}.pfy';data=measurement(i).pfy;p.write_bytes(data)
        d=copy.deepcopy(D11);d.update(origin='SYNTHETIC',file=dict(path=str(p),bytes=len(data),sha256=sha(data)),
            capture=dict(session_id='synthetic-session',original_sensor_frame_id=i))
        return d
    def test_pfy_to_fast_and_actual_worker_interface(self):
        for i in range(4):r=self.session.ingest(self.descriptor(i),self.root,i*100_000_000)
        self.assertEqual(r['fast']['presence'],'absent');self.assertEqual(self.ctrl.validity,'VALID')
        self.assertEqual(self.ctrl.request_image(self.session.last_measurement,request_id='test')['outcome'],'RUNNING')
        self.ctrl.future.result();self.ctrl.poll();self.assertTrue(self.ctrl.results[-1]['valid_numeric'])
    def test_missing_original_sequence_rejected(self):
        d=self.descriptor(1);d['capture'].pop('original_sensor_frame_id')
        self.assertEqual(self.session.ingest(d,self.root,100)['reason'],'SEQUENCE_ID')
    def test_duplicate_never_warms_up(self):
        d=self.descriptor(1)
        for _ in range(4):r=self.session.ingest(d,self.root,100)
        self.assertNotEqual(self.ctrl.validity,'VALID');self.assertFalse(r['fast']['valid'])
    def test_descriptor_integrity_failure_invalidates(self):
        d=self.descriptor(1);d['file']['sha256']='0'*64
        self.assertEqual(self.session.ingest(d,self.root,100)['reason'],'FILE_HASH');self.assertEqual(self.ctrl.validity,'RELOCALIZING')
    def test_backend_wrong_identity_faults(self):
        for i in range(4):self.session.ingest(self.descriptor(i),self.root,i*100_000_000)
        self.ctrl.backend=lambda req:dict(valid_backend(req),measurement_sha256='0'*64)
        self.ctrl.request_image(self.session.last_measurement,request_id='bad');self.ctrl.future.result();self.ctrl.poll()
        self.assertTrue(self.ctrl.faulted);self.assertIsNone(self.ctrl.last_result)
    def test_new_context_requires_reference(self):
        det=FastDetector(REFERENCE,STABLE,FAST);det.process(measurement(0),now_ns=0)
        m=measurement(1,ctx=self.ctx.changed(calibration_id='different'))
        self.assertEqual(det.process(m,now_ns=m.frame_complete_ns)['reason'],'FAST_REFERENCE_CONTEXT')
    def test_selected_candidate_commits_registered_reference(self):
        bank,q=synthetic_bank(2);other,_,_=historical_case(CONFIG,WORKSPACE,'Chair_mushroom')
        second=dataclasses.replace(bank.candidates[1],model=other['model'],model_id=other['model_id'])
        bank=ModelBank(bank.rows,bank.color_weights,[bank.candidates[0],second],bank.policy,bank.scope)
        other_ctx=self.ctx.changed(model_id=other['model_id'],model_hash=other['model']['sha256'],feature_basis_id=bank.bank_id)
        b=dict(context=other_ctx.data,reference=REFERENCE,stable_mask=STABLE,fast_policy=FAST)
        self.session.bindings[other['model']['sha256']]=b;self.session.bank=bank
        m=measurement(1,pfy=feature_pfy(q[:,4]))
        result=self.session.localize(m)
        self.assertTrue(result['valid']);self.assertEqual(result['switch_state'],'COMMITTED_WARMUP')
        self.assertEqual(self.ctrl.context.data['model_hash'],other['model']['sha256'])
        self.assertEqual(self.ctrl.context.data['model_epoch'],'1');self.assertNotEqual(self.ctrl.validity,'VALID')
        previous=self.session.detector;self.session.refresh_binding();self.assertIsNot(previous,self.session.detector)
    def test_raw_motion_to_fast(self):
        raw,d,cal,anchor,contract=create_fixture(self.root/'motion')
        self.ctrl.close();self.ctx=context(session='synthetic_motion').changed(calibration_id=cal['calibration_id'],roi_id=cal['roi_id'],operator_id=cal['operator_id'])
        self.ctrl=Controller(self.ctx,valid_backend,dict(POLICY,capture_gap_ms=10000))
        mask=np.zeros((378,378),bool);mask[15:42,15:-15]=True;mask[-42:-15,15:-15]=True;mask[15:-15,15:42]=True;mask[15:-15,-42:-15]=True
        adapter=MotionAdapter(raw,mask,cal,contract,dict(tracking_low=450,tracking_high=1450,minimum_inliers=12))
        f,desc=frame_for(self.root/'motion',FileSource(),d,raw,0);data,_=adapter.convert(f,desc,self.root/'motion')
        b=dict(context=self.ctx.data,reference=decode_pfy(data)[2],stable_mask=STABLE,fast_policy=dict(FAST,radiometric_normalizer=1000,ttl_ms=5000),motion=adapter)
        s=OfflineSession(self.ctrl,{D11['model']['sha256']:b},WORKSPACE)
        for i in range(4):
            _,desc=frame_for(self.root/'motion',FileSource(),d,generate(similarity(i,-i,.1*i)),i+1)
            r=s.ingest(desc,self.root/'motion',(i+1)*100_000_000)
            self.assertEqual(r['status'],'MEASUREMENT_ACCEPTED')
        self.assertEqual(self.ctrl.validity,'VALID');self.assertTrue(r['provenance']['complete_support'])

    def test_nested_calibration_paths_resolve_from_calibration(self):
        raw,d,cal,anchor,contract=create_fixture(self.root/'nested')
        from raw_measurement import convert_raw
        f,d=frame_for(self.root/'nested',FileSource(),d,raw,1)
        rgb=decode_pfy(convert_raw(f,d,cal,self.root/'nested',D11['model']['sha256'])[0])[2]
        self.ctrl.close();ctx=context(session='synthetic_motion').changed(calibration_id=cal['calibration_id'],roi_id=cal['roi_id'],operator_id=cal['operator_id'])
        self.ctrl=Controller(ctx,valid_backend,POLICY)
        binding=dict(context=ctx.data,reference=rgb,stable_mask=STABLE,fast_policy=dict(FAST,radiometric_normalizer=1000))
        s=OfflineSession(self.ctrl,{D11['model']['sha256']:binding},WORKSPACE)
        d['file']['path']='nested/'+d['file']['path'];d['calibration']['path']='nested/'+d['calibration']['path']
        result=s.ingest(d,self.root,100_000_000)
        self.assertEqual(result['status'],'MEASUREMENT_ACCEPTED')

if __name__=='__main__':unittest.main(verbosity=2)
