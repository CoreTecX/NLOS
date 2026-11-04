from pathlib import Path
import copy,io,json,os,struct,sys,tempfile,unittest
from unittest.mock import patch
from types import SimpleNamespace
import importlib.util,subprocess
REPO=Path(__file__).resolve().parents[2]
for p in ['src/software/camera_service/production_input','src/software/meas_norm','tests/offline_engineering']:
    sys.path.insert(0,str(REPO/p))
import numpy as np
from contracts import FileSource,InputError,sha,read_json,decode_pfy
from inputs import release_info
from native_backend import validate_runtime,elf_identity,prepare_native,within
from capture_bridge import r12_replay,CapturedFrameSource
from raw_measurement import convert_raw
from calibration_tools import estimate_arrays,prepare,validate_real_evidence
from fixture import create,record,save_json
WORKSPACE=Path('D:/学习/26秋嵌入式/PenumbraSight工作区')

class NBTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp=tempfile.TemporaryDirectory(prefix='nb_tests_',dir='D:/Temp');cls.base=Path(cls.temp.name)
        cls.config=read_json(REPO/'configs/offline_engineering.json')
        cls.fixture=cls.base/'fixture';cls.desc,cls.cal,cls.expected=create(cls.fixture,cls.config,WORKSPACE)
        cls.r12=WORKSPACE/'03_任务产出/R12_First_Real_RAW_20261020/data'
        cls.frame,cls.realdesc,_,_=r12_replay(cls.r12)
        cls.meta=read_json(cls.r12/'source_first_frame.json')
        cls.sample=read_json(cls.r12/'sample_manifest.json')
    @classmethod
    def tearDownClass(cls):cls.temp.cleanup()
    def failcode(self,code,fn):
        with self.assertRaises(InputError) as result:fn()
        self.assertEqual(result.exception.code,code)
    def source(self,live=False):
        return CapturedFrameSource(self.sample['boot_id'],self.sample['camera_identity'],self.realdesc['format'],self.realdesc['capture'],
            origin='REAL_CAMERA_LIVE' if live else 'HISTORICAL_REAL_CAMERA_SINGLE_FRAME')
    def test_real_r12_same_raw_identity(self):
        self.assertEqual(sha(self.frame.data),self.sample['raw_sha256'])
        self.assertEqual(self.frame.envelope['source_kind'],'REPLAY_SINGLE')
        self.assertIsNone(self.realdesc['format']['roi_sensor_offset'])
    def test_real_r12_missing_calibration(self):
        self.failcode('MISSING_CALIBRATION',lambda:convert_raw(self.frame,self.realdesc,None,self.base,None))
    def test_real_r12_rejects_synthetic(self):
        self.failcode('CALIBRATION_SCOPE',lambda:convert_raw(self.frame,self.realdesc,self.cal,self.fixture,self.desc['model']['sha256']))
    def test_live_needs_receiver_context(self):
        self.failcode('LIVE_CONTEXT',lambda:self.source(True).accept(self.frame.data,self.meta))
    def test_live_packet_to_common_frame(self):
        s=self.source(True);ctx=dict(boot_id=s.session_id,format=s.fmt,capture=s.capture,sensor_test_pattern_off=True)
        s.offer(self.frame.data,self.meta,ctx);f,d,b,t=s.next()
        self.assertEqual(f.data,self.frame.data);self.assertEqual(f.envelope['camera_timestamp_ticks'],str(self.sample['camera_timestamp_ticks']))
        self.assertFalse(f.envelope['adapter_accessed_hardware']);self.assertIsNone(d['calibration'])
    def test_live_format_change_refused(self):
        s=self.source(True);ctx=dict(boot_id=s.session_id,format=dict(s.fmt,width=610),capture=s.capture,sensor_test_pattern_off=True)
        self.failcode('LIVE_FORMAT',lambda:s.accept(self.frame.data,self.meta,ctx))
    def test_live_radiometry_change_refused(self):
        s=self.source(True);ctx=dict(boot_id=s.session_id,format=s.fmt,capture=dict(s.capture,gain_raw=65),sensor_test_pattern_off=True)
        self.failcode('LIVE_CAPTURE',lambda:s.accept(self.frame.data,self.meta,ctx))
    def test_duplicate_frame_refused(self):
        s=self.source();s.accept(self.frame.data,self.meta)
        self.failcode('FRAME_ORDER',lambda:s.accept(self.frame.data,self.meta))
    def test_incomplete_packet_frame_refused(self):
        self.failcode('INCOMPLETE_FRAME',lambda:self.source().accept(self.frame.data,dict(self.meta,complete=False)))
    def test_truncated_payload_refused(self):
        self.failcode('RAW_SIZE',lambda:self.source().accept(self.frame.data[:-2],self.meta))
    def test_clock_reversal_refused(self):
        self.failcode('CAPTURE_TIME',lambda:self.source().accept(self.frame.data,dict(self.meta,complete_monotonic=0)))
    def test_test_pattern_rejected(self):
        self.failcode('TEST_PATTERN',lambda:self.source().accept(self.frame.data,dict(self.meta,sensor_test_pattern_off=False)))
    def test_bounded_queue_keeps_newer(self):
        s=self.source()
        for i in range(1,6):s.offer(self.frame.data,dict(self.meta,number=i,block_id=i))
        self.assertEqual(s.dropped,3);self.assertEqual(s.next()[0].envelope['original_sensor_frame_id'],'4')
        self.assertEqual(s.next()[0].envelope['original_sensor_frame_id'],'5');self.assertIsNone(s.next())
    def test_architecture_mismatch(self):
        self.failcode('RUNTIME_PLATFORM',lambda:validate_runtime({'runtime_target':'linux_aarch64'},'Windows','AMD64'))
    def test_pl_refused_on_windows(self):
        self.failcode('PL_PLATFORM',lambda:validate_runtime({'backend_mode':'pl'},'Windows','AMD64'))
    def test_linux_cpu_plan_static_elf(self):
        root,manifest=release_info(self.config,WORKSPACE);dest=self.base/'elf';dest.mkdir()
        cfg=dict(self.config,runtime_target='linux_aarch64',backend_mode='cpu')
        with patch('native_backend.platform.system',return_value='Linux'),patch('native_backend.platform.machine',return_value='aarch64'):
            plan=prepare_native(cfg,WORKSPACE,root,manifest,dest)
        self.assertEqual(plan['command'][-2:],['cpu','20000']);self.assertEqual(plan['libraries'],{})
        self.assertTrue(plan['identity']['static']);self.assertFalse(plan['hardware_accessed'])
        self.assertNotIn('.exe',plan['command'][0]);self.assertEqual(sha((dest/'solver').read_bytes()),manifest['files']['bin/ps_full_aarch64']['sha256'])
    def test_pl_requires_current_receipt(self):
        root,manifest=release_info(self.config,WORKSPACE);dest=self.base/'pl';dest.mkdir()
        with patch('native_backend.platform.system',return_value='Linux'),patch('native_backend.platform.machine',return_value='aarch64'):
            self.failcode('PL_NOT_AUTHORIZED',lambda:prepare_native(dict(self.config,backend_mode='pl'),WORKSPACE,root,manifest,dest))
    def test_wrong_elf_rejected(self):self.failcode('ELF_FORMAT',lambda:elf_identity(b'MZ'+bytes(100)))
    def test_path_containment(self):
        self.assertTrue(within(self.base/'a',self.base));self.assertFalse(within(self.base.parent/'other',self.base))
    def failed_solver(self,name,effect=None,returncode=3):
        path=REPO/'src/software/full_model/offline_pipeline/run.py'
        spec=importlib.util.spec_from_file_location('_nb_failure_pipeline',path);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        args=SimpleNamespace(config=REPO/'configs/offline_engineering.json',workspace=WORKSPACE,output=self.base/name,
            case='D11_rgb',raw_sample=None,input=None,action='reconstruct',runtime_root=self.base/(name+'_runtime'),timeout=1)
        with patch.object(module.subprocess,'run',side_effect=effect,return_value=SimpleNamespace(returncode=returncode)):
            result=module.execute(args)
        self.assertEqual(result['status'],'FAILED');self.assertFalse((args.output/'reconstruction_native.png').exists())
        return result
    def test_solver_failure_has_no_success_image(self):
        self.assertEqual(self.failed_solver('failed_native')['error_code'],'SOLVER_FAILED')
    def test_solver_timeout_preserves_explicit_error(self):
        result=self.failed_solver('timeout_native',subprocess.TimeoutExpired(['solver'],1))
        self.assertEqual(result['error_code'],'SOLVER_TIMEOUT')
    def test_flat_gain_independent_planes(self):
        yy,xx=np.mgrid[:252,:252];signal=500+xx*2+yy
        dark=np.ones((4,252,252))*8;flat=np.stack([signal+8]*4)
        arrays,diag=estimate_arrays(dark,flat,np.zeros((126,126,3)))
        for y,x in ((0,0),(0,1),(1,0),(1,1)):
            corrected=(flat[0]-arrays['dark'])*arrays['flat_gain']
            np.testing.assert_allclose(corrected[y::2,x::2],np.median(signal[y::2,x::2]),rtol=1e-14)
        self.assertEqual(diag['invalid_pixels'],0)
    def test_bad_pixels_flagged_not_interpolated(self):
        dark=np.ones((4,252,252))*8;flat=np.ones_like(dark)*900;flat[:,0,0]=4095;flat[:,2,2]=8
        arrays,diag=estimate_arrays(dark,flat,np.zeros((126,126,3)))
        self.assertFalse(arrays['valid'][0,0]);self.assertFalse(arrays['valid'][2,2]);self.assertEqual(diag['invalid_pixels'],2)
    def test_insufficient_calibration_samples(self):
        self.failcode('CALIBRATION_SAMPLES',lambda:estimate_arrays(np.zeros((1,252,252)),np.ones((4,252,252))*900,np.zeros((126,126,3))))
    def make_spec(self,name='cal'):
        folder=self.base/name;folder.mkdir();desc=copy.deepcopy(self.desc);rows={}
        yy,xx=np.mgrid[:504,:504];signal=700+xx//4+yy//4
        for kind in ('dark','flat'):
            rows[kind]=[]
            for i,delta in enumerate((-1,0,0,1)):
                a=np.full((504,504),4+delta,dtype='<u2') if kind=='dark' else (signal+4+delta).astype('<u2')
                path=folder/(kind+str(i)+'.raw');path.write_bytes(a.tobytes())
                d=copy.deepcopy(desc);d['file']=record(path);d['capture']['original_sensor_frame_id']=i+1+(10 if kind=='flat' else 0)
                d['calibration']=None;d['sample_id']=path.stem;save_json(folder/(path.stem+'.json'),d)
                rows[kind].append(record(folder/(path.stem+'.json')))
        np.save(folder/'background.npy',np.zeros((126,126,3)),allow_pickle=False)
        spec=dict(schema_version=1,scope='SYNTHETIC_TEST_ONLY',format=desc['format'],capture=desc['capture'],model=desc['model'],
            model_id=desc['model_id'],background=record(folder/'background.npy'),dark=rows['dark'],flat=rows['flat'],
            roi_xywh=[0,0,504,504],green_weights=[.25,.75],rgb_scales=[1,1,1],linear_response_verified=True,
            calibration_id='NB07_SYNTHETIC',geometry_id='synthetic-grid',roi_id='full504',radiometry_id='synthetic-dark-flat')
        save_json(folder/'manifest.json',spec);return folder,spec
    def test_generated_calibration_full_conversion_oracle(self):
        folder,spec=self.make_spec('generate');out=folder/'output';c,diag=prepare(folder/'manifest.json',out)
        frame=FileSource().load(self.desc,self.fixture)
        pfy,context,_=convert_raw(frame,self.desc,c,out,self.desc['model']['sha256']);_,_,actual=decode_pfy(pfy)
        raw=FileSource.decode_raw(frame.data,self.desc['format'])
        with np.load(out/'corrections.npz') as z:
            dark=z['dark'];gain=z['flat_gain']
            expected=np.zeros((126,126,3))
            for y in range(126):
                for x in range(126):
                    channels=[]
                    for oy,ox in ((0,0),(0,1),(1,0),(1,1)):
                        samples=[(int(raw[4*y+2*dy+oy,4*x+2*dx+ox])-dark[4*y+2*dy+oy,4*x+2*dx+ox])*gain[4*y+2*dy+oy,4*x+2*dx+ox] for dy in range(2) for dx in range(2)]
                        channels.append(sum(samples)/4)
                    expected[y,x]=[channels[0],.25*channels[1]+.75*channels[2],channels[3]]
        np.testing.assert_allclose(actual,expected,rtol=2e-15,atol=2e-12)
        self.assertEqual(context['calibration_scope'],'SYNTHETIC_TEST_ONLY')
    def test_cannot_relabel_synthetic_as_real_calibration(self):
        folder,spec=self.make_spec('relabel');spec['scope']='REAL_CAPTURE';spec['linear_response_verified']=False;save_json(folder/'manifest.json',spec)
        self.failcode('CALIBRATION_SCOPE',lambda:prepare(folder/'manifest.json',folder/'output'))
    def test_synthetic_not_promotable_by_evidence_flags(self):
        self.failcode('CALIBRATION_SCOPE',lambda:validate_real_evidence(self.cal,{},self.base))
    def test_repeated_calibration_capture_refused(self):
        folder,spec=self.make_spec('duplicate');spec['dark'][1]=spec['dark'][0];save_json(folder/'manifest.json',spec)
        self.failcode('CALIBRATION_SAMPLES',lambda:prepare(folder/'manifest.json',folder/'output'))

if __name__=='__main__':unittest.main(verbosity=2)
