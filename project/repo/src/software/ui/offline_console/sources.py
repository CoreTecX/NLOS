"""Explicit synthetic live source and pinned finite offline manifest source."""
from pathlib import Path
import io,json,time,uuid
import numpy as np
from contracts import Frame,FileSource,read_json,pinned,encode_pfy,decode_pfy,sha,require,resolve
from inputs import historical_case
from messages import Context
from session import MotionAdapter
from storage import load_bank


def load_bindings(spec,base):
    bindings={}
    for b in spec['bindings']:
        with np.load(io.BytesIO(pinned(base,b['reference_arrays'])),allow_pickle=False) as z:
            value=dict(context=b['context'],reference=z['reference'],stable_mask=z['stable_mask'],fast_policy=b['fast_policy'])
        if b.get('motion'):
            motion=b['motion'];cal=json.loads(pinned(base,motion['calibration']));contract=json.loads(pinned(base,motion['contract']))
            with np.load(io.BytesIO(pinned(base,motion['anchor_arrays'])),allow_pickle=False) as z:
                value['motion']=MotionAdapter(z['raw'],z['stable_mask'],cal,contract,motion['policy'],resolve(base,motion['calibration']['path']).parent)
        bindings[b['context']['model_hash']]=value
    return bindings


class SyntheticSource:
    def __init__(self,mode,config,dynamic,workspace):
        require(mode in ('dynamic','d11'),'SOURCE','Unknown synthetic source')
        self.mode=mode;self.desc,_,_=historical_case(config,workspace,'D11_rgb');self.models=[self.desc]
        self.original=pinned(workspace,self.desc['file']);_,self.scales,self.d11=decode_pfy(self.original)
        self.index=0;self.session_id='synthetic-console-'+uuid.uuid4().hex;self.period_ms=100;self.base=Path(workspace)
        yy,xx=np.mgrid[:126,:126];self.xx,self.yy=xx,yy
        self.reference=self.d11 if mode=='d11' else np.stack([10+.005*xx+.003*yy]*3,axis=2)
        self.stable=(xx<16)|(xx>109)|(yy<16)|(yy>109)
        self.context=Context.create(source_session_id=self.session_id,compat_generation=0,geometry_epoch=0,model_id=self.desc['model_id'],
            model_hash=self.desc['model']['sha256'],model_epoch=0,calibration_id='synthetic-console-reference',radiometry_epoch=0,
            roi_id='full126',mask_id='FULL_126_RGB',operator_id='FROZEN_FULL_MODEL',feature_basis_id='synthetic-shadow-v1',required_format_version='PSFULL1_PFY')
        normalizer=10. if mode=='dynamic' else max(float(np.mean(self.reference)),1e-3)
        self.normalizer=normalizer
        self.bindings={self.desc['model']['sha256']:dict(context=self.context.data,reference=self.reference,stable_mask=self.stable,
            fast_policy=dict(dynamic['fast_policy'],radiometric_normalizer=normalizer))}
        self.label='合成动态测量' if mode=='dynamic' else 'D11冻结测量 · 合成时间回放'
    def next(self):
        i=self.index;self.index+=1
        if self.mode=='d11':data=self.original
        else:
            phase=i%120;x=None
            if 20<=phase<50:x=42+(phase-20)*.7
            elif 50<=phase<85:x=63.
            elif 85<=phase<100:x=63-(phase-85)*1.2
            a=self.reference.copy()
            if x is not None:a+=1.5*np.exp(-((self.xx-x)**2/(2*8**2)+(self.yy-64)**2/(2*12**2)))[:,:,None]*np.array([.9,1.,1.1])
            a+=np.random.default_rng(20261019+i).normal(0,.02,a.shape)
            data=encode_pfy(a,[1,1,1],self.context.data['model_id'])
        now=time.monotonic_ns();uid=self.session_id+':'+str(i)
        fmt=dict(version=1,name='PSFULL1_PFY',shape=[126,126,3],model_id=self.context.data['model_id'])
        envelope=dict(schema_version=1,source_kind='SYNTHETIC',source_session_id=self.session_id,frame_uid=uid,
            original_capture_session_id=self.session_id,original_sensor_frame_id=str(i),original_block_id=str(i),
            t_frame_complete_mono_ns=str(now),format=fmt,exposure_us=None,gain_raw=None,gain_units=None,
            complete=True,raw_sha256=sha(data),hardware_accessed=False,is_true_motion_sequence=False,
            sample_time_ns=str(i*100_000_000),label=self.label)
        d=dict(self.desc,origin='SYNTHETIC',sample_id=uid,capture=dict(session_id=self.session_id,original_sensor_frame_id=i))
        return Frame('PFY',data,json.dumps(envelope)),d,self.base,i*100_000_000


class ManifestSource:
    def __init__(self,path,workspace):
        path=Path(path);self.base=path.parent;self.spec=read_json(path)
        require(self.spec['schema_version']==1 and self.spec['scope']=='SYNTHETIC_TEST_ONLY','SOURCE_SCOPE','Only registered synthetic manifests supported here')
        self.context=Context.create(**self.spec['context']);self.bindings=load_bindings(self.spec,self.base)
        self.models=self.spec['models'];self.source=FileSource();self.index=0;self.label='合成文件序列';self.normalizer=None
        self.bank=load_bank(self.base/self.spec['bank'],workspace) if self.spec.get('bank') else None
        self.localize_requested=False
        self.frames=self.spec['frames'];require(0<len(self.frames)<=100000,'SOURCE_LENGTH','Bounded nonempty manifest required')
        self.period_ms=100
    def next(self):
        if self.index>=len(self.frames):return None
        row=self.frames[self.index];self.index+=1;d=json.loads(pinned(self.base,row['descriptor']))
        self.localize_requested=bool(row.get('localize'))
        require(d.get('origin')=='SYNTHETIC','SOURCE_SCOPE','No relabeling real frames as synthetic')
        return self.source.load(d,self.base),d,self.base,int(row['sample_time_ns'])
