from pathlib import Path
import sys,json,time,copy
REPO=Path(__file__).resolve().parents[2]
for folder in ('src/software/camera_service/production_input','src/software/meas_norm',
               'src/software/full_model/fast_path','src/software/full_model/dual_mode',
               'src/software/meas_norm/camera_motion','src/software/model_loader/localization'):
    sys.path.insert(0,str(REPO/folder))
import numpy as np
from contracts import read_json,encode_pfy,decode_pfy,sha
from inputs import historical_case
from messages import Context,Measurement

CONFIG=read_json(REPO/'configs/offline_engineering.json')
WORKSPACE=Path(CONFIG['workspace_hint'])
D11,_,_=historical_case(CONFIG,WORKSPACE,'D11_rgb')

DYNAMIC_CONFIG=read_json(REPO/'configs/offline_dynamic.json')
FAST=DYNAMIC_CONFIG['fast_policy']
POLICY=DYNAMIC_CONFIG['scheduler_policy']
yy,xx=np.mgrid[:126,:126]
REFERENCE=np.stack([10+.005*xx+.003*yy]*3,axis=2)
STABLE=(xx<16)|(xx>109)|(yy<16)|(yy>109)

def context(session='synthetic-session',model=None):
    d=model or D11
    return Context.create(source_session_id=session,compat_generation=0,geometry_epoch=0,model_id=d['model_id'],
        model_hash=d['model']['sha256'],model_epoch=0,calibration_id='synthetic_reference',radiometry_epoch=0,
        roi_id='full126',mask_id='FULL_126_RGB',operator_id='FROZEN_FULL_MODEL',feature_basis_id='synthetic_shadow_v1',
        required_format_version='PSFULL1_PFY')

def scene(x=None,amplitude=1.5,noise=0.,seed=0,light=0.):
    a=REFERENCE.copy()+light
    if x is not None:
        shape=amplitude*np.exp(-((xx-x)**2/(2*8**2)+(yy-64)**2/(2*12**2)))
        a+=shape[:,:,None]*np.array([.9,1.,1.1])
    if noise:a+=np.random.default_rng(seed).normal(0,noise,a.shape)
    return a

def measurement(index,x=None,ctx=None,now=None,pfy=None,**kwargs):
    ctx=ctx or context()
    now=index*100_000_000 if now is None else now
    return Measurement(pfy or encode_pfy(scene(x,**kwargs),[1,1,1],ctx.data['model_id']),ctx,
                       ctx.data['source_session_id']+':'+str(index),index,now,'SYNTHETIC',now)

class Clock:
    def __init__(self):self.ns=0
    def __call__(self):return self.ns
    def step(self,ms=100):self.ns+=int(ms*1e6);return self.ns

def valid_backend(req):
    return dict(status='COMPLETE',state_acceptance=True,measurement_sha256=sha(req.measurement.pfy),
                model_hash=req.measurement.context.data['model_hash'])

def warm(controller,detector,clock,count=4,start=0):
    event=None
    for i in range(start,start+count):
        now=clock.step();m=measurement(i,ctx=controller.context,now=now)
        event=controller.ingest(m,detector)
    return m,event
