from pathlib import Path
import json
import numpy as np
from support import *
from contracts import FileSource
from raw_measurement import calibration_key
from registration import FixedAnchor,transform_points

H=W=756
ROI=[126,126,504,504]

def write_json(path,value):path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n','utf8')
def record(path):return dict(path=path.name,bytes=path.stat().st_size,sha256=sha(path.read_bytes()))

def similarity(dx=0.,dy=0.,angle=0.,scale=1.):
    theta=np.radians(angle);a=scale*np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
    center=np.array([378.,378.]);t=center-a@center+np.array([dx,dy])
    return np.vstack((np.column_stack((a,t)),[0,0,1.]))

def field(x,y,c,target=1.):
    # Stable wall texture is outside the sensing ROI; target modulation stays inside it.
    texture=180*np.sin(x*.13)*np.cos(y*.17)+110*np.sin(x*.075+y*.11)
    baseline=900+60*np.sin(x/41)+45*np.cos(y/33)
    outside=(x<118)|(x>638)|(y<118)|(y>638)
    blob=target*(300+70*c)*np.exp(-((x-(270+90*c))**2/(2*65**2)+(y-385)**2/(2*85**2)))
    return baseline+texture*outside+blob

def generate(H_ref_to_cur,target=1.):
    inv=np.linalg.inv(H_ref_to_cur);raw=np.empty((H,W),dtype='<u2')
    for oy,ox,c in ((0,0,0),(0,1,1),(1,0,1),(1,1,2)):
        yy,xx=np.mgrid[oy:H:2,ox:W:2];xy=transform_points(inv,np.column_stack((xx.ravel(),yy.ravel())))
        values=field(xy[:,0],xy[:,1],c,target)+4
        raw[oy::2,ox::2]=np.rint(values.reshape(yy.shape)).astype('<u2')
    return raw

def create_fixture(root):
    root=Path(root);root.mkdir(exist_ok=False,parents=True)
    reference=generate(np.eye(3));np.save(root/'reference_raw.npy',reference,allow_pickle=False)
    valid=np.ones((H,W),bool)
    np.savez(root/'corrections.npz',dark=np.full((H,W),4.),flat_gain=np.ones((H,W)),valid=valid,background=np.zeros((126,126,3)))
    fmt=dict(width=W,height=H,pixel_format='BayerRG12',dtype='<u2',bayer_pattern='RGGB',valid_bits=12,row_stride_bytes=W*2,
             roi_sensor_offset=[0,0],binning=[1,1],orientation='identity_RGGB_confirmed')
    cal=dict(schema_version=1,scope='SYNTHETIC_TEST_ONLY',operator_id='rggb_static_area_mean_v1',
        calibration_id='synthetic_motion_reference',geometry_id='synthetic_planar_wall',roi_id='fixed_reference_roi',
        radiometry_id='synthetic_linear',mask_id='FULL_126_RGB',negative_policy='REJECT',normalization='LEGACY_SOLVER_DS',
        linear_response_verified=True,model_id=D11['model_id'],model_sha256=D11['model']['sha256'],
        sensor={k:fmt[k] for k in ('width','height','pixel_format','bayer_pattern','roi_sensor_offset','binning','orientation')},
        capture=dict(exposure_us=5000,gain_raw=64),arrays=record(root/'corrections.npz'),roi_xywh=ROI,
        green_weights=[.5,.5],rgb_scales=[1,1,1])
    write_json(root/'validation.json',dict(scope=cal['scope'],passed=True,calibration_key=calibration_key(cal),model_sha256=cal['model_sha256']))
    cal['validation']=record(root/'validation.json');write_json(root/'calibration.json',cal)
    mask=np.zeros((H//2,W//2),bool);mask[15:42,15:-15]=True;mask[-42:-15,15:-15]=True
    mask[15:-15,15:42]=True;mask[15:-15,-42:-15]=True
    anchor=FixedAnchor(reference,mask,ROI,calibration_key(cal),dict(tracking_low=450,tracking_high=1450,minimum_inliers=12))
    cap=dict(session_id='synthetic_motion',original_sensor_frame_id=0,original_block_id=0,camera_timestamp_ticks=None,
             camera_tick_frequency_hz=None,exposure_us=5000,gain_raw=64,gain_units=None)
    desc=dict(schema_version=1,kind='RAW',sample_id='synthetic_motion',origin='SYNTHETIC',complete=True,format=fmt,
              capture=cap,model=D11['model'],model_id=D11['model_id'],calibration=record(root/'calibration.json'))
    contract=dict(scope='SYNTHETIC_TEST_ONLY',model_sha256=D11['model']['sha256'],reference_calibration_key=calibration_key(cal),
                  anchor_id=anchor.anchor_id,sampler='SAME_COLOR_BILINEAR_AREA_V1',distortion_model='IDENTITY_REGISTERED',
                  forward_equivalence_passed=True,independent_holdout_passed=True,
                  evidence_scope='Synthetic analytic wall only; real forward equivalence not measured')
    write_json(root/'motion_contract.json',contract)
    return reference,desc,cal,anchor,contract

def frame_for(root,source,desc,raw,index):
    p=Path(root)/f'frame_{index}.raw';p.write_bytes(raw.tobytes())
    d=copy.deepcopy(desc);d['file']=record(p);d['capture']['original_sensor_frame_id']=index;d['capture']['original_block_id']=index
    return source.load(d,root),d
