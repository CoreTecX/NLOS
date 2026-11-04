"""Explicit synthetic Bayer fixture; never a camera calibration or physical truth."""
from pathlib import Path
import copy
import json
import sys

REPO=Path(__file__).resolve().parents[2]
for folder in ('src/software/camera_service/production_input','src/software/meas_norm'):
    sys.path.insert(0,str(REPO/folder))
import numpy as np
from contracts import read_json,sha,decode_pfy,encode_pfy
from inputs import historical_case
from raw_measurement import calibration_key


def save_json(path,value):
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n','utf8')


def record(path,relative=True):
    return dict(path=path.name if relative else str(path),bytes=path.stat().st_size,sha256=sha(path.read_bytes()))


def certify(base,c):
    evidence=dict(scope=c['scope'],passed=True,calibration_key=calibration_key(c),model_sha256=c['model_sha256'],
                  meaning='Synthetic scalar-oracle fixture only; no geometry or radiometry of a real camera has been measured')
    save_json(base/'validation.json',evidence)
    c['validation']=record(base/'validation.json')
    save_json(base/'calibration.json',c)


def create(base,config,workspace):
    base=Path(base);base.mkdir(parents=True,exist_ok=False)
    desc,_,_=historical_case(config,workspace,'D11_rgb')
    _,scales,rgb=decode_pfy(Path(desc['file']['path']).read_bytes())
    level=np.rint(rgb/rgb.max()*2800).astype(np.uint16)+100
    h=w=504
    raw=np.empty((h,w),dtype='<u2')
    for oy,ox,ch in ((0,0,0),(0,1,1),(1,0,1),(1,1,2)):
        raw[oy::2,ox::2]=np.repeat(np.repeat(level[:,:,ch],2,axis=0),2,axis=1)+4
    # Two green planes have different raw levels and radiometric gains.
    raw[1::2,0::2]=(raw[1::2,0::2]-4)//2+4
    dark=np.full((h,w),4.,dtype=np.float64)
    gain=np.ones((h,w),dtype=np.float64);gain[1::2,0::2]=2.
    valid=np.ones((h,w),dtype=bool)
    background=np.full((126,126,3),50.,dtype=np.float64)
    np.savez(base/'corrections.npz',dark=dark,flat_gain=gain,valid=valid,background=background)
    (base/'frame.raw').write_bytes(raw.tobytes())
    fmt=dict(width=w,height=h,pixel_format='BayerRG12',dtype='<u2',bayer_pattern='RGGB',valid_bits=12,
             row_stride_bytes=w*2,roi_sensor_offset=[0,0],binning=[1,1],orientation='identity_RGGB_confirmed')
    cap=dict(session_id='SYNTHETIC_FIXTURE',original_sensor_frame_id=1,original_block_id=1,
             camera_timestamp_ticks=None,camera_tick_frequency_hz=None,exposure_us=5000,gain_raw=64,gain_units=None)
    c=dict(schema_version=1,scope='SYNTHETIC_TEST_ONLY',operator_id='rggb_static_area_mean_v1',
           calibration_id='synthetic_fixture_v1',geometry_id='synthetic_static_grid',roi_id='synthetic_roi',
           radiometry_id='synthetic_dark_flat',mask_id='FULL_126_RGB',negative_policy='REJECT',
           normalization='LEGACY_SOLVER_DS',linear_response_verified=True,model_id=desc['model_id'],
           model_sha256=desc['model']['sha256'],sensor={k:fmt[k] for k in ('width','height','pixel_format','bayer_pattern','roi_sensor_offset','binning','orientation')},
           capture=dict(exposure_us=5000,gain_raw=64),arrays=record(base/'corrections.npz'),
           roi_xywh=[0,0,w,h],green_weights=[.25,.75],rgb_scales=scales.tolist())
    certify(base,c)
    rawdesc=dict(schema_version=1,kind='RAW',sample_id='synthetic_bayer_from_historical_D11',complete=True,
                 origin='SYNTHETIC',file=record(base/'frame.raw'),format=fmt,capture=cap,
                 model=desc['model'],model_id=desc['model_id'],calibration=record(base/'calibration.json'))
    save_json(base/'input.json',rawdesc)
    # Scalar oracle sums each sensor sample explicitly, independent of reshape/mean implementation.
    expected=np.empty((126,126,3),dtype=np.float64)
    for y in range(126):
        for x in range(126):
            channels=[]
            for oy,ox in ((0,0),(0,1),(1,0),(1,1)):
                total=0.
                for yy in range(2):
                    for xx in range(2):
                        sy,sx=4*y+2*yy+oy,4*x+2*xx+ox
                        total+=(int(raw[sy,sx])-float(dark[sy,sx]))*float(gain[sy,sx])
                channels.append(total/4.)
            expected[y,x]=[channels[0]-50,.25*channels[1]+.75*channels[2]-50,channels[3]-50]
    np.save(base/'expected_rgb.npy',expected,allow_pickle=False)
    (base/'expected.pfy').write_bytes(encode_pfy(expected,scales,desc['model_id']))
    direct=copy.deepcopy(desc);direct.pop('golden');direct['file']=record(base/'expected.pfy');direct['sample_id']='synthetic_scalar_oracle_pfy';direct['origin']='SYNTHETIC'
    save_json(base/'direct_pfy.json',direct)
    return rawdesc,c,expected


if __name__=='__main__':
    create(Path(sys.argv[1]),read_json(REPO/'configs/offline_engineering.json'),sys.argv[2])
