"""Dark/flat preparation with provenance and explicit real-calibration gates.

All generated real packages remain DRAFT. Physical geometry, linear response,
forward model and independent holdout validation are separate measured inputs.
Python 3.7+, NumPy; no camera or board control.
"""
from pathlib import Path
import argparse
import json
import sys
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'camera_service/production_input'))
from contracts import FileSource, require, pinned, read_json, sha, write_json, load_model
from raw_measurement import calibration_key


def record(path):
    path=Path(path)
    return dict(path=path.name,bytes=path.stat().st_size,sha256=sha(path.read_bytes()))


def estimate_arrays(darks, flats, background, dark_std_limit=8., min_flat_signal=64.):
    darks=np.asarray(darks,dtype=np.float64);flats=np.asarray(flats,dtype=np.float64)
    require(darks.ndim==3 and flats.ndim==3 and darks.shape[1:]==flats.shape[1:] and
            darks.shape[0]>=4 and flats.shape[0]>=4,'CALIBRATION_SAMPLES','At least four matching dark and flat frames required')
    require(all(np.isfinite(a).all() and np.all(a>=0) and np.all(a<=4095) for a in (darks,flats)),
            'CALIBRATION_RANGE','Input samples must be valid RAW12')
    require(np.isfinite(dark_std_limit) and dark_std_limit>0 and np.isfinite(min_flat_signal) and min_flat_signal>0,
            'CALIBRATION_POLICY','Invalid noise/signal limits')
    dark=np.median(darks,axis=0);signal=np.median(flats,axis=0)-dark
    noise=np.std(darks,axis=0,ddof=1)
    valid=(noise<=dark_std_limit)&(signal>=min_flat_signal)&np.all(flats<4095,axis=0)&np.all(darks<4095,axis=0)
    gain=np.ones_like(dark)
    # Normalize each Bayer plane independently to retain inter-channel response.
    targets=[]
    for oy,ox in ((0,0),(0,1),(1,0),(1,1)):
        mask=valid[oy::2,ox::2];plane=signal[oy::2,ox::2]
        require(mask.any(),'FLAT_SIGNAL','A Bayer plane contains no valid flat samples')
        target=float(np.median(plane[mask]));targets.append(target)
        g=gain[oy::2,ox::2];g[mask]=target/plane[mask]
    background=np.asarray(background,dtype=np.float64)
    require(background.shape==(126,126,3) and np.isfinite(background).all() and np.all(background>=0),
            'BACKGROUND','An explicit nonnegative full measurement background is required')
    return dict(dark=dark,flat_gain=gain,valid=valid,background=background),dict(
        dark_frames=len(darks),flat_frames=len(flats),invalid_pixels=int((~valid).sum()),
        saturated_pixels=int(np.any(flats>=4095,axis=0).sum()),max_dark_std=float(noise.max()),
        per_bayer_signal_median=targets,invalid_pixel_policy='REJECT_REQUIRED_SUPPORT_NO_INTERPOLATION',
        flat_assumption='Uniform target and fixed exposure/gain; not proof of sensor linearity')


def prepare(manifest_path,output):
    manifest_path=Path(manifest_path);base=manifest_path.parent;spec=read_json(manifest_path)
    output=Path(output);require(not output.exists(),'OUTPUT_EXISTS','Choose a new calibration directory')
    require(spec['schema_version']==1 and spec['scope'] in ('SYNTHETIC_TEST_ONLY','REAL_CAPTURE'),
            'CALIBRATION_SCOPE','Explicit source scope required')
    synthetic=spec['scope']=='SYNTHETIC_TEST_ONLY';fmt=spec['format'];capture=spec['capture']
    model,mid=load_model(spec,base)
    require(spec['model_id']==mid,'CALIBRATION_MODEL','Wrong model identity')
    rows={};sources=[]
    for kind in ('dark','flat'):
        rows[kind]=[];seen=set()
        require(4<=len(spec[kind])<=128,'CALIBRATION_SAMPLES','Use 4 to 128 dark/flat samples')
        for item in spec[kind]:
            desc_path=(base/item['path']).resolve();desc=json.loads(pinned(base,item));frame=FileSource().load(desc,desc_path.parent)
            require(frame.kind=='RAW' and (frame.envelope['source_kind']=='SYNTHETIC')==synthetic,
                    'CALIBRATION_SCOPE','Synthetic and real sources cannot be mixed or relabeled')
            require(desc['format']==fmt and all(desc['capture'].get(k)==capture.get(k) for k in ('exposure_us','gain_raw')),
                    'CALIBRATION_CONTEXT','Dark/flat configuration differs')
            if not synthetic:
                require(desc.get('camera_identity')==spec.get('camera_identity') and spec.get('camera_identity') is not None and
                        frame.envelope.get('sensor_test_pattern_off') is True,'CALIBRATION_CAMERA','Captured identity/test-pattern state missing')
            identity=(desc['capture'].get('session_id'),desc['capture'].get('original_sensor_frame_id'))
            require(identity not in seen and None not in identity,'CALIBRATION_SAMPLES','Repeated or missing capture identity')
            seen.add(identity)
            rows[kind].append(FileSource.decode_raw(frame.data,fmt).copy())
            sources.append(dict(role=kind,raw_sha256=sha(frame.data),capture_session=identity[0],frame_id=identity[1],descriptor_sha256=item['sha256']))
    bgbytes=pinned(base,spec['background']);import io
    background=np.load(io.BytesIO(bgbytes),allow_pickle=False)
    arrays,diagnostic=estimate_arrays(rows['dark'],rows['flat'],background,**spec.get('estimator_policy',{}))
    x,y,w,h=spec['roi_xywh']
    require(all(type(v) is int and v%2==0 for v in (x,y,w,h)) and x>=0 and y>=0 and w>0 and h>0 and
            x+w<=fmt['width'] and y+h<=fmt['height'] and w%252==h%252==0,'ROI_SUPPORT','Complete aligned 126x126 Bayer measurement support required')
    require(arrays['valid'][y:y+h,x:x+w].all(),'INVALID_SUPPORT','Invalid pixels inside required model support; adjust geometry/model or recapture')
    require(spec.get('linear_response_verified') is True if synthetic else spec.get('linear_response_verified') is False,
            'LINEARITY_SCOPE','Real response must be validated separately; synthetic fixture must declare its known linear construction')
    output.mkdir(parents=True)
    np.savez(output/'corrections.npz',**arrays)
    c=dict(schema_version=1,scope='SYNTHETIC_TEST_ONLY' if synthetic else 'DRAFT_REAL_CALIBRATION',
        operator_id='rggb_static_area_mean_v1',calibration_id=spec['calibration_id'],geometry_id=spec['geometry_id'],
        roi_id=spec['roi_id'],radiometry_id=spec['radiometry_id'],mask_id='FULL_126_RGB',negative_policy='REJECT',
        normalization='LEGACY_SOLVER_DS',linear_response_verified=synthetic,model_id=mid,model_sha256=sha(model),
        sensor={k:fmt[k] for k in ('width','height','pixel_format','bayer_pattern','roi_sensor_offset','binning','orientation')},
        capture={k:capture[k] for k in ('exposure_us','gain_raw')},arrays=record(output/'corrections.npz'),
        roi_xywh=spec['roi_xywh'],green_weights=spec['green_weights'],rgb_scales=spec['rgb_scales'],
        source_scope=spec['scope'],source_frames=sources,camera_identity=spec.get('camera_identity'),
        source_manifest_sha256=sha(manifest_path.read_bytes()))
    ev=dict(scope=c['scope'],passed=synthetic,calibration_key=calibration_key(c),model_sha256=c['model_sha256'],
            meaning='Synthetic fixture only; real packages require four independent measured validation reports')
    write_json(output/'validation.json',ev);c['validation']=record(output/'validation.json')
    write_json(output/'calibration.json',c);write_json(output/'diagnostics.json',diagnostic)
    return c,diagnostic


def validate_real_evidence(draft,evidence_specs,base):
    require(draft['scope']=='DRAFT_REAL_CALIBRATION' and draft['source_scope']=='REAL_CAPTURE',
            'CALIBRATION_SCOPE','Only real-capture drafts can receive physical validation')
    training={f['raw_sha256'] for f in draft['source_frames']}
    reports={}
    for role in ('geometry','radiometry','forward_model','independent_holdout'):
        ev=json.loads(pinned(base,evidence_specs[role]))
        require(ev.get('scope')=='REAL_MEASURED' and ev.get('passed') is True and
                ev.get('draft_key')==calibration_key(draft) and ev.get('camera_identity')==draft['camera_identity'] and
                ev.get('model_sha256')==draft['model_sha256'] and ev.get('role')==role,
                'CALIBRATION_EVIDENCE','Physical evidence identity/scope/key mismatch: '+role)
        require(bool(ev.get('measurements')) and bool(ev.get('capture_records')),
                'CALIBRATION_EVIDENCE','Measured values and captured files are required: '+role)
        hashes=[]
        for rec in ev['capture_records']:
            data=pinned(base,rec);hashes.append(sha(data))
        if role=='independent_holdout':
            require(training.isdisjoint(hashes),'HOLDOUT_LEAKAGE','Holdout reuses dark/flat training data')
        reports[role]=ev
    return reports


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--manifest',required=True);parser.add_argument('--output',required=True)
    args=parser.parse_args();cal,diag=prepare(args.manifest,args.output)
    print(json.dumps(dict(scope=cal['scope'],calibration_id=cal['calibration_id'],diagnostics=diag),indent=2))
