"""Static, registered Bayer area integration; motion/warp belongs to O06."""
from io import BytesIO
import json
import time

import numpy as np

from contracts import FileSource, require, pinned, sha, encode_pfy, validate_measurement


def calibration_key(calibration):
    # The evidence binds all conversion parameters and array hashes, not its own hash.
    value = {k:v for k,v in calibration.items() if k != 'validation'}
    return sha(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode())


def diagnostics(rgb):
    scales = np.ones(3)
    ds = validate_measurement(rgb, scales)
    flat = rgb.reshape(-1,3,order='F')
    rows = np.arange(126*126)
    vertical = flat[rows%126 != 125] - flat[(rows[rows%126 != 125])+1]
    horizontal = flat[:-126] - flat[126:]
    difference = np.concatenate((vertical,horizontal),axis=0) / np.sqrt(2.) / ds
    cross = rgb[1:,1:] - rgb[:-1,1:] - rgb[1:,:-1] + rgb[:-1,:-1]
    center = np.median(cross,axis=(0,1))
    noise = np.median(np.abs(cross-center),axis=(0,1))/(2*.67448975)
    return dict(legacy_author_ds=ds, spatial2d_shape=list(difference.shape),
        normalized_difference_l2=float(np.linalg.norm(difference)), noise_mad_rgb=noise.tolist(),
        normalization='Original solver author-ds; no per-pixel whitening or display stretch',
        noise_scope='Diagnostic only, not calibrated SNR or confidence'), difference


def validated_raw(frame, descriptor, calibration, base, model_hash):
    require(frame.kind == 'RAW', 'INPUT_KIND', 'RAW input required')
    require(calibration is not None, 'MISSING_CALIBRATION', 'RAW needs registered geometry, radiometry and a matched model')
    envelope = frame.envelope
    require(envelope['raw_sha256'] == sha(frame.data) and envelope['format'] == descriptor['format'] and
            envelope['exposure_us'] == descriptor['capture'].get('exposure_us') and
            envelope['gain_raw'] == descriptor['capture'].get('gain_raw'),
            'FRAME_CONTEXT', 'RAW bytes and conversion context must belong to the validated frame')
    c = calibration
    require(c['schema_version'] == 1 and c['operator_id'] == 'rggb_static_area_mean_v1',
            'CALIBRATION_SCHEMA', 'Unsupported calibration/operator; motion and masked operators are not implemented')
    synthetic = frame.envelope['source_kind'] == 'SYNTHETIC'
    require(c['scope'] == ('SYNTHETIC_TEST_ONLY' if synthetic else 'REAL_CALIBRATED'),
            'CALIBRATION_SCOPE', 'Synthetic calibration cannot authorize a real camera frame')
    require(c['model_sha256'] == model_hash and c['model_id'] == descriptor['model_id'],
            'CALIBRATION_MODEL', 'Calibration/model fingerprint mismatch')
    for key in ('calibration_id','geometry_id','roi_id','radiometry_id','mask_id'):
        require(isinstance(c[key],str) and bool(c[key]), 'CALIBRATION_ID', 'Missing '+key)
    require(c['mask_id'] == 'FULL_126_RGB' and c['negative_policy'] == 'REJECT' and
            c['normalization'] == 'LEGACY_SOLVER_DS' and c['linear_response_verified'] is True,
            'MEASUREMENT_CONTRACT', 'Full support, linear radiometry, nonnegative measurement and original normalization required')
    fmt, capture = descriptor['format'], descriptor['capture']
    sensor = c['sensor']
    for key in ('width','height','pixel_format','bayer_pattern','roi_sensor_offset','binning','orientation'):
        require(fmt.get(key) is not None and fmt[key] == sensor[key], 'SENSOR_GEOMETRY', 'Unregistered/mismatched '+key)
    require(sensor['orientation'] == 'identity_RGGB_confirmed', 'SENSOR_GEOMETRY', 'Bayer orientation is not registered')
    for key in ('exposure_us','gain_raw'):
        require(capture.get(key) is not None and capture[key] == c['capture'][key], 'RADIOMETRY', 'Exposure/gain outside exact calibration')
    evidence = json.loads(pinned(base, c['validation']))
    require(evidence['calibration_key'] == calibration_key(c) and evidence['passed'] is True,
            'CALIBRATION_EVIDENCE', 'Missing/mismatched calibration validation')
    require(evidence['scope'] == c['scope'] and evidence['model_sha256'] == model_hash,
            'CALIBRATION_EVIDENCE', 'Validation scope/model mismatch')
    if not synthetic:
        require(envelope.get('camera_identity') is not None and
                envelope['camera_identity']==c.get('camera_identity') and
                descriptor.get('camera_identity')==c.get('camera_identity'),
                'CALIBRATION_CAMERA','Real calibration must bind camera model, serial and MAC')
        require(envelope.get('sensor_test_pattern_off') is True,'TEST_PATTERN','Real scene capture must have test pattern disabled')
        require(all(evidence.get(k) is True for k in ('geometry_passed','radiometry_passed','forward_model_passed','independent_holdout_passed')),
                'CALIBRATION_EVIDENCE', 'Real calibration requires geometry/radiometry/model/holdout acceptance')
    raw = FileSource.decode_raw(frame.data, fmt)
    h, w = raw.shape
    with np.load(BytesIO(pinned(base, c['arrays'])), allow_pickle=False) as arr:
        require(set(arr.files) == {'dark','flat_gain','valid','background'}, 'CALIBRATION_ARRAYS', 'Expected dark, flat_gain, valid, background')
        dark, gain, valid, background = [arr[k].copy() for k in ('dark','flat_gain','valid','background')]
    require(dark.shape == gain.shape == valid.shape == (h,w) and background.shape == (126,126,3),
            'CALIBRATION_SHAPE', 'Calibration arrays do not match sensor/full measurement')
    require(valid.dtype == np.bool_ and np.isfinite(dark).all() and np.isfinite(gain).all() and
            np.isfinite(background).all() and np.all(gain>0) and np.all(dark>=0) and np.all(background>=0),
            'CALIBRATION_ARRAYS', 'Invalid correction arrays')
    return raw, dark, gain, valid, background


def convert_raw(frame, descriptor, calibration, base, model_hash):
    raw,dark,gain,valid,background = validated_raw(frame,descriptor,calibration,base,model_hash)
    c=calibration
    h,w=raw.shape
    x,y,rw,rh = c['roi_xywh']
    require(all(type(v) is int and v%2==0 for v in (x,y,rw,rh)) and x>=0 and y>=0 and rw>0 and rh>0 and
            x+rw<=w and y+rh<=h and rw%252==rh%252==0,
            'ROI_SUPPORT', 'Aligned full-cell rectangular ROI required; no resize, padding or partial cells')
    selected = raw[y:y+rh,x:x+rw]
    require(valid[y:y+rh,x:x+rw].all(), 'INVALID_SUPPORT', 'A required RAW sample is invalid')
    require(np.all(selected < 4095), 'SATURATED_SUPPORT', 'A required RAW sample is saturated')
    corrected = (selected.astype(np.float64)-dark[y:y+rh,x:x+rw])*gain[y:y+rh,x:x+rw]
    require(np.isfinite(corrected).all() and np.all(corrected>=0), 'NEGATIVE_CORRECTION', 'Dark correction is negative/nonfinite; no clipping allowed')
    by,bx = rh//252,rw//252
    def area(oy,ox):
        plane = corrected[oy::2,ox::2]
        return plane.reshape(126,by,126,bx).mean(axis=(1,3), dtype=np.float64)
    weights = np.asarray(c['green_weights'],dtype=np.float64)
    require(weights.shape==(2,) and np.isfinite(weights).all() and np.all(weights>=0) and
            abs(float(weights.sum())-1.)<1e-12, 'COLOR_WEIGHTS', 'Registered nonnegative green weights must sum to one')
    rgb = np.stack((area(0,0),weights[0]*area(0,1)+weights[1]*area(1,0),area(1,1)),axis=2) - background
    require(np.isfinite(rgb).all() and np.all(rgb>=0), 'NEGATIVE_BACKGROUND', 'Background subtraction is negative/nonfinite; model contract cannot represent it')
    scales = np.asarray(c['rgb_scales'],dtype=np.float64)
    pfy = encode_pfy(rgb,scales,descriptor['model_id'])
    diag,difference = diagnostics(rgb)
    context = dict(model_id=descriptor['model_id'],model_hash=model_hash,
        calibration_id=c['calibration_id'],calibration_key=calibration_key(c),geometry_id=c['geometry_id'],
        operator_id=c['operator_id'],roi_id=c['roi_id'],mask_id=c['mask_id'],radiometry_id=c['radiometry_id'],
        measurement_format='PSFULL1_PFY',measurement_hash=sha(pfy),
        measurement_id=sha(frame.data+pfy),source_frame_uids=[frame.envelope['frame_uid']],k=1,
        t_measurement_ready_mono_ns=str(time.monotonic_ns()), calibration_scope=c['scope'],
        physical_quality_accepted=False,diagnostics=diag)
    return pfy,context,difference
