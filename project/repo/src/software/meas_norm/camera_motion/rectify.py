"""Calibrated same-color backward sampling onto the fixed complete measurement grid."""
import json
import numpy as np
from contracts import require,pinned,sha,encode_pfy
from raw_measurement import validated_raw,calibration_key,diagnostics
from registration import transform_points,core_scope,validated_domain


def bilinear_checked(plane,valid,x,y):
    h,w=plane.shape
    require(np.isfinite(x).all() and np.isfinite(y).all(),'INVALID_SUPPORT','Nonfinite sampling coordinates')
    x0=np.floor(x).astype(np.int64);y0=np.floor(y).astype(np.int64)
    require(np.all(x0>=0) and np.all(y0>=0) and np.all(x0+1<w) and np.all(y0+1<h),'INVALID_SUPPORT','Interpolation support outside sensor; no edge padding')
    require(np.all(valid[y0,x0]&valid[y0+1,x0]&valid[y0,x0+1]&valid[y0+1,x0+1]),'INVALID_SUPPORT','Interpolation requires an invalid/saturated sample')
    dx=x-x0;dy=y-y0
    return (plane[y0,x0]*(1-dx)*(1-dy)+plane[y0,x0+1]*dx*(1-dy)+plane[y0+1,x0]*(1-dx)*dy+plane[y0+1,x0+1]*dx*dy)


def rectify_raw(frame,descriptor,calibration,base,model_hash,registration,motion_contract):
    envelope=frame.envelope
    require(registration.frame_uid==envelope['frame_uid'] and registration.source_session_id==envelope['source_session_id'],
            'TRANSFORM_FRAME','Mapping belongs to another frame/session')
    require(registration.calibration_key==calibration_key(calibration),'TRANSFORM_CALIBRATION','Mapping/calibration mismatch')
    contract=motion_contract
    require(contract is not None,'MISSING_MOTION_CALIBRATION','Geometry fit does not establish forward-model compatibility')
    require(contract['scope']==calibration['scope'] and contract['model_sha256']==model_hash and
            contract['reference_calibration_key']==calibration_key(calibration) and contract['anchor_id']==registration.anchor_id,
            'MOTION_MODEL','Motion contract does not match reference/model')
    require(contract['sampler']=='SAME_COLOR_BILINEAR_AREA_V1' and contract['distortion_model']=='IDENTITY_REGISTERED' and
            contract['forward_equivalence_passed'] is True and contract['independent_holdout_passed'] is True,
            'MOTION_MODEL','Unverified interpolation/optics/forward model')
    require(contract['scope'] in ('SYNTHETIC_TEST_ONLY','REAL_CALIBRATED'),'MOTION_SCOPE','Unknown motion evidence scope')
    require(contract['scope']=='SYNTHETIC_TEST_ONLY' or contract.get('validated_domain') is not None,
            'MOTION_DOMAIN','Real motion requires explicit independently measured bounds')
    domain=validated_domain(contract.get('validated_domain'))
    control_error=registration.diagnostics.get('independent_control_p95_grid_pixels')
    require(type(control_error) in (int,float) and np.isfinite(control_error) and
            0<=control_error<=domain['max_control_p95_grid_pixels'],
            'CONTROL_ERROR','Control error exceeds the model-specific validated domain')
    if envelope['source_kind']!='SYNTHETIC':
        evidence=json.loads(pinned(base,contract['validation']))
        check_value={k:v for k,v in contract.items() if k!='validation'}
        digest=sha(json.dumps(check_value,sort_keys=True,separators=(',',':')).encode())
        require(evidence['contract_sha256']==digest and evidence['scope']=='REAL_CALIBRATED' and evidence['passed'] is True,
                'MOTION_EVIDENCE','Real motion needs independent physical equivalence evidence')
    raw,dark,gain,valid,background=validated_raw(frame,descriptor,calibration,base,model_hash)
    require(tuple(raw.shape)==registration.sensor_shape,'TRANSFORM_FORMAT','Wrong mapping dimensions')
    x,y,rw,rh=calibration['roi_xywh']
    require(all(type(v) is int and v%2==0 for v in (x,y,rw,rh)) and rw>0 and rh>0 and rw%252==rh%252==0,
            'ROI_SUPPORT','Registered integer same-color area grid required')
    corrected=(raw.astype(np.float64)-dark)*gain
    valid=valid&(raw<4095)&np.isfinite(corrected)&(corrected>=0)
    reference_to_current=np.linalg.inv(registration.current_to_reference)
    require(core_scope(reference_to_current,calibration['roi_xywh'])['passed'],'OUTSIDE_CAM_CORE','Mapping exceeds registered joint motion boundary')
    require(core_scope(reference_to_current,calibration['roi_xywh'],domain)['passed'],
            'OUTSIDE_MODEL_DOMAIN','Mapping exceeds independently validated model bounds')
    by,bx=rh//252,rw//252
    channels=[]
    for oy,ox in ((0,0),(0,1),(1,0),(1,1)):
        ys,xs=np.mgrid[0:rh//2,0:rw//2]
        points=np.column_stack(((x+2*xs+ox).ravel(),(y+2*ys+oy).ravel()))
        sampled=transform_points(reference_to_current,points)
        sx=(sampled[:,0]-ox)*.5;sy=(sampled[:,1]-oy)*.5
        plane=bilinear_checked(corrected[oy::2,ox::2],valid[oy::2,ox::2],sx,sy)
        channels.append(plane.reshape(126,by,126,bx).mean(axis=(1,3)))
    weights=np.asarray(calibration['green_weights'],np.float64)
    require(weights.shape==(2,) and np.all(weights>=0) and abs(float(weights.sum())-1)<1e-12,'COLOR_WEIGHTS','Invalid green weights')
    rgb=np.stack((channels[0],weights[0]*channels[1]+weights[1]*channels[2],channels[3]),axis=2)-background
    require(np.isfinite(rgb).all() and np.all(rgb>=0),'NEGATIVE_BACKGROUND','Rectified measurement is negative/nonfinite; no clipping')
    pfy=encode_pfy(rgb,calibration['rgb_scales'],descriptor['model_id']);diag,_=diagnostics(rgb)
    return pfy,dict(measurement_hash=sha(pfy),model_hash=model_hash,model_id=descriptor['model_id'],
        calibration_key=calibration_key(calibration),transform_id=registration.transform_id,
        source_frame_uid=registration.frame_uid,anchor_id=registration.anchor_id,
        scope=contract['scope'],validated_domain=domain,physical_quality_accepted=False,complete_support=True,diagnostics=diag)
