"""Frozen-background shadow features, presence hysteresis and registered direction."""
import time
import numpy as np
from contracts import require


class FastDetector:
    def __init__(self,reference,stable_mask,policy):
        reference=np.asarray(reference,dtype=np.float64)
        stable_mask=np.asarray(stable_mask,dtype=bool)
        require(reference.shape==(126,126,3) and stable_mask.shape==(126,126) and stable_mask.sum()>=100 and (~stable_mask).sum()>=100,
                'FAST_REFERENCE','Full RGB reference and independent stable area required')
        require(np.isfinite(reference).all() and np.all(reference>=0),'FAST_REFERENCE','Invalid baseline')
        self.reference=np.frombuffer(reference.tobytes(),np.float64).reshape(reference.shape)
        self.stable=np.frombuffer(stable_mask.tobytes(),bool).reshape(stable_mask.shape)
        self.policy=dict(policy)
        for k in ('presence_high','presence_low','gradient_min','max_illumination_shift','direction_min_pixels','ttl_ms','gap_ms'):
            require(type(policy[k]) in (int,float) and np.isfinite(policy[k]) and policy[k]>0,'FAST_POLICY',k)
        require(policy['presence_high']>policy['presence_low'] and type(policy['confirm_frames']) is int and policy['confirm_frames']>=1,'FAST_POLICY','Invalid hysteresis')
        require(type(policy['noise_floor']) in (int,float) and np.isfinite(policy['noise_floor']) and policy['noise_floor']>=0,
                'FAST_POLICY','Invalid noise floor')
        require(type(policy['allow_additive_correction']) is bool,'FAST_POLICY','Correction must be explicitly enabled or disabled')
        self.stable_limit=policy.get('max_stable_residual',policy['presence_high'])
        require(type(self.stable_limit) in (int,float) and np.isfinite(self.stable_limit) and self.stable_limit>0,
                'FAST_POLICY','Invalid stable-reference residual limit')
        self.scene_limit=policy.get('scene_change_high',.002)
        require(type(self.scene_limit) in (int,float) and np.isfinite(self.scene_limit) and self.scene_limit>0,
                'FAST_POLICY','Invalid accumulated scene-change limit')
        require(policy['scope'] in ('SYNTHETIC_TEST_ONLY','REAL_CALIBRATED'),'FAST_POLICY','Missing validation scope')
        require(policy['direction_polarity'] in (-1,1,None),'FAST_POLICY','Direction must be registered')
        self.normalizer=float(policy['radiometric_normalizer'])
        require(np.isfinite(self.normalizer) and self.normalizer>0,'FAST_POLICY','Invalid fixed scale')
        self.context=None;self.reference_context=None;self.last_sequence=-1;self.last_time=None;self.last_centroid=None;self.previous=None
        self.presence='unknown';self.last_confirmed='unknown';self.high_count=0;self.low_count=0
        self.processed=0;self.duplicates=0;self.invalid=0;self.scene_anchor=None

    def reset(self,context=None):
        self.context=context;self.last_sequence=-1;self.last_time=None;self.last_centroid=None;self.previous=None
        self.presence='unknown';self.last_confirmed='unknown';self.high_count=0;self.low_count=0
        self.scene_anchor=None

    def unknown(self,m,reason,now_ns):
        self.invalid+=1;self.presence='unknown';self.high_count=self.low_count=0;self.previous=None;self.last_centroid=None
        self.scene_anchor=None
        return dict(frame_uid=m.frame_uid,capture_key=list(m.capture_key),valid=False,reason=reason,presence='unknown',
            transition='NONE',direction='unknown',score=None,score_type='UNCALIBRATED_SHADOW_FEATURE_SCORE',
            confidence_probability=None,activity=None,context=m.context.data,publish_ns=str(now_ns))

    def process(self,m,valid=True,now_ns=None):
        start=time.perf_counter_ns();now_ns=time.monotonic_ns() if now_ns is None else now_ns
        if m.k!=1:return self.unknown(m,'FAST_REQUIRES_SINGLE_FRAME',now_ns)
        # A model/geometry/radiometry change needs a newly registered reference.
        binding=tuple(m.context.data[k] for k in ('geometry_epoch','model_id','model_hash','calibration_id',
                    'radiometry_epoch','roi_id','mask_id','operator_id','feature_basis_id'))
        if self.reference_context is None:self.reference_context=binding
        if self.reference_context!=binding:return self.unknown(m,'FAST_REFERENCE_CONTEXT',now_ns)
        if self.context!=m.context.key:self.reset(m.context.key)
        if m.sensor_sequence<=self.last_sequence:
            self.duplicates+=1
            return dict(frame_uid=m.frame_uid,valid=False,reason='DUPLICATE_OR_OUT_OF_ORDER',presence='unknown',transition='NONE',direction='unknown')
        old_time=self.last_time;self.last_sequence=m.sensor_sequence;self.last_time=m.sample_time_ns
        if not valid:return self.unknown(m,'INPUT_INVALID',now_ns)
        if self.policy['scope']=='SYNTHETIC_TEST_ONLY' and m.source_kind!='SYNTHETIC':return self.unknown(m,'UNCALIBRATED_REAL_INPUT',now_ns)
        if now_ns<m.frame_complete_ns:return self.unknown(m,'CLOCK_DOMAIN_MISMATCH',now_ns)
        if now_ns-m.frame_complete_ns>self.policy['ttl_ms']*1e6:return self.unknown(m,'EXPIRED',now_ns)
        if old_time is not None and (m.sample_time_ns<=old_time or m.sample_time_ns-old_time>self.policy['gap_ms']*1e6):
            return self.unknown(m,'CAPTURE_GAP_OR_CLOCK',now_ns)
        rgb=m.rgb
        residual=(rgb-self.reference)/self.normalizer
        shift=np.median(residual[self.stable],axis=0)
        if np.max(np.abs(shift))>self.policy['max_illumination_shift']:
            return self.unknown(m,'ILLUMINATION_OUT_OF_RANGE',now_ns)
        stable_error=float(np.percentile(np.abs(residual[self.stable]-shift),95))
        if stable_error>self.stable_limit:
            return self.unknown(m,'STABLE_REFERENCE_CHANGED',now_ns)
        # Only a registered stable reference region may estimate additive illumination.
        if self.policy['allow_additive_correction']:residual=residual-shift
        elif np.max(np.abs(shift))>self.policy['presence_low']:return self.unknown(m,'UNREGISTERED_ILLUMINATION',now_ns)
        gray=residual.mean(axis=2)
        signal=gray[~self.stable]
        score=float(np.sqrt(np.mean(signal*signal)))
        gradient=float(np.sqrt((np.mean(np.diff(gray,axis=0)**2)+np.mean(np.diff(gray,axis=1)**2))/2))
        activity=0. if self.previous is None else float(np.sqrt(np.mean((gray-self.previous)**2)))
        self.previous=gray
        scene_delta=0. if self.scene_anchor is None else float(np.sqrt(np.mean((gray-self.scene_anchor)**2)))
        scene_changed=scene_delta>self.scene_limit
        if self.scene_anchor is None or scene_changed:self.scene_anchor=gray
        old=self.presence;transition='NONE'
        if score>=self.policy['presence_high'] and gradient>=self.policy['gradient_min']:
            self.high_count+=1;self.low_count=0
            if self.high_count>=self.policy['confirm_frames']:self.presence='present'
        elif score<=self.policy['presence_low']:
            self.low_count+=1;self.high_count=0
            if self.low_count>=self.policy['confirm_frames']:self.presence='absent'
        else:self.high_count=self.low_count=0
        if self.presence in ('present','absent'):
            if self.presence!=self.last_confirmed:
                if self.presence=='present':transition='APPEARED'
                elif self.last_confirmed=='present':transition='DISAPPEARED'
            elif old=='unknown':transition='RESTORED'
            self.last_confirmed=self.presence
        direction='unknown';centroid=None
        if self.presence=='present' and score>=self.policy['presence_low']:
            mass=np.maximum(np.abs(gray)-self.policy['noise_floor'],0);mass[self.stable]=0
            total=float(mass.sum())
            if total>0:
                centroid=float(np.dot(mass.sum(axis=0),np.arange(126))/total)
                if self.last_centroid is not None and self.policy['direction_polarity'] is not None:
                    delta=(centroid-self.last_centroid)*self.policy['direction_polarity']
                    if abs(delta)>=self.policy['direction_min_pixels']:direction='right' if delta>0 else 'left'
                self.last_centroid=centroid
        else:self.last_centroid=None
        self.processed+=1
        return dict(frame_uid=m.frame_uid,capture_key=list(m.capture_key),valid=self.presence!='unknown',reason=None,
            presence=self.presence,transition=transition,direction=direction,direction_coordinate_id=self.policy['direction_coordinate_id'],
            centroid_pixels=centroid,score=score,gradient_score=gradient,activity=activity,stable_residual_p95=stable_error,
            scene_change_score=scene_delta,scene_changed=scene_changed,
            score_type='UNCALIBRATED_SHADOW_FEATURE_SCORE',confidence_probability=None,
            context=m.context.data,publish_ns=str(now_ns),expires_ns=str(m.frame_complete_ns+int(self.policy['ttl_ms']*1e6)),
            processing_ms=(time.perf_counter_ns()-start)/1e6)
