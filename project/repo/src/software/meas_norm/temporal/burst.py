"""Bounded aligned linear-measurement averaging; FAST always retains K=1."""
from collections import deque
import json
import numpy as np
from contracts import require,decode_pfy,encode_pfy,sha
from messages import Measurement
from quality import QualityEstimator


class BurstAccumulator:
    def __init__(self,policy):
        self.policy=dict(policy);self.frames=deque(maxlen=8);self.resets=0;self.rejected=0;self.last_reason='EMPTY'
        self.last_quality=None;self.motion_limited=False;self.seen_last=None
        self.centroids=deque(maxlen=8)
        for k in ('max_gap_ms','max_span_ms','max_motion_rms','normalizer','target_snr_db'):
            require(np.isfinite(policy[k]) and policy[k]>0,'BURST_POLICY','Invalid '+k)
        self.requested_k=1;self.set_k(policy.get('k',1))

    def reset(self,reason='INVALID'):
        self.frames.clear();self.last_quality=None;self.motion_limited=False;self.last_reason=reason;self.resets+=1
        self.centroids.clear()

    def set_k(self,value):
        require((type(value) is int and value in (1,2,4,8)) or value=='AUTO','BURST_K','Only 1,2,4,8,AUTO supported')
        self.requested_k=value;self.reset('K_CHANGED')

    @staticmethod
    def key(m):
        return (m.context.key,m.capture_signature,m.temporal_scope,decode_pfy(m.pfy)[1].tobytes())

    def observe(self,m,event,quality,valid=True):
        if not valid or not quality.get('valid') or not event.get('valid'):
            self.reset('INPUT_OR_QUALITY_INVALID');return
        if m.k!=1:self.reset('AGGREGATE_REINPUT');self.rejected+=1;return
        if self.seen_last is not None and m.capture_key[0]==self.seen_last[0] and m.sensor_sequence<=self.seen_last[1]:
            self.reset('DUPLICATE_OR_OUT_OF_ORDER');self.rejected+=1;return
        self.seen_last=m.capture_key;self.motion_limited=False
        if self.frames:
            last=self.frames[-1]
            if self.key(m)!=self.key(last):self.reset('COMPATIBILITY_CHANGED')
            elif m.sensor_sequence!=last.sensor_sequence+1:self.reset('MISSING_FRAME')
            elif m.sample_time_ns<=last.sample_time_ns or m.sample_time_ns-last.sample_time_ns>self.policy['max_gap_ms']*1e6:
                self.reset('SAMPLE_GAP_OR_CLOCK')
            else:
                delta=float(np.sqrt(np.mean(((m.rgb-last.rgb)/self.policy['normalizer'])**2)))
                # Conservative total change: no assumed noise independence can erase real motion.
                center=event.get('centroid_pixels');first_center=self.centroids[0] if self.centroids else None
                moving=event.get('direction') in ('left','right') or event.get('transition') in ('APPEARED','DISAPPEARED')
                moving=moving or (center is not None and first_center is not None and abs(center-first_center)>self.policy.get('max_centroid_span_pixels',1.5))
                if delta>self.policy['max_motion_rms'] or moving:
                    self.reset('MOTION_FORCED_SINGLE');self.motion_limited=True
        self.frames.append(m);self.centroids.append(event.get('centroid_pixels'));self.last_quality=dict(quality)
        while self.frames and m.sample_time_ns-self.frames[0].sample_time_ns>self.policy['max_span_ms']*1e6:self.frames.popleft();self.centroids.popleft()

    def select(self,m):
        desired=self.requested_k
        decision=dict(requested_k=desired,effective_k=None,available=len(self.frames),reason=None,
                      wait_ms=None,scope=m.temporal_scope,physical_quality_accepted=False)
        if not self.frames or self.frames[-1].capture_key!=m.capture_key or self.frames[-1].context.key!=m.context.key:
            return None,dict(decision,reason=self.last_reason)
        registered=m.temporal_scope in ('SYNTHETIC_DECLARED','REGISTERED_SEQUENCE') and bool(m.capture_signature)
        if m.source_kind=='REPLAY_SINGLE' or not registered:desired=1;decision['reason']='NO_REGISTERED_SEQUENCE_K1_ONLY'
        elif self.motion_limited:desired=1;decision['reason']='MOTION_FORCED_SINGLE'
        elif desired=='AUTO':
            snr=self.last_quality.get('snr_db') if self.last_quality else None
            if snr is None:desired=1;decision['reason']='UNOBSERVABLE_NO_ASSUMED_GAIN'
            else:
                desired=next((k for k in (1,2,4,8) if snr+10*np.log10(k)>=self.policy['target_snr_db']),8)
                decision['reason']='INDEPENDENT_NOISE_GAIN_HEURISTIC_NOT_QUALITY_GUARANTEE'
        if len(self.frames)<desired:return None,dict(decision,effective_k=desired,reason='WAITING_DISTINCT_FRAMES')
        selected=list(self.frames)[-desired:];wait=(selected[-1].sample_time_ns-selected[0].sample_time_ns)/1e6
        decision.update(effective_k=desired,wait_ms=wait,oldest_host_ns=str(selected[0].frame_complete_ns),newest_host_ns=str(m.frame_complete_ns))
        if desired==1:return m,decision
        # Average calibrated linear RGB before the frozen solver computes its own ds.
        rgb=np.zeros((126,126,3),np.float64)
        for item in selected:rgb+=item.rgb/desired
        data=encode_pfy(rgb,m.scales,m.context.data['model_id'])
        members=[dict(frame_uid=x.frame_uid,source_session_id=x.capture_key[0],sensor_sequence=str(x.sensor_sequence),
                      sample_time_ns=str(x.sample_time_ns),frame_complete_ns=str(x.frame_complete_ns),transform_id=x.transform_id,
                      measurement_sha256=sha(x.pfy)) for x in selected]
        encoded=json.dumps(members,sort_keys=True,separators=(',',':'))
        out=Measurement(data,m.context,'burst:'+sha(encoded.encode()+data),m.sensor_sequence,m.frame_complete_ns,m.source_kind,
                        m.sample_time_ns,'ALIGNED_MEMBER_MAPS',m.capture_signature,m.temporal_scope,encoded)
        return out,decision


class QualityPipeline:
    def __init__(self,reference,stable_mask,quality_policy,burst_policy):
        self.estimator=QualityEstimator(reference,stable_mask,quality_policy);self.burst=BurstAccumulator(burst_policy)
        self.last_quality=None;self.last_decision=None
    def reset(self,reason):self.burst.reset(reason);self.last_quality=None;self.last_decision=None
    def observe(self,m,event,ready):
        quality=self.estimator.evaluate(m,ready and event.get('valid',False));self.last_quality=quality
        self.burst.observe(m,event,quality,ready)
        _,self.last_decision=self.burst.select(m)
        if ready and event.get('valid') and not quality['valid']:
            event=dict(event,valid=False,presence='unknown',direction='unknown',transition='NONE',reason='QUALITY_INVALID:'+str(quality['reason']))
        return dict(event,quality=quality,burst=self.last_decision,k_fast=1)
    def select(self,m):
        value,self.last_decision=self.burst.select(m);return value,self.last_decision
