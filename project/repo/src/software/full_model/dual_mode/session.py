"""Offline acquisition -> registered measurement -> FAST -> complete IMAGE owner."""
from pathlib import Path
import io,json,time
import numpy as np
from contracts import FileSource,require,pinned,sha,InputError,resolve
from raw_measurement import convert_raw,calibration_key
from messages import Measurement
from detector import FastDetector
from registration import FixedAnchor
from rectify import rectify_raw


class MotionAdapter:
    def __init__(self,raw,mask,calibration,contract,policy,calibration_base=None):
        self.calibration=calibration;self.contract=contract
        self.calibration_base=calibration_base
        self.anchor=FixedAnchor(raw,mask,calibration['roi_xywh'],calibration_key(calibration),policy)
        require(contract['anchor_id']==self.anchor.anchor_id,'ANCHOR_ID','Wrong fixed motion anchor')

    def convert(self,frame,descriptor,base):
        raw=FileSource.decode_raw(frame.data,descriptor['format'])
        registration=self.anchor.register(raw,frame.envelope['frame_uid'],frame.envelope['source_session_id'],calibration_key(self.calibration))
        data,provenance=rectify_raw(frame,descriptor,self.calibration,self.calibration_base or base,descriptor['model']['sha256'],registration,self.contract)
        return data,provenance


class OfflineSession:
    def __init__(self,controller,bindings,model_base,bank=None,replay_session='offline-sequence',temporal_config=None):
        self.controller=controller;self.bindings=bindings;self.model_base=Path(model_base);self.bank=bank
        self.source=FileSource(replay_session);self.binding_key=None;self.detector=None
        self.accepted=0;self.rejected=0;self.last_localization=None;self.last_measurement=None
        self.temporal_config=temporal_config;self.temporal=None;self.last_frame=None
        self.refresh_binding()

    def refresh_binding(self):
        c=self.controller.context
        key=tuple(c.data[k] for k in ('model_hash','model_epoch','calibration_id','geometry_epoch','radiometry_epoch','feature_basis_id'))
        if self.binding_key==key:return
        b=self.bindings.get(c.data['model_hash'])
        require(b is not None,'SESSION_BINDING','No registered FAST/motion reference for active model')
        for field in ('calibration_id','roi_id','operator_id','mask_id'):
            require(b['context'][field]==c.data[field],'SESSION_BINDING','Incompatible reference '+field)
        self.detector=FastDetector(b['reference'],b['stable_mask'],b['fast_policy'])
        if self.temporal_config is not None:
            from burst import QualityPipeline
            self.temporal=QualityPipeline(b['reference'],b['stable_mask'],self.temporal_config['quality'],self.temporal_config['burst'])
            self.controller.image_selector=self.temporal.select;self.controller.frame_observer=self.temporal.observe
            self.controller.on_invalidate=self.temporal.reset
        self.binding_key=key

    def ingest(self,descriptor,base,sample_time_ns):
        try:frame=self.source.load(descriptor,base)
        except (InputError,KeyError,OSError,ValueError) as e:return self.reject(e)
        return self.ingest_frame(frame,descriptor,base,sample_time_ns)

    def reject(self,error):
        self.rejected+=1;self.controller.invalidate('FRAME_REJECTED:'+getattr(error,'code',type(error).__name__))
        self.last_measurement=None;self.last_frame=None
        return dict(status='FRAME_REJECTED',reason=getattr(error,'code',type(error).__name__),error=str(error),presence='unknown',physical_quality_accepted=False)

    def ingest_frame(self,frame,descriptor,base,sample_time_ns):
        ctrl=self.controller;ctrl.poll();self.refresh_binding()
        try:
            env=frame.envelope
            require(frame.kind in ('RAW','PFY') and env['complete'] is True and env['raw_sha256']==sha(frame.data),
                    'FRAME_INTEGRITY','Incomplete or mismatched immutable input')
            sequence=env['original_sensor_frame_id']
            if frame.kind=='RAW':
                require(descriptor.get('calibration') is not None,'MISSING_CALIBRATION','Real RAW needs a registered calibration and model')
            require(sequence is not None and env['original_capture_session_id']==ctrl.context.data['source_session_id'],
                    'SEQUENCE_ID','A registered original capture session and ID are required')
            require(descriptor['model_id']==ctrl.context.data['model_id'] and descriptor['model']['sha256']==ctrl.context.data['model_hash'],
                    'SESSION_MODEL','Input belongs to another active model; no PFY relabeling')
            b=self.bindings[ctrl.context.data['model_hash']];provenance={}
            if frame.kind=='PFY':data=frame.data
            elif b.get('motion') is not None:data,provenance=b['motion'].convert(frame,descriptor,base)
            else:
                calibration=json.loads(pinned(base,descriptor['calibration']))
                calbase=resolve(base,descriptor['calibration']['path']).parent
                data,provenance,*_=convert_raw(frame,descriptor,calibration,calbase,descriptor['model']['sha256'])
            m=Measurement(data,ctrl.context,env['frame_uid'],int(sequence),int(env['t_frame_complete_mono_ns']),
                env['source_kind'],sample_time_ns,provenance.get('transform_id','IDENTITY'),
                sha(json.dumps(dict(format=env['format'],exposure=env.get('exposure_us'),gain=env.get('gain_raw'),
                                    gain_units=env.get('gain_units')),sort_keys=True).encode()),
                'SYNTHETIC_DECLARED' if env['source_kind']=='SYNTHETIC' else 'UNKNOWN')
            event=ctrl.ingest(m,self.detector);self.accepted+=1;self.last_measurement=m;self.last_frame=frame
            return dict(status='MEASUREMENT_ACCEPTED',measurement=m.identity,fast=event,provenance=provenance)
        except (InputError,KeyError,OSError,ValueError) as e:
            return self.reject(e)

    def localize(self,m,old_geometry_valid=True):
        require(self.bank is not None,'SESSION_BANK','No registered candidate library')
        result=self.bank.locate(m.pfy,m.source_kind,geometry_valid=old_geometry_valid)
        self.last_localization=result
        if not result['valid']:
            if not old_geometry_valid:self.controller.invalidate('LOCALIZATION_UNRESOLVED')
            return result
        if result['model_hash']==self.controller.context.data['model_hash']:return result
        b=self.bindings.get(result['model_hash'])
        if b is None:return dict(result,valid=False,reason='CANDIDATE_REFERENCE_NOT_REGISTERED')
        try:
            self.bank.verify_candidate(result['candidate'],self.model_base)
            fields={k:v for k,v in b['context'].items() if k not in ('source_session_id','compat_generation','model_epoch')}
            for k in ('geometry_epoch','radiometry_epoch'):fields[k]=int(fields[k])
            candidate=self.controller.context.changed(**fields)
            require(candidate.data['model_hash']==result['model_hash'] and candidate.data['model_id']==result['model_id'],
                    'SESSION_BINDING','Selected model/reference identity mismatch')
            result['switch_state']=self.controller.propose_model(candidate,True,old_geometry_valid)
        except (InputError,OSError,ValueError) as e:
            result=dict(result,valid=False,reason='CANDIDATE_VERIFICATION_FAILED',error=str(e))
            self.controller.propose_model(self.controller.context,False,old_geometry_valid)
        return result
