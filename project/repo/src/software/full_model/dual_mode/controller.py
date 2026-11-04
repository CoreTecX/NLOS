"""Single-owner bounded FAST/IMAGE scheduling; complete solves drain naturally."""
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from collections import OrderedDict,deque
import threading,time,uuid,math
from contracts import require,sha
from messages import Audit


class RecoveryGate:
    def __init__(self,frames=3,gap_ms=500,deadline_ms=5000):
        self.frames=frames;self.gap_ns=int(gap_ms*1e6);self.deadline_ns=int(deadline_ms*1e6)
        self.valid=False;self.count=0;self.last=None;self.context=None;self.start=None;self.failure=None

    def reset(self):
        self.valid=False;self.count=0;self.last=None;self.context=None;self.start=None;self.failure=None

    def observe(self,m,good,now):
        key=m.context.key
        if self.context!=key:self.reset();self.context=key
        if self.start is None:self.start=now
        duplicate=self.last is not None and (m.capture_key==self.last[0] or m.sensor_sequence<=self.last[2])
        gap=self.last is not None and (m.sample_time_ns<=self.last[1] or m.sample_time_ns-self.last[1]>self.gap_ns)
        if not good or duplicate or gap:
            self.count=0;self.valid=False;self.failure='INVALID_OR_REPEATED_OR_GAP'
            if not duplicate:self.last=(m.capture_key,m.sample_time_ns,m.sensor_sequence)
            return False
        if not self.valid and now-self.start>self.deadline_ns:
            self.failure='RECOVERY_DEADLINE';return False
        self.last=(m.capture_key,m.sample_time_ns,m.sensor_sequence)
        self.count+=1;self.valid=self.count>=self.frames
        return self.valid


@dataclass(frozen=True)
class Request:
    id: str
    measurement: object
    origin: str
    requested_ns: int
    deadline_ns: int
    policy_epoch: int
    scene_epoch: int


class Controller:
    def __init__(self,context,backend,policy,clock=time.monotonic_ns,loader=None):
        self.owner=threading.get_ident();self.context=context;self.backend=backend;self.policy=dict(policy);self.clock=clock
        for key in ('minimum_dwell_ms','stable_wait_ms','trigger_cooldown_ms','fast_period_ms','image_period_ms',
                    'pending_ttl_ms','capture_gap_ms','soft_timeout_ms','drain_timeout_ms','image_stale_ms','fast_budget_ms'):
            require(type(policy[key]) in (int,float) and math.isfinite(policy[key]) and policy[key]>0,'POLICY','Missing positive '+key)
        require(all(type(policy[k]) in (int,float) and math.isfinite(policy[k]) for k in ('activity_high','activity_low')) and
                policy['activity_high']>policy['activity_low']>=0,'POLICY','Invalid activity hysteresis')
        self.strategy='FAST';self.control='MANUAL';self.validity='INITIALIZING';self.image_state='IDLE'
        self.policy_epoch=0;self.scene_epoch=0;self.gate=RecoveryGate(gap_ms=policy['capture_gap_ms'])
        self.pool=ThreadPoolExecutor(max_workers=1,thread_name_prefix='full_image_owner')
        self.future=None;self.running=None;self.pending=None;self.pending_model=None;self.loader=loader or (lambda ctx:ctx)
        self.publish_allowed=True;self.stop_images=False;self.stopped=False;self.faulted=False;self.retired=False
        self.commands=deque();self.acks=OrderedDict();self.client_sequences={};self.audit=Audit(policy.get('audit_capacity',1024))
        self.results=deque(maxlen=128);self.dropped_results=0;self.last_result=None
        self.last_input_ns=None;self.last_capture=None;self.last_mode_ns=clock();self.stable_since=None
        self.last_trigger_ns=None;self.started_ns=None;self.drain_started_ns=None;self.discarded_pending=0
        self.latest_measurement=None;self.worker_thread=None;self.closed=False;self.retired_events=set()
        self.image_selector=None;self.frame_observer=None;self.on_invalidate=None

    def _owner(self):require(threading.get_ident()==self.owner,'OWNER','Controller must run on its owning thread')

    def _ack(self,request_id,outcome,now,**fields):
        row=dict(request_id=request_id,outcome=outcome,received_ns=str(now),applied_ns=None,policy_epoch=str(self.policy_epoch))
        row.update(fields)
        self.acks[request_id]=row;self.acks.move_to_end(request_id)
        while len(self.acks)>256:self.acks.popitem(last=False)
        self.audit.add('CONTROL_ACK',**row)
        return dict(row)

    def command(self,request_id,client,sequence,kind,strategy=None):
        self._owner();now=self.clock()
        if request_id in self.acks:return dict(self.acks[request_id])
        if kind=='STOP':
            self.stop(all_work=True);return self._ack(request_id,'APPLIED',now,applied_ns=str(now))
        if self.stopped or self.faulted:return self._ack(request_id,'STOPPED_OR_FAULTED',now)
        if kind not in ('MANUAL','AUTO') or (kind=='MANUAL' and strategy not in ('FAST','IMAGE')):
            return self._ack(request_id,'UNSUPPORTED_COMMAND',now)
        if type(sequence) is not int or sequence<=self.client_sequences.get(client,-1):return self._ack(request_id,'EXPIRED_SEQUENCE',now)
        if client not in self.client_sequences and len(self.client_sequences)>=16:return self._ack(request_id,'BUSY_CLIENT_LIMIT',now)
        if len(self.commands)>=16:return self._ack(request_id,'BUSY_QUEUE_FULL',now)
        if kind=='AUTO' and not self.policy.get('auto_registered',False):return self._ack(request_id,'AUTO_NOT_REGISTERED',now)
        self.client_sequences[client]=sequence;self.commands.append((request_id,kind,strategy,now))
        return self._ack(request_id,'QUEUED',now)

    def _apply_commands(self,now):
        while self.commands:
            rid,kind,strategy,received=self.commands.popleft()
            self.control=kind
            if kind=='MANUAL':self.strategy=strategy
            if kind=='MANUAL' and strategy=='FAST' and self.pending and self.pending.origin=='AUTO':
                self.audit.add('PENDING_DISCARDED',request_id=self.pending.id,reason='MANUAL_FAST')
                self.pending=None;self.discarded_pending+=1
            self.policy_epoch+=1;self.last_mode_ns=now;self.stable_since=None
            row=self.acks.get(rid,dict(request_id=rid,received_ns=str(received)))
            row.update(outcome='APPLIED',applied_ns=str(now),policy_epoch=str(self.policy_epoch),control=self.control,strategy=self.strategy)
            self.audit.add('CONTROL_APPLIED',**row)

    def invalidate(self,reason,event_id=None):
        self._owner();now=self.clock()
        if self.on_invalidate:self.on_invalidate(reason)
        if not self.retired:
            self.context=self.context.changed(compat_generation=int(self.context.data['compat_generation'])+1)
            self.retired=True
        self.validity='RELOCALIZING';self.gate.reset();self.stable_since=None;self.latest_measurement=None
        if self.pending:self.discarded_pending+=1;self.audit.add('PENDING_DISCARDED',request_id=self.pending.id,reason=reason)
        self.pending=None;self.publish_allowed=False
        if self.future:
            self.image_state='DRAINING'
            if self.drain_started_ns is None:self.drain_started_ns=now
        self.audit.add('CONTEXT_INVALIDATED',reason=reason,generation=self.context.data['compat_generation'],at_ns=str(now))

    def propose_model(self,candidate,verified,old_geometry_valid=True):
        self._owner()
        if not verified:
            if not old_geometry_valid:self.invalidate('CANDIDATE_INVALID_OLD_GEOMETRY_INVALID')
            return 'REJECTED_CANDIDATE'
        self.invalidate('MODEL_SWITCH')
        self.pending_model=candidate.changed(compat_generation=int(self.context.data['compat_generation']),
                                             model_epoch=int(self.context.data['model_epoch'])+1)
        self.audit.add('MODEL_PREPARED',model_id=candidate.data['model_id'])
        self.poll()
        return 'DRAINING' if self.future else ('FAULTED' if self.faulted else 'COMMITTED_WARMUP')

    def _commit(self):
        if self.pending_model is None or self.future is not None or self.faulted:return
        candidate=self.pending_model;self.pending_model=None
        try:
            activated=self.loader(candidate)
            require(activated.key==candidate.key,'MODEL_TRANSACTION','Loader returned wrong context')
            self.context=activated;self.gate.reset();self.validity='INITIALIZING';self.retired=False;self.publish_allowed=True
            self.audit.add('MODEL_COMMITTED',context=self.context.data)
        except Exception as e:
            self.faulted=True;self.validity='FAULTED';self.image_state='FAULTED';self.audit.add('MODEL_COMMIT_FAILED',error=str(e))

    def request_image(self,m,origin='MANUAL',request_id=None):
        self._owner();now=self.clock();rid=request_id or str(uuid.uuid4())
        if rid in self.acks:return dict(self.acks[rid])
        if self.stopped or self.stop_images or self.faulted:return self._ack(rid,'STOPPED_OR_FAULTED',now)
        if self.validity!='VALID' or m.context.key!=self.context.key:return self._ack(rid,'INVALID_CONTEXT',now)
        if now<m.frame_complete_ns or now-m.frame_complete_ns>self.policy['capture_gap_ms']*1e6:
            return self._ack(rid,'EXPIRED_MEASUREMENT',now)
        if self.image_selector is not None:
            selected,decision=self.image_selector(m)
            if selected is None:return self._ack(rid,'WAITING_BURST',now,burst=decision)
            require(selected.context.key==m.context.key and selected.sensor_sequence==m.sensor_sequence,
                    'BURST_CONTEXT','Selector changed model or ending capture')
            m=selected
        request=Request(rid,m,origin,now,now+int(self.policy['pending_ttl_ms']*1e6),self.policy_epoch,self.scene_epoch)
        if self.future is None:
            self._start(request);return self._ack(rid,'RUNNING',now)
        if self.pending is not None:
            if self.pending.origin=='MANUAL':return self._ack(rid,'BUSY',now)
            # Replace only the waiting snapshot; keep its original expiration.
            request=Request(rid,m,origin,now,min(request.deadline_ns,self.pending.deadline_ns),self.policy_epoch,self.scene_epoch)
            self.discarded_pending+=1;self.audit.add('PENDING_REPLACED',previous=self.pending.id,replacement=rid)
        self.pending=request
        return self._ack(rid,'QUEUED',now)

    def _start(self,request):
        self.running=request;self.started_ns=self.clock();self.publish_allowed=True;self.drain_started_ns=None;self.image_state='RUNNING'
        def work():
            thread=threading.get_ident()
            if self.worker_thread is None:self.worker_thread=thread
            require(self.worker_thread==thread,'OWNER','Backend ownership changed')
            return self.backend(request)
        self.future=self.pool.submit(work)
        self.audit.add('IMAGE_STARTED',request_id=request.id,frame_uid=request.measurement.frame_uid)

    def cancel_image(self,request_id):
        self._owner()
        if self.pending and self.pending.id==request_id:self.pending=None;self.audit.add('PENDING_CANCELLED',request_id=request_id);return 'CANCELLED_PENDING'
        if self.running and self.running.id==request_id:
            self.publish_allowed=False;self.image_state='DRAINING';self.drain_started_ns=self.clock()
            self.audit.add('IMAGE_PUBLICATION_CANCELLED',request_id=request_id);return 'DRAINING'
        return 'UNKNOWN_REQUEST'

    def stop(self,all_work=False):
        self._owner();self.stop_images=True;self.pending=None
        if all_work:self.stopped=True;self.commands.clear();self.validity='INVALID'
        if self.future:
            self.publish_allowed=False;self.image_state='DRAINING'
            if self.drain_started_ns is None:self.drain_started_ns=self.clock()
        self.audit.add('STOP_REQUESTED',all_work=all_work,drained=self.future is None)

    def poll(self):
        self._owner();now=self.clock();self._apply_commands(now)
        # Retire a lost capture context before accepting any late worker result.
        if self.last_input_ns is not None and now-self.last_input_ns>self.policy['capture_gap_ms']*1e6 and self.validity=='VALID':
            self.invalidate('CAPTURE_GAP')
        if self.future is not None and self.future.done():
            request=self.running
            try:
                output=self.future.result()
                valid_output=output.get('status')=='COMPLETE' and output.get('state_acceptance') is True
                if valid_output:
                    require(output.get('measurement_sha256')==request.measurement.sha256 and
                            output.get('model_hash')==request.measurement.context.data['model_hash'],
                            'RESULT_IDENTITY','Backend result does not belong to submitted measurement/model')
                compatible=request.measurement.context.key==self.context.key and not self.faulted and self.validity=='VALID'
                freshness='CURRENT'
                if not compatible:freshness='STALE_CONTEXT'
                elif not self.publish_allowed:freshness='CANCELLED'
                elif request.scene_epoch!=self.scene_epoch:freshness='SCENE_CHANGED'
                elif now-request.measurement.frame_complete_ns>self.policy['image_stale_ms']*1e6:freshness='HISTORICAL_AGE'
                result=dict(request_id=request.id,frame_uid=request.measurement.frame_uid,context=request.measurement.context.data,
                    measurement=request.measurement.identity,
                    scene_epoch=str(request.scene_epoch),
                    sensor_sequence=str(request.measurement.sensor_sequence),
                    policy_epoch=str(request.policy_epoch),valid_numeric=valid_output,freshness=freshness,
                    publish_allowed=bool(valid_output and compatible and self.publish_allowed and request.scene_epoch==self.scene_epoch),output=output,
                    host_age_ms=(now-request.measurement.frame_complete_ns)/1e6,capture_age_ms=None,
                    queue_wait_ms=(self.started_ns-request.requested_ns)/1e6,
                    worker_elapsed_ms=(now-self.started_ns)/1e6,
                    completed_ns=str(now))
                if self.last_result and self.last_result['context']==result['context'] and int(self.last_result['sensor_sequence'])>request.measurement.sensor_sequence:
                    result.update(publish_allowed=False,freshness='OUT_OF_ORDER')
                if len(self.results)==self.results.maxlen:self.dropped_results+=1
                self.results.append(result)
                if result['publish_allowed']:self.last_result=result
                self.audit.add('IMAGE_FINISHED',request_id=request.id,freshness=freshness,valid_numeric=valid_output)
            except Exception as e:
                self.faulted=True;self.validity='FAULTED';self.pending=None;self.audit.add('IMAGE_FAILED',error=str(e))
            self.future=None;self.running=None;self.started_ns=None;self.drain_started_ns=None
            self.image_state='FAULTED' if self.faulted else 'IDLE'
        elif self.future is not None:
            if now-self.started_ns>self.policy['soft_timeout_ms']*1e6 and self.image_state!='DRAINING':
                self.publish_allowed=False;self.image_state='DRAINING';self.drain_started_ns=now;self.pending=None
                self.audit.add('IMAGE_SOFT_TIMEOUT')
            if self.drain_started_ns is not None and now-self.drain_started_ns>self.policy['drain_timeout_ms']*1e6:
                self.faulted=True;self.validity='FAULTED';self.image_state='FAULTED';self.pending=None
                self.audit.add('DRAIN_TIMEOUT_NO_HARD_RESET')
        self._commit()
        if self.pending and (now>self.pending.deadline_ns or self.pending.measurement.context.key!=self.context.key or
                self.pending.scene_epoch!=self.scene_epoch or now<self.pending.measurement.frame_complete_ns or
                now-self.pending.measurement.frame_complete_ns>self.policy['capture_gap_ms']*1e6):
            self.audit.add('PENDING_EXPIRED',request_id=self.pending.id);self.pending=None;self.discarded_pending+=1
        if self.future is None and self.pending and self.validity=='VALID' and not(self.stopped or self.stop_images or self.faulted):
            req=self.pending;self.pending=None;self._start(req)

    def ingest(self,m,detector,input_valid=True):
        started=time.perf_counter_ns();self._owner();self.poll();now=self.clock()
        if self.stopped:return detector.unknown(m,'STOPPED',now)
        distinct=self.last_capture is None or (m.capture_key[0]!=self.last_capture[0] or m.capture_key[1]>self.last_capture[1])
        if distinct:self.last_input_ns=now;self.last_capture=m.capture_key
        compatible=m.context.key==self.context.key and not self.faulted
        if not input_valid and compatible:self.invalidate('INPUT_INVALID');compatible=False
        ready=self.gate.observe(m,input_valid and compatible,now) if compatible else False
        if ready:
            self.validity='VALID';self.retired=False;self.latest_measurement=m
        elif compatible:
            self.validity='INITIALIZING';self.latest_measurement=None
        event=detector.process(m,valid=ready,now_ns=now)
        if self.frame_observer:event=self.frame_observer(m,event,ready)
        event['pipeline_processing_ms']=(time.perf_counter_ns()-started)/1e6
        if self.validity=='VALID' and event.get('reason') not in (None,'DUPLICATE_OR_OUT_OF_ORDER'):
            self.invalidate('FAST_INPUT_INVALID:'+event['reason'])
        if event.get('valid'):
            if event['transition'] in ('APPEARED','DISAPPEARED') or event['activity']>self.policy['activity_high'] or event.get('scene_changed',False):
                self.scene_epoch+=1
                if self.pending:
                    self.audit.add('PENDING_DISCARDED',request_id=self.pending.id,reason='SCENE_CHANGED')
                    self.pending=None;self.discarded_pending+=1
            self._auto(m,event,now)
        self.audit.add('FAST_UPDATE',frame_uid=m.frame_uid,valid=event.get('valid'),reason=event.get('reason'),transition=event.get('transition'))
        return event

    def _auto(self,m,event,now):
        if self.control=='AUTO':
            activity=event['activity'];desired=self.strategy
            if activity>=self.policy['activity_high'] or event['transition']=='APPEARED' or event.get('scene_changed',False):desired='FAST';self.stable_since=None
            elif activity<=self.policy['activity_low']:
                if self.stable_since is None:self.stable_since=now
                if now-self.stable_since>=self.policy['stable_wait_ms']*1e6:desired='IMAGE'
            else:self.stable_since=None
            if desired!=self.strategy and now-self.last_mode_ns>=self.policy['minimum_dwell_ms']*1e6:
                self.strategy=desired;self.policy_epoch+=1;self.last_mode_ns=now;self.audit.add('AUTO_MODE',strategy=desired)
        if event.get('pipeline_processing_ms',event.get('processing_ms',0))>self.policy['fast_budget_ms']:
            self.audit.add('IMAGE_ADMISSION_PAUSED_FAST_BUDGET');return
        period=self.policy['image_period_ms'] if self.strategy=='IMAGE' else self.policy['fast_period_ms']
        trigger=(self.control=='AUTO' and event['transition']=='APPEARED') or self.strategy=='IMAGE'
        if trigger and (self.last_trigger_ns is None or now-self.last_trigger_ns>=max(period,self.policy['trigger_cooldown_ms'])*1e6):
            self.request_image(m,'AUTO');self.last_trigger_ns=now

    def close(self):
        self._owner()
        if self.closed:return
        self.stop(all_work=True)
        # Drain the active solve; the default does not cancel pending futures.
        self.pool.shutdown(wait=True)
        self.poll();self.closed=True
