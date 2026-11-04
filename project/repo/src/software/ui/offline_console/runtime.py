"""Single control-owner worker, isolated complete solve, bounded UI/save/log lanes."""
from bootstrap import REPO
from pathlib import Path
import copy,io,json,queue,threading,time,uuid
import numpy as np
from PIL import Image
from contracts import read_json,write_json,sha
from controller import Controller
from session import OfflineSession
from backend import FrozenCpuBackend
from sources import SyntheticSource,ManifestSource
from presentation import DisplayMailbox
from saver import SnapshotSaver,SaveSnapshot
from logger import BoundedLog
from quality import reconstruction_quality


def preview_bytes(rgb):
    a=np.asarray(rgb,np.float64);lo=float(a.min());hi=float(a.max())
    return np.uint8(np.clip((a-lo)/max(hi-lo,1e-12),0,1)*255).tobytes()


class ConsoleEngine:
    def __init__(self,workspace,output,source_mode='dynamic',manifest=None,backend_factory=FrozenCpuBackend):
        self.workspace=Path(workspace);self.output=Path(output);self.mode=source_mode;self.manifest=manifest
        self.config=read_json(REPO/'configs/offline_console.json');self.backend_factory=backend_factory
        self.run_id=uuid.uuid4().hex;self.mailbox=DisplayMailbox(self.config['event_history_capacity'])
        self.commands=queue.Queue(maxsize=self.config['command_capacity']);self.stop_event=threading.Event();self.finished=threading.Event()
        self.thread=None;self.controller=None;self.session=None;self.saver=None;self.log=None
        self.command_sequence=0;self.busy_commands=0;self.failed=False;self.error=None;self.source_eof=False
        self.latest_preview=None;self.latest_result=None;self.latest_image_files=();self.update_id=0;self.processed=0;self.summary=None
        self.display_commits=0;self.last_source_ns=None;self.metrics=[];self.created_output=False
    def start(self):
        if self.thread is not None:raise RuntimeError('Use a fresh engine for each run')
        self.thread=threading.Thread(target=self._run,name='offline_control_owner',daemon=True);self.thread.start()
    def submit(self,kind,**args):
        if kind=='STOP':self.stop_event.set();return dict(outcome='STOP_REQUESTED')
        if self.finished.is_set():return dict(outcome='STOPPED')
        self.command_sequence+=1;command=dict(kind=kind,request_id=uuid.uuid4().hex,sequence=self.command_sequence,**args)
        try:self.commands.put_nowait(command);return dict(outcome='QUEUED',request_id=command['request_id'])
        except queue.Full:self.busy_commands+=1;return dict(outcome='BUSY')
    def _event(self,row):
        self.mailbox.event(dict(run_id=self.run_id,**row))
        if self.log:self.log.emit(row)
    def _publish(self,kind,**fields):
        self.update_id+=1;self.mailbox.publish(kind,dict(run_id=self.run_id,update_id=self.update_id,**fields))
    def record_display(self,kind,frame_uid,frame_ns,transition='NONE'):
        self.display_commits+=1
        if self.log and not self.log.closing:self.log.emit(dict(event='TK_DISPLAY_SUBMITTED',kind=kind,frame_uid=frame_uid,transition=transition,
            frame_complete_to_submit_ms=(time.monotonic_ns()-int(frame_ns))/1e6,pixel_light_latency_measured=False))
    def _state(self,phase):
        c=self.controller
        self._publish('state',phase=phase,context=c.context.data if c else None,validity=c.validity if c else 'INITIALIZING',
            strategy=c.strategy if c else 'FAST',control=c.control if c else 'MANUAL',image_state=c.image_state if c else 'IDLE',
            scene_epoch=str(c.scene_epoch) if c else '0',processed=self.processed,source_eof=self.source_eof,
            pending_image=bool(c and c.pending),running_image=bool(c and c.future),error=self.error,
            source_label=self.source.label if hasattr(self,'source') else '',output=str(self.output),
            save=self.saver.status() if self.saver else {},telemetry=self.log.status() if self.log else {},
            mailbox=self.mailbox.counts(),busy_commands=self.busy_commands,
            requested_k=self.session.temporal.burst.requested_k if self.session and self.session.temporal else 1)
    def _save(self):
        if self.session is None or self.session.last_measurement is None or self.latest_preview is None:return dict(outcome='NO_FRAME')
        m=self.session.last_measurement;f=self.session.last_frame
        # No disk read or PNG compression on the control path. Serialize fixed display pixels on writer thread.
        files=[('measurement.pfy',m.pfy),('preview.rgb',self.latest_preview)]
        if f and f.kind=='RAW':files.append(('input.raw',f.data))
        files.extend(self.latest_image_files)
        metadata=dict(run_id=self.run_id,measurement=m.identity,quality=self.session.temporal.last_quality,
            burst=self.session.temporal.last_decision,preview_shape=[126,126,3],preview_is_display_only=True,
            reconstruction=self.latest_result,frame_envelope=f.envelope if f else None,
            note='Input and reconstruction may have different source times. Reconstruction retains its own provenance.',hardware_accessed=False)
        return self.saver.submit(SaveSnapshot(json.dumps(metadata,ensure_ascii=False,allow_nan=False),tuple(files)))
    def _commands(self):
        c=self.controller
        while not self.commands.empty():
            cmd=self.commands.get_nowait();kind=cmd['kind']
            try:
                if kind=='MODE':r=c.command(cmd['request_id'],'ui',cmd['sequence'],'MANUAL',cmd['strategy']);c.poll();r=c.acks.get(cmd['request_id'],r)
                elif kind=='AUTO':r=c.command(cmd['request_id'],'ui',cmd['sequence'],'AUTO');c.poll();r=c.acks.get(cmd['request_id'],r)
                elif kind=='K':
                    self.session.temporal.burst.set_k(cmd['value']);c.policy_epoch+=1;r=dict(outcome='APPLIED',requested_k=cmd['value'])
                elif kind=='IMAGE':
                    m=self.session.last_measurement;r=c.request_image(m,request_id=cmd['request_id']) if m else dict(outcome='NO_FRAME')
                elif kind=='SAVE':r=self._save()
                elif kind=='LOCALIZE':
                    if self.session.bank is None:r=dict(outcome='NO_MODEL_BANK')
                    elif self.session.last_measurement is None:r=dict(outcome='NO_FRAME')
                    else:r=dict(outcome='LOCALIZATION_RESULT',localization=self.session.localize(self.session.last_measurement))
                else:r=dict(outcome='UNSUPPORTED_COMMAND')
            except (ValueError,KeyError) as e:r=dict(outcome='REJECTED',error=str(e))
            self._event(dict(kind='COMMAND_ACK',command=kind,**dict(r,request_id=cmd['request_id'])))
    def _results(self):
        c=self.controller
        while c.results:
            result=c.results.popleft();self.latest_result=copy.deepcopy(result)
            take=getattr(self.backend,'take_artifacts',None)
            files=take(result['request_id']) if take else ()
            data=dict(files).get('image_reconstruction_native.png')
            self.latest_image_files=tuple(files)
            self._publish('image',result=result,png=data,quality=reconstruction_quality(result))
            self._event(dict(kind='IMAGE_RESULT',result=result))
    def _run(self):
        try:
            self.output.mkdir(parents=True,exist_ok=False)
            self.created_output=True
            self.log=BoundedLog(self.output/'events.jsonl',self.config['telemetry_capacity'])
            self.saver=SnapshotSaver(self.output/'saved',self._event)
            config=read_json(REPO/'configs/offline_engineering.json');dynamic=read_json(REPO/'configs/offline_dynamic.json')
            self.source=ManifestSource(self.manifest,self.workspace) if self.mode=='manifest' else SyntheticSource(self.mode,config,dynamic,self.workspace)
            backend=self.backend_factory(REPO,REPO/'configs/offline_engineering.json',self.workspace,self.output/'images',self.source.models)
            self.backend=backend
            temporal=copy.deepcopy(self.config)
            if self.source.normalizer is not None:
                temporal['quality']['normalizer']=self.source.normalizer;temporal['burst']['normalizer']=self.source.normalizer
            self.controller=Controller(self.source.context,backend,dynamic['scheduler_policy'])
            self.session=OfflineSession(self.controller,self.source.bindings,self.workspace,bank=getattr(self.source,'bank',None),temporal_config=temporal)
            next_frame=time.monotonic_ns();last_state=0;stopping=False
            self._state('RUNNING')
            while True:
                now=time.monotonic_ns();c=self.controller
                if self.stop_event.is_set() and not stopping:
                    c.stop(all_work=True);stopping=True;self._event(dict(kind='STOP_DRAINING'))
                if not stopping:self._commands()
                c.poll();self._results()
                if c.faulted and not stopping:self.failed=True;self.stop_event.set();continue
                if not stopping and not self.source_eof and now>=next_frame:
                    item=self.source.next()
                    if item is None:self.source_eof=True;self._event(dict(kind='SOURCE_ENDED'))
                    else:
                        frame,d,base,sample_ns=item;began=time.perf_counter_ns();result=self.session.ingest_frame(frame,d,base,sample_ns)
                        self.processed+=1;self.last_source_ns=int(frame.envelope['t_frame_complete_mono_ns'])
                        if result['status']=='MEASUREMENT_ACCEPTED':
                            m=self.session.last_measurement;event=result['fast'];self.latest_preview=preview_bytes(m.rgb)
                            if getattr(self.source,'localize_requested',False) and self.session.bank is not None:
                                self._event(dict(kind='LOCALIZATION_RESULT',result=self.session.localize(m)))
                            self._publish('preview',rgb=self.latest_preview,shape=[126,126,3],frame_uid=m.frame_uid,context=m.context.data,
                                frame_complete_ns=str(m.frame_complete_ns),source_kind=m.source_kind)
                            self._publish('fast',event=event,frame_uid=m.frame_uid,context=m.context.data,frame_complete_ns=str(m.frame_complete_ns))
                            self.log.emit(dict(event='FAST_UPDATE',frame_uid=m.frame_uid,context=m.context.data,fast=event,
                                frame_complete_to_fast_ms=(time.monotonic_ns()-m.frame_complete_ns)/1e6,
                                sensor_capture_latency_measured=False))
                        else:self._event(dict(kind='INPUT_REJECTED',**result))
                        elapsed=(time.perf_counter_ns()-began)/1e6
                        self.metrics.append(elapsed)
                        if len(self.metrics)>4096:self.metrics.pop(0)
                    # Source is paced without generating hidden catch-up duplicate frames.
                    next_frame=now+self.config['frame_period_ms']*1_000_000
                if now-last_state>100_000_000:
                    self._state('DRAINING' if stopping else 'SOURCE_ENDED' if self.source_eof else 'RUNNING');last_state=now
                if stopping and c.future is None:break
                self.stop_event.wait(.005) if not stopping else time.sleep(.005)
        except Exception as e:
            self.failed=True;self.error=str(e);self._event(dict(kind='ENGINE_ERROR',error=str(e)))
        finally:
            if self.controller:self.controller.close();self._results()
            close_backend=getattr(getattr(self,'backend',None),'close',None)
            if close_backend:close_backend()
            self._state('DRAINING_SAVE')
            if self.saver:self.saver.close(wait=True)
            if self.log:self.log.close(wait=True)
            self.summary=dict(run_id=self.run_id,failed=self.failed,error=self.error,source_mode=self.mode,processed=self.processed,
                hardware_accessed=False,save=self.saver.status() if self.saver else {},telemetry=self.log.status() if self.log else {},
                mailbox=self.mailbox.counts(),display_commits=self.display_commits,latest_result=self.latest_result,
                pipeline_ms=dict(count=len(self.metrics),p95=float(np.percentile(self.metrics,95)),max=max(self.metrics)) if self.metrics else None,
                closed=True)
            try:
                if self.created_output:write_json(self.output/'console_summary.json',self.summary)
            except OSError as e:self.error='Summary write failed: '+str(e);self.failed=True
            finally:self._state('CLOSED');self.finished.set()
