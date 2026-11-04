"""Separate latest display channels, historical result gates and live age labels."""
from collections import deque
import copy,threading,time


class DisplayMailbox:
    def __init__(self,event_capacity=128):
        self.lock=threading.Lock();self.slots={};self.events=deque(maxlen=event_capacity);self.replaced={};self.dropped_events=0
    def publish(self,kind,value):
        if kind not in ('state','preview','fast','image'):raise ValueError('Unknown display channel')
        with self.lock:
            if kind in self.slots:self.replaced[kind]=self.replaced.get(kind,0)+1
            self.slots[kind]=copy.deepcopy(value)
    def event(self,value):
        with self.lock:
            if len(self.events)==self.events.maxlen:self.dropped_events+=1
            self.events.append(copy.deepcopy(value))
    def take(self):
        with self.lock:
            slots=self.slots;events=list(self.events);self.slots={};self.events.clear()
            return slots,events
    def counts(self):
        with self.lock:return dict(pending_channels=len(self.slots),replaced=dict(self.replaced),dropped_events=self.dropped_events)


class Presenter:
    def __init__(self,run_id,ttl_ms=500,image_stale_ms=1000):
        self.run_id=run_id;self.ttl_ns=int(ttl_ms*1e6);self.image_stale_ns=int(image_stale_ms*1e6)
        self.state={};self.preview=None;self.fast=None;self.image=None;self.history=None;self.rejected=0;self.last_order={}
    def apply(self,kind,value):
        if value.get('run_id')!=self.run_id:self.rejected+=1;return False
        order=int(value.get('update_id',0))
        if order<=self.last_order.get(kind,-1):self.rejected+=1;return False
        self.last_order[kind]=order
        if kind=='image':
            self.history=value
            result=value['result']
            if not result.get('valid_numeric') or not result.get('publish_allowed') or result['context']!=self.state.get('context'):
                self.rejected+=1;return False
            if self.image and self.image['result']['context']==result['context'] and int(result['sensor_sequence'])<int(self.image['result']['sensor_sequence']):
                self.rejected+=1;return False
        setattr(self,kind,value);return True
    def view(self,now_ns=None):
        now=time.monotonic_ns() if now_ns is None else now_ns
        fast=copy.deepcopy(self.fast['event']) if self.fast else dict(valid=False,presence='unknown',direction='unknown')
        if self.fast and (now-int(self.fast['frame_complete_ns'])>self.ttl_ns or self.fast['context']!=self.state.get('context') or self.state.get('validity')!='VALID'):
            fast.update(valid=False,presence='unknown',direction='unknown',transition='NONE',reason='DISPLAY_EXPIRED_OR_INVALID_CONTEXT')
        image=None
        if self.image:
            result=self.image['result'];age=max(0,(now-int(result['measurement']['frame_complete_ns']))/1e6)
            label=result['freshness']
            if result['context']!=self.state.get('context') or self.state.get('validity')!='VALID':label='STALE_CONTEXT'
            elif result.get('scene_epoch')!=self.state.get('scene_epoch'):label='SCENE_CHANGED'
            elif age*1e6>self.image_stale_ns:label='HISTORICAL_AGE'
            image=dict(age_ms=age,label=label,k=result['measurement'].get('k',1),numeric_passed=result['valid_numeric'])
        return dict(state=self.state,fast=fast,image=image,history_available=self.history is not None,rejected=self.rejected)
