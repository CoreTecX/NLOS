"""Immutable completed-camera-frame adapter shared by live receive and RAW replay.

This module has no socket/display/board access. The diagnostic receiver can call
offer(); the consumer calls next(). Unknown geometry remains unknown.
"""
from pathlib import Path
import copy
import json
import math
import queue
import threading
import time

from contracts import Frame, FileSource, require, read_json, sha, u64


class CapturedFrameSource:
    def __init__(self, session_id, camera_identity, sensor_format, capture, origin='REAL_CAMERA_LIVE',
                 binding=None, base='.', delivery_mode=None, max_frame_bytes=16*1024*1024, clock=time.monotonic_ns):
        require(origin in ('REAL_CAMERA_LIVE','HISTORICAL_REAL_CAMERA_SINGLE_FRAME'), 'SOURCE_KIND', 'Real capture adapter requires explicit provenance')
        require(isinstance(session_id,str) and bool(session_id), 'CAPTURE_SESSION', 'Capture session required')
        require(all(isinstance(camera_identity.get(k),str) and camera_identity[k] for k in ('model','serial','mac')),
                'CAMERA_IDENTITY','Camera model/serial/MAC required')
        self.session_id=session_id
        self.identity=copy.deepcopy(camera_identity);self.fmt=copy.deepcopy(sensor_format)
        self.capture=copy.deepcopy(capture);self.origin=origin
        self.delivery_mode=delivery_mode or ('latest' if origin=='REAL_CAMERA_LIVE' else 'fifo')
        require(self.delivery_mode in ('latest','fifo','lossless'),'DELIVERY_MODE','Choose latest, fifo or lossless')
        require(type(max_frame_bytes) is int and 0<max_frame_bytes<=128*1024*1024,'FRAME_BUDGET','Invalid frame memory budget')
        require(all(type(self.fmt.get(k)) is int and 0<self.fmt[k]<=8192 for k in ('width','height')),
                'RAW_DIMENSIONS','Bounded sensor dimensions required')
        self.expected_bytes=self.fmt['width']*self.fmt['height']*2
        require(self.expected_bytes<=max_frame_bytes,'FRAME_BUDGET','Sensor frame exceeds the queue budget')
        self.max_frame_bytes=max_frame_bytes;self.clock=clock
        self.binding=copy.deepcopy(binding or {});self.base=Path(base)
        self.last_number=None;self.last_complete=None;self.last_timestamp=None
        self.lock=threading.Lock();self.pending=queue.Queue(maxsize=2)
        self.dropped=0;self.accepted=0;self.rejected=0
        self.offered=0;self.enqueued=0;self.dequeued=0;self.busy=0;self.producer_replaced=0;self.consumer_coalesced=0
        self.max_pending=0;self.last_delivery=None

    def offer(self, raw, meta, receiver_context=None):
        # This hook runs only after the receiver has reassembled a complete frame.
        # It does not perform calibration, solve, save, or block the receive loop.
        require(len(raw)==self.expected_bytes,'RAW_SIZE','Only bounded complete RAW may enter the queue')
        with self.lock:
            self.offered+=1
            if self.pending.full():
                if self.delivery_mode=='lossless':
                    self.busy+=1
                    return dict(outcome='BUSY_RETRY_SAME_FRAME',number=meta.get('number'))
                self.pending.get_nowait();self.dropped+=1;self.producer_replaced+=1
            item=(bytes(raw),copy.deepcopy(meta),copy.deepcopy(receiver_context),self.clock())
            self.pending.put_nowait(item);self.enqueued+=1;self.max_pending=max(self.max_pending,self.pending.qsize())
            return dict(outcome='QUEUED',number=meta.get('number'))

    def next(self):
        with self.lock:
            try:item=self.pending.get_nowait()
            except queue.Empty:return None
            if self.delivery_mode=='latest':
                while not self.pending.empty():
                    item=self.pending.get_nowait();self.dropped+=1;self.consumer_coalesced+=1
            self.dequeued+=1;now=self.clock()
            timing=dict(delivery_mode=self.delivery_mode,queue_wait_ms=max(0,now-item[3])/1e6,
                        enqueued_mono_ns=str(item[3]),dequeued_mono_ns=str(now),discarded_total=self.dropped,
                        sensor_capture_latency_measured=False)
            self.last_delivery=timing
        frame,desc,base,sample=self.accept(*item[:3])
        envelope=frame.envelope;envelope['delivery']=timing
        return Frame(frame.kind,frame.data,json.dumps(envelope,allow_nan=False)),desc,base,sample

    def status(self):
        with self.lock:
            return dict(delivery_mode=self.delivery_mode,offered=self.offered,enqueued=self.enqueued,dequeued=self.dequeued,
                        pending=self.pending.qsize(),dropped=self.dropped,busy=self.busy,max_pending=self.max_pending,
                        producer_replaced=self.producer_replaced,consumer_coalesced=self.consumer_coalesced,
                        accepted=self.accepted,rejected=self.rejected,max_queue_payload_bytes=2*self.expected_bytes,
                        last_delivery=self.last_delivery)

    def accept(self, raw, meta, receiver_context=None):
        try:
            return self._accept(raw,meta,receiver_context)
        except Exception:
            self.rejected+=1
            raise

    def _accept(self, raw, meta, receiver_context):
        raw=bytes(raw);m=copy.deepcopy(meta);fmt=copy.deepcopy(self.fmt);cap=copy.deepcopy(self.capture)
        require(m.get('complete') is True,'INCOMPLETE_FRAME','Receiver has not completed this frame')
        require(self.origin!='REAL_CAMERA_LIVE' or receiver_context is not None,
                'LIVE_CONTEXT','Live input requires receiver boot, format and radiometry context')
        number=m.get('number');block=m.get('block_id');ticks=m.get('timestamp_ticks')
        require(type(number) is int and number>0 and (self.last_number is None or number>self.last_number),
                'FRAME_ORDER','Duplicate or reversed sensor sequence; restart requires new session')
        require(type(block) is int and 0<=block<65536,'BLOCK_ID','Expected GVSP 16-bit block ID')
        u64(ticks)
        if ticks is not None and self.last_timestamp is not None:
            require(ticks>=self.last_timestamp,'CAMERA_TIME_ORDER','Camera timestamp reversed; establish a new capture session')
        leader,complete=m.get('leader_monotonic'),m.get('complete_monotonic')
        require(all(type(v) in (int,float) and math.isfinite(v) and v>=0 for v in (leader,complete)) and complete>=leader,
                'CAPTURE_TIME','Invalid receiver monotonic times')
        require(self.last_complete is None or complete>=self.last_complete,'CAPTURE_TIME','Receiver time reversed')
        require(type(m.get('packet_count')) is int and m['packet_count']>0,'PACKET_COUNT','Complete packet accounting required')
        if receiver_context is not None:
            ctx=receiver_context
            require(ctx.get('boot_id')==self.session_id,'CAPTURE_SESSION','Receiver boot/session changed')
            for name in ('width','height','pixel_format','dtype','roi_sensor_offset','binning'):
                require(fmt.get(name)==ctx['format'].get(name),'LIVE_FORMAT','Receiver format differs: '+name)
            for name in ('exposure_us','gain_raw'):
                require(cap.get(name)==ctx['capture'].get(name),'LIVE_CAPTURE','Receiver radiometry differs: '+name)
            require(ctx.get('sensor_test_pattern_off') is True,'TEST_PATTERN','Camera test pattern must be disabled')
        FileSource.decode_raw(raw,fmt)
        if 'sha256' in m:
            require(m['sha256']==sha(raw) and m.get('bytes')==len(raw),'RAW_SIDECAR','Archived RAW identity differs')
        if self.origin=='HISTORICAL_REAL_CAMERA_SINGLE_FRAME':
            require(m.get('sensor_test_pattern_off') is True,'TEST_PATTERN','Archived test-pattern state unknown')
        for key in ('exposure_us','gain_raw'):
            value=cap.get(key)
            require(type(value) in (int,float) and math.isfinite(value) and value>=0 and (key!='exposure_us' or value>0),
                    'CAPTURE_METADATA','Invalid '+key)
        cap.update(session_id=self.session_id,original_sensor_frame_id=number,original_block_id=block,camera_timestamp_ticks=ticks)
        source='CAMERA' if self.origin=='REAL_CAMERA_LIVE' else 'REPLAY_SINGLE'
        now_ns=time.monotonic_ns() if source=='REPLAY_SINGLE' else int(complete*1e9)
        env=dict(schema_version=1,source_kind=source,source_session_id=self.session_id,
            frame_uid=self.session_id+':'+str(number),original_capture_session_id=self.session_id,
            original_sensor_frame_id=str(number),original_block_id=str(block),camera_timestamp_ticks=u64(ticks),
            camera_tick_frequency_hz=cap.get('camera_tick_frequency_hz'),camera_clock_mapping=None,
            clock_domain=('REPLAY_HOST:' if source=='REPLAY_SINGLE' else 'BOARD_MONOTONIC:')+self.session_id,
            t_frame_complete_mono_ns=str(now_ns),original_complete_monotonic=complete,
            format=fmt,exposure_us=cap['exposure_us'],gain_raw=cap['gain_raw'],gain_units=cap.get('gain_units'),
            camera_identity=self.identity,sensor_test_pattern_off=True,complete=True,raw_sha256=sha(raw),hash_state='VERIFIED',
            sample_id=self.session_id+':'+str(number),is_true_motion_sequence=source=='CAMERA',
            hardware_accessed=source=='CAMERA',adapter_accessed_hardware=False,
            unknown_time_reason='No calibrated mapping between sensor clock and receiver clock')
        descriptor=dict(schema_version=1,kind='RAW',sample_id=env['sample_id'],complete=True,origin=self.origin,
                        format=fmt,capture=cap,camera_identity=self.identity,
                        model=None,model_id=None,calibration=None)
        require(set(self.binding)<=set(('model','model_id','calibration')),
                'BINDING_CONTEXT','Only model/calibration fields may be bound to a capture')
        descriptor.update(copy.deepcopy(self.binding))
        require(descriptor['kind']=='RAW' and descriptor['format']==fmt and descriptor['capture']==cap,
                'BINDING_CONTEXT','A model binding must not overwrite capture context')
        self.last_number=number;self.last_complete=complete;self.last_timestamp=ticks;self.accepted+=1
        return Frame('RAW',raw,json.dumps(env,allow_nan=False)),descriptor,self.base,now_ns


def r12_replay(data_directory):
    base=Path(data_directory);sample=read_json(base/'sample_manifest.json')
    raw=(base/'first_real_frame.raw').read_bytes();meta=read_json(base/'source_first_frame.json')
    require(len(raw)==sample['raw_bytes'] and sha(raw)==sample['raw_sha256'],'RAW_SIDECAR','R12 hash mismatch')
    fmt=dict(width=sample['width'],height=sample['height'],pixel_format=sample['pixel_format'],dtype=sample['numpy_dtype'],
             valid_bits=12,bayer_pattern='RGGB',row_stride_bytes=sample['width']*2,
             roi_sensor_offset=None,binning=sample['binning'],orientation=None)
    capture=dict(exposure_us=sample['exposure_us'],gain_raw=sample['gain_register'],gain_units='register',camera_tick_frequency_hz=None)
    source=CapturedFrameSource(sample['boot_id'],sample['camera_identity'],fmt,capture,
                               origin='HISTORICAL_REAL_CAMERA_SINGLE_FRAME',base=base)
    return source.accept(raw,meta)
