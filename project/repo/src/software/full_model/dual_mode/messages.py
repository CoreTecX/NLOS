"""Immutable full-measurement messages and bounded lifecycle utilities."""
from dataclasses import dataclass,field
from collections import deque
import hashlib,json,threading,time
import numpy as np
from contracts import decode_pfy,require

COMPAT_FIELDS=('source_session_id','compat_generation','geometry_epoch','model_id','model_hash','model_epoch',
               'calibration_id','radiometry_epoch','roi_id','mask_id','operator_id','feature_basis_id','required_format_version')


@dataclass(frozen=True)
class Context:
    encoded: str
    _key: tuple = field(init=False,repr=False,compare=False)

    def __post_init__(self):
        data=json.loads(self.encoded)
        object.__setattr__(self,'_key',tuple(data[k] for k in COMPAT_FIELDS))

    @classmethod
    def create(cls,**fields):
        require(all(k in fields and fields[k] is not None for k in COMPAT_FIELDS),'CONTEXT','Incomplete compatibility key')
        require(fields['required_format_version']=='PSFULL1_PFY','CONTEXT','Legacy/unknown ABI cannot be reinterpreted')
        for k in ('compat_generation','geometry_epoch','model_epoch','radiometry_epoch'):
            require(type(fields[k]) is int and 0<=fields[k]<2**64,'CONTEXT','Invalid epoch')
            fields[k]=str(fields[k])
        require(all(isinstance(fields[k],str) and len(fields[k])==n and all(c in '0123456789abcdef' for c in fields[k])
                    for k,n in (('model_hash',64),('model_id',32))),'CONTEXT','Model fingerprint required')
        return cls(json.dumps(fields,sort_keys=True,separators=(',',':'),allow_nan=False))

    @property
    def data(self):return json.loads(self.encoded)

    @property
    def key(self):return self._key

    def changed(self,**fields):
        d=self.data
        for k in ('compat_generation','geometry_epoch','model_epoch','radiometry_epoch'):d[k]=int(d[k])
        d.update(fields)
        return Context.create(**d)


@dataclass(frozen=True)
class Measurement:
    pfy: bytes
    context: Context
    frame_uid: str
    sensor_sequence: int
    frame_complete_ns: int
    source_kind: str
    sample_time_ns: int
    transform_id: str = 'IDENTITY'
    capture_signature: str = ''
    temporal_scope: str = 'UNKNOWN'
    members_json: str = '[]'
    _rgb: object = field(init=False,repr=False,compare=False)
    _scales: object = field(init=False,repr=False,compare=False)
    _sha256: str = field(init=False,repr=False,compare=False)
    _k: int = field(init=False,repr=False,compare=False)

    def __post_init__(self):
        require(isinstance(self.pfy,bytes),'MEASUREMENT','Immutable PFY bytes required')
        require(self.source_kind in ('SYNTHETIC','REPLAY_SEQUENCE','REPLAY_SINGLE','CAMERA'),'SOURCE_KIND','Unsupported source')
        require(type(self.sensor_sequence) is int and 0<=self.sensor_sequence<2**64,'FRAME_ID','Invalid original capture sequence')
        require(type(self.frame_complete_ns) is int and type(self.sample_time_ns) is int and
                self.frame_complete_ns>=0 and self.sample_time_ns>=0,'CLOCK','Invalid timestamp')
        _,scales,rgb=decode_pfy(self.pfy,self.context.data['model_id'])
        # The views ultimately reference immutable bytes, not a writable ndarray.
        object.__setattr__(self,'_rgb',rgb);object.__setattr__(self,'_scales',scales)
        object.__setattr__(self,'_sha256',hashlib.sha256(self.pfy).hexdigest())
        members=json.loads(self.members_json)
        require(isinstance(members,list) and len(members) in (0,1,2,4,8),'BURST_IDENTITY','Unsupported member count')
        object.__setattr__(self,'_k',len(members) or 1)
        if members:
            require(len({(v['source_session_id'],v['sensor_sequence']) for v in members})==len(members),
                    'BURST_IDENTITY','Repeated capture in aggregate')
            require(int(members[-1]['sensor_sequence'])==self.sensor_sequence,'BURST_IDENTITY','Wrong aggregate ending sequence')

    @property
    def k(self):return self._k

    @property
    def capture_key(self):return (self.context.data['source_session_id'],self.sensor_sequence)

    @property
    def rgb(self):return self._rgb.view()

    @property
    def scales(self):return self._scales.view()

    @property
    def sha256(self):return self._sha256

    @property
    def identity(self):
        return dict(frame_uid=self.frame_uid,original_sensor_frame_id=str(self.sensor_sequence),
                    context=self.context.data,measurement_sha256=self.sha256,
                    frame_complete_ns=str(self.frame_complete_ns),sample_time_ns=str(self.sample_time_ns),
                    source_kind=self.source_kind,transform_id=self.transform_id,capture_signature=self.capture_signature,
                    temporal_scope=self.temporal_scope,k=self.k,members=json.loads(self.members_json))


class Audit:
    def __init__(self,capacity=1024):
        require(type(capacity) is int and capacity>0,'CAPACITY','Positive audit capacity required')
        self.rows=deque(maxlen=capacity);self.dropped=0;self.total=0

    def add(self,event,**fields):
        self.total+=1
        if len(self.rows)==self.rows.maxlen:self.dropped+=1
        self.rows.append(dict(event=event,**fields))

    @property
    def complete(self):return self.dropped==0


class LeasePool:
    """Bounded immutable slots; stale generations cannot access reused storage."""
    def __init__(self,capacity=6,max_bytes=10027008):
        require(capacity>0 and max_bytes>0,'CAPACITY','Invalid pool bounds')
        self.slots=[dict(generation=0,readers=0,data=None) for _ in range(capacity)]
        self.max_bytes=max_bytes;self.lock=threading.Lock()

    def put(self,data):
        require(isinstance(data,bytes) and 0<len(data)<=self.max_bytes,'LEASE_SIZE','Invalid immutable buffer')
        with self.lock:
            for i,s in enumerate(self.slots):
                if s['readers']==0:
                    s.update(generation=s['generation']+1,readers=1,data=data)
                    return (i,s['generation'])
        return None

    def _slot(self,token):
        i,generation=token
        require(type(i) is int and 0<=i<len(self.slots),'STALE_LEASE','Unknown slot')
        s=self.slots[i]
        require(s['readers']>0 and s['generation']==generation,'STALE_LEASE','Stale/released lease')
        return s

    def get(self,token):
        with self.lock:return self._slot(token)['data']

    def retain(self,token):
        with self.lock:self._slot(token)['readers']+=1

    def release(self,token):
        with self.lock:
            s=self._slot(token);s['readers']-=1
            if not s['readers']:s['data']=None


class LatestSlot:
    def __init__(self):self.value=None;self.replaced=0
    def put(self,value):
        previous=self.value
        if previous is not None:self.replaced+=1
        self.value=value
        return previous
    def take(self):value=self.value;self.value=None;return value
