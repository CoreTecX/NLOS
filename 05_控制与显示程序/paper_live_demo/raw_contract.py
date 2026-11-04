"""Lossless BayerRG12 capture contract; deliberately no reconstruction backend."""
from pathlib import Path
import hashlib,json,os,re,shutil,time
import numpy as np

FORMAT=0x01100011
IMAGE_KEYS=('width','height','offset_x','offset_y','pixel_format','super_bayer','binning',
 'reverse_x','reverse_y','exposure_us','exposure_auto','gain','gain_auto','gamma_enable',
 'lut_enable','balance_white_auto','trigger_mode','acquisition_mode','test_pattern','compression','chunk',
 'frame_rate','fps_unit','payload')

def atomic_json(path,obj):
    path=Path(path);tmp=path.with_name(path.name+'.tmp')
    tmp.write_text(json.dumps(obj,indent=2),encoding='utf-8');tmp.replace(path)

def validate_parameters(params):
    w,h=params['width'],params['height']
    if not (w>0 and h>0 and w%2==0 and h%2==0):raise ValueError('Requires an even Bayer ROI')
    if params['pixel_format']!=FORMAT:raise ValueError('Only unpacked BayerRG12 is supported; camera unchanged')
    if params['payload']!=w*h*2:raise ValueError('Payload/container mismatch')
    if any(params[k]!=0 for k in ('test_pattern','compression','chunk','super_bayer','trigger_mode','stream_config')):
        raise ValueError('Unsupported stream setting; no implicit camera reconfiguration')
    if params['acquisition_mode']!=2:raise ValueError('Camera must already be in continuous acquisition mode')
    if params['reverse_x'] or params['reverse_y'] or params['offset_x']%2 or params['offset_y']%2:
        raise ValueError('Noncanonical Bayer phase requires explicit validation; camera unchanged')
    if any(params[k]!=0 for k in ('exposure_auto','gain_auto','balance_white_auto')):
        raise ValueError('Automatic exposure/gain/white balance must be reviewed before fixed-parameter capture')
    return {'width':w,'height':h,'pixel_format':'BayerRG12','pfnc':FORMAT,'container_bits':16,
      'valid_bits':12,'alignment':'LSB','dtype':'>u2' if params['packet_size']&(1<<29) else '<u2',
      'row_bytes':w*2,'payload_bytes':w*h*2,'bayer_pattern':'RGGB',
      'bayer_phase_basis':'PFNC BayerRG12, even ROI offsets, reverse off; physical color validation pending SC05',
      'gamma_enable':params['gamma_enable'],'lut_enable':params['lut_enable'],
      'linearity_calibrated':False,'sensor_offset_xy':[params['offset_x'],params['offset_y']],
      'binning_xy':[params['binning']&65535,params['binning']>>16]}

def validate_request(req,boot,now):
    if req.get('boot_id')!=boot:raise ValueError('stale boot')
    if not re.fullmatch('[a-f0-9]{32}',req.get('job_id','')):raise ValueError('invalid job id')
    count=req.get('frames')
    if type(count) is not int or not 1<=count<=64:raise ValueError('frames must be 1..64')
    age=now-float(req['requested_monotonic'])
    if not 0<=age<=30:raise ValueError('expired/future request')

def require_same_parameters(before,after):
    diff={k:[before.get(k),after.get(k)] for k in IMAGE_KEYS if before.get(k)!=after.get(k)}
    if diff:raise ValueError('camera parameters changed during job: '+repr(diff))

def validate_leader(meta,p):
    if (meta['width'],meta['height'],meta['pfnc'])!=(p['width'],p['height'],p['pixel_format']):
        raise ValueError('leader size/format differs from camera readback')
    xy=(meta['offset_x'],meta['offset_y'])
    if xy not in ((0,0),(p['offset_x'],p['offset_y'])):
        raise ValueError('unsupported wire offset: '+repr(xy))
    return xy

def summary_rgb(raw_mean):
    r=raw_mean[0::2,0::2];g=(raw_mean[0::2,1::2]+raw_mean[1::2,0::2])*.5;b=raw_mean[1::2,1::2]
    rgb=np.stack((r,g,b),axis=2)
    h,w=rgb.shape[:2]
    nh=max(d for d in range(1,min(128,h)+1) if h%d==0)
    nw=max(d for d in range(1,min(128,w)+1) if w%d==0)
    return rgb.reshape(nh,h//nh,nw,w//nw,3).mean(axis=(1,3)).astype('<f4'),{'rows':nh,'cols':nw,
      'bayer_cell_block_yx':[h//nh,w//nw],'scope':'full_ROI_linear_cell_means_no_crop_no_stretch_no_white_balance'}

class CaptureStore:
    def __init__(self,session,req,boot,parameters,identity):
        validate_request(req,boot,time.monotonic())
        self.contract=validate_parameters(parameters)
        self.req=req;self.params=dict(parameters);self.path=Path(session)/'jobs'/req['job_id']
        self.path.mkdir(parents=True,exist_ok=False)
        self.records=[];self.last_ts=None;self.acc=None
        self.info={'schema_version':1,'state':'CAPTURING','job_id':req['job_id'],'boot_id':boot,
          'request':req,'contract':self.contract,'identity':identity,'parameters_before':parameters,
          'frames':self.records,'backend':'capture_only','reconstruction':False,
          'time_basis':'camera timestamp ticks plus host monotonic; board wall clock is not trusted'}
        self.save()
    def save(self):atomic_json(self.path/'capture.json',self.info)
    def accept(self,raw,meta):
        if not meta.get('complete'):raise ValueError('incomplete frame')
        if meta['leader_monotonic']<=max(self.req['requested_monotonic'],self.req.get('not_before_monotonic',0)):return False
        if self.last_ts is not None and meta['timestamp_ticks']<=self.last_ts:raise ValueError('duplicate/nonmonotonic camera timestamp')
        if len(raw)!=self.contract['payload_bytes']:raise ValueError('invalid raw length')
        xy=validate_leader(meta,self.params)
        if self.records and xy!=(self.records[0]['offset_x'],self.records[0]['offset_y']):
            raise ValueError('wire offsets changed during job')
        a=np.frombuffer(raw,dtype=self.contract['dtype']).reshape(self.contract['height'],self.contract['width'])
        if int(a.max())>4095:raise ValueError('RAW12 alignment violation')
        name='frame_%04d.raw'%(len(self.records)+1)
        partial=self.path/(name+'.partial');partial.write_bytes(raw);partial.rename(self.path/name)
        row=dict(meta,file=name,bytes=len(raw),sha256=hashlib.sha256(raw).hexdigest())
        row['quality']={'min':int(a.min()),'max':int(a.max()),'mean':float(a.mean()),
                        'zero_fraction':float(np.mean(a==0)),'saturated_fraction':float(np.mean(a>=4095))}
        self.records.append(row);self.last_ts=meta['timestamp_ticks']
        if self.acc is None:self.acc=a.astype(np.float64)
        else:self.acc+=a
        self.info['captured']=len(self.records);self.save()
        return True
    def finish(self,parameters):
        require_same_parameters(self.params,parameters)
        if len(self.records)!=self.req['frames']:raise ValueError('wrong final frame count')
        mean=(self.acc/len(self.records)).astype('<f4');self.acc=None
        np.save(str(self.path/'mean_raw.npy'),mean,allow_pickle=False)
        rgb,spatial=summary_rgb(mean);np.save(str(self.path/'measurement_rgb.npy'),rgb,allow_pickle=False)
        self.info.update(state='COMPLETE',parameters_after=parameters,parameters_stable=True,
          summary=spatial,quality={'min':float(mean.min()),'max':float(mean.max()),
          'mean':float(mean.mean()),'percentiles':np.percentile(mean,[1,50,95,99,99.9]).tolist(),
          'saturated_fraction':float(np.mean(mean>=4095)),'zero_fraction':float(np.mean(mean==0))},
          finished_monotonic=time.monotonic())
        self.info['quality']['scope']='mean_raw; per-frame quality is recorded in frames[].quality'
        self.info['derived_files']={name:{'bytes':(self.path/name).stat().st_size,'sha256':hashlib.sha256((self.path/name).read_bytes()).hexdigest()} for name in ('mean_raw.npy','measurement_rgb.npy')}
        self.save();return self.info
    def fail(self,error):
        self.acc=None;self.info.update(state='ERROR',error=str(error),finished_monotonic=time.monotonic());self.save()

class FrameAssembler:
    def __init__(self,params):self.params=params;self.frame=None;self.incomplete=0;self.invalid=0;self.wire_xy=None
    def push(self,data,now):
        import struct
        if len(data)<8:self.invalid+=1;return None
        status,block,info=struct.unpack_from('!HHI',data);kind=info>>24;pid=info&0xffffff;body=data[8:]
        if status or kind&0x80:self.invalid+=1;return None
        if kind==1:
            if self.frame is not None:self.incomplete+=1
            self.frame=None
            if len(body)<36:self.invalid+=1;return None
            flags,typ,ts,pf,w,h,x,y,px,py=struct.unpack_from('!HHQIIIIIHH',body)
            p=self.params
            if (typ,px,py)!=(1,0,0):raise ValueError('Unsupported payload type/padding')
            xy=validate_leader(dict(width=w,height=h,pfnc=pf,offset_x=x,offset_y=y),p)
            if self.wire_xy is not None and xy!=self.wire_xy:raise ValueError('wire offsets changed during stream')
            self.wire_xy=xy
            self.frame={'block_id':block,'parts':{},'timestamp_ticks':ts,'leader_monotonic':now,
                        'width':w,'height':h,'offset_x':x,'offset_y':y,'pfnc':pf,
                        'sensor_offset_xy':[p['offset_x'],p['offset_y']],
                        'wire_offset_interpretation':'zero_relative_ROI' if xy==(0,0) else 'sensor_coordinates'}
        elif kind==3 and self.frame is not None and self.frame['block_id']==block:
            parts=self.frame['parts']
            if not 0<pid<100000:raise ValueError('invalid packet ID')
            if pid in parts and parts[pid]!=body:raise ValueError('conflicting duplicate packet')
            parts[pid]=body
        elif kind==2 and self.frame is not None and self.frame['block_id']==block:
            f=self.frame;self.frame=None;parts=f.pop('parts')
            if pid<=1 or len(parts)!=pid-1 or set(parts)!=set(range(1,pid)):
                self.incomplete+=1;return None
            raw=b''.join(parts[i] for i in range(1,pid))
            if len(raw)!=self.params['payload']:self.incomplete+=1;return None
            f.update(complete=True,complete_monotonic=now,packet_count=pid-1)
            return raw,f
        return None
