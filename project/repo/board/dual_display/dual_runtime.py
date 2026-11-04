from pathlib import Path
import json,time,os,hashlib,subprocess,queue,threading,re,shutil
import numpy as np
import cv2
ROOT=Path('/home/root/ps_dualview_20261030')
SESSION=None
def atomic(p,obj):
    t=p.with_suffix('.tmp');t.write_text(json.dumps(obj,indent=2));t.replace(p)
class Session:
    def __init__(self,out,exposure_us=200000):
        global SESSION
        SESSION=self;self.out=out;self.exposure_us=exposure_us;self.boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        self.capture=None;self.proc=None;self.frames=[];self.metas=[];self.last_poll=0;self.lock=threading.Lock()
        self.status=dict(state='WAITING',boot_id=self.boot,session=str(out),job_id=None,backend='pl')
        atomic(out/'command_state.json',self.status);self.q=queue.Queue(maxsize=32)
        threading.Thread(target=self.run,daemon=True).start()
    def feed(self,raw,meta,width,height,dtype):
        try:self.q.put_nowait((raw,meta,width,height,dtype))
        except queue.Full:pass
    def set(self,**kw):
        self.status.update(kw);atomic(self.out/'command_state.json',self.status)
    def run(self):
        while True:
            try:
                raw,meta,w,h,dtype=self.q.get(timeout=.5)
                if meta['number']%8==0:
                    a=np.frombuffer(raw,dtype).reshape(h,w)
                    def levels(b):
                        return dict(percentiles=[float(x) for x in np.percentile(b,[1,50,95,99,99.9])],saturated_fraction=float(np.mean(b>=4090)),black_fraction=float(np.mean(b<=4)))
                    atomic(self.out/'brightness.json',dict(frame=meta['number'],monotonic=time.monotonic(),exposure_us=self.exposure_us,raw_max=4095,full=levels(a),center=levels(a[h//4:3*h//4,w//4:3*w//4])))
                if self.proc is not None:
                    if self.proc.poll() is not None:
                        result=json.loads((self.capture/'status.json').read_text())
                        if result['state'] not in ('COMPLETE','NONCONVERGED','ERROR'):result=dict(state='ERROR',error='Worker exited without final result')
                        self.set(**result);self.proc=None;self.capture=None
                    continue
                request=self.out/'request.json'
                if self.capture is None and request.exists():
                    r=json.loads(request.read_text());request.unlink()
                    if r.get('restore_job'):
                        ident=r['restore_job'];assert re.fullmatch('[a-f0-9]{32}',ident)
                        saved=json.loads((self.out/'jobs'/ident/'status.json').read_text())
                        assert saved['boot_id']==self.boot and saved['state'] in ('COMPLETE','NONCONVERGED')
                        assert saved['solver']['backend']=='pl'
                        self.set(**saved);continue
                    if r.get('reset'):
                        self.status=dict(state='WAITING',boot_id=self.boot,session=str(self.out),job_id=None,backend='pl');atomic(self.out/'command_state.json',self.status);continue
                    ident=r['job_id'];assert re.fullmatch('[a-f0-9]{32}',ident)
                    if r['boot_id']!=self.boot:raise ValueError('Stale boot request')
                    if time.monotonic()-r['requested_monotonic']>15:raise ValueError('Expired capture request')
                    if shutil.disk_usage(str(self.out)).free<128*1024*1024:raise RuntimeError('Storage low: preserve results and free space before another capture')
                    # Discard frames received before the command was issued.
                    self.capture=self.out/'jobs'/ident;self.capture.mkdir(parents=True,exist_ok=False)
                    self.request=r;self.frames=[];self.metas=[];self.set(state='CAPTURING',job_id=ident,captured=0,total=16,error=None)
                if self.capture is not None:
                    if meta['leader_monotonic']<=self.request['requested_monotonic']:continue
                    assert (w,h)==(612,512)
                    index=len(self.frames)+1;(self.capture/('%02d.raw'%index)).write_bytes(raw)
                    self.frames.append(np.frombuffer(raw,dtype).reshape(h,w).astype(np.float64))
                    self.metas.append(dict(meta,raw_sha256=hashlib.sha256(raw).hexdigest(),dtype=dtype))
                    self.set(captured=index)
                    if index==16:
                        np.save(str(self.capture/'mean_raw.npy'),np.mean(self.frames,axis=0));self.frames=[]
                        mean=np.load(str(self.capture/'mean_raw.npy'));quality=dict(mean=float(mean.mean()),median=float(np.median(mean)),saturated_fraction=float(np.mean(mean>=4090)))
                        atomic(self.capture/'input.json',dict(job_id=self.capture.name,boot_id=self.boot,frames=self.metas,request=self.request,exposure_us=self.exposure_us,input_quality=quality,scope='UNCALIBRATED_CAMERA_DIAGNOSTIC'))
                        self.set(state='RUNNING');f=(self.capture/'worker.log').open('w')
                        self.proc=subprocess.Popen(['/usr/bin/python3','-B',str(ROOT/'pl_worker.py'),str(self.capture)],stdout=f,stderr=subprocess.STDOUT);f.close()
            except queue.Empty:continue
            except Exception as e:
                self.frames=[];self.capture=None;self.set(state='ERROR',error=repr(e))
class Display:
    def __init__(self):
        os.environ['DISPLAY']=':0';self.name='PenumbraSight | LIVE + FPGA';self.draw_count=0;self.last_check=0;self.last_job=None;self.result=None
        self.assets={p.stem:cv2.cvtColor(cv2.imread(str(p)),cv2.COLOR_BGR2RGB) for p in (ROOT/'assets').glob('*.png')}
        cv2.namedWindow(self.name,cv2.WINDOW_NORMAL);cv2.setWindowProperty(self.name,cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)
    def label(self,c,name,x,y):
        a=self.assets[name];h,w=a.shape[:2];c[y:y+h,x:x+w]=a
    def present(self,rgb,textlines):
        c=np.full((800,1280,3),(12,18,30),np.uint8)
        for x in (22,650):cv2.rectangle(c,(x,126),(x+608,696),(38,52,72),-1)
        self.label(c,'title',28,22);self.label(c,'left',40,84);self.label(c,'right',668,84)
        h,w=rgb.shape[:2];scale=min(584/w,540/h);nw,nh=int(w*scale),int(h*scale)
        c[146+(540-nh)//2:146+(540-nh)//2+nh,34+(584-nw)//2:34+(584-nw)//2+nw]=cv2.resize(rgb,(nw,nh))
        status=SESSION.status if SESSION else {'state':'WAITING'};state=status['state'];job=status.get('job_id')
        if state in ('COMPLETE','NONCONVERGED') and job!=self.last_job:
            self.result=cv2.imread(str(SESSION.out/'jobs'/job/'reconstruction.png'));self.last_job=job
        if state=='WAITING':self.result=None;self.last_job=None
        if self.result is not None:
            im=cv2.cvtColor(self.result,cv2.COLOR_BGR2RGB);im=cv2.resize(im,(576,464),interpolation=cv2.INTER_NEAREST);c[177:641,666:1242]=im
        else:self.label(c,'waiting' if state=='WAITING' else 'processing' if state in ('RUNNING','CAPTURING') else 'failed',835,355)
        self.label(c,'diagnostic',660,707)
        if state in ('CAPTURING','RUNNING'):self.label(c,'processing',840,650)
        if state=='ERROR':self.label(c,'failed',840,650)
        if state=='COMPLETE':self.label(c,'complete',840,650)
        if state=='NONCONVERGED':self.label(c,'nonconverged',790,650)
        cv2.putText(c,'LIVE | '+textlines[-1][:80],(38,735),0,.42,(180,201,220),1)
        detail='Status: '+state+' | '+(job[:10] if job else 'click PC button to reconstruct')
        cv2.putText(c,detail,(38,774),0,.55,(123,211,237),1)
        cv2.imshow(self.name,cv2.cvtColor(c,cv2.COLOR_RGB2BGR));cv2.waitKey(1);self.draw_count+=1
        return c,hashlib.sha256(c.tobytes()).hexdigest(),None
    def close(self):cv2.destroyAllWindows()
