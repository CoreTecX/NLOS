"""Preserve image parameters, preview in software, capture per-job full RAW.
Does not load PL, execute a solver, or replace the existing reconstruction UI.
"""
from pathlib import Path
import argparse,fcntl,hashlib,json,os,queue,signal,socket,struct,subprocess,threading,time,traceback,uuid,shutil
import numpy as np
import cv2
from camera_transport import Control,address_probe,REG,HOST,CAMERA
from raw_contract import atomic_json,validate_parameters,validate_request,CaptureStore,FrameAssembler,IMAGE_KEYS

ROOT=Path('/home/root/ps_paper_live_20261103')
EXTRA={'offset_x':0x303e0,'offset_y':0x30420,'reverse_x':0x30608,'reverse_y':0x3060c,
       'gamma_enable':0x31308,'lut_enable':0x31700,'balance_white_auto':0x312e0}
REG.update(EXTRA)
def snapshot(c):return {k:c.read(addr) for k,addr in REG.items() if k not in ('start','stop')}

def set_mtu(value):
    # Cadence GEM only accepts MTU changes while the dedicated camera NIC is down.
    subprocess.run(['ip','link','set','dev','eth0','down'],check=True)
    try:subprocess.run(['ip','link','set','dev','eth0','mtu',str(value)],check=True)
    finally:subprocess.run(['ip','link','set','dev','eth0','up'],check=True)
    begin=time.monotonic();deadline=begin+15;stable_since=None
    while True:
        now=time.monotonic()
        if Path('/sys/class/net/eth0/carrier').read_text().strip()=='1':
            if stable_since is None:stable_since=now
            if now-begin>=4 and now-stable_since>=2:break
        else:stable_since=None
        if now>deadline:raise RuntimeError('Camera link did not stabilize after MTU change')
        time.sleep(.1)
def identity(c):
    result={}
    for name,addr,n in [('model',0x68,32),('serial',0xd8,16),('manufacturer',0x48,32)]:
        b=c.request(0x84,struct.pack('!IHH',addr,0,n))
        if struct.unpack_from('!I',b)[0]!=addr:raise ValueError('identity memory address')
        result[name]=b[4:].split(b'\0',1)[0].decode('ascii','replace')
    return result

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',required=True);ap.add_argument('--duration',type=float,default=0)
    args=ap.parse_args();out=Path(args.output)
    lock=open('/tmp/ps_fb15_camera.lock','a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    out.mkdir(parents=True,exist_ok=False)
    boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    state={'schema_version':1,'state':'STARTING','boot_id':boot,'pid':os.getpid(),'session':str(out),
           'complete_frames':0,'incomplete_frames':0,'displayed_frames':0,'display_skipped':0,
           'backend':'CAMERA_PREVIEW_WITH_PAPER_PC_CPU','reconstruction':False}
    command={'state':'IDLE','job_id':None,'backend':'capture_only'}
    stop=threading.Event();renderq=queue.Queue(1);rawq=queue.Queue(4)
    signals=lambda *_:stop.set()
    signal.signal(signal.SIGTERM,signals);signal.signal(signal.SIGINT,signals)
    c=rx=None;owned=added=started=False;changed=[];initial={};threads=[];initial_mtu=None;mtu_changed=False
    assetroot=Path(__file__).parent/'assets'
    assets={p.stem:cv2.cvtColor(cv2.imread(str(p)),cv2.COLOR_BGR2RGB) for p in assetroot.glob('*.png')}
    def save_state():state['last_state_monotonic']=time.monotonic();atomic_json(out/'state.json',state)
    def set_command(**kwargs):command.update(kwargs);atomic_json(out/'capture_state.json',command)
    save_state();set_command()
    (ROOT/'current.txt').write_text(str(out))
    job_guard=threading.Lock();holder={'job':None,'enqueued':0}
    try:
        cv2.setNumThreads(1)
        before_ip=subprocess.check_output(['ip','-4','addr','show','dev','eth0'],universal_newlines=True)
        atomic_json(out/'network_before.json',{'ip':before_ip})
        if 'inet ' in before_ip:raise RuntimeError('Existing IPv4 address; inspect owner before startup')
        if Path('/sys/class/net/eth0/carrier').read_text().strip()!='1':raise RuntimeError('Camera link down')
        initial_mtu=int(Path('/sys/class/net/eth0/mtu').read_text())
        if initial_mtu!=9000:
            mtu_changed=True;set_mtu(9000)
        address_probe();subprocess.run(['ip','addr','add',HOST+'/24','dev','eth0'],check=True);added=True
        c=Control();initial=snapshot(c);ident=identity(c)
        ident['timestamp_ticks_per_second']=(c.read(0x93c)<<32)|c.read(0x940)
        atomic_json(out/'camera_before.json',initial);atomic_json(out/'camera_identity.json',ident)
        if ident['model']!='MV-CS050-10GC':raise RuntimeError('Device model differs from verified register map')
        validate_parameters(initial)
        if initial['ccp']!=0:raise RuntimeError('Another controller owns camera')
        c.write(REG['ccp'],2);owned=True
        if c.read(REG['ccp'])!=2:raise RuntimeError('Control acquisition failed')
        c.write(REG['stop'],0)
        def change(key,value):
            if initial[key]!=value:
                changed.append(key);c.write(REG[key],value)
                if c.read(REG[key])!=value:raise RuntimeError('Transport write readback failed: '+key)
        change('heartbeat',10000)
        # Change only temporary transport, never exposure/ROI/binning/fps/pixel format.
        change('packet_delay',1000)
        change('packet_size',(initial['packet_size']&0x7fff0000)|8164)
        rx=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);rx.setsockopt(socket.SOL_SOCKET,33,32*1024*1024)
        rx.bind((HOST,0));rx.settimeout(.1)
        change('destination',struct.unpack('!I',socket.inet_aton(HOST))[0])
        change('port',(initial['port']&0xffff0000)|rx.getsockname()[1])
        active=snapshot(c);contract=validate_parameters(active)
        if any(initial[k]!=active[k] for k in IMAGE_KEYS):raise RuntimeError('Image settings unexpectedly changed')
        atomic_json(out/'camera_active.json',active);atomic_json(out/'raw_contract.json',contract)
        atomic_json(out/'parameter_policy.json',{'image_parameters_preserved':True,'temporary_transport_changes':changed,
           'before':{k:initial[k] for k in changed},'active':{k:active[k] for k in changed}})
        state.update(width=active['width'],height=active['height'],exposure_us=active['exposure_us'],
                     network_mtu=9000,initial_network_mtu=initial_mtu,
                     binning=contract['binning_xy'],pixel_format='BayerRG12',identity=ident,
                     preserved_image_parameters=True,receiver_buffer_bytes=rx.getsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF))
        def writer():
            while not stop.is_set() or not rawq.empty():
                try:job,raw,meta=rawq.get(timeout=.2)
                except queue.Empty:continue
                try:
                    if job.info['state']!='CAPTURING':continue
                    if job.accept(raw,meta):set_command(captured=len(job.records))
                    if len(job.records)==job.req['frames']:
                        info=job.finish(snapshot(c));set_command(state='COMPLETE',job_id=job.req['job_id'],captured=len(job.records),quality=info['quality'])
                        with job_guard:
                            if holder['job'] is job:holder['job']=None
                except Exception as e:
                    job.fail(repr(e));set_command(state='ERROR',error=repr(e))
                    with job_guard:
                        if holder['job'] is job:holder['job']=None
        def render():
            try:
                os.environ['DISPLAY']=':0';name='PenumbraSight SC03 | LIVE + RAW'
                cv2.namedWindow(name,cv2.WINDOW_NORMAL);cv2.setWindowProperty(name,cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)
                last=0;paper_image=None;paper_job=None;displayed_paper_job=None
                while not stop.is_set() or not renderq.empty():
                    try:raw,meta=renderq.get(timeout=.2)
                    except queue.Empty:continue
                    a=np.frombuffer(raw,dtype=contract['dtype']).reshape(active['height'],active['width'])
                    # Preview only: subselect entire Bayer cells, independent of saved full RAW.
                    step=max(1,int(np.ceil(max(a.shape)/1008.)));stride=2*step
                    r=a[0::stride,0::stride];g=(a[0::stride,1::stride].astype(np.float32)+a[1::stride,0::stride])*.5;b=a[1::stride,1::stride]
                    rgb=np.stack((r,g,b),axis=2);lo,hi=np.percentile(rgb,[.5,99.5]);span=max(hi-lo,64.)
                    rgb=np.clip((rgb-lo)*255/span,0,255).astype('uint8')
                    canvas=np.full((800,1280,3),(8,35,76),np.uint8)
                    for x in (22,650):cv2.rectangle(canvas,(x,126),(x+608,696),(0,151,201),-1)
                    for key,x,y in [('title',28,22),('left',40,84),('right',668,84)]:
                        asset=assets.get(key)
                        if asset is not None:canvas[y:y+asset.shape[0],x:x+asset.shape[1]]=asset
                    h,w=rgb.shape[:2];scale=min(584/w,540/h);nw,nh=int(w*scale),int(h*scale)
                    x,y=34+(584-nw)//2,146+(540-nh)//2
                    canvas[y:y+nh,x:x+nw]=cv2.resize(rgb,(nw,nh))
                    canvas[140:687,662:1250]=(8,35,76)
                    paper={'state':'WAITING'}
                    pf=out/'paper_state.json'
                    if pf.exists():paper=json.loads(pf.read_text())
                    pj=paper.get('job_id');ps=paper['state']
                    if ps=='COMPLETE' and paper.get('boot_id')==boot:
                        if pj!=paper_job:
                            image_path=out/'paper_jobs'/pj/'reconstruction.png'
                            data=image_path.read_bytes()
                            if hashlib.sha256(data).hexdigest()!=paper['image_sha256']:raise ValueError('Paper image checksum mismatch')
                            paper_image=cv2.cvtColor(cv2.imread(str(image_path)),cv2.COLOR_BGR2RGB);paper_job=pj
                        if paper_image is not None:
                            canvas[177:641,666:1242]=cv2.resize(paper_image,(576,464),interpolation=cv2.INTER_NEAREST)
                            displayed_paper_job=pj
                    else:paper_image=None;paper_job=None;displayed_paper_job=None
                    if ps!='COMPLETE':
                        label=assets.get('running' if ps=='RUNNING' else 'not_reconstructed')
                        if label is not None:canvas[370:370+label.shape[0],850:850+label.shape[1]]=label
                    state.update(paper_state=ps,paper_job_id=pj,paper_displayed_job_id=displayed_paper_job)
                    if ps=='COMPLETE':
                        atomic_json(out/'paper_display_ack.json',dict(job_id=pj,image_sha256=paper['image_sha256'],boot_id=boot,displayed_monotonic=time.monotonic(),camera_frame=meta['number']))
                    cv2.imshow(name,cv2.cvtColor(canvas,cv2.COLOR_RGB2BGR));cv2.waitKey(1)
                    state['displayed_frames']+=1;state['last_display_frame']=meta['number']
                    if time.monotonic()-last>3:
                        cv2.imwrite(str(out/'latest_preview.jpg'),cv2.cvtColor(canvas,cv2.COLOR_RGB2BGR),[cv2.IMWRITE_JPEG_QUALITY,85])
                        cv2.imwrite(str(out/'camera_preview.jpg'),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR),[cv2.IMWRITE_JPEG_QUALITY,90])
                        atomic_json(out/'preview_metadata.json',{'frame':meta,'display_only_stretch':[float(lo),float(hi)],'raw_unchanged':True})
                        last=time.monotonic()
                cv2.destroyAllWindows()
            except Exception:state['render_error']=traceback.format_exc();stop.set()
        def heartbeat():
            while not stop.wait(.5):
                try:
                    if c.read(REG['ccp'])!=2:raise RuntimeError('Camera ownership lost')
                except Exception:state['heartbeat_error']=traceback.format_exc();stop.set()
        for fn in (writer,render,heartbeat):
            t=threading.Thread(target=fn);t.start();threads.append(t)
        assembler=FrameAssembler(active);timestamps=[]
        c.write(REG['start'],0x80000000);started=True;state['state']='LIVE'
        begin=time.monotonic();last_frame=begin;last_state=begin;last_poll=0
        while not stop.is_set():
            now=time.monotonic()
            if args.duration and now-begin>=args.duration:break
            if now-last_frame>12:raise RuntimeError('No complete frame for 12 seconds')
            if now-last_state>1:
                state.update(incomplete_frames=assembler.incomplete,invalid_packets=assembler.invalid,uptime_seconds=now-begin)
                save_state();last_state=now
            if now-last_poll>.1:
                last_poll=now
                if (out/'stop').exists():break
                if (out/'request.json').exists():
                    request_path=out/'request.json'
                    try:
                        req=json.loads(request_path.read_text());request_path.unlink()
                        with job_guard:
                            if holder['job'] is not None:raise ValueError('Capture busy')
                            validate_request(req,boot,now)
                            required=active['payload']*req['frames']+active['width']*active['height']*4+192*1024*1024
                            if shutil.disk_usage(str(out)).free<required:raise RuntimeError('Insufficient space; export retained jobs first')
                            holder['job']=CaptureStore(out,req,boot,snapshot(c),ident)
                            holder['enqueued']=0
                            set_command(state='CAPTURING',job_id=req['job_id'],captured=0,total=req['frames'],error=None)
                    except Exception as e:
                        atomic_json(out/'last_rejected_request.json',{'error':repr(e),'monotonic':time.monotonic()})
                        if holder['job'] is None:set_command(state='ERROR',error=repr(e))
            try:data,peer=rx.recvfrom(16384)
            except socket.timeout:continue
            if peer[0]!=CAMERA:continue
            frame=assembler.push(data,time.monotonic())
            if frame is None:continue
            raw,meta=frame;last_frame=time.monotonic();state['complete_frames']+=1;meta['number']=state['complete_frames']
            timestamps.append(last_frame);timestamps=timestamps[-40:]
            state['receive_fps']=(len(timestamps)-1)/(timestamps[-1]-timestamps[0]) if len(timestamps)>1 else 0
            if state['complete_frames']==1:
                (out/'first_frame.raw').write_bytes(raw)
                atomic_json(out/'first_frame.json',dict(meta,contract=contract,sha256=hashlib.sha256(raw).hexdigest()))
            with job_guard:job=holder['job']
            if job is not None and job.info['state']=='CAPTURING':
                if meta['leader_monotonic']>job.req.get('not_before_monotonic',job.req['requested_monotonic']) and holder['enqueued']<job.req['frames']:
                    try:rawq.put_nowait((job,raw,meta));holder['enqueued']+=1
                    except queue.Full:state['capture_queue_dropped_frames']=state.get('capture_queue_dropped_frames',0)+1
            if renderq.full():
                try:renderq.get_nowait();state['display_skipped']+=1
                except queue.Empty:pass
            renderq.put_nowait((raw,meta))
        state['state']='STOPPING'
    except Exception:state.update(state='ERROR',error=traceback.format_exc());print(state['error'],flush=True)
    finally:
        stop.set()
        for thread in threads:thread.join(timeout=15)
        cleanup=[]
        if any(t.is_alive() for t in threads):cleanup.append('worker did not stop before deadline')
        job=holder['job']
        if job is not None and job.info['state']=='CAPTURING':job.fail('Capture interrupted');set_command(state='ERROR',error='Capture interrupted')
        if c and owned:
            try:c.write(REG['stop'],0)
            except Exception as e:cleanup.append('stop '+repr(e))
            for key in reversed(changed):
                try:
                    c.write(REG[key],initial[key])
                    if c.read(REG[key])!=initial[key]:raise RuntimeError(key)
                except Exception as e:cleanup.append('restore '+repr(e))
            try:
                restored=snapshot(c);atomic_json(out/'camera_restored.json',restored)
                if any(initial[k]!=restored[k] for k in IMAGE_KEYS):raise RuntimeError('image parameter restoration mismatch')
                c.write(REG['ccp'],0)
                if c.read(REG['ccp'])!=0:raise RuntimeError('ownership release failed')
            except Exception as e:cleanup.append(repr(e))
        if rx:rx.close()
        if c:c.close()
        if added:
            try:subprocess.run(['ip','addr','del',HOST+'/24','dev','eth0'],check=True)
            except Exception as e:cleanup.append(repr(e))
        if mtu_changed:
            try:set_mtu(initial_mtu)
            except Exception as e:cleanup.append('MTU restore '+repr(e))
        state.update(cleanup_errors=cleanup,transport_restored=owned and not cleanup)
        if state['state']!='ERROR' and not cleanup and not state.get('render_error') and not state.get('heartbeat_error'):state['state']='STOPPED'
        else:state['state']='ERROR'
        save_state();os.sync();print(json.dumps(state),flush=True)

if __name__=='__main__':main()
