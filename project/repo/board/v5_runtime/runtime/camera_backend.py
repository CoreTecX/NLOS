"""Device-bound diagnostic GigE preview. Not the production reconstruction backend.

Uses only addresses verified against this camera's GenICam XML. Stops acquisition,
restores all changed registers and removes the owned IPv4 address on clean exit.
Incomplete GVSP frames are counted and discarded, never displayed or saved.
"""
from pathlib import Path
import argparse,fcntl,hashlib,json,mmap,os,queue,signal,socket,struct,threading,time,traceback
import subprocess
import numpy as np
import cv2

HOST='192.168.1.10';CAMERA='192.168.1.64';BOOT='4a1634cc-7db2-4d38-90b5-3beda4a9cb91'
REG={'width':0x30360,'height':0x303a0,'pixel_format':0x30610,'super_bayer':0x30614,
 'test_pattern':0x30680,'compression':0x306c8,'acquisition_mode':0x30800,'start':0x30804,'stop':0x30808,
 'frame_rate':0x3081c,'fps_unit':0x10238,'trigger_mode':0x30840,'exposure_us':0x30b04,
 'exposure_auto':0x30b08,'gain':0x31200,'gain_auto':0x31220,'payload':0x31900,
 'chunk':0x33c00,'heartbeat':0x938,'ccp':0xa00,'port':0xd00,'packet_size':0xd04,
 'packet_delay':0xd08,'destination':0xd18,'stream_config':0x960,'binning':0x304a0}

def write_json(p,value):
 tmp=p.with_suffix(p.suffix+'.tmp');tmp.write_text(json.dumps(value,indent=2));tmp.replace(p)

class Control:
 def __init__(self):
  self.s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);self.s.bind((HOST,0));self.s.settimeout(.35);self.rid=0x100;self.lock=threading.Lock()
 def request(self,cmd,payload):
  with self.lock:
   self.rid=(self.rid+1)%65535 or 1;rid=self.rid
   packet=struct.pack('!BBHHH',0x42,1,cmd,len(payload),rid)+payload
   for attempt in range(3):
    self.s.sendto(packet,(CAMERA,3956));end=time.monotonic()+.4
    while time.monotonic()<end:
     try:data,peer=self.s.recvfrom(4096)
     except socket.timeout:break
     if peer!=(CAMERA,3956) or len(data)<8:continue
     status,ack,n,seq=struct.unpack_from('!HHHH',data)
     if seq!=rid:continue
     if status or ack!=cmd+1 or len(data)!=8+n:raise RuntimeError('GVCP status=%04x ack=%04x cmd=%04x'%(status,ack,cmd))
     return data[8:]
   raise RuntimeError('GVCP timeout cmd=%04x'%cmd)
 def read(self,a):
  b=self.request(0x80,struct.pack('!I',a));assert len(b)==4
  return struct.unpack('!I',b)[0]
 def write(self,a,v):
  b=self.request(0x82,struct.pack('!II',a,v));assert len(b)==4,b.hex()
 def close(self):self.s.close()

def address_probe():
 mac=bytes.fromhex(Path('/sys/class/net/eth0/address').read_text().strip().replace(':',''))
 s=socket.socket(socket.AF_PACKET,socket.SOCK_RAW,socket.htons(0x0806));s.bind(('eth0',0));s.settimeout(.1)
 p=b'\xff'*6+mac+struct.pack('!HHHBBH',0x0806,1,0x0800,6,4,1)+mac+b'\0'*4+b'\0'*6+socket.inet_aton(HOST)
 try:
  for _ in range(3):
   s.send(p);end=time.monotonic()+.4
   while time.monotonic()<end:
    try:f=s.recv(4096)
    except socket.timeout:continue
    if len(f)>=42 and f[6:12]!=mac and (f[28:32]==socket.inet_aton(HOST) or (f[28:32]==b'\0'*4 and f[38:42]==socket.inet_aton(HOST))):raise RuntimeError('Camera-side IPv4 conflict')
 finally:s.close()

class Display:
 def __init__(self):
  self.fd=os.open('/dev/fb0',os.O_RDWR);b=bytearray(160);fcntl.ioctl(self.fd,0x4600,b,True);self.v=struct.unpack('<40I',b)
  v=self.v;assert v[6]==16 and v[8:10]==(11,5) and v[11:13]==(5,6) and v[14:16]==(0,5)
  self.stride=int(Path('/sys/class/graphics/fb0/stride').read_text());self.m=mmap.mmap(self.fd,self.stride*v[3],flags=mmap.MAP_SHARED,prot=mmap.PROT_READ|mmap.PROT_WRITE)
  colors=np.array([[[255,0,0],[0,255,0],[0,0,255]]],np.uint8)
  assert cv2.cvtColor(colors,cv2.COLOR_RGB2BGR565).tobytes()==bytes([0,248,224,7,31,0])
  self.tty=os.open('/dev/tty0',os.O_RDWR);fcntl.ioctl(self.tty,0x4b3a,1);self.draw_count=0;self.digest=None;self.readback=False
 def present(self,rgb,textlines):
  v=self.v;canvas=np.zeros((v[1],v[0],3),np.uint8);canvas[:]=(25,20,16)
  h,w=rgb.shape[:2];scale=min((v[0]-30)/w,(v[1]-94)/h);nw=int(w*scale);nh=int(h*scale)
  image=cv2.resize(rgb,(nw,nh),interpolation=cv2.INTER_LINEAR);x=(v[0]-nw)//2;y=65+(v[1]-94-nh)//2;canvas[y:y+nh,x:x+nw]=image
  for i,s in enumerate(textlines):cv2.putText(canvas,s,(16,26+i*23),cv2.FONT_HERSHEY_SIMPLEX,.58,(230,240,235),1,cv2.LINE_AA)
  cv2.putText(canvas,'REAL CAMERA / preview only / raw preserved',(16,v[1]-13),cv2.FONT_HERSHEY_SIMPLEX,.5,(230,240,235),1,cv2.LINE_AA)
  packed=cv2.cvtColor(canvas,cv2.COLOR_RGB2BGR565)
  if self.stride==v[0]*2 and v[4]==0:raw=packed.tobytes()
  else:
   screen=np.zeros((v[1],self.stride),np.uint8);screen[:,v[4]*2:(v[4]+v[0])*2]=packed.reshape(v[1],v[0]*2);raw=screen.tobytes()
  start=v[5]*self.stride;self.m[start:start+len(raw)]=raw
  # Readback of uncached scanout memory is sampled, not performed for every frame.
  if self.draw_count<2 or self.draw_count%240==0:
   self.digest=hashlib.sha256(raw).hexdigest();self.readback=self.m[start:start+len(raw)]==raw;self.last_check=self.draw_count
  self.draw_count+=1
  return canvas,self.digest,self.readback
 def close(self):
  self.m.close();os.close(self.fd);fcntl.ioctl(self.tty,0x4b3a,0);os.close(self.tty)

def main(frame_sink=None):
 ap=argparse.ArgumentParser();ap.add_argument('--output',required=True);ap.add_argument('--fps',type=float,default=4);ap.add_argument('--duration',type=float,default=0);args=ap.parse_args()
 assert Path('/proc/sys/kernel/random/boot_id').read_text().strip()==BOOT
 assert 0<args.fps<=10
 out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
 lock=Path('/home/root/ps_board/camera_live.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 stop=threading.Event();q=queue.Queue(maxsize=1);signal.signal(signal.SIGTERM,lambda *_:stop.set());signal.signal(signal.SIGINT,lambda *_:stop.set())
 stats=dict(state='STARTING',boot_id=BOOT,pid=os.getpid(),camera_ip=CAMERA,host_ip=HOST,complete_frames=0,incomplete_frames=0,packets=0,invalid_packets=0,displayed_frames=0,display_skipped=0,first_frame_saved=False,requested_fps=args.fps,production_algorithm_connected=False)
 c=None;rx=None;display=None;address_added=False;owned=False;changed=[];initial={};worker=None;heartbeat=None;started=False
 write_json(out/'state.json',stats)
 try:
  cv2.setNumThreads(1)
  ipv4=subprocess.check_output(['/sbin/ip','-4','addr','show','dev','eth0'],universal_newlines=True);assert 'inet ' not in ipv4,ipv4
  assert Path('/sys/class/net/eth0/carrier').read_text().strip()=='1'
  address_probe();subprocess.run(['/sbin/ip','addr','add',HOST+'/24','dev','eth0'],check=True);address_added=True
  c=Control();assert c.read(REG['ccp'])==0,'Camera already controlled'
  initial={k:c.read(a) for k,a in REG.items() if k not in ['stop']};write_json(out/'camera_before.json',initial)
  assert initial['test_pattern']==0 and initial['compression']==0 and initial['chunk']==0
  assert initial['trigger_mode']==0 and initial['stream_config']==0
  c.write(REG['ccp'],2);owned=True;assert c.read(REG['ccp'])==2
  c.write(REG['stop'],0)
  def change(k,v):
   if initial[k]!=v:
    changed.append(k);c.write(REG[k],v);assert c.read(REG[k])==v,(k,v,c.read(REG[k]))
  change('heartbeat',10000)
  # Preview-only 4x4 binning is supported by the camera inquiry 0xd000d000.
  changed.extend(['width','height'])
  change('binning',0x00040004)
  change('exposure_us',20000)
  change('pixel_format',0x01100011)
  change('super_bayer',0)
  change('acquisition_mode',2)
  change('frame_rate',(1<<30)|round(args.fps*initial['fps_unit']))
  # 10 us between packets limits burst pressure without changing global network settings.
  change('packet_delay',1000)
  change('packet_size',(initial['packet_size']&0x7fff0000)|1500)
  rx=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
  rx.setsockopt(socket.SOL_SOCKET,33,32*1024*1024) # SO_RCVBUFFORCE, board root
  rx.bind((HOST,0));rx.settimeout(.2)
  change('destination',struct.unpack('!I',socket.inet_aton(HOST))[0]);change('port',(initial['port']&0xffff0000)|rx.getsockname()[1])
  active={k:c.read(REG[k]) for k in initial};write_json(out/'camera_active.json',active)
  width=active['width'];height=active['height'];payload=active['payload'];assert payload==width*height*2 and width%4==0 and height%4==0
  stats.update(preview_binning=[4,4],full_resolution_first_frame='../live02/first_frame.raw',width=width,height=height,pixel_format='BayerRG12',payload_bytes=payload,exposure_us=active['exposure_us'],receive_buffer_bytes=rx.getsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF),camera_test_pattern_off=active['test_pattern']==0)
  display=Display()
  display.present(np.zeros((512,612,3),np.uint8),['PenumbraSight / LIVE CAMERA','Waiting for a complete real frame...'])
  def heartbeat_fn():
   while not stop.wait(.5):
    try:assert c.read(REG['ccp'])==2
    except Exception as e:stats['heartbeat_error']=repr(e);stop.set()
  def render_fn():
   last_preview=0
   try:
    while not stop.is_set() or not q.empty():
     try:raw,meta=q.get(timeout=.2)
     except queue.Empty:continue
     dtype='>u2' if active['packet_size']&(1<<29) else '<u2';a=np.frombuffer(raw,dtype=dtype).reshape(height,width)
     # Bayer RGGB 2x2 cells sampled every four sensor pixels preserve the field of view.
     red=a[0::2,0::2];green=(a[0::2,1::2].astype(np.uint32)+a[1::2,0::2])*.5;blue=a[1::2,1::2]
     rgb16=np.stack((red,green,blue),axis=2)
     lo,hi=np.percentile(rgb16,[.5,99.5]);span=max(hi-lo,64.)
     rgb=np.clip((rgb16-lo)*255/span,0,255).astype(np.uint8)
     count=meta['number'];lines=['PenumbraSight / LIVE CAMERA / MV-CS050-10GC','Frame %d | %dx%d RAW12 | %.1f fps input | display stretch'%(count,width,height,stats.get('receive_fps',0.))]
     canvas,digest,readback=display.present(rgb,lines)
     stats['displayed_frames']+=1;stats['last_display_frame']=count;stats['last_display_block_id']=meta['block_id'];stats['last_display_monotonic']=time.monotonic();stats['latest_receive_to_display_ms']=(time.monotonic()-meta['complete_monotonic'])*1000;stats['framebuffer_readback_passed']=readback;stats['framebuffer_sha256']=digest;stats['framebuffer_readback_display_index']=display.last_check
     if not stats['first_frame_saved']:
      p=out/'first_frame.raw';p.write_bytes(raw)
      info=dict(meta,width=width,height=height,pixel_format='BayerRG12',raw_byte_order=dtype,bytes=len(raw),sha256=hashlib.sha256(raw).hexdigest(),min=int(a.min()),max=int(a.max()),median=float(np.median(a)),low_four_bits_all_zero=bool(np.all((a&15)==0)),sensor_test_pattern_off=True,display_only_stretch=[float(lo),float(hi)],algorithm_reconstruction=False)
      assert info['max']<=4095,'RAW12 container mismatch'
      assert cv2.imwrite(str(out/'first_frame_preview.png'),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR))
      assert cv2.imwrite(str(out/'first_screen.png'),cv2.cvtColor(canvas,cv2.COLOR_RGB2BGR))
      write_json(out/'first_frame.json',info);stats['first_frame_saved']=True;stats['first_frame']=info
     now=time.monotonic()
     if now-last_preview>=3:
      cv2.imwrite(str(out/'latest_preview.jpg'),cv2.cvtColor(canvas,cv2.COLOR_RGB2BGR),[cv2.IMWRITE_JPEG_QUALITY,85]);last_preview=now
   except Exception:
    stats['render_error']=traceback.format_exc();stop.set()
  worker=threading.Thread(target=render_fn);worker.start();heartbeat=threading.Thread(target=heartbeat_fn);heartbeat.start()
  c.write(REG['start'],0x80000000);started=True;stats['state']='LIVE'
  t0=time.monotonic();last_state=t0;frame=None;received_times=[]
  while not stop.is_set():
   now=time.monotonic()
   if args.duration and now-t0>=args.duration:break
   if now-(received_times[-1] if received_times else t0)>8:raise RuntimeError('No complete real frame for eight seconds')
   if now-last_state>=1:
    if (out/'stop').exists():break
    stats['uptime_seconds']=now-t0;stats['receive_fps']=(len(received_times)-1)/(received_times[-1]-received_times[0]) if len(received_times)>1 else 0
    stats['last_state_monotonic']=now;write_json(out/'state.json',stats);last_state=now
   try:data,peer=rx.recvfrom(16384)
   except socket.timeout:continue
   if peer[0]!=CAMERA or len(data)<8:stats['invalid_packets']+=1;continue
   status,block,info=struct.unpack_from('!HHI',data);kind=info>>24;pid=info&0xffffff;body=data[8:];stats['packets']+=1
   if status!=0 or kind&0x80:stats['invalid_packets']+=1;continue
   if kind==1:
    if frame is not None:stats['incomplete_frames']+=1
    if len(body)<36:stats['invalid_packets']+=1;frame=None;continue
    flags,typ,ts,pf,w,h,x,y,px,py=struct.unpack_from('!HHQIIIIIHH',body)
    if (typ,pf,w,h,px,py)!=(1,0x01100011,width,height,0,0):raise RuntimeError('Unexpected GVSP image leader '+repr((typ,pf,w,h,px,py)))
    frame=dict(block=block,packets={},timestamp_ticks=ts,leader_monotonic=now,roi_sensor_offset=[x,y])
   elif kind==3 and frame is not None and frame['block']==block:
    if pid==0 or pid>100000 or (pid in frame['packets'] and frame['packets'][pid]!=body):raise RuntimeError('GVSP packet identity conflict')
    frame['packets'][pid]=body
   elif kind==2 and frame is not None and frame['block']==block:
    parts=frame['packets'];expected=pid-1
    if expected<=0 or len(parts)!=expected or set(parts)!=set(range(1,pid)):
     stats['incomplete_frames']+=1;stats['last_missing_packets']=expected-len(parts);frame=None;continue
    raw=b''.join(parts[i] for i in range(1,pid))
    if len(raw)!=payload:stats['incomplete_frames']+=1;frame=None;continue
    stats['complete_frames']+=1;received_times.append(time.monotonic());received_times=received_times[-80:]
    meta=dict(number=stats['complete_frames'],block_id=block,timestamp_ticks=frame['timestamp_ticks'],leader_monotonic=frame['leader_monotonic'],complete_monotonic=time.monotonic(),packet_count=expected,complete=True)
    if frame_sink is not None:
     context=dict(boot_id=BOOT,sensor_test_pattern_off=active['test_pattern']==0,
       format=dict(width=width,height=height,pixel_format='BayerRG12',dtype='>u2' if active['packet_size']&(1<<29) else '<u2',
         roi_sensor_offset=frame['roi_sensor_offset'],binning=[active['binning']&0xffff,active['binning']>>16]),
       capture=dict(exposure_us=active['exposure_us'],gain_raw=active['gain']))
     frame_sink(raw,meta,context)
     stats['production_frames_offered']=stats.get('production_frames_offered',0)+1
    if q.full():
     try:q.get_nowait();stats['display_skipped']+=1
     except queue.Empty:pass
    q.put_nowait((raw,meta));frame=None
  stats['state']='STOPPING'
 except Exception:
  stats['error']=traceback.format_exc();stats['state']='ERROR';print(stats['error'],flush=True)
 finally:
  stop.set()
  if heartbeat:heartbeat.join(timeout=3)
  cleanup=[]
  if c and owned:
   try:c.write(REG['stop'],0)
   except Exception as e:cleanup.append('stop: '+repr(e))
   for k in reversed(changed):
    try:
     c.write(REG[k],initial[k]);assert c.read(REG[k])==initial[k],k
    except Exception as e:cleanup.append(k+': '+repr(e))
   try:
    restored={k:c.read(REG[k]) for k in changed};write_json(out/'camera_restored.json',restored)
    for k in changed:assert restored[k]==initial[k],k
   except Exception as e:cleanup.append('final readback: '+repr(e))
   try:c.write(REG['ccp'],0);assert c.read(REG['ccp'])==0
   except Exception as e:cleanup.append('ccp: '+repr(e))
  if worker:worker.join(timeout=15)
  if worker and worker.is_alive():cleanup.append('render worker still active')
  if display:display.close()
  if rx:rx.close()
  if c:c.close()
  if address_added:
   try:subprocess.run(['/sbin/ip','addr','del',HOST+'/24','dev','eth0'],check=True)
   except Exception as e:cleanup.append('IPv4: '+repr(e))
  stats['cleanup_errors']=cleanup;stats['restored_camera_parameters']=not cleanup and owned;stats['owned_ipv4_removed']=address_added and not cleanup
  if stats['state']!='ERROR' and not stats.get('render_error') and not stats.get('heartbeat_error'):stats['state']='STOPPED'
  else:stats['state']='ERROR'
  write_json(out/'state.json',stats);os.sync()
  print(json.dumps(stats),flush=True)

if __name__=='__main__':main()
