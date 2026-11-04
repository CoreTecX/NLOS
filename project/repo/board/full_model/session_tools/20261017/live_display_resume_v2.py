"""Show each newly completed board result; independent audit status stays explicit."""
from pathlib import Path
import base64,fcntl,hashlib,json,math,mmap,os,struct,time,zlib
S=Path('/home/root/ps_runs_20261017/steps5_12_01');D=S/'display';R=S/'results/dataset45_pl_resume_01'; OLD=S/'results/dataset97_pl_01'; BOOT=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
assets=json.loads(zlib.decompress((D/'live_assets.json.z').read_bytes()).decode())
rows={r['key']:r for r in json.loads((S/'deployment/dataset/dataset_manifest.json').read_text())['cases']}
lock=(D/'viewer.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
fd=os.open('/dev/fb0',os.O_RDWR);info=bytearray(160);fcntl.ioctl(fd,0x4600,info,True);v=struct.unpack('<40I',info)
native_stride=int(Path('/sys/class/graphics/fb0/stride').read_text());stride=2048
if v[0]<1024 or v[1]<768 or v[6]!=16 or native_stride<v[0]*2 or tuple(v[8:10])!=(11,5) or tuple(v[11:13])!=(5,6) or tuple(v[14:16])!=(0,5):raise RuntimeError('Unexpected framebuffer')
mapping=mmap.mmap(fd,native_stride*v[3],flags=mmap.MAP_SHARED,prot=mmap.PROT_READ|mmap.PROT_WRITE)
start=v[5]*native_stride;canvas_x=(v[0]-1024)//2+v[4];canvas_y=(v[1]-768)//2
template=bytearray(zlib.decompress((D/'screen_D11_rgb.rgb565.z').read_bytes()))
def color(r,g,b):return struct.pack('<H',((r>>3)<<11)|((g>>2)<<5)|(b>>3))
background=color(16,26,43)
def patch(frame,p):
    raw=zlib.decompress(base64.b64decode(p['data']));w=p['width']*2
    for y in range(p['height']):
        off=(p['y']+y)*stride+p['x']*2;frame[off:off+w]=raw[y*w:(y+1)*w]
for p in assets['patches']:patch(template,p)
def text(frame,x,y,value,rgb=(225,237,250)):
    c=color(*rgb)
    for character in str(value):
        for index in assets['atlas'].get(character,[]):
            yy=y+index//14;xx=x+index%14
            if xx<1024 and yy<768:
                offset=yy*stride+xx*2;frame[offset:offset+2]=c
        x+=14
def show(key,number,total,accepted):
    source=OLD if key in completed_before else R
    p=source/(key+'.pfo');raw=p.read_bytes();meta=json.loads((source/(key+'.json')).read_text())
    if len(raw)!=75232 or raw[:8]!=b'PSFULL1\0' or struct.unpack_from('<IIII',raw,8)!=(1,4,29,36) or zlib.crc32(raw[64:])!=struct.unpack_from('<I',raw,32)[0] or raw[40:56].hex()!=rows[key]['model_id']:raise RuntimeError('Invalid display PFO')
    if meta['converged'] is not True or not(meta['primal']<meta['primal_limit'] and meta['dual']<meta['dual_limit']):raise RuntimeError('Result did not satisfy original residual checks')
    digest=hashlib.sha256(raw).hexdigest();audit=accepted.get(key)==digest
    values=struct.unpack_from('<3132d',raw,64+2*3132*8)
    if not all(math.isfinite(x) and x>=0 for x in values):raise RuntimeError('Invalid physical RGB')
    maximum=max(values);mapped=[]
    for value in values:
        x=min(1.,max(0.,value/maximum if maximum>0 else 0.));x=12.92*x if x<=.0031308 else 1.055*x**(1/2.4)-.055;mapped.append(round(x*255))
    frame=bytearray(template)
    if audit:
        for y in range(62,94):off=y*stride+26*2;frame[off:off+980*2]=color(28,44,67)*980
        text(frame,30,64,'Independent numerical + state audit: PASSED',(128,223,200))
        for y in range(694,730):off=y*stride+26*2;frame[off:off+980*2]=background*980
        text(frame,30,699,'%d / 97 reconstructions independently verified.' % len(accepted),(225,237,250))
    for y in range(29):
        line=b''.join(color(*mapped[(y*36+x)*3:(y*36+x)*3+3])*18 for x in range(36))
        for rep in range(18):off=(131+y*18+rep)*stride+28*2;frame[off:off+1296]=line
    for y in range(120,665):off=y*stride+698*2;frame[off:off+326*2]=background*326
    for y,value,rgb in [(131,'LATEST RESULT',(150,174,205)),(170,key[:22],(255,255,255)),(200,key[22:44],(255,255,255)),(252,'FPGA / OVERLAP',(225,237,250)),(291,'150 MHz / FP64',(160,185,211)),(345,'%.3f s'%meta['seconds_solver'],(128,223,200)),(389,str(meta['iterations'])+' iterations',(225,237,250)),(445,'Generated: %d/%d'%(number,total),(225,237,250)),(492,'AUDIT: PASSED' if audit else 'AUDIT: PENDING',(128,223,200) if audit else (242,200,125)),(548,'36 x 29 RGB',(160,185,211)),(588,'sRGB / NEAREST',(160,185,211))]:text(frame,708,y,value,rgb)
    screen=bytearray(background*(native_stride*v[1]//2))
    for y in range(768):
        off=(canvas_y+y)*native_stride+canvas_x*2;screen[off:off+2048]=frame[y*2048:(y+1)*2048]
    mapping[start:start+len(screen)]=screen;readback=mapping[start:start+len(screen)]
    if readback!=screen:raise RuntimeError('Live framebuffer readback mismatch')
    record=dict(width=v[0],height=v[1],stride=native_stride,offset=start,boot_id=BOOT,case=key,number=number,total=total,source=str(p),source_sha256=digest,independent_acceptance=audit,seconds_solver=meta['seconds_solver'],iterations=meta['iterations'],framebuffer_sha256=hashlib.sha256(readback).hexdigest(),uptime=Path('/proc/uptime').read_text().strip())
    temp=D/'live_state.tmp';temp.write_text(json.dumps(record,indent=2));temp.replace(D/'live_state.json')
    with (D/'live_history.jsonl').open('a') as f:f.write(json.dumps(record)+'\n')
    print('DISPLAY_CASE',key,number,total,'ACCEPTED' if audit else 'AUDIT_PENDING',flush=True)
prior=json.loads((OLD/'target_suite.json').read_text())['cases']
completed_before={r['key'] for r in prior}
if len(prior)!=52:raise RuntimeError('Unexpected pause count')
last=None;last_key=None;was_accepted=False
try:
    while not (D/'resume.stop').exists():
        try:
            fresh=json.loads((R/'target_suite.json').read_text())['cases'] if (R/'target_suite.json').exists() else []
            done=prior+fresh
            if len({r['key'] for r in done})!=len(done):raise RuntimeError('Duplicate continuation case')
            accepted=json.loads((D/'resume_accepted_cases.json').read_text()) if (D/'resume_accepted_cases.json').exists() else {}
        except (OSError,ValueError):time.sleep(1);continue
        if last is None:last=max(0,len(done)-1)
        for i in range(last,len(done)):
            if done[i]['exit_code']!=0:raise RuntimeError('Batch case failed; preserve previous display')
            last_key=done[i]['key'];show(last_key,i+1,97,accepted);last=i+1;was_accepted=last_key in accepted
        if last_key in accepted and not was_accepted:show(last_key,last,97,accepted);was_accepted=True
        if len(done)==97 and len(accepted)==97:break
        time.sleep(1)
finally:
    mapping.close();os.close(fd);lock.close()
