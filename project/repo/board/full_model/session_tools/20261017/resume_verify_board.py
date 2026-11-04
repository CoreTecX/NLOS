"""Verify retained results and immutable deployment after the lunch power cycle."""
import hashlib,json,mmap,os,struct,subprocess
from pathlib import Path
S=Path('/home/root/ps_runs_20261017/steps5_12_01')
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()

active=[]
for p in Path('/proc').iterdir():
    if not p.name.isdigit():continue
    try:
        comm=(p/'comm').read_text().strip()
        if comm.startswith(('ps_full_','ps_board_app')):active.append(p.name)
    except OSError:pass
if active:raise RuntimeError('Algorithm active: '+repr(active))
Path('/var/lock').resolve().mkdir(parents=True,exist_ok=True)
r=dict(boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),packages=[],active=active)
for name in ('performance','fallback','dataset'):
    root=S/'deployment'/name
    manifest=root/('package_manifest.json' if name=='dataset' else 'release_manifest.json')
    obj=json.loads(manifest.read_text())['files']
    entries=obj if name=='dataset' else [dict(file=k,**v) for k,v in obj.items()]
    for row in entries:
        p=root/row['file']
        if p.stat().st_size!=row['bytes'] or sha(p)!=row['sha256']:raise RuntimeError('Retained file changed: '+str(p))
    r['packages'].append(dict(package=name,files=len(entries),passed=True,manifest_sha256=sha(manifest)))
cp=json.loads((S/'pause_checkpoint.json').read_text())
suite=json.loads((S/'results/dataset97_pl_01/target_suite.json').read_text())
done=[x['key'] for x in suite['cases'] if x['exit_code']==0]
if done!=cp['completed'] or len(done)!=52 or len(cp['remaining'])!=45:raise RuntimeError('Checkpoint mismatch')
r.update(completed=done,remaining=cp['remaining'],checkpoint_sha256=sha(S/'pause_checkpoint.json'))
r['configurations']=[]
for row in json.loads((S/'session.json').read_text())['backups']:
    same=sha(Path(row['source']))==row['sha256']==sha(Path(row['backup']))
    r['configurations'].append(dict(path=row['source'],unchanged=same))
if not all(x['unchanged'] for x in r['configurations']):raise RuntimeError('Boot configuration changed')
fd=os.open('/dev/mem',os.O_RDWR|os.O_SYNC)
try:
    with mmap.mmap(fd,4096,offset=0xff0a0000) as m:
        v=struct.unpack_from('<I',m,0x54)[0];struct.pack_into('<I',m,0x54,v&~0x80000000)
        r['pl_reset_held']=not bool(struct.unpack_from('<I',m,0x54)[0]&0x80000000)
finally:os.close(fd)
r['temperatures']={}
for p in Path('/sys/bus/iio/devices').glob('iio:device*/in_temp*_raw'):
    stem=str(p)[:-4]
    r['temperatures'][p.name]=(float(p.read_text())+float(Path(stem+'_offset').read_text()))*float(Path(stem+'_scale').read_text())/1000
env=os.environ.copy();env['PATH']='/usr/sbin:/sbin:/usr/bin:/bin'
r['dmesg']=subprocess.check_output(['dmesg'],env=env).decode(errors='replace')
r['disk']=subprocess.check_output(['df','-h'],env=env).decode(errors='replace')
r['passed']=True
with (S/'resume_verified.json').open('x') as f:json.dump(r,f,indent=2)
print(json.dumps(r),flush=True)
