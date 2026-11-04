"""Retrieve a locked board snapshot and verify every byte before extracting locally."""
from pathlib import Path
import argparse,hashlib,json,re,subprocess,tarfile,time
from jtag_transfer import serial,REMOTE,ROOT,ENV,XSDB,BOOT

def main():
    p=argparse.ArgumentParser();p.add_argument('--name',required=True);p.add_argument('--path',action='append',required=True);p.add_argument('--reuse-export',action='store_true',help='Attach to the same active export before any archive bytes were downloaded');a=p.parse_args()
    if not a.name.replace('_','').isalnum():raise ValueError('Invalid export name')
    local=ROOT/'downloads'/a.name;local.mkdir(parents=True,exist_ok=a.reuse_export)
    if (local/'results.tar.gz').exists():raise RuntimeError('Existing archive must be inspected; it will not be overwritten')
    spec=dict(name=a.name,paths=a.path)
    base="import json,os,subprocess,time\nfrom pathlib import Path\ns=Path('"+REMOTE+"')\ne=s/'exports'/"+repr(a.name)+"\n"
    code=base+"(s/'exports').mkdir(exist_ok=True)\np=s/'exports'/"+repr(a.name+'.spec.json')+"\nwith p.open('x') as f:json.dump("+repr(spec)+",f)\nwith (s/'exports'/"+repr(a.name+'.log')+").open('x') as log:r=subprocess.Popen(['/usr/bin/python3.7','-B',str(s/'tools/board_export.py'),str(p)],stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)\nprint('PS_TRANSFER_JSON='+json.dumps(dict(pid=r.pid)),flush=True)\n"
    if a.reuse_export:
        code=base+"spec=json.loads((s/'exports'/"+repr(a.name+'.spec.json')+").read_text())\nif spec!="+repr(spec)+":raise RuntimeError('Existing export specification differs')\nif (e/'released').exists():raise RuntimeError('Export lease already released')\nprint('PS_TRANSFER_JSON='+json.dumps(dict(reusing=True)),flush=True)\n"
    serial(code,'export_reuse_'+a.name if a.reuse_export else 'export_start_'+a.name)
    deadline=time.monotonic()+180
    while True:
        code=base+"r={}\nfor n in ['ready.json','failed.json']:\n if (e/n).exists():r[n]=json.loads((e/n).read_text())\nprint('PS_TRANSFER_JSON='+json.dumps(r),flush=True)\n"
        state=serial(code,'export_poll_'+a.name)
        if 'failed.json' in state:raise RuntimeError(state['failed.json'])
        if 'ready.json' in state:manifest=state['ready.json'];break
        if time.monotonic()>deadline:raise RuntimeError('Export preparation timeout')
        time.sleep(1)
    if manifest['boot_id']!=BOOT:raise RuntimeError('Board rebooted')
    (local/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf8')
    archive=local/'results.tar.gz';offset=0;h=hashlib.sha256();limit=8*1024*1024
    with archive.open('xb') as out:
        while offset<manifest['bytes']:
            n=min(limit,manifest['bytes']-offset);words=((n+63)//64)*16
            code=base+"import mmap,hashlib\nlease=json.loads((e/'ready.json').read_text());os.kill(lease['pid'],0)\nif (e/'released').exists() or Path('/proc/sys/kernel/random/boot_id').read_text().strip()!='"+BOOT+"':raise RuntimeError('Export lease ended or boot changed')\nwith Path(lease['path']).open('rb') as f:\n f.seek("+str(offset)+");data=f.read("+str(n)+")\nif len(data)!="+str(n)+":raise RuntimeError('Short archive read')\nfd=os.open('/dev/mem',os.O_RDWR|os.O_SYNC)\ntry:\n with mmap.mmap(fd,"+str(words*4)+",flags=mmap.MAP_SHARED,prot=mmap.PROT_READ|mmap.PROT_WRITE,offset=0x50000000) as m:\n  m[:]=data+b'\\0'*((-len(data))%64);check=m[:]\nfinally:os.close(fd)\nif check[:len(data)]!=data:raise RuntimeError('DDR write/readback mismatch')\nprint('PS_TRANSFER_JSON='+json.dumps(dict(sha256=hashlib.sha256(data).hexdigest(),bytes=len(data))),flush=True)\n"
            chunk=serial(code,'export_chunk_'+a.name+'_'+str(offset))
            binary=local/'chunk.bin';tcl=local/'read_chunk.tcl'
            tcl.write_text('connect -url tcp:127.0.0.1:33121\ntargets -set -filter {name == "PSU"}\nmrd -bin -file {'+binary.as_posix()+'} -size w 0x50000000 '+str(words)+'\nputs "PS_READ_COMPLETE"\ndisconnect\nexit\n',encoding='utf8')
            run=subprocess.run([XSDB,str(tcl)],cwd=str(ROOT),env=ENV,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,encoding='utf8',errors='replace',timeout=150)
            (local/('read_'+str(offset)+'.log')).write_text(run.stdout,encoding='utf8')
            if run.returncode or 'PS_READ_COMPLETE' not in run.stdout:raise RuntimeError('JTAG read failed')
            data=binary.read_bytes()
            if len(data)!=words*4:raise RuntimeError('JTAG read length mismatch')
            data=data[:n]
            if hashlib.sha256(data).hexdigest()!=chunk['sha256']:raise RuntimeError('JTAG read hash mismatch')
            out.write(data);h.update(data);offset+=n
            print('DOWNLOAD_PROGRESS',a.name,offset,manifest['bytes'],flush=True)
    if h.hexdigest()!=manifest['sha256']:raise RuntimeError('Whole download hash mismatch')
    serial(base+"(e/'release').write_text('host verified whole archive')\nprint('PS_TRANSFER_JSON='+json.dumps(dict(release_requested=True)),flush=True)\n",'export_release_'+a.name)
    dest=local/'content';dest.mkdir()
    with tarfile.open(str(archive),'r:gz') as tar:
        members=tar.getmembers()
        if any(not m.isfile() or dest.resolve() not in (dest/m.name).resolve().parents for m in members):raise RuntimeError('Invalid export archive paths')
        tar.extractall(str(dest),members=members)
    for row in manifest['files']:
        path=dest/row['path']
        if path.stat().st_size!=row['bytes'] or hashlib.sha256(path.read_bytes()).hexdigest()!=row['sha256']:raise RuntimeError('Extracted result hash mismatch')
    (local/'verified.json').write_text(json.dumps(dict(passed=True,files=len(manifest['files']),archive_sha256=h.hexdigest(),boot_id=BOOT),indent=2),encoding='utf8')
    print('DOWNLOAD_VERIFIED',str(dest),flush=True)

if __name__=='__main__':main()
