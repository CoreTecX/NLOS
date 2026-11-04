"""Checked, resumable JTAG transfer through an exclusively leased no-map DDR buffer."""
from pathlib import Path
import hashlib,json,os,re,subprocess,time
ROOT=Path(__file__).resolve().parent
SERIAL=ROOT/'serial_command.ps1'
XSDB=r'D:\vivado\2025.2\Vivado\bin\xsdb.bat'
REMOTE='/home/root/ps_runs_20261017/steps5_12_01'
ENV={k.upper():v for k,v in os.environ.items()}
BOOT='de4ce291-1757-4556-aa85-e201e3d06f51'

def serial(code,name,timeout=40):
    script=ROOT/(name+'.py');script.write_text(code,encoding='utf8')
    result=subprocess.run(['powershell','-NoProfile','-ExecutionPolicy','Bypass','-File',str(SERIAL),'-Script',str(script),'-Log',str(ROOT/(name+'.serial.log')),'-TimeoutSeconds',str(timeout)],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,encoding='utf8',errors='replace',env=ENV,timeout=timeout+12)
    if result.returncode:raise RuntimeError('Serial operation failed: '+name+' '+result.stdout[-1200:])
    match=re.findall(r'PS_TRANSFER_JSON=(\{[^\r\n]*\})',result.stdout)
    if not match:raise RuntimeError('Missing serial response: '+name)
    return json.loads(match[-1])

def main():
    manifest=json.loads((ROOT/'transfer_manifest.json').read_text('utf8'));archive=Path(manifest['archive']);size=manifest['bytes'];digest=manifest['sha256']
    common="import os,json,hashlib,mmap\nfrom pathlib import Path\ns=Path('"+REMOTE+"')\nif Path('/proc/sys/kernel/random/boot_id').read_text().strip()!='"+BOOT+"':raise RuntimeError('Board boot changed')\nlease=json.loads((s/'transfer_lease.json').read_text());os.kill(lease['pid'],0)\nif (s/'transfer_lease_released').exists():raise RuntimeError('Transfer lease expired')\n"
    status=serial(common+"p=s/'transfer_state.json'\nr=json.loads(p.read_text()) if p.exists() else dict(next_offset=0,archive_sha256='"+digest+"')\nprint('PS_TRANSFER_JSON='+json.dumps(r),flush=True)\n",'transfer_status')
    if status['archive_sha256']!=digest:raise RuntimeError('Resume belongs to another archive')
    offset=status['next_offset'];start=time.monotonic();initial=offset;records=[];chunk_limit=32*1024*1024
    with archive.open('rb') as source:
        source.seek(offset)
        while offset<size:
            data=source.read(min(chunk_limit,size-offset));chunk_hash=hashlib.sha256(data).hexdigest();padded=data+b'\0'*((-len(data))%64)
            binary=ROOT/'transfer_chunk.bin';binary.write_bytes(padded)
            tcl=ROOT/'transfer_chunk.tcl';tcl.write_text('connect -url tcp:127.0.0.1:33121\ntargets -set -filter {name == "PSU"}\nset start [clock milliseconds]\nmwr -bypass-cache-sync -bin -file {'+binary.as_posix()+'} 0x50000000 '+str(len(padded)//4)+'\nputs "TRANSFER_BARRIER=[mrd -value 0x50000000]"\nputs "TRANSFER_WRITE_MS=[expr {[clock milliseconds]-$start}]"\ndisconnect\nexit\n',encoding='utf8')
            tic=time.monotonic();run=subprocess.run([XSDB,str(tcl)],cwd=str(ROOT),env=ENV,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,encoding='utf8',errors='replace',timeout=150)
            (ROOT/('jtag_chunk_'+str(offset)+'.log')).write_text(run.stdout,encoding='utf8')
            barrier=re.search(r'TRANSFER_BARRIER=(\d+)',run.stdout)
            if run.returncode or 'TRANSFER_WRITE_MS=' not in run.stdout or not barrier or int(barrier[1])!=int.from_bytes(data[:4],'little'):raise RuntimeError('JTAG chunk write failed at '+str(offset))
            code=common+"""fd=os.open('/dev/mem',os.O_RDONLY|os.O_SYNC)
try:
 with mmap.mmap(fd,"""+str(len(padded))+""",flags=mmap.MAP_SHARED,prot=mmap.PROT_READ,offset=0x50000000) as m:aligned=m[:]
 data=aligned[:"""+str(len(data))+"""]
finally:os.close(fd)
digest=hashlib.sha256(data).hexdigest()
if digest!='"""+chunk_hash+"""':raise RuntimeError('Chunk hash mismatch')
part=s/'incoming.tar.gz.part'
offset="""+str(offset)+"""
with part.open('xb' if offset==0 else 'r+b') as f:
 f.seek(0,2)
 if f.tell()!=offset:raise RuntimeError('Resume file length mismatch')
 f.write(data);f.flush();os.fsync(f.fileno())
r=dict(next_offset=offset+len(data),archive_sha256='"""+digest+"""',chunk_sha256=digest)
(s/'transfer_state.json').write_text(json.dumps(r))
print('PS_TRANSFER_JSON='+json.dumps(r),flush=True)
"""
            response=serial(code,'receive_chunk_'+str(offset))
            if response['next_offset']!=offset+len(data) or response['chunk_sha256']!=chunk_hash:raise RuntimeError('Invalid board transfer acknowledgement')
            offset=response['next_offset'];record=dict(next_offset=offset,total=size,percent=round(offset*100/size,2),chunk_bytes=len(data),chunk_sha256=chunk_hash,seconds=time.monotonic()-tic)
            records.append(record);(ROOT/'transfer_progress.json').write_text(json.dumps(dict(**record,elapsed_seconds=time.monotonic()-start,bytes_this_run=offset-initial,records=records),indent=2),encoding='utf8')
            print('TRANSFER_PROGRESS',json.dumps(record),flush=True)
    code=common+"""part=s/'incoming.tar.gz.part';h=hashlib.sha256()
with part.open('rb') as f:
 for block in iter(lambda:f.read(1048576),b''):h.update(block)
r=dict(sha256=h.hexdigest(),bytes=part.stat().st_size,archive_sha256='"""+digest+"""')
if r['sha256']!=r['archive_sha256'] or r['bytes']!="""+str(size)+""":raise RuntimeError('Whole archive verification failed')
final=s/'incoming.tar.gz'
if final.exists():raise RuntimeError('Final archive already exists')
part.rename(final);r['passed']=True;r['path']=str(final)
(s/'transfer_complete.json').write_text(json.dumps(r))
print('PS_TRANSFER_JSON='+json.dumps(r),flush=True)
"""
    final=serial(code,'transfer_finalize',60);(ROOT/'transfer_complete.json').write_text(json.dumps(final,indent=2),encoding='utf8');print('TRANSFER_COMPLETE',json.dumps(final),flush=True)

if __name__=='__main__':main()
