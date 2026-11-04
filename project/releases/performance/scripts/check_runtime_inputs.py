"""Compare input rejection semantics against a separately built baseline.

Both stages must be ASCII directories produced by full_model.py verify. This
does not access a board. Invalid large inputs are kept in an isolated temp dir.
"""
from pathlib import Path
import argparse, hashlib, json, struct, subprocess, tempfile, zlib
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--stage',required=True,type=Path)
parser.add_argument('--baseline-stage',required=True,type=Path)
parser.add_argument('--output',required=True,type=Path)
parser.add_argument('--work-root',default='D:/Temp',type=Path)
args=parser.parse_args()
stage=args.stage.resolve();BASE=args.baseline_stage.resolve()
dest=args.output.resolve();dest.mkdir(parents=True,exist_ok=False)
args.work_root.mkdir(parents=True,exist_ok=True)
work=Path(tempfile.mkdtemp(prefix='ps_runtime_negative_',dir=args.work_root)).resolve()
if not str(work).isascii():raise ValueError('Use an ASCII --work-root')
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for block in iter(lambda:f.read(1048576),b''):h.update(block)
    return h.hexdigest()
def write(p,value):p.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def run(args,cwd,log,allowed=(0,)):
    r=subprocess.run([str(x) for x in args],cwd=cwd,capture_output=True,encoding='utf-8',errors='replace',timeout=1800)
    log.write_text(r.stdout+r.stderr,encoding='utf-8')
    if r.returncode not in allowed:raise RuntimeError(str(args)+'\n'+r.stdout+r.stderr)
    return r
model=BASE/'data/D11_rgb.pfm';measurement=BASE/'data/D11_rgb.pfy'
mb=model.read_bytes();yb=measurement.read_bytes();records=[]
def crc(data):struct.pack_into('<I',data,32,zlib.crc32(memoryview(data)[64:]))
def execute(name,kind,mutate,valid_crc=False):
    data=bytearray(mb if kind=='model' else yb);mutate(data)
    if valid_crc:crc(data)
    target=work/('input.pfm' if kind=='model' else 'input.pfy');target.write_bytes(data)
    sources=[target if kind=='model' else model,target if kind!='model' else measurement]
    cases=[]
    for tag,exe in [('baseline',BASE/'bin/ps_full_host.exe'),('candidate',stage/'bin/ps_full_host.exe')]:
        prefix=dest/(name+'_'+tag)
        result=run([exe,*sources,prefix,'cpu','1'],work,prefix.with_suffix('.log'),allowed=(2,))
        assert not prefix.with_suffix('.pfo').exists() and not prefix.with_suffix('.json').exists(),name
        cases.append(result.stderr.strip())
    assert cases[0]==cases[1],(name,cases)
    records.append(dict(case=name,kind=kind,input_sha256=sha(target),crc_repaired=valid_crc,error=cases[0],same_rejection=True))
def put(off,fmt,value):return lambda b:struct.pack_into(fmt,b,off,value)
for name,kind,func,fix in [
    ('magic','measurement',lambda b:b.__setitem__(0,0),False),
    ('version','measurement',put(8,'<I',2),False),
    ('kind','measurement',put(12,'<I',1),False),
    ('dimensions','measurement',put(16,'<I',125),False),
    ('reserved','measurement',put(36,'<I',1),False),
    ('payload_length','measurement',put(24,'<Q',8),False),
    ('identity','measurement',lambda b:b.__setitem__(40,b[40]^1),False),
    ('truncated','measurement',lambda b:b.pop(),False),
    ('trailing','measurement',lambda b:b.append(0),False),
    ('crc','measurement',lambda b:b.__setitem__(150,b[150]^1),False),
    ('negative_scale','measurement',put(64,'<d',-1.),True),
    ('zero_scale','measurement',put(64,'<d',0.),True),
    ('nan_scale','measurement',put(64,'<d',float('nan')),True),
    ('negative_raw','measurement',put(88,'<d',-1.),True),
    ('inf_raw','measurement',put(88,'<d',float('inf')),True),
    ('nan_raw','measurement',put(88,'<d',float('nan')),True),
    ('overflow_raw','measurement',put(88,'<d',1e308),True),
    ('empty_raw','measurement',lambda b:b.__setitem__(slice(88,None),b'\0'*(len(b)-88)),True),
    ('uniform_raw','measurement',lambda b:b.__setitem__(slice(88,None),struct.pack('<d',1.)*((len(b)-88)//8)),True),
    ('model_crc','model',lambda b:b.__setitem__(15000,b[15000]^1),False),
    ('model_norm','model',put(64,'<d',0.),True),
    ('model_scale','model',put(72,'<d',0.),True),
    ('model_k_nan','model',put(64+8+1044*8+1044*15876*2,'<d',float('nan')),True),
    ('model_transpose','model',put(64+8+1044*8+1044*15876*2,'<d',1234567.),True),
    ('discarded_F_nan','model',put(len(mb)-8,'<d',float('nan')),True),
]:execute(name,kind,func,fix)
write(dest/'checks.json',dict(passed=True,cases=records,count=len(records),identity_crc_finite_diagnostics_preserved=True))
print('NEGATIVE_INPUTS_PASS',len(records),flush=True)
