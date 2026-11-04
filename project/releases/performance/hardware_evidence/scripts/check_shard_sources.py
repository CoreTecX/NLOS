"""Verify every parallel simulator consumed identical compiled design and input vectors."""
from pathlib import Path
import hashlib,json
root=Path(__file__).resolve().parents[1];master=root/'build/rtl_lite'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    inputs={p.relative_to(master).as_posix():sha(p) for p in master.glob('*.hex')}
    for p in (master/'xsim.dir').rglob('*'):
        if p.is_file() and p.suffix in ('.exe','.mem','.reloc','.rtti','.svtype','.type','.version'):
            inputs[p.relative_to(master).as_posix()]=sha(p)
    inputs['run.tcl']=sha(master/'run.tcl');assert len(inputs)>20
    for case in (-1,10,11,12,13,14):
        wd=root/'build'/('rtl_shard_'+str(case))
        for rel,expected in inputs.items():assert sha(wd/rel)==expected,(case,rel)
        assert 'CASE='+str(case) in (wd/'options.txt').read_text()
    (root/'reports/rtl_shard_source_audit.json').write_text(json.dumps(dict(passed=True,shards=6,files_per_shard=len(inputs),files=inputs),indent=2))
    print('RTL_SHARD_SOURCE_AUDIT_PASS shards=6 files_per_shard='+str(len(inputs)))
if __name__=='__main__':main()
