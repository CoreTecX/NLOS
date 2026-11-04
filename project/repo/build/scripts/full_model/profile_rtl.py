"""Observe paired exported RTL without changing the HLS DUT (n=64, continuous/stalled)."""
from pathlib import Path
import argparse,hashlib,json,shutil,subprocess,re
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--release',type=Path,required=True,help='Frozen M8 release with build/rtl_lite')
p.add_argument('--output',type=Path,required=True,help='New ASCII working directory')
p.add_argument('--tools',type=Path,default=Path('D:/vivado/2025.2/Vivado/bin'))
a=p.parse_args();root=a.release.resolve();out=a.output.resolve()
assert str(out).isascii() and out!=root and root not in out.parents
out.mkdir(parents=True,exist_ok=False)
paired=json.loads((root/'reports/rtl_lite_sources.json').read_text('utf-8'))
src=root/'build/rtl_lite'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
for rel,digest in paired.items():
    f=src/rel;assert sha(f)==digest,rel
    d=out/rel;d.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(f,d)
for f in src.glob('*.hex'):shutil.copy2(f,out/f.name)
for name in ['rtl_lite.prj','run.tcl']:shutil.copy2(src/name,out/name)
repo=Path(__file__).resolve().parents[3]
shutil.copy2(repo/'sim/full_model/profile_rtl_tb.sv',out/'rtl_lite_tb.sv')
def run(args,name):
    with (out/(name+'.console.txt')).open('w') as f:
        p=subprocess.run([str(x) for x in args],cwd=out,stdout=f,stderr=subprocess.STDOUT,timeout=900)
    assert p.returncode==0,name
run([a.tools/'xelab.bat','xil_defaultlib.rtl_lite_tb','xil_defaultlib.glbl','-prj','rtl_lite.prj','-L','xil_defaultlib','-L','unisims_ver','-L','xpm','-L','floating_point_v7_1_21','-L','floating_point_v7_0_26','-relax','-s','p1_profile','-log','elab.log'],'elab')
records=[]
for stalled in [False,True]:
    name='stalled' if stalled else 'continuous'
    (out/'options.txt').write_text('p1_profile\n-tclbatch run.tcl\n--noieeewarnings\n-testplusarg CASE=9\n'+('' if stalled else '-testplusarg P1_CONTINUOUS\n')+'-log '+name+'.log\n',encoding='ascii')
    run([a.tools/'xsim.bat','-f','options.txt'],name)
    text=(out/(name+'.log')).read_text(errors='replace')
    assert 'RTL_SHARD_PASS case=9' in text and 'Fatal:' not in text
    counters=dict((k,int(v)) for k,v in re.findall(r'(\w+)=(-?\d+)',re.search(r'P1_RTL_COUNTERS.*',text).group()))
    assert counters['mat_beats']==2048 and counters['vec_reads']==192 and counters['vec_writes']==192
    assert counters['total']==sum(counters[k] for k in ['load','compute','reduction_write','control'])
    records.append(dict(n=64,stalled=stalled,counters=counters))
(out/'counters.json').write_text(json.dumps(dict(passed=True,physical_board=False,dut_unchanged=True,cases=records),indent=2))
print('RTL_PHASE_OBSERVATIONS_PASS')
