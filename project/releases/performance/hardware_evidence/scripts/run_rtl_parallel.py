"""Run independent test cases concurrently against unchanged exported RTL / vendor models."""
from pathlib import Path
import subprocess,shutil,time,json,re,sys,hashlib,os,ctypes
root=Path(__file__).resolve().parents[1];work=root/'build/rtl_lite';reports=root/'reports'
tool=Path('D:/vivado/2025.2/Vivado/bin')
def run(cmd,cwd,log):
    with log.open('w') as f:
        p=subprocess.run([str(x) for x in cmd],cwd=str(cwd),stdout=f,stderr=subprocess.STDOUT)
    if p.returncode:raise RuntimeError('Tool failure '+str(log))
def memory_ready():
    if os.name!='nt':return True
    class Memory(ctypes.Structure):
        _fields_=[('length',ctypes.c_uint32),('load',ctypes.c_uint32)]+[(n,ctypes.c_uint64) for n in ['total','available','total_page','avail_page','total_virtual','avail_virtual','extended']]
    memory=Memory();memory.length=ctypes.sizeof(memory)
    if not ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(memory)):raise RuntimeError('Cannot query available memory')
    return memory.available >= int(os.environ.get('PS_RTL_MIN_FREE_MIB','2048'))*1024*1024

def main():
    if '--resume-compiled' not in sys.argv:
        run(['D:/chang/python.exe','scripts/make_rtl_lite.py'],root,reports/'rtl_generate.log')
        run(['D:/CCC/mingw64/bin/g++.exe','-std=c++17','-O3','-ffp-contract=off','-fno-fast-math','-Wno-unknown-pragmas',
             'tests/rtl_vectors.cpp','src/reference_gemv.cpp','-o','bin/rtl_vectors.exe'],root,reports/'rtl_vectors_build.log')
        run([root/'bin/rtl_vectors.exe','data/D11_rgb.pfm','data/D11_rgb.pfy',work],root,reports/'rtl_vectors.log')
        run(['D:/chang/python.exe','scripts/check_rtl_sources.py'],root,reports/'rtl_pairing.log')
        run([tool/'xelab.bat','xil_defaultlib.rtl_lite_tb','xil_defaultlib.glbl','-prj','rtl_lite.prj',
            '-L','xil_defaultlib','-L','unisims_ver','-L','xpm','-L','floating_point_v7_1_21','-L','floating_point_v7_0_26',
            '-relax','-s','rtl_lite','-log',reports/'rtl_lite_elab.log'],work,reports/'rtl_elab_console.log')
    jobs_limit=5 # Task-local maximum; launches remain staggered and memory gated.
    if not 1<=jobs_limit<=6:raise ValueError('PS_RTL_JOBS must be 1..6')
    pending=[];queued=[];results=[];shards=[-1,10,11,12,13,14]
    for case in shards:
        wd=root/'build'/('rtl_shard_'+str(case))
        if wd.exists() and '--resume-compiled' not in sys.argv:
            raise RuntimeError('Use a fresh ASCII build directory; refusing stale shard '+str(wd))
        wd.mkdir(exist_ok=True)
        if not (wd/'xsim.dir').exists():
            shutil.copytree(work/'xsim.dir',wd/'xsim.dir')
            for p in work.glob('*.hex'):shutil.copy2(p,wd/p.name)
            shutil.copy2(work/'run.tcl',wd/'run.tcl')
        else:
            for src in (work/'xsim.dir').rglob('*'):
                if src.is_file() and src.suffix in ('.exe','.mem','.reloc','.rtti','.svtype','.type','.version'):
                    dst=wd/'xsim.dir'/src.relative_to(work/'xsim.dir')
                    if hashlib.sha256(src.read_bytes()).digest()!=hashlib.sha256(dst.read_bytes()).digest():
                        raise RuntimeError('Compiled shard mismatch '+str(dst))
            for src in work.glob('*.hex'):
                if hashlib.sha256(src.read_bytes()).digest()!=hashlib.sha256((wd/src.name).read_bytes()).digest():
                    raise RuntimeError('Shard vector mismatch '+src.name)
        # Each process has a private snapshot directory, wave database and log.
        log=reports/('rtl_shard_'+str(case)+'.log');console=log.with_suffix('.console.txt')
        options=wd/'options.txt'
        options.write_text('rtl_lite\n-tclbatch run.tcl\n--noieeewarnings\n-testplusarg CASE='+str(case)+'\n-log '+log.as_posix()+'\n')
        if '--resume-compiled' in sys.argv and log.exists():
            previous=log.read_text(errors='replace')
            if 'RTL_SHARD_PASS case='+str(case) in previous and 'Fatal:' not in previous:
                results.append(dict(case=case,passed=True,reused_completed_shard=True,seconds=None,log=log.name))
                continue
            shutil.copy2(log,log.with_suffix('.interrupted.log'))
        queued.append((case,wd,log,console))
    last_start=-1e9
    while pending or queued:
        while queued and len(pending)<jobs_limit and time.monotonic()-last_start>=10 and memory_ready():
            case,wd,log,console=queued.pop(0)
            f=console.open('w');cmd=[str(tool/'xsim.bat'),'-f','options.txt']
            p=subprocess.Popen(cmd,cwd=str(wd),stdout=f,stderr=subprocess.STDOUT)
            pending.append((case,p,f,log,time.monotonic()));last_start=time.monotonic();print('RTL_SHARD_STARTED',case,flush=True)
        for item in pending[:]:
            case,p,f,log,tic=item
            if p.poll() is None:continue
            f.close();text=log.read_text(errors='replace') if log.exists() else ''
            passed=p.returncode==0 and 'RTL_SHARD_PASS case='+str(case) in text and 'Fatal:' not in text
            results.append(dict(case=case,passed=passed,seconds=time.monotonic()-tic,log=log.name))
            print('RTL_SHARD_FINISHED',case,passed,flush=True);pending.remove(item)
        time.sleep(2)
    text='\n'.join((reports/('rtl_shard_'+str(k)+'.log')).read_text(errors='replace') for k in shards)
    cases=re.findall(r'RTL_LITE_CASE_PASS n=(-?\d+) cycles=(\d+) stalls=(\d+) matrix_beats=(\d+) status=([0-9a-f]+)',text)
    passed=len(cases)==19 and all(r['passed'] for r in results)
    report=dict(passed=passed,shards=results,cases=[dict(n=int(n),cycles=int(c),stalls=int(s),matrix_beats=int(b),status=h) for n,c,s,b,h in cases])
    (reports/'rtl_parallel.json').write_text(json.dumps(report,indent=2))
    if not passed:raise RuntimeError('RTL suite failed')
    (reports/'rtl_lite_sim.log').write_text(text+'\nRTL_LITE_ALL_PASS cases=19 independent_shards=6\n')
    print('RTL_LITE_ALL_PASS cases=19 independent_shards=6',flush=True)
if __name__=='__main__':main()
