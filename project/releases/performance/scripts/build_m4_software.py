"""Compile native and AArch64 software; never program or access hardware."""
from pathlib import Path
import argparse, hashlib, json, subprocess

ROOT=Path(__file__).resolve().parents[1]
FLAGS=['-std=c++17','-O3','-ffp-contract=off','-fno-fast-math','-Wall','-Wextra','-Werror',
       '-Wno-unknown-pragmas','-Wno-unused-label']
HOST=Path('g++')
ARM=Path('aarch64-linux-gnu-g++')

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    (ROOT/'bin').mkdir(exist_ok=True)
    (ROOT/'reports').mkdir(exist_ok=True)
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--host',type=Path,default=HOST)
    p.add_argument('--arm',type=Path,default=ARM);a=p.parse_args();records=[]
    jobs=[('ps_full_host.exe',a.host,[],['src/main.cpp','src/reference_gemv.cpp']),
          ('ps_full_legacy_host.exe',a.host,['-DPS_DCT_LEGACY'],['src/main.cpp','src/reference_gemv.cpp']),
          ('ps_full_packed_host.exe',a.host,['-DPS_TEST_PACKED_HOST'],['src/main.cpp','src/full_gemv.cpp','src/reference_gemv.cpp']),
          ('dct_test.exe',a.host,[],['tests/dct_test.cpp']),
          ('ps_full_aarch64',a.arm,['-static','-mcpu=cortex-a53'],['src/main.cpp','src/reference_gemv.cpp']),
          ('ps_full_legacy_aarch64',a.arm,['-static','-mcpu=cortex-a53','-DPS_DCT_LEGACY'],['src/main.cpp','src/reference_gemv.cpp']),
          ('dct_test_aarch64',a.arm,['-static','-mcpu=cortex-a53'],['tests/dct_test.cpp']),
          ('dct_test_portable_aarch64',a.arm,['-static','-mcpu=cortex-a53','-DPS_DCT_PORTABLE'],['tests/dct_test.cpp'])]
    for name,compiler,extra,sources in jobs:
        cmd=[str(compiler),*FLAGS,*extra,*sources,'-o','bin/'+name]
        result=subprocess.run(cmd,cwd=ROOT,capture_output=True,text=True)
        (ROOT/'reports'/('build_'+name+'.log')).write_text(result.stdout+result.stderr,'utf-8')
        if result.returncode:raise RuntimeError(name+': '+result.stderr)
        records.append({'output':'bin/'+name,'command':cmd,'compiler_sha256':sha(compiler),
                        'version':subprocess.check_output([str(compiler),'--version'],text=True),
                        'binary_sha256':sha(ROOT/'bin'/name)})
        print('BUILT',name,flush=True)
    (ROOT/'reports/build.json').write_text(json.dumps({'jobs':records,'sources':{
        str(f.relative_to(ROOT)):sha(f) for d in ('src','tests','reference') for f in (ROOT/d).glob('*')
        if f.suffix in ('.cpp','.hpp','.inc')}},indent=2),'utf-8')

if __name__=='__main__':main()
