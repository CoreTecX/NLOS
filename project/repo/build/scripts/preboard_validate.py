"""Run host gates, optional AArch64 build, and hash the reviewed source set.

Hardware/product acceptance is deliberately kept REQUIRED in the result.
"""
from pathlib import Path
import argparse
import hashlib
import json
import os
import platform
import subprocess
import sys
import time


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--repo',type=Path,default=Path(__file__).resolve().parents[2])
    ap.add_argument('--make',default='make')
    ap.add_argument('--cc',help='AArch64 Linux cross compiler')
    ap.add_argument('--production-model',type=Path)
    ap.add_argument('--bitstream',type=Path)
    ap.add_argument('--output',type=Path,default=Path('build/preboard_validation'))
    args=ap.parse_args();r=args.repo.resolve();out=args.output.resolve();out.mkdir(parents=True,exist_ok=True)
    checks={}
    def run(name,command):
        started=time.monotonic()
        with (out/(name+'.log')).open('w',encoding='utf8') as log:
            proc=subprocess.run(command,cwd=r,stdout=log,stderr=subprocess.STDOUT)
        checks[name]={'status':'PASS' if proc.returncode==0 else 'FAIL',
                      'exit_code':proc.returncode,'elapsed_s':time.monotonic()-started}
        return proc.returncode==0
    ok=run('host_regression',[args.make,'PYTHON='+sys.executable.replace('\\','/'),'test'])
    if args.production_model:
        ok=run('production_frame',[sys.executable,str(r/'build/scripts/preboard_frame_e2e.py'),
                                  '--production-model',str(args.production_model.resolve())]) and ok
    if args.cc:
        ok=run('aarch64_build',[sys.executable,str(r/'build/scripts/build_board_release.py'),
                               '--cc',args.cc,'--static','--output',str(out/'ps_board_app')]) and ok
    files=subprocess.check_output(['git','-C',str(r),'ls-files','--cached','--others','--exclude-standard','-z']).decode().split('\0')
    hashes={}
    for name in sorted(set(files)):
        p=r/name
        if p.is_file() and (name.startswith(('src/','board/','configs/','build/scripts/')) or name in ('makefile','readme.md')):
            hashes[name]=hashlib.sha256(p.read_bytes()).hexdigest()
    payload={'generated_at':time.strftime('%Y-%m-%dT%H:%M:%S%z'),'python':sys.version,
             'platform':platform.platform(),'base_commit':subprocess.check_output(['git','-C',str(r),'rev-parse','HEAD']).decode().strip(),
             'software_gate':'PASS' if ok else 'FAIL','checks':checks,'source_sha256':hashes,
             'rtl_gate':'RUN_SEPARATELY','hardware_acceptance':'REQUIRED',
             'product_acceptance':'NOT_ESTABLISHED','note':'No hardware timing, camera, display or scene quality claim.'}
    if args.bitstream:
        payload['bitstream']={'path':str(args.bitstream.resolve()),'sha256':hashlib.sha256(args.bitstream.read_bytes()).hexdigest()}
    (out/'validation.json').write_text(json.dumps(payload,ensure_ascii=False,indent=2),encoding='utf8')
    print(json.dumps({k:payload[k] for k in ('software_gate','rtl_gate','hardware_acceptance','product_acceptance')},ensure_ascii=False))
    return 0 if ok else 1


if __name__=='__main__':
    sys.exit(main())
