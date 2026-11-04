"""Offline tamper checks on a disposable package; restores every changed byte."""
from pathlib import Path
import argparse,importlib.util,json,copy,hashlib,sys
sys.dont_write_bytecode=True

def main():
    p=argparse.ArgumentParser();p.add_argument('--package',type=Path,required=True);p.add_argument('--report',type=Path,required=True);a=p.parse_args();root=a.package.resolve()
    sys.path.insert(0,str(root/'scripts'))
    spec=importlib.util.spec_from_file_location('check',root/'scripts/release_selfcheck.py');check=importlib.util.module_from_spec(spec);spec.loader.exec_module(check)
    path=root/'release_manifest.json';original=path.read_bytes();manifest=json.loads(original);checks=[]
    check.validate(root);checks.append('actual_package_pass')
    def rejects(label,mutate):
        candidate=copy.deepcopy(manifest);mutate(candidate);path.write_text(json.dumps(candidate),'utf-8')
        try:
            try:check.validate(root)
            except (ValueError,RuntimeError,FileNotFoundError,KeyError):checks.append(label)
            else:raise RuntimeError('Tamper accepted: '+label)
        finally:path.write_bytes(original)
    rejects('old_software_revision',lambda m:m.update(software_revision='old'))
    rejects('wrong_hardware_token',lambda m:m.update(hardware_release='old'))
    rejects('wrong_layout',lambda m:m.update(layout_version=0))
    rejects('false_board_claim',lambda m:m.update(physical_board_test=True))
    rejects('incomplete_acceptance',lambda m:m.update(offline_acceptance=False))
    rejects('missing_required_driver',lambda m:m['files'].pop('src/pl_job.hpp'))
    rejects('wrong_bitstream_clock',lambda m:m.update(hardware_profile='p56_150' if m['hardware_profile']=='p56_100' else 'p56_100'))
    rejects('manifest_escape',lambda m:m['files'].update({'../outside':dict(bytes=0,sha256='0'*64)}))
    for rel in ['bin/ps_full_aarch64','bin/full_board_fan_safe.bit','src/pl_job.hpp','scripts/run_target.py','configs/hardware_profiles.json','data/manifest.json']:
        file=root/rel;raw=file.read_bytes()
        try:
            file.write_bytes(raw+b'!')
            try:check.validate(root)
            except (ValueError,RuntimeError):checks.append('tampered_'+rel)
            else:raise RuntimeError('Tampered bytes accepted: '+rel)
        finally:file.write_bytes(raw)
    assert path.read_bytes()==original;check.validate(root)
    a.report.write_text(json.dumps(dict(passed=True,checks=checks,hardware_access=False,all_bytes_restored=True),indent=2)+'\n','utf-8')
    print('PACKAGE_NEGATIVE_CHECKS_PASS',len(checks))
if __name__=='__main__':main()
