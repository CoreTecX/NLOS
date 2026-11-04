"""Exercise rejection branches without opening hardware or touching real inputs."""
from pathlib import Path
import sys,importlib.util,tempfile,json,hashlib,shutil
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('launcher',ROOT/'scripts/run_target.py');module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
def main():
    checks=[]
    with tempfile.TemporaryDirectory(prefix='ps_m56_launcher_',dir='D:/Temp') as temp:
        root=Path(temp);module.ROOT=root
        for d in ('bin','data','scripts','configs'):(root/d).mkdir()
        elf=bytearray(64);elf[:6]=b'\x7fELF\x02\x01';elf[18:20]=(183).to_bytes(2,'little')
        files={'bin/ps_full_aarch64':bytes(elf),'bin/ps_full_legacy_aarch64':bytes(elf),'bin/dct_test_aarch64':bytes(elf),'bin/full_board_fan_safe.bit':b'fixture bitstream',
               'data/model.pfm':b'fixture model','data/measurement.pfy':b'fixture measurement',
               'data/manifest.json':b'[]','scripts/run_target.py':b'fixture launcher','scripts/hardware_profile.py':b'fixture clock profile'}
        catalog=dict(hardware_release=module.TOKEN,layout_abi='0x32525001',profiles={'p56_100':dict(offline_passed=True,actual_mhz=100.0,pl0_value=0x01010f00,
            bitstream_sha256=hashlib.sha256(files['bin/full_board_fan_safe.bit']).hexdigest(),
            elf_sha256={name:hashlib.sha256(bytes(elf)).hexdigest() for name in ['ps_full_aarch64','ps_full_legacy_aarch64','ps_full_aarch64_profile']})})
        files['configs/hardware_profiles.json']=json.dumps(catalog).encode('utf-8')
        for rel,b in files.items():(root/rel).write_bytes(b)
        manifest=dict(release_id=module.TOKEN,hardware_release=module.TOKEN,hardware_profile='p56_100',layout_version=0x32525001,
                      files={rel:dict(sha256=hashlib.sha256(b).hexdigest()) for rel,b in files.items()})
        rows=[dict(model=dict(file='model.pfm'),measurement=dict(file='measurement.pfy'))]
        def save():(root/'release_manifest.json').write_text(json.dumps(manifest))
        def reject(name,dct='optimized',profile=False):
            try:module.preflight('pl',rows,dct,profile)
            except (RuntimeError,FileNotFoundError):checks.append(name);return
            raise RuntimeError('Failed to reject '+name)
        save();module.preflight('cpu',rows);module.preflight('pl',rows);module.preflight('cpu',rows,'legacy');module.preflight('pl',rows,'legacy');checks+=['valid_cpu_fixture','valid_pl_fixture','valid_legacy_cpu','valid_legacy_pl']
        reject('profile_missing_from_frozen_manifest',profile=True)
        name='bin/ps_full_aarch64_profile';(root/name).write_bytes(elf)
        manifest['files'][name]=dict(sha256=hashlib.sha256(elf).hexdigest());save()
        module.preflight('cpu',rows,profile=True);checks.append('valid_profile_manifest')
        (root/name).write_bytes(elf+b'!');reject('tampered_profile_binary',profile=True)
        (root/name).write_bytes(elf);reject('legacy_profile_rejected','legacy',True)
        manifest['release_id']='20261014';save();reject('old_release');manifest['release_id']=module.TOKEN
        manifest['hardware_profile']='p56_150';save();reject('unapproved_or_mismatched_bitstream_clock');manifest['hardware_profile']='p56_100'
        manifest['hardware_profile']='unknown';save();reject('unknown_hardware_profile');manifest['hardware_profile']='p56_100'
        manifest['hardware_release']='old';save();reject('wrong_hardware_token');manifest['hardware_release']=module.TOKEN
        manifest['layout_version']=0;save();reject('wrong_layout');manifest['layout_version']=0x32525001;save()
        for rel,b in files.items():
            (root/rel).write_bytes(b+b'!');reject('tampered_'+rel,'legacy' if 'legacy' in rel else 'optimized');(root/rel).write_bytes(b)
        bad=bytearray(elf);bad[18:20]=(62).to_bytes(2,'little');(root/'bin/ps_full_aarch64').write_bytes(bad)
        manifest['files']['bin/ps_full_aarch64']['sha256']=hashlib.sha256(bad).hexdigest();save();reject('wrong_elf_arch_with_matching_hash')
    from unittest.mock import patch
    from types import SimpleNamespace
    with tempfile.TemporaryDirectory(prefix='ps_m4_selftest_',dir='D:/Temp') as temp:
        output=Path(temp)
        good=dict(passed=True,bit_exact=True,uses_neon=True,cases=3220)
        for label,code,payload,should_pass in [('valid_selftest',0,good,True),('selftest_failed',1,good,False),('selftest_not_neon',0,dict(good,uses_neon=False),False),('selftest_not_exact',0,dict(good,bit_exact=False),False),('selftest_incomplete',0,dict(good,cases=1),False),('selftest_bad_json',0,'invalid-json',False)]:
            result=SimpleNamespace(returncode=code,stdout=json.dumps(payload) if isinstance(payload,dict) else payload,stderr='')
            with patch.object(module.subprocess,'run',return_value=result):
                try:module.dct_selftest(output);passed=True
                except (RuntimeError,ValueError):passed=False
                assert passed==should_pass,label
                assert (output/'dct_selftest.log').exists()
                checks.append(label)
    (ROOT/'reports/launcher_checks.json').write_text(json.dumps(dict(passed=True,checks=checks,hardware_access=False),indent=2))
    print('LAUNCHER_CHECKS_PASS',len(checks),'hardware_access=0')
if __name__=='__main__':main()
