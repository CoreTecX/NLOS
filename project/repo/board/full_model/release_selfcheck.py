"""Read-only, relocatable final package check. Never initializes board hardware."""
from pathlib import Path
import hashlib,json,struct,subprocess,sys
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from hardware_profile import TOKEN,SOFTWARE_REVISION,verify_binding

def sha(path):
    digest=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1048576),b''):digest.update(block)
    return digest.hexdigest()

def validate(root):
    root=root.resolve();manifest=json.loads((root/'release_manifest.json').read_text('utf-8'))
    if manifest.get('release_id')!=TOKEN or manifest.get('hardware_release')!=TOKEN or manifest.get('software_revision')!=SOFTWARE_REVISION or manifest.get('layout_version')!=0x32525001:raise ValueError('Wrong package identity')
    if manifest.get('offline_acceptance') is not True or manifest.get('physical_board_test') is not False:raise ValueError('Wrong offline/board status')
    required={'bin/ps_full_aarch64','bin/ps_full_legacy_aarch64','bin/ps_full_aarch64_profile','bin/dct_test_aarch64','bin/full_board_fan_safe.bit','bin/full_board.xsa','configs/hardware_profiles.json','reports/acceptance.json','data/manifest.json','scripts/run_target.py','scripts/hardware_profile.py','scripts/accept_state.py','src/full_runtime.hpp','src/pl_backend.hpp','src/pl_job.hpp','board_acceptance.csv'}
    if not required<=manifest.get('files',{}).keys():raise ValueError('Incomplete manifest')
    for rel,entry in manifest['files'].items():
        path=(root/rel).resolve()
        if Path(rel).is_absolute() or root not in path.parents:raise ValueError('Manifest path escapes package')
        if not path.is_file() or path.stat().st_size!=entry['bytes'] or sha(path)!=entry['sha256']:raise ValueError('File mismatch: '+rel)
    acceptance=json.loads((root/'reports/acceptance.json').read_text('utf-8'))
    if acceptance.get('offline_gates_passed') is not True or acceptance.get('physical_board_test') is not False:raise ValueError('Acceptance gate incomplete')
    rows=json.loads((root/'data/manifest.json').read_text('utf-8'))
    if len(rows)!=15 or len({r['key'] for r in rows})!=15:raise ValueError('Incomplete case suite')
    for row in rows:
        for key in ('model','measurement','golden'):
            rel='data/'+row[key]['file']
            if rel not in manifest['files'] or sha(root/rel)!=row[key]['sha256']:raise ValueError('Input manifest mismatch')
    catalog=json.loads((root/'configs/hardware_profiles.json').read_text('utf-8'))
    for name in ['ps_full_aarch64','ps_full_legacy_aarch64','ps_full_aarch64_profile']:
        verify_binding(manifest,catalog,name)
        elf=(root/'bin'/name).read_bytes()
        if elf[:6]!=b'\x7fELF\x02\x01' or struct.unpack_from('<H',elf,18)[0]!=183:raise ValueError('Not AArch64 ELF')
        off=struct.unpack_from('<Q',elf,32)[0];size,count=struct.unpack_from('<HH',elf,54)
        if {2,3}.intersection(struct.unpack_from('<I',elf,off+i*size)[0] for i in range(count)):raise ValueError('Dynamic dependency')
    entry=catalog['profiles'][manifest['hardware_profile']]
    if sha(root/'bin/full_board.xsa')!=entry['xsa_sha256']:raise ValueError('XSA/hardware mismatch')
    return manifest

def main():
    manifest=validate(ROOT)
    for backend in ['cpu','pl']:
        subprocess.run([sys.executable,'-B',str(ROOT/'scripts/run_target.py'),'--backend',backend,'--all','--preflight-only'],check=True)
    print('P11_OFFLINE_SELFCHECK_PASS',manifest['hardware_profile'],'files='+str(len(manifest['files'])),'board_access=0')
if __name__=='__main__':main()
