"""Read-only candidate checks and explicit application-only installation. Python 3.7+."""
from pathlib import Path, PurePosixPath
from contextlib import contextmanager
import argparse, hashlib, importlib, json, os, platform, re, shutil, struct, sys, tempfile


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1048576), b''):
            h.update(block)
    return h.hexdigest()


def read(path):
    return json.loads(Path(path).read_text('utf-8-sig'))


def child(root, name):
    rel = PurePosixPath(name)
    if not isinstance(name, str) or '\\' in name or ':' in name or rel.is_absolute() or '..' in rel.parts or not rel.parts:
        raise ValueError('Unsafe package path: ' + str(name))
    path = root.joinpath(*rel.parts)
    if any(p.is_symlink() for p in [path] + list(path.parents)):
        raise ValueError('Symlink in package path: ' + name)
    if root not in path.resolve().parents:
        raise ValueError('Path leaves package')
    return path


def verify(root):
    root = Path(root).resolve()
    manifest = read(root / 'candidate_manifest.json')
    meta = read(root / 'candidate.json')
    if manifest['candidate_id'] != meta['candidate_id'] or not re.fullmatch(r'[a-zA-Z0-9][a-zA-Z0-9._-]{0,79}', meta['candidate_id']):
        raise ValueError('Candidate identity mismatch')
    if meta['new_board_verified'] or meta['r14_complete'] or meta['nb16_continuous_stability_passed']:
        raise ValueError('Offline candidate must preserve pending physical/stability gates')
    if set(meta['releases']) != {'performance','fallback'} or (meta['full_dimension'],meta['arithmetic'],meta['iteration_limit'],meta['layout_abi']) != (1044,'FP64',20000,'0x32525001'):
        raise ValueError('Full algorithm and both frozen releases are required')
    actual = set()
    for p in root.rglob('*'):
        if p.is_symlink():
            raise ValueError('Symlink found in candidate')
        if p.is_file():
            actual.add(p.relative_to(root).as_posix())
    expected = set(manifest['files']) | {'candidate_manifest.json'}
    if actual != expected:
        raise ValueError('Missing or unexpected package files: ' + repr(sorted(actual ^ expected)[:10]))
    for rel, record in manifest['files'].items():
        p = child(root, rel)
        if p.stat().st_size != record['bytes'] or sha(p) != record['sha256']:
            raise ValueError('Hash/length mismatch: ' + rel)
    for name, binding in meta['releases'].items():
        release = child(root, binding['path'] + '/release_manifest.json')
        data = read(release)
        if sha(release) != binding['manifest_sha256'] or data['hardware_profile'] != binding['hardware_profile']:
            raise ValueError('Frozen profile binding mismatch: ' + name)
        if data['layout_version'] != 0x32525001 or data['hardware_release'] != '20261016-rp2-p56-v1' or data['software_revision'] != '20261017-p789-p1011-v1':
            raise ValueError('Frozen software/hardware/ABI mismatch')
        for rel, record in data['files'].items():
            p = child(release.parent, rel)
            if p.stat().st_size != record['bytes'] or sha(p) != record['sha256']:
                raise ValueError('Frozen file mismatch: ' + name + '/' + rel)
    cfg = read(root / 'configs/linux_cpu.json')
    if cfg['backend_mode'] != 'cpu' or cfg['allow_pl'] or cfg.get('pl_readiness') or cfg['host_libraries'] or cfg['resource_cache_enabled']:
        raise ValueError('Unsafe default backend configuration')
    if cfg['release'] != meta['releases']['performance']['path'] or cfg['release_manifest_sha256'] != meta['releases']['performance']['manifest_sha256']:
        raise ValueError('CPU configuration binding mismatch')
    active = json.dumps([cfg, read(root / 'configs/deployment.json')])
    if re.search(r'COM\d+|[A-Za-z]:[\\/]|boot_id|board_mac', active, re.I):
        raise ValueError('Old host/board identity in active configuration')
    cores=read(root/'configs/core_source_lock.json')['files']
    if len(cores)!=65:raise ValueError('Expected complete 65-entry core source mapping')
    for record in cores:
        if sha(child(root/'repo',record['path'])) != record['sha256']:
            raise ValueError('Full algorithm core changed: '+record['path'])
    wheels=read(root/'configs/wheel_lock.json')['wheels']
    if {(w['package'],w['version']) for w in wheels}!={('numpy','1.21.6'),('scipy','1.7.3'),('Pillow','9.5.0')}:
        raise ValueError('Target dependency lock incomplete')
    for w in wheels:
        p=child(root,'wheelhouse/'+w['filename'])
        if p.stat().st_size!=w['bytes'] or sha(p)!=w['sha256']:
            raise ValueError('Target dependency wheel changed')
    native_files=[rel for rel in manifest['files'] if rel.startswith('native/bin/')]
    if len(native_files)!=8:raise ValueError('Eight freshly built ARM files required')
    for rel in manifest['files']:
        if rel.startswith('native/bin/'):
            data=child(root,rel).read_bytes()
            if len(data)<64 or data[:6]!=b'\x7fELF\x02\x01' or struct.unpack_from('<H',data,18)[0]!=183:
                raise ValueError('Expected little endian AArch64 ELF64')
            off=struct.unpack_from('<Q',data,32)[0];size,count=struct.unpack_from('<HH',data,54)
            if size<56 or count==0 or off+size*count>len(data) or any(struct.unpack_from('<I',data,off+i*size)[0] in (2,3) for i in range(count)):
                raise ValueError('Expected bounded static ELF without dynamic loader')
    catalog=child(root,cfg['raw_catalog']['path'])
    if sha(catalog)!=cfg['raw_catalog']['sha256'] or catalog.stat().st_size!=cfg['raw_catalog']['bytes']:
        raise ValueError('Historical RAW catalog pin changed')
    def raw_pins(value):
        if isinstance(value,dict):
            if 'workspace_path' in value:
                p=child(root,value['workspace_path'])
                if sha(p)!=value['sha256'] or p.stat().st_size!=value['bytes']:
                    raise ValueError('Historical RAW source changed')
            for v in value.values():raw_pins(v)
        elif isinstance(value,list):
            for v in value:raw_pins(v)
    raw_pins(read(catalog))
    # The batch manifest and every input pin must be self contained.
    batch = read(root/'data/batch_manifest.json')
    if len(batch['items']) != 97 or len(set(x['id'] for x in batch['items'])) != 97:
        raise ValueError('97 input identities required')
    for item in batch['items']:
        p = child(root/'data',item['input']['path'])
        if sha(p) != item['input']['sha256'] or p.stat().st_size != item['input']['bytes']:
            raise ValueError('Descriptor pin mismatch')
        desc = read(p)
        for role in ('file','model','golden'):
            pin = desc[role]
            asset = child(root, (p.parent / pin['path']).resolve().relative_to(root).as_posix())
            if sha(asset) != pin['sha256'] or asset.stat().st_size != pin['bytes']:
                raise ValueError('Input pin mismatch: ' + item['id'])
    return dict(passed=True,candidate_id=meta['candidate_id'],files=len(manifest['files']),
                eligible_inputs=97,hardware_accessed=False,new_board_verified=False,
                production_acceptance=False,pending_gates=meta['pending_gates'])


def preflight(root):
    result = verify(root)
    result.update(target_passed=False,dependencies={},errors=[])
    if platform.system() != 'Linux' or platform.machine().lower() not in ('aarch64','arm64'):
        result['errors'].append('Run target preflight on AArch64 Linux')
    if sys.version_info < (3,7):
        result['errors'].append('Python 3.7+ required')
    lock=read(Path(root)/'configs/runtime_dependency_lock.json')['legacy_arm_target']
    if sys.version_info[:2] != (3,7):
        result['errors'].append('Packaged target lock/wheels cover CPython 3.7; validate a separate environment for another Python version')
    for name in ('numpy','scipy','PIL'):
        try:
            mod = importlib.import_module(name)
            result['dependencies'][name] = getattr(mod,'__version__','unknown')
            key='Pillow' if name=='PIL' else name
            if result['dependencies'][name]!=lock[key]:
                result['errors'].append('Target dependency lock mismatch: '+key)
        except Exception as e:
            result['errors'].append(name + ': ' + str(e))
    try:
        import numpy as np
        from scipy.fft import dctn,idctn
        v=np.arange(12,dtype=np.float64).reshape(3,4)
        if not np.allclose(v,idctn(dctn(v,norm='ortho'),norm='ortho'),rtol=0,atol=1e-12):
            result['errors'].append('DCT numerical preflight failed')
    except Exception as e:
        result['errors'].append('DCT: '+str(e))
    result['target_passed'] = not result['errors']
    return result


def atomic(path, value):
    fd, name = tempfile.mkstemp(prefix='.pointer_',dir=str(path.parent))
    try:
        with os.fdopen(fd,'w',encoding='utf-8') as stream:
            json.dump(value,stream,indent=2);stream.write('\n');stream.flush();os.fsync(stream.fileno())
        os.replace(name,str(path))
    finally:
        if os.path.exists(name):os.unlink(name)


def destination(path):
    supplied = Path(path).absolute()
    if any(p.is_symlink() for p in [supplied]+list(supplied.parents)):
        raise ValueError('Installation destination must not contain symlinks')
    dest = supplied.resolve()
    if dest == Path(dest.anchor) or len(dest.parts)<3:
        raise ValueError('Choose a dedicated application directory')
    dest.mkdir(parents=True,exist_ok=True)
    for name in ('slots','current.json'):
        if (dest/name).is_symlink():raise ValueError('Installation metadata must not be symlinks')
    return dest


def slot_from_pointer(dest, pointer):
    slot = child(dest,pointer['slot'])
    if slot.parent != dest/'slots':raise ValueError('Pointer must name a direct version slot')
    checked=verify(slot)
    if checked['candidate_id']!=pointer['candidate_id'] or sha(slot/'candidate_manifest.json')!=pointer['manifest_sha256']:
        raise ValueError('Installed slot identity mismatch')
    return slot


@contextmanager
def installation_lock(dest):
    path=dest/'.install.lock'
    try:path.mkdir()
    except FileExistsError:raise ValueError('Another install/rollback or an interrupted operation holds .install.lock; inspect before retry')
    try:yield
    finally:path.rmdir()


def _install(root, dest):
    root=Path(root).resolve(); result=verify(root);dest=destination(dest)
    if root==dest or root in dest.parents or dest in root.parents:
        raise ValueError('Source and installation must be separate trees')
    required=sum(p.stat().st_size for p in root.rglob('*') if p.is_file())+64*1024*1024
    if shutil.disk_usage(str(dest)).free<required:raise ValueError('Insufficient space for a separate verified version slot')
    slots=dest/'slots';slots.mkdir(exist_ok=True)
    target=slots/result['candidate_id']
    if target.exists():raise ValueError('Version already exists; no overwrite')
    previous=read(dest/'current.json') if (dest/'current.json').exists() else None
    if previous:slot_from_pointer(dest,previous)
    incoming=Path(tempfile.mkdtemp(prefix='.incoming_',dir=str(slots)))
    # Incomplete staging remains available for diagnosis; never remove user trees.
    for p in root.iterdir():
        if p.is_dir():shutil.copytree(str(p),str(incoming/p.name))
        else:shutil.copy2(str(p),str(incoming/p.name))
    verify(incoming)
    if os.name!='nt':
        for base in ('native/bin','tools'):
            for p in (incoming/base).glob('*'):
                if p.is_file() and p.read_bytes()[:4]==b'\x7fELF':p.chmod(0o700)
    incoming.rename(target)
    pointer=dict(candidate_id=result['candidate_id'],slot=target.relative_to(dest).as_posix(),
                 manifest_sha256=sha(target/'candidate_manifest.json'),previous=previous,
                 application_only=True,services_started=False,hardware_accessed=False)
    atomic(dest/'current.json',pointer)
    return dict(passed=True,installed=str(target),pointer=str(dest/'current.json'),hardware_accessed=False,
                application_only=True,production_acceptance=False)


def install(root,dest):
    dest=destination(dest)
    with installation_lock(dest):return _install(root,dest)


def _rollback(dest):
    dest=destination(dest);current=read(dest/'current.json');slot_from_pointer(dest,current)
    previous=current.get('previous')
    if not previous:raise ValueError('No verified previous application slot; factory system is unchanged')
    slot=slot_from_pointer(dest,previous);atomic(dest/'current.json',previous)
    return dict(passed=True,selected=str(slot),hardware_accessed=False,fpga_reconfigured=False,
                scope='APPLICATION_POINTER_ONLY; hardware profile rollback is a separate field operation')


def rollback(dest):
    dest=destination(dest)
    with installation_lock(dest):return _rollback(dest)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('action',choices=['verify','target-preflight','install','rollback'])
    default=next((x for x in Path(__file__).resolve().parents if (x/'candidate.json').is_file()),Path(__file__).resolve().parents[1])
    p.add_argument('--candidate',type=Path,default=default)
    p.add_argument('--destination',type=Path)
    a=p.parse_args()
    if a.action in ('install','rollback') and a.destination is None:p.error('--destination is required')
    try:
        if a.action=='verify':r=verify(a.candidate)
        elif a.action=='target-preflight':r=preflight(a.candidate)
        elif a.action=='install':r=install(a.candidate,a.destination)
        else:r=rollback(a.destination)
        print(json.dumps(r,indent=2,ensure_ascii=False));return 0 if r.get('target_passed',r['passed']) else 2
    except Exception as e:
        print(json.dumps(dict(passed=False,error=str(e),hardware_accessed=False),ensure_ascii=False));return 2


if __name__=='__main__':raise SystemExit(main())
