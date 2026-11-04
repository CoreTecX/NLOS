"""Portable, read-only dataset validation. Compatible with board Python 3.7."""
from pathlib import Path
import hashlib,json,math,re,struct,zlib

SCHEMA='psight-dataset-v1'
SOFTWARE='20261017-p789-p1011-v1'
HARDWARE='20261016-rp2-p56-v1'
READY={'frozen_independent_reference','independent_reference_passed'}

def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda:f.read(1048576),b''):h.update(block)
    return h.hexdigest()

def contained(root,name):
    if not isinstance(name,str) or not name or Path(name).is_absolute() or '\\' in name or ':' in name:
        raise ValueError('Invalid package relative path')
    path=(root/name).resolve()
    if root not in path.parents:raise ValueError('Path escapes package')
    return path

def check_file(root,entry):
    p=contained(root,entry['file'])
    if not p.is_file() or p.stat().st_size!=entry['bytes'] or sha(p)!=entry['sha256']:
        raise ValueError('Dataset file mismatch: '+entry['file'])
    return p

def check_blob(path,kind,shape,identity,payload_size):
    raw=path.read_bytes()
    if len(raw)!=64+payload_size or raw[:8]!=b'PSFULL1\0' or struct.unpack_from('<IIII',raw,8)!=(1,kind,*shape):raise ValueError('Blob header mismatch: '+str(path))
    if struct.unpack_from('<Q',raw,24)[0]!=payload_size or raw[36:40]!=bytes(4) or raw[56:64]!=bytes(8):raise ValueError('Blob length/reserved mismatch')
    if raw[40:56].hex()!=identity or zlib.crc32(raw[64:])!=struct.unpack_from('<I',raw,32)[0]:raise ValueError('Blob identity/CRC mismatch')
    return raw[64:]

def validate(root,verify_all=True):
    root=Path(root).resolve()
    seal=json.loads((root/'package_manifest.json').read_text('utf8'))
    if seal.get('schema')!=SCHEMA:raise ValueError('Unknown dataset package schema')
    sealed={};checked=set()
    for entry in seal['files']:
        if entry['file'] in sealed:raise ValueError('Duplicate sealed file')
        sealed[entry['file']]=entry
    for required in ('dataset_manifest.json','release_lock.json','scripts/dataset_contract.py','scripts/run_dataset.py'):
        if required not in sealed:raise ValueError('Missing sealed package metadata: '+required)
    def verified(entry):
        expected=sealed.get(entry['file'])
        if expected is None or any(entry[k]!=expected[k] for k in ('sha256','bytes')):raise ValueError('File is not bound to package manifest')
        key=(entry['file'],entry['sha256'])
        if key not in checked:
            path=check_file(root,entry);checked.add(key);return path
        return contained(root,entry['file'])
    if verify_all:
        for entry in seal['files']:verified(entry)
    else:
        verified(sealed['dataset_manifest.json']);verified(sealed['release_lock.json'])
    m=json.loads((root/'dataset_manifest.json').read_text('utf8'))
    if (m.get('schema'),m.get('software_revision'),m.get('hardware_release'))!=(SCHEMA,SOFTWARE,HARDWARE):raise ValueError('Dataset version mismatch')
    if m.get('physical_board_run') is not False:raise ValueError('Prepared dataset must not claim a board run')
    lock=json.loads((root/'release_lock.json').read_text('utf8'))
    if lock.get('schema')!=SCHEMA or lock.get('software_revision')!=SOFTWARE or lock.get('hardware_release')!=HARDWARE:raise ValueError('Release lock version mismatch')
    keys=set();models={};n=1044;wall=126*126
    for row in m['cases']:
        if not re.fullmatch(r'[A-Za-z0-9_-]+',row['key']) or row['key'] in keys:raise ValueError('Duplicate/invalid case key')
        keys.add(row['key'])
        if not re.fullmatch('[0-9a-f]{32}',row['model_id']):raise ValueError('Invalid model identity')
        model=verified(row['model'])
        if row['model_id'] in models and models[row['model_id']]!=row['model']['sha256']:raise ValueError('Ambiguous model identity')
        if row['model_id'] not in models:
            check_blob(model,1,(29,36),row['model_id'],8+n*8+n*wall*2+3*n*n*8+2*n*8)
            models[row['model_id']]=row['model']['sha256']
        measurement=verified(row['measurement'])
        body=check_blob(measurement,2,(126,126),row['model_id'],(3+wall*3)*8)
        values=struct.unpack('<'+str(3+wall*3)+'d',body)
        if not all(math.isfinite(v) for v in values) or min(values[:3])<=0 or min(values[3:])<0:raise ValueError('Invalid measurement values')
        if row['reference_status'] in READY:
            gold=verified(row['golden']);body=check_blob(gold,3,(29,36),row['model_id'],3*n*3*8)
            if not all(math.isfinite(v[0]) for v in struct.iter_unpack('<d',body)):raise ValueError('Nonfinite reference')
    return m,lock

def select_cases(manifest,keys=None,all_ready=False):
    rows={r['key']:r for r in manifest['cases']}
    if all_ready:selected=[r for r in manifest['cases'] if r['reference_status'] in READY]
    else:
        if not keys or len(set(keys))!=len(keys) or set(keys)-rows.keys():raise ValueError('Missing, duplicate or unknown case selection')
        selected=[rows[k] for k in keys]
    if not selected or any(r['reference_status'] not in READY for r in selected):raise ValueError('Selected case lacks a passing independent reference')
    return selected

def validate_release(root,lock):
    root=Path(root).resolve();manifest_path=root/'release_manifest.json';digest=sha(manifest_path)
    matching=[(k,v) for k,v in lock['packages'].items() if v['release_manifest_sha256']==digest]
    if len(matching)!=1:raise ValueError('Release does not match either pinned complete package')
    name,pin=matching[0]
    for rel,entry in pin['files'].items():check_file(root,dict(file=rel,**entry))
    return name,pin
