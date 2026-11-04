"""Pinned, versioned model-bank storage; no pickle or per-frame decomposition."""
from pathlib import Path
import io,json
import numpy as np
from contracts import require,pinned,sha,write_json,read_json
from bank import ModelBank,Candidate,build_candidate


def save_bank(bank,folder):
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=False)
    arrays={'rows':bank.rows};records=[]
    for i,c in enumerate(bank.candidates):
        arrays[f'coarse_{i}']=c.coarse;arrays[f'full_{i}']=c.full
        records.append(dict(id=c.id,model=c.model,model_id=c.model_id,position_m=c.position_m))
    np.savez(folder/'basis.npz',**arrays)
    data=(folder/'basis.npz').read_bytes()
    write_json(folder/'bank.json',dict(schema_version=1,bank_id=bank.bank_id,scope=bank.scope,
        color_weights=bank.color_weights,policy=bank.policy,candidates=records,
        arrays=dict(path='basis.npz',bytes=len(data),sha256=sha(data))))


def load_bank(path,model_base):
    path=Path(path);d=read_json(path)
    require(d['schema_version']==1,'BANK_VERSION','Unknown bank schema')
    if d['scope']=='REAL_CALIBRATED':
        ev=json.loads(pinned(path.parent,d['validation']))
        require(ev.get('passed') is True and ev.get('scope')=='REAL_CALIBRATED' and
                ev.get('bank_id')==d['bank_id'] and ev.get('position_coordinate_id'),
                'BANK_EVIDENCE','Real bank requires independent physical/position evidence')
    with np.load(io.BytesIO(pinned(path.parent,d['arrays'])),allow_pickle=False) as z:
        candidates=[Candidate(c['id'],z[f'coarse_{i}'],z[f'full_{i}'],c['model'],c['model_id'],c['position_m']) for i,c in enumerate(d['candidates'])]
        bank=ModelBank(z['rows'],d['color_weights'],candidates,d['policy'],d['scope'])
    require(bank.bank_id==d['bank_id'],'BANK_HASH','Bank identity mismatch')
    for c in bank.candidates:bank.verify_candidate(c.id,model_base)
    return bank


def build_bank(spec_path,output,model_base):
    spec=read_json(spec_path)
    require(spec['scope']!='REAL_CALIBRATED','BANK_EVIDENCE','Build candidate offline; physical validation is a separate registration step')
    candidates=[build_candidate(c['id'],c['model'],model_base,spec['rows'],spec['kappa_coarse'],spec['kappa_full'],c.get('position_m'))[0] for c in spec['candidates']]
    bank=ModelBank(spec['rows'],spec['color_weights'],candidates,spec['policy'],spec['scope'])
    save_bank(bank,output)
    return bank
