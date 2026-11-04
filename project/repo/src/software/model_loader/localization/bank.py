"""Cached coarse/top-K subspace search, with full-model identity and rejection gates."""
from dataclasses import dataclass
from pathlib import Path
import hashlib,json
import numpy as np
from contracts import require,pinned,blob,MODEL_BYTES,decode_pfy,sha


def measurement_feature(pfy,rows,color_weights):
    _,_,rgb=decode_pfy(pfy)
    weights=np.asarray(color_weights,np.float64)
    require(weights.shape==(3,) and np.isfinite(weights).all() and np.all(weights>=0) and abs(weights.sum()-1)<1e-12,
            'LOCALIZATION_COLOR','Invalid registered color weights')
    image=rgb@weights
    return (image[1:]-image[:-1]).reshape(-1)[rows]


def model_feature_matrix(data,rows):
    _,payload=blob(data,1,(29,36),MODEL_BYTES)
    n=1044;m=126*126
    norm=np.frombuffer(payload,dtype='<f8',count=1)[0]
    scales=np.frombuffer(payload,dtype='<f8',count=n,offset=8)
    q=np.frombuffer(payload,dtype='<i2',count=n*m,offset=8+n*8).reshape(n,m)
    rows=np.asarray(rows,dtype=np.int64);y=rows//126;x=rows%126
    require(np.all(rows>=0) and np.all(rows<125*126),'LOCALIZATION_ROWS','Bad derivative row selection')
    # Old localization applies row difference in C image order; PFM stores MATLAB-ordered wall columns.
    a=(q[:,y+1+126*x].astype(np.float64)-q[:,y+126*x].astype(np.float64)).T
    return a*(scales/(np.sqrt(2)*norm))[None,:]


def cached_basis(A,kappa):
    require(0<kappa<=1,'LOCALIZATION_KAPPA','Rank threshold must be positive')
    A=np.asarray(A,np.float64)
    require(A.ndim==2 and np.isfinite(A).all(),'LOCALIZATION_MATRIX','Invalid forward feature matrix')
    u,s,_=np.linalg.svd(A,full_matrices=False)
    require(len(s)>0 and s[0]>0,'LOCALIZATION_MATRIX','Zero subspace')
    keep=s>=kappa*s[0]
    return u[:,keep],s[keep]


@dataclass(frozen=True)
class Candidate:
    id: str
    coarse: np.ndarray
    full: np.ndarray
    model: dict
    model_id: str
    position_m: object


class ModelBank:
    def __init__(self,rows,color_weights,candidates,policy,scope):
        rows=np.asarray(rows,dtype=np.int64)
        require(rows.ndim==1 and len(rows)>=8 and len(np.unique(rows))==len(rows) and np.all(rows>=0) and np.all(rows<15750),
                'LOCALIZATION_ROWS','Unique bounded derivative rows required')
        require(scope in ('SYNTHETIC_TEST_ONLY','HISTORICAL_MODEL_LIBRARY','REAL_CALIBRATED'),'LOCALIZATION_SCOPE','Missing bank scope')
        require(len(candidates)>=2 and len({c.id for c in candidates})==len(candidates),'LOCALIZATION_LIBRARY','At least two distinct candidates required')
        require(policy['top_k']>=2 and 0<policy['min_fit']<=1 and 0<policy['min_margin']<=1,'LOCALIZATION_POLICY','Invalid acceptance thresholds')
        self.rows=np.frombuffer(rows.tobytes(),rows.dtype);self.color_weights=tuple(color_weights);self.policy=dict(policy);self.scope=scope
        checked=[]
        for c in candidates:
            values=[]
            for u in (c.coarse,c.full):
                u=np.asarray(u,np.float64)
                require(u.ndim==2 and u.shape[0]==len(rows) and u.shape[1]>0 and np.isfinite(u).all() and
                        np.allclose(u.T@u,np.eye(u.shape[1]),atol=1e-9),'LOCALIZATION_BASIS','Nonorthonormal basis')
                values.append(np.frombuffer(u.tobytes(),np.float64).reshape(u.shape))
            checked.append(Candidate(c.id,*values,dict(c.model),c.model_id,c.position_m))
        self.candidates=tuple(checked)
        metadata=dict(scope=scope,policy=self.policy,color_weights=self.color_weights,
                      candidates=[dict(id=c.id,model=c.model,model_id=c.model_id,position_m=c.position_m,
                          coarse_shape=c.coarse.shape,full_shape=c.full.shape) for c in checked])
        self.bank_id=sha(rows.tobytes()+json.dumps(metadata,sort_keys=True,allow_nan=False).encode()+
                         b''.join(c.coarse.tobytes()+c.full.tobytes() for c in checked))

    def locate(self,pfy,source_kind,geometry_valid=True):
        if not geometry_valid:return dict(valid=False,reason='WALL_GEOMETRY_INVALID',candidate=None)
        if self.scope!='REAL_CALIBRATED' and source_kind!='SYNTHETIC':
            return dict(valid=False,reason='PHYSICAL_LIBRARY_NOT_CALIBRATED',candidate=None)
        y=measurement_feature(pfy,self.rows,self.color_weights)
        energy=float(y@y)
        if not np.isfinite(energy) or energy<=1e-20:return dict(valid=False,reason='UNOBSERVABLE',candidate=None)
        coarse=[float(np.sum((c.coarse.T@y)**2)/energy) for c in self.candidates]
        order=sorted(range(len(coarse)),key=lambda i:(-coarse[i],self.candidates[i].id))[:self.policy['top_k']]
        full=sorted([(min(1.,max(0.,float(np.sum((self.candidates[i].full.T@y)**2)/energy))),i) for i in order],reverse=True)
        # Coarse scores are lower bounds, not safe pruning upper bounds. Certify
        # omitted candidates too, so a weak coarse score cannot hide the winner
        # or a runner-up that should cause ambiguity rejection. SVD stays offline.
        omitted=[i for i in range(len(self.candidates)) if i not in order]
        full+= [(min(1.,max(0.,float(np.sum((self.candidates[i].full.T@y)**2)/energy))),i) for i in omitted]
        full.sort(reverse=True)
        best,i=full[0];margin=best-full[1][0]
        candidate=self.candidates[i]
        reason=None if best>=self.policy['min_fit'] and margin>=self.policy['min_margin'] else ('OUT_OF_LIBRARY' if best<self.policy['min_fit'] else 'AMBIGUOUS')
        return dict(valid=reason is None,reason=reason,candidate=candidate.id if reason is None else None,
            model_id=candidate.model_id if reason is None else None,model_hash=candidate.model['sha256'] if reason is None else None,
            position_m=candidate.position_m if reason is None else None,position_scope=self.scope,physical_mm_accuracy_accepted=False,
            fit=best,residual_fraction=float(np.sqrt(max(0.,1-best))),margin=margin,coarse_scores=coarse,
            top_k=[dict(candidate=self.candidates[j].id,fit=score) for score,j in full[:self.policy['top_k']]],
            evaluated_candidates=len(full),exhaustive_certificate=True,bank_id=self.bank_id)

    def candidate_context(self,result,current):
        require(result['valid'] and result['bank_id']==self.bank_id,'LOCALIZATION_RESULT','Unaccepted/wrong bank result')
        c=next(c for c in self.candidates if c.id==result['candidate'])
        return current.changed(model_id=c.model_id,model_hash=c.model['sha256'],feature_basis_id=self.bank_id)

    def verify_candidate(self,candidate_id,base):
        c=next(c for c in self.candidates if c.id==candidate_id)
        data=pinned(base,c.model)
        blob(data,1,(29,36),MODEL_BYTES,c.model_id)
        return True


def build_candidate(candidate_id,model_record,base,rows,kappa_coarse=.75,kappa_full=.25,position_m=None):
    data=pinned(base,model_record);mid,_=blob(data,1,(29,36),MODEL_BYTES)
    A=model_feature_matrix(data,rows)
    # One SVD supplies both nested cached ranks; no SVD on the per-frame path.
    u,s,_=np.linalg.svd(A,full_matrices=False)
    require(len(s)>0 and s[0]>0,'LOCALIZATION_MATRIX','Zero matrix')
    coarse=u[:,s>=kappa_coarse*s[0]];full=u[:,s>=kappa_full*s[0]]
    return Candidate(candidate_id,coarse,full,model_record,mid,position_m),A
