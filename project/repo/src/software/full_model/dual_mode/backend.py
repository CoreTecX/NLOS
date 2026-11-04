"""Actual complete CPU solver backend for the bounded image owner."""
from pathlib import Path
from types import SimpleNamespace
import importlib.util,json,threading,time,copy
from collections import OrderedDict
from contracts import read_json,write_json,sha,require,load_model
from resources import VerifiedResources


class FrozenCpuBackend:
    def __init__(self,repo,config,workspace,output_root,models):
        self.repo=Path(repo);self.config=Path(config);self.workspace=Path(workspace);self.root=Path(output_root)
        self.root.mkdir(parents=True,exist_ok=False)
        config_value=read_json(self.config)
        self.resources=VerifiedResources(config_value.get('resource_cache_bytes',64*1024*1024)) if config_value.get('resource_cache_enabled',False) else None
        self.artifacts=OrderedDict();self.artifact_lock=threading.Lock();self.artifacts_replaced=0
        self.models={};self.measurement_descriptors={}
        for descriptor in models:
            data,mid=load_model(descriptor,self.workspace) if self.resources is None else self.resources.model(descriptor,self.workspace)
            self.models[descriptor['model']['sha256']]=copy.deepcopy(descriptor)
            self.measurement_descriptors[(descriptor['model']['sha256'],descriptor['file']['sha256'])]=copy.deepcopy(descriptor)
        path=self.repo/'src/software/full_model/offline_pipeline/run.py'
        spec=importlib.util.spec_from_file_location('_complete_offline_solver',path)
        self.pipeline=importlib.util.module_from_spec(spec);spec.loader.exec_module(self.pipeline)
        self.completed=0

    def __call__(self,request):
        began=time.perf_counter_ns()
        m=request.measurement
        require(m.context.data['model_hash'] in self.models,'MODEL_NOT_REGISTERED','Model not registered in CPU backend')
        d=dict(self.measurement_descriptors.get((m.context.data['model_hash'],m.sha256),self.models[m.context.data['model_hash']]))
        require(d['model_id']==m.context.data['model_id'],'MODEL_ID','Context/model mismatch')
        folder=self.root/('job_'+request.id)
        require(folder.resolve().parent==self.root.resolve(),'REQUEST_ID','Invalid request identifier')
        folder.mkdir()
        path=folder/'measurement.pfy';path.write_bytes(m.pfy)
        if d['file']['sha256']!=m.sha256:d.pop('golden',None)
        d.update(file=dict(path=str(path),bytes=len(m.pfy),sha256=m.sha256),
                 kind='PFY',sample_id=m.frame_uid,origin=m.source_kind)
        write_json(folder/'input.json',d)
        write_json(folder/'request.json',dict(id=request.id,origin=request.origin,measurement=m.identity,
                    policy_epoch=str(request.policy_epoch),requested_ns=str(request.requested_ns)))
        result=self.pipeline.execute(SimpleNamespace(config=self.config,workspace=self.workspace,output=folder/'result',
                  case=None,raw_sample=None,input=folder/'input.json',action='reconstruct',runtime_root=None,timeout=1800),resources=self.resources)
        artifact_start=time.perf_counter_ns();bundle=[];total=0
        if result['status']=='COMPLETE':
            # Files are read on the complete-solve owner, never on FAST/UI owner.
            for name in ('reconstruction_native.png','reconstruction_normalized.npy','reconstruction_physical.npy','measurement.pfy','summary.json','state_acceptance.json'):
                file=folder/'result'/name
                if file.is_file():
                    require(total+file.stat().st_size<=16*1024*1024,'ARTIFACT_BUDGET','Result snapshot exceeds 16 MiB')
                    value=file.read_bytes();total+=len(value)
                    require(total<=16*1024*1024,'ARTIFACT_BUDGET','Result snapshot grew while reading')
                    bundle.append(('image_'+name,value))
        with self.artifact_lock:
            self.artifacts[request.id]=tuple(bundle)
            while len(self.artifacts)>2:self.artifacts.popitem(last=False);self.artifacts_replaced+=1
        self.completed+=1
        return dict(status=result['status'],state_acceptance=result.get('state_acceptance',False),
                    reference_acceptance=result.get('reference_acceptance'),output_directory=str(folder/'result'),
                    iterations=result.get('iterations'),seconds_solver=result.get('seconds_solver'),
                    measurement_sha256=m.sha256,model_hash=m.context.data['model_hash'],stages_ms=result.get('stages_ms'),
                    resource_cache=self.resources.status() if self.resources else dict(enabled=False),
                    artifact_read_ms=(time.perf_counter_ns()-artifact_start)/1e6,worker_total_ms=(time.perf_counter_ns()-began)/1e6)

    def take_artifacts(self,request_id):
        with self.artifact_lock:return self.artifacts.pop(request_id,())

    def close(self):
        if self.resources:self.resources.clear()
        with self.artifact_lock:self.artifacts.clear()
