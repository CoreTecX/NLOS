"""Manifest-driven offline runner. No camera, serial port, network or board access."""
from pathlib import Path
import argparse,json,sys,time,io
REPO=Path(__file__).resolve().parents[4]
for folder in ('src/software/camera_service/production_input','src/software/meas_norm','src/software/full_model/fast_path',
               'src/software/meas_norm/camera_motion','src/software/model_loader/localization','src/software/meas_norm/temporal'):
    sys.path.insert(0,str(REPO/folder))
import numpy as np
from contracts import read_json,write_json,pinned,require,resolve
from messages import Context
from controller import Controller
from backend import FrozenCpuBackend
from session import OfflineSession,MotionAdapter
from storage import load_bank,build_bank


def execute(manifest,output,workspace,config=None):
    manifest=Path(manifest);spec=read_json(manifest);base=manifest.parent;output=Path(output)
    require(spec['schema_version']==1 and spec['scope']=='SYNTHETIC_TEST_ONLY','SEQUENCE_SCOPE',
            'This offline runner accepts explicitly labeled synthetic sequences; real replay timing/calibration is not registered yet')
    require(0<len(spec['frames'])<=100000,'SEQUENCE_LENGTH','Bounded nonempty frame list required')
    output.mkdir(parents=True,exist_ok=False);ctx=Context.create(**spec['context']);bindings={}
    for b in spec['bindings']:
        with np.load(io.BytesIO(pinned(base,b['reference_arrays'])),allow_pickle=False) as z:
            value=dict(context=b['context'],reference=z['reference'],stable_mask=z['stable_mask'],fast_policy=b['fast_policy'])
        if b.get('motion'):
            motion=b['motion'];cal=json.loads(pinned(base,motion['calibration']));contract=json.loads(pinned(base,motion['contract']))
            with np.load(io.BytesIO(pinned(base,motion['anchor_arrays'])),allow_pickle=False) as z:
                value['motion']=MotionAdapter(z['raw'],z['stable_mask'],cal,contract,motion['policy'],resolve(base,motion['calibration']['path']).parent)
        bindings[b['context']['model_hash']]=value
    backend=FrozenCpuBackend(REPO,Path(config) if config is not None else REPO/'configs/offline_engineering.json',workspace,output/'images',spec['models'])
    ctrl=Controller(ctx,backend,spec['scheduler_policy'])
    bank=load_bank(base/spec['bank'],workspace) if spec.get('bank') else None
    session=OfflineSession(ctrl,bindings,workspace,bank,temporal_config=spec.get('temporal_config'))
    started=time.monotonic_ns();count=0
    try:
        with (output/'events.jsonl').open('x',encoding='utf8') as stream:
            for record in spec['frames']:
                target=started+int(record['at_ms']*1e6)
                while time.monotonic_ns()<target:
                    ctrl.poll();time.sleep(min(.01,(target-time.monotonic_ns())/1e9) if target>time.monotonic_ns() else 0)
                d=json.loads(pinned(base,record['descriptor']))
                require(d.get('origin')=='SYNTHETIC','SEQUENCE_SCOPE','Every frame must be labeled synthetic')
                item=session.ingest(d,base,int(record['sample_time_ns']))
                for command in record.get('commands',[]):ctrl.command(**command)
                ctrl.poll()
                if record.get('localize') and item['status']=='MEASUREMENT_ACCEPTED':
                    item['localization']=session.localize(session.last_measurement)
                if record.get('image') and item['status']=='MEASUREMENT_ACCEPTED':
                    item['image_ack']=ctrl.request_image(session.last_measurement,request_id=record.get('request_id'))
                stream.write(json.dumps(item,allow_nan=False)+'\n');stream.flush();count+=1
            while ctrl.future is not None or ctrl.pending is not None:
                ctrl.poll();time.sleep(.01)
        result=dict(scope=spec['scope'],frames=count,accepted=session.accepted,rejected=session.rejected,
                    faulted=ctrl.faulted,results=list(ctrl.results),audit=list(ctrl.audit.rows),audit_dropped=ctrl.audit.dropped,
                    failed_image_results=sum(not row['valid_numeric'] for row in ctrl.results),
                    physical_quality_accepted=False,hardware_accessed=False)
        write_json(output/'summary.json',result)
        return result
    finally:ctrl.close()


def main():
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='action',required=True)
    run=sub.add_parser('run');run.add_argument('--manifest',required=True,type=Path);run.add_argument('--output',required=True,type=Path)
    run.add_argument('--workspace',required=True,type=Path)
    run.add_argument('--config',type=Path,help='Explicit CPU runtime configuration, including Linux candidate paths')
    bank=sub.add_parser('build-bank');bank.add_argument('--spec',required=True,type=Path);bank.add_argument('--output',required=True,type=Path)
    bank.add_argument('--workspace',required=True,type=Path)
    a=p.parse_args()
    if a.action=='build-bank':result=dict(bank_id=build_bank(a.spec,a.output,a.workspace).bank_id)
    else:result=execute(a.manifest,a.output,a.workspace,a.config)
    print(json.dumps(result,ensure_ascii=False,allow_nan=False))
    return int(result.get('faulted',False) or result.get('rejected',0)>0 or result.get('failed_image_results',0)>0)

if __name__=='__main__':sys.exit(main())
