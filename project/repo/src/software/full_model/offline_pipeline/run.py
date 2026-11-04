"""Offline CPU full reconstruction and registered RAW/PFY input pipeline. No hardware I/O."""
from pathlib import Path
import argparse
from datetime import datetime
import html
import importlib.util
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time

REPO = Path(__file__).resolve().parents[4]
sys.path.insert(0,str(REPO/'src/software/camera_service/production_input'))
sys.path.insert(0,str(REPO/'src/software/meas_norm'))
for key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ.setdefault(key,'1')
import numpy as np
from PIL import Image

from contracts import InputError, FileSource, require, pinned, read_json, write_json, sha, load_model, decode_pfy, blob, resolve
from inputs import release_info, release_file, historical_case, historical_raw
from raw_measurement import convert_raw, diagnostics
from native_backend import within, prepare_native


def import_checked(path, data, name):
    module = importlib.util.module_from_spec(importlib.util.spec_from_loader(name,loader=None))
    module.__file__ = str(path)
    exec(compile(data,str(path),'exec'),module.__dict__)
    return module


def images(output, model_id):
    _, payload = blob((output/'result.pfo').read_bytes(),4,(29,36),75232,model_id)
    arrays = np.frombuffer(payload,'<f8').reshape(3,29,36,3)
    require(np.isfinite(arrays).all(),'NONFINITE_RESULT','Nonfinite reconstruction')
    np.save(output/'reconstruction_normalized.npy',arrays[1],allow_pickle=False)
    np.save(output/'reconstruction_physical.npy',arrays[2],allow_pickle=False)
    with Image.open(output/'result.ppm') as im:
        im.save(output/'reconstruction_native.png')
        im.resize((576,464),getattr(Image, 'Resampling', Image).NEAREST).save(output/'reconstruction_large.png')


def execute(args, resources=None, input_snapshot=None):
    config = read_json(args.config)
    workspace = Path(args.workspace or config['workspace_hint']).resolve()
    output = Path(args.output).resolve()
    require(not output.exists(),'OUTPUT_EXISTS','Choose a new output directory')
    if within(output, workspace):
        require(within(output, workspace/'99_临时工作区'), 'OUTPUT_LOCATION', 'Runtime outputs in workspace must be under 99_临时工作区')
    output.mkdir(parents=True)
    started = time.perf_counter()
    summary = dict(schema_version=1,status='RUNNING',hardware_accessed=False,algorithm_executed=False,
        solver_changed=False,full_dimension=1044,arithmetic='FP64',iteration_limit=20000,
        started_at=datetime.now().astimezone().isoformat(),board_performance=None,physical_quality_accepted=False)
    events = [];stages={};stage_started=time.perf_counter()
    def stage(name):
        nonlocal stage_started
        now=time.perf_counter();stages[name]=(now-stage_started)*1000;stage_started=now
    def event(name, **data):
        item = dict(event=name,host_elapsed_ms=(time.perf_counter()-started)*1000,**data)
        events.append(item)
        with (output/'events.jsonl').open('a',encoding='utf8') as stream:
            stream.write(json.dumps(item,ensure_ascii=False,allow_nan=False)+'\n')
    try:
        row = None
        if input_snapshot is not None:
            desc,base=input_snapshot
        elif args.case:
            desc,base,row = historical_case(config,workspace,args.case)
        elif args.raw_sample:
            desc,base = historical_raw(config,workspace,args.raw_sample),workspace
        else:
            desc,base = read_json(args.input),Path(args.input).resolve().parent
        frame = FileSource().load(desc,base)
        write_json(output/'frame.json',frame.envelope)
        write_json(output/'input_descriptor.json',desc)
        event('INPUT_VALIDATED',kind=frame.kind,sha256=sha(frame.data))
        stage('input_validation_ms')
        if args.action=='inspect':
            summary.update(status='INSPECTED',input_kind=frame.kind,measurement_ready=frame.kind=='PFY',
                           calibration_required=frame.kind=='RAW')
            return summary
        if frame.kind=='RAW':
            require(desc.get('calibration') is not None,'MISSING_CALIBRATION',
                    'RAW lacks registered geometry/radiometry/model; preview is not a reconstruction input')
        model,model_id = load_model(desc,base) if resources is None else resources.model(desc,base)
        model_hash=desc['model']['sha256']  # Verified either by load_model or immutable snapshot admission.
        stage('model_load_ms')
        if frame.kind=='RAW':
            cbytes = pinned(base,desc['calibration'])
            cal = json.loads(cbytes)
            calbase = resolve(base,desc['calibration']['path']).parent
            measurement,context,difference = convert_raw(frame,desc,cal,calbase,model_hash)
        else:
            measurement = frame.data
            _,scales,rgb = decode_pfy(measurement,model_id)
            diag,difference = diagnostics(rgb)
            context = dict(model_id=model_id,model_hash=model_hash,measurement_id=sha(measurement),
                measurement_hash=sha(measurement),source_frame_uids=[frame.envelope['frame_uid']],k=1,
                calibration_scope='HISTORICAL_MATCHED_PFY' if row else 'REGISTERED_PFY',
                operator_id='FROZEN_FULL_MODEL',diagnostics=diag,physical_quality_accepted=False)
        (output/'measurement.pfy').write_bytes(measurement)
        np.save(output/'normalized_spatial_difference.npy',difference,allow_pickle=False)
        write_json(output/'measurement.json',context)
        summary.update(input_kind=frame.kind,measurement_sha256=sha(measurement),model_sha256=model_hash,
                       model_id=model_id,calibration_scope=context['calibration_scope'])
        event('MEASUREMENT_READY',sha256=sha(measurement))
        stage('measurement_prepare_save_ms')
        if args.action=='prepare':
            summary['status']='MEASUREMENT_READY'
            return summary
        try:
            from scipy.fft import dctn, idctn
        except ImportError as error:
            raise InputError('RUNTIME_DEPENDENCY', 'Independent state audit needs scipy.fft: '+str(error))
        release,manifest = release_info(config,workspace)
        runtime_root = Path(args.runtime_root or config['ascii_runtime_root']).resolve()
        require(str(runtime_root).isascii(),'ASCII_RUNTIME','Native solver requires an ASCII runtime root')
        runtime_root.mkdir(parents=True,exist_ok=True)
        runtime = Path(tempfile.mkdtemp(prefix='run_',dir=runtime_root))
        launch = prepare_native(config, workspace, release, manifest, runtime, resources=resources)
        exe, dll_hashes = launch['binary'], launch['libraries']
        (runtime/'model.pfm').write_bytes(model)
        (runtime/'measurement.pfy').write_bytes(measurement)
        del model
        command, env = launch['command'], launch['env']
        summary.update(algorithm_executed=True,runtime_directory=str(runtime),command=command,
            software_revision=manifest['software_revision'],solver_sha256=manifest['files'][exe]['sha256'],host_libraries=dll_hashes,
            runtime_target=launch['target'],backend=launch['backend'],hardware_accessed=launch['hardware_accessed'])
        event('SOLVER_STARTED',backend=launch['backend'],limit=20000)
        stage('runtime_prepare_ms')
        with (output/'solver.log').open('wb') as log:
            result = subprocess.run(command,cwd=runtime,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=args.timeout)
        stage('native_process_ms')
        summary['solver_exit_code']=result.returncode
        for suffix in ('.json','.pfo','.pfs','.ppm'):
            src = runtime/('result'+suffix)
            if src.is_file():
                shutil.copy2(src,output/src.name)
        require(result.returncode==0,'SOLVER_FAILED','Full solver failed or did not converge; see solver.log')
        stage('result_copy_ms')
        get_release=release_file if resources is None else resources.release
        state_bytes = get_release(release,manifest,'scripts/accept_state.py')
        auditor = import_checked(release/'scripts/accept_state.py',state_bytes,'_frozen_state_auditor')
        state = auditor.evaluate(runtime/'model.pfm',runtime/'measurement.pfy',output/'result')
        write_json(output/'state_acceptance.json',state)
        require(state['passed'],'STATE_ACCEPTANCE','Independent FP64 residual/update audit failed')
        stage('independent_state_audit_ms')
        gold = None
        if desc.get('golden'):
            reference = pinned(base,desc['golden'])
            blob(reference,3,(29,36),75232,model_id)
            audit = import_checked(release/'scripts/accept_board.py',get_release(release,manifest,'scripts/accept_board.py'),'_frozen_numeric_auditor')
            # Reference is consumed only after inference, in an isolated audit directory.
            audit_root=output/'reference_audit'
            (audit_root/'data').mkdir(parents=True)
            (audit_root/'data/reference.pfg').write_bytes(reference)
            audit.ROOT=audit_root
            gold=audit.evaluate(dict(key=desc['sample_id'],model_id=model_id,golden={'file':'reference.pfg'}),output/'result.pfo')
            write_json(output/'reference_acceptance.json',gold)
            require(gold['passed'],'REFERENCE_ACCEPTANCE','Frozen independent reference check failed')
        stage('independent_reference_audit_ms')
        images(output,model_id)
        result_json=read_json(output/'result.json')
        summary.update(status='COMPLETE',state_acceptance=True,reference_acceptance=gold['passed'] if gold else None,
            numeric_scope='FROZEN_REFERENCE_AND_INDEPENDENT_STATE' if gold else 'INDEPENDENT_STATE_ONLY_NO_GOLDEN',
            iterations=result_json['iterations'],seconds_solver=result_json['seconds_solver'],
            result_shape=[29,36,3],new_reconstruction_images=1)
        event('RECONSTRUCTION_ACCEPTED',iterations=result_json['iterations'])
        title=html.escape(desc['sample_id'])
        (output/'index.html').write_text('<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>'+title+
            '</title><style>body{font:18px sans-serif;background:#13202c;color:#eff6ff;margin:36px}img{image-rendering:pixelated;max-width:100%}a{color:#8bd2ff}</style>'+
            '<h1>'+title+' · 完整算法重建</h1><p>'+html.escape(launch['target']+' / '+launch['backend'])+' · 1044 维 FP64 · '+str(result_json['iterations'])+
            ' 次迭代</p><img src="reconstruction_large.png"><p>显示图作统一亮度映射；原始数值保存在 PFO 和 NPY。</p>'+
            '<p>数值验收范围：'+summary['numeric_scope']+'。真实相机成像质量未验收。</p>'+
            '<p><a href="summary.json">运行记录</a> · <a href="state_acceptance.json">独立状态验收</a></p></html>','utf8')
        stage('render_and_save_ms')
        return summary
    except (InputError,OSError,ValueError,KeyError,TypeError,subprocess.TimeoutExpired) as error:
        code = error.code if isinstance(error,InputError) else ('SOLVER_TIMEOUT' if isinstance(error,subprocess.TimeoutExpired) else type(error).__name__)
        summary.update(status='BLOCKED' if code=='MISSING_CALIBRATION' else 'FAILED',error_code=code,error=str(error))
        event('STOPPED',code=code,error=str(error))
        return summary
    finally:
        stage('remaining_or_failed_stage_ms')
        summary['output_files']={p.relative_to(output).as_posix():sha(p.read_bytes()) for p in output.rglob('*') if p.is_file()}
        stage('artifact_hash_ms')
        summary['stages_ms']=stages
        summary['resources']=resources.status() if resources is not None else dict(enabled=False)
        summary['elapsed_host_seconds']=time.perf_counter()-started
        summary['finished_at']=datetime.now().astimezone().isoformat()
        write_json(output/'summary.json',summary)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=Path(__file__).resolve().parents[4]/'configs/offline_engineering.json')
    p.add_argument('--workspace',type=Path)
    source=p.add_mutually_exclusive_group(required=True)
    source.add_argument('--case');source.add_argument('--raw-sample');source.add_argument('--input',type=Path)
    p.add_argument('--action',choices=['inspect','prepare','reconstruct'],default='reconstruct')
    p.add_argument('--output',type=Path,required=True);p.add_argument('--runtime-root',type=Path)
    p.add_argument('--timeout',type=float,default=1800)
    args=p.parse_args()
    p.error('timeout must be finite and positive') if not np.isfinite(args.timeout) or args.timeout<=0 else None
    try:
        summary=execute(args)
    except (OSError,InputError,ValueError) as error:
        print(json.dumps(dict(status='FAILED',error=str(error)),ensure_ascii=False));return 2
    print(json.dumps(summary,ensure_ascii=False,indent=2,allow_nan=False))
    return 0 if summary['status'] in ('COMPLETE','INSPECTED','MEASUREMENT_READY') else 2


if __name__=='__main__':
    raise SystemExit(main())
