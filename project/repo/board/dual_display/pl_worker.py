"""Real board PL reconstruction of fresh camera frames; no CPU fallback."""
from pathlib import Path
import os,sys,json,hashlib,struct,zlib,fcntl,time,traceback
import numpy as np
import cv2
ROOT=Path('/home/root/ps_dualview_20261030')
RELEASE=Path('/home/root/penumbrasight_app/slots/20261024-nb18-rc1/releases/performance')
def atomic(p,obj):
    t=p.with_suffix('.tmp');t.write_text(json.dumps(obj,indent=2));t.replace(p)
def main(job):
    record=json.loads((job/'input.json').read_text());record.update(state='RUNNING',backend='pl',model_matches_scene=False,physical_calibration_passed=False)
    atomic(job/'status.json',record)
    sys.path.insert(0,str(RELEASE/'scripts'));import run_target as launch
    row=next(r for r in json.loads((RELEASE/'data/manifest.json').read_text()) if r['key']=='D11_rgb')
    lock=open('/var/lock/psight-full-launch.lock','a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    manifest=launch.preflight('pl',[row]);launch.dct_selftest(job)
    a=np.load(str(job/'mean_raw.npy'),allow_pickle=False)
    assert a.shape==(512,612) and np.isfinite(a).all()
    # Same diagnostic center ROI as previous camera runs, not a calibrated wall ROI.
    a=a[4:508,54:558];r=a[0::2,0::2];g=(a[0::2,1::2]+a[1::2,0::2])*.5;b=a[1::2,1::2]
    rgb=np.stack([r,g,b],axis=2).reshape(126,2,126,2,3).mean(axis=(1,3))
    payload=np.ones(3,dtype='<f8').tobytes()+np.asarray(rgb.reshape(-1,3,order='F'),dtype='<f8').tobytes()
    header=struct.pack('<8sIIIIQII16s8s',b'PSFULL1\0',1,2,126,126,len(payload),zlib.crc32(payload),0,bytes.fromhex(row['model_id']),b'\0'*8)
    measurement=job/'measurement.pfy';measurement.write_bytes(header+payload)
    record.update(measurement_sha256=hashlib.sha256(header+payload).hexdigest(),roi=[54,4,504,504],rgb_scales=[1,1,1],model_case='D11_rgb',model_id=row['model_id'])
    atomic(job/'status.json',record)
    ready=json.loads((ROOT/'pl_ready.json').read_text())
    assert ready['boot_id']==Path('/proc/sys/kernel/random/boot_id').read_text().strip(),'Initialize FPGA before starting camera'
    assert ready['bit_sha256']==manifest['files']['bin/full_board_fan_safe.bit']['sha256']
    assert Path('/sys/class/fpga_manager/fpga0/state').read_text().strip()=='operating'
    # Reprogramming PL while GigE is live disrupts camera ownership on this board.
    # Initialize once before camera acquisition; never reload per button press.
    oldlock=None
    env=os.environ.copy();env.update(PS_FULL_LOADED=launch.TOKEN,PS_PL_SCHEDULE='overlap',PS_EXPORT_STATE='1')
    prefix=job/'result';start=time.monotonic()
    with (job/'solver.log').open('w') as log:
        proc=launch.run_case([str(RELEASE/'bin/ps_full_aarch64'),str(RELEASE/'data'/row['model']['file']),str(measurement),str(prefix),'pl'],log,env,'pl')
    if proc.returncode not in (0,3):raise RuntimeError('PL solver exit %s; inspect solver.log'%proc.returncode)
    export(job,row,record,time.monotonic()-start,proc.returncode)
    if oldlock:oldlock.close()
    lock.close()
def export(job,row,record,seconds,returncode):
    prefix=job/'result'
    data=prefix.with_suffix('.pfo').read_bytes()
    assert len(data)==75232 and data[:8]==b'PSFULL1\0' and struct.unpack_from('<IIII',data,8)==(1,4,29,36)
    assert struct.unpack_from('<Q',data,24)[0]==len(data)-64 and data[40:56].hex()==row['model_id']
    assert zlib.crc32(data[64:])==struct.unpack_from('<I',data,32)[0]
    arrays=np.frombuffer(data[64:],'<f8').reshape(3,29,36,3);assert np.isfinite(arrays).all()
    f=arrays[1];np.save(str(job/'reconstruction_normalized.npy'),f)
    lo=float(f.min());hi=float(f.max());image=np.uint8(np.clip((f-lo)/max(hi-lo,1e-30)*255,0,255))
    assert cv2.imwrite(str(job/'reconstruction.png'),cv2.cvtColor(image,cv2.COLOR_RGB2BGR))
    solver=json.loads(prefix.with_suffix('.json').read_text())
    assert solver.get('backend')=='pl','Backend mismatch: refusing to label CPU result as FPGA'
    assert (returncode==0 and solver.get('converged') is True) or (returncode==3 and solver.get('converged') is False and solver.get('iterations')==20000)
    record.update(state='COMPLETE' if solver['converged'] else 'NONCONVERGED',seconds=seconds,iterations=solver.get('iterations'),solver=solver,solver_exit_code=returncode,result_sha256=hashlib.sha256(data).hexdigest())
    atomic(job/'status.json',record)
if __name__=='__main__':
    job=Path(sys.argv[1])
    try:main(job)
    except BaseException:
        atomic(job/'status.json',dict(state='ERROR',job_id=job.name,backend='pl',error=traceback.format_exc(),physical_calibration_passed=False));raise
