"""Board-side explicit SC03 capture/inspection/export, never executes a solver."""
from pathlib import Path
import argparse,hashlib,json,os,tarfile,time,uuid,shutil,io
from raw_contract import atomic_json
ROOT=Path('/home/root/ps_sc03_20261101')
def main():
    ap=argparse.ArgumentParser();ap.add_argument('action',choices=['status','capture','stop','export'])
    ap.add_argument('--frames',type=int,default=4);ap.add_argument('--pc-time',default=None)
    ap.add_argument('--job');ap.add_argument('--output');ap.add_argument('--include-raw',action='store_true')
    args=ap.parse_args();session=Path((ROOT/'current.txt').read_text().strip());state=json.loads((session/'state.json').read_text())
    command=json.loads((session/'capture_state.json').read_text())
    if args.action=='status':print(json.dumps({'session':str(session),'camera':state,'capture':command}));return
    if args.action in ('capture','stop'):
        boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        if state['boot_id']!=boot or state['state']!='LIVE' or not 0<=time.monotonic()-state['last_state_monotonic']<5:
            raise RuntimeError('No fresh live SC03 session')
        os.kill(state['pid'],0)
    if args.action=='stop':(session/'stop').touch();print('STOP_REQUESTED');return
    if args.action=='capture':
        if not 1<=args.frames<=64:raise ValueError('frames 1..64')
        if command['state']=='CAPTURING' or (session/'request.json').exists():raise RuntimeError('BUSY')
        req={'job_id':uuid.uuid4().hex,'boot_id':boot,'requested_monotonic':time.monotonic(),
             'frames':args.frames,'pc_request_time':args.pc_time,'mode':'RAW_ONLY'}
        active=json.loads((session/'camera_active.json').read_text())
        fps=(active['frame_rate']&0x3fffffff)/float(active['fps_unit'])
        guard=max(2.,2*active['exposure_us']/1e6+2/max(fps,.1))
        req.update(not_before_monotonic=req['requested_monotonic']+guard,freshness_guard_seconds=guard)
        atomic_json(session/'request.json',req);print(json.dumps(req));return
    if not args.output:raise ValueError('--output required for export')
    jobs=[]
    for p in sorted((session/'jobs').iterdir()):
        if args.job and p.name!=args.job:continue
        info=json.loads((p/'capture.json').read_text())
        if info['state']=='COMPLETE':jobs.append((p,info))
    if not jobs:raise ValueError('No completed jobs selected')
    output=Path(args.output)
    if output.exists() or Path(str(output)+'.partial').exists():raise FileExistsError(str(output))
    files=[]
    for job,info in jobs:
        for p in sorted(job.iterdir()):
            if p.suffix=='.raw' and not args.include_raw:continue
            if p.suffix not in ('.raw','.json','.npy','.jpg','.png'):continue
            if not args.include_raw and p.name=='mean_raw.npy':continue
            expected=next((r['sha256'] for r in info['frames'] if r['file']==p.name),None)
            expected=expected or info.get('derived_files',{}).get(p.name,{}).get('sha256')
            digest=hashlib.sha256(p.read_bytes()).hexdigest()
            if expected and digest!=expected:raise ValueError('Source integrity mismatch '+str(p))
            files.append({'path':p,'name':str(p.relative_to(session)),'bytes':p.stat().st_size,'sha256':digest})
    required=sum(r['bytes'] for r in files)+64*1024*1024
    output.parent.mkdir(parents=True,exist_ok=True)
    if shutil.disk_usage(str(output.parent)).free<required:raise RuntimeError('Export destination space too low')
    manifest={'schema_version':1,'kind':'full_raw_bundle' if args.include_raw else 'summary_bundle',
       'session':str(session),'boot_id':state['boot_id'],'raw_included':args.include_raw,
       'files':[{k:v for k,v in r.items() if k!='path'} for r in files],
       'raw_location':str(session/'jobs'),'raw_retained':True,
       'note':'summary bundle is not a replacement for original frames; full RAW remain on the board until verified off-device copy'}
    with tarfile.open(str(output)+'.partial','w:gz',compresslevel=1) as tf:
        for r in files:tf.add(str(r['path']),arcname=r['name'],recursive=False)
        b=json.dumps(manifest,indent=2).encode();ti=tarfile.TarInfo('bundle_manifest.json');ti.size=len(b);tf.addfile(ti,io.BytesIO(b))
    with tarfile.open(str(output)+'.partial','r:gz') as tf:
        for r in files:
            if hashlib.sha256(tf.extractfile(r['name']).read()).hexdigest()!=r['sha256']:raise ValueError('Archive readback mismatch')
    Path(str(output)+'.partial').rename(output)
    result={'file':str(output),'bytes':output.stat().st_size,'sha256':hashlib.sha256(output.read_bytes()).hexdigest(),
       'raw_included':args.include_raw,'jobs':len(jobs),'members':len(files),'archive_readback_verified':True}
    atomic_json(Path(str(output)+'.json'),result);print(json.dumps(result))
if __name__=='__main__':main()
