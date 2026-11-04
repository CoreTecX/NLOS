"""Reproducible host-only NB08 scenario matrix and NB10 ground-truth report."""
from support import *
import argparse,importlib.util
from detector import FastDetector
from contracts import write_json,InputError,FileSource
from motion_fixture import create_fixture,generate,similarity,frame_for,ROI
from raw_measurement import calibration_key,convert_raw
from registration import transform_points
from rectify import rectify_raw


def stats(a):
    return dict(count=len(a),p50_ms=float(np.percentile(a,50)),p95_ms=float(np.percentile(a,95)),max_ms=float(max(a)))


def fast_report(output,baseline):
    policy=read_json(REPO/'configs/nb08_nb10_acceptance.json')
    write_json(output/'frozen_acceptance.json',policy)
    spec=importlib.util.spec_from_file_location('nb08_baseline',baseline)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    classes=[('previous',module.FastDetector),('current',FastDetector)];rows=[];latencies={k:[] for k,_ in classes}
    for speed in policy['fast_speeds_pixels_per_frame']:
        for amplitude in policy['fast_amplitudes']:
            for sign in (-1,1):
                for seed in policy['seeds']:
                    frames=[];index=0;first=85 if sign<0 else 40
                    for phase,count in [('absent',4),('appear',4),('move',10),('stationary',6),('partial_occlusion',3),('stationary',4),('full_occlusion',4),('return',4),('leave',4)]:
                        for j in range(count):
                            x=None if phase in ('absent','full_occlusion','leave') else first+sign*speed*(j+1 if phase=='move' else (0 if phase=='appear' else 10))
                            a=amplitude*(.5 if phase=='partial_occlusion' else 1.)
                            frames.append((phase,measurement(index,x,amplitude=a,noise=.005,seed=seed+index,light=.2*np.sin(index/9))))
                            index+=1
                    for name,cls in classes:
                        det=cls(REFERENCE,STABLE,FAST);events=[];directions=[];steady=[];delays=[]
                        expected=[(4,'APPEARED'),(31,'DISAPPEARED'),(35,'APPEARED'),(39,'DISAPPEARED')]
                        for i,(phase,m) in enumerate(frames):
                            start=time.perf_counter_ns();e=det.process(m,now_ns=m.frame_complete_ns)
                            latencies[name].append((time.perf_counter_ns()-start)/1e6)
                            if e['transition'] in ('APPEARED','DISAPPEARED'):events.append((i,e['transition']))
                            if phase=='move':directions.append(e['direction'])
                            if phase in ('stationary','partial_occlusion'):steady.append(e['presence']=='present')
                        for when,kind in expected:
                            hits=[i-when for i,k in events if k==kind and 0<=i-when<=2]
                            if hits:delays.append(min(hits)*100)
                        expected_direction='right' if sign>0 else 'left';answered=[v for v in directions if v!='unknown']
                        rows.append(dict(implementation=name,speed=speed,amplitude=amplitude,seed=seed,direction=expected_direction,
                            events=events,event_hits=len(delays),expected_events=4,event_delay_ms=delays,
                            direction_answered=len(answered),direction_correct=sum(v==expected_direction for v in answered),
                            direction_frames=len(directions),stationary_and_partial_occlusion_preserved=all(steady)))
    negatives=[]
    for name,cls in classes:
        det=cls(REFERENCE,STABLE,FAST);false_events=0
        for i in range(1000):
            e=det.process(measurement(i,noise=.005,seed=9900+i,light=.5*np.sin(i/40)),now_ns=i*100_000_000)
            false_events+=e['transition']=='APPEARED'
        negatives.append(dict(implementation=name,samples=1000,simulated_seconds=100,false_events=int(false_events),physical_endurance=False))
    summary=[]
    for name,_ in classes:
        group=[r for r in rows if r['implementation']==name];expected=sum(r['expected_events'] for r in group)
        answered=sum(r['direction_answered'] for r in group);correct=sum(r['direction_correct'] for r in group)
        summary.append(dict(implementation=name,sequences=len(group),expected_events=expected,event_hits=sum(r['event_hits'] for r in group),
            event_recall=sum(r['event_hits'] for r in group)/expected,extra_events=sum(len(r['events'])-r['event_hits'] for r in group),
            direction_answered=answered,direction_correct=correct,direction_total=sum(r['direction_frames'] for r in group),
            direction_conditional_accuracy=correct/answered if answered else None,
            stationary_and_partial_occlusion_preserved=all(r['stationary_and_partial_occlusion_preserved'] for r in group),
            processing=stats(latencies[name])))
    current=summary[-1]
    passed=(current['event_recall']>=.95 and current['extra_events']==0 and current['stationary_and_partial_occlusion_preserved'] and
            current['direction_conditional_accuracy']>=.9 and negatives[-1]['false_events']==0 and current['processing']['p95_ms']<=100)
    result=dict(scope='SYNTHETIC_GAUSSIAN_SHADOWS_HOST_ONLY',passed=passed,summary=summary,negative=negatives,
        policy_sha256=sha((output/'frozen_acceptance.json').read_bytes()),baseline_sha256=sha(Path(baseline).read_bytes()),
        board_measured=False,confidence_is_probability=False,
        low_speed_note='0.2 pixel/frame lies below the registered 0.3 direction threshold; unknown direction is retained.',
        occlusion_note='Full occlusion and physical departure are observationally ambiguous; events describe visible measurement evidence only.')
    write_json(output/'fast_metrics.json',result);write_json(output/'fast_scenarios.json',rows)
    print(json.dumps(result,ensure_ascii=True),flush=True);assert passed


def motion_report(output):
    root=output/'motion_fixture';raw,desc,cal,anchor,contract=create_fixture(root);source=FileSource()
    f,d=frame_for(root,source,desc,raw,0)
    reference=decode_pfy(convert_raw(f,d,cal,root,D11['model']['sha256'])[0])[2];rows=[]
    rng=np.random.default_rng(719326)
    for i in range(24):
        h=similarity(*rng.uniform(-10,10,2),angle=rng.uniform(-2.4,2.4),scale=rng.uniform(.985,1.015))
        moved=generate(h);f,d=frame_for(root,source,desc,moved,i+1);start=time.perf_counter_ns()
        reg=anchor.register(moved,f.envelope['frame_uid'],f.envelope['source_session_id'],calibration_key(cal))
        pfy,prov=rectify_raw(f,d,cal,root,D11['model']['sha256'],reg,contract)
        elapsed=(time.perf_counter_ns()-start)/1e6
        pts=np.array([[x,y] for x in np.linspace(130,626,11) for y in np.linspace(130,626,11)])
        err=np.linalg.norm(transform_points(np.linalg.inv(reg.current_to_reference),pts)-transform_points(h,pts),axis=1)/4
        uncompensated=decode_pfy(convert_raw(f,d,cal,root,D11['model']['sha256'])[0])[2]
        relative=float(np.linalg.norm(decode_pfy(pfy)[2]-reference)/np.linalg.norm(reference))
        rows.append(dict(transform=h.tolist(),true_control_p95_grid_pixels=float(np.percentile(err,95)),relative_measurement_error=relative,
            uncompensated_relative_error=float(np.linalg.norm(uncompensated-reference)/np.linalg.norm(reference)),processing_ms=elapsed,
            passed=bool(relative<.01 and np.percentile(err,95)<.5),diagnostics=reg.diagnostics))
    negatives=[]
    center=np.array([[1,0,378],[0,1,378],[0,0,1.]])
    cases=[('perspective',generate(center@np.array([[1,0,0],[0,1,0],[.00015,0,1.]])@np.linalg.inv(center))),
           ('translation_out_of_range',generate(similarity(45))),('rotation_out_of_range',generate(similarity(angle=4))),
           ('reference_lost',np.full_like(raw,900))]
    for i,(name,moved) in enumerate(cases):
        try:
            reg=anchor.register(moved,'negative-'+str(i),'negative-session',calibration_key(cal))
            negatives.append(dict(case=name,rejected=False))
        except InputError as e:negatives.append(dict(case=name,rejected=True,reason=e.code))
    report=dict(scope='SYNTHETIC_ANALYTIC_WALL',passed=all(r['passed'] for r in rows) and all(r['rejected'] for r in negatives),
        accepted=len(rows),max_true_control_p95_grid_pixels=max(r['true_control_p95_grid_pixels'] for r in rows),
        max_relative_measurement_error=max(r['relative_measurement_error'] for r in rows),
        processing=stats([r['processing_ms'] for r in rows]),transforms=rows,negative_cases=negatives,
        physical_forward_equivalence_measured=False,physical_motion_range_measured=False,board_measured=False)
    write_json(output/'motion_metrics.json',report)
    print(json.dumps({k:v for k,v in report.items() if k!='transforms'},ensure_ascii=True),flush=True);assert report['passed']


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--baseline',type=Path,required=True);args=p.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    fast_report(args.output,args.baseline);motion_report(args.output)
