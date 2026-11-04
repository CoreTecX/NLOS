"""Reproducible synthetic event metrics and actual frozen-solver coexistence."""
from support import *
import argparse,csv,hashlib,subprocess,os
from detector import FastDetector
from controller import Controller
from backend import FrozenCpuBackend
from contracts import pinned,write_json


def stats(values):
    a=np.asarray(values,float)
    return dict(count=len(a),p50_ms=float(np.percentile(a,50)),p95_ms=float(np.percentile(a,95)),max_ms=float(a.max()))


def fast_metrics(output):
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    write_json(output/'policy_frozen_before_evaluation.json',dict(fast=FAST,scheduler=POLICY,
        seed=950191,scope='SYNTHETIC_TEST_ONLY',physical_metrics_accepted=False))
    policy_hash=sha((output/'policy_frozen_before_evaluation.json').read_bytes())
    rng=np.random.default_rng(950191);det=FastDetector(REFERENCE,STABLE,FAST);idx=0;latencies=[];expected=[];observed=[];directions=[]
    def feed(x=None,**kw):
        nonlocal idx
        now=idx*100_000_000;m=measurement(idx,x,now=now,**kw);start=time.perf_counter_ns()
        r=det.process(m,now_ns=now);latencies.append((time.perf_counter_ns()-start)/1e6)
        if r.get('transition') in ('APPEARED','DISAPPEARED'):observed.append(dict(event=r['transition'],at_ms=idx*100))
        idx+=1;return r
    for _ in range(4):feed()
    for trial in range(120):
        right=trial%2==0;center=float(rng.uniform(35,55) if right else rng.uniform(70,90));amplitude=float(rng.uniform(.6,1.8))
        expected.append(dict(event='APPEARED',at_ms=idx*100))
        for _ in range(4):feed(center,amplitude=amplitude,noise=.005,seed=int(rng.integers(1<<30)))
        votes=[]
        for j in range(1,7):
            r=feed(center+(j if right else -j)*1.6,amplitude=amplitude,noise=.005,seed=int(rng.integers(1<<30)))
            votes.append(r['direction'])
        answered=[v for v in votes if v!='unknown'];prediction=max(set(answered),key=answered.count) if answered else 'unknown'
        directions.append(dict(expected='right' if right else 'left',predicted=prediction,unknown_frames=votes.count('unknown'),total_frames=len(votes)))
        for _ in range(12):
            r=feed(center+(6 if right else -6)*1.6,amplitude=amplitude)
            assert r['presence']=='present','Stationary target was erased'
        expected.append(dict(event='DISAPPEARED',at_ms=idx*100))
        for _ in range(5):feed(noise=.005,seed=int(rng.integers(1<<30)))
    matched=set();delays=[];counts={}
    for kind in ('APPEARED','DISAPPEARED'):
        labels=[x for x in expected if x['event']==kind];hits=0
        for label in labels:
            candidates=[(i,o) for i,o in enumerate(observed) if i not in matched and o['event']==kind and 0<=o['at_ms']-label['at_ms']<=300]
            if candidates:
                i,o=candidates[0];matched.add(i);hits+=1;delays.append(o['at_ms']-label['at_ms'])
        counts[kind]=dict(expected=len(labels),matched=hits,recall=hits/len(labels))
    negative_start=idx;before=len(observed)
    # 18,000 samples at 100 ms timestamps = 30 minutes of simulated negatives.
    # No claim of 30-minute physical acquisition or wall-clock endurance.
    for j in range(18000):
        feed(noise=.003,seed=int(rng.integers(1<<30)),light=.2*np.sin(j/200))
    negatives=len(observed)-before;answered=sum(d['predicted']!='unknown' for d in directions)
    correct=sum(d['expected']==d['predicted'] for d in directions)
    report=dict(scope='SYNTHETIC_GAUSSIAN_SHADOW_SEQUENCES',policy_sha256=policy_hash,
        events=counts,unmatched_extra_events=len(observed[:before])-len(matched),event_delay=stats(delays),
        direction=dict(total_events=len(directions),answered=answered,correct=correct,coverage=answered/len(directions),
                       conditional_accuracy=correct/answered if answered else None,total_accuracy=correct/len(directions)),
        negative=dict(samples=18000,simulated_duration_seconds=1800,wall_clock_duration_claimed=False,false_events=negatives,
                      false_events_per_simulated_minute=negatives/30),fast_processing=stats(latencies),
        physical_event_metrics_accepted=False,board_latency_measured=False)
    report['passed']=all(v['recall']>=.95 for v in counts.values()) and negatives/30<=1 and correct/len(directions)>=.9 and report['unmatched_extra_events']==0
    write_json(output/'metrics.json',report);write_json(output/'event_labels.json',expected);write_json(output/'observed_events.json',observed)
    write_json(output/'direction_events.json',directions)
    print(json.dumps(report,indent=2));assert report['passed']
    return report


def coexistence(output):
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    data=pinned(WORKSPACE,D11['file']);rgb=decode_pfy(data)[2];ctx=context()
    fast=dict(FAST,radiometric_normalizer=max(float(np.mean(rgb)),1e-3),ttl_ms=1000)
    det=FastDetector(rgb,STABLE,fast)
    backend=FrozenCpuBackend(REPO,REPO/'configs/offline_engineering.json',WORKSPACE,output/'images',[D11])
    ctrl=Controller(ctx,backend,dict(POLICY,capture_gap_ms=1000,audit_capacity=4096))
    timings=[];updates=[];first_future=None;mode_survived=False
    start=time.monotonic_ns();i=0;requested=False
    try:
        while True:
            target=start+i*100_000_000
            while time.monotonic_ns()<target:time.sleep(min(.005,max(0,(target-time.monotonic_ns())/1e9)))
            now=time.monotonic_ns();m=measurement(i,ctx=ctrl.context,now=now,pfy=data)
            began=time.perf_counter_ns();event=ctrl.ingest(m,det);elapsed=(time.perf_counter_ns()-began)/1e6
            if requested and ctrl.future is not None:timings.append(elapsed);updates.append(dict(sequence=i,at_ns=str(now),valid=event['valid']))
            if i==4:
                ctrl.command('mode-image','metric',1,'MANUAL','IMAGE');ctrl.poll()
                ack=ctrl.request_image(m,request_id='D11_actual_complete');assert ack['outcome']=='RUNNING';first_future=ctrl.future
                ctrl.command('mode-fast','metric',2,'MANUAL','FAST');ctrl.poll()
                mode_survived=ctrl.future is first_future;requested=True
            if requested and ctrl.future is None:break
            if time.monotonic_ns()-start>180*1e9:raise RuntimeError('Coexistence deadline exceeded')
            i+=1
        result=ctrl.results[-1];assert result['valid_numeric'] and result['output']['reference_acceptance'] is True
        assert mode_survived and len(timings)>3 and all(r['valid'] for r in updates)
        metrics=dict(scope='WINDOWS_CPU_ACTUAL_COMPLETE_D11_WITH_SYNTHETIC_TIMESTAMP_EMISSIONS',
            actual_solver_result=result,fast_updates_during_solve=len(timings),fast_frame_complete_to_return=stats(timings),
            image_mode_to_fast_kept_same_future=mode_survived,images_completed=backend.completed,
            elapsed_seconds=(time.monotonic_ns()-start)/1e9,board_measured=False,physical_capture_age_ms=None,
            passed=True,fast_target_p95_100ms_met=stats(timings)['p95_ms']<=100,
            note='The original D11 PFY is repeated as explicitly synthetic emissions; this is a concurrency/numeric test, not new physical frames or dynamic recall.')
        write_json(output/'metrics.json',metrics);write_json(output/'fast_updates.json',updates);write_json(output/'audit.json',list(ctrl.audit.rows))
        print(json.dumps(metrics,indent=2));return metrics
    finally:ctrl.close()


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('action',choices=['fast','coexistence']);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    (fast_metrics if a.action=='fast' else coexistence)(a.output)
