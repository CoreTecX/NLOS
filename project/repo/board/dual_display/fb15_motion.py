"""Uncalibrated image-plane diagnostic. Does not bypass production FAST gates."""
from pathlib import Path
import time,json,cv2,numpy as np
class Motion:
 def __init__(self,out):
  self.out=Path(out);self.samples=[];self.bg=None;self.threshold=None;self.seq=0;self.present=False;self.high=0;self.low=0;self.track=0;self.last=None;self.seen=False;self.times=[];self.latencies=[];self.counts={};self.positions=[]
 def process(self,rgb16,meta):
  start=time.perf_counter();self.seq+=1
  # Fixed RAW12 scale; display stretching never enters detection.
  image=cv2.resize(rgb16.astype(np.float32),(126,126),interpolation=cv2.INTER_AREA)/4095.
  gray=image.mean(axis=2);event='NONE';bbox=None;centroid=None;direction='unknown';state='WARMUP'
  if self.bg is None:
   self.samples.append(gray.copy())
   if len(self.samples)>=40:
    samples=np.stack(self.samples);self.bg=np.median(samples,axis=0);noise=np.abs(samples-self.bg)
    self.threshold=float(max(.025,np.percentile(noise,99.5)*5));np.savez(str(self.out/'diagnostic_background.npz'),background=self.bg,threshold=self.threshold)
    self.samples=[]
  else:
   diff=np.abs(gray-self.bg);mask=(diff>self.threshold).astype(np.uint8)
   mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
   mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
   area=float(mask.mean());n,labels,stats,cent=cv2.connectedComponentsWithStats(mask,8)
   idx=1+int(np.argmax(stats[1:,cv2.CC_STAT_AREA])) if n>1 else None
   valid=idx is not None and stats[idx,cv2.CC_STAT_AREA]>=50 and area<.65
   if area>=.65:
    state='UNKNOWN_SCENE_CHANGE';event='INVALIDATED';self.present=False;self.high=self.low=0;self.last=None
   else:
    self.high=self.high+1 if valid else 0;self.low=self.low+1 if not valid else 0
    if self.high>=2 and not self.present:
     self.present=True;self.track+=1;event='REACQUIRED' if self.seen else 'APPEARED';self.seen=True
    if self.low>=3 and self.present:self.present=False;event='LOST';self.last=None
    state='PRESENT' if self.present else 'ABSENT'
    if self.present and valid:
     x,y,w,h,sz=map(int,stats[idx]);bbox=[x,y,w,h];centroid=list(map(float,cent[idx]))
     if self.last is not None:
      delta=centroid[0]-self.last[0]
      if abs(delta)>=1:direction='image_right' if delta>0 else 'image_left'
     self.last=centroid;self.positions.append(centroid)
  elapsed=(time.perf_counter()-start)*1000;age=(time.monotonic()-meta['complete_monotonic'])*1000
  self.times.append(elapsed);self.latencies.append(age)
  self.counts[event]=self.counts.get(event,0)+1
  row=dict(sequence=self.seq,sensor_block_id=meta['block_id'],timestamp_ticks=meta['timestamp_ticks'],state=state,event=event,bbox_126=bbox,centroid_126=centroid,direction=direction,track_id=self.track,processing_ms=elapsed,receive_complete_to_decision_ms=age,threshold=self.threshold,scope='UNCALIBRATED_IMAGE_DIAGNOSTIC',confidence_probability=None)
  with (self.out/'motion.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
  if self.seq%4==0 or event!='NONE':
   summary=dict(scope='UNCALIBRATED_IMAGE_DIAGNOSTIC',frames=self.seq,state=state,events=self.counts,threshold=self.threshold,tracks=self.track,processing_p50_ms=float(np.percentile(self.times,50)),processing_p95_ms=float(np.percentile(self.times,95)),receive_complete_to_decision_p95_ms=float(np.percentile(self.latencies,95)),formal_fast_accepted=False)
   p=self.out/'motion_status.tmp';p.write_text(json.dumps(summary));p.replace(self.out/'motion_status.json')
  return row
 def annotate(self,rgb,row):
  if row['bbox_126']:
   x,y,w,h=row['bbox_126'];sx=rgb.shape[1]/126.;sy=rgb.shape[0]/126.
   cv2.rectangle(rgb,(int(x*sx),int(y*sy)),(int((x+w)*sx),int((y+h)*sy)),(0,255,0),2)
  cv2.putText(rgb,row['state']+' '+row['direction'],(8,24),0,.55,(0,255,0),1)
  return rgb
