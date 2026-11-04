import subprocess,time,json,hashlib,glob,os
root='/home/root/ps_board/'
frames=sorted(glob.glob('/mnt/boot/ps_data/frames/*.psf'))
results=[]
start=time.monotonic()
def telemetry():
 d={}
 for p in glob.glob('/sys/bus/iio/devices/iio:device*/in_temp*'):
  try:d[p]=open(p).read().strip()
  except OSError:pass
 for line in open('/proc/meminfo'):
  if line.startswith(('MemAvailable:','MemFree:')):d[line.split(':')[0]]=line.split(':')[1].strip()
 return d
for i in range(61):
 frame=frames[i%len(frames)] if i<13 else frames[(i-13)%len(frames)]
 prefix=root+'extended_current'
 t=time.monotonic()
 with open(root+'extended_current.log','w') as log:
  try:rc=subprocess.call(['python3',root+'run_board.py','--frame',frame,'--output',prefix],stdout=log,stderr=subprocess.STDOUT,timeout=150)
  except subprocess.TimeoutExpired:rc=124
 row={'index':i,'frame':os.path.basename(frame),'rc':rc,'seconds':time.monotonic()-t,'telemetry':telemetry()}
 if rc==0:
  row['result']=json.load(open(prefix+'.json'))
  row['sha256']=hashlib.sha256(open(prefix+'.codes.bin','rb').read()).hexdigest()
  if i<13:
   import shutil
   for ext in ['.json','.codes.bin','.log']:
    shutil.copyfile(prefix+ext,root+'verified_'+os.path.basename(frame)+ext)
 results.append(row)
 json.dump({'elapsed_seconds':time.monotonic()-start,'results':results},open(root+'extended_validation.json','w'),indent=2)
 print('VALIDATION_CASE',i,row['frame'],rc,round(row['seconds'],2),row.get('sha256'),flush=True)
 if rc:break
print('EXTENDED_DONE',flush=True)
