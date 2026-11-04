import subprocess,time,json,hashlib
root='/home/root/ps_board/'
frames='/mnt/boot/ps_data/frames/'
cases=[('smilehat',['image_test_smilehat_f000.psf']),('it0',['image_test_it0_f000.psf']),('ambient1',['image_test_ambient1_f000.psf'])]
results=[]
for name,fs in cases:
 args=['python3',root+'run_board.py','--output',root+name]
 for f in fs:args+=['--frame',frames+f]
 start=time.monotonic()
 with open(root+name+'.log','w') as log:
  rc=subprocess.call(args,stdout=log,stderr=subprocess.STDOUT)
 row={'case':name,'returncode':rc,'seconds':time.monotonic()-start}
 if not rc:
  row['result']=json.load(open(root+name+'.json'))
  row['codes_sha256']=hashlib.sha256(open(root+name+'.codes.bin','rb').read()).hexdigest()
 results.append(row)
 json.dump(results,open(root+'suite_remaining.json','w'),indent=2)
 print('SUITE_CASE',name,rc,flush=True)
 if rc:break
print('SUITE_DONE',flush=True)
