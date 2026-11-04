import glob,json,time,os
samples=[]
for _ in range(100):
    sample={'uptime_seconds':float(open('/proc/uptime').read().split()[0])}
    sample['temperature_millicelsius']={p:int(open(p).read()) for p in glob.glob('/sys/class/hwmon/hwmon*/temp1_input')}
    sample['memory_kb']={line.split(':')[0]:int(line.split()[1]) for line in open('/proc/meminfo') if line.startswith(('MemAvailable:','MemFree:'))}
    samples.append(sample)
    with open('/home/root/ps_board/validation_telemetry.json','w') as f:json.dump(samples,f,indent=2)
    log=open('/home/root/ps_board/extended_validation.log').read()
    if 'EXTENDED_DONE' in log:break
    time.sleep(15)
