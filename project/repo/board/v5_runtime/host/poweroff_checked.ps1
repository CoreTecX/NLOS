param([Parameter(Mandatory=$true)][string]$BootId,[Parameter(Mandatory=$true)][string]$LogDirectory,[string]$PortName='COM10')
$ErrorActionPreference='Stop'
if ($BootId -notmatch '^[0-9a-f-]{36}$') {throw 'Invalid boot ID'}
$source=@'
from pathlib import Path
import os,subprocess,json,struct
assert Path('/proc/sys/kernel/random/boot_id').read_text().strip()=='BOOT_ID'
mounts=Path('/proc/mounts').read_text()
root=next(x.split() for x in mounts.splitlines() if x.split()[1]=='/')
assert 'ro' in root[3].split(',') and not any(x.split()[1]=='/mnt/boot' for x in mounts.splitlines())
with open('/dev/mmcblk1p2','rb',buffering=0) as f:f.seek(1024);sb=f.read(1024)
assert struct.unpack_from('<H',sb,56)[0]==0xef53 and not struct.unpack_from('<I',sb,96)[0]&4
os.sync()
print('PS_V5_SHUTDOWN_READY',flush=True)
subprocess.Popen(['/bin/sh','-c','/bin/sleep 3; exec /sbin/poweroff -f -d'],stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
'@
$source=$source.Replace('BOOT_ID',$BootId)
$encoded=[Convert]::ToBase64String([System.Text.Encoding]::UTF8.GetBytes($source))
$command='python3 -c "import base64;exec(base64.b64decode('''+$encoded+'''))"'
$port=[System.IO.Ports.SerialPort]::new($PortName,115200,[System.IO.Ports.Parity]::None,8,[System.IO.Ports.StopBits]::One)
$port.DtrEnable=$false;$port.RtsEnable=$false;$port.WriteTimeout=5000
$mutex=[System.Threading.Mutex]::new($false,'Local\PenumbraSight_COM10_20261017');$owns=$false;$captured=''
try {
 $owns=$mutex.WaitOne(15000);if(-not $owns){throw 'Serial console busy'}
 $port.Open();$captured+=$port.ReadExisting();$port.Write("stty -echo`n");Start-Sleep -Milliseconds 200;$port.Write($command+"`n")
 $timer=[System.Diagnostics.Stopwatch]::StartNew()
 while($timer.Elapsed.TotalSeconds -lt 25){$captured+=$port.ReadExisting();if($captured.Contains('reboot: Power down')){break};Start-Sleep -Milliseconds 100}
 [System.IO.File]::WriteAllText((Join-Path $LogDirectory 'poweroff.serial.log'),$captured,[System.Text.UTF8Encoding]::new($false))
 if(-not $captured.Contains('PS_V5_SHUTDOWN_READY') -or -not $captured.Contains('reboot: Power down')){throw 'Power-down confirmation absent; inspect console log'}
 $receipt=[ordered]@{state='POWER_DOWN_CONFIRMED';confirmed_at=[DateTimeOffset]::Now.ToString('o');boot_id=$BootId;root_readonly=$true;boot_unmounted=$true;ext4_needs_recovery=$false;console_power_down_observed=$true}
 $text=$receipt|ConvertTo-Json
 [System.IO.File]::WriteAllText((Join-Path $LogDirectory 'poweroff.json'),$text,[System.Text.UTF8Encoding]::new($false));$text
} finally {if($port.IsOpen){$port.Close()};$port.Dispose();if($owns){$mutex.ReleaseMutex()};$mutex.Dispose()}
