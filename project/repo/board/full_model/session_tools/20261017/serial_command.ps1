param([Parameter(Mandatory=$true)][string]$Script,[Parameter(Mandatory=$true)][string]$Log,[int]$TimeoutSeconds=35,[string]$PortName='COM10')
$ErrorActionPreference='Stop'
$source=[System.IO.File]::ReadAllText($Script,[System.Text.Encoding]::UTF8)
$encoded=[Convert]::ToBase64String([System.Text.Encoding]::UTF8.GetBytes($source))
$token='PS_DONE_'+[guid]::NewGuid().ToString('N')
$commands="python3 - <<'PS_SOURCE'`nimport base64,traceback`ntry:`n exec(base64.b64decode('"+$encoded+"'))`nexcept BaseException:`n print('PS_REMOTE_EXCEPTION')`n traceback.print_exc()`nprint('"+$token+"',flush=True)`nPS_SOURCE`nstty echo`n"
if ($encoded.Length -gt 3800) {throw 'Script exceeds safe single-line console transfer length'}
$port=New-Object System.IO.Ports.SerialPort $PortName,115200,None,8,One
$port.ReadTimeout=300;$port.WriteTimeout=5000;$port.DtrEnable=$false;$port.RtsEnable=$false
$text=''
$mutex=New-Object System.Threading.Mutex($false,'Local\PenumbraSight_COM10_20261017')
$ownsMutex=$false
try {
 $ownsMutex=$mutex.WaitOne(15000)
 if (-not $ownsMutex) {throw 'Another task owns the serial console'}
 $port.Open()
 $text+=$port.ReadExisting()
 $port.Write("stty -echo`n")
 Start-Sleep -Milliseconds 200
 $text+=$port.ReadExisting()
 $port.Write($commands)
 $timer=[System.Diagnostics.Stopwatch]::StartNew()
 $seen=$false
 while ($timer.Elapsed.TotalSeconds -lt $TimeoutSeconds) {
  $text+=$port.ReadExisting()
  if ($text.Contains($token)) {$seen=$true;Start-Sleep -Milliseconds 300;$text+=$port.ReadExisting();break}
  Start-Sleep -Milliseconds 100
 }
 [System.IO.File]::WriteAllText($Log,$text,(New-Object System.Text.UTF8Encoding $false))
 $text
 if (-not $seen) {throw 'Serial command deadline reached; remote state must be inspected before further commands'}
 if ($text.Contains('PS_REMOTE_EXCEPTION')) {throw 'Remote Python script failed; inspect saved log'}
} finally { if ($port.IsOpen) {$port.Close()};$port.Dispose();if ($ownsMutex) {$mutex.ReleaseMutex()};$mutex.Dispose() }
