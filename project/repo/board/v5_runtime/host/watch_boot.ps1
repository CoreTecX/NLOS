param([Parameter(Mandatory=$true)][string]$Log,[int]$Seconds=45,[string]$PortName='COM10')
$ErrorActionPreference='Stop'
if($Seconds -lt 1 -or $Seconds -gt 55){throw 'Use bounded intervals of at most 55 seconds'}
$port=[System.IO.Ports.SerialPort]::new($PortName,115200,[System.IO.Ports.Parity]::None,8,[System.IO.Ports.StopBits]::One)
$port.DtrEnable=$false;$port.RtsEnable=$false;$text=''
$mutex=[System.Threading.Mutex]::new($false,'Local\PenumbraSight_COM10_20261017');$owns=$false
try {
 $owns=$mutex.WaitOne(15000);if(-not $owns){throw 'Serial console busy'}
 $port.Open();$timer=[System.Diagnostics.Stopwatch]::StartNew()
 while($timer.Elapsed.TotalSeconds -lt $Seconds){$text+=$port.ReadExisting();Start-Sleep -Milliseconds 100}
 [System.IO.File]::AppendAllText($Log,$text,[System.Text.UTF8Encoding]::new($false))
 [ordered]@{captured_characters=$text.Length;linux_seen=$text.Contains('Booting Linux');v5_console_seen=$text.Contains('PenumbraSight V5 recovery console')}|ConvertTo-Json
} finally {if($port.IsOpen){$port.Close()};$port.Dispose();if($owns){$mutex.ReleaseMutex()};$mutex.Dispose()}
