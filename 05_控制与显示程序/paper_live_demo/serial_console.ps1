param([Parameter(Mandatory=$true)][string]$Script,[Parameter(Mandatory=$true)][string]$Log,[int]$TimeoutSeconds=30,[string]$PortName='COM14',[int]$BaudRate=115200)
$ErrorActionPreference='Stop'
$encoded=[Convert]::ToBase64String([IO.File]::ReadAllBytes($Script))
$token='SC03_DONE_'+[guid]::NewGuid().ToString('N')
$shellToken='SC03_SHELL_'+[guid]::NewGuid().ToString('N')
$port=New-Object IO.Ports.SerialPort $PortName,$BaudRate,None,8,One
$port.ReadTimeout=300;$port.WriteTimeout=5000;$port.DtrEnable=$false;$port.RtsEnable=$false
$mutex=New-Object Threading.Mutex($false,'Local\PenumbraSight_COM10_20261017');$owns=$false;$text=''
function Read-Serial([int]$Milliseconds){
 $until=[Diagnostics.Stopwatch]::StartNew()
 while($until.ElapsedMilliseconds -lt $Milliseconds){$script:text+=$port.ReadExisting();Start-Sleep -Milliseconds 40}
}
function Exact-Line([string]$Value){return [regex]::IsMatch($script:text,'(?m)^[> ]*'+[regex]::Escape($Value)+'\r?$')}
try{
 $owns=$mutex.WaitOne(10000);if(-not $owns){throw 'SERIAL_BUSY'}
 $port.Open();Read-Serial 500;$port.Write("`n");Read-Serial 500
 $ready=$false
 # Boot diagnostics may arrive after login/password prompts; do not require end-of-buffer.
 for($attempt=0;$attempt -lt 24;$attempt++){
  $tail=$text.Substring([Math]::Max(0,$text.Length-3000))
  $loginMatches=[regex]::Matches($tail,'(?im)login:\s*')
  $passwordMatches=[regex]::Matches($tail,'(?im)Password:\s*')
  $lastLogin=-1;$lastPassword=-1
  if($loginMatches.Count){$lastLogin=$loginMatches[$loginMatches.Count-1].Index}
  if($passwordMatches.Count){$lastPassword=$passwordMatches[$passwordMatches.Count-1].Index}
  if($lastLogin -gt $lastPassword){
   $port.Write("root`n");Read-Serial 900
   $port.Write("root`n");Read-Serial 1800
  } elseif($lastPassword -ge 0 -and $tail -notmatch 'root@[^\r\n]*[#$]'){
   $port.Write("root`n");Read-Serial 1800
  }
  $port.Write("printf '\n"+$shellToken+"\n'`n");Read-Serial 900
  if(Exact-Line $shellToken){$ready=$true;break}
  Read-Serial 800
 }
 if(-not $ready){throw 'NO_SHELL_RESPONSE: no reset or Ctrl-C was sent'}
 $port.Write("stty -echo`n");Read-Serial 300
 $lines=New-Object System.Collections.Generic.List[string]
 $lines.Add("python3 - <<'SC03_SOURCE'");$lines.Add('import base64,traceback');$lines.Add('payload=(')
 for($i=0;$i -lt $encoded.Length;$i+=96){$lines.Add(" '"+$encoded.Substring($i,[Math]::Min(96,$encoded.Length-$i))+"'")}
 $lines.Add(')');$lines.Add('try:');$lines.Add(' exec(base64.b64decode(payload))');$lines.Add('except BaseException:');$lines.Add(" print('SC03_REMOTE_EXCEPTION')");$lines.Add(' traceback.print_exc()');$lines.Add("print('"+$token+"',flush=True)");$lines.Add('SC03_SOURCE')
 foreach($line in $lines){$port.Write($line+"`n");Read-Serial 15}
 $timer=[Diagnostics.Stopwatch]::StartNew();$seen=$false
 while($timer.Elapsed.TotalSeconds -lt $TimeoutSeconds){Read-Serial 100;if(Exact-Line $token){$seen=$true;break}}
 if(-not $seen){throw 'SERIAL_TIMEOUT: inspect saved log before another command'}
 if(Exact-Line 'SC03_REMOTE_EXCEPTION'){throw 'REMOTE_EXCEPTION'}
}finally{
 [IO.File]::WriteAllText($Log,$text,(New-Object Text.UTF8Encoding $false));$text
 if($port.IsOpen){$port.Close()};$port.Dispose();if($owns){$mutex.ReleaseMutex()};$mutex.Dispose()
}
