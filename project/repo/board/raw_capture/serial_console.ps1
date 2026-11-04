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
 for($attempt=0;$attempt -lt 3;$attempt++){
  $tail=$text.Substring([Math]::Max(0,$text.Length-2000))
  if($tail -match '(?i)Password:\s*$'){$port.Write("root`n");Read-Serial 700}
  elseif($tail -match '(?i)login:\s*$'){
   $port.Write("root`n");Read-Serial 500
   if($text.Substring([Math]::Max(0,$text.Length-1200)) -match '(?i)Password:\s*$'){$port.Write("root`n");Read-Serial 700}
  }
  $port.Write("printf '\n"+$shellToken+"\n'`n");Read-Serial 700
  if(Exact-Line $shellToken){$ready=$true;break}
  $port.Write("`n");Read-Serial 400
 }
 if(-not $ready){throw 'NO_SHELL_RESPONSE: no reset or Ctrl-C was sent'}
 $port.Write("stty -echo`n");Read-Serial 300
 $lines=New-Object System.Collections.Generic.List[string]
 $lines.Add("python3 - <<'SC03_SOURCE'");$lines.Add('import base64,traceback');$lines.Add('payload=(')
 for($i=0;$i -lt $encoded.Length;$i+=96){$lines.Add(" '"+$encoded.Substring($i,[Math]::Min(96,$encoded.Length-$i))+"'")}
 $lines.Add(')');$lines.Add('try:');$lines.Add(' exec(base64.b64decode(payload))');$lines.Add('except BaseException:');$lines.Add(" print('SC03_REMOTE_EXCEPTION')");$lines.Add(' traceback.print_exc()');$lines.Add("print('"+$token+"',flush=True)");$lines.Add('SC03_SOURCE')
 foreach($line in $lines){$port.Write($line+"`n");Read-Serial 65}
 $timer=[Diagnostics.Stopwatch]::StartNew();$seen=$false
 while($timer.Elapsed.TotalSeconds -lt $TimeoutSeconds){Read-Serial 100;if(Exact-Line $token){$seen=$true;break}}
 if(-not $seen){throw 'SERIAL_TIMEOUT: inspect saved log before another command'}
 if(Exact-Line 'SC03_REMOTE_EXCEPTION'){throw 'REMOTE_EXCEPTION'}
}finally{
 [IO.File]::WriteAllText($Log,$text,(New-Object Text.UTF8Encoding $false));$text
 if($port.IsOpen){$port.Close()};$port.Dispose();if($owns){$mutex.ReleaseMutex()};$mutex.Dispose()
}
