param([Parameter(Mandatory=$true)][string]$Script,[Parameter(Mandatory=$true)][string]$Log,[int]$TimeoutSeconds=110,[string]$PortName='COM14',[int]$BaudRate=115200)
$ErrorActionPreference='Stop'
$encoded=[Convert]::ToBase64String([IO.File]::ReadAllBytes($Script))
$token='PS_DONE_'+[guid]::NewGuid().ToString('N')
$shellToken='PS_SHELL_'+[guid]::NewGuid().ToString('N')
$lines=New-Object System.Collections.Generic.List[string]
$lines.Add("python3 - <<'PS_SOURCE'");$lines.Add('import base64,traceback');$lines.Add('payload=(')
for($i=0;$i -lt $encoded.Length;$i+=96){$lines.Add(" '"+$encoded.Substring($i,[Math]::Min(96,$encoded.Length-$i))+"'")}
$lines.Add(')');$lines.Add('try:');$lines.Add(' exec(base64.b64decode(payload))');$lines.Add('except BaseException:');$lines.Add(" print('PS_REMOTE_EXCEPTION')");$lines.Add(' traceback.print_exc()');$lines.Add("print('"+$token+"',flush=True)");$lines.Add('PS_SOURCE')
$port=New-Object IO.Ports.SerialPort $PortName,$BaudRate,None,8,One
$port.ReadTimeout=300;$port.WriteTimeout=5000;$port.DtrEnable=$false;$port.RtsEnable=$false
$mutex=New-Object Threading.Mutex($false,'Local\PenumbraSight_COM10_20261017');$owns=$false;$text=''
function Read-Serial([int]$Milliseconds){
 $until=[Diagnostics.Stopwatch]::StartNew()
 while($until.ElapsedMilliseconds -lt $Milliseconds){$script:text+=$port.ReadExisting();Start-Sleep -Milliseconds 50}
}
function Exact-Line([string]$Value){return [regex]::IsMatch($script:text,'(?m)^[> ]*'+[regex]::Escape($Value)+'\r?$')}
try{
 $owns=$mutex.WaitOne(15000);if(-not $owns){throw 'SERIAL_BUSY: Another task owns serial console'}
 $port.Open();Read-Serial 150
 $port.Write([string][char]3);$port.Write("`n");Read-Serial 400
 $ready=$false
 for($attempt=0;$attempt -lt 4;$attempt++){
  $tail=$text.Substring([Math]::Max(0,$text.Length-1600))
  if($tail -match '(?i)Password:\s*$'){$port.Write("root`n");Read-Serial 700}
  elseif($tail -match '(?i)login:\s*$'){
   $port.Write("root`n");Read-Serial 500
   if($text.Substring([Math]::Max(0,$text.Length-1200)) -match '(?i)Password:\s*$'){$port.Write("root`n");Read-Serial 700}
  }
  $port.Write("stty -echo`n");Read-Serial 200
  $port.Write("printf '\n"+$shellToken+"\n'`n");Read-Serial 500
  if(Exact-Line $shellToken){$ready=$true;break}
  $port.Write([string][char]3);$port.Write("`n");Read-Serial 300
 }
 if(-not $ready){throw 'LOGIN_NOT_READY: Board shell not ready; check boot/login before sending commands'}
 # A fresh login may initialize the terminal after the first stty; apply again.
 $port.Write("stty -echo`n");Read-Serial 350
 foreach($line in $lines){$port.Write($line+"`n");Read-Serial 80}
 $timer=[Diagnostics.Stopwatch]::StartNew();$seen=$false
 while($timer.Elapsed.TotalSeconds -lt $TimeoutSeconds){
  Read-Serial 100
  if(Exact-Line $token){$seen=$true;break}
 }
 if(-not $seen){throw 'SERIAL_TIMEOUT: Command did not return a verified completion marker'}
 if(Exact-Line 'PS_REMOTE_EXCEPTION'){throw 'REMOTE_ERROR: Board program failed; inspect traceback'}
}finally{
 [IO.File]::WriteAllText($Log,$text,(New-Object Text.UTF8Encoding $false))
 $text
 if($port.IsOpen){$port.Close()};$port.Dispose();if($owns){$mutex.ReleaseMutex()};$mutex.Dispose()
}
