<#
.SYNOPSIS
    用 Vivado xsim（xvlog / xelab / xsim）串行跑通 src/fpga/rtl/tb 下的 5 个 testbench。

.DESCRIPTION
    本机 iverilog 不在 PATH，RTL 仿真只能用 Vivado 自带的 xsim。脚本做三件事：
      1. 设置 PATH / XILINXD_LICENSE_FILE，并把 TEMP/TMP 指到纯 ASCII 目录（默认 D:\Temp），
         避免 xsim 的中间文件落进中文用户目录；
      2. 逐个 testbench（严格串行，一次只起一个仿真进程，避免 2.6 GB 内存下 OOM）执行
         xvlog -sv  ->  xelab --timescale 1ns/1ps  ->  xsim -R；
      3. 以 TB 自己打印的 "=== <tb> : PASS ===" / "FAIL" 标记为准判定通过与否，
         任一 TB 未通过即 exit 1。

    路径来源（2026-10-02 起）：**环境变量优先，默认值保留**，默认仍能在本机跑。
      PS_VIVADO_BIN        Vivado bin 目录（默认 D:\vivado\2025.2\Vivado\bin）
      XILINXD_LICENSE_FILE License 文件（默认 D:\xilinx_lic\Xilinx_official_DRURY.lic）
      PS_TEMP_DIR          xsim 中间文件目录（默认 D:\Temp）
    显式命令行参数（-VivadoBin 等）优先级最高，便于临时覆盖。

    关键点（不是可选项）：
      * xelab --timescale 1ns/1ps 必须带。5 个 RTL 模块文首没有 timescale，
        而 TB 有（timescale 1ns/1ps）。xsim 对「同一次 elaboration 里既有带
        timescale 又有不带的模块」直接报 [XSIM 43-4099] 并中止。加 --timescale 给
        缺省模块补上默认值即可 —— 这样不必修改 RTL 源码。
      * 判定用日志里的 PASS/FAIL 文本，不只看进程退出码：xsim 运行期断言失败时
        $finish 仍可能返回 0。
      * 过滤日志统一「先取全文再筛」，不用 Select-Object -First 截断管道
        （那会在 N 行后关闭管道、杀掉 Vivado 进程，留下半截日志）。

.PARAMETER RepoRoot
    仓库根。默认从脚本位置向上 3 级推出（src/fpga/sim -> 仓库根）。

.PARAMETER WorkDir
    中间产物目录，默认 <RepoRoot>\build\xsim。每个 TB 一个子目录，互不干扰。

.EXAMPLE
    powershell -NoProfile -ExecutionPolicy Bypass -File src\fpga\sim\run_xsim.ps1
#>
[CmdletBinding()]
param(
    [string]$RepoRoot   = '',
    [string]$WorkDir    = '',
    [string]$VivadoBin  = $env:PS_VIVADO_BIN,
    [string]$LicenseFile= $env:XILINXD_LICENSE_FILE,
    [string]$TempDir    = $env:PS_TEMP_DIR
)

# 环境变量未给时回落到本机默认值（**默认值保留**，所以不带任何变量也能在本机跑）。
if ([string]::IsNullOrWhiteSpace($VivadoBin))   { $VivadoBin   = 'D:\vivado\2025.2\Vivado\bin' }
if ([string]::IsNullOrWhiteSpace($LicenseFile)) { $LicenseFile = 'D:\xilinx_lic\Xilinx_official_DRURY.lic' }
if ([string]::IsNullOrWhiteSpace($TempDir))     { $TempDir     = 'D:\Temp' }

$ErrorActionPreference = 'Continue'

# ---------------------------------------------------------------- 仓库根
if ([string]::IsNullOrWhiteSpace($RepoRoot)) {
    $scriptPath = $MyInvocation.MyCommand.Path
    if ([string]::IsNullOrWhiteSpace($scriptPath)) { $scriptPath = $PSCommandPath }
    $simDir   = Split-Path -Parent $scriptPath        # src/fpga/sim
    $RepoRoot = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $simDir))
}

if ([string]::IsNullOrWhiteSpace($WorkDir)) { $WorkDir = Join-Path $RepoRoot 'build\xsim' }

# ---------------------------------------------------------------- 环境
$env:PATH = "$VivadoBin;$env:PATH"
$env:XILINXD_LICENSE_FILE = $LicenseFile
if (Test-Path $TempDir) { $env:TEMP = $TempDir; $env:TMP = $TempDir }

$xvlog = Join-Path $VivadoBin 'xvlog.bat'
$xelab = Join-Path $VivadoBin 'xelab.bat'
$xsim  = Join-Path $VivadoBin 'xsim.bat'
foreach ($tool in @($xvlog, $xelab, $xsim)) {
    if (-not (Test-Path $tool)) { Write-Host "[run_xsim] 找不到工具：$tool"; exit 2 }
}

$rtlDir = Join-Path $RepoRoot 'src\fpga\rtl'
$tbDir  = Join-Path $rtlDir  'tb'
New-Item -ItemType Directory -Force -Path $WorkDir | Out-Null

Write-Host "[run_xsim] 仓库根   : $RepoRoot"
Write-Host "[run_xsim] 中间产物 : $WorkDir"
Write-Host "[run_xsim] Vivado   : $VivadoBin"
Write-Host ("[run_xsim] TEMP/TMP : {0}" -f $env:TEMP)

# 串行顺序：先小后大（compositor 跑整帧，放最后）
$tbList = @('raw_preproc', 'vertical_diff', 'fast_alert', 'confidence_engine', 'compositor')

$failed  = @()
$results = @()

foreach ($mod in $tbList) {
    $tbName = "${mod}_tb"
    $snap   = "sim_$mod"
    $rtl    = Join-Path $rtlDir "$mod.sv"
    $tb     = Join-Path $tbDir  "$tbName.sv"
    $dir    = Join-Path $WorkDir $mod
    $log    = Join-Path $dir 'xsim_run.log'

    Write-Host ("=" * 72)
    Write-Host "[run_xsim] >>> $mod"

    if (-not (Test-Path $rtl)) { Write-Host "[run_xsim] $mod : FAIL (缺 RTL $rtl)"; $failed += $mod; continue }
    if (-not (Test-Path $tb))  { Write-Host "[run_xsim] $mod : FAIL (缺 TB  $tb)"; $failed += $mod; continue }

    New-Item -ItemType Directory -Force -Path $dir | Out-Null
    Get-ChildItem -Path $dir -Filter '*.log' -ErrorAction SilentlyContinue | Remove-Item -Force -ErrorAction SilentlyContinue
    "### xsim run: $mod    $(Get-Date -Format s)" | Out-File -FilePath $log -Encoding utf8
    "### cmd: xvlog -sv $rtl $tb"           | Out-File -FilePath $log -Append -Encoding utf8
    "### cmd: xelab $tbName -s $snap --timescale 1ns/1ps" | Out-File -FilePath $log -Append -Encoding utf8
    "### cmd: xsim $snap -R"                 | Out-File -FilePath $log -Append -Encoding utf8

    $xvCode = -1; $xlCode = -1; $xsCode = -1

    Push-Location $dir
    try {
        $out = (& $xvlog -sv $rtl $tb 2>&1)
        $xvCode = $LASTEXITCODE
        ($out | Out-String) | Out-File -FilePath $log -Append -Encoding utf8

        if ($xvCode -eq 0) {
            $out = (& $xelab $tbName -s $snap --timescale 1ns/1ps 2>&1)
            $xlCode = $LASTEXITCODE
            ($out | Out-String) | Out-File -FilePath $log -Append -Encoding utf8

            if ($xlCode -eq 0) {
                $out = (& $xsim $snap -R 2>&1)
                $xsCode = $LASTEXITCODE
                ($out | Out-String) | Out-File -FilePath $log -Append -Encoding utf8
            }
        }
    } finally {
        Pop-Location
    }

    $text    = Get-Content -Path $log -Raw
    $isPass  = $text -match "===\s+$tbName\s+:\s+PASS\s+==="
    $isFail  = $text -match "===\s+$tbName\s+:\s+FAIL"
    $verdict = if ($isPass) { 'PASS' } elseif ($isFail) { 'FAIL' } else { 'FAIL(无判据标记)' }
    if ($verdict -ne 'PASS') { $failed += $mod }

    # 关键行：先取全文再筛（不截断管道）
    $keyLines = @($text -split "`r?`n" | Where-Object { $_ -match '\[\s*ERR|:\s*PASS|:\s*FAIL|\[INFO\]\s+T\d' })
    foreach ($line in $keyLines) { Write-Host "    $line" }

    Write-Host ("[run_xsim] {0} : {1}   (xvlog={2} xelab={3} xsim={4})  日志: {5}" -f `
                $mod, $verdict, $xvCode, $xlCode, $xsCode, $log)

    $results += [pscustomobject]@{
        Module = $mod; Verdict = $verdict
        Xvlog = $xvCode; Xelab = $xlCode; Xsim = $xsCode; Log = $log
    }
}

Write-Host ("=" * 72)
Write-Host "[run_xsim] 汇总"
foreach ($r in $results) {
    Write-Host ("    {0,-18} {1}" -f $r.Module, $r.Verdict)
}

if ($failed.Count -gt 0) {
    Write-Host ("[run_xsim] 有 TB 未通过：{0}" -f ($failed -join ', '))
    exit 1
}
Write-Host "[run_xsim] 5/5 全部通过"
exit 0
