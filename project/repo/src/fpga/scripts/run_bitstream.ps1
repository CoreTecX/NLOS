# ============================================================================
# PenumbraSight PL 实现到 bitstream 的一键启动 · src/fpga/scripts/run_bitstream.ps1
#
# 用途：把「块设计 wrapper -> 约束 -> 综合 -> 实现 -> write_bitstream」这一趟
#       连同环境准备（PATH、并发数、日志落盘）一次做完。
#
# 用法
# ----------------------------------------------------------------------------
#   powershell -ExecutionPolicy Bypass -File src\fpga\scripts\run_bitstream.ps1
#   powershell -ExecutionPolicy Bypass -File src\fpga\scripts\run_bitstream.ps1 -Jobs 2
#   powershell -ExecutionPolicy Bypass -File src\fpga\scripts\run_bitstream.ps1 -Repo D:\psight\build_t2
#
# 为什么需要这个脚本
# ----------------------------------------------------------------------------
#   1. **Vivado 与 Vitis 都不在 PATH 里**（本机 2026-09-30 核实：`where vivado`
#      返回空，但 `D:\vivado\2025.2\Vivado\bin\vivado.bat` 存在）。makefile 的
#      `PS_WHERE = $(shell where $(1))` 依赖 PATH，所以直接 `make` 会报「找不到」。
#   2. 并发数必须按**内存**定：本机 15.5 GB 物理内存，2026-09-29 实测
#      `-jobs 8` 会让 8 个子运行同时起、各要 1.7~1.9 GB 而 OOM；而单子运行峰值
#      也不小（zynq_ps 3971 MB / sctl_ctrl 3276 / core_proj 3167 / rst_pl 2964）。
#      故默认 `-Jobs 1` 串行。页面文件上限 37 GB 是兜底，不是让人放开并发。
#   3. 一趟跑 2~2.5 小时，日志必须落盘，不能只看终端回看。
#
# 前置（脚本会自己检查）
# ----------------------------------------------------------------------------
#   - 构建区是**纯 ASCII 路径**（中文用户名/路径在 Vivado 2025.2 上会出问题）
#   - `build/bd_project/bd_project.xpr` 存在且已 validate 通过
#   - `src/fpga/hls/ip_repo/` 里四个核的 component.xml 比 HLS 源码新
# ============================================================================

param(
    [int]$Jobs = 0,
    [string]$Repo = '',
    [string]$Xilinx = '',
    [switch]$CheckOnly
)

# ---------------------------------------------------------------------------
# 路径/并发来源（2026-10-02 起）：**环境变量优先，默认值保留**，默认仍能在本机跑。
#   PS_PL_JOBS      并发数（等价 -Jobs；默认 1）
#   PS_BUILD_REPO   ASCII 构建区仓库根（等价 -Repo；默认 D:\psight\build_t2）
#   PS_XILINX_ROOT  Xilinx 安装根（等价 -Xilinx；默认 D:\vivado\2025.2）
# 显式命令行参数优先级最高（-Jobs N / -Repo ... / -Xilinx ...）。
# ---------------------------------------------------------------------------
if ($Jobs -le 0) {
    $Jobs = if ($env:PS_PL_JOBS -and [int]$env:PS_PL_JOBS -gt 0) { [int]$env:PS_PL_JOBS } else { 1 }
}
if ([string]::IsNullOrWhiteSpace($Repo)) {
    $Repo = if ($env:PS_BUILD_REPO) { $env:PS_BUILD_REPO } else { 'D:\psight\build_t2' }
}
if ([string]::IsNullOrWhiteSpace($Xilinx)) {
    $Xilinx = if ($env:PS_XILINX_ROOT) { $env:PS_XILINX_ROOT } else { 'D:\vivado\2025.2' }
}

$ErrorActionPreference = 'Stop'

function Say($m) { Write-Host $m }
function Fail($m) { Write-Host "错误：$m" -ForegroundColor Red; exit 1 }

# ---- 1. 工具链 ------------------------------------------------------------
$vbin = Join-Path $Xilinx 'Vivado\bin'
$tbin = Join-Path $Xilinx 'Vitis\bin'
$vivado = Join-Path $vbin 'vivado.bat'
if (-not (Test-Path $vivado)) { Fail "找不到 $vivado。用 -Xilinx 指定安装根（当前 $Xilinx）" }
$env:PATH = "$vbin;$tbin;$env:PATH"
Say ("Vivado      : " + $vivado)

# ---- 2. 构建区与前置 ------------------------------------------------------
if (-not (Test-Path $Repo)) { Fail "构建区不存在：$Repo" }
$Repo = (Resolve-Path $Repo).Path
if ($Repo -match '[^\x00-\x7F]') { Fail "构建区路径含非 ASCII 字符，Vivado 2025.2 会出问题：$Repo" }
Say ("构建区      : " + $Repo)

$xpr = Join-Path $Repo 'build\bd_project\bd_project.xpr'
if (-not (Test-Path $xpr)) { Fail "找不到块设计工程 $xpr。先跑 src/fpga/bd/penumbrasight_bd.tcl" }
Say ("块设计工程  : " + (Get-Item $xpr).LastWriteTime.ToString('yyyy-MM-dd HH:mm'))

$ipRepo = Join-Path $Repo 'src\fpga\hls\ip_repo'
if (-not (Test-Path $ipRepo)) { Fail "找不到 IP 仓 $ipRepo" }
$ips = Get-ChildItem $ipRepo -Directory -ErrorAction SilentlyContinue | ForEach-Object {
    $c = Get-ChildItem $_.FullName -Recurse -Filter 'component.xml' -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($c) { [pscustomobject]@{ 核 = $_.Name; 时间 = $c.LastWriteTime } }
}
Say "IP 仓        :"
$ips | ForEach-Object { Say ("               " + $_.核.PadRight(10) + $_.时间.ToString('yyyy-MM-dd HH:mm')) }

# IP 是否比 HLS 源码新 —— 这是 2026-09-29 踩过的坑：源码改了两轮（修法 C 两步），
# 而 IP 还停在改动之前，跑出来的位流会是旧核。
$newestSrc = Get-ChildItem (Join-Path $Repo 'src\fpga\hls') -Recurse -Include '*.cpp','*.hpp' -ErrorAction SilentlyContinue |
             Sort-Object LastWriteTime -Descending | Select-Object -First 1
if ($newestSrc) {
    $oldestIp = $ips | Sort-Object 时间 | Select-Object -First 1
    Say ("HLS 源码最新: " + $newestSrc.Name + "  " + $newestSrc.LastWriteTime.ToString('yyyy-MM-dd HH:mm'))
    if ($oldestIp -and $oldestIp.时间 -lt $newestSrc.LastWriteTime) {
        Say ""
        Say "警告：有 IP 比 HLS 源码旧，位流里会是旧核。"
        Say ("      最旧 IP " + $oldestIp.核 + " " + $oldestIp.时间.ToString('yyyy-MM-dd HH:mm') +
             "  <  最新源码 " + $newestSrc.LastWriteTime.ToString('yyyy-MM-dd HH:mm'))
        Say "      先重跑 HLS 并重出 IP（见 sim/results/pl_t2_rebuild_and_seam_20260929.md）"
        Say ""
    }
}

# ---- 3. 内存 --------------------------------------------------------------
$os = Get-CimInstance Win32_OperatingSystem
$freeGB = [math]::Round($os.FreePhysicalMemory / 1MB, 2)
$pf = Get-CimInstance Win32_PageFileUsage | Select-Object -First 1
Say ("可用内存    : " + $freeGB + " GB")
if ($pf) { Say ("页面文件    : 已用 " + $pf.CurrentUsage + " MB / 上限 " + $pf.AllocatedBaseSize + " MB") }
Say ("并发数      : " + $Jobs + "（子运行峰值实测 3~4 GB，物理不够时靠页面文件兜底，只是慢）")

if ($CheckOnly) { Say "`n（-CheckOnly：只做检查，不启动）"; exit 0 }

# ---- 4. 启动 --------------------------------------------------------------
$logDir = Join-Path $Repo 'logs'
if (-not (Test-Path $logDir)) { New-Item -ItemType Directory -Path $logDir | Out-Null }
$stamp = Get-Date -Format 'yyyyMMdd_HHmmss'
$log = Join-Path $logDir "pl_bitstream_$stamp.log"

$env:PS_PL_JOBS = "$Jobs"
Say ""
Say ("启动        : vivado -mode batch（日志 " + $log + "）")
Say "预计        2~2.5 小时。中途不要停父进程 —— 停了会留下孤儿 Vivado 继续占内存。"
Say ""

Push-Location $Repo
try {
    & $vivado -mode batch -nojournal -nolog -source 'src/fpga/scripts/build_pl.tcl' 2>&1 |
        Tee-Object -FilePath $log
    $rc = $LASTEXITCODE
} finally { Pop-Location }

Say ""
Say ("退出码      : " + $rc)
$bit = Get-ChildItem (Join-Path $Repo 'build\bd_project\bd_project.runs\impl_1') -Filter '*.bit' -ErrorAction SilentlyContinue
if ($bit) {
    Say ("位流        : " + $bit.FullName + "  " + [math]::Round($bit.Length / 1MB, 2) + " MB")
} else {
    Say "位流        : 未生成，看日志末尾的 ERROR 行"
}
foreach ($r in @('report_utilization_impl.rpt', 'report_timing_impl.rpt', 'report_drc.rpt')) {
    $p = Join-Path $Repo "build\pl\$r"
    if (Test-Path $p) { Say ("报告        : " + $p) }
}
