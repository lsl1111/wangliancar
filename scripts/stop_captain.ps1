param(
    [Parameter(Mandatory = $true)][string]$RuntimeDir,
    [double]$TimeoutSec = 5
)
$ErrorActionPreference = 'Stop'
$runtimePath = [IO.Path]::GetFullPath($RuntimeDir)
$pidPath = Join-Path $runtimePath 'captain.pid'
$stopPath = Join-Path $runtimePath 'captain.stop'
if (-not (Test-Path -LiteralPath $pidPath)) {
    Write-Output 'Captain is not running or the PID file is missing.'
    exit 0
}
$pidText = (Get-Content -LiteralPath $pidPath -Raw).Trim()
if ($pidText -notmatch '^[1-9][0-9]*$') {
    Write-Error 'Invalid Captain PID file.'
    exit 1
}
$captainPid = [int]$pidText
$captainProcess = Get-Process -Id $captainPid -ErrorAction SilentlyContinue
if ($null -eq $captainProcess) { exit 0 }
$startTime = $captainProcess.StartTime
Set-Content -LiteralPath $stopPath -Value $pidText -Encoding Ascii
Write-Output "Graceful stop requested for Captain PID $captainPid."
$timer = [Diagnostics.Stopwatch]::StartNew()
while ($timer.Elapsed.TotalSeconds -lt $TimeoutSec) {
    $current = Get-Process -Id $captainPid -ErrorAction SilentlyContinue
    if ($null -eq $current -or $current.StartTime -ne $startTime) { exit 0 }
    Start-Sleep -Milliseconds 100
}
# Retain the old emergency stop behavior for an unresponsive SDK, but never
# terminate a reused PID or a process that does not run this project's main.py.
$current = Get-Process -Id $captainPid -ErrorAction SilentlyContinue
if ($null -eq $current -or $current.StartTime -ne $startTime) { exit 0 }
$processInfo = Get-CimInstance Win32_Process -Filter "ProcessId=$captainPid"
$projectPath = Split-Path -Parent $runtimePath
$mainPath = Join-Path $projectPath 'main.py'
$batchMainPath = Join-Path $projectPath 'scripts\..\main.py'
if ($current.ProcessName -notmatch '^python(w)?$' -or
    ($processInfo.CommandLine.IndexOf($mainPath, [StringComparison]::OrdinalIgnoreCase) -lt 0 -and
     $processInfo.CommandLine.IndexOf($batchMainPath, [StringComparison]::OrdinalIgnoreCase) -lt 0)) {
    Write-Error 'Refusing to force-stop a process outside this Captain project.'
    exit 1
}
Write-Warning 'Captain did not exit within the timeout; forcing stop. Final evaluation save may be incomplete.'
Stop-Process -Id $captainPid -ErrorAction SilentlyContinue
exit 0
