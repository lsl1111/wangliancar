@echo off
setlocal
set "PID_FILE=%~dp0..\runtime_data\captain.pid"

if not exist "%PID_FILE%" (
  echo Captain is not running or the PID file is missing.
  exit /b 0
)

set /p CAPTAIN_PID=<"%PID_FILE%"
powershell.exe -NoProfile -Command "Stop-Process -Id %CAPTAIN_PID% -ErrorAction SilentlyContinue"
echo Stop request sent to Captain PID %CAPTAIN_PID%.

