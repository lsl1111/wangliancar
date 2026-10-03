@echo off
setlocal
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0stop_captain.ps1" -RuntimeDir "%~dp0..\runtime_data"
exit /b %ERRORLEVEL%
