@echo off
setlocal
set "PROJECT_DIR=%~dp0.."
set "PYTHON_EXE=E:\Sim-One\Tools\python36\python.exe"

if not exist "%PYTHON_EXE%" (
  echo [ERROR] SimOne Python not found: %PYTHON_EXE%
  pause
  exit /b 1
)

title NEVC Captain Runtime
cd /d "%PROJECT_DIR%"
"%PYTHON_EXE%" "%PROJECT_DIR%\main.py" %*
set "EXIT_CODE=%ERRORLEVEL%"
if not "%EXIT_CODE%"=="0" pause
exit /b %EXIT_CODE%

