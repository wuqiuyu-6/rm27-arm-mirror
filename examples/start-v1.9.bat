@echo off
rem RM27 v1.9 - manual assembly (keyboard) - web HMI
cd /d "%~dp0.."
set PY=..\..\omx-sim\.venv\Scripts\python.exe
if not exist "%PY%" set PY=python
set PYTHONPATH=%CD%\src
set PORT=8009

echo.
echo   [rm27-v1.9] manual assembly  http://127.0.0.1:%PORT%
echo   Close this window to stop.
echo.
start "" "http://127.0.0.1:%PORT%"
"%PY%" -m rm27_v19 --port %PORT%
echo.
echo   Exit code: %errorlevel%
pause
