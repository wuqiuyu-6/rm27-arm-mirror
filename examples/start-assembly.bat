@echo off
rem RM27 assembly simulation - start server + web HMI
cd /d "%~dp0.."
set PY=python
set PORT=8005

echo.
echo   [rm27-assembly] open http://127.0.0.1:%PORT%
echo   Close this window to stop.
echo.
start "" "http://127.0.0.1:%PORT%"
"%PY%" -m rm27_assembly all --port %PORT%
echo.
echo   Exit code: %errorlevel%
pause
