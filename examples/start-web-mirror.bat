@echo off
rem RM27 arm mirror - web HMI (3D page + gripper buttons)
cd /d "%~dp0.."
set PY=python
set PORT=8007

echo.
echo   [rm27-arm-mirror] web HMI  http://127.0.0.1:%PORT%
echo   Close this window to stop.
echo.
start "" "http://127.0.0.1:%PORT%"
"%PY%" web_mirror.py --port %PORT% --serial-port COM6
echo.
echo   Exit code: %errorlevel%
pause
