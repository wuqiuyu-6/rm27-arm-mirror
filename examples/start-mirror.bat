@echo off
rem RM27 arm mirror - read COM8 (config/joints.yaml) and drive the simulated arm
cd /d "%~dp0.."
set PY=python

echo.
echo   [rm27-arm-mirror] serial from config/joints.yaml (COM8)
echo   Close this window to stop.
echo   In the sim window: left drag = rotate, right drag = pan, wheel = zoom.
echo.
"%PY%" -m arm_mirror view --baud 115200
echo.
echo   Exit code: %errorlevel%
pause
