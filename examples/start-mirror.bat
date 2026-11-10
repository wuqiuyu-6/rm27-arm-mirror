@echo off
rem RM27 arm mirror - read COM6 and drive the simulated 6-axis arm
cd /d "%~dp0.."
set PY=python

echo.
echo   [rm27-arm-mirror] serial COM6 @ 115200
echo   Close this window to stop.
echo   In the sim window: left drag = rotate, right drag = pan, wheel = zoom.
echo.
"%PY%" -m arm_mirror view --port COM6 --baud 115200
echo.
echo   Exit code: %errorlevel%
pause
