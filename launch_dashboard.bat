@echo off
echo ======================================================================
echo    Starting Active Eco-Assist BMS Digital Cockpit Telemetry Console
echo ======================================================================
echo.
echo Launching index.html in your default web browser...
start "" "%~dp0index.html"
echo.
echo Dashboard launched! You can also regenerate native C telemetry using:
echo   bms_simulator.exe --json ^> telemetry.json
echo.
pause
