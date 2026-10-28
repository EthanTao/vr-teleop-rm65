@echo off
chcp 65001 >nul
cd /d "%~dp0"
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\setup\start_simulation.ps1" -InputSource manual %*
set "RM65_EXIT_CODE=%errorlevel%"
if not "%RM65_EXIT_CODE%"=="0" pause
exit /b %RM65_EXIT_CODE%
