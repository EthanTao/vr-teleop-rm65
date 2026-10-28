@echo off
chcp 65001 >nul
cd /d "%~dp0"
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\setup\start_real_robot.ps1" %*
set "RM65_EXIT_CODE=%errorlevel%"
if not "%RM65_EXIT_CODE%"=="0" pause
exit /b %RM65_EXIT_CODE%
