@echo off
cd /d "%~dp0"
"D:\STM32\KEIL5\UV4\UV4.exe" -r "STM32F103C8T6_BASE.uvprojx" -j0 -o "build_diag.log"
echo UV4EXIT=%ERRORLEVEL%
