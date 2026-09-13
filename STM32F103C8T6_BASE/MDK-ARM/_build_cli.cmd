@echo off
rem UV4 command-line build wrapper (dual-target). Usage: _build_cli.cmd target log
rem Note: uvprojx must keep UTF-8 BOM and original XML formatting (Keil rejects
rem DOM-rewritten files with exit 15). ASCII-relative filenames work fine from
rem this (Chinese) directory; no junction needed.
cd /d "%~dp0"
if "%~1"=="" (set TGT=F103-run) else (set TGT=%~1)
if "%~2"=="" (set LOG=build_cli.log) else (set LOG=%~2)
"D:\STM32\KEIL5\UV4\UV4.exe" -r "STM32F103C8T6_BASE.uvprojx" -t "%TGT%" -j0 -o "%LOG%"
echo UV4EXIT=%ERRORLEVEL%
