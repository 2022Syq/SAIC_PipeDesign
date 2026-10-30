@echo off
cd /d "%~dp0"
set PYTHONUTF8=1
"D:\Miniforge3\envs\occ\python.exe" -m pipe_ui.app
pause
