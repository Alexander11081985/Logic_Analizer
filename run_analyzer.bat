@echo off
cd /d "%~dp0"
py -3 tools\rx7500_analyzer.py
if errorlevel 1 pause

