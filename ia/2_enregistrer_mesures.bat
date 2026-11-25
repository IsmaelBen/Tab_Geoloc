@echo off
chcp 65001 >nul
cd /d "%~dp0"
set PY=py
where py >nul 2>nul || set PY=python
where %PY% >nul 2>nul || (echo Python introuvable : installer Python depuis python.org en cochant "Add python to PATH". & pause & exit /b 1)
echo ============================================================
echo  ENREGISTREMENT DES MESURES : touchez un bouton de distance sur le Tab5. Ctrl+C pour arreter.
echo ============================================================
%PY% analyse_reseau.py capture AUTO AUTO
echo.
pause
