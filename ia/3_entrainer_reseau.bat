@echo off
chcp 65001 >nul
cd /d "%~dp0"
set PY=py
where py >nul 2>nul || set PY=python
where %PY% >nul 2>nul || (echo Python introuvable : installer Python depuis python.org en cochant "Add python to PATH". & pause & exit /b 1)
echo ============================================================
echo  ANALYSE ET ENTRAINEMENT DU RESEAU DE NEURONES (quelques minutes)
echo ============================================================
%PY% analyse_reseau.py analyse
if exist resultats\comparaison.png start "" "resultats"
echo.
pause
