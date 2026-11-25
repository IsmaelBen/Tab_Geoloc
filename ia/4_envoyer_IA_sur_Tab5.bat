@echo off
chcp 65001 >nul
cd /d "%~dp0"
set PY=py
where py >nul 2>nul || set PY=python
where %PY% >nul 2>nul || (echo Python introuvable : installer Python depuis python.org en cochant "Add python to PATH". & pause & exit /b 1)
echo ============================================================
echo  ENVOI DU PROGRAMME IA SUR LE TAB5 (arreter l'enregistrement avant)
echo ============================================================
%PY% analyse_reseau.py televerser Tab5_IA
echo.
pause
