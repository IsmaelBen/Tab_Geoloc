@echo off
chcp 65001 >nul
cd /d "%~dp0"
set PY=py
where py >nul 2>nul || set PY=python
where %PY% >nul 2>nul || (echo Python introuvable : installer Python depuis python.org en cochant "Add python to PATH". & pause & exit /b 1)
echo ============================================================
echo  INSTALLATION DES BIBLIOTHEQUES PYTHON (une seule fois)
echo ============================================================
%PY% -m pip install numpy pandas scikit-learn matplotlib pyserial
echo.
pause
