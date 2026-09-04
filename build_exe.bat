@echo off
REM ============================================================
REM  build_exe.bat - сборка miib_capture_gui.py в один .exe
REM  Запусти этот .bat из папки с miib_capture_gui.py
REM ============================================================

pip install pyserial numpy pyinstaller

pyinstaller --onefile --noconsole --uac-admin --name MIIB_Capture miib_capture_gui.py

echo.
echo Готово! exe находится в папке dist\MIIB_Capture.exe
pause
