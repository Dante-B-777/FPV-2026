@echo off
REM Launch the serial capture without fighting PowerShell quoting.
REM
REM The project lives under "OneDrive - Cal Poly", and that space-hyphen-space
REM makes PowerShell split the path into four arguments. %~dp0 is this file's
REM own directory, already quoted-safe, so the path never has to be typed.
REM
REM   log.bat              -> COM3, tag "flight"
REM   log.bat COM5         -> COM5, tag "flight"
REM   log.bat COM3 hover05 -> COM3, tag "hover05"

setlocal

set PORT=%1
if "%PORT%"=="" set PORT=COM3

set TAG=%2
if "%TAG%"=="" set TAG=flight

echo Port : %PORT%
echo Tag  : %TAG%
echo.

python "%~dp0capture.py" %PORT% --tag %TAG% --quiet

endlocal
