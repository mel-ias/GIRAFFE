@echo off

IF %1 == "" (
echo keine Eingabe
pause
) ELSE (
cd %~dp0
PWDatConverter.exe I %1 xyzRGB
pause
)
