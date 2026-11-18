@echo off
REM ============================================================
REM  project3 烧录脚本：STM8S103F3P6 I2C Slave + 4x7SEG
REM  纯 cmd 调用，不经过 PowerShell，输出不会被截断
REM ============================================================
setlocal
set FLASH=D:\stm8s_project\stm8flash.exe
set HEX=D:\stm8s_project\project3\project3.ihx
set LOG=%~dp0flash.log

if not exist "%FLASH%" (
    echo [ERROR] stm8flash.exe not found: %FLASH%
    goto :end
)
if not exist "%HEX%" (
    echo [ERROR] firmware not found: %HEX%
    goto :end
)

echo [INFO] Flashing %HEX%
echo.

"%FLASH%" -c stlinkv2 -p stm8s103f3 -w "%HEX%" > "%LOG%" 2>&1
set RC=%ERRORLEVEL%

type "%LOG%"
echo.
if %RC% EQU 0 (
    echo [OK] Flash succeeded.
) else (
    echo [FAIL] ExitCode=%RC%
)

:end
echo.
pause
endlocal
