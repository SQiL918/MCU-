@echo off
rem =====================================================================
rem  Build and run the host simulation of <release>\Keil????\App\path.c
rem  Needs Visual Studio 2022 (MSVC).  No other dependency.
rem  Double-click, or run from an "x64 Native Tools" prompt.
rem =====================================================================
setlocal enabledelayedexpansion
set HERE=%~dp0
rem Strip the trailing backslash: "cl" would treat \" as an escaped quote.
if "%HERE:~-1%"=="\" set HERE=%HERE:~0,-1%

rem The Keil project folder has a non-ASCII name, so find it by wildcard
rem instead of hard coding it (keeps this file ASCII-only).
set PROJ=
for /d %%D in ("%HERE%\..\Keil*") do set PROJ=%%~D
if not defined PROJ (
  echo [ERROR] Keil project folder not found next to this directory.
  pause
  exit /b 1
)

set VCVARS=
for %%P in (
  "%ProgramFiles%\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
  "%ProgramFiles%\Microsoft Visual Studio\2022\Professional\VC\Auxiliary\Build\vcvars64.bat"
  "%ProgramFiles%\Microsoft Visual Studio\2022\Enterprise\VC\Auxiliary\Build\vcvars64.bat"
  "%ProgramFiles(x86)%\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
  "D:\visual studio\visual stdio 2022\VC\Auxiliary\Build\vcvars64.bat"
) do (
  if exist %%P if not defined VCVARS set VCVARS=%%P
)

if defined VCVARS (
  call %VCVARS% >nul
) else (
  where cl >nul 2>nul
  if errorlevel 1 (
    echo.
    echo [ERROR] cl.exe not found. Open "x64 Native Tools Command Prompt for
    echo         VS 2022" and run this script there, or edit VCVARS here.
    pause
    exit /b 1
  )
)

pushd "%HERE%"
cl /nologo /W3 /utf-8 /D_CRT_SECURE_NO_WARNINGS /I"%HERE%" /I"%PROJ%\App" /I"%PROJ%\Bsp" "%HERE%\sim_q2.c" "%PROJ%\App\path.c" /Fe:"%HERE%\sim_q2.exe"
if errorlevel 1 (
  echo.
  echo [ERROR] build failed
  popd
  pause
  exit /b 1
)
echo.
sim_q2.exe
popd
pause
