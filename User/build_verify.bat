@echo off
setlocal

rem ============================================================
rem  SmartGuard Build Verification Script
rem  Background: UV4 may link stale object files in some cases,
rem              producing a false positive of "0 Error 0
rem              Warning" while the changes were never compiled.
rem  Function: full rebuild + verify that every source file was
rem            actually recompiled (timestamp audit)
rem  Exit code: 0 = all checks passed;
rem             1 = build errors / not a full rebuild / stale
rem                 source files found
rem ============================================================

set "FAIL=0"
set "STALE_COUNT=0"

rem Project root = parent of this script's directory (User\)
pushd "%~dp0.."
set "ROOT=%CD%"
set "LOG=%ROOT%\build_log.txt"
set "STMP=%ROOT%\build_stale.tmp"
set "UV4=D:\Keil_v5\UV4\UV4.exe"

echo ============================================================
echo  SmartGuard Build Verification
echo  Project directory: %ROOT%
echo ============================================================
echo.

if not exist "%UV4%" (
    echo [ERROR] Keil UV4 not found: %UV4%
    popd
    exit /b 1
)

rem ---------- 1. Full rebuild ----------
echo [1/4] Running full Keil rebuild ^(-r Rebuild^)...
"%UV4%" -r "%ROOT%\User\Template.uvprojx" -j0 -o "%LOG%"
echo      UV4 exit code: %ERRORLEVEL%
echo.

if not exist "%LOG%" (
    echo [ERROR] build log not generated: %LOG%
    popd
    exit /b 1
)

rem ---------- 2. Build result ----------
echo [2/4] Checking build result...
findstr /C:"- 0 Error(s)" "%LOG%" >nul
if errorlevel 1 (
    echo      Build result: errors found, see below
    findstr /R /C:"Error(s)" "%LOG%"
    set "FAIL=1"
) else (
    echo      Build result: 0 Error
)
echo.

rem ---------- 3. Full-rebuild marker ----------
echo [3/4] Checking full-rebuild marker...
findstr /C:"Rebuild target" "%LOG%" >nul
if errorlevel 1 (
    echo      Full-rebuild marker: NO ^(incremental build, stale objects possible^)
    set "FAIL=1"
) else (
    echo      Full-rebuild marker: YES
)
echo.

rem ---------- 4. Timestamp audit ----------
echo [4/4] Auditing source vs object timestamps...
if not exist "Object\Template.axf" (
    echo      [ERROR] Object\Template.axf not found, cannot audit timestamps
    set "FAIL=1"
    goto :summary
)

if exist "%STMP%" del /f /q "%STMP%" >nul 2>&1
powershell -NoProfile -Command "$axf=(Get-Item 'Object\Template.axf').LastWriteTime; $stale=@(); $nocrf=@(); Get-ChildItem -Path 'User','Hardware','Library\src','Core' -Recurse -Include *.c,*.h | ForEach-Object { if ($_.Extension -eq '.c') { $crf='Object\' + $_.BaseName + '.crf'; if (Test-Path $crf) { if ($_.LastWriteTime -gt (Get-Item $crf).LastWriteTime) { $stale += $_.Name } } else { $nocrf += $_.FullName } } else { if ($_.LastWriteTime -gt $axf) { $stale += $_.Name } } }; $stale | ForEach-Object { Write-Output ('STALE|' + $_) }; $nocrf | ForEach-Object { Write-Output ('NOCRF|' + $_) }" > "%STMP%"

for /f "tokens=1,* delims=|" %%A in (%STMP%) do (
    if "%%A"=="STALE" (
        echo      Not compiled: %%B ^(source newer than object^)
        set /a STALE_COUNT+=1
    )
    if "%%A"=="NOCRF" (
        echo      Info: %%B not compiled ^(no .crf, disabled module^)
    )
)
if exist "%STMP%" del /f /q "%STMP%" >nul 2>&1

if %STALE_COUNT% GTR 0 (
    echo.
    echo      Uncompiled source files: %STALE_COUNT%
    set "FAIL=1"
) else (
    echo      Uncompiled source files: none
)
echo.

:summary
echo ============================================================
echo  Verification Summary
echo ============================================================
findstr /C:"Program Size" "%LOG%"
if %FAIL%==0 (
    echo  Result: PASS ^(full rebuild + all sources compiled + 0 Error^)
) else (
    echo  Result: FAIL, see messages above
)
echo ============================================================

popd
exit /b %FAIL%
