@echo off
setlocal enabledelayedexpansion
rem ===================================================================
rem Full verification that needs no FPGA board.
rem   1) RTL self-checking regressions (3)
rem   2) Golden model vs RTL simulation (3 scenarios)
rem   3) Python unit tests / noise+packing verification
rem
rem NOTE: keep this file ASCII-only. Korean text in a .bat breaks the
rem cmd parser under the default OEM codepage.
rem ===================================================================

set HERE=%~dp0
set PROJ=%HERE%..\..
set PY=%LOCALAPPDATA%\Programs\Python\Python312\python.exe
set FIX=%HERE%fixtures
set FAIL=0

echo ==========================================================
echo [1/3] RTL self-checking regressions
echo ==========================================================
call "%HERE%run_full_verification.bat" >nul 2>&1
if errorlevel 1 (echo   FAIL tb_pl_full_verification & set FAIL=1) else (echo   OK   tb_pl_full_verification)

call "%HERE%run_carla_axi_replay.bat" >nul 2>&1
if errorlevel 1 (echo   FAIL tb_carla_axi_replay & set FAIL=1) else (echo   OK   tb_carla_axi_replay)

call "%HERE%run_risk_reliability_matrix.bat" >nul 2>&1
if errorlevel 1 (echo   FAIL tb_risk_reliability_matrix & set FAIL=1) else (echo   OK   tb_risk_reliability_matrix)

echo.
echo ==========================================================
echo [2/3] Golden model vs RTL simulation
echo ==========================================================
pushd "%PROJ%\CARLA_FPGA_PROJECT"
rem standstill_slope opens consistency_mask_4/5/6 (situation == 000), which is
rem the only way relations 9..16 are ever exercised. Real driving captures never
rem activate them: 0 active samples in 3489.
for %%S in (turn straight brake_ice standstill_slope) do (
    "%PY%" make_trace_vectors.py %%S --samples 60 --out "%FIX%\vectors_%%S.csv" >nul
    call "%HERE%run_pl_trace.bat" "%FIX%\vectors_%%S.csv" >nul 2>&1
    "%PY%" compare_golden_vs_pl.py --vectors "%FIX%\vectors_%%S.csv" >nul 2>&1
    if errorlevel 1 (echo   FAIL golden-vs-rtl %%S & set FAIL=1) else (echo   OK   golden-vs-rtl %%S)
)

echo.
echo ==========================================================
echo [3/3] Python verification
echo ==========================================================
"%PY%" verify_sensor_noise.py >nul 2>&1
if errorlevel 1 (echo   FAIL verify_sensor_noise & set FAIL=1) else (echo   OK   verify_sensor_noise)

"%PY%" -m unittest test_scenario_pl_alignment >nul 2>&1
if errorlevel 1 (echo   FAIL test_scenario_pl_alignment & set FAIL=1) else (echo   OK   test_scenario_pl_alignment)
popd

echo.
echo ==========================================================
if "%FAIL%"=="0" (echo OVERALL: PASS) else (echo OVERALL: FAIL)
echo ==========================================================
exit /b %FAIL%
