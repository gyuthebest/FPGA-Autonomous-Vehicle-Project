@echo off
setlocal enabledelayedexpansion
rem ===================================================================
rem Run this once the FPGA board is connected and powered.
rem   1) offline regression gate  (must pass before touching hardware)
rem   2) rebuild bitstream + XSA if RTL changed since last build
rem   3) program the board and bring up the A53 UDP bridge
rem   4) probe that the bridge is listening
rem
rem After this completes, start CARLA and run the live verification:
rem   set FPGA_ENABLED=1 & set CARLA_MAP=Town04 & set CARLA_LIVE_VERIFY=1
rem   python CARLA_FPGA_PROJECT\main.py
rem
rem NOTE: keep this file ASCII-only (cmd parser + OEM codepage).
rem ===================================================================

set HERE=%~dp0
set PROJ=%HERE%..\..
set VIVADO_BIN=C:\Xilinx\Vivado\2022.2\bin
rem program_and_bringup.tcl and probe_a53_state.tcl use XSCT commands
rem (connect / targets / fpga / dow / mrd) that Vivado tcl does not provide.
set XSCT=C:\Xilinx\Vitis\2022.2\bin\xsct.bat
set REPORTS=%PROJ%\verification_reports

set SKIPBUILD=0
if /I "%~1"=="skipbuild" set SKIPBUILD=1

echo ==========================================================
echo [1/4] Offline regression gate
echo ==========================================================
call "%HERE%run_offline_verification.bat"
if errorlevel 1 (
    echo.
    echo ABORT: offline regression failed. Fix before programming the board.
    exit /b 1
)

if "%SKIPBUILD%"=="1" (
    echo.
    echo [2/4] Skipped by request ^(skipbuild^)
    goto :program
)

echo.
echo ==========================================================
echo [2/4] Rebuild bitstream and XSA
echo ==========================================================
call "%VIVADO_BIN%\vivado.bat" -mode batch -nolog -nojournal ^
  -source "%HERE%refresh_packaged_ip.tcl" > "%REPORTS%\board_refresh.log" 2>&1
if errorlevel 1 (echo ABORT: packaged IP refresh failed, see board_refresh.log & exit /b 1)

call "%VIVADO_BIN%\vivado.bat" -mode batch -nolog -nojournal ^
  -source "%HERE%build_full_project_88888mhz.tcl" > "%REPORTS%\board_build.log" 2>&1
if errorlevel 1 (echo ABORT: build failed, see board_build.log & exit /b 1)

echo   Checking timing closure...
findstr /C:"TIMING_SETUP_WNS" "%REPORTS%\board_build.log"
findstr /C:"TIMING_HOLD_WHS" "%REPORTS%\board_build.log"

:program
echo.
echo ==========================================================
echo [3/4] Program board and bring up A53 bridge
echo ==========================================================
call "%XSCT%" "%HERE%program_and_bringup.tcl" > "%REPORTS%\board_program.log" 2>&1
if errorlevel 1 (echo ABORT: programming failed, see board_program.log & exit /b 1)
echo   Programmed.

echo.
echo ==========================================================
echo [4/4] Probe A53 state
echo ==========================================================
call "%XSCT%" "%HERE%probe_a53_state.tcl" > "%REPORTS%\board_probe.log" 2>&1
type "%REPORTS%\board_probe.log"
type "%PROJ%\verification_reports\bringup\a53_state.txt"

echo.
echo ==========================================================
echo BOARD READY.
echo Next: start CARLA Town04, then run
echo   set FPGA_ENABLED=1
echo   set CARLA_MAP=Town04
echo   set CARLA_LIVE_VERIFY=1
echo   python CARLA_FPGA_PROJECT\main.py
echo Then:
echo   python CARLA_FPGA_PROJECT\compare_golden_vs_pl.py --board ^<capture.csv^>
echo ==========================================================
exit /b 0
