@echo off
setlocal
rem Record the PL decision pipeline to CSV, one row per clock. No board needed.
rem NOTE: keep ASCII-only (cmd parser + OEM codepage).
rem Usage:
rem   run_pl_trace.bat                  - default smoke vectors
rem   run_pl_trace.bat <vectors.csv>    - use a specific vector file
rem   run_pl_trace.bat <vectors.csv> vcd - also dump a VCD waveform

set VIVADO_BIN=C:\Xilinx\Vivado\2022.2\bin
set SOURCE_ROOT=%~dp0..
set RUN_DIR=%TEMP%\fpga_av_pl_trace
set OUT_DIR=%~dp0..\..\verification_reports

set VECTORS=%~1
if "%VECTORS%"=="" set VECTORS=%~dp0fixtures\pl_vectors_smoke.csv

set VCDARG=
if /I "%~2"=="vcd" set VCDARG=-d TRACE_VCD

if not exist "%RUN_DIR%" mkdir "%RUN_DIR%"
copy /Y "%~dp0tb_pl_trace.sv" "%RUN_DIR%\tb_pl_trace.sv" >nul
copy /Y "%VECTORS%" "%RUN_DIR%\vectors.csv" >nul
pushd "%RUN_DIR%"

call "%VIVADO_BIN%\xvlog.bat" -sv %VCDARG% ^
  "%SOURCE_ROOT%\new\types_pkg.sv" ^
  "%SOURCE_ROOT%\new\preprocessor.sv" ^
  "%SOURCE_ROOT%\new\consistency_checker.sv" ^
  "%SOURCE_ROOT%\new\mask_20s.sv" ^
  "%SOURCE_ROOT%\new\sensor_checker.sv" ^
  "%SOURCE_ROOT%\new\sensor_reliability.sv" ^
  "%SOURCE_ROOT%\new\risk_types.sv" ^
  "%SOURCE_ROOT%\new\risk_control.sv" ^
  "%SOURCE_ROOT%\ip\sensor_input_1_0\hdl\sensor_input_v1_0_S00_AXI.v" ^
  "%SOURCE_ROOT%\new\top_controller.sv" ^
  tb_pl_trace.sv
if errorlevel 1 goto :fail

call "%VIVADO_BIN%\xelab.bat" -debug typical -top tb_pl_trace -snapshot tb_pl_trace_snap
if errorlevel 1 goto :fail

call "%VIVADO_BIN%\xsim.bat" tb_pl_trace_snap -runall
if errorlevel 1 goto :fail

if not exist "%OUT_DIR%" mkdir "%OUT_DIR%"
copy /Y "%RUN_DIR%\pl_trace.csv" "%OUT_DIR%\pl_trace.csv" >nul
if exist "%RUN_DIR%\pl_trace.vcd" copy /Y "%RUN_DIR%\pl_trace.vcd" "%OUT_DIR%\pl_trace.vcd" >nul

popd
echo.
echo Trace CSV : %OUT_DIR%\pl_trace.csv
if exist "%OUT_DIR%\pl_trace.vcd" echo Waveform  : %OUT_DIR%\pl_trace.vcd
exit /b 0

:fail
set RC=%errorlevel%
popd
exit /b %RC%
