@echo off
setlocal
rem risk/reliability gap regression. Previously run by hand without a .bat.
rem NOTE: keep ASCII-only (cmd parser + OEM codepage).
set VIVADO_BIN=C:\Xilinx\Vivado\2022.2\bin
set SOURCE_ROOT=%~dp0..
set RUN_DIR=%TEMP%\fpga_av_risk_reliability_matrix

if not exist "%RUN_DIR%" mkdir "%RUN_DIR%"
copy /Y "%~dp0tb_risk_reliability_matrix.sv" "%RUN_DIR%\tb_risk_reliability_matrix.sv" >nul
copy /Y "%~dp0run_risk_reliability_matrix.tcl" "%RUN_DIR%\run_risk_reliability_matrix.tcl" >nul
pushd "%RUN_DIR%"

call "%VIVADO_BIN%\xvlog.bat" -sv ^
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
  tb_risk_reliability_matrix.sv
if errorlevel 1 goto :fail

call "%VIVADO_BIN%\xelab.bat" -debug typical -top tb_risk_reliability_matrix ^
  -snapshot tb_risk_reliability_matrix_snap
if errorlevel 1 goto :fail

call "%VIVADO_BIN%\xsim.bat" tb_risk_reliability_matrix_snap ^
  -tclbatch run_risk_reliability_matrix.tcl
if errorlevel 1 goto :fail
popd
exit /b 0

:fail
set RC=%errorlevel%
popd
exit /b %RC%
