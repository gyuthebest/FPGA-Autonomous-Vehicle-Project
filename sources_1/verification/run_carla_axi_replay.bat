@echo off
setlocal
set VIVADO_BIN=C:\Xilinx\Vivado\2022.2\bin
set SOURCE_ROOT=%~dp0..
set RUN_DIR=%TEMP%\fpga_av_carla_axi_replay
set VECTOR_FILE=%~1
if "%VECTOR_FILE%"=="" set VECTOR_FILE=%~dp0fixtures\pl_vectors_smoke.csv

if not exist "%VECTOR_FILE%" (
  echo Vector file not found: %VECTOR_FILE%
  exit /b 2
)
if not exist "%RUN_DIR%" mkdir "%RUN_DIR%"
if not exist "%~dp0..\..\verification_reports" mkdir "%~dp0..\..\verification_reports"
copy /Y "%~dp0tb_carla_axi_replay.sv" "%RUN_DIR%\tb_carla_axi_replay.sv" >nul
copy /Y "%~dp0run_carla_axi_replay.tcl" "%RUN_DIR%\run_carla_axi_replay.tcl" >nul
copy /Y "%VECTOR_FILE%" "%RUN_DIR%\pl_vectors.csv" >nul
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
  tb_carla_axi_replay.sv
if errorlevel 1 goto :fail

call "%VIVADO_BIN%\xelab.bat" -debug typical -top tb_carla_axi_replay -snapshot tb_carla_axi_replay_snap
if errorlevel 1 goto :fail

call "%VIVADO_BIN%\xsim.bat" tb_carla_axi_replay_snap -tclbatch run_carla_axi_replay.tcl
if errorlevel 1 goto :fail

copy /Y "pl_replay_results.csv" "%~dp0..\..\verification_reports\pl_replay_results.csv" >nul
popd
echo Result: %~dp0..\..\verification_reports\pl_replay_results.csv
exit /b 0

:fail
set RC=%errorlevel%
popd
exit /b %RC%
