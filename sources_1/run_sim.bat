@echo off
set VIVADO_BIN=C:\Xilinx\Vivado\2022.2\bin

echo Compiling sources...
call "%VIVADO_BIN%\xvlog.bat" -sv new\types_pkg.sv new\preprocessor.sv new\consistency_checker.sv new\mask_20s.sv new\sensor_checker.sv new\sensor_reliability.sv new\risk_types.sv new\risk_control.sv tb_core_logic.sv
if %errorlevel% neq 0 exit /b %errorlevel%

echo Elaborating design...
call "%VIVADO_BIN%\xelab.bat" -debug typical -top tb_core_logic -snapshot tb_core_logic_snap
if %errorlevel% neq 0 exit /b %errorlevel%

echo Running simulation...
call "%VIVADO_BIN%\xsim.bat" tb_core_logic_snap -tclbatch run.tcl
if %errorlevel% neq 0 exit /b %errorlevel%

echo Simulation complete.
