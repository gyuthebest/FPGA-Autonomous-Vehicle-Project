@echo off
set XILINX_VIVADO=C:\Xilinx\Vivado\2022.2

echo Compiling sources...
call %XILINX_VIVADO%\bin\xvlog -sv timescale.sv ..\new\types_pkg.sv
call %XILINX_VIVADO%\bin\xvlog -sv timescale.sv ..\new\preprocessor.sv
call %XILINX_VIVADO%\bin\xvlog -sv timescale.sv ..\new\consistency_checker.sv
call %XILINX_VIVADO%\bin\xvlog -sv timescale.sv ..\new\mask_20s.sv
call %XILINX_VIVADO%\bin\xvlog -sv timescale.sv ..\new\sensor_checker.sv
call %XILINX_VIVADO%\bin\xvlog -sv timescale.sv ..\new\sensor_reliability.sv
call %XILINX_VIVADO%\bin\xvlog -sv timescale.sv ..\new\risk_types.sv
call %XILINX_VIVADO%\bin\xvlog -sv timescale.sv ..\new\risk_control.sv
call %XILINX_VIVADO%\bin\xvlog -sv timescale.sv risk_control_wrapper.sv
call %XILINX_VIVADO%\bin\xvlog -sv timescale.sv ..\new\top_controller.sv
call %XILINX_VIVADO%\bin\xvlog -sv timescale.sv ..\ip\sensor_input_1_0\hdl\sensor_input_v1_0_S00_AXI.v
call %XILINX_VIVADO%\bin\xvlog -sv timescale.sv tb_top_axi_integration.sv

echo Elaborating design...
call %XILINX_VIVADO%\bin\xelab -debug typical -top tb_top_axi_integration -snapshot tb_top_axi_integration_snap

echo Running simulation...
call %XILINX_VIVADO%\bin\xsim tb_top_axi_integration_snap -tclbatch run_sim.tcl

if %errorlevel% neq 0 (
    echo Simulation failed.
    exit /b %errorlevel%
)

echo Simulation complete.
