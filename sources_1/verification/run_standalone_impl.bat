@echo off
setlocal
set VIVADO_BIN=C:\Xilinx\Vivado\2022.2\bin
call "%VIVADO_BIN%\vivado.bat" -mode batch -nolog -nojournal ^
  -source "%~dp0run_standalone_impl.tcl"
exit /b %errorlevel%
