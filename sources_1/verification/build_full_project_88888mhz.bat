@echo off
setlocal
set VIVADO_BIN=C:\Xilinx\Vivado\2022.2\bin
call "%VIVADO_BIN%\vivado.bat" -mode batch -nolog -nojournal ^
  -source "%~dp0build_full_project_88888mhz.tcl"
exit /b %errorlevel%
