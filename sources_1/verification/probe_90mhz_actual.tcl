set verification_dir [file dirname [file normalize [info script]]]
set repo_root [file normalize [file join $verification_dir .. ..]]
set project_file [file join $repo_root FPGA_project FPGA_project.xpr]
set bd_file [file join $repo_root FPGA_project FPGA_project.srcs sources_1 bd design_1 design_1.bd]
open_project $project_file
open_bd_design $bd_file
set ps [get_bd_cells zynq_ultra_ps_e_0]
set_property CONFIG.PSU__CRL_APB__PL0_REF_CTRL__FREQMHZ 90 $ps
validate_bd_design
puts "PL0_REQUEST_FREQMHZ=[get_property CONFIG.PSU__CRL_APB__PL0_REF_CTRL__FREQMHZ $ps]"
puts "PL0_ACT_FREQMHZ=[get_property CONFIG.PSU__CRL_APB__PL0_REF_CTRL__ACT_FREQMHZ $ps]"
puts "PL0_DIVISOR0=[get_property CONFIG.PSU__CRL_APB__PL0_REF_CTRL__DIVISOR0 $ps]"
puts "PL0_DIVISOR1=[get_property CONFIG.PSU__CRL_APB__PL0_REF_CTRL__DIVISOR1 $ps]"
puts "PL0_SRCSEL=[get_property CONFIG.PSU__CRL_APB__PL0_REF_CTRL__SRCSEL $ps]"
close_project
