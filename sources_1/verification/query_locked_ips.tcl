set verification_dir [file dirname [file normalize [info script]]]
set repo_root [file normalize [file join $verification_dir .. ..]]
set project_file [file join $repo_root FPGA_project FPGA_project.xpr]
set bd_file [file join $repo_root FPGA_project FPGA_project.srcs sources_1 bd design_1 design_1.bd]
open_project $project_file
set_property ip_repo_paths [list $repo_root] [current_project]
update_ip_catalog -rebuild
open_bd_design $bd_file
puts "GET_IPS_DEFAULT=[get_ips -quiet *]"
puts "GET_IPS_ALL=[get_ips -all -quiet *]"
puts "GET_IPS_CONTROLLER_ALL=[get_ips -all -quiet *top_controller*]"
puts "BD_CELLS=[get_bd_cells -quiet *]"
report_ip_status -file [file join $repo_root verification_reports ip_status_before_upgrade.rpt]
close_project
