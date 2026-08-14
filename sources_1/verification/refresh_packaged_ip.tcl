# Run only after every Vivado GUI using FPGA_project.xpr has been saved/closed.
set verification_dir [file dirname [file normalize [info script]]]
set repo_root [file normalize [file join $verification_dir .. ..]]
set component_file [file join $repo_root component.xml]
set project_file [file join $repo_root FPGA_project FPGA_project.xpr]
set bd_file [file join $repo_root FPGA_project FPGA_project.srcs sources_1 bd design_1 design_1.bd]

puts "Refreshing packaged core: $component_file"
ipx::open_core $component_file
set core [ipx::current_core]
ipx::update_checksums $core
ipx::save_core $core
ipx::unload_core $core

puts "Refreshing Block Design project: $project_file"
open_project $project_file
set_property ip_repo_paths [list $repo_root] [current_project]
update_ip_catalog -rebuild
open_bd_design $bd_file

set controller_ips [get_ips -all -quiet *top_controller*]
if {[llength $controller_ips] < 1} {
    error "No top_controller IP instance found"
}
upgrade_ip $controller_ips

set ps [get_bd_cells zynq_ultra_ps_e_0]
set controller [get_bd_cells top_controller_0]
set_property CONFIG.PSU__CRL_APB__PL0_REF_CTRL__FREQMHZ 90 $ps
set_property CONFIG.CLK_FREQ_HZ 88888000 $controller
set_property CONFIG.SAMPLE_RATE_HZ 20 $controller
validate_bd_design
set actual_pl0_mhz [get_property CONFIG.PSU__CRL_APB__PL0_REF_CTRL__ACT_FREQMHZ $ps]
puts "PL0_ACT_FREQMHZ=$actual_pl0_mhz"
if {$actual_pl0_mhz != "88.888000"} {
    error "Unexpected PL0 actual frequency: $actual_pl0_mhz MHz"
}
save_bd_design
generate_target all [get_files $bd_file]
export_ip_user_files -of_objects [get_files $bd_file] -no_script -sync -force -quiet
report_ip_status -file [file join $repo_root verification_reports ip_status_after_refresh.rpt]
close_project
puts "PACKAGED_IP_REFRESH=PASS"
