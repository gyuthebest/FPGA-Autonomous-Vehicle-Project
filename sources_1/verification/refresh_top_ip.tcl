set d [file normalize [file dirname [info script]]]
set root [file normalize [file join $d .. ..]]
open_project [file join $root FPGA_project FPGA_project.xpr]
update_ip_catalog -rebuild
set bd [get_files */design_1.bd]
reset_target all $bd
generate_target all $bd
export_ip_user_files -of_objects $bd -sync -force -quiet
puts "TOP_IP_REFRESH_COMPLETE"
close_project
