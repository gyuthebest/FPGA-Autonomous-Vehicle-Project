set d [file normalize [file dirname [info script]]]
set s [file normalize [file join $d .. new]]
create_project distance_debug [file normalize [file join $d .. .. verification_reports distance_debug_proj]] -force -part xczu2cg-sfvc784-1-e
read_verilog -sv [list \
    [file join $s types_pkg.sv] \
    [file join $s consistency_checker.sv] \
    [file join $s mask_20s.sv] \
    [file join $s sensor_checker.sv] \
    [file join $s sensor_reliability.sv] \
    [file join $d tb_distance_debug.sv]]
set_property top tb_distance_debug [current_fileset -simset]
launch_simulation
run all
