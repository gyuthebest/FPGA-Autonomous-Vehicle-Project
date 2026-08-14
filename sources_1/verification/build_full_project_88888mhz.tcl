set verification_dir [file dirname [file normalize [info script]]]
set repo_root [file normalize [file join $verification_dir .. ..]]
set project_file [file join $repo_root FPGA_project FPGA_project.xpr]
set report_dir [file join $repo_root verification_reports full_project_88888mhz]
file mkdir $report_dir

open_project $project_file
config_ip_cache -disable_cache

# Refresh the generated Block Design IP copy from the packaged-IP repository.
# Without this, ipshared/<hash> can retain old HDL even when the OOC run is
# reset, producing a successful but stale bitstream.
update_ip_catalog -rebuild
set design_bd [get_files -quiet */design_1.bd]
if {[llength $design_bd] != 0} {
    reset_target all $design_bd
    generate_target all $design_bd
    export_ip_user_files -of_objects $design_bd -sync -force -quiet
}

# The block design consumes top_controller through an out-of-context IP run.
# Reset it explicitly; resetting only synth_1 can silently reuse a stale DCP
# after editing the packaged SystemVerilog sources.
set top_ip_run [get_runs -quiet design_1_top_controller_0_0_synth_1]
if {[llength $top_ip_run] != 0} {
    reset_run $top_ip_run
}
reset_run synth_1
launch_runs synth_1 -jobs 4
wait_on_run synth_1
set synth_status [get_property STATUS [get_runs synth_1]]
puts "SYNTH_STATUS=$synth_status"
if {![string match "*Complete*" $synth_status]} {
    error "Synthesis did not complete: $synth_status"
}

open_run synth_1
report_utilization -file [file join $report_dir post_synth_utilization.rpt]
report_timing_summary -file [file join $report_dir post_synth_timing.rpt]
close_design

launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1
set impl_status [get_property STATUS [get_runs impl_1]]
puts "IMPL_STATUS=$impl_status"
if {![string match "*Complete*" $impl_status]} {
    error "Implementation/bitstream did not complete: $impl_status"
}

open_run impl_1
report_timing_summary -delay_type min_max -max_paths 20 \
    -file [file join $report_dir implemented_timing_summary.rpt]
report_utilization -file [file join $report_dir implemented_utilization.rpt]
report_drc -file [file join $report_dir implemented_drc.rpt]
report_clock_utilization -file [file join $report_dir clock_utilization.rpt]

# --- Hard timing gate -------------------------------------------------------
# The build must fail loudly on any negative slack. A bitstream that closes
# implementation but violates setup/hold will produce intermittent, extremely
# hard to diagnose behaviour on the board, so it must never reach programming.
set wns [get_property STATS.WNS [get_runs impl_1]]
set tns [get_property STATS.TNS [get_runs impl_1]]
set whs [get_property STATS.WHS [get_runs impl_1]]
set ths [get_property STATS.THS [get_runs impl_1]]
set failing [get_property STATS.FAILED_NETS [get_runs impl_1]]

puts "TIMING_SETUP_WNS=$wns"
puts "TIMING_SETUP_TNS=$tns"
puts "TIMING_HOLD_WHS=$whs"
puts "TIMING_HOLD_THS=$ths"

set timing_errors [list]
if {$wns < 0} { lappend timing_errors "setup WNS=$wns ns" }
if {$tns < 0} { lappend timing_errors "setup TNS=$tns ns" }
if {$whs < 0} { lappend timing_errors "hold WHS=$whs ns" }
if {$ths < 0} { lappend timing_errors "hold THS=$ths ns" }
if {[llength $timing_errors] != 0} {
    puts "TIMING_CLOSURE=FAIL"
    error "Timing not met: [join $timing_errors {, }] -- see implemented_timing_summary.rpt"
}
puts "TIMING_CLOSURE=PASS"

# --- DRC gate ---------------------------------------------------------------
# get_drc_violations has no -severity option in Vivado 2022.2; severity is a
# property of the returned objects.  The old form raised
#   ERROR: [Common 17-170] Unknown option '-severity'
# which aborted the build *after* the bitstream was written but *before*
# write_hw_platform, so the XSA silently went stale.
set drc_errors [get_drc_violations -quiet -filter {SEVERITY == "Error"}]
set drc_critical [get_drc_violations -quiet -filter {SEVERITY == "Critical Warning"}]
puts "DRC_ERRORS=[llength $drc_errors]"
puts "DRC_CRITICAL_WARNINGS=[llength $drc_critical]"
if {[llength $drc_errors] != 0} {
    error "DRC reported [llength $drc_errors] error(s) -- see implemented_drc.rpt"
}

set bit_file [file join $repo_root FPGA_project FPGA_project.runs impl_1 design_1_wrapper.bit]
if {![file exists $bit_file]} {
    error "Bitstream was not created: $bit_file"
}
write_hw_platform -fixed -include_bit -force \
    -file [file join $repo_root FPGA_project design_1_wrapper.xsa]
puts "BITSTREAM_FILE=$bit_file"
puts "FULL_PROJECT_BUILD=PASS"
close_project
