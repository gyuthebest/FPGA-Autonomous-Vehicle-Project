set verification_dir [file dirname [file normalize [info script]]]
set repo_root [file normalize [file join $verification_dir .. ..]]
set project_file [file join $repo_root FPGA_project FPGA_project.xpr]
set report_dir [file join $repo_root verification_reports full_project_88888mhz]

file mkdir $report_dir
open_project $project_file

set synth_status [get_property STATUS [get_runs synth_1]]
if {![string match "*Complete*" $synth_status]} {
    error "Cannot continue: synthesis is not complete ($synth_status)"
}

reset_run impl_1
launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1

set impl_status [get_property STATUS [get_runs impl_1]]
if {![string match "*Complete*" $impl_status]} {
    error "Implementation did not complete: $impl_status"
}

open_run impl_1
report_timing_summary -delay_type min_max -max_paths 20 \
    -file [file join $report_dir implemented_timing_summary.rpt]
report_utilization -file [file join $report_dir implemented_utilization.rpt]
report_drc -file [file join $report_dir implemented_drc.rpt]

set bit_file [file join $repo_root FPGA_project FPGA_project.runs impl_1 design_1_wrapper.bit]
if {![file exists $bit_file]} {
    error "Bitstream was not created: $bit_file"
}

write_hw_platform -fixed -include_bit -force \
    -file [file join $repo_root FPGA_project design_1_wrapper.xsa]
close_project
exit
