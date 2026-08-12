# Vivado 2022.2 out-of-context verification for the complete PL pipeline.
set script_dir {C:/Users/kim05/FPGA Project/FPGA Project.srcs/sources_1}
set report_dir [file join $script_dir verification_reports]
file mkdir $report_dir

create_project -in_memory -part xczu2cg-sfvc784-1-e
set_property target_language Verilog [current_project]

read_verilog -sv [list \
    [file join $script_dir new types_pkg.sv] \
    [file join $script_dir new consistency_checker.sv] \
    [file join $script_dir new mask_20s.sv] \
    [file join $script_dir new preprocessor.sv] \
    [file join $script_dir new sensor_checker.sv] \
    [file join $script_dir new sensor_reliability.sv] \
    [file join $script_dir new risk_types.sv] \
    [file join $script_dir new risk_control.sv] \
    [file join $script_dir ip sensor_input_1_0 hdl sensor_input_v1_0_S00_AXI.v] \
    [file join $script_dir new top_controller.sv]]

if {[catch {
    synth_design -top top_controller -part xczu2cg-sfvc784-1-e -flatten_hierarchy rebuilt
} synth_error]} {
    # Vivado 2022.2 can report a Windows realtime/tmp cleanup error after a
    # successful synthesis. Continue only when an in-memory design exists.
    if {[llength [get_designs -quiet]] == 0} {
        error $synth_error
    }
    puts "WARNING: continuing after post-synthesis cleanup error: $synth_error"
}
create_clock -name S_AXI_ACLK -period 10.000 [get_ports S_AXI_ACLK]

check_timing -file [file join $report_dir check_timing.rpt]
report_timing_summary -delay_type min_max -report_unconstrained \
    -file [file join $report_dir timing_summary.rpt]
report_utilization -file [file join $report_dir utilization.rpt]
report_drc -file [file join $report_dir drc.rpt]

write_checkpoint -force [file join $report_dir top_controller_synth.dcp]
puts "SOURCE_VERIFICATION_COMPLETE"
