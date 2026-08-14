set verification_dir [file dirname [file normalize [info script]]]
set source_root [file normalize [file join $verification_dir ..]]
set report_dir [file normalize [file join $verification_dir .. .. verification_reports standalone_synth]]
file mkdir $report_dir

read_verilog -sv [list [file join $source_root new types_pkg.sv]]
read_verilog -sv [list [file join $source_root new preprocessor.sv]]
read_verilog -sv [list [file join $source_root new consistency_checker.sv]]
read_verilog -sv [list [file join $source_root new mask_20s.sv]]
read_verilog -sv [list [file join $source_root new sensor_checker.sv]]
read_verilog -sv [list [file join $source_root new sensor_reliability.sv]]
read_verilog -sv [list [file join $source_root new risk_types.sv]]
read_verilog -sv [list [file join $source_root new risk_control.sv]]
read_verilog -sv [list [file join $source_root ip sensor_input_1_0 hdl sensor_input_v1_0_S00_AXI.v]]
read_verilog -sv [list [file join $source_root new top_controller.sv]]

synth_design -top top_controller -part xczu2cg-sfvc784-1-e \
    -generic CLK_FREQ_HZ=100000000 -generic SAMPLE_RATE_HZ=20
create_clock -name s_axi_aclk -period 10.000 [get_ports S_AXI_ACLK]
report_utilization -file [file join $report_dir standalone_utilization.rpt]
report_timing_summary -file [file join $report_dir standalone_timing_summary.rpt]
write_checkpoint -force [file join $report_dir standalone_top_controller.dcp]
puts "STANDALONE_SYNTHESIS=PASS"
