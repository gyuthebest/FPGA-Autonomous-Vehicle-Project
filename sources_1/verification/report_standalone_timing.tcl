set verification_dir [file dirname [file normalize [info script]]]
set report_dir [file normalize [file join $verification_dir .. .. verification_reports standalone_synth]]
open_checkpoint [file join $report_dir standalone_top_controller.dcp]
create_clock -name s_axi_aclk -period 10.000 [get_ports S_AXI_ACLK]
report_timing_summary -delay_type min_max -max_paths 20 \
    -file [file join $report_dir standalone_timing_100mhz.rpt]
puts "STANDALONE_TIMING_100MHZ=PASS"
