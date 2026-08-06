# ============================================================
# 모듈 단독 타이밍 측정 (Out-of-Context 합성)
#   사용법 :  vivado -mode batch -source timing_ooc.tcl -tclargs stuck_check 10.0
#            (모듈명, 목표 클럭주기 ns)
# ============================================================

set PART   xczu2cg-sfvc784-1-e     ;# TODO: 실제 부품번호로 교체
set MODULE [lindex $argv 0]
set PERIOD [lindex $argv 1]
if {$MODULE eq ""} { set MODULE stuck_check }
if {$PERIOD eq ""} { set PERIOD 10.0 }

set SRC [file normalize [file join [file dirname [info script]] .. sources_1 new]]

read_verilog -sv [glob $SRC/types_pkg.sv]
read_verilog -sv [glob $SRC/*.sv]

synth_design -top $MODULE -part $PART -mode out_of_context

create_clock -period $PERIOD -name clk [get_ports clk]
set_input_delay  0 -clock clk [all_inputs]
set_output_delay 0 -clock clk [all_outputs]

puts "\n================ $MODULE  (target ${PERIOD}ns) ================"
report_timing_summary -delay_type max -no_header
puts "\n---------------- critical path ----------------"
report_timing -delay_type max -max_paths 3 -nworst 1 -input_pins
puts "\n---------------- 로직 단수 분포 ----------------"
report_design_analysis -logic_level_distribution
puts "\n---------------- 자원 ----------------"
report_utilization -hierarchical
