log_wave -recursive *
run all
set failures [get_value /tb_pl_full_verification/fail_count]
if {$failures != "0"} {
    puts "ERROR: self-checking testbench reported $failures failures"
    quit 1
}
quit 0
