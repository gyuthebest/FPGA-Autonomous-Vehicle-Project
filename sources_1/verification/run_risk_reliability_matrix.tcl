run all
set failures [get_value /tb_risk_reliability_matrix/fail_count]
if {$failures != "0"} {
    puts "ERROR: risk/reliability matrix reported $failures failures"
    quit 1
}
quit 0
