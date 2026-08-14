run all
set failures [get_value /tb_carla_axi_replay/fail_count]
if {$failures != "0"} {
    puts "ERROR: CARLA AXI replay reported $failures failures"
    quit 1
}
quit 0
