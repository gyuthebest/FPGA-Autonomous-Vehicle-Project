set repo_root [file normalize [file join [file dirname [info script]] ../..]]
set ws        [file join $repo_root vitis_workspace]
set xsa       [file join $repo_root FPGA_project design_1_wrapper.xsa]

if {![file exists $xsa]} {
    error "XSA not found: $xsa"
}

setws $ws
platform create -name carla_fpga_platform -hw $xsa \
    -proc psu_cortexa53_0 -os standalone
platform write
platform generate

app create -name carla_fpga_bridge \
    -platform carla_fpga_platform \
    -domain standalone_domain \
    -template {Empty Application}

bsp setlib -name lwip211
bsp config stdin psu_uart_1
bsp config stdout psu_uart_1
bsp regenerate

puts "VITIS_WORKSPACE=$ws"
puts "APP_SOURCE=[file join $ws carla_fpga_bridge src]"
exit
