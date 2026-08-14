set repo_root [file normalize [file join [file dirname [info script]] ../..]]
set ws [file join $repo_root vitis_workspace]

setws $ws
app build -name carla_fpga_bridge

set elf [file join $ws carla_fpga_bridge Debug carla_fpga_bridge.elf]
if {![file exists $elf]} {
    error "ELF was not generated: $elf"
}
puts "ELF=$elf"
puts "ELF_SIZE=[file size $elf]"
exit
