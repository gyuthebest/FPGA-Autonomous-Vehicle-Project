set script_dir [file dirname [file normalize [info script]]]
set repo_root [file normalize [file join $script_dir .. ..]]
set bit_file [file join $repo_root FPGA_project FPGA_project.runs impl_1 design_1_wrapper.bit]
if {![file exists $bit_file]} {
    error "Bitstream not found: $bit_file"
}

open_hw_manager
connect_hw_server -url localhost:3121
set opened 0
foreach target [get_hw_targets] {
    if {![catch {open_hw_target $target}]} {
        set opened 1
        break
    }
}
if {!$opened} { error "No JTAG hardware target could be opened" }

set device [lindex [get_hw_devices -filter {PART =~ "xczu2*"}] 0]
if {$device eq ""} { error "xczu2 device was not found" }
current_hw_device $device
refresh_hw_device -update_hw_probes false $device
set_property PROGRAM.FILE $bit_file $device
program_hw_devices $device
refresh_hw_device $device
puts "PROGRAMMED_DEVICE=$device"
puts "PROGRAMMED_BITSTREAM=$bit_file"
close_hw_manager
