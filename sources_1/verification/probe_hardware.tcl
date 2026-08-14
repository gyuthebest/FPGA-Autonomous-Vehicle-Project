set report_dir [file normalize [file join [file dirname [info script]] ../../verification_reports/bringup]]
file mkdir $report_dir
set fp [open [file join $report_dir hardware_probe.txt] w]

open_hw_manager
connect_hw_server -url localhost:3121
set targets [get_hw_targets]
puts $fp "HW_TARGET_COUNT=[llength $targets]"
foreach t $targets {
    puts $fp "HW_TARGET=$t"
    if {![catch {open_hw_target $t} err]} {
        set devices [get_hw_devices]
        puts $fp "HW_DEVICE_COUNT=[llength $devices]"
        foreach d $devices {
            puts $fp "HW_DEVICE=$d PART=[get_property PART $d] PROGRAMMED=[get_property PROGRAM.HW_CFGMEM $d]"
        }
        close_hw_target
    } else {
        puts $fp "OPEN_ERROR=$err"
    }
}
close_hw_manager
close $fp
exit
