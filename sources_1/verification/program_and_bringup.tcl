set repo_root [file normalize [file join [file dirname [info script]] ../..]]
set report_dir [file join $repo_root verification_reports bringup]
set bit [file join $repo_root FPGA_project FPGA_project.runs impl_1 design_1_wrapper.bit]
set elf [file join $repo_root vitis_workspace carla_fpga_bridge Debug carla_fpga_bridge.elf]
set psu_init_tcl [file join $repo_root vitis_workspace carla_fpga_platform hw psu_init.tcl]
set log_path [file join $report_dir program_and_bringup.log]

file mkdir $report_dir
set log_fp [open $log_path w]

proc note {message} {
    global log_fp
    puts $message
    puts $log_fp $message
    flush $log_fp
}

proc select_target {filter description} {
    for {set retry 0} {$retry < 10} {incr retry} {
        if {![catch {targets -set -nocase -filter $filter} result]} {
            note "TARGET_SELECTED=$description"
            return
        }
        after 500
    }
    error "Unable to select $description with filter: $filter"
}

foreach required [list $bit $elf $psu_init_tcl] {
    if {![file exists $required]} {
        error "Required bring-up artifact is missing: $required"
    }
}

if {[catch {
    note "BRINGUP_BEGIN=[clock format [clock seconds] -format {%Y-%m-%d %H:%M:%S}]"
    note "BIT=$bit"
    note "ELF=$elf"

    connect -url tcp:127.0.0.1:3121
    note "TCF_CONNECTED=1"

    select_target {name =~ "*PSU*"} "PSU"
    rst -system
    after 3000

    select_target {name =~ "*PSU*"} "PSU_AFTER_RESET"
    source $psu_init_tcl
    psu_init
    psu_post_config
    note "PS_INITIALIZED=1"

    fpga -file $bit
    after 1000
    note "BITSTREAM_PROGRAMMED=1"

    # The generated psu_post_config is empty for this XSA. PL power/isolation
    # and fabric reset release are emitted as separate procedures.
    psu_ps_pl_isolation_removal
    psu_ps_pl_reset_config
    after 1000
    note "PL_ISOLATION_AND_RESET_RELEASED=1"

    # Keep the PSU debug target selected for the AXI test. The A53 is still
    # reset at this point, so issuing `stop` to it would fail on the L2 reset.
    select_target {name =~ "*PSU*"} "PSU_FOR_AXI_TEST"

    # AXI smoke test: commit a harmless sensor frame with a known sequence.
    # REG8.manual_mode is asserted for this frame so the synchronous TD/MRM
    # state is initialized even when PL configuration did not clock the reset
    # branch.  The first live CARLA frame returns manual_mode to its real value.
    for {set address 0x80000000} {$address <= 0x80000020} {incr address 4} {
        mwr -force $address 0x00000000
    }
    mwr -force 0x80000020 0x00000001
    set test_seq 0xA5A50001
    mwr -force 0x80000024 $test_seq
    after 100
    set risk_seq [mrd -force -value 0x8000002C]
    set rel_seq  [mrd -force -value 0x80000030]
    note [format "AXI_RISK_SEQ=0x%08X" $risk_seq]
    note [format "AXI_REL_SEQ=0x%08X" $rel_seq]
    if {$risk_seq != $test_seq || $rel_seq != $test_seq} {
        error [format "AXI sequence smoke test failed: expected 0x%08X" $test_seq]
    }
    note "AXI_SMOKE_PASS=1"

    select_target {name =~ "*Cortex-A53*#0*"} "CORTEX_A53_0"
    rst -processor
    dow $elf
    note "ELF_DOWNLOADED=1"
    con
    note "A53_RUNNING=1"
    after 2000

    disconnect
    note "BRINGUP_COMPLETE=1"
    note "BRINGUP_END=[clock format [clock seconds] -format {%Y-%m-%d %H:%M:%S}]"
} bringup_error bringup_options]} {
    note "BRINGUP_COMPLETE=0"
    note "BRINGUP_ERROR=$bringup_error"
    catch {disconnect}
    close $log_fp
    puts stderr $bringup_error
    exit 1
}

close $log_fp
exit
