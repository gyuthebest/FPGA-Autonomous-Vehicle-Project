set repo_root [file normalize [file join [file dirname [info script]] ../..]]
set report_path [file join $repo_root verification_reports bringup a53_state.txt]
set fp [open $report_path w]

connect -url tcp:127.0.0.1:3121
targets -set -nocase -filter {name =~ "*Cortex-A53*#0*"}
if {[catch {state} cpu_state]} {
    puts $fp "A53_STATE_ERROR=$cpu_state"
} else {
    puts $fp "A53_STATE=$cpu_state"
}
if {[catch {rrd pc} pc]} {
    puts $fp "A53_PC_ERROR=$pc"
} else {
    puts $fp "A53_PC=$pc"
}
disconnect
close $fp
exit
