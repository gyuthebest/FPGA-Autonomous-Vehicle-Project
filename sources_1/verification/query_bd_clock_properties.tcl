set verification_dir [file dirname [file normalize [info script]]]
set repo_root [file normalize [file join $verification_dir .. ..]]
set project_file [file join $repo_root FPGA_project FPGA_project.xpr]
set bd_file [file join $repo_root FPGA_project FPGA_project.srcs sources_1 bd design_1 design_1.bd]
open_project $project_file
open_bd_design $bd_file

set ps [get_bd_cells zynq_ultra_ps_e_0]
puts "PS_CELL=$ps"
foreach property_name [lsort [list_property $ps]] {
    if {[regexp -nocase {PL0.*FREQ|FREQ.*PL0|PL_CLK0|CRL_APB} $property_name]} {
        puts "$property_name=[get_property $property_name $ps]"
    }
}

set controller [get_bd_cells top_controller_0]
puts "CONTROLLER_CELL=$controller"
foreach property_name [lsort [list_property $controller]] {
    if {[regexp -nocase {CLK_FREQ|SAMPLE_RATE|VLNV} $property_name]} {
        puts "$property_name=[get_property $property_name $controller]"
    }
}
close_project
