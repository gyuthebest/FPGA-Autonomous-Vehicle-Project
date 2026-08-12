# Board preset loader for this project.
#
# The seminar project in Downloads/week3 (2) was programmed successfully on
# the target board.  Load that exact generated Zynq UltraScale+ MPSoC preset
# instead of maintaining a second, easily-diverged copy of the DDR/MIO data.

set _reference_preset {C:/Users/kim05/Downloads/week3 (2)/board_preset.tcl}

if {![file isfile $_reference_preset]} {
    error "Validated week3 board preset was not found: $_reference_preset"
}

source $_reference_preset
unset _reference_preset
