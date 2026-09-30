# Run with Diamond's diamondc (Linux) or pnmainc.exe (Windows).
# Reference command flow: LiteX litex/build/lattice/diamond.py.
set source_dir [file dirname [file normalize [info script]]]
set build_dir [file join $source_dir build-diamond]
file mkdir $build_dir
cd $build_dir

set project_file [file join $build_dir led_chaser.ldf]
if {[file exists $project_file]} {
    prj_project open $project_file
} else {
    prj_project new -name led_chaser -impl impl \
        -dev LCMXO2-2000HC-4TG100C -synthesis synplify
    prj_src add [file join $source_dir led_chaser.v] -work work
    prj_src add [file join $source_dir board.lpf]
    prj_impl option top led_chaser
    prj_project save
}

prj_run Synthesis -impl impl -forceOne
prj_run Translate -impl impl
prj_run Map -impl impl
prj_run PAR -impl impl
prj_run Export -impl impl -task Bitgen
prj_run Export -impl impl -task Jedecgen
prj_project close

set jed_file [file join $build_dir impl led_chaser_impl.jed]
if {![file exists $jed_file] || [file size $jed_file] == 0} {
    error "No JEDEC file generated at $jed_file; inspect the Diamond build reports."
}
puts "JEDEC: $jed_file"
puts "Check the pin and timing reports before programming."
