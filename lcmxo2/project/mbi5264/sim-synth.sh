#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
bash ./synth.sh
yosys_data="$(yosys-config --datdir)"
iverilog -g2012 -Wall -DNO_INCLUDES -s tb_led -I "$yosys_data/lattice" \
    -o build-open/tb_led_synth \
    "$yosys_data/lattice/cells_sim_xo2.v" \
    build-open/led-synth.v sim/dcca_model.v sim/tb_led.v
vvp build-open/tb_led_synth
