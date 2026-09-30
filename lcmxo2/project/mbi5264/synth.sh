#!/usr/bin/env bash
set -euo pipefail

cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
export PATH="$PWD/../../toolchain/install/bin:$PATH"
mkdir -p build-open/state
export XDG_STATE_HOME="$PWD/build-open/state"

yosys -Q -T -l build-open/synthesis.log \
    -p 'read_verilog -sv rtl/decoder.sv rtl/hc595.sv rtl/main.sv; synth_lattice -family xo2 -top led -json build-open/led.json; check -assert'

printf '\nSynthesized core: %s/build-open/led.json\n' "$PWD"
