#!/usr/bin/env bash
set -euo pipefail

cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
export PATH="$PWD/../../toolchain/install/bin:$PATH"
mkdir -p build-open/state
export XDG_STATE_HOME="$PWD/build-open/state"

yosys -Q -T -l build-open/synthesis.log \
    -p 'read_verilog -sv rtl/main.sv; synth_lattice -family xo2 -top io_test -json build-open/io_test.json; check -assert; write_verilog -noattr build-open/io_test-synth.v'

nextpnr-machxo2 --device LCMXO2-2000HC-4TG144I \
    --json build-open/io_test.json --lpf board.lpf --freq 2.08 \
    --textcfg build-open/io_test.config \
    --write build-open/io_test-routed.json --log build-open/nextpnr.log

ecppack --compress build-open/io_test.config build-open/io_test.bit

printf '\nBitstream: %s/build-open/io_test.bit\n' "$PWD"
