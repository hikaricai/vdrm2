#!/usr/bin/env bash
set -euo pipefail

cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
export PATH="$PWD/toolchain/install/bin:$PATH"
mkdir -p build-open/state
export XDG_STATE_HOME="$PWD/build-open/state"

for tool in yosys nextpnr-machxo2 ecppack; do
    if ! command -v "$tool" >/dev/null 2>&1; then
        printf 'Missing tool: %s; see README.md\n' "$tool" >&2
        exit 1
    fi
done

yosys -Q -T -l build-open/synthesis.log \
    -p 'read_verilog led_chaser.v; synth_lattice -family xo2 -top led_chaser -json build-open/led_chaser.json'
nextpnr-machxo2 --device LCMXO2-2000HC-4TG100C \
    --json build-open/led_chaser.json --lpf board.lpf \
    --textcfg build-open/led_chaser.config \
    --write build-open/led_chaser-routed.json \
    --log build-open/nextpnr.log
# openFPGALoader 1.0.0 requires compressed MachXO2 .bit for internal Flash.
ecppack --compress build-open/led_chaser.config build-open/led_chaser.bit

printf '\nBitstream: %s/build-open/led_chaser.bit\n' "$PWD"
