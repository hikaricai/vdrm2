#!/usr/bin/env bash
set -euo pipefail

cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
export PATH="$PWD/../../toolchain/install/bin:$PATH"
bash ./synth.sh

nextpnr-machxo2 --device LCMXO2-2000HC-4TG144C \
    --json build-open/led.json --lpf board.lpf --freq 16.25 \
    --textcfg build-open/led.config \
    --write build-open/led-routed.json --log build-open/nextpnr.log
ecppack --compress build-open/led.config build-open/led-current-edge.bit
cp build-open/led-current-edge.bit build-open/led.bit

printf '\nBitstream: %s/build-open/led.bit\n' "$PWD"
