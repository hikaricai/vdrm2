#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
mkdir -p build-open
iverilog -g2012 -Wall -s tb_led -o build-open/tb_led \
    rtl/main.sv rtl/select_address.sv rtl/decoder.sv sim/tb_led.v
vvp build-open/tb_led
