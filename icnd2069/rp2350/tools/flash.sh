#!/bin/bash
set -euo pipefail

# Run from icnd2069/rp2350: bash tools/flash.sh
if [[ $# -ne 0 ]]; then
    echo "Usage: bash tools/flash.sh (edit config.json to change parameters)" >&2
    exit 2
fi
if [[ ! -f Cargo.toml || ! -f src/protocol.rs ]]; then
    echo "Run this script from icnd2069/rp2350." >&2
    exit 2
fi

echo "Build and flash using config.json"
cargo build --release --locked
probe-rs download --chip RP235x --protocol swd --speed 1000 --verify \
    target/thumbv8m.main-none-eabihf/release/icnd2069-demo
probe-rs reset --chip RP235x --protocol swd --speed 1000
echo "Flashed and reset using config.json"
