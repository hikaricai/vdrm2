#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
rustup toolchain install 1.98.0 --profile minimal --component rustfmt --component clippy
rustup target add --toolchain 1.98.0 riscv32imafc-unknown-none-elf
cargo install espflash --version 4.6.0 --locked --root .tools
python3 -m venv .venv
.venv/bin/python -m pip install pyserial==3.5
cargo build --release --locked
