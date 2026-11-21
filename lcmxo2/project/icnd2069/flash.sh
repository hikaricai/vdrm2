#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
bash ./build.sh
openFPGALoader -c dirtyJtag --freq 100000 --detect \
    2>&1 | tee build-open/detect.log
if ! grep -qi 'idcode 0x12ba043' build-open/detect.log; then
    printf 'Expected LCMXO2-1200HC IDCODE 0x012ba043; aborting.\n' >&2
    exit 1
fi
OPENFPGALOADER_MACHXO2_FEABITS=0x620 \
openFPGALoader -c dirtyJtag --freq 100000 -f --verify build-open/led.bit \
    2>&1 | tee build-open/program.log
