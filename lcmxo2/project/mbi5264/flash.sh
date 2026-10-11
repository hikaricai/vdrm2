#!/usr/bin/env bash
set -euo pipefail

cd -- "$(dirname -- "${BASH_SOURCE[0]}")"

# Stop on a failed build; never program a stale bitstream.
bash ./build.sh

openFPGALoader -c dirtyJtag --freq 100000 --detect \
    2>&1 | tee build-open/detect.log
if ! grep -q 'idcode 0x12bb043' build-open/detect.log; then
    printf 'Expected LCMXO2-2000HC IDCODE 0x012bb043; aborting.\n' >&2
    exit 1
fi
OPENFPGALOADER_MACHXO2_FEABITS=0x620 \
openFPGALoader -c dirtyJtag --freq 100000 -f --verify build-open/led.bit \
    2>&1 | tee build-open/program.log
