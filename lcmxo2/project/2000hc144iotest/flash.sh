#!/usr/bin/env bash
set -euo pipefail

cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
loader="$PWD/../../toolchain/build-loader/openFPGALoader"

if [[ ! -x "$loader" ]]; then
    printf 'Missing patched programmer: %s\nRun: cmake --build lcmxo2/toolchain/build-loader\n' \
        "$loader" >&2
    exit 1
fi

# Stop on a failed build and verify the connected part before erasing Flash.
bash ./build.sh
"$loader" -c dirtyJtag --freq 100000 --detect \
    2>&1 | tee build-open/detect.log
if ! grep -q 'idcode 0x12bb043' build-open/detect.log; then
    printf 'Expected LCMXO2-2000HC IDCODE 0x012bb043; aborting.\n' >&2
    exit 1
fi

OPENFPGALOADER_MACHXO2_FEABITS=0x620 \
"$loader" -c dirtyJtag --freq 100000 -f --verify \
    build-open/io_test.bit 2>&1 | tee build-open/program.log
