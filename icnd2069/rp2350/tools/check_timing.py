#!/usr/bin/env python3
"""Verify real DMA buffers using the same build.rs JSON parser as firmware.

Run cargo build --release --locked first. Does not change config.json or hardware.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--config", type=Path, default=ROOT / "config.json")
config = json.loads(parser.parse_args().config.read_text())
builders = list((ROOT / "target/release/build").glob("icnd2069-demo-*/build-script-build"))
if not builders:
    raise SystemExit("Run cargo build --release --locked first")
builder = max(builders, key=lambda p: p.stat().st_mtime)
clean_env = {k: v for k, v in os.environ.items() if not k.startswith("ICND2069_")}


def run(command, **kwargs):
    result = subprocess.run(command, text=True, capture_output=True, **kwargs)
    if result.returncode:
        raise RuntimeError(result.stdout + result.stderr)
    return result.stdout


with tempfile.TemporaryDirectory(prefix="icnd2069-timing-check-") as temp:
    temp = Path(temp)
    binary = str(temp / "check")
    def build_env(settings):
        (temp / "config.json").write_text(json.dumps(settings))
        output = run([str(builder)], env=clean_env | {
            "CARGO_MANIFEST_DIR": str(temp), "OUT_DIR": str(temp)})
        fields = {}
        for line in output.splitlines():
            if line.startswith("cargo:rustc-env="):
                key, value = line.removeprefix("cargo:rustc-env=").split("=", 1)
                fields[key] = value
        return clean_env | fields

    def check(settings):
        env = build_env(settings)
        run(["rustc", "--edition=2021", "-O", str(ROOT / "tools/check_timing.rs"),
             "-o", binary], env=env)
        print(run([binary], env=env).strip(), flush=True)

    reference = dict(en_start=15, gray_start=732, row_start=28,
                     gray_word_gap=8, gray_row_gap=1, row_period=81, frame_tail=430)
    current = config | {"timing": config.get("timing", reference)}
    check(current)
    full = {k: v for k, v in current.items() if k != "repeat_latch"}
    check(full)
    for board in ("hdl", "dev", "direct"):
        for latch in (False, True):
            case = current | {"board": board, "chains": 2, "cfg_chains": 2,
                              "lines": 16, "scan_repeats": 1,
                              "pattern": "diagonal_rev"}
            if not latch:
                case.pop("repeat_latch", None)
            check(case)
    # Exact serial reference timing, on host only: 8 chains × 40 rows × 64 scans.
    check(full | {"board": "direct", "chains": 8, "cfg_chains": 8,
                  "lines": 40, "scan_repeats": 64, "keep_scan_time": False,
                  "timing": reference})
    # Every timing parameter differs: ensure values are consumed, not hardcoded.
    altered = dict(en_start=21, gray_start=900, row_start=40,
                   gray_word_gap=3, gray_row_gap=5, row_period=97, frame_tail=510)
    check(full | {"timing": altered})
    check(current | {"timing": altered, "keep_scan_time": True, "disable_rt5990": True})
    check(full | {"lines": 1, "scan_repeats": 1})

    # Invalid ordering/periods must fail at build time rather than on the device.
    for field, value, message in [
        ("en_start", 3, "timing.en_start"),
        ("gray_start", 16, "timing.gray_start"),
        ("row_start", 10, "timing.row_start"),
        ("row_period", 13, "minimum row period"),
    ]:
        env = build_env(current | {"timing": reference | {field: value}})
        result = subprocess.run(["rustc", "--edition=2021", str(ROOT / "tools/check_timing.rs"),
                                 "-o", binary], env=env, text=True, capture_output=True)
        assert result.returncode != 0 and message in result.stderr, result.stdout + result.stderr
    print("PASS invalid timing overlap/period rejected", flush=True)

    # Byte-for-byte regression when timing is omitted, all boards.
    baseline = temp / "baseline.rs"
    baseline.write_text(run(["git", "show", "HEAD:icnd2069/rp2350/src/protocol.rs"], cwd=ROOT))
    template = '''#![allow(dead_code)]
#[path = {source}] mod protocol;
use protocol::*;
use std::io::Write;
fn main() {{
    let image = pattern_frame_range(Pattern::DiagonalReverse, [255, 31]);
    let mut config = vec![0; FRAME_WORDS + CFG_POST_CLR_DCLKS];
    let mut normal = vec![0; NORMAL_FRAME_DCLKS];
    prepare_frames(&mut config, &mut normal, &image);
    let mut stdout = std::io::stdout().lock();
    for word in config.iter().chain(normal.iter()) {{
        stdout.write_all(&word.to_le_bytes()).unwrap();
    }}
}}
'''
    for board in ("hdl", "dev", "direct"):
        legacy = {k: v for k, v in current.items() if k != "timing"}
        env = build_env(legacy | {"board": board, "pattern": "diagonal_rev"})
        outputs = []
        for source in (baseline, ROOT / "src/protocol.rs"):
            code = template.format(source=json.dumps(str(source)))
            run(["rustc", "--edition=2021", "--crate-name", "dump", "-O", "-", "-o", binary],
                input=code, env=env)
            outputs.append(subprocess.check_output([binary]))
        assert outputs[0] == outputs[1], board
        print(f"PASS {board}: timing omitted, {len(outputs[0])} DMA bytes match HEAD", flush=True)
