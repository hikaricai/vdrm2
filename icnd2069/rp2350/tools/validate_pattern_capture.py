#!/usr/bin/env python3
"""Build/flash one static pattern and archive a 50ms DSView capture.

Run with the NumPy interpreter used by inspect_dsl.py. Existing --capture
imports evidence without touching hardware. Every artifact refuses overwrite.
"""
import argparse
import contextlib
import hashlib
import json
import os
import re
from pathlib import Path
import shutil
import subprocess

import numpy as np
from inspect_dsl import inspect


def image(pattern, gray, rows, channels):
    def on(row, out):
        return {
            "reference": out != channels - 1, "white": True, "black": False,
            "out0": out == 0, "out1": out == 1,
            "alternating_rows": row % 2 == 0,
            "checker": (row + out) % 2 == 0,
            "split": row < rows // 2,
        }[pattern]
    return np.array([gray if on(r, c) else 0
                     for r in range(rows) for c in range(channels - 1, -1, -1)], dtype="<u2")


def run(argv, cwd, log, env=None):
    result = subprocess.run(argv, cwd=cwd, env=env, text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            timeout=90)
    log.write(result.stdout)
    log.flush()
    if result.returncode:
        raise RuntimeError(f"{argv[0]} failed; see {log.name}\n{result.stdout[-2000:]}")
    return result.stdout


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("pattern", choices=["reference", "white", "black", "out0", "out1",
                                      "alternating_rows", "checker", "split"])
    p.add_argument("gray", type=int)
    p.add_argument("directory", type=Path)
    p.add_argument("--capture", type=Path)
    p.add_argument("--elf", type=Path, help="known matching ELF for an imported capture")
    p.add_argument("--chain-copies", type=int, choices=range(1, 9), default=1)
    args = p.parse_args()
    assert 0 <= args.gray <= 8192
    root = Path(__file__).resolve().parents[1]
    source = (root / "src/protocol.rs").read_text()
    rows = int(re.search(r"pub const ROWS: usize = (\d+);", source)[1])
    channels = int(re.search(r"pub const CHANNELS: usize = (\d+);", source)[1])
    registers_text = source.split("pub const REGISTERS:")[1].split("];")[0]
    registers = [f"{int(a,16):02X}{int(v,16):02X}" for a, v in
                 re.findall(r"\(0x([0-9a-f]+), 0x([0-9a-f]+)\)", registers_text)]
    registers.insert(0, f"02{rows-1:02X}")
    groups = int(re.search(r"pub const GROUPS: usize = (\d+);", source)[1])
    registers.insert(1, f"03{groups-1:02X}")
    full_config = "const FULL_CONFIG_EVERY_FRAME: bool = true;" in (root / "src/main.rs").read_text()
    expected_config = ["00AA", "01AA", *registers, "0055", "0155"]
    dest = args.directory.resolve() / f"{args.pattern}_{args.gray}"
    dest.mkdir(parents=True, exist_ok=False)
    env = dict(os.environ, ICND2069_PATTERN=args.pattern, ICND2069_GRAY=str(args.gray),
               ICND2069_CHAIN_COPIES=str(args.chain_copies), ICND2069_GRAY_MODE="static")
    elf = root / "target/thumbv8m.main-none-eabihf/release/icnd2069-demo"
    imported = args.capture is not None
    with (dest / "operations.log").open("w") as log:
        if not args.capture:
            run(["cargo", "build", "--release", "--locked"], root, log, env)
            run(["probe-rs", "download", "--chip", "RP235x", "--protocol", "swd",
                 "--speed", "1000", "--verify", str(elf)], root, log)
            run(["probe-rs", "reset", "--chip", "RP235x", "--protocol", "swd",
                 "--speed", "1000"], root, log)
            text = run(["swift", "tools/capture.swift"], root, log)
            args.capture = Path(next(s.split("=", 1)[1] for s in text.splitlines()
                                     if s.startswith("SAVED_PATH=")))
        shutil.copy2(args.capture, dest / "capture.dsl")
        if not imported or args.elf:
            shutil.copy2(args.elf if imported else elf, dest / "firmware.elf")
        with contextlib.redirect_stdout(log):
            data = inspect(dest / "capture.dsl", None, args.chain_copies)
        (dest / "analysis.json").write_text(json.dumps(data, indent=2))
    expected = image(args.pattern, args.gray, rows, channels)
    expected_hash = hashlib.sha256(expected.tobytes()).hexdigest()
    spacing = 16 * args.chain_copies + 8
    gray_spacing = {str(spacing): rows * channels - rows, str(spacing + 1): rows - 1}
    config_spacing = ({str(spacing): len(expected_config) - 1} if full_config else
                      {str(spacing): 3, str(spacing + 1): 1})
    checks = {
        "threshold_0p4v": float(data["threshold_v"]) == 0.4,
        "duration_50ms": 49.9 <= data["duration_ms"] <= 50.1,
        "has_complete_frames": bool(data["frames"]),
        "chain_copies_match": not data["bad_chain_copies"],
        "transfer_length_matches": all(f["gray_latch_spacing_dclks"] == gray_spacing
                                       and f["config_latch_spacing_dclks"] == config_spacing
                                       for f in data["frames"]),
        "gray_matches": all(f["gray_words"] == rows * channels and f["gray_sha256"] == expected_hash
                            for f in data["frames"]),
        "config_batch_matches": all(f["registers"] == expected_config if full_config else
                                  (len(f["registers"]) == 5 and
                                  f["registers"][:2] == ["00AA", "01AA"] and
                                  f["registers"][-2:] == ["0055", "0155"])
                                  for f in data["frames"]),
    }
    summary = {
        "pattern": args.pattern, "gray": args.gray, "capture": str(dest / "capture.dsl"),
        "rows": rows, "channels": channels, "full_config_every_frame": full_config,
        "chain_copies": args.chain_copies,
        "checks": checks, "complete_frames": len(data["frames"]),
        "row_pulses_per_frame": [f["row_pulses"] for f in data["frames"]],
        "frame_period_us": [f["period_us"] for f in data["frames"]],
        "outputs": {},
    }
    for name, o in data["outputs"].items():
        h = o["high_width_samples"]
        summary["outputs"][name] = {
            "rises": o["rises"], "active_windows": o["active_row_windows"],
            "windows": o["row_windows"],
            "width_modes_ns": sorted(((int(k)*10, v) for k, v in h.items()),
                                    key=lambda a: -a[1])[:5],
            "width_median_ns": o["pulse_width_ns_stats"].get("p50"),
            "row_high_median_ns": o["row_high_ns_stats"].get("p50"),
            "total_high_us": o["total_high_samples"] / 100,
        }
    (dest / "summary.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    assert all(checks.values()), "Input validation failed; evidence saved."


if __name__ == "__main__":
    main()
