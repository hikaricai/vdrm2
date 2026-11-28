#!/usr/bin/env python3
"""Check emitted PIO instructions; run cargo build --release --locked first."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import tempfile
from config_env import build_env

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--presets", action="store_true",
                    help="Check every DCLK preset, with matching and 1 MHz configuration rates")
args = parser.parse_args()
config = json.loads((root / "config.json").read_text())
deps = root / "target/release/deps"
libraries = list(deps.glob("libpio_core-*.rlib"))
if not libraries:
    raise SystemExit("Run cargo build --release --locked first (host pio-core is required)")
library = max(libraries, key=lambda path: path.stat().st_mtime)
env = build_env(config)
with tempfile.TemporaryDirectory(prefix="icnd2069-dclk-") as temp:
    binary = str(Path(temp) / "check")
    command = ["rustc", "--edition=2021", "-O", str(root / "tools/check_dclk.rs"),
               "--extern", f"pio={library}", "-L", f"dependency={deps}", "-o", binary]
    cases = [(config["dclk_mhz"], config.get("cfg_dclk_mhz", config["dclk_mhz"]),
              config.get("gclk_mhz", 0)), (30, 1, 75), (12.5, 1, 31.25),
             (12.5, 1.25, 31.25), (25, 25, 62.5)]
    if args.presets:
        presets = [1, 3, 6, 10, 12.5, 20, 25, 26, 28, 30, 31, 32, 33, 34, 35, 40, 45]
        cases = [(dclk, cfg, 0) for dclk in presets
                 for cfg in ([dclk] if dclk == 1 else [dclk, 1])]
        cases += [(26, 13, 78), (28, 14, 84)]
    for dclk, cfg, gclk in cases:
        case_env = env | {
            "ICND2069_DCLK_MHZ": str(dclk),
            "ICND2069_CFG_DCLK_HZ": str(round(cfg * 1_000_000)),
            "ICND2069_GCLK_HZ": str(round(gclk * 1_000_000)),
        }
        subprocess.run(command, env=case_env, check=True)
        subprocess.run([binary], check=True)
    for cfg, message in [
        (0, "positive and no faster"),
        (26_000_000, "positive and no faster"),
        (7_000_000, "integer PIO half-period"),
        (1, "divider exceeds 65535"),
    ]:
        result = subprocess.run(command, env=env | {
            "ICND2069_DCLK_MHZ": "25", "ICND2069_CFG_DCLK_HZ": str(cfg),
            "ICND2069_GCLK_HZ": "62500000",
        }, text=True, capture_output=True)
        assert result.returncode != 0 and message in result.stderr, result.stderr
    print("PASS invalid configuration frequencies rejected")
