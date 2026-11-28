#!/usr/bin/env python3
"""Check emitted PIO instructions; run cargo build --release --locked first."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import tempfile

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
env = {k: v for k, v in os.environ.items() if not k.startswith("ICND2069_")}
env.update({
    "ICND2069_BOARD": config.get("board", "hdl"),
    "ICND2069_CHAIN_COPIES": str(config["chains"]),
    "ICND2069_CFG_CHAIN_COPIES": str(config.get("cfg_chains", config["chains"])),
    "ICND2069_CFG_PRE_CLR_CHAINS": str(config.get("cfg_pre_clr_chains", 0)),
    "ICND2069_ROW_ON_CFG": str(int(config.get("row_on_cfg", False))),
    "ICND2069_SCAN_REPEATS": str(config["scan_repeats"]),
    "ICND2069_LINES": str(config.get("lines", 54)),
    "ICND2069_KEEP_SCAN_TIME": str(int(config.get("keep_scan_time", False))),
    "ICND2069_CONFIG_INTERVAL": str(config["config_interval"]),
    "ICND2069_PWM_WIDTH": str(config.get("pwm_width", 16)),
    "ICND2069_CONTINUOUS_LATCH": str(int("repeat_latch" in config)),
    "ICND2069_REPEAT_LATCH_GAP": str(config.get("repeat_latch", {}).get("repeat_latch_gap", 1)),
    "ICND2069_PATTERN": "diagonal",
})
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
