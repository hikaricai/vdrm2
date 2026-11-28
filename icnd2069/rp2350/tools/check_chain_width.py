#!/usr/bin/env python3
"""Compile/run host waveform checks without editing JSON or accessing hardware."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--baseline", type=Path, help="Previous protocol.rs for timing comparison")
parser.add_argument("--config", type=Path, default=root / "config.json",
                    help="JSON settings to verify (default: firmware config.json)")
parser.add_argument("--lines-only", action="store_true",
                    help="Only verify scan-row variations (1, 16, 54, 64)")
parser.add_argument("--cfg-only", action="store_true",
                    help="Only verify configuration/data cascade combinations")
parser.add_argument("--boards-only", action="store_true",
                    help="Verify dev/hdl/direct wiring, OUT15, reverse diagonal and both latch modes")
args = parser.parse_args()
config = json.loads(args.config.read_text())
env = {k: v for k, v in os.environ.items() if not k.startswith("ICND2069_")}


def run(command, **kwargs):
    result = subprocess.run(command, text=True, capture_output=True, **kwargs)
    if result.returncode:
        raise RuntimeError(result.stdout + result.stderr)
    return result.stdout


def params(chains, latch, gap=1, pattern="diagonal", target=0, repeats=None, keep_scan=None, color=None, lines=None, cfg_chains=None, pre_clr_chains=None, row_on_cfg=None, board="hdl"):
    if pattern.startswith("col_"):
        target = int(pattern[4:])
        pattern = "col"
    return env | {
        "ICND2069_BOARD": board,
        "ICND2069_DCLK_MHZ": str(config["dclk_mhz"]),
        "ICND2069_CFG_DCLK_HZ": str(round(
            config.get("cfg_dclk_mhz", config["dclk_mhz"]) * 1_000_000)),
        "ICND2069_GCLK_HZ": str(round(config.get("gclk_mhz", 0) * 1_000_000)),
        "ICND2069_PWM_WIDTH": str(config.get("pwm_width", 16)),
        "ICND2069_SCAN_REPEATS": str(repeats or config["scan_repeats"]),
        "ICND2069_LINES": str(config.get("lines", 54) if lines is None else lines),
        "ICND2069_KEEP_SCAN_TIME": str(int(
            config.get("keep_scan_time", False) if keep_scan is None else keep_scan)),
        "ICND2069_CONFIG_INTERVAL": str(config["config_interval"]),
        "ICND2069_CHAIN_COPIES": str(chains),
        "ICND2069_CFG_CHAIN_COPIES": str(
            config.get("cfg_chains", chains) if cfg_chains is None else cfg_chains),
        "ICND2069_CFG_PRE_CLR_CHAINS": str(
            config.get("cfg_pre_clr_chains", 0) if pre_clr_chains is None else pre_clr_chains),
        "ICND2069_ROW_ON_CFG": str(int(
            config.get("row_on_cfg", False) if row_on_cfg is None else row_on_cfg)),
        "ICND2069_CONTINUOUS_LATCH": str(latch),
        "ICND2069_REPEAT_LATCH_GAP": str(gap),
        "ICND2069_PATTERN": pattern,
        "ICND2069_COLOR": color if color is not None else config.get("color", "rgb"),
        "ICND2069_TARGET_CHANNEL": str(target),
    }


with tempfile.TemporaryDirectory(prefix="icnd2069-chain-width-") as temp:
    binary = str(Path(temp) / "check")
    cases = [(2, latch, 1, pattern, 0, 1)
             for pattern in ["diagonal", "diagonal_rev"] for latch in [0, 1]]
    cases += [(2, 1, gap, "diagonal", 0, 1) for gap in [4, 64]]
    cases += [(2, 1, 1, "normal", target, 1) for target in [0, 14, 15, 29]]
    cases += [(2, 0, 1, "normal", 0, 1), (2, 1, 1, "diagonal", 0, 64)]
    cases = [(*case, False) for case in cases]
    cases += [(2, latch, 1, "diagonal", 0, repeats, True)
              for latch in [0, 1] for repeats in [1, 2, 3, 4]]
    cases += [(n, latch, 1, f"col_{column}", 0, 1, False)
              for n, column in [(2, 0), (2, 1), (2, 14), (2, 15), (2, 29)]
              for latch in [0, 1]]
    cases = [(*case, config.get("color", "rgb")) for case in cases]
    cases += [(2, latch, 1, "col_16", 0, 1, False, color)
              for color in ["r", "g", "b", "rg", "rb", "gb", "rgb"]
              if color != config.get("color", "rgb") for latch in [0, 1]]
    cases = [(*case, config.get("lines", 54)) for case in cases]
    if args.lines_only:
        cases = []
    cases += [(2, latch, 1, pattern, 0, 1, keep, "rgb", lines)
              for lines in [1, 16, 54, 64] if args.lines_only or lines != config.get("lines", 54)
              for pattern in ["diagonal", "diagonal_rev", "col_16"] for latch in [0, 1]
              for keep in [False, True]]
    cases = [(*case, config.get("cfg_chains", case[0])) for case in cases]
    if args.cfg_only:
        cases = []
    cases += [(chains, latch, 1, "diagonal", 0, 1, False, "rgb", 54, cfg_chains)
              for cfg_chains, chains in [(1, 2), (2, 2), (4, 2), (6, 2), (8, 2)]
              for latch in [0, 1]]
    cases += [(2, latch, 1, "diagonal", 0, 1, False, "r", 54, 2, preclear, rows)
              for preclear, rows in [(0, False), (2, True), (64, False), (64, True)]
              for latch in [0, 1]]
    if args.boards_only:
        cases = [
            (chains, latch, 1, pattern, 0, 1, False, "rgb", lines, chains, 0, False, board)
            for board, chains, lines in [("dev", 1, 16), ("dev", 2, 54), ("hdl", 2, 54),
                                        ("direct", 1, 16), ("direct", 2, 54)]
            for pattern in ["diagonal_rev", "col_14" if board == "direct" and chains == 1 else "col_15"]
            for latch in [0, 1]
        ]
        cases += [(1, 1, 1, "diagonal", 0, 1, False, "g", 16, 1, 0, True, "dev")]
        cases += [(2, 1, 1, "diagonal_rev", 0, 1, False, color, 54, 2, 0, True, "direct")
                  for color in ["r", "g", "b"]]
        cases += [(config["chains"], int("repeat_latch" in config),
                   config.get("repeat_latch", {}).get("repeat_latch_gap", 1),
                   config.get("pattern", "diagonal"), 0, config["scan_repeats"],
                   config.get("keep_scan_time", False), config.get("color", "rgb"),
                   config.get("lines", 54), config.get("cfg_chains", config["chains"]),
                   config.get("cfg_pre_clr_chains", 0), config.get("row_on_cfg", False),
                   config.get("board", "hdl"))]
    for case in cases:
        for tool in ["check_diagonal_latch", "check_config_interval"]:
            run(["rustc", "--edition=2021", "-C", "opt-level=1",
                 str(root / "tools" / f"{tool}.rs"), "-o", binary], env=params(*case))
            run([binary])
        print(f"PASS chains/latch/gap/pattern/target/repeats/keep_scan/color/lines/cfg_chains[/preclear/row_on_cfg/board]={case}", flush=True)

    # A short row period cannot preserve the existing control pulse widths.
    result = subprocess.run(
        ["rustc", "--edition=2021", str(root / "tools/check_diagonal_latch.rs"),
         "-o", binary], text=True, capture_output=True,
        env=params(2, 1, repeats=64, keep_scan=True))
    assert result.returncode != 0 and "minimum row period is 14 DCLK" in result.stderr, (
        result.stdout + result.stderr)
    print("PASS keep_scan_time rejects 64 sweeps in the short gray window", flush=True)
    for chains, cfg_chains in [(1, 2), (3, 2)]:
        result = subprocess.run(
            ["rustc", "--edition=2021", str(root / "tools/check_diagonal_latch.rs"),
             "-o", binary], text=True, capture_output=True,
            env=params(chains, 0, cfg_chains=cfg_chains))
        assert result.returncode != 0 and "hdl board requires" in result.stderr, result.stderr
    print("PASS non-two-chip grayscale configurations rejected", flush=True)
    for cfg_chains in [0, 9]:
        result = subprocess.run(
            ["rustc", "--edition=2021", str(root / "tools/check_diagonal_latch.rs"),
             "-o", binary], text=True, capture_output=True,
            env=params(2, 0, cfg_chains=cfg_chains))
        assert result.returncode != 0 and "cfg_chains must be 1..=8" in result.stderr, result.stderr
    print("PASS out-of-range configuration lengths rejected", flush=True)

    sources = ([args.baseline.resolve()] if args.baseline else []) + [root / "src/protocol.rs"]
    results = []
    for source in sources:
        code = f'''#![allow(dead_code)]
#[path = {json.dumps(str(source))}] mod protocol;
use protocol::*;
fn main() {{
    println!("{{}} {{}} {{}} {{}} {{}} {{}} {{}}", NORMAL_FRAME_DCLKS, FRAME_DCLKS,
        DATA_END, ROW_DCLKS, NORMAL_FRAME_PIO_CYCLES, CONFIG_FRAME_PIO_CYCLES,
        SYS_HZ / PIO_CLKDIV_INT);
}}
'''
        run(["rustc", "--edition=2021", "--crate-name", "metrics", "-", "-o", binary],
            input=code, env=params(config["chains"], 1,
                config.get("repeat_latch", {}).get("repeat_latch_gap", 1),
                pattern=config.get("pattern", "diagonal"), board=config.get("board", "hdl")))
        result = list(map(int, run([binary]).split()))
        interval = config["config_interval"]
        fps = result[6] * interval / ((interval - 1) * result[4] + result[5])
        results.append((result, fps))
        print(f"{source}: normal={result[0]}, config={result[1]}, "
              f"data_end={result[2]}, ROW={result[3]}, average theoretical FPS={fps:.2f}")
    if len(results) == 2:
        print(f"Theoretical FPS increase={(results[1][1] / results[0][1] - 1) * 100:.2f}%")
