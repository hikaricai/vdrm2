#!/usr/bin/env python3
"""Verify actual DMA words through the project's real CPLD RTL, without hardware."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
from config_env import build_env

root = Path(__file__).resolve().parents[1]
rtl = root.parents[1] / "lcmxo2/project/icnd2069/rtl"
config = json.loads((root / "config.json").read_text())
hdl_config = config.get("board", "hdl") == "hdl"
env = build_env(config | {"board": "hdl", "chains": 2,
    "cfg_chains": config.get("cfg_chains", config["chains"]) if hdl_config else 2})
with tempfile.TemporaryDirectory(prefix="icnd2069-cpld-") as temp:
    temp = Path(temp)
    sim, emitter, wave = temp / "sim", temp / "emit", temp / "wave.txt"
    subprocess.run(["iverilog", "-g2012", "-s", "tb_cpld_wave", "-o", str(sim),
                    *map(str, [rtl / "main.sv", rtl / "select_address.sv", rtl / "decoder.sv",
                               root / "tools/tb_cpld_wave.sv"])], check=True)
    cases = ([
        (config.get("cpld_addresses", [7, 7, 7]), int("repeat_latch" in config),
         int(config.get("row_on_cfg", False)), config.get("color", "rgb"), config.get("pattern", "diagonal")),
    ] if hdl_config else []) + [
        ([0, 3, 7], 1, 0, "rgb", "diagonal"),
        ([3, 0, 6], 0, 1, "rg", "col_29"),
        ([5, 2, 1], 1, 1, "b", "col_15"),
    ]
    if not hdl_config:
        print(f"board={config.get('board')} has no CPLD; running HDL regression cases only", flush=True)
    for addresses, latch, rows, color, pattern in cases:
        case_env = env | {
            **{f"ICND2069_CPLD_ADDRESS_{i+1}": str(a) for i, a in enumerate(addresses)},
            "ICND2069_CONTINUOUS_LATCH": str(latch), "ICND2069_ROW_ON_CFG": str(rows),
            "ICND2069_COLOR": color,
            "ICND2069_PATTERN": "col" if pattern.startswith("col_") else pattern,
            "ICND2069_TARGET_CHANNEL": pattern[4:] if pattern.startswith("col_") else "0",
        }
        subprocess.run(["rustc", "--edition=2021", "-O", str(root / "tools/emit_cpld_wave.rs"),
                        "-o", str(emitter)], env=case_env, check=True)
        with wave.open("w") as output:
            subprocess.run([str(emitter)], stdout=output, check=True)
        print(f"addresses={addresses}, latch={latch}, row_on_cfg={rows}, color={color}, pattern={pattern}", flush=True)
        args = ["vvp", str(sim), f"+wave={wave}"]
        if config["dclk_mhz"] < 26:
            args.append("+six_cycles")
        subprocess.run(args, check=True)
