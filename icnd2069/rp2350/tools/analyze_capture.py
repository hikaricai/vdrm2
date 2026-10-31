#!/usr/bin/env python3
"""Analyze six-channel DSView v3 captures using only the Python standard library.

CH0 DCLK, CH1 SIN, CH2 LE, CH3 ROW, CH4 OUT0, CH5 OUT1.
Output counts and intervals describe digital threshold crossings, not LED current.
"""
import argparse
import bisect
import collections
import configparser
import datetime
import hashlib
import json
from pathlib import Path
import zipfile


BITS = [tuple(i for i in range(8) if v & (1 << i)) for v in range(256)]
COMMANDS = {1: "DATA_LATCH", 3: "VSYNC", 5: "WR_CFG", 7: "RD_CFG",
            9: "SOFT_RST", 11: "EN_OP", 12: "DIS_OP", 14: "PRE_ACT"}


def edges(data, n):
    rising, falling = [], []
    previous = 0
    for offset, value in enumerate(data):
        old = ((value << 1) | previous) & 255
        rising.extend(offset * 8 + b for b in BITS[value & ~old])
        falling.extend(offset * 8 + b for b in BITS[old & ~value])
        previous = value >> 7
    # Initial high is not an observed edge.
    return ([x for x in rising if 0 < x < n],
            [x for x in falling if 0 < x < n])


def bit(data, i):
    return (data[i // 8] >> (i % 8)) & 1


def distribution(values):
    counts = collections.Counter(values)
    return {str(k): v for k, v in sorted(counts.items())}


def measure(path, pattern="x", gray=512):
    with zipfile.ZipFile(path) as z:
        config = configparser.ConfigParser()
        config.read_string(z.read("header").decode())
        assert config["version"]["version"] == "3", "DSView v3 required"
        h = config["header"]
        assert h["device mode"] == "0", "logic capture required"
        n, blocks = int(h["total samples"]), int(h["total blocks"])
        value, unit = h["samplerate"].split()
        rate = float(value) * {"Hz": 1, "kHz": 1e3, "MHz": 1e6, "GHz": 1e9}[unit]
        planes = []
        for ch in range(6):
            assert f"probe{ch}" in h, f"CH{ch} not enabled"
            plane = b"".join(z.read(f"L-{ch}/{b}") for b in range(blocks))
            assert (n + 7) // 8 <= len(plane) <= (n + 7) // 8 + 7
            planes.append(plane)
        session = json.loads(z.read("session"))
    all_edges = [edges(p, n) for p in planes]
    stats = {}
    for ch, (up, down) in enumerate(all_edges):
        widths = []
        for start in up:
            j = bisect.bisect_right(down, start)
            if j < len(down):
                widths.append(down[j] - start)
        stats[str(ch)] = {
            "signal": ["CLK", "DATA", "LE", "ROW", "OUT0", "OUT1"][ch],
            "initial": bit(planes[ch], 0), "final": bit(planes[ch], n - 1),
            "rising": len(up), "falling": len(down),
            "high_width_samples": distribution(widths),
            "rise_period_samples": distribution(b - a for a, b in zip(up, up[1:])),
        }
    clk = all_edges[0][0]
    commands = []
    for start in all_edges[2][0]:
        end_index = bisect.bisect_right(all_edges[2][1], start)
        if end_index == len(all_edges[2][1]):
            continue
        end = all_edges[2][1][end_index]
        first = bisect.bisect_left(clk, start)
        last = bisect.bisect_left(clk, end)
        count = last - first
        # Data is the last 16 rising-edge samples before LE falls.
        word = None
        if last >= 16:
            word = 0
            for sample in clk[last - 16:last]:
                word = word * 2 + bit(planes[1], sample)
        commands.append({"sample": end, "time_us": end * 1e6 / rate,
                         "le_clocks": count, "command": COMMANDS.get(count, "UNKNOWN"),
                         "word": word})
    syncs = [c["sample"] for c in commands if c["command"] == "VSYNC"]
    frames = []
    for start, end in zip(syncs, syncs[1:]):
        data = [c["word"] for c in commands
                if start < c["sample"] < end and c["command"] == "DATA_LATCH"]
        row_start = bisect.bisect_right(all_edges[3][0], start)
        row_end = bisect.bisect_left(all_edges[3][0], end)
        frame = {"start_sample": start, "period_us": (end - start) * 1e6 / rate,
                 "gray_words": len(data), "gray_histogram": distribution(data),
                 "row_pulses": row_end - row_start}
        if len(data) == 256 and row_end - row_start == 1024:
            expected = [gray if pattern == "uniform" or out in (row, 15-row) else 0
                        for row in range(16) for out in range(15, -1, -1)]
            frame[f"{pattern}_pattern_mismatches"] = sum(a != b for a, b in zip(data, expected))
        elif len(data) == 256 and row_end == row_start and all(v == 0 for v in data):
            frame["kind"] = "black_preload"
        for ch in (4, 5):
            up = [t for t in all_edges[ch][0] if start < t < end]
            hist = collections.Counter()
            for t in up:
                idx = bisect.bisect_right(all_edges[3][0], t) - 1 - row_start
                hist[idx % 16 if idx >= 0 else -1] += 1
            frame[f"out{ch-4}_rising"] = len(up)
            frame[f"out{ch-4}_row_histogram"] = dict(sorted(hist.items()))
        frames.append(frame)
    regs = [f'{c["word"] >> 8:02X}={c["word"] & 255:02X}'
            for c in commands if c["command"] == "WR_CFG" and c["word"] is not None]
    trigger_ms = int(h.get("trigger time", "0"))
    return {
        "file": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "samples": n, "samplerate_hz": rate, "duration_ms": n * 1000 / rate,
        "trigger_time": datetime.datetime.fromtimestamp(trigger_ms / 1000,
                         datetime.timezone.utc).isoformat(),
        "threshold_v": session.get("Threshold Level"),
        "expected_pattern": pattern, "expected_gray": gray,
        "channels": stats, "command_counts": dict(collections.Counter(
            c["command"] for c in commands)),
        "register_writes": regs, "complete_frames": frames, "commands": commands,
        "limitations": [
            "First/last partial LE pulses and frames excluded.",
            "OUT digital voltage is not LED current or brightness.",
            "Row attribution uses ROW rising edges, not RT5990 output feedback.",
        ],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("-o", "--output", type=Path, required=True)
    parser.add_argument("--pattern", choices=("x", "uniform"), default="x",
                        help="expected image; default preserves historical X captures")
    parser.add_argument("--gray", type=int, default=512,
                        help="raw 16-bit gray value (uniform demo: 255)")
    args = parser.parse_args()
    if not 0 <= args.gray <= 65535:
        parser.error("--gray must fit in 16 bits")
    if args.output.exists():
        parser.error("output exists; choose a new filename")
    result = measure(args.capture, args.pattern, args.gray)
    with args.output.open("x") as f:
        json.dump(result, f, indent=2, ensure_ascii=False)
        f.write("\n")
    print(json.dumps({k: v for k, v in result.items()
                      if k not in ("commands", "channels", "complete_frames")}, indent=2))
    for ch, stats in result["channels"].items():
        print("CH" + ch, stats["signal"], "rising", stats["rising"],
              "high widths", stats["high_width_samples"])
    print("Frames:", len(result["complete_frames"]))
    if result["complete_frames"]:
        print(json.dumps(result["complete_frames"][0], indent=2))


if __name__ == "__main__":
    main()
