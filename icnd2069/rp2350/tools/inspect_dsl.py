#!/usr/bin/env python3
"""Inspect complete six-channel DSL captures with bounded bit-unpacking memory.

Requires NumPy. CH0 CLK, CH1 SIN, CH2 LAT, CH3 ROW, CH4/5 OUT0/1.
Cached edge arrays are optional and should live outside the workspace.
"""
import argparse
import collections
import configparser
import hashlib
import json
from pathlib import Path
import zipfile

import numpy as np

COMMANDS = {1: "DATA_LATCH", 3: "VSYNC", 5: "WR_CFG", 7: "RD_CFG",
            9: "SOFT_RST", 11: "EN_OP", 12: "DIS_OP", 14: "PRE_ACT"}


def hist(a):
    v, c = np.unique(a, return_counts=True)
    return {str(int(x)): int(y) for x, y in zip(v, c)}


def stats(a):
    if not len(a):
        return {}
    return {"min": float(a.min()), "max": float(a.max()),
            "mean": float(a.mean()), "p01": float(np.percentile(a, 1)),
            "p50": float(np.percentile(a, 50)), "p99": float(np.percentile(a, 99))}


def load(path, deglitch_samples=0):
    cfg = configparser.ConfigParser()
    ee, sin = {}, []
    with zipfile.ZipFile(path) as z:
        cfg.read_string(z.read("header").decode())
        h = dict(cfg["header"])
        n, blocks = int(h["total samples"]), int(h["total blocks"])
        assert h["samplerate"] == "100 MHz"
        for c in range(6):
            up, down, offset, previous = [], [], 0, None
            initial = None
            for b in range(blocks):
                packed = z.read(f"L-{c}/{b}")
                if c == 1:
                    sin.append(packed)
                    continue
                x = np.unpackbits(np.frombuffer(packed, np.uint8), bitorder="little")
                x = x[:max(0, min(len(x), n - offset))]
                if not len(x):
                    continue
                if initial is None:
                    initial = int(x[0])
                if previous is not None and previous != x[0]:
                    (up if x[0] else down).append(np.array([offset], np.uint32))
                up.append((np.flatnonzero(x[1:] > x[:-1]) + offset + 1).astype(np.uint32))
                down.append((np.flatnonzero(x[1:] < x[:-1]) + offset + 1).astype(np.uint32))
                previous = int(x[-1])
                offset += len(x)
            if c != 1:
                ee[c] = (np.concatenate(up), np.concatenate(down), initial, previous)
                print(path.name, "CH", c, "rises", len(ee[c][0]), flush=True)
        session = json.loads(z.read("session"))
        if deglitch_samples:
            assert deglitch_samples >= 3 and deglitch_samples % 2 == 1
            changes = {}
            for c in range(4):
                packed = b"".join(z.read(f"L-{c}/{b}") for b in range(blocks))
                x = np.unpackbits(np.frombuffer(packed, np.uint8), bitorder="little")[:n]
                pad = np.pad(x, deglitch_samples // 2, mode="edge")
                prefix = np.r_[0, np.cumsum(pad, dtype=np.int64)]
                y = ((prefix[deglitch_samples:] - prefix[:-deglitch_samples])
                     > deglitch_samples // 2).astype(np.uint8)
                changes[f"CH{c}"] = {
                    "raw_rises": int(np.count_nonzero(x[1:] > x[:-1])),
                    "filtered_rises": int(np.count_nonzero(y[1:] > y[:-1])),
                    "changed_samples": int(np.count_nonzero(x != y)),
                }
                if c == 1:
                    sin = [np.packbits(y, bitorder="little").tobytes()]
                else:
                    ee[c] = ((np.flatnonzero(y[1:] > y[:-1])+1).astype(np.uint32),
                             (np.flatnonzero(y[1:] < y[:-1])+1).astype(np.uint32),
                             int(y[0]), int(y[-1]))
            session["analysis_filter"] = {
                "method": "centered majority; only CH0..3; OUT0/1 untouched",
                "window_samples": deglitch_samples, "changes": changes}
    return h, n, np.frombuffer(b"".join(sin), np.uint8), ee, session


def high_integral(up, down, initial, n, times):
    starts = np.r_[np.uint32(0), up] if initial else up
    ends = np.r_[down, np.uint32(n)] if len(down) < len(starts) else down
    assert len(starts) == len(ends)
    widths = ends.astype(np.int64) - starts
    prefix = np.r_[np.int64(0), np.cumsum(widths)]
    i = np.searchsorted(starts, times, side="right") - 1
    if not len(starts):
        return np.zeros(len(times), np.int64)
    safe = np.maximum(i, 0)
    result = prefix[safe] + np.clip(times.astype(np.int64) - starts[safe], 0, widths[safe])
    return np.where(i < 0, 0, result)


def inspect(path, cache=None, chain_copies=8, deglitch_samples=0):
    assert 1 <= chain_copies <= 8
    h, n, sin, ee, session = load(path, deglitch_samples)
    clk, clk_down = ee[0][:2]
    le_up, le_down = ee[2][:2]
    paired = np.searchsorted(le_down, le_up, side="right")
    valid = paired < len(le_down)
    le_up, le_fall = le_up[valid], le_down[paired[valid]]
    first, last = np.searchsorted(clk, le_up), np.searchsorted(clk, le_fall)
    widths = last - first
    words = np.zeros(len(last), np.uint16)
    copies_bad = []
    for start in range(0, len(last), 4096):
        end = min(start + 4096, len(last))
        idx = last[start:end, None] + np.arange(-16 * chain_copies, 0)
        good = idx[:, 0] >= 0
        samples = clk[np.maximum(idx, 0)]
        bits = ((sin[samples // 8] >> (samples % 8)) & 1).reshape(-1, chain_copies, 16)
        values = (bits.astype(np.uint16) * (1 << np.arange(15, -1, -1, dtype=np.uint16))).sum(axis=2)
        words[start:end] = values[:, -1]
        check = good & np.isin(widths[start:end], [1, 5])
        bad = check & np.any(values != values[:, -1, None], axis=1)
        copies_bad.extend((start + np.flatnonzero(bad)).tolist())
    sync_i = np.flatnonzero(widths == 3)
    sync = le_fall[sync_i]
    rows, row_down = ee[3][:2]
    row_end_idx = np.searchsorted(row_down, rows, side="right")
    row_good = row_end_idx < len(row_down)
    row_width_clks = (np.searchsorted(clk, row_down[row_end_idx[row_good]])
                      - np.searchsorted(clk, rows[row_good]))
    row_clk = np.searchsorted(clk, rows)
    frames = []
    reg_values = collections.defaultdict(set)
    for t, value in zip(widths, words):
        if t == 5:
            reg_values[int(value >> 8)].add(int(value & 255))
    for a, b in zip(sync_i, sync_i[1:]):
        start, end = int(le_fall[a]), int(le_fall[b])
        w, v = widths[a+1:b], words[a+1:b]
        gray = v[w == 1]
        transfer_falls = last[a+1:b]
        ri, rj = np.searchsorted(rows, [start, end], side="right")
        fr = {"start_sample": start, "period_us": (end-start)/100,
              "commands": {COMMANDS.get(int(k), k): c for k, c in hist(w).items()},
              "registers": [f"{int(x):04X}" for x in v[w == 5]],
              "gray_words": len(gray), "gray_histogram": hist(gray),
              "gray_latch_spacing_dclks": hist(np.diff(transfer_falls[w == 1])),
              "config_latch_spacing_dclks": hist(np.diff(transfer_falls[w == 5])),
              "gray_sha256": hashlib.sha256(gray.astype("<u2").tobytes()).hexdigest(),
              "first_gray_row": [int(x) for x in gray[:16]],
              "row_pulses": int(rj-ri),
              "row_period_dclks": hist(np.diff(row_clk[ri:rj])),
              "first_row_after_vsync_dclks": int(row_clk[ri]-last[a]) if rj>ri else None}
        frames.append(fr)
    out = {"file": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
           "duration_ms": n/100000, "trigger_time_ms": h["trigger time"],
           "threshold_v": session.get("Threshold Level"),
           "analysis_filter": session.get("analysis_filter"),
           "command_counts": {COMMANDS.get(int(k), k): c for k, c in hist(widths).items()},
           "chain_copies": chain_copies, "bad_chain_copies": copies_bad,
           "register_values": {f"{a:02X}": [f"{v:02X}" for v in sorted(values)]
                               for a, values in sorted(reg_values.items())},
           "clk_period_samples": hist(np.diff(clk)),
           "row_width_dclks": hist(row_width_clks),
           "row_period_dclks": hist(np.diff(row_clk)),
           "frames": frames, "outputs": {}}
    high_idx = np.searchsorted(clk_down, clk, side="right")
    good = high_idx < len(clk_down)
    out["clk_high_samples"] = hist(clk_down[high_idx[good]]-clk[good])
    for name, edge in [("lat_rise", le_up), ("lat_fall", le_fall), ("row_rise", rows)]:
        i = np.searchsorted(clk, edge)-1
        good = i >= 0
        out[name+"_after_clk_samples"] = hist(edge[good]-clk[i[good]])
    for c in [4, 5]:
        up, down, initial, final = ee[c]
        j = np.searchsorted(down, up, side="right")
        good = j < len(down)
        pulse_width = down[j[good]]-up[good]
        ri = np.searchsorted(rows, up, side="right")-1
        good_phase = ri >= 0
        phase = up[good_phase]-rows[ri[good_phase]]
        # First crossing in each ROW window, separately from later glitches.
        oi = np.searchsorted(up, rows, side="left")
        valid = oi < len(up)
        active = np.zeros(len(rows), bool)
        delay = np.zeros(len(rows), np.int64)
        delay[valid] = up[oi[valid]].astype(np.int64)-rows[valid]
        window_end = np.r_[rows[1:], np.uint32(n)]
        active[valid] = up[oi[valid]] < window_end[valid]
        total = high_integral(up, down, initial, n, np.r_[rows, np.uint32(n)])
        per_row_high = np.diff(total)
        frame_high = high_integral(up, down, initial, n, sync)
        out["outputs"][f"OUT{c-4}"] = {
            "rises": len(up), "falls": len(down), "initial": initial, "final": final,
            "high_width_samples": hist(pulse_width),
            "pulse_width_ns_stats": stats(pulse_width.astype(np.float64)*10),
            "rise_after_row_ns_stats": stats(phase.astype(np.float64)*10),
            "first_rise_after_row_samples": hist(delay[active]),
            "active_row_windows": int(active.sum()), "row_windows": len(rows),
            "row_high_ns_stats": stats(per_row_high.astype(np.float64)*10),
            "total_high_samples": int(high_integral(up, down, initial, n, np.array([n]))[0]),
            "first_edges_ms": [float(x)/100000 for x in up[:20]]}
        for i, f in enumerate(frames):
            f[f"out{c-4}_rises"] = int(np.searchsorted(up, sync[i+1])-np.searchsorted(up, sync[i]))
            f[f"out{c-4}_high_samples"] = int(frame_high[i+1]-frame_high[i])
        if cache:
            np.save(cache / f"out{c-4}_up.npy", up)
            np.save(cache / f"out{c-4}_down.npy", down)
            np.save(cache / f"out{c-4}_row_high.npy", per_row_high)
    if cache:
        for name, array in [("clk", clk), ("clk_down", clk_down), ("row", rows),
                            ("row_down", row_down), ("lat_up", le_up),
                            ("lat_down", le_fall), ("cmd_width", widths), ("cmd_word", words)]:
            np.save(cache / f"{name}.npy", array)
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("capture", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument("--cache", type=Path)
    p.add_argument("--chain-copies", type=int, choices=range(1, 9), default=8,
                   help="words per cascaded transfer (historical captures default to 8)")
    p.add_argument("--deglitch-samples", type=int, default=0,
                   help="optional odd majority window on CH0..3; original capture untouched")
    args = p.parse_args()
    assert not args.output.exists(), "refuse to overwrite evidence"
    if args.cache:
        args.cache.mkdir(parents=True, exist_ok=False)
    data = inspect(args.capture, args.cache, args.chain_copies, args.deglitch_samples)
    args.output.write_text(json.dumps(data, indent=2))
    print(json.dumps({k:v for k,v in data.items() if k not in
                     ["frames","outputs","clk_period_samples","row_period_dclks"]}, indent=2))
    print("frames", len(data["frames"]), "first", data["frames"][:1])
    for c, o in data["outputs"].items():
        print(c, {k:v for k,v in o.items() if k not in
                  ["high_width_samples","first_rise_after_row_samples"]})


if __name__ == "__main__":
    main()
