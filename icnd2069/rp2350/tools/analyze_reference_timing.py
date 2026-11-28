#!/usr/bin/env python3
"""Measure the historical 261024 reference DSL in observed DCLK rising edges.

Requires numpy. Historical wiring ONLY: CLK=0, LE=1, RGB=2/5/6,
ROW=3; CH4/7 are unidentified RT5990 pins. No hardware access.
"""
import argparse
import collections
import configparser
import hashlib
import json
from pathlib import Path
import zipfile

import numpy as np


def hist(values):
    return dict(sorted(collections.Counter(map(int, values)).items()))


def extract(z, ch, blocks, n):
    rises, falls, raw = [], [], []
    previous, offset = None, 0
    for block in range(blocks):
        data = z.read(f"L-{ch}/{block}")
        raw.append(data)
        bits = np.unpackbits(np.frombuffer(data, dtype=np.uint8), bitorder="little")
        bits = bits[:max(0, n - offset)]
        change = np.flatnonzero(bits[1:] != bits[:-1]) + 1
        if previous is not None and len(bits) and bits[0] != previous:
            change = np.r_[0, change]
        rises.append((change[bits[change] == 1] + offset).astype(np.int64))
        falls.append((change[bits[change] == 0] + offset).astype(np.int64))
        if len(bits):
            previous = bits[-1]
        offset += len(bits)
    return np.concatenate(rises), np.concatenate(falls), b"".join(raw)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("capture", type=Path)
    ap.add_argument("-o", "--output", type=Path, required=True)
    ap.add_argument("--cache", type=Path, help="Optional local NPZ edge cache")
    ap.add_argument("--rust-jsonl", type=Path, help="Cross-check existing Rust decoder output")
    args = ap.parse_args()
    digest = hashlib.sha256(args.capture.read_bytes()).hexdigest()
    cache = None
    if args.cache and args.cache.exists():
        cache = np.load(args.cache)
        assert str(cache["sha256"]) == digest
    edges, rgb = {}, {}
    with zipfile.ZipFile(args.capture) as z:
        cfg = configparser.ConfigParser()
        cfg.read_string(z.read("header").decode())
        h = cfg["header"]
        assert cfg["version"]["version"] == "3" and h["device mode"] == "0"
        n, blocks = int(h["total samples"]), int(h["total blocks"])
        assert h["samplerate"] == "250 MHz"
        for ch in range(8):
            if cache is not None:
                if ch in (2, 5, 6):
                    rgb[ch] = np.frombuffer(
                        b"".join(z.read(f"L-{ch}/{b}") for b in range(blocks)), dtype=np.uint8)
                else:
                    edges[ch] = (cache[f"up{ch}"], cache[f"down{ch}"])
            else:
                up, down, data = extract(z, ch, blocks, n)
                if ch in (2, 5, 6):
                    rgb[ch] = np.frombuffer(data, dtype=np.uint8)
                else:
                    edges[ch] = up, down
            print(f"Loaded CH{ch}", flush=True)
    if args.cache and cache is None:
        np.savez(args.cache, sha256=digest,
                 **{f"{kind}{ch}": arr for ch, pair in edges.items()
                    for kind, arr in zip(("up", "down"), pair)})
    clk = edges[0][0]
    count = lambda sample: np.searchsorted(clk, sample, side="left")
    le_up, le_down = edges[1]
    # Keep only pulses with both edges captured.
    le_up = le_up[le_up < le_down[-1]]
    le_down = le_down[np.searchsorted(le_down, le_up, side="right")]
    begin, end = count(le_up), count(le_down)
    widths = end - begin
    sync = np.flatnonzero(widths == 3)
    data = np.flatnonzero(widths == 1)
    commands = {1: "DATA_LATCH", 3: "VSYNC", 5: "WR_CFG", 7: "RD_CFG",
                9: "SOFT_RST", 11: "EN_OP", 12: "DIS_OP", 14: "PRE_ACT"}
    # Independently decode the last 128 sampled bits of every data/config pulse.
    payload = np.flatnonzero(np.isin(widths, [1, 5]))
    samples = clk[end[payload, None] - 128 + np.arange(128)]
    decoded = {}
    for ch, raw in rgb.items():
        bits = (raw[samples // 8] >> (samples % 8).astype(np.uint8)) & 1
        decoded[ch] = (bits.reshape(-1, 8, 16).astype(np.uint32)
                       * (1 << np.arange(15, -1, -1))).sum(axis=2)
    pindex = {int(command): i for i, command in enumerate(payload)}
    row_up, row_down = edges[3]
    row_up = row_up[row_up < row_down[-1]]
    row_down = row_down[np.searchsorted(row_down, row_up, side="right")]
    ru, rd = count(row_up), count(row_down)
    frames = []
    sequence_hist = collections.Counter()
    row_schedules, gray_schedules = collections.Counter(), collections.Counter()
    row_periods, data_periods = [], []
    payload_errors = 0
    extra = {str(ch): [] for ch in (4, 7)}
    for f, s in enumerate(sync):
        stop = sync[f + 1] if f + 1 < len(sync) else len(widths)
        frame_data = data[(data > s) & (data < stop)]
        if not len(frame_data):
            continue
        origin = begin[s]
        rows = np.flatnonzero((row_up >= le_up[s]) &
                             (row_up < (le_up[stop] if stop < len(widths) else n)))
        row0 = rows[0]
        first, last = frame_data[0], frame_data[-1]
        preamble = list(range(s, first))
        sequence_hist[tuple(map(int, widths[preamble]))] += 1
        event = {
            "frame_1based": f + 1, "sync_start_sample": int(le_up[s]),
            "sync_end_sample": int(le_down[s]), "sync_start_dclk_index": int(origin),
            "complete_sync_interval": f + 1 < len(sync),
            "first_gray_bit_sample": int(clk[end[first] - 128]),
            "first_gray_latch_start_sample": int(le_up[first]),
            "first_gray_latch_end_sample": int(le_down[first]),
            "last_gray_latch_end_sample": int(le_down[last]),
            "first_row_sample": int(row_up[row0]),
            "first_row_end_sample": int(row_down[row0]),
            "preamble": [{"command": commands.get(int(widths[i]), "UNKNOWN"),
                          "start": int(begin[i] - origin), "end": int(end[i] - origin),
                          "rgb_words": {str(ch): [int(x) for x in decoded[ch][pindex[i]]]
                                        for ch in (2, 5, 6)} if widths[i] == 5 else None}
                         for i in preamble],
            "gray_latches": len(frame_data),
            "first_gray_bit": int(end[first] - 128 - origin),
            "first_gray_latch_start": int(begin[first] - origin),
            "first_gray_latch_end": int(end[first] - origin),
            "last_gray_latch_end": int(end[last] - origin),
            "first_row_start": int(ru[row0] - origin),
            "first_row_end": int(rd[row0] - origin),
            "rows_observed": len(rows),
            "row_width_counts": hist(rd[rows] - ru[rows]),
            "last_row_start": int(ru[rows[-1]] - origin),
            "last_row_end": int(rd[rows[-1]] - origin),
            "rows_started_by_first_gray_latch": int(np.sum(row_up[rows] < le_down[first])),
            "rows_started_by_last_gray_latch": int(np.sum(row_up[rows] < le_down[last])),
        }
        for j, i in enumerate(frame_data):
            expected = 0 if j % 16 == 0 else 0x2B8
            for ch in (2, 5, 6):
                payload_errors += int(np.sum(decoded[ch][pindex[i]] != expected))
        data_periods.extend(np.diff(end[frame_data]).tolist())
        gray_schedules[hashlib.sha256(
            (end[frame_data] - origin).astype("<i8").tobytes()).hexdigest()] += 1
        if f + 1 < len(sync):
            row_schedules[hashlib.sha256(
                (ru[rows] - origin).astype("<i8").tobytes()).hexdigest()] += 1
            event["frame_dclks"] = int(begin[stop] - origin)
            event["frame_samples"] = int(le_up[stop] - le_up[s])
            row_periods.extend(np.diff(ru[rows]).tolist())
            event["last_row_to_next_sync"] = int(begin[stop] - rd[rows[-1]])
            for ch in (4, 7):
                up, down = edges[ch]
                u = up[(up >= le_up[s]) & (up < le_up[stop])]
                d = down[np.searchsorted(down, u, side="right")]
                extra[str(ch)].append({"pulses": len(u),
                    "first_start": int(count(u[0]) - origin),
                    "width_counts": hist(count(d) - count(u)),
                    "period_counts": hist(np.diff(count(u)))})
        frames.append(event)
    complete = [f for f in frames if f["complete_sync_interval"]]
    keys = ["first_gray_bit", "first_gray_latch_start", "first_gray_latch_end",
            "last_gray_latch_end", "first_row_start", "first_row_end",
            "rows_observed", "last_row_start", "last_row_end", "frame_dclks",
            "last_row_to_next_sync", "rows_started_by_first_gray_latch",
            "rows_started_by_last_gray_latch"]
    result = {
        "capture": str(args.capture), "sha256": digest, "samples": n,
        "sample_rate_hz": 250000000, "duration_seconds": n / 250000000,
        "coordinate": "C(t)=number of observed DCLK rises strictly before t; origin=VSYNC LE rise",
        "command_counts": {commands.get(k, f"UNKNOWN_{k}"): v for k, v in hist(widths).items()},
        "dclk_period_samples": hist(np.diff(clk)),
        "command_edge_coincidences": int(np.sum(np.isin(le_up, clk)) +
                                         np.sum(np.isin(le_down, clk))),
        "sequence_counts": {str(k): v for k, v in sequence_hist.items()},
        "gray_latch_schedule_sha256_counts": dict(gray_schedules),
        "complete_row_schedule_sha256_counts": dict(row_schedules),
        "complete_frames": len(complete),
        "all_gray_payload_mismatches": payload_errors,
        "complete_frame_distributions": {k: hist(f[k] for f in complete) for k in keys},
        "gray_latch_period_counts": hist(data_periods),
        "row_period_counts_within_frames": hist(row_periods),
        "row_width_counts_complete_frames": dict(sum(
            (collections.Counter(f["row_width_counts"]) for f in complete),
            collections.Counter())),
        "first_frame_row_start_offsets_first_80": (
            ru[(row_up >= le_up[sync[0]]) & (row_up < le_up[sync[1]])]
            - begin[sync[0]])[:80].tolist(),
        "first_frame_gray_latch_end_offsets_first_32": (
            end[data[(data > sync[0]) & (data < sync[1])]]
            - begin[sync[0]])[:32].tolist(),
        "auxiliary_pins": extra,
        "frames": frames,
    }
    if args.rust_jsonl:
        checked, checked_words = 0, 0
        with args.rust_jsonl.open() as source:
            for line in source:
                item = json.loads(line)
                if item["type"] != "command":
                    continue
                event = item["event"]
                i = checked
                assert event["complete"] and not event["warnings"]
                assert event["lat_start_sample"] == int(le_up[i])
                assert event["end_sample"] == int(le_down[i])
                assert event["lat_clocks"] == int(widths[i])
                assert event["command"] == commands[int(widths[i])]
                if i in pindex:
                    for ch, color in ((2, "red"), (5, "green"), (6, "blue")):
                        assert [int(x, 16) for x in event[color]["words_hex"]] == [
                            int(x) for x in decoded[ch][pindex[i]]]
                        checked_words += 8
                checked += 1
        assert checked == len(widths)
        result["independent_rust_crosscheck"] = {
            "commands_checked": checked, "rgb_payload_words_checked": checked_words,
            "command_start_end_width_and_payload": "PASS"}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({k: v for k, v in result.items()
                      if k not in ("frames", "auxiliary_pins")}, indent=2))
    print("FIRST FRAME", json.dumps(frames[0], indent=2))


if __name__ == "__main__":
    main()
