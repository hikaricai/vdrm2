#!/usr/bin/env python3
"""Verify a saved single-chip, one-sweep white gray-cycle capture.

No hardware access. Preserve raw DSL and OUT edges. Compare raw decoding with
an explicitly documented five-sample input majority filter, and check output
against both same-frame and previous-frame gray.
"""
import argparse
import collections
import hashlib
import json
from pathlib import Path
import re

import numpy as np
from inspect_dsl import inspect


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("directory", type=Path, help="contains capture.dsl and matching firmware.elf")
    args = p.parse_args()
    dest = args.directory.resolve()
    capture = dest / "capture.dsl"
    source = (Path(__file__).resolve().parents[1] / "src/protocol.rs").read_text()
    regs = source.split("pub const REGISTERS:")[1].split("];")[0]
    expected_regs = ["00AA", "01AA", "020F", "0300"] + [
        f"{int(a,16):02X}{int(v,16):02X}"
        for a, v in re.findall(r"\(0x([0-9a-f]+), 0x([0-9a-f]+)\)", regs)
    ] + ["0055", "0155"]
    assert len(expected_regs) == 49
    sequence = [0, 1, 2, 4, 8, 16, 32, 64, 128, 255]
    analyses = {}
    for name, window in [("raw", 0), ("filtered", 5)]:
        path = dest / f"{name}.json"
        if path.exists():
            data = json.loads(path.read_text())
        else:
            edges = dest / "edges" if window else None
            if edges:
                edges.mkdir(exist_ok=True)
            data = inspect(capture, edges, 1, window)
            path.write_text(json.dumps(data, indent=2))
        assert data["sha256"] == hashlib.sha256(capture.read_bytes()).hexdigest()
        assert data["chain_copies"] == 1
        assert (data["analysis_filter"] or {}).get("window_samples", 0) == window
        analyses[name] = data
    raw, data = analyses["raw"], analyses["filtered"]
    frames = data["frames"]
    gray = [int(next(iter(f["gray_histogram"]))) for f in frames]
    decode_fields = ["commands", "registers", "gray_words", "gray_sha256", "row_pulses",
                     "gray_latch_spacing_dclks", "config_latch_spacing_dclks"]
    checks = {
        "50ms_0p4V": 49.9 <= data["duration_ms"] <= 50.1 and float(data["threshold_v"]) == 0.4,
        "many_complete_frames": len(frames) >= 50,
        "49_configs_every_frame": all(f["registers"] == expected_regs for f in frames),
        "256_uniform_gray_words": all(
            f["gray_words"] == 256 and len(f["gray_histogram"]) == 1 and
            f["gray_sha256"] == hashlib.sha256(np.full(256, g, dtype="<u2").tobytes()).hexdigest()
            for f, g in zip(frames, gray)),
        "gray_changes_every_frame_in_order": all(
            b == sequence[(sequence.index(a) + 1) % len(sequence)]
            for a, b in zip(gray, gray[1:])),
        "all_ten_grays_observed": set(gray) == set(sequence),
        "one_sweep_16_rows": all(f["row_pulses"] == 16 for f in frames),
        "single_chip_transfer_spacing": all(
            f["gray_latch_spacing_dclks"] == {"24": 240, "25": 15}
            and f["config_latch_spacing_dclks"] == {"24": 48} for f in frames),
        "only_expected_commands": all(f["commands"] == {
            "DATA_LATCH": 256, "WR_CFG": 49, "EN_OP": 1, "PRE_ACT": 1} for f in frames),
        "raw_and_filtered_decode_agree": len(raw["frames"]) == len(frames) and all(
            all(a[k] == b[k] for k in decode_fields)
            for a, b in zip(raw["frames"], frames)),
    }
    edges = dest / "edges"
    clk = np.load(edges / "clk.npy")
    lat = np.load(edges / "lat_down.npy")
    widths = np.load(edges / "cmd_width.npy")
    rows = np.load(edges / "row.npy")
    sync = lat[widths == 3]
    sync_clk = np.searchsorted(clk, sync)
    frame_clocks = np.diff(sync_clk)
    checks["8696_clocks_every_frame"] = bool(np.all(frame_clocks == 8696))
    checks["no_long_clock_stall"] = int(np.max(np.diff(clk))) <= 13
    checks["first_row_phase_stable"] = all(f["first_row_after_vsync_dclks"] == 7392 for f in frames)
    periods = np.array([f["period_us"] for f in frames])
    summary = {
        "capture_sha256": data["sha256"],
        "firmware_sha256": hashlib.sha256((dest / "firmware.elf").read_bytes()).hexdigest(),
        "complete_frames": len(frames),
        "gray_sequence": sequence,
        "frame_period_us": {"min": float(periods.min()), "mean": float(periods.mean()),
                            "max": float(periods.max())},
        "fps": float(1e6 / periods.mean()),
        "speedup_over_saved_16p66994ms": float(16669.94 / periods.mean()),
        "max_clock_period_ns": int(np.max(np.diff(clk))) * 10,
        "input_filter": data["analysis_filter"],
        "output_lag_comparison": {},
        "outputs_by_previous_frame_gray": {},
    }
    # Do not remove short OUT pulses. Check each logical row window and the
    # entire frame; output must occur only when previous-frame gray is nonzero.
    for c in [0, 1]:
        up = np.load(edges / f"out{c}_up.npy")
        down = np.load(edges / f"out{c}_down.npy")
        lag_errors = {}
        grouped = collections.defaultdict(list)
        row_errors = 0
        for lag in [0, 1]:
            errors = 0
            for i in range(1, len(frames)):
                expected = 16 if gray[i-lag] else 0
                errors += frames[i][f"out{c}_rises"] != expected
            lag_errors[str(lag)] = int(errors)
        for i in range(1, len(frames)):
            start, end = sync[i], sync[i+1]
            rr = rows[(rows >= start) & (rows < end)]
            ends = np.r_[rr[1:], end]
            counts = np.searchsorted(up, ends) - np.searchsorted(up, rr)
            row_errors += int(np.count_nonzero(counts != int(gray[i-1] != 0)))
            pulses = up[(up >= start) & (up < end)]
            di = np.searchsorted(down, pulses, side="right")
            assert np.all(di < len(down))
            pulse_ns = (down[di].astype(np.int64) - pulses) * 10
            grouped[gray[i-1]].append({
                "rises": len(pulses),
                "high_us": frames[i][f"out{c}_high_samples"] / 100,
                "pulse_width_ns": pulse_ns.tolist(),
            })
        checks[f"out{c}_previous_frame_response"] = lag_errors["1"] == 0 and row_errors == 0
        summary["output_lag_comparison"][f"OUT{c}"] = {
            "frame_count": len(frames)-1, "mismatches_by_lag": lag_errors,
            "row_window_mismatches_at_lag1": row_errors}
        summary["outputs_by_previous_frame_gray"][f"OUT{c}"] = {
            str(g): {
                "frames": len(values),
                "rises_per_frame": sorted(set(v["rises"] for v in values)),
                "mean_high_us_per_frame": float(np.mean([v["high_us"] for v in values])),
                "median_pulse_width_ns": float(np.median([
                    t for v in values for t in v["pulse_width_ns"]])) if g else None,
            } for g, values in sorted(grouped.items())}
    summary["checks"] = {k: bool(v) for k, v in checks.items()}
    summary["gray_resolution_caveat"] = (
        "At 0.4V, zero is off and every tested nonzero gray gives 16 approximately "
        "4.66/4.68us pulses. Input gray changes verified; proportional brightness not verified."
    )
    output = dest / "summary.json"
    assert not output.exists(), "refuse to overwrite verification evidence"
    output.write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))
    assert all(checks.values()), "verification failed; saved all evidence"


if __name__ == "__main__":
    main()
