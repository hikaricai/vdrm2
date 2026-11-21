#!/usr/bin/env python3
"""Verify static row masks and gray response from archived pattern captures.

Keep raw edge counts in analysis.json. For the row-mask check only, reject
OE pulses shorter than 200ns and associate OUT pulses >=1us with OE windows.
This threshold is appropriate to the measured ~4.65us full pulses in this run;
it must not be used to declare unmeasured very-low-gray pulses absent.
"""
import argparse
import contextlib
import json
from pathlib import Path
import re

import numpy as np
from inspect_dsl import load, high_integral


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("directory", type=Path)
    args = p.parse_args()
    entries = []
    for path in sorted(args.directory.glob("*/summary.json")):
        case = json.loads(path.read_text())
        rows_count = case.get("rows", 40)  # Archived 40-scan summaries predate metadata.
        data = json.loads((path.parent / "analysis.json").read_text())
        with (path.parent / "row_mask_analysis.log").open("w") as log:
            with contextlib.redirect_stdout(log):
                _, n, _, ee, _ = load(path.parent / "capture.dsl")
        rows, down = ee[3][:2]
        j = np.searchsorted(down, rows, side="right")
        paired = j < len(down)
        duration = np.zeros(len(rows), np.int64)
        duration[paired] = down[j[paired]].astype(np.int64) - rows[paired]
        rows = rows[duration >= 20]
        frame_rows = []
        for f in data["frames"]:
            start = f["start_sample"]
            end = start + round(f["period_us"] * 100)
            frame_rows.append(rows[(rows > start) & (rows <= end)])
        detail = {
            "oe_short_crossings_rejected": len(ee[3][0]) - len(rows),
            "oe_pulses_per_complete_frame": [len(r) for r in frame_rows],
            "outputs": {},
        }
        for c in [4, 5]:
            up, down, initial, _ = ee[c]
            j = np.searchsorted(down, up, side="right")
            paired = j < len(down)
            widths = np.zeros(len(up), np.int64)
            widths[paired] = down[j[paired]].astype(np.int64) - up[paired]
            main_up = up[widths >= 100]
            main_widths = widths[widths >= 100] * 10
            active_counts, mismatches, patterns = [], [], []
            for r in frame_rows:
                end = np.r_[r[1:], r[-1] + 650]
                k = np.searchsorted(main_up, r)
                found = k < len(main_up)
                active = np.zeros(len(r), bool)
                active[found] = main_up[k[found]] < end[found]
                active_counts.append(int(active.sum()))
                # Masks only checked at the measured saturated gray setting.
                if case["gray"] == 8192 or case["gray"] == 0:
                    index = np.arange(len(r)) % rows_count
                    expected = {
                        "white": np.ones(len(r), bool),
                        "black": np.zeros(len(r), bool),
                        "out0": np.full(len(r), c == 4),
                        "out1": np.full(len(r), c == 5),
                        "alternating_rows": index % 2 == 0,
                        "checker": index % 2 == c - 4,
                        "split": index < rows_count // 2,
                        "reference": np.ones(len(r), bool),
                    }[case["pattern"]]
                    if case["gray"] == 0:
                        expected[:] = False
                    mismatches.append(int(np.count_nonzero(active != expected)))
                    patterns.append("".join("1" if v else "0" for v in active[:rows_count]))
            name = f"OUT{c-4}"
            detail["outputs"][name] = {
                "main_pulses_per_frame": active_counts,
                "main_width_median_ns": float(np.median(main_widths)) if len(main_widths) else None,
                "row_mask_mismatches_per_frame": mismatches,
                "first_scan_group_active": patterns,
                "total_high_us_per_frame": [f[f"out{c-4}_high_samples"] / 100
                                            for f in data["frames"]],
                "short_out_pulses": int(np.count_nonzero(widths < 100)),
            }
        # Verify the actual rotating config word against the source's table.
        source = (Path(__file__).resolve().parents[1] / "src/protocol.rs").read_text()
        allowed = {0x0055, 0x0155, (0x02 << 8) | (rows_count - 1)}
        allowed.update((int(a, 16) << 8) | int(v, 16) for a, v in
                       re.findall(r"\(0x([0-9a-f]+), 0x([0-9a-f]+)\)", source))
        detail["configuration_values_match_source"] = (
            case["checks"]["config_batch_matches"] if case.get("full_config_every_frame")
            else all(int(f["registers"][2], 16) in allowed for f in data["frames"]))
        detail["scan_count_pass"] = all(len(r) == 64 * rows_count for r in frame_rows)
        detail["row_mask_pass"] = all(v == 0 for o in detail["outputs"].values()
                                     for v in o["row_mask_mismatches_per_frame"])
        # These are measured calibration points for this preserved scan/config
        # and voltage threshold, not a claim about untested gray levels.
        if case["pattern"] == "white":
            # 16-scan/full-config gray 255 has also been measured at 512
            # windows/frame. Other 16-scan gray points are not yet calibrated.
            measured_counts = ({0: 0, 64: 320, 128: 640, 255: 1280,
                                512: 2560, 2048: 2560, 4096: 2560, 8192: 2560}
                               if rows_count == 40 else {255: 512})
            expected_count = measured_counts.get(case["gray"])
            detail["gray_response_pass"] = (expected_count is None or all(
                count == expected_count for o in detail["outputs"].values()
                for count in o["main_pulses_per_frame"]))
        else:
            detail["gray_response_pass"] = True
        (path.parent / "row_mask_checks.json").write_text(json.dumps(detail, indent=2))
        entries.append({"case": f'{case["pattern"]}_{case["gray"]}',
                        "input_checks": case["checks"], **detail})
        print(entries[-1]["case"],
              "OE", detail["oe_pulses_per_complete_frame"],
              "main pulses", [o["main_pulses_per_frame"] for o in detail["outputs"].values()],
              "mask errors", [o["row_mask_mismatches_per_frame"] for o in detail["outputs"].values()])
    out = args.directory / "validated_results.json"
    out.write_text(json.dumps(entries, indent=2))
    assert entries, "No archived captures found."
    assert all(all(e["input_checks"].values()) and e["configuration_values_match_source"]
               and e["scan_count_pass"] and e["row_mask_pass"]
               and e["gray_response_pass"] for e in entries)


if __name__ == "__main__":
    main()
