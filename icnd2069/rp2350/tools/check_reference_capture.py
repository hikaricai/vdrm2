"""Verify six-channel RP2350 reference-flow DSL captures at 100 MHz."""
import argparse
import array
import bisect
import collections
import configparser
import hashlib
import json
import zipfile
from pathlib import Path

from analyze_capture import BITS, edges, bit, COMMANDS

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("capture", type=Path)
parser.add_argument("output", type=Path)
args = parser.parse_args()
if args.output.exists():
    parser.error("output already exists")
with zipfile.ZipFile(args.capture) as z:
    cfg = configparser.ConfigParser()
    cfg.read_string(z.read("header").decode())
    h = cfg["header"]
    n, blocks = int(h["total samples"]), int(h["total blocks"])
    assert h["samplerate"] == "100 MHz"
    planes = [b"".join(z.read(f"L-{c}/{b}") for b in range(blocks)) for c in range(6)]
    threshold = json.loads(z.read("session")).get("Threshold Level")
clk = array.array("I")
prev = 0
for i, v in enumerate(planes[0]):
    old = ((v << 1) | prev) & 255
    clk.extend(i * 8 + b for b in BITS[v & ~old] if 0 < i * 8 + b < n)
    prev = v >> 7
print("CLK loaded", len(clk), flush=True)
ee = {c: edges(planes[c], n) for c in [2, 3, 4, 5]}
ci = lambda t: bisect.bisect_left(clk, t)
commands = []
for t in ee[2][0]:
    j = bisect.bisect_right(ee[2][1], t)
    if j == len(ee[2][1]):
        continue
    fall = ee[2][1][j]
    a, b = ci(t), ci(fall)
    word = 0
    for x in clk[max(0, b - 16):b]:
        word = (word << 1) | bit(planes[1], x)
    if b >= 128 and b - a in [1, 5]:
        for off in range(b - 128, b, 16):
            value = 0
            for x in clk[off:off + 16]:
                value = (value << 1) | bit(planes[1], x)
            assert value == word, (fall, off, value, word)
    commands.append((fall, COMMANDS.get(b - a, "UNKNOWN"), word))
syncs = [t for t, cmd, _ in commands if cmd == "VSYNC"]
frames, rotation = [], []
expected = [
    0x0055,0x0155,0x0227,0x033f,0x0404,0x050a,0x0601,0x0720,
    0x0820,0x0908,0x0a08,0x0b00,0x0c01,0x0d01,0x0e07,0x0f01,
    0x10c2,0x1121,0x1201,0x1300,0x1400,0x1500,0x1600,0x17f0,
    0x181f,0x1920,0x1a1f,0x1b10,0x1c72,0x1d82,0x1e41,0x1f04,
    0x2048,0x2101,0x221c,0x7000,0x7100,0x7200,0x7300,0x7400,
    0xf000,0xf100,0xf200,0xf300,0xf400,0xf500,0x2301,
]
for start, end in zip(syncs, syncs[1:]):
    cs = [c for c in commands if start < c[0] < end]
    rows = ee[3][0][bisect.bisect_right(ee[3][0], start):bisect.bisect_left(ee[3][0], end)]
    writes = [v for _, cmd, v in cs if cmd == "WR_CFG"]
    data = [v for _, cmd, v in cs if cmd == "DATA_LATCH"]
    assert len(writes) == 5 and writes[:2] == [0xaa, 0x1aa] and writes[3:] == [0x55, 0x155]
    assert len(data) == 640 and set(data) == {255}
    assert [c[1] for c in cs] == ["EN_OP", "PRE_ACT"] + ["WR_CFG"] * 5 + ["DATA_LATCH"] * 640
    assert ci(cs[0][0]) - ci(start) == 23
    assert ci(cs[1][0]) - ci(start) == 40
    assert len(rows) == 2560
    widths = [ci(ee[3][1][bisect.bisect_right(ee[3][1], t)]) - ci(t) for t in rows]
    assert widths == [12] + [4] * 2559
    assert {ci(b) - ci(a) for a, b in zip(rows, rows[1:])} == {81}
    assert ci(rows[0]) - ci(start) == 25
    rotation.append(writes[2])
    last_fall = ee[3][1][bisect.bisect_right(ee[3][1], rows[-1])]
    frames.append({
        "period_us": (end - start) / 100,
        "register": f"{writes[2]:04X}", "data_words": len(data),
        "row_pulses": len(rows),
        "row_active_span_us": (last_fall - rows[0]) / 100,
        "last_row_fall_to_vsync_us": (end - last_fall) / 100,
    })
assert len(frames) >= 47, "capture must cover a complete configuration rotation"
offset = expected.index(rotation[0])
assert all(v == expected[(offset + i) % len(expected)] for i, v in enumerate(rotation))
gaps = collections.Counter(b - a for a, b in zip(clk, clk[1:]))
assert set(gaps) <= {7, 8, 9}, gaps
outputs = {
    f"OUT{c-4}": {"rises": len(ee[c][0]), "falls": len(ee[c][1]),
                  "initial": bit(planes[c], 0), "final": bit(planes[c], n-1)}
    for c in [4, 5]
}
result = {
    "file": str(args.capture), "sha256": hashlib.sha256(args.capture.read_bytes()).hexdigest(),
    "duration_ms": n / 100000, "trigger_time_ms": h["trigger time"], "threshold_v": threshold,
    "commands": dict(collections.Counter(c[1] for c in commands)),
    "clk_rise_gap_samples": dict(gaps), "frames": frames, "outputs": outputs,
    "verified": "47-register rotation incl 0227; 5 cfg; 640 gray255; 8 copies; 2560 ROW; 12/4 widths; 81 DCLK cadence; continuous DCLK",
}
with args.output.open("x") as f:
    json.dump(result, f, indent=2)
print(json.dumps({k: v for k, v in result.items() if k != "frames"}, indent=2))
print("Complete frames:", len(frames), "First frame:", frames[0])
