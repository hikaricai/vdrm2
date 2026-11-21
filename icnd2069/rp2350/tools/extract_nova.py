"""Extract a complete official frame in DCLK rising-edge coordinates."""
import array
import bisect
import configparser
import json
import struct
import sys
import zipfile
from pathlib import Path
from analyze_capture import BITS, bit

root = Path(__file__).resolve().parents[1]
source = root.parent / "captures/reference/nova_to_devboard_100m_2s.dsl"
with zipfile.ZipFile(source) as z:
    cfg = configparser.ConfigParser()
    cfg.read_string(z.read("header").decode())
    h = cfg["header"]
    n, blocks = int(h["total samples"]), int(h["total blocks"])
    planes = [b"".join(z.read(f"L-{c}/{b}") for b in range(blocks)) for c in range(4)]

def edges(data):
    up, down = array.array("I"), array.array("I")
    prev = 0
    for i, v in enumerate(data):
        old = ((v << 1) | prev) & 255
        up.extend(i * 8 + b for b in BITS[v & ~old] if 0 < i * 8 + b < n)
        down.extend(i * 8 + b for b in BITS[old & ~v] if 0 < i * 8 + b < n)
        prev = v >> 7
    return up, down

clk = edges(planes[0])[0]
le = edges(planes[2])
row = edges(planes[3])
ci = lambda t: bisect.bisect_left(clk, t)
commands = []
for t in le[0]:
    j = bisect.bisect_right(le[1], t)
    if j == len(le[1]):
        continue
    fall = le[1][j]
    commands.append((ci(fall), ci(fall) - ci(t)))
sync = [t for t, w in commands if w == 3]
# Begin eight clocks before VSYNC fall; all three LE-high clocks are included.
a, end = sync[0] - 8, sync[1] - 8
body = [(t - a, w) for t, w in commands if a <= t < end]
assert len(body) == 648
rotation_fall = [t for t, w in body if w == 5][2]
count = end - a
pins = [0x100] * 208334
for i, t in enumerate(clk[a:end]):
    pins[i] |= bit(planes[1], t) << 3 | bit(planes[2], t) | bit(planes[3], t) << 12
rows = [ci(t) - a for t in row[0] if clk[a] < t < clk[end]]
assert len(rows) == 2560
# Unmeasured RT5990 controls retain explicitly documented board assumptions.
for i, start in enumerate(rows):
    stop = rows[i + 1] - 8 if i + 1 < len(rows) else start + 72
    for t in range(start - 8, stop):
        pins[t] &= ~0x100
        if t < start:
            pins[t] |= 0x100
        if i % 40 == 0 and t < start + 2:
            pins[t] |= 1 << 10
        if start - 4 <= t < start - 2:
            pins[t] |= 1 << 6
assert count < len(pins)
data = struct.pack("<" + "H" * len(pins), *pins)
output = root.parent / "captures/legacy_nova"
output.mkdir(parents=True, exist_ok=True)
dest = output / "nova_frame.bin"
dest.write_bytes(data)
meta = {"source": str(source), "source_frame_dclks": count,
        "buffer_dclks": len(pins), "vsync_fall": 8,
        "rotation_fall": rotation_fall, "row_pulses": len(rows),
        "row_periods": {str(v): sum(y-x == v for x,y in zip(rows, rows[1:])) for v in range(80,84)},
        "command_falls": body}
(output / "nova_frame.json").write_text(json.dumps(meta, indent=2))
print(json.dumps({k:v for k,v in meta.items() if k != "command_falls"}, indent=2))
