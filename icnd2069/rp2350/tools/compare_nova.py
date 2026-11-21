"""Decode captured inputs and report coarse OUT0/OUT1 voltage activity."""
import argparse
import array
import bisect
import collections
import configparser
import hashlib
import json
import zipfile
from pathlib import Path
from analyze_capture import BITS, bit, COMMANDS

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("capture", type=Path)
p.add_argument("output", type=Path)
p.add_argument("--rows", type=int, default=40)
p.add_argument("--gray", type=int, help="uniform expected gray; default is original reference pattern")
p.add_argument("--full-config", action="store_true")
p.add_argument("--limit-ms", type=float, help="inspect only this prefix of an existing capture")
args = p.parse_args()
assert not args.output.exists()
with zipfile.ZipFile(args.capture) as z:
    cfg = configparser.ConfigParser()
    cfg.read_string(z.read("header").decode())
    h = cfg["header"]
    n, blocks = int(h["total samples"]), int(h["total blocks"])
    if args.limit_ms:
        n = min(n, int(args.limit_ms * 100000))
    assert h["samplerate"] == "100 MHz"
    planes = [b"".join(z.read(f"L-{c}/{b}") for b in range(blocks)) for c in range(6)]
    threshold = json.loads(z.read("session"))["Threshold Level"]
planes = [v[:(n+7)//8] for v in planes]
def edges(data):
    up, down = array.array("I"), array.array("I")
    prev = 0
    for i,v in enumerate(data):
        old = ((v << 1) | prev) & 255
        up.extend(i*8+b for b in BITS[v & ~old] if 0 < i*8+b < n)
        down.extend(i*8+b for b in BITS[old & ~v] if 0 < i*8+b < n)
        prev = v >> 7
    return up, down
ee = {}
for c in [0,2,3,4,5]:
    ee[c] = edges(planes[c])
    print("edges", c, len(ee[c][0]), flush=True)
clk = ee[0][0]
ci = lambda t: bisect.bisect_left(clk,t)
cs = []
copies_ok = True
for t in ee[2][0]:
    j = bisect.bisect_right(ee[2][1],t)
    if j == len(ee[2][1]):
        continue
    fall = ee[2][1][j]
    a,b = ci(t),ci(fall)
    word = 0
    for x in clk[max(0,b-16):b]:
        word = word << 1 | bit(planes[1],x)
    if b >= 128 and b-a in [1,5]:
        for offset in range(b-128,b,16):
            value = 0
            for x in clk[offset:offset+16]:
                value = value << 1 | bit(planes[1],x)
            copies_ok &= value == word
    cs.append((fall,b-a,word))
sync = [t for t,w,v in cs if w == 3]
ct = [t for t,w,v in cs]
frames = []
for start,end in zip(sync,sync[1:]):
    body = cs[bisect.bisect_right(ct,start):bisect.bisect_left(ct,end)]
    data = [v for t,w,v in body if w == 1]
    writes = [v for t,w,v in body if w == 5]
    rows = ee[3][0][bisect.bisect_right(ee[3][0],start):bisect.bisect_left(ee[3][0],end)]
    frames.append({
        "period_us": (end-start)/100,
        "gray_words": len(data), "gray_correct": data == (
            [args.gray] * (args.rows * 16) if args.gray is not None
            else ([0]+[8192]*15)*args.rows),
        "gray_histogram": dict(collections.Counter(data)),
        "registers": [f"{v:04X}" for v in writes],
        "commands": dict(collections.Counter(COMMANDS.get(w,str(w)) for t,w,v in body)),
        "row_pulses":len(rows),
        "row_period_dclks":dict(collections.Counter(ci(b)-ci(a) for a,b in zip(rows,rows[1:]))),
        "row_width_dclks":dict(collections.Counter(ci(ee[3][1][bisect.bisect_right(ee[3][1],t)])-ci(t) for t in rows)),
        "first_row_dclks":ci(rows[0])-ci(start) if rows else None,
        "command_falls_dclks": [ci(t)-ci(start) for t,w,v in body[:5]],
        "latch_period_dclks": dict(collections.Counter(
            ci(b)-ci(a) for a,b in zip(
                [t for t,w,v in body if w == 1],
                [t for t,w,v in body if w == 1][1:]))),
        "outputs": {f"OUT{c-4}":bisect.bisect_left(ee[c][0],end)-bisect.bisect_left(ee[c][0],start) for c in [4,5]},
    })
out = {"file":str(args.capture), "sha256":hashlib.sha256(args.capture.read_bytes()).hexdigest(),
       "duration_ms":n/100000, "threshold_v":threshold, "trigger_time":h["trigger time"],
       "copies_ok":copies_ok, "clk_gaps":dict(collections.Counter(b-a for a,b in zip(clk,clk[1:]))),
       "outputs": {f"OUT{c-4}":{"rises":len(ee[c][0]),"falls":len(ee[c][1]),"initial":bit(planes[c],0),"final":bit(planes[c],n-1),
                             "active_frames":sum(f["outputs"][f"OUT{c-4}"]>0 for f in frames)} for c in [4,5]},
       "frames":frames}
out["clk_high_ns"] = dict(collections.Counter(
    (ee[0][1][bisect.bisect_right(ee[0][1], t)]-t)*10
    for t in clk if bisect.bisect_right(ee[0][1],t) < len(ee[0][1])))
out["lat_rise_after_clk_ns"] = dict(collections.Counter(
    (t-clk[ci(t)-1])*10 for t in ee[2][0] if ci(t)>0))
out["lat_fall_after_clk_ns"] = dict(collections.Counter(
    (t-clk[ci(t)-1])*10 for t in ee[2][1] if ci(t)>0))
if args.full_config:
    # Independent expected reference red configuration; only 02 changes.
    values = [args.rows-1,0x3f,0x04,0x0a,0x01,0x20,0x20,0x08,0x08,0x00,
              0x01,0x01,0x07,0x01,0xc2,0x21,0x01,0x00,0x00,0x00,
              0x00,0xf0,0x1f,0x20,0x1f,0x10,0x72,0x82,0x41,0x04,
              0x48,0x01,0x1c]
    expected = [0x00aa,0x01aa] + [(a<<8)|v for a,v in zip(range(2,0x23),values)]
    expected += [a<<8 for a in [0x70,0x71,0x72,0x73,0x74,0xf0,0xf1,0xf2,0xf3,0xf4,0xf5]]
    expected += [0x2301,0x0055,0x0155]
    assert len(frames) >= 2, "need at least two complete frames"
    assert copies_ok
    assert set(out["clk_gaps"]) <= {7,8,9}, out["clk_gaps"]
    for f in frames:
        assert f["registers"] == [f"{v:04X}" for v in expected], f["registers"]
        assert f["gray_correct"] and f["row_pulses"] == args.rows*64
        assert f["commands"] == {"EN_OP":1,"PRE_ACT":1,"WR_CFG":49,"DATA_LATCH":args.rows*16}
        assert f["command_falls_dclks"] == [23,40,177,313,449]
        assert f["first_row_dclks"] == 25
        assert f["row_width_dclks"] == {12:1,4:args.rows*64-1}
        assert f["row_period_dclks"] == ({80:141,81:1982,82:392,83:44}
                                        if args.rows == 40 else {81:args.rows*64-1})
        assert f["latch_period_dclks"] == {136:args.rows*15,137:args.rows-1}
        assert 16660 < f["period_us"] < 16680, f["period_us"]
    out["input_verified"] = True
args.output.write_text(json.dumps(out,indent=2))
print(json.dumps({k:v for k,v in out.items() if k != "frames"},indent=2))
print("frames",len(frames),"first",frames[:1])
