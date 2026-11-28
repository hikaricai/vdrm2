#!/usr/bin/env python3
"""Check raw-register JSON parsing and the current hardware adaptation."""
import copy
import json
from pathlib import Path
import re
import subprocess
import tempfile

from config_env import ROOT, build_env

config = json.loads((ROOT / "config.json").read_text())
if "regs" not in config:
    raise SystemExit("Current config does not enable regs")
env = build_env(config)
reference = (ROOT.parents[1] / "dsview/icnd2069/ICND2069-抓包寄存器汇总.md").read_text()
captured = {
    int(a, 16): [int(r, 16), int(g, 16), int(b, 16)]
    for a, r, g, b in re.findall(
        r"^\| ([0-9A-F]{2}) \| ([0-9A-F]{2}) \| ([0-9A-F]{2}) \| ([0-9A-F]{2}) \|",
        reference, re.M)
}
assert len(captured) == 45


def decode(raw):
    assert len(raw) == 45 * 8
    return {int(raw[i:i + 2], 16): [int(raw[j:j + 2], 16) for j in range(i + 2, i + 8, 2)]
            for i in range(0, len(raw), 8)}


current = decode(env["ICND2069_REGS"])
differences = {f"0x{a:02X}": values for a, values in current.items() if values != captured[a]}
print(f"Current board={config['board']}, rows={env['ICND2069_LINES']}, "
      f"groups={env['ICND2069_SCAN_REPEATS']}, chains={config['chains']}")
print(f"Differences from captured registers: {differences}", flush=True)
assert all(current[int(a, 16)] == ([int(v, 16)] * 3 if ',' not in v
           else [int(part.strip(), 16) for part in v.split(',')])
           for a, v in config["regs"].items() if not a.startswith('_'))

with tempfile.TemporaryDirectory(prefix="icnd2069-register-check-") as directory:
    binary = str(Path(directory) / "check")

    def check(settings):
        settings_env = build_env(settings)
        result = subprocess.run(["rustc", "--edition=2021", "-O",
            str(ROOT / "tools/check_config_interval.rs"), "-o", binary],
            env=settings_env, capture_output=True, text=True)
        assert result.returncode == 0, result.stdout + result.stderr
        result = subprocess.run([binary], capture_output=True, text=True)
        assert result.returncode == 0, result.stdout + result.stderr
        print(result.stdout.strip(), flush=True)
        return decode(settings_env["ICND2069_REGS"])

    check(config)
    overridden = copy.deepcopy(config)
    overridden["regs_high_level"] = {
        "lines": 16, "scan_repeats": 2, "pwm_width": 24,
        "gclk": "12 / (2 * 4)", "gclk_n_code": 2,
    }
    overridden["regs"]["0x04"] = "0x84"
    emitted = check(overridden)
    expected = {2: 0x0f, 3: 1, 4: 0x82, 5: 12, 6: 2, 7: 24, 8: 24}
    assert emitted == (current | {a: [v] * 3 for a, v in expected.items()})
    geometry = build_env(overridden)
    assert (geometry["ICND2069_LINES"], geometry["ICND2069_SCAN_REPEATS"]) == ("16", "2")
    fallback = overridden | {"regs_high_level": {"lines": 16}}
    assert decode(build_env(fallback)["ICND2069_REGS"]) == (current | {2: [15] * 3, 4: [0x84] * 3})

    # Change every non-geometry byte, including the PLL/PWM and RGB fields.
    # Arbitrary raw values test serialization only, never sent to hardware.
    changed = copy.deepcopy(config)
    changed.pop("regs_high_level", None)
    for key in changed["regs"]:
        if key.startswith('_') or key in ("0x02", "0x03"):
            continue
        address = int(key, 16)
        changed["regs"][key] = f"0x{address:02X},0x{address ^ 0x55:02X},0x{address ^ 0xAA:02X}"
    changed["regs"]["0x02"] = "0x0F"
    changed["regs"]["0x03"] = "0x01"
    emitted = check(changed)
    assert emitted[2] == [15] * 3 and emitted[3] == [1] * 3
    for key, value in changed["regs"].items():
        if key.startswith('_'):
            continue
        values = [int(part, 16) for part in value.split(',')]
        assert emitted[int(key, 16)] == (values * 3 if len(values) == 1 else values)
    print("PASS all 45 registers configurable; single/RGB values reach serial GPIO words", flush=True)

bad = []
for key in ("lines", "scan_repeats", "gclk_mhz", "pwm_width", "keep_scan_time"):
    bad.append((config | {key: False if key == "keep_scan_time" else 1}, f"rename {key}"))
for key, value, message in [
    ("0x00", "0xAA", "automatic access controls"),
    ("0x24", "0x00", "unsupported register"),
    ("0x0C", "0x01,0x02", "one byte or R,G,B"),
    ("0x0C", "0x100", "must be 0xXX"),
    ("0x02", "0x35,0x27,0x35", "matching RGB"),
    ("0x03", "0x7F", "scan_repeats must be 1..=64"),
]:
    case = copy.deepcopy(config)
    case.pop("regs_high_level", None)
    case["regs"][key] = value
    bad.append((case, message))
case = copy.deepcopy(config)
del case["regs"]["0x04"]
bad.append((case, "missing register 0x04"))
for fields, message in [
    ({"lines": 0}, "lines must be 1..=64"),
    ({"scan_repeats": 65}, "scan_repeats must be 1..=64"),
    ({"gclk": "10/(4*1)"}, "set gclk and gclk_n_code together"),
    ({"gclk": "0/(4*1)", "gclk_n_code": 1}, "M and P must be 1..=31"),
    ({"gclk": "4*1/10", "gclk_n_code": 1}, "gclk must be P/(M*N)"),
]:
    bad.append((config | {"regs_high_level": fields}, message))
for settings, message in bad:
    try:
        build_env(settings)
    except RuntimeError as error:
        assert message in str(error), str(error)
    else:
        raise AssertionError(f"invalid config accepted: {message}")
print(f"PASS {len(bad)} invalid/ambiguous register configurations rejected")
