"""Use the firmware's build.rs parser for host waveform checks."""
import json
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def build_env(config):
    builders = list((ROOT / "target/release/build").glob("icnd2069-demo-*/build-script-build"))
    if not builders:
        raise RuntimeError("Run cargo build --release --locked first")
    builder = max(builders, key=lambda p: p.stat().st_mtime)
    env = {k: v for k, v in os.environ.items() if not k.startswith("ICND2069_")}
    with tempfile.TemporaryDirectory(prefix="icnd2069-config-") as directory:
        (Path(directory) / "config.json").write_text(json.dumps(config))
        result = subprocess.run([str(builder)], env=env | {
            "CARGO_MANIFEST_DIR": directory, "OUT_DIR": directory},
            text=True, capture_output=True)
        if result.returncode:
            raise RuntimeError(result.stdout + result.stderr)
    for line in result.stdout.splitlines():
        if line.startswith("cargo:rustc-env="):
            key, value = line.removeprefix("cargo:rustc-env=").split("=", 1)
            env[key] = value
    return env
