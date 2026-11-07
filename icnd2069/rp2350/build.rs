use serde::Deserialize;
use std::{env, fs, path::PathBuf};

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Config {
    scan_repeats: u8,
    config_interval: u32,
    chains: u8,
    dclk_mhz: f64,
    // Optional documentation only; does not affect the selected frequency.
    _dclk_mhz_doc: Option<String>,
    pattern: Pattern,
    // Inactive example kept in config.json for convenient pattern switching.
    _pattern: Option<serde_json::Value>,
}

#[derive(Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
enum Pattern {
    Diagonal {
        #[serde(default = "default_latch_gap")]
        repeat_latch_gap: u8,
    },
    Normal {
        gray: Gray,
    },
}

#[derive(Deserialize)]
#[serde(untagged)]
enum Gray {
    Uniform(u16),
    Range([u16; 2]),
}

fn default_latch_gap() -> u8 {
    1
}

fn main() {
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    fs::write(out.join("memory.x"), include_bytes!("memory.x")).unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rerun-if-changed=config.json");

    let path = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap()).join("config.json");
    let json = fs::read_to_string(&path).expect("cannot read config.json");
    let config: Config = serde_json::from_str(&json).expect("invalid config.json");
    assert!((1..=64).contains(&config.scan_repeats), "config.json: scan_repeats must be 1..=64");
    assert!(config.config_interval > 0, "config.json: config_interval must be 1..=4294967295");
    assert!((1..=8).contains(&config.chains), "config.json: chains must be 1..=8");
    assert!(
        [1.0, 3.0, 6.0, 10.0, 12.5, 20.0, 25.0, 30.0, 31.0, 32.0, 33.0, 34.0, 35.0, 40.0, 45.0].contains(&config.dclk_mhz),
        "config.json: dclk_mhz must be 1, 3, 6, 10, 12.5, 20, 25, 30, 31, 32, 33, 34, 35, 40 or 45"
    );
    let (name, variant, gray, continuous_latch, latch_gap) = match config.pattern {
        Pattern::Diagonal { repeat_latch_gap } => {
            assert!((1..=64).contains(&repeat_latch_gap),
                "config.json: pattern.repeat_latch_gap must be 1..=64");
            ("diagonal", "Diagonal", [64, 64], true, repeat_latch_gap)
        }
        Pattern::Normal { gray } => {
            let gray = match gray {
                Gray::Uniform(value) => [value; 2],
                Gray::Range(range) => range,
            };
            assert!(gray.iter().all(|&value| value <= 8192),
                "config.json: pattern.gray must be 0..=8192 or [start, end], each in 0..=8192");
            ("normal", "Normal", gray, false, 1)
        }
    };

    // Internal compiler inputs for protocol.rs and host waveform tools.
    // Always emit every value, so old shell exports cannot override JSON.
    println!("cargo:rustc-env=ICND2069_SCAN_REPEATS={}", config.scan_repeats);
    println!("cargo:rustc-env=ICND2069_CONFIG_INTERVAL={}", config.config_interval);
    println!("cargo:rustc-env=ICND2069_CHAIN_COPIES={}", config.chains);
    println!("cargo:rustc-env=ICND2069_DCLK_MHZ={}", config.dclk_mhz);
    println!("cargo:rustc-env=ICND2069_PATTERN={name}");
    println!("cargo:rustc-env=ICND2069_GRAY_MODE=static");
    println!("cargo:rustc-env=ICND2069_CONTINUOUS_LATCH={}", u8::from(continuous_latch));
    println!("cargo:rustc-env=ICND2069_SHORT_DATA={}", u8::from(continuous_latch));
    println!("cargo:rustc-env=ICND2069_REPEAT_LATCH_GAP={latch_gap}");
    println!("cargo:rustc-env=ICND2069_TARGET_CHANNEL=0");
    fs::write(
        out.join("pattern_config.rs"),
        format!(
            "const PATTERN: protocol::Pattern = protocol::Pattern::{variant};\n\
             const PATTERN_NAME: &str = \"{name}\";\n\
             const GRAY_RANGE: [u16; 2] = {gray:?};\n"
        ),
    ).unwrap();
}
