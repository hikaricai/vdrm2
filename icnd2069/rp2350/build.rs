use serde::Deserialize;
use std::{env, fs, path::PathBuf};

#[derive(Deserialize)]
// Unknown fields are ignored so renamed options and debug notes can stay.
struct Config {
    scan_repeats: u8,
    #[serde(default)]
    keep_scan_time: bool,
    config_interval: u32,
    chains: u8,
    dclk_mhz: f64,
    #[serde(default)]
    cfg_dclk_mhz: Option<f64>,
    #[serde(default)]
    gclk_mhz: Option<f64>,
    #[serde(default = "default_pwm_width")]
    pwm_width: u8,
    gray: Gray,
    #[serde(default, deserialize_with = "read_repeat_latch")]
    repeat_latch: Option<RepeatLatch>,
}

#[derive(Deserialize)]
struct RepeatLatch {
    #[serde(default = "default_latch_gap")]
    repeat_latch_gap: u8,
}

// A missing field disables repeat latch; a present field must be an object.
fn read_repeat_latch<'de, D>(deserializer: D) -> Result<Option<RepeatLatch>, D::Error>
where
    D: serde::Deserializer<'de>,
{
    RepeatLatch::deserialize(deserializer).map(Some)
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

fn default_pwm_width() -> u8 {
    16
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
    assert!(config.pwm_width >= 4,
        "config.json: pwm_width must be 4..=255; current RGB reg0C requires 4*pwm_width > 13");
    let cfg_dclk_hz = config.cfg_dclk_mhz.unwrap_or(config.dclk_mhz) * 1_000_000.0;
    assert!(cfg_dclk_hz.is_finite() && cfg_dclk_hz > 0.0
        && cfg_dclk_hz <= config.dclk_mhz * 1_000_000.0
        && (cfg_dclk_hz - cfg_dclk_hz.round()).abs() < 0.001,
        "config.json: cfg_dclk_mhz must be positive, no faster than dclk_mhz, and specify whole Hz");
    let gclk_hz = match config.gclk_mhz {
        None => 0, // Retain the legacy DCLK-specific PLL preset.
        Some(mhz) => {
            let hz = mhz * 1_000_000.0;
            assert!(hz.is_finite() && hz > 25_000_000.0 && hz <= 1_000_000_000.0
                && (hz - hz.round()).abs() < 0.001,
                "config.json: gclk_mhz must be >25 MHz and specify a whole number of Hz");
            let target = hz.round() as u64;
            let dclk = (config.dclk_mhz * 1_000_000.0) as u64;
            // Restore the actual captured registers, not the equivalent M=2/P=5.
            let captured_pll = dclk == 12_500_000 && target == 31_250_000;
            assert!(captured_pll || (1..=31u64).any(|m| {
                let product = target * m;
                4_000_000 * m < dclk && dclk < 10_000_000 * m
                    && product % dclk == 0 && (1..=31).contains(&(product / dclk))
            }), "config.json: gclk_mhz cannot be generated exactly at this dclk_mhz with M/P=1..31 and 4 MHz < DCLK/M < 10 MHz; change the requested frequency");
            if captured_pll {
                println!("cargo:warning=Captured PLL M=4/P=10: DCLK/M=3.125 MHz is below the guide's input range; GCLK assumes N=1");
            }
            if target >= 100_000_000 {
                println!("cargo:warning=GCLK target reaches/exceeds guide's strict 100 MHz upper limit (experimental)");
            }
            target as u32
        }
    };
    let gray = match config.gray {
        Gray::Uniform(value) => [value; 2],
        Gray::Range(range) => range,
    };
    assert!(gray.iter().all(|&value| value <= 8192),
        "config.json: gray must be 0..=8192 or [start, end], each in 0..=8192");
    let (continuous_latch, latch_gap) = match config.repeat_latch {
        Some(RepeatLatch { repeat_latch_gap }) => {
            assert!(gray.iter().all(|&value| value <= 255),
                "config.json: repeat_latch uses 8-bit gray, each value must be 0..=255");
            assert!((1..=64).contains(&repeat_latch_gap),
                "config.json: repeat_latch.repeat_latch_gap must be 1..=64");
            (true, repeat_latch_gap)
        }
        None => (false, 1),
    };

    // Internal compiler inputs for protocol.rs and host waveform tools.
    // Always emit every value, so old shell exports cannot override JSON.
    println!("cargo:rustc-env=ICND2069_SCAN_REPEATS={}", config.scan_repeats);
    println!("cargo:rustc-env=ICND2069_KEEP_SCAN_TIME={}", u8::from(config.keep_scan_time));
    println!("cargo:rustc-env=ICND2069_CONFIG_INTERVAL={}", config.config_interval);
    println!("cargo:rustc-env=ICND2069_CHAIN_COPIES={}", config.chains);
    println!("cargo:rustc-env=ICND2069_DCLK_MHZ={}", config.dclk_mhz);
    println!("cargo:rustc-env=ICND2069_CFG_DCLK_HZ={}", cfg_dclk_hz.round() as u32);
    println!("cargo:rustc-env=ICND2069_GCLK_HZ={gclk_hz}");
    println!("cargo:rustc-env=ICND2069_PWM_WIDTH={}", config.pwm_width);
    // The image is identical in both modes; only the transfer method changes.
    println!("cargo:rustc-env=ICND2069_PATTERN=diagonal");
    println!("cargo:rustc-env=ICND2069_GRAY_MODE=static");
    println!("cargo:rustc-env=ICND2069_CONTINUOUS_LATCH={}", u8::from(continuous_latch));
    println!("cargo:rustc-env=ICND2069_SHORT_DATA={}", u8::from(continuous_latch));
    println!("cargo:rustc-env=ICND2069_REPEAT_LATCH_GAP={latch_gap}");
    println!("cargo:rustc-env=ICND2069_TARGET_CHANNEL=0");
    fs::write(
        out.join("pattern_config.rs"),
        format!(
            "const PATTERN: protocol::Pattern = protocol::Pattern::Diagonal;\n\
             const PATTERN_NAME: &str = \"diagonal\";\n\
             const GRAY_RANGE: [u16; 2] = {gray:?};\n"
        ),
    ).unwrap();
}
