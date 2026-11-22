use serde::Deserialize;
use std::{env, fs, path::PathBuf};

#[derive(Deserialize)]
// Unknown fields are ignored so renamed options and debug notes can stay.
struct Config {
    scan_repeats: u8,
    #[serde(default = "default_lines")]
    lines: u8,
    #[serde(default)]
    keep_scan_time: bool,
    config_interval: u32,
    chains: u8,
    #[serde(default)]
    cfg_chains: Option<u8>,
    #[serde(default)]
    pin_drive: Option<PinDrive>,
    #[serde(default = "default_cpld_addresses")]
    cpld_addresses: [u8; 3],
    #[serde(default)]
    cfg_pre_clr_chains: u8,
    #[serde(default)]
    row_on_cfg: bool,
    #[serde(default)]
    disable_rt5990: bool,
    dclk_mhz: f64,
    #[serde(default)]
    cfg_dclk_mhz: Option<f64>,
    #[serde(default)]
    gclk_mhz: Option<f64>,
    #[serde(default = "default_pwm_width")]
    pwm_width: u8,
    #[serde(default = "default_pattern")]
    pattern: String,
    #[serde(default = "default_color")]
    color: String,
    gray: Gray,
    #[serde(default, deserialize_with = "read_repeat_latch")]
    repeat_latch: Option<RepeatLatch>,
}

#[derive(Clone, Copy, Deserialize, Default)]
enum PinDriveStrength {
    #[serde(rename = "2mA")]
    Ma2,
    #[default]
    #[serde(rename = "4mA")]
    Ma4,
    #[serde(rename = "8mA")]
    Ma8,
    #[serde(rename = "12mA")]
    Ma12,
}

impl PinDriveStrength {
    fn milliamps(self) -> u8 {
        match self {
            Self::Ma2 => 2,
            Self::Ma4 => 4,
            Self::Ma8 => 8,
            Self::Ma12 => 12,
        }
    }
}

#[derive(Clone, Copy, Deserialize, Default)]
#[serde(rename_all = "lowercase")]
enum PinSlewRate {
    #[default]
    Slow,
    Fast,
}

impl PinSlewRate {
    fn variant(self) -> &'static str {
        match self {
            Self::Slow => "Slow",
            Self::Fast => "Fast",
        }
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct PinDrive {
    #[serde(default)]
    strength: PinDriveStrength,
    #[serde(default)]
    cfg_strength: Option<PinDriveStrength>,
    #[serde(default)]
    slew_rate: PinSlewRate,
    #[serde(default)]
    cfg_slew_rate: Option<PinSlewRate>,
    pins: PinSelection,
}

#[derive(Deserialize)]
#[serde(untagged)]
enum PinSelection {
    One(String),
    Many(Vec<String>),
}

impl PinSelection {
    fn mask(&self) -> u32 {
        let names = match self {
            Self::One(name) => std::slice::from_ref(name),
            Self::Many(names) => names.as_slice(),
        };
        let mapping = [
            ("r1", 0), ("g1", 1), ("b1", 2), ("sel1", 3),
            ("r2", 4), ("g2", 5), ("b2", 6), ("sel2", 7),
            ("r3", 8), ("g3", 9), ("b3", 10), ("sel3", 11),
            ("lat1", 12), ("lat2", 13), ("lat3", 14), ("dclk", 15),
            ("lck", 16), ("dclk2", 17), ("lbk", 18), ("ldata", 20), ("row", 22),
        ];
        names.iter().fold(0, |mask, name| {
            let (_, pin) = mapping.iter().find(|(field, _)| *field == name)
                .unwrap_or_else(|| panic!("config.json: pin_drive.pins: unknown Pins field {name:?}"));
            mask | (1u32 << pin)
        })
    }
}

fn default_pattern() -> String {
    "diagonal".into()
}

fn default_cpld_addresses() -> [u8; 3] {
    [7; 3]
}

fn default_lines() -> u8 {
    54
}

fn default_color() -> String {
    "rgb".into()
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
    let (drive_ma, cfg_drive_ma, slew, cfg_slew, pin_mask) = match &config.pin_drive {
        Some(drive) => (
            drive.strength.milliamps(),
            drive.cfg_strength.unwrap_or(drive.strength).milliamps(),
            drive.slew_rate.variant(),
            drive.cfg_slew_rate.unwrap_or(drive.slew_rate).variant(),
            drive.pins.mask(),
        ),
        None => (4, 4, "Slow", "Slow", 0),
    };
    fs::write(
        out.join("pin_config.rs"),
        format!(
            "pub const PIN_DRIVE_STRENGTH: Drive = Drive::_{drive_ma}mA;\n\
             pub const CFG_PIN_DRIVE_STRENGTH: Drive = Drive::_{cfg_drive_ma}mA;\n\
             pub const PIN_SLEW_RATE: SlewRate = SlewRate::{slew};\n\
             pub const CFG_PIN_SLEW_RATE: SlewRate = SlewRate::{cfg_slew};\n\
             pub const PIN_DRIVE_MASK: u32 = {pin_mask:#x};\n"
        ),
    ).unwrap();
    assert!((1..=64).contains(&config.scan_repeats), "config.json: scan_repeats must be 1..=64");
    assert!((1..=64).contains(&config.lines), "config.json: lines must be 1..=64");
    assert!(config.config_interval > 0, "config.json: config_interval must be 1..=4294967295");
    assert_eq!(config.chains, 2, "config.json: this board requires chains=2");
    let cfg_chains = config.cfg_chains.unwrap_or(config.chains);
    assert!((1..=8).contains(&cfg_chains), "config.json: cfg_chains must be 1..=8");
    assert!(config.cpld_addresses.iter().all(|&a| a <= 7),
        "config.json: cpld_addresses must contain three addresses in 0..=7");
    assert!(config.cfg_pre_clr_chains <= 64, "config.json: cfg_pre_clr_chains must be 0..=64");
    assert!(matches!(config.color.as_str(), "r" | "g" | "b" | "rg" | "rb" | "gb" | "rgb"),
        "config.json: color must be r, g, b, rg, rb, gb or rgb");
    let (pattern_kind, pattern_variant, target_channel) = if config.pattern == "diagonal" {
        ("diagonal", "Diagonal".to_owned(), 0)
    } else {
        let column = config.pattern.strip_prefix("col_")
            .filter(|value| !value.is_empty() && value.bytes().all(|b| b.is_ascii_digit()))
            .and_then(|value| value.parse::<usize>().ok())
            .expect("config.json: pattern must be diagonal or col_x (x is a nonnegative decimal column)");
        assert!(column < usize::from(config.chains) * 15,
            "config.json: col_x column must be in 0..chains*15-1 (OUT15 is unconnected)");
        ("col", format!("Column({column})"), column)
    };
    assert!(
        [1.0, 3.0, 6.0, 10.0, 12.5, 20.0, 25.0, 26.0, 28.0, 30.0, 31.0, 32.0, 33.0, 34.0, 35.0, 40.0, 45.0].contains(&config.dclk_mhz),
        "config.json: dclk_mhz must be 1, 3, 6, 10, 12.5, 20, 25, 26, 28, 30, 31, 32, 33, 34, 35, 40 or 45"
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
    println!("cargo:rustc-env=ICND2069_LINES={}", config.lines);
    println!("cargo:rustc-env=ICND2069_KEEP_SCAN_TIME={}", u8::from(config.keep_scan_time));
    println!("cargo:rustc-env=ICND2069_CONFIG_INTERVAL={}", config.config_interval);
    println!("cargo:rustc-env=ICND2069_CHAIN_COPIES={}", config.chains);
    println!("cargo:rustc-env=ICND2069_CFG_CHAIN_COPIES={cfg_chains}");
    for (bank, address) in config.cpld_addresses.iter().enumerate() {
        println!("cargo:rustc-env=ICND2069_CPLD_ADDRESS_{}={address}", bank + 1);
    }
    println!("cargo:rustc-env=ICND2069_CFG_PRE_CLR_CHAINS={}", config.cfg_pre_clr_chains);
    println!("cargo:rustc-env=ICND2069_ROW_ON_CFG={}", u8::from(config.row_on_cfg));
    println!("cargo:rustc-env=ICND2069_DISABLE_RT5990={}", u8::from(config.disable_rt5990));
    println!("cargo:rustc-env=ICND2069_DCLK_MHZ={}", config.dclk_mhz);
    println!("cargo:rustc-env=ICND2069_CFG_DCLK_HZ={}", cfg_dclk_hz.round() as u32);
    println!("cargo:rustc-env=ICND2069_GCLK_HZ={gclk_hz}");
    println!("cargo:rustc-env=ICND2069_PWM_WIDTH={}", config.pwm_width);
    let pattern_name = &config.pattern;
    println!("cargo:rustc-env=ICND2069_PATTERN={pattern_kind}");
    println!("cargo:rustc-env=ICND2069_COLOR={}", config.color);
    println!("cargo:rustc-env=ICND2069_GRAY_MODE=static");
    println!("cargo:rustc-env=ICND2069_CONTINUOUS_LATCH={}", u8::from(continuous_latch));
    println!("cargo:rustc-env=ICND2069_SHORT_DATA={}", u8::from(continuous_latch));
    println!("cargo:rustc-env=ICND2069_REPEAT_LATCH_GAP={latch_gap}");
    println!("cargo:rustc-env=ICND2069_TARGET_CHANNEL={target_channel}");
    fs::write(
        out.join("pattern_config.rs"),
        format!(
            "const PATTERN: protocol::Pattern = protocol::Pattern::{pattern_variant};\n\
             const PATTERN_NAME: &str = \"{pattern_name}\";\n\
             const GRAY_RANGE: [u16; 2] = {gray:?};\n"
        ),
    ).unwrap();
}
