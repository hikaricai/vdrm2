use serde::Deserialize;
use std::{env, fs, path::PathBuf};

#[derive(Deserialize)]
// Unknown fields are ignored so renamed options and debug notes can stay.
struct Config {
    #[serde(default = "default_board")]
    board: String,
    #[serde(default = "default_latch_gap")]
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
    #[serde(default)]
    timing: Option<Timing>,
    #[serde(default)]
    regs: Option<std::collections::BTreeMap<String, String>>,
    #[serde(default)]
    regs_high_level: Option<RegisterOverrides>,
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

#[derive(Deserialize)]
struct RegisterOverrides {
    lines: Option<u8>,
    scan_repeats: Option<u8>,
    pwm_width: Option<u8>,
    gclk: Option<String>,
    gclk_n_code: Option<u8>,
}

impl RegisterOverrides {
    fn apply(&self, registers: &mut std::collections::BTreeMap<u8, [u8; 3]>) {
        let mut set = |addr: u8, mask: u8, value: u8| {
            let rgb = registers.get_mut(&addr)
                .unwrap_or_else(|| panic!("config.json: missing register 0x{addr:02X}"));
            for old in rgb { *old = (*old & !mask) | value; }
        };
        if let Some(lines) = self.lines {
            assert!((1..=64).contains(&lines), "config.json: regs_high_level.lines must be 1..=64");
            set(0x02, 0x3f, lines - 1);
        }
        if let Some(repeats) = self.scan_repeats {
            assert!((1..=64).contains(&repeats), "config.json: regs_high_level.scan_repeats must be 1..=64");
            set(0x03, 0x7f, repeats - 1);
        }
        if let Some(width) = self.pwm_width {
            assert!(width >= 4, "config.json: regs_high_level.pwm_width must be 4..=255");
            set(0x07, 0xff, width);
            set(0x08, 0xff, width);
        }
        match (&self.gclk, self.gclk_n_code) {
            (Some(expression), Some(code)) => {
                let format = "config.json: gclk must be P/(M*N) with integer factors";
                let (p, factors) = expression.split_once('/').expect(format);
                let factors = factors.trim().strip_prefix('(')
                    .and_then(|s| s.strip_suffix(')')).expect(format);
                let (m, n) = factors.split_once('*').expect(format);
                let parse = |s: &str| s.trim().parse::<u8>().expect(format);
                let (m, n, p) = (parse(m), parse(n), parse(p));
                assert!((1..=31).contains(&m) && (1..=31).contains(&p),
                    "config.json: gclk M and P must be 1..=31");
                assert!(matches!(n, 1 | 2 | 4) && code <= 3,
                    "config.json: gclk N must be 1, 2 or 4; gclk_n_code must be 0..=3");
                set(0x04, 0x1f, m);
                set(0x05, 0x1f, p);
                // The guide does not map mathematical N to the hardware encoding.
                set(0x06, 0x03, code);
            }
            (None, None) => {}
            _ => panic!("config.json: set gclk and gclk_n_code together"),
        }
    }
}

#[derive(Deserialize)]
struct Timing {
    en_start: Option<u32>,
    gray_start: u32,
    row_start: u32,
    gray_word_gap: u32,
    gray_row_gap: u32,
    row_period: u32,
    frame_tail: u32,
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
    fn mask(&self, board: &str) -> u32 {
        let names = match self {
            Self::One(name) => std::slice::from_ref(name),
            Self::Many(names) => names.as_slice(),
        };
        let mapping: &[(&str, u8)] = if board == "dev" { &[
            ("sin", 13), ("le", 10), ("dclk", 15), ("row", 22),
            ("lck", 16), ("lbk", 18), ("ldata", 20),
        ] } else if board == "direct" { &[
            ("r1", 5), ("g1", 4), ("b1", 3),
            ("r2", 11), ("g2", 10), ("b2", 9),
            ("r3", 2), ("g3", 1), ("b3", 0),
            ("r4", 8), ("g4", 7), ("b4", 6),
            ("dclk", 19), ("le", 18), ("row", 13),
            ("lck", 16), ("lbk", 15), ("ldata", 17),
        ] } else { &[
            ("r1", 0), ("g1", 1), ("b1", 2), ("sel1", 3),
            ("r2", 4), ("g2", 5), ("b2", 6), ("sel2", 7),
            ("r3", 8), ("g3", 9), ("b3", 10), ("sel3", 11),
            ("lat1", 12), ("lat2", 13), ("lat3", 14), ("dclk", 15),
            ("lck", 16), ("dclk2", 17), ("lbk", 18), ("ldata", 20), ("row", 22),
        ] };
        names.iter().fold(0, |mask, name| {
            let (_, pin) = mapping.iter().find(|(field, _)| *field == name)
                .unwrap_or_else(|| panic!("config.json: pin_drive.pins: unknown {board} Pins field {name:?}"));
            mask | (1u32 << pin)
        })
    }
}

fn default_board() -> String {
    "hdl".into()
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
    Text(String),
}

fn parse_gray_word(word: &str) -> u16 {
    let word = word.trim();
    if let Some(hex) = word.strip_prefix("0x").or_else(|| word.strip_prefix("0X")) {
        u16::from_str_radix(hex, 16)
    } else {
        word.parse()
    }.expect("config.json: gray words must be in 0..=65535 (0x0000..=0xffff)")
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
    let mut config: Config = serde_json::from_str(&json).expect("invalid config.json");
    assert!(config.regs_high_level.is_none() || config.regs.is_some(),
        "config.json: regs_high_level requires regs");
    let mut raw_regs = String::new();
    if let Some(regs) = &config.regs {
        let fields: serde_json::Value = serde_json::from_str(&json).unwrap();
        for key in ["lines", "scan_repeats", "gclk_mhz", "pwm_width", "keep_scan_time"] {
            assert!(fields.get(key).is_none(),
                "config.json: regs controls registers directly; rename {key} to _{key}");
        }
        let byte = |text: &str| -> u8 {
            let hex = text.trim().strip_prefix("0x")
                .expect("config.json: register addresses/values require 0xXX");
            assert_eq!(hex.len(), 2, "config.json: register byte must be 0xXX");
            u8::from_str_radix(hex, 16).expect("config.json: invalid register hex byte")
        };
        let mut values = std::collections::BTreeMap::new();
        for (key, value) in regs {
            if key.starts_with('_') { continue; }
            let addr = byte(key);
            assert!(matches!(addr, 0x02..=0x23 | 0x70..=0x74 | 0xf0..=0xf5),
                "config.json: unsupported register {key}; 00/01 are automatic access controls");
            let parts: Vec<_> = value.split(',').map(byte).collect();
            let rgb = match parts.as_slice() {
                [v] => [*v; 3],
                [r, g, b] => [*r, *g, *b],
                _ => panic!("config.json: {key} requires one byte or R,G,B bytes"),
            };
            assert!(values.insert(addr, rgb).is_none(), "config.json: duplicate register {key}");
        }
        if let Some(overrides) = &config.regs_high_level {
            overrides.apply(&mut values);
        }
        for addr in (0x02..=0x22).chain(0x70..=0x74).chain(0xf0..=0xf5).chain([0x23]) {
            let rgb = values.get(&addr)
                .unwrap_or_else(|| panic!("config.json: missing register 0x{addr:02X}"));
            raw_regs.push_str(&format!("{addr:02X}{:02X}{:02X}{:02X}", rgb[0], rgb[1], rgb[2]));
        }
        for addr in [0x02, 0x03] {
            let rgb = values[&addr];
            assert!(rgb[0] == rgb[1] && rgb[1] == rgb[2],
                "config.json: shared ROW requires matching RGB scan registers");
        }
        config.lines = (values[&0x02][0] & 0x3f) + 1;
        config.scan_repeats = (values[&0x03][0] & 0x7f) + 1;
    }
    println!("cargo:rustc-env=ICND2069_REGS={raw_regs}");
    println!("cargo:rustc-env=ICND2069_TIMING={}", u8::from(config.timing.is_some()));
    // Emit all fields even when absent, so shell exports cannot change JSON.
    let timing = config.timing.as_ref();
    match timing.and_then(|t| t.en_start) {
        Some(start) => println!("cargo:rustc-env=ICND2069_TIMING_EN_START={start}"),
        None => println!("cargo:rustc-env=ICND2069_TIMING_EN_START=off"),
    }
    for (key, value) in [
        ("GRAY_START", timing.map_or(732, |t| t.gray_start)),
        ("ROW_START", timing.map_or(28, |t| t.row_start)),
        ("GRAY_WORD_GAP", timing.map_or(8, |t| t.gray_word_gap)),
        ("GRAY_ROW_GAP", timing.map_or(1, |t| t.gray_row_gap)),
        ("ROW_PERIOD", timing.map_or(81, |t| t.row_period)),
        ("FRAME_TAIL", timing.map_or(430, |t| t.frame_tail)),
    ] {
        println!("cargo:rustc-env=ICND2069_TIMING_{key}={value}");
    }
    assert!(matches!(config.board.as_str(), "dev" | "hdl" | "direct"),
        "config.json: board must be dev, hdl or direct");
    let dev = config.board == "dev";
    for board in ["dev", "hdl", "direct"] {
        println!("cargo:rustc-check-cfg=cfg(icnd2069_{board})");
    }
    println!("cargo:rustc-cfg=icnd2069_{}", config.board);
    println!("cargo:rustc-env=ICND2069_BOARD={}", config.board);
    let led_channels = if dev { 16 } else { 15 };
    let (drive_ma, cfg_drive_ma, slew, cfg_slew, pin_mask) = match &config.pin_drive {
        Some(drive) => (
            drive.strength.milliamps(),
            drive.cfg_strength.unwrap_or(drive.strength).milliamps(),
            drive.slew_rate.variant(),
            drive.cfg_slew_rate.unwrap_or(drive.slew_rate).variant(),
            drive.pins.mask(&config.board),
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
    if config.board != "hdl" {
        assert!((1..=8).contains(&config.chains), "config.json: dev/direct chains must be 1..=8");
    } else {
        assert_eq!(config.chains, 2, "config.json: hdl board requires chains=2");
    }
    let cfg_chains = config.cfg_chains.unwrap_or(config.chains);
    assert!((1..=8).contains(&cfg_chains), "config.json: cfg_chains must be 1..=8");
    assert!(config.cpld_addresses.iter().all(|&a| a <= 7),
        "config.json: cpld_addresses must contain three addresses in 0..=7");
    assert!(config.cfg_pre_clr_chains <= 64, "config.json: cfg_pre_clr_chains must be 0..=64");
    assert!(matches!(config.color.as_str(), "r" | "g" | "b" | "rg" | "rb" | "gb" | "rgb"),
        "config.json: color must be r, g, b, rg, rb, gb or rgb");
    let (pattern_kind, pattern_variant, target_channel) = if config.pattern == "diagonal" {
        ("diagonal", "Diagonal".to_owned(), 0)
    } else if config.pattern == "diagonal_rev" {
        ("diagonal_rev", "DiagonalReverse".to_owned(), 0)
    } else {
        let column = config.pattern.strip_prefix("col_")
            .filter(|value| !value.is_empty() && value.bytes().all(|b| b.is_ascii_digit()))
            .and_then(|value| value.parse::<usize>().ok())
            .expect("config.json: pattern must be diagonal, diagonal_rev or col_x (x is a nonnegative decimal column)");
        assert!(column < usize::from(config.chains) * led_channels,
            "config.json: col_x column must be in 0..chains*{led_channels}-1 for {}", config.board);
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
        Gray::Text(text) => match text.split_once(',') {
            Some((start, end)) => [parse_gray_word(start), parse_gray_word(end)],
            None => [parse_gray_word(&text); 2],
        },
    };
    let (continuous_latch, latch_gap) = match config.repeat_latch {
        Some(RepeatLatch { repeat_latch_gap }) => {
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
