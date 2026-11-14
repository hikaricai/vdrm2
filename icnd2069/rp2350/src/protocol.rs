//! ICND2069 configurable-scan image and full configuration using fixed DCLK timing.
//! Each u32 describes pins for one DCLK rising-edge transfer.
//! Reference: captures/reference/nova_to_devboard_100m_2s.dsl and the startup summary.

/// Scan rows from config.json; register 02h receives ROWS - 1.
pub const ROWS: usize = {
    let rows = match option_env!("ICND2069_LINES") {
        Some(value) => unsigned_setting(value) as usize,
        None => 54,
    };
    assert!(rows >= 1 && rows <= 64, "lines must be 1..=64");
    rows
};
pub const CHANNELS: usize = 16;
/// The single-chip test board connects all OUT0..OUT15 to COL0..COL15.
pub const LED_CHANNELS: usize = 16;
/// Number of complete ROWS-row sweeps per frame; build-time variable, default 1.
pub const SCAN_REPEATS: usize = match option_env!("ICND2069_SCAN_REPEATS") {
    Some(value) => {
        let bytes = value.as_bytes();
        assert!(!bytes.is_empty());
        let mut n = 0;
        let mut i = 0;
        while i < bytes.len() {
            assert!(bytes[i] >= b'0' && bytes[i] <= b'9');
            n = n * 10 + (bytes[i] - b'0') as usize;
            i += 1;
        }
        assert!(n >= 1 && n <= 64);
        n
    }
    None => 1,
};
pub const GROUPS: usize = SCAN_REPEATS;
/// Fit all sweeps into the gray-loading window, regardless of repeat count.
pub const KEEP_SCAN_TIME: bool = match option_env!("ICND2069_KEEP_SCAN_TIME") {
    Some(value) => match value.as_bytes() {
        b"0" => false,
        b"1" => true,
        _ => panic!("ICND2069_KEEP_SCAN_TIME must be 0 or 1"),
    },
    None => false,
};
/// Build-time preset in MHz. Integer PIO dividers keep each DCLK uniform.
pub const DCLK_HZ: u32 = match option_env!("ICND2069_DCLK_MHZ") {
    Some(value) => match value.as_bytes() {
        b"1" => 1_000_000,
        b"3" => 3_000_000,
        b"6" => 6_000_000,
        b"10" => 10_000_000,
        b"12.5" => 12_500_000,
        b"20" => 20_000_000,
        b"25" => 25_000_000,
        b"26" => 26_000_000,
        b"28" => 28_000_000,
        b"30" => 30_000_000,
        b"31" => 31_000_000,
        b"32" => 32_000_000,
        b"33" => 33_000_000,
        b"34" => 34_000_000,
        b"35" => 35_000_000,
        b"40" => 40_000_000,
        b"45" => 45_000_000,
        _ => panic!("ICND2069_DCLK_MHZ must be 1, 3, 6, 10, 12.5, 20, 25, 26, 28, 30, 31, 32, 33, 34, 35, 40 or 45"),
    },
    None => 12_500_000,
};
pub const PIO_CYCLES_PER_DCLK: u32 = if DCLK_HZ >= 26_000_000 { 4 } else { 6 };
pub const SYS_HZ: u32 = match DCLK_HZ {
    // Every supported preset >=26 MHz uses divider 1, four cycles/DCLK.
    26_000_000..=45_000_000 => DCLK_HZ * 4,
    10_000_000 | 20_000_000 => 120_000_000,
    // Six PIO cycles/DCLK: integer dividers 8 and 4 respectively.
    3_000_000 | 6_000_000 => 144_000_000,
    _ => 150_000_000,
};
// Pico 2's 12 MHz crystal feeds the integer system PLL (refdiv=1).
// 104/112 MHz use /3/3 to keep their VCO above 750 MHz.
// 124 MHz cannot use /3/2: its VCO would be 744 MHz, below 750 MHz.
// Use 1488 MHz /4/3 instead. Keep the standard 1500 MHz /5/2 at 150 MHz.
pub const SYS_PLL_FBDIV: u16 = match SYS_HZ {
    104_000_000 => 78,
    112_000_000 => 84,
    120_000_000 => 80,
    124_000_000 => 124,
    150_000_000 => 125,
    _ => (SYS_HZ / 2_000_000) as u16,
};
pub const SYS_PLL_POST_DIV1: u8 = match SYS_HZ {
    120_000_000 | 124_000_000 => 4,
    150_000_000 => 5,
    _ => 3,
};
pub const SYS_PLL_POST_DIV2: u8 = match SYS_HZ {
    104_000_000 | 112_000_000 | 124_000_000 => 3,
    _ => 2,
};
/// Compile-time proof of an exact PLL ratio and zero fractional PIO divider.
pub const PIO_CLKDIV_INT: u32 = {
    let vco = 12_000_000u64 * SYS_PLL_FBDIV as u64;
    let postdiv = SYS_PLL_POST_DIV1 as u64 * SYS_PLL_POST_DIV2 as u64;
    assert!(SYS_PLL_FBDIV >= 16 && SYS_PLL_FBDIV <= 320);
    assert!(SYS_PLL_POST_DIV1 >= 1 && SYS_PLL_POST_DIV1 <= 7);
    assert!(SYS_PLL_POST_DIV2 >= 1 && SYS_PLL_POST_DIV2 <= SYS_PLL_POST_DIV1);
    assert!(vco >= 750_000_000 && vco <= 1_600_000_000);
    assert!(vco == SYS_HZ as u64 * postdiv);
    let denominator = PIO_CYCLES_PER_DCLK * DCLK_HZ;
    assert!(SYS_HZ % denominator == 0);
    let divider = SYS_HZ / denominator;
    assert!(divider >= 1 && divider <= 65535);
    divider
};
// Guide V4.2 p.13: M/P occupy five bits; 4 MHz < DCLK/M < 10 MHz.
// Without an explicit GCLK target retain the historical presets, including
// the >=20 MHz 5x experiments and the out-of-range 1/3 MHz input experiments.
const LEGACY_PLL_MP: (u8, u8) = match DCLK_HZ {
    1_000_000 | 3_000_000 | 6_000_000 => (1, 10),
    10_000_000 | 12_500_000 => (2, 12),
    20_000_000 | 25_000_000 | 26_000_000 | 28_000_000 => (3, 15),
    30_000_000..=35_000_000 => (4, 20),
    40_000_000 | 45_000_000 => (5, 25),
    _ => panic!("missing ICND2069 PLL preset"),
};
const fn unsigned_setting(value: &str) -> u32 {
    let bytes = value.as_bytes();
    assert!(!bytes.is_empty());
    let mut n = 0u32;
    let mut i = 0;
    while i < bytes.len() {
        assert!(bytes[i] >= b'0' && bytes[i] <= b'9');
        n = n * 10 + (bytes[i] - b'0') as u32;
        i += 1;
    }
    n
}
pub const CFG_DCLK_HZ: u32 = match option_env!("ICND2069_CFG_DCLK_HZ") {
    Some(value) => unsigned_setting(value),
    None => DCLK_HZ,
};
/// Exact 16.8 divider for the same fixed PIO program at configuration speed.
pub const CFG_CLKDIV_BITS: u32 = {
    assert!(CFG_DCLK_HZ > 0 && CFG_DCLK_HZ <= DCLK_HZ,
        "cfg_dclk_mhz must be positive and no faster than dclk_mhz");
    assert!(SYS_HZ % (2 * CFG_DCLK_HZ) == 0,
        "cfg_dclk_mhz needs an integer PIO half-period");
    let numerator = (SYS_HZ as u64) << 8;
    let denominator = PIO_CYCLES_PER_DCLK as u64 * CFG_DCLK_HZ as u64;
    assert!(numerator % denominator == 0,
        "cfg_dclk_mhz needs an exact 16.8 PIO divider");
    let bits = numerator / denominator;
    assert!(bits >= 256 && bits <= 0x00ff_ffff, "cfg_dclk_mhz divider exceeds 65535");
    bits as u32
};
/// Zero selects the legacy PLL preset. Cargo emits this from config.json.
pub const GCLK_TARGET_HZ: u32 = match option_env!("ICND2069_GCLK_HZ") {
    Some(value) => unsigned_setting(value),
    None => 0,
};
const PLL_MP: (u8, u8) = if GCLK_TARGET_HZ == 0 {
    LEGACY_PLL_MP
} else if DCLK_HZ == 12_500_000 && GCLK_TARGET_HZ == 31_250_000 {
    // dsview/icnd2069/ICND2069-抓包寄存器汇总.md: 04=04, 05=0A.
    // Deliberate capture-replay exception: DCLK/M=3.125 MHz is below
    // the guide's range. Do not substitute the equivalent ratio M=2/P=5.
    (4, 10)
} else {
    assert!(GCLK_TARGET_HZ > 25_000_000, "GCLK must exceed 25 MHz");
    let dclk = DCLK_HZ as u64;
    let mut m = 1u64;
    let mut result = (0, 0);
    while m <= 31 {
        let product = GCLK_TARGET_HZ as u64 * m;
        let p = product / dclk;
        if 4_000_000 * m < dclk && dclk < 10_000_000 * m
            && product % dclk == 0 && p >= 1 && p <= 31
        {
            result = (m as u8, p as u8);
            break;
        }
        m += 1;
    }
    assert!(result.0 != 0, "GCLK target has no exact valid M/P pair");
    result
};
pub const PLL_M: u8 = PLL_MP.0;
pub const PLL_P: u8 = PLL_MP.1;
// Retain register 06h=01h. Its mapping to the mathematical divisor is not
// explicit in the guide; frequency and row-time calculations ASSUME N=1.
pub const PLL_N: u8 = 0x01;
/// Guide p.14: reg07=reg08, nominal acceleration 256/PWM_WIDTH,
/// with 4*PWM_WIDTH display GCLK ticks per row/group.
pub const PWM_WIDTH: u8 = {
    let width = match option_env!("ICND2069_PWM_WIDTH") {
        Some(value) => unsigned_setting(value),
        None => 16,
    };
    assert!(width >= 4 && width <= 255, "PWM width must be 4..255 for current RGB reg0C");
    width as u8
};
pub const GCLK_HZ_ASSUMING_N1: u32 = {
    let dclk = DCLK_HZ as u64;
    let m = PLL_M as u64;
    let p = PLL_P as u64;
    assert!(m >= 1 && m <= 31 && p >= 1 && p <= 31);
    if DCLK_HZ >= 20_000_000 {
        assert!(4_000_000 * m < dclk && dclk < 10_000_000 * m);
    }
    (dclk * p / m) as u32
};
// External scan timing is independent of the chip's PWM/PLL registers.
// keep_scan_time may instead fit all rows into the gray-loading window.
// ROW high remains 12 DCLK for the first pulse and four thereafter.
pub const BASE_ROW_DCLKS: usize = 60;
// Software input limit retained from the saved NOVA image. This is NOT a
// calibrated PWM full scale. The previous 40-scan capture saturated by 512;
// the new 16-scan/full-config mode must be measured independently.
pub const MAX_GRAY_INPUT: u16 = if CONTINUOUS_LATCH { 255 } else { 8192 };
#[cfg(not(target_os = "none"))]
pub const DEMO_GRAY: u16 = 8192;
/// Grayscale ICND2069 chips per input; chip 0 is nearest the MCU.
pub const CHAIN_COPIES: usize = match option_env!("ICND2069_CHAIN_COPIES") {
    Some(value) => {
        let bytes = value.as_bytes();
        assert!(bytes.len() == 1 && bytes[0] >= b'1' && bytes[0] <= b'8');
        (bytes[0] - b'0') as usize
    }
    None => 1,
};
pub const TRANSFER_DCLKS: usize = 16 * CHAIN_COPIES;
/// Configuration cascade length; defaults to the grayscale cascade length.
pub const CFG_CHAIN_COPIES: usize = {
    let copies = match option_env!("ICND2069_CFG_CHAIN_COPIES") {
        Some(value) => unsigned_setting(value) as usize,
        None => CHAIN_COPIES,
    };
    assert!(copies >= 1 && copies <= 8, "cfg_chains must be 1..=8");
    copies
};
pub const CFG_TRANSFER_DCLKS: usize = 16 * CFG_CHAIN_COPIES;
/// Flush all configured/data shift registers after locking configuration.
/// LE stays low: these zeros do not latch an image or switch display buffers.
pub const CFG_POST_CLR_DCLKS: usize = if CFG_TRANSFER_DCLKS > TRANSFER_DCLKS {
    CFG_TRANSFER_DCLKS
} else {
    TRANSFER_DCLKS
};
/// Extra zero-valued 16-bit chip equivalents before each WR_CFG.
pub const CFG_PRE_CLR_DCLKS: usize = {
    let chains = match option_env!("ICND2069_CFG_PRE_CLR_CHAINS") {
        Some(value) => unsigned_setting(value) as usize,
        None => 0,
    };
    assert!(chains <= 64, "cfg_pre_clr_chains must be 0..=64");
    chains * 16
};
pub const CFG_WRITE_DCLKS: usize = CFG_PRE_CLR_DCLKS + CFG_TRANSFER_DCLKS;
/// Whether the configuration frame may issue VSYNC and scan.
pub const ROW_ON_CFG: bool = match option_env!("ICND2069_ROW_ON_CFG") {
    Some(value) => match value.as_bytes() {
        b"0" => false,
        b"1" => true,
        _ => panic!("row_on_cfg must be boolean"),
    },
    None => false,
};
/// Logical image width includes all connected outputs.
pub const TOTAL_CHANNELS: usize = LED_CHANNELS * CHAIN_COPIES;
// Starting with a cleared chain, shift one gray byte and move it to the
// selected chip. Clearing afterwards only needs to flush that byte out.
pub const fn latch_gray_dclks(row: usize) -> usize {
    8 + 16 * latch_chip(row)
}
pub const fn latch_clear_dclks(row: usize) -> usize {
    16 * (CHAIN_COPIES - latch_chip(row))
}
#[cfg(not(target_os = "none"))]
pub const COLUMN_PATTERN: bool = match option_env!("ICND2069_PATTERN") {
    Some(value) => matches!(value.as_bytes(), b"col"),
    None => false,
};
/// Both diagonal and fixed-column patterns permit repeated zero latches.
pub const CONTINUOUS_LATCH: bool = match option_env!("ICND2069_CONTINUOUS_LATCH") {
    Some(value) => {
        let bytes = value.as_bytes();
        assert!(bytes.len() == 1 && (bytes[0] == b'0' || bytes[0] == b'1'));
        bytes[0] == b'1'
    }
    None => match option_env!("ICND2069_SHORT_DATA") {
    Some(value) => {
        let bytes = value.as_bytes();
        assert!(bytes.len() == 1 && (bytes[0] == b'0' || bytes[0] == b'1'));
        bytes[0] == b'1'
    }
    None => true,
    },
};
/// Compatibility alias for waveform sizing and existing host tools.
pub const SHORT_DATA: bool = CONTINUOUS_LATCH;
/// The diagonal advances the selected OUT with each logical data row.
pub const DIAGONAL_LATCH: bool = match option_env!("ICND2069_PATTERN") {
    Some(value) => matches!(value.as_bytes(), b"diagonal"),
    None => true,
};
pub const fn latch_pixel(row: usize) -> usize {
    if DIAGONAL_LATCH { row % TOTAL_CHANNELS } else { TARGET_CHANNEL }
}
pub const fn latch_target(row: usize) -> usize {
    latch_pixel(row) % LED_CHANNELS
}
pub const fn latch_chip(row: usize) -> usize {
    latch_pixel(row) / LED_CHANNELS
}
/// Logical column (chip * LED_CHANNELS + OUT). Latch order is OUT15 through OUT0.
pub const TARGET_CHANNEL: usize = match option_env!("ICND2069_TARGET_CHANNEL") {
    Some(value) => {
        let bytes = value.as_bytes();
        let mut n = 0;
        let mut i = 0;
        assert!(!bytes.is_empty());
        while i < bytes.len() {
            assert!(bytes[i] >= b'0' && bytes[i] <= b'9');
            n = n * 10 + (bytes[i] - b'0') as usize;
            i += 1;
        }
        assert!(n < TOTAL_CHANNELS);
        n
    }
    None => 0,
};
/// Complete LAT-low DCLK cycles between repeated one-clock DATA_LATCH pulses.
pub const REPEAT_LATCH_GAP: usize = match option_env!("ICND2069_REPEAT_LATCH_GAP") {
    Some(value) => {
        let bytes = value.as_bytes();
        let mut n = 0;
        let mut i = 0;
        assert!(!bytes.is_empty());
        while i < bytes.len() {
            assert!(bytes[i] >= b'0' && bytes[i] <= b'9');
            n = n * 10 + (bytes[i] - b'0') as usize;
            i += 1;
        }
        assert!(n >= 1 && n <= 64);
        n
    }
    None => 1,
};

// SCH_Schematic3_2026-10-24.pdf: a single SIN on the old R1 network.
// Use the reference R register values; there are no G/B data inputs.
pub const PIO_OUT_COUNT: u8 = 23;
pub const SIN: u32 = 1 << 13;
pub const RGB_SIN: [u32; 3] = [SIN, 0, 0];
pub const COLOR: &str = match option_env!("ICND2069_COLOR") {
    Some(value) => value,
    None => "rgb",
};
/// Gray data only. Register writes and clear operations still address all colors.
pub const GRAY_SIN: u32 = match COLOR.as_bytes() {
    b"r" => RGB_SIN[0],
    b"g" => RGB_SIN[1],
    b"b" => RGB_SIN[2],
    b"rg" => RGB_SIN[0] | RGB_SIN[1],
    b"rb" => RGB_SIN[0] | RGB_SIN[2],
    b"gb" => RGB_SIN[1] | RGB_SIN[2],
    b"rgb" => SIN,
    _ => panic!("color must be r, g, b, rg, rb, gb or rgb"),
};
pub const LE: u32 = 1 << 10;
pub const LCK: u32 = 1 << 16;
pub const BK: u32 = 1 << 18;
pub const DIN: u32 = 1 << 20;
pub const ROW: u32 = 1 << 22;
// DCLK on GP15 is driven by side-set. All waveform words keep that bit low.
// The reference's six channels do not measure RT5990 DIN/LCK/BK.
// Preserve the existing board assumptions; do not claim measured parity.
pub const BK_ACTIVE_HIGH: bool = true;
pub const DIN_ONE_SELECTS: bool = true;
const fn blank(on: bool) -> u32 {
    if on == BK_ACTIVE_HIGH {
        BK
    } else {
        0
    }
}
const fn select_bit(selected: bool) -> u32 {
    if selected == DIN_ONE_SELECTS {
        DIN
    } else {
        0
    }
}
// BK is inactive outside each row pulse, preventing an over-wide first pulse.
pub const PARK: u32 = blank(false) | select_bit(false);
pub const VSYNC: usize = 3;
pub const WR_CFG: usize = 5;
pub const EN_OP: usize = 11;
pub const PRE_ACT: usize = 14;

// R/G/B values from dsview/icnd2069/ICND2069-抓包寄存器汇总.md (B/C).
// Override the reference row/group/PLL settings and PWM width from JSON.
// 00/01 are access controls, sent outside this list to keep the whole batch
// unlocked. The reference rotation includes them but they are not payload.
pub const REGISTERS: &[(u8, [u8; 3])] = &[
    (0x02, [(ROWS - 1) as u8; 3]),
    (0x03, [(GROUPS - 1) as u8; 3]),
    (0x04, [PLL_M; 3]),
    (0x05, [PLL_P; 3]),
    (0x06, [PLL_N; 3]),
    (0x07, [PWM_WIDTH; 3]),
    (0x08, [PWM_WIDTH; 3]),
    (0x09, [0x08; 3]),
    (0x0a, [0x08; 3]),
    (0x0b, [0x00; 3]),
    (0x0c, [0x01, 0x0c, 0x0d]),
    (0x0d, [0x01; 3]),
    (0x0e, [0x07, 0x06, 0x0a]),
    (0x0f, [0x01; 3]),
    (0x10, [0xc2; 3]),
    (0x11, [0x21; 3]),
    (0x12, [0x01; 3]),
    (0x13, [0x00; 3]),
    (0x14, [0x00; 3]),
    (0x15, [0x00; 3]),
    (0x16, [0x00; 3]),
    (0x17, [0xf0; 3]),
    (0x18, [0x1f, 0x1f, 0x2f]),
    (0x19, [0x20, 0x80, 0xa0]),
    (0x1a, [0x1f; 3]),
    (0x1b, [0x10; 3]),
    (0x1c, [0x72, 0x76, 0x64]),
    (0x1d, [0x82; 3]),
    (0x1e, [0x41, 0x46, 0x48]),
    (0x1f, [0x04, 0x20, 0x20]),
    (0x20, [0x48; 3]),
    (0x21, [0x01; 3]),
    (0x22, [0x1c; 3]),
    (0x70, [0x00; 3]),
    (0x71, [0x00; 3]),
    (0x72, [0x00; 3]),
    (0x73, [0x00; 3]),
    (0x74, [0x00; 3]),
    (0xf0, [0x00; 3]),
    (0xf1, [0x00; 3]),
    (0xf2, [0x00; 3]),
    (0xf3, [0x00; 3]),
    (0xf4, [0x00; 3]),
    (0xf5, [0x00; 3]),
    (0x23, [0x01, 0x01, 0x05]),
];

fn config_words(address: u8, values: [u8; 3]) -> [u16; 3] {
    values.map(|value| (address as u16) << 8 | value as u16)
}

fn rgb_bit(values: [u16; 3], bit: usize) -> u32 {
    let mut pins = 0;
    for color in 0..3 {
        if values[color] & (1 << bit) != 0 {
            pins |= RGB_SIN[color];
        }
    }
    pins
}
pub const CONFIG_WRITES: usize = REGISTERS.len() + 4;
pub const CONFIG_INTERVAL: u32 = match option_env!("ICND2069_CONFIG_INTERVAL") {
    Some(value) => {
        let bytes = value.as_bytes();
        assert!(!bytes.is_empty());
        let mut n: u32 = 0;
        let mut i = 0;
        while i < bytes.len() {
            assert!(bytes[i] >= b'0' && bytes[i] <= b'9');
            n = n * 10 + (bytes[i] - b'0') as u32;
            i += 1;
        }
        assert!(n > 0);
        n
    }
    None => 10000,
};
// One word per clock. Keep the same 34-clock prefix and even clock boundary
// as the former packed format, so configuration skipping preserves timing.
pub const COMMON_PREFIX_WORDS: usize = 34;
/// Suppress the display-buffer switch during a configuration-only frame.
/// Keep all prefix clocks so both DMA streams retain their original timing.
#[cfg(not(target_os = "none"))]
pub static CONFIG_IDLE_PREFIX: [u32; COMMON_PREFIX_WORDS] = [PARK; COMMON_PREFIX_WORDS];
/// Physical RT5990 chain, independent of the configured ICND2069 scan rows.
pub const RT5990_CLEAR_ROWS: usize = 16;
/// Two setup clocks, 16 LCK pulses (two high/two low), two trailing clocks.
/// SIN/LE also stay zero here; each ICND2069 write has its own extra preclear.
pub const RT5990_CLEAR_DCLKS: usize = 4 + RT5990_CLEAR_ROWS * 4;
pub const CONFIG_COMMAND_START: usize = COMMON_PREFIX_WORDS + RT5990_CLEAR_DCLKS;
pub const DATA_START: usize = CONFIG_COMMAND_START + 29
    + CONFIG_WRITES * (8 + CFG_WRITE_DCLKS) + CFG_POST_CLR_DCLKS;
pub const CONFIG_END_WORDS: usize = (DATA_START + 1) & !1;
pub const CONFIG_SKIP_DCLKS: usize = CONFIG_END_WORDS - COMMON_PREFIX_WORDS;
/// Full-frame duration in PIO cycles, including slow configuration clocks.
#[cfg(not(target_os = "none"))]
pub const CONFIG_FRAME_PIO_CYCLES: u64 = (FRAME_DCLKS + CFG_POST_CLR_DCLKS) as u64 * PIO_CYCLES_PER_DCLK as u64
    + CONFIG_END_WORDS as u64
        * (SYS_HZ / PIO_CLKDIV_INT / CFG_DCLK_HZ - PIO_CYCLES_PER_DCLK) as u64;
#[cfg(not(target_os = "none"))]
pub const NORMAL_FRAME_PIO_CYCLES: u64 =
    NORMAL_FRAME_DCLKS as u64 * PIO_CYCLES_PER_DCLK as u64;
pub const DATA_ROW_DCLKS: usize = if SHORT_DATA {
    TRANSFER_DCLKS + 8 + (CHANNELS - 2) * (REPEAT_LATCH_GAP + 1)
} else {
    CHANNELS * TRANSFER_DCLKS
};
// Configuration leaves arbitrary bits in the shift registers. Clear them
// before the first latch. Every OUT0 row clears without a latch at row end.
pub const DATA_PREFIX_DCLKS: usize = if SHORT_DATA { TRANSFER_DCLKS } else { 0 };
// Every OUT0 row has 15 zero latches and an unlatched zero transfer
// after its gray. Other rows have 14 zero repeats and a latched zero transfer.
pub const LATCH_EXTRA_DCLKS: usize = {
    let mut clocks = 0;
    let mut row = 0;
    while SHORT_DATA && row < ROWS {
        if latch_target(row) == 0 {
            clocks += REPEAT_LATCH_GAP + 1;
        }
        row += 1;
    }
    clocks
};
// Full writes retain the historical one-clock inter-row separation. In
// continuous latch mode it would create a second consecutive LAT-low clock
// before the next repeated latch, so it is deliberately absent.
pub const DATA_INTER_ROW_DCLKS: usize = if SHORT_DATA { 0 } else { ROWS - 1 };
pub const DATA_PAYLOAD_END: usize =
    DATA_START
        + DATA_PREFIX_DCLKS + ROWS * DATA_ROW_DCLKS + DATA_INTER_ROW_DCLKS
        + LATCH_EXTRA_DCLKS;
// Preserve the previous even-clock head boundary and PARK padding.
pub const DATA_END: usize = (DATA_PAYLOAD_END + 1) & !1;
// Scan the displayed frame while loading the next image. Begin only after
// configuration, so skipping its DMA segment shifts both streams equally.
pub const FIRST_ROW: usize = CONFIG_END_WORDS + 16;
pub const SCAN_DCLKS: usize = if KEEP_SCAN_TIME {
    DATA_END - FIRST_ROW
} else {
    GROUPS * ROWS * BASE_ROW_DCLKS
};
/// Minimum row period; kept-time scans distribute the remainder one clock at a time.
pub const ROW_DCLKS: usize = {
    let period = SCAN_DCLKS / (GROUPS * ROWS);
    // Preserve the initial 12-DCLK ROW high and at least two low clocks.
    // This also leaves room for the 8-DCLK BK and 2-DCLK DIN/LCK timing.
    assert!(period >= 14,
        "keep_scan_time: too many scan_repeats for the gray-loading window; reduce scan_repeats (minimum row period is 14 DCLK)");
    period
};
pub const SCAN_END: usize = FIRST_ROW + SCAN_DCLKS;
// VSYNC waits for both streams. Retain a PARK tail for DMA bank completion
// even when a long data transfer outlasts the scan.
pub const FRAME_DCLKS: usize = (
    if DATA_END + 16 > SCAN_END { DATA_END + 16 } else { SCAN_END }
    + 1
) & !1;
pub const FRAME_WORDS: usize = FRAME_DCLKS;
pub const NORMAL_FRAME_DCLKS: usize = FRAME_DCLKS - CONFIG_SKIP_DCLKS;
#[cfg(not(target_os = "none"))]
pub const HEAD_WORDS: usize = DATA_END;
#[cfg(not(target_os = "none"))]
pub const TAIL_WORDS: usize = FRAME_WORDS - HEAD_WORDS;
#[cfg(not(target_os = "none"))]
pub const ROW_RUN_COUNT: usize = GROUPS * ROWS * 2 + 1;
#[cfg(not(target_os = "none"))]
pub const CONFIG_ROW_RUN_COUNT: usize = if ROW_ON_CFG { 0 } else { ROW_RUN_COUNT };
#[cfg(not(target_os = "none"))]
pub const VSYNC_FALL: usize = 8;
pub const fn row_start(index: usize) -> usize {
    FIRST_ROW + index * SCAN_DCLKS / (GROUPS * ROWS)
}

pub const fn row_period(index: usize) -> usize {
    // Refer to ROW_DCLKS so the pulse-width bound is checked for host tools too.
    assert!(ROW_DCLKS >= 14);
    row_start(index + 1) - row_start(index)
}

/// Full-write sweep is uniform across all pixels; latch mode keeps its sweep.
#[cfg(not(target_os = "none"))]
pub const GRAY_SWEEP: &[u16] = if CONTINUOUS_LATCH { &[
    0, 31, 63, 95, 127, 159, 191, 223, 255,
] } else { &[
    0, 15, 31, 63, 127, 255, 511, 1023, 2047, 4095, 8191,
] };

/// Patch only the gray-data SIN bits in a buffer not currently used by DMA.
#[cfg(not(target_os = "none"))]
pub fn update_white(words: &mut [u32], gray: u16) {
    assert!(gray <= MAX_GRAY_INPUT);
    if SHORT_DATA {
        let mut row_start = DATA_START + DATA_PREFIX_DCLKS;
        for row in 0..ROWS {
            let target = latch_target(row);
            let before_target = (CHANNELS - 1 - target) * (REPEAT_LATCH_GAP + 1);
            let start = row_start + before_target;
            for t in start..start + latch_gray_dclks(row) {
                let offset = t - start;
                let mask = SIN;
                words[t] = (words[t] & !mask)
                    | if offset < 8 && gray & (1 << (7 - offset)) != 0 { GRAY_SIN } else { 0 };
            }
            row_start += DATA_ROW_DCLKS
                + if target == 0 { REPEAT_LATCH_GAP + 1 } else { 0 };
        }
        assert_eq!(row_start, DATA_PAYLOAD_END);
        return;
    }
    let mut start = DATA_START;
    for row in 0..ROWS {
        for transfer in 0..CHANNELS {
            start += usize::from(row != 0 && transfer == 0);
            let value = gray;
            for t in start..start + TRANSFER_DCLKS {
                let bit = 15 - (t - start) % 16;
                let mask = SIN;
                words[t] = (words[t] & !mask)
                    | if value & (1 << bit) != 0 { GRAY_SIN } else { 0 };
            }
            start += TRANSFER_DCLKS;
        }
    }
    assert_eq!(start, DATA_PAYLOAD_END);
}

pub struct Waveform<'a> {
    words: &'a mut [u32],
    tail: &'a mut [u32],
    clocks: usize,
}
impl<'a> Waveform<'a> {
    pub fn new(words: &'a mut [u32]) -> Self {
        Self { words, tail: &mut [], clocks: 0 }
    }
    #[cfg(not(target_os = "none"))]
    pub fn split(words: &'a mut [u32], tail: &'a mut [u32]) -> Self {
        Self { words, tail, clocks: 0 }
    }
    fn word_mut(&mut self, index: usize) -> &mut u32 {
        if index < self.words.len() {
            &mut self.words[index]
        } else {
            &mut self.tail[index - self.words.len()]
        }
    }
    pub fn sample(&mut self, pins: u32) {
        let clock = self.clocks;
        *self.word_mut(clock) = pins;
        self.clocks += 1;
    }
    pub fn hold(&mut self, pins: u32, clocks: usize) {
        for _ in 0..clocks {
            self.sample(pins);
        }
    }
    fn command_at(&mut self, start: usize, width: usize) {
        assert!(start >= self.clocks);
        self.hold(PARK, start - self.clocks);
        self.hold(PARK | LE, width);
        // The following hold/transfer emits the falling edge. It is not an
        // extra DCLK appended to the command's high interval.
    }
    fn shift_config(&mut self, value: u16, gap: usize) {
        self.shift_config_rgb([value; 3], gap);
    }
    // Requires a zeroed shift chain before every gray transfer.
    fn shift_latch_gray(&mut self, value: u16, row: usize) {
        assert!(value <= 255, "continuous latch gray must fit in 8 bits");
        let clocks = latch_gray_dclks(row);
        for t in 0..clocks {
            let data = if t < 8 && value & (1 << (7 - t)) != 0 { GRAY_SIN } else { 0 };
            let latch = if t + 1 == clocks { LE } else { 0 };
            self.sample(PARK | data | latch);
        }
    }
    fn clear_latch_gray(&mut self, row: usize, latch: bool) {
        self.hold(PARK, latch_clear_dclks(row) - 1);
        self.sample(PARK | if latch { LE } else { 0 });
    }
    fn shift_pixels(&mut self, values: &[u16; TOTAL_CHANNELS], out: usize, gap: usize) {
        self.hold(PARK, gap);
        // Farthest chip first; the last serial word remains nearest SIN.
        for chip in (0..CHAIN_COPIES).rev() {
            let value = if out < LED_CHANNELS { values[chip * LED_CHANNELS + out] } else { 0 };
            assert!(value <= MAX_GRAY_INPUT);
            for bit in (0..16).rev() {
                let data = if value & (1 << bit) != 0 { GRAY_SIN } else { 0 };
                self.sample(PARK | data | if chip == 0 && bit == 0 { LE } else { 0 });
            }
        }
    }
    fn shift_config_rgb(&mut self, values: [u16; 3], gap: usize) {
        self.hold(PARK, gap + CFG_PRE_CLR_DCLKS);
        for copy in 0..CFG_CHAIN_COPIES {
            for bit in (0..16).rev() {
                let data = rgb_bit(values, bit);
                let latch = if copy == CFG_CHAIN_COPIES - 1 && bit < WR_CFG {
                    LE
                } else {
                    0
                };
                self.sample(PARK | data | latch);
            }
        }
    }
}
pub type Frame = [[u16; TOTAL_CHANNELS]; ROWS];

/// Keep the moving diagonal or the fixed selected channel for latch mode.
#[cfg(not(target_os = "none"))]
pub fn target_frame(mut frame: Frame) -> Frame {
    for (row, values) in frame.iter_mut().enumerate() {
        for (out, value) in values.iter_mut().enumerate() {
            if out != latch_pixel(row) {
                *value = 0;
            }
        }
    }
    frame
}

/// Uniform gray throughout a frame; advance one gray level per frame.
#[cfg(not(target_os = "none"))]
pub fn sweep_frame(frame_index: usize) -> Frame {
    [[GRAY_SWEEP[frame_index % GRAY_SWEEP.len()]; TOTAL_CHANNELS]; ROWS]
}

#[derive(Clone, Copy, Debug)]
// The build selects one variant, while the encoder supports every pattern.
#[allow(dead_code)]
pub enum Pattern {
    Diagonal,
    Column(usize),
    Normal,
}

/// Logical row/channel masks; only SIN data changes, never scan timing.
#[cfg(not(target_os = "none"))]
pub fn pattern_frame(pattern: Pattern, gray: u16) -> Frame {
    pattern_frame_range(pattern, [gray; 2])
}

/// Normal rotates the full chain-width gradient right by one column per row.
/// Diagonal keeps column (row % TOTAL_CHANNELS), fading gray[1] to gray[0].
/// Column fades gray[1] to gray[0] from the first to the last row.
/// Column = chip * LED_CHANNELS + OUT; chip 0 is nearest the MCU.
/// Integer interpolation rounds to the nearest gray and preserves endpoints.
pub fn pattern_frame_range(pattern: Pattern, gray: [u16; 2]) -> Frame {
    assert!(gray.iter().all(|&value| value <= MAX_GRAY_INPUT));
    if let Pattern::Column(column) = pattern {
        assert!(column < TOTAL_CHANNELS);
    }
    let mut frame = [[0; TOTAL_CHANNELS]; ROWS];
    for (row, values) in frame.iter_mut().enumerate() {
        for (out, value) in values.iter_mut().enumerate() {
            let (position, last) = match pattern {
                Pattern::Diagonal => {
                    // One nonzero-capable channel per row permits zero repeats.
                    let diagonal_row = row % TOTAL_CHANNELS;
                    if out != diagonal_row {
                        continue;
                    }
                    (TOTAL_CHANNELS - 1 - diagonal_row, TOTAL_CHANNELS - 1)
                }
                Pattern::Column(column) => {
                    if out != column {
                        continue;
                    }
                    // A single row is the first endpoint of the column.
                    if ROWS == 1 {
                        *value = gray[1];
                        continue;
                    }
                    (ROWS - 1 - row, ROWS - 1)
                }
                Pattern::Normal => (
                    (out + TOTAL_CHANNELS - row % TOTAL_CHANNELS) % TOTAL_CHANNELS,
                    TOTAL_CHANNELS - 1,
                ),
            };
            // Weighted endpoints also support descending ranges without
            // unsigned subtraction. Serial transfer still runs OUT15..OUT0.
            let position = position as u32;
            let last = last as u32;
            *value = ((u32::from(gray[0]) * (last - position)
                + u32::from(gray[1]) * position + last / 2) / last) as u16;
        }
    }
    frame
}

pub fn encode_frame(words: &mut [u32], frame: &Frame) -> usize {
    encode_with_config(words, frame, None)
}

pub fn encode_with_config(words: &mut [u32], frame: &Frame, rotating: Option<[u16; 3]>) -> usize {
    assert_eq!(words.len(), FRAME_WORDS);
    let mut wave = Waveform::new(words);
    encode_wave(&mut wave, frame, rotating)
}

/// Each gray has its own input head; all frames share an identical scan tail.
/// Both slices must be initialized before DMA starts and then stay immutable.
#[cfg(not(target_os = "none"))]
pub fn encode_split(head: &mut [u32], tail: &mut [u32], frame: &Frame) {
    assert_eq!(DATA_END % 2, 0);
    assert_eq!(head.len(), HEAD_WORDS);
    assert_eq!(tail.len(), TAIL_WORDS);
    encode_wave(&mut Waveform::split(head, tail), frame, None);
}

fn encode_wave(wave: &mut Waveform<'_>, frame: &Frame, rotating: Option<[u16; 3]>) -> usize {
    wave.command_at(5, VSYNC);
    wave.hold(PARK, COMMON_PREFIX_WORDS - wave.clocks);
    // LCK shifts the RT5990 chain; ICND2069 ROW stays low throughout.
    // Blank while flushing old selections, with DIN, SIN and LE held low.
    let clear = blank(true) | select_bit(false);
    wave.hold(clear, 2);
    for _ in 0..RT5990_CLEAR_ROWS {
        wave.hold(clear | LCK, 2);
        wave.hold(clear, 2);
    }
    wave.hold(clear, 2);
    // Both clearing and EN_OP belong to the skipped configuration segment.
    // Enable before PRE_ACT so it cannot cancel configuration access.
    wave.command_at(CONFIG_COMMAND_START, EN_OP);
    wave.command_at(CONFIG_COMMAND_START + 14, PRE_ACT);
    wave.shift_config(0x00aa, 9);
    wave.shift_config(0x01aa, 8);
    if let Some(value) = rotating {
        wave.shift_config_rgb(value, 8);
    } else {
        for &(address, value) in REGISTERS {
            wave.shift_config_rgb(config_words(address, value), 8);
        }
    }
    wave.shift_config(0x0055, if rotating.is_some() { 9 } else { 8 });
    wave.shift_config(0x0155, 8);
    wave.hold(PARK, CFG_POST_CLR_DCLKS);
    if SHORT_DATA {
        wave.hold(PARK, TRANSFER_DCLKS);
    }
    for (row, values) in frame.iter().enumerate() {
        if SHORT_DATA {
            let target = latch_target(row);
            for (pixel, &value) in values.iter().enumerate() {
                assert!(pixel == latch_pixel(row) || value == 0);
            }
            for out in (0..CHANNELS).rev() {
                if out == target {
                    wave.shift_latch_gray(values[latch_pixel(row)], row);
                } else if out + 1 == target {
                    wave.clear_latch_gray(row, true);
                } else {
                    // SIN stays zero, so the cleared register remains zero.
                    wave.hold(PARK, REPEAT_LATCH_GAP);
                    wave.hold(PARK | LE, 1);
                }
            }
            if target == 0 {
                // No channel remains in this row: clear without a latch,
                // so the next row starts with zero in the shift register.
                wave.clear_latch_gray(row, false);
            }
        } else {
            for out in (0..CHANNELS).rev() {
                // Reference has an extra DCLK between logical data rows.
                let gap = usize::from(row != 0 && out == CHANNELS - 1);
                wave.shift_pixels(values, out, gap);
            }
        }
    }
    if rotating.is_none() {
        assert_eq!(wave.clocks, DATA_PAYLOAD_END);
        wave.hold(PARK, DATA_END - DATA_PAYLOAD_END);
        assert_eq!(wave.clocks, DATA_END);
    }
    wave.hold(PARK, FRAME_DCLKS - wave.clocks);

    // Overlay independently running ROW on the serialized command/data bus.
    // GROUPS complete sweeps x ROWS rows; distribute fractional periods in DCLK.
    let mut start = FIRST_ROW;
    for i in 0..GROUPS * ROWS {
        let period = row_period(i);
        for t in start - 8..start + period - 8 {
            let mut row_pins = blank(t < start);
            row_pins |= select_bit(i % ROWS == 0 && t < start + 2);
            if (start - 4..start - 2).contains(&t) {
                row_pins |= LCK;
            }
            let width = if i == 0 { 12 } else { 4 };
            if (start..start + width).contains(&t) {
                row_pins |= ROW;
            }
            let mask = BK | DIN | LCK | ROW;
            let word = wave.word_mut(t);
            *word = (*word & !mask) | row_pins;
        }
        start += period;
    }
    FRAME_WORDS
}

#[cfg(not(target_os = "none"))]
pub fn encode_row_runs(words: &mut [u32; ROW_RUN_COUNT]) {
    // SM1 counts the same DCLK IRQ events as SM0. Edge at clock t follows
    // its IRQ, so the corresponding run boundary is t+1. Derive the pulse
    // count from ROWS rather than replaying the old fixed 40-scan binary.
    let mut position = 0;
    let mut start = FIRST_ROW;
    for i in 0..GROUPS * ROWS {
        let rise = start + 1;
        let width = if i == 0 { 12 } else { 4 };
        let low_count = rise - position;
        assert!(low_count > 0);
        words[2 * i] = ((low_count - 1) as u32) << 1;
        words[2 * i + 1] = (((width - 1) as u32) << 1) | 1;
        position = rise + width;
        start += row_period(i);
    }
    assert!(position < FRAME_DCLKS);
    // Finish the last row window, then immediately start the next frame.
    words[ROW_RUN_COUNT - 1] = ((FRAME_DCLKS - position - 1) as u32) << 1;
}

/// Keep every run duration and DMA/FIFO cadence, but hold ROW low all frame.
#[cfg(not(target_os = "none"))]
pub fn encode_config_row_runs(words: &mut [u32], normal: &[u32; ROW_RUN_COUNT]) {
    assert_eq!(words.len(), CONFIG_ROW_RUN_COUNT);
    for (word, &run) in words.iter_mut().zip(normal) {
        *word = run & !1;
    }
}

#[cfg(not(target_os = "none"))]
pub fn rotation_word(index: usize) -> [u16; 3] {
    match index % (REGISTERS.len() + 2) {
        0 => [0x0055; 3],
        1 => [0x0155; 3],
        i => {
            let (a, v) = REGISTERS[i - 2];
            config_words(a, v)
        }
    }
}

#[cfg(not(target_os = "none"))]
pub fn startup_reset(words: &mut [u32]) {
    // Diagnostic only: extend the first LE pulse from 3 to 9 rising edges.
    for t in 8..14 {
        words[t] |= LE;
    }
}

#[cfg(not(target_os = "none"))]
pub fn update_rotation(words: &mut [u32], value: [u16; 3]) {
    // First configuration word follows EN_OP, PRE_ACT and their gaps (37 clocks).
    // Derive the third word's position for the configured chain length.
    // Only SIN changes. Preserve LE and the overlapping ROW controls.
    let start = CONFIG_COMMAND_START + 37 + CFG_PRE_CLR_DCLKS + 2 * (CFG_WRITE_DCLKS + 8);
    for t in start..start + CFG_TRANSFER_DCLKS {
        let bit = 15 - (t - start) % 16;
        let mask = SIN;
        words[t] = (words[t] & !mask) | rgb_bit(value, bit);
    }
}
