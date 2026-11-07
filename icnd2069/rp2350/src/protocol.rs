//! ICND2069 54-scan image and full configuration using fixed DCLK timing.
//! Each u32 describes pins for one DCLK rising-edge transfer.
//! Reference: captures/reference/nova_to_devboard_100m_2s.dsl and the startup summary.

/// ICND2069 register 02h is programmed as 53, selecting 54 scan rows.
pub const ROWS: usize = 54;
pub const CHANNELS: usize = 16;
/// Number of complete 54-row sweeps per frame; build-time variable, default 1.
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
        b"30" => 30_000_000,
        b"31" => 31_000_000,
        b"32" => 32_000_000,
        b"33" => 33_000_000,
        b"34" => 34_000_000,
        b"35" => 35_000_000,
        b"40" => 40_000_000,
        b"45" => 45_000_000,
        _ => panic!("ICND2069_DCLK_MHZ must be 1, 3, 6, 10, 12.5, 20, 25, 30, 31, 32, 33, 34, 35, 40 or 45"),
    },
    None => 12_500_000,
};
pub const PIO_CYCLES_PER_DCLK: u32 = if DCLK_HZ >= 30_000_000 { 4 } else { 6 };
pub const SYS_HZ: u32 = match DCLK_HZ {
    // Every supported preset >=30 MHz uses divider 1, four cycles/DCLK.
    30_000_000..=45_000_000 => DCLK_HZ * 4,
    10_000_000 | 20_000_000 => 120_000_000,
    // Six PIO cycles/DCLK: integer dividers 8 and 4 respectively.
    3_000_000 | 6_000_000 => 144_000_000,
    _ => 150_000_000,
};
// Pico 2's 12 MHz crystal feeds the integer system PLL (refdiv=1).
// 124 MHz cannot use /3/2: its VCO would be 744 MHz, below 750 MHz.
// Use 1488 MHz /4/3 instead. Keep the standard 1500 MHz /5/2 at 150 MHz.
pub const SYS_PLL_FBDIV: u16 = match SYS_HZ {
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
pub const SYS_PLL_POST_DIV2: u8 = if SYS_HZ == 124_000_000 { 3 } else { 2 };
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
// Keep the 4-DCLK ROW high pulse and RT5990 control strobes unchanged.
// At 54 scan rows, 60 DCLK/row makes the complete row-control segment
// finish alongside the normal-frame gray-data transfer.
pub const ROW_DCLKS: usize = 60;
// Programming Guide V4.2 p.13-14: with 8 < DCLK < 16 MHz, the highest
// listed valid PLL ratio is P=12, M=2, N=1, so R=6 and GCLK=75 MHz at
// the default 12.5 MHz DCLK. At 6 MHz use the 4<DCLK<8 table's
// M=1/P=10 setting. Retain N's existing encoding. 1/3 MHz use the same
// low-range registers for timing experiments, outside the guide's PLL
// input constraints; valid LED output is not guaranteed at those clocks.
pub const PLL_M: u8 = match DCLK_HZ {
    1_000_000 | 3_000_000 | 6_000_000 => 0x01,
    10_000_000 | 12_500_000 => 0x02,
    _ => 0x04,
};
pub const PLL_P: u8 = if matches!(DCLK_HZ, 10_000_000 | 12_500_000) { 0x0c } else { 0x0a };
pub const PLL_N: u8 = 0x01;
// Software input limit retained from the saved NOVA image. This is NOT a
// calibrated PWM full scale. The previous 40-scan capture saturated by 512;
// the new 16-scan/full-config mode must be measured independently.
pub const MAX_GRAY_INPUT: u16 = 8192;
pub const DEMO_GRAY: u16 = 8192;
/// One identical word per cascaded ICND2069; build-time variable, default 1.
pub const CHAIN_COPIES: usize = match option_env!("ICND2069_CHAIN_COPIES") {
    Some(value) => {
        let bytes = value.as_bytes();
        assert!(bytes.len() == 1 && bytes[0] >= b'1' && bytes[0] <= b'8');
        (bytes[0] - b'0') as usize
    }
    None => 1,
};
pub const TRANSFER_DCLKS: usize = 16 * CHAIN_COPIES;
/// Enable repeated latches of zero; disabled shifts all 16 channel words.
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
pub const fn latch_target(row: usize) -> usize {
    if DIAGONAL_LATCH { row % CHANNELS } else { TARGET_CHANNEL }
}
/// Physical OUT channel. Serial DATA_LATCH order remains OUT15 through OUT0.
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
        assert!(n < CHANNELS);
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

// Each sample maps directly to Pico 2 GP0..17. RGB1..4 occupy GP0..11.
// Configuration is color-specific; gray data still describes one image
// mirrored onto all 12 inputs, not independently addressable RGB planes.
pub const RGB_SIN: [u32; 3] = [0x0249, 0x0492, 0x0924];
pub const SIN: u32 = 0x0fff;
pub const LE: u32 = 1 << 13;
pub const LCK: u32 = 1 << 15;
pub const BK: u32 = 1 << 16;
pub const DIN: u32 = 1 << 17;
// GP12/14 are zero here: side-set owns DCLK, and sticky SM1 output owns ROW.
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
pub const ENABLE: usize = 11;
pub const PRE_ACT: usize = 14;

// R/G/B values from dsview/icnd2069/ICND2069-抓包寄存器汇总.md (B/C).
// Keep the current row/group/PLL settings instead of the reference scan mode.
// 00/01 are access controls, sent outside this list to keep the whole batch
// unlocked. The reference rotation includes them but they are not payload.
pub const REGISTERS: &[(u8, [u8; 3])] = &[
    (0x02, [(ROWS - 1) as u8; 3]),
    (0x03, [(GROUPS - 1) as u8; 3]),
    (0x04, [PLL_M; 3]),
    (0x05, [PLL_P; 3]),
    (0x06, [PLL_N; 3]),
    (0x07, [0x20; 3]),
    (0x08, [0x20; 3]),
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
pub const CONFIG_END_WORDS: usize = (49 + CONFIG_WRITES * (8 + TRANSFER_DCLKS) + 1) & !1;
pub const CONFIG_SKIP_DCLKS: usize = CONFIG_END_WORDS - COMMON_PREFIX_WORDS;
pub const DATA_ROW_DCLKS: usize = if SHORT_DATA {
    2 * TRANSFER_DCLKS + (CHANNELS - 2) * (REPEAT_LATCH_GAP + 1)
} else {
    CHANNELS * TRANSFER_DCLKS
};
// Configuration leaves arbitrary bits in the shift registers. Clear them
// before the first latch. OUT0 also needs an unlatched clear at frame end.
pub const DATA_PREFIX_DCLKS: usize = if SHORT_DATA { TRANSFER_DCLKS } else { 0 };
pub const DATA_SUFFIX_DCLKS: usize =
    if SHORT_DATA && !DIAGONAL_LATCH && TARGET_CHANNEL == 0 { TRANSFER_DCLKS } else { 0 };
// Every diagonal OUT0 has 15 zero latches and an unlatched zero transfer
// after its gray. Other rows have 14 zero repeats and a latched zero transfer.
pub const DIAGONAL_EXTRA_DCLKS: usize =
    if SHORT_DATA && DIAGONAL_LATCH {
        ((ROWS + CHANNELS - 1) / CHANNELS) * (REPEAT_LATCH_GAP + 1)
    } else {
        0
    };
// Full writes retain the historical one-clock inter-row separation. In
// continuous latch mode it would create a second consecutive LAT-low clock
// before the next repeated latch, so it is deliberately absent.
pub const DATA_INTER_ROW_DCLKS: usize = if SHORT_DATA { 0 } else { ROWS - 1 };
pub const DATA_PAYLOAD_END: usize =
    49 + CONFIG_WRITES * (8 + TRANSFER_DCLKS)
        + DATA_PREFIX_DCLKS + ROWS * DATA_ROW_DCLKS + DATA_INTER_ROW_DCLKS
        + DATA_SUFFIX_DCLKS + DIAGONAL_EXTRA_DCLKS;
// Preserve the previous even-clock head boundary and PARK padding.
pub const DATA_END: usize = (DATA_PAYLOAD_END + 1) & !1;
// Scan the displayed frame while loading the next image. Begin only after
// configuration, so skipping its DMA segment shifts both streams equally.
pub const FIRST_ROW: usize = CONFIG_END_WORDS + 16;
pub const SCAN_END: usize = FIRST_ROW + scan_period();
// VSYNC waits for both streams. Retain a PARK tail for DMA bank completion
// even when a long data transfer outlasts the scan.
pub const FRAME_DCLKS: usize = (
    if DATA_END + 16 > SCAN_END { DATA_END + 16 } else { SCAN_END }
    + 1
) & !1;
pub const FRAME_WORDS: usize = FRAME_DCLKS;
pub const NORMAL_FRAME_DCLKS: usize = FRAME_DCLKS - CONFIG_SKIP_DCLKS;
pub const HEAD_WORDS: usize = DATA_END;
pub const TAIL_WORDS: usize = FRAME_WORDS - HEAD_WORDS;
pub const ROW_RUN_COUNT: usize = GROUPS * ROWS * 2 + 1;
pub const VSYNC_FALL: usize = 8;
pub const fn row_period(_index: usize) -> usize {
    ROW_DCLKS
}

const fn scan_period() -> usize {
    GROUPS * ROWS * ROW_DCLKS
}

/// Full-write sweep is uniform across all pixels; latch mode keeps its sweep.
pub const GRAY_SWEEP: &[u16] = if CONTINUOUS_LATCH { &[
    0, 511, 1608, 2705, 3802, 4900, 5997, 7094, 8191,
] } else { &[
    0, 15, 31, 63, 127, 255, 511, 1023, 2047, 4095, 8191,
] };

/// Patch only the gray-data SIN bits in a buffer not currently used by DMA.
pub fn update_white(words: &mut [u32], gray: u16) {
    assert!(gray <= MAX_GRAY_INPUT);
    if SHORT_DATA {
        let mut row_start = 49 + CONFIG_WRITES * (8 + TRANSFER_DCLKS)
            + DATA_PREFIX_DCLKS;
        for row in 0..ROWS {
            let target = latch_target(row);
            let before_target = if !DIAGONAL_LATCH && target == 0 {
                TRANSFER_DCLKS + (CHANNELS - 2) * (REPEAT_LATCH_GAP + 1)
            } else {
                (CHANNELS - 1 - target) * (REPEAT_LATCH_GAP + 1)
            };
            let start = row_start + before_target;
            for t in start..start + TRANSFER_DCLKS {
                let bit = 15 - (t - start) % 16;
                let mask = SIN;
                words[t] = (words[t] & !mask)
                    | if gray & (1 << bit) != 0 { mask } else { 0 };
            }
            row_start += DATA_ROW_DCLKS
                + if DIAGONAL_LATCH && target == 0 { REPEAT_LATCH_GAP + 1 } else { 0 };
        }
        assert_eq!(row_start + DATA_SUFFIX_DCLKS, DATA_PAYLOAD_END);
        return;
    }
    let mut start = 49 + CONFIG_WRITES * (8 + TRANSFER_DCLKS);
    for row in 0..ROWS {
        for transfer in 0..CHANNELS {
            start += usize::from(row != 0 && transfer == 0);
            for t in start..start + TRANSFER_DCLKS {
                let bit = 15 - (t - start) % 16;
                let mask = SIN;
                words[t] = (words[t] & !mask)
                    | if gray & (1 << bit) != 0 { mask } else { 0 };
            }
            start += TRANSFER_DCLKS;
        }
    }
    assert_eq!(start, DATA_END);
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
    fn shift(&mut self, value: u16, le_clocks: usize, gap: usize) {
        self.shift_rgb([value; 3], le_clocks, gap);
    }
    fn shift_rgb(&mut self, values: [u16; 3], le_clocks: usize, gap: usize) {
        self.hold(PARK, gap);
        for copy in 0..CHAIN_COPIES {
            for bit in (0..16).rev() {
                let data = rgb_bit(values, bit);
                let latch = if copy == CHAIN_COPIES - 1 && bit < le_clocks {
                    LE
                } else {
                    0
                };
                self.sample(PARK | data | latch);
            }
        }
    }
}
pub type Frame = [[u16; CHANNELS]; ROWS];

/// Keep the moving diagonal or the fixed selected channel for latch mode.
pub fn target_frame(mut frame: Frame) -> Frame {
    for (row, values) in frame.iter_mut().enumerate() {
        for (out, value) in values.iter_mut().enumerate() {
            if out != latch_target(row) {
                *value = 0;
            }
        }
    }
    frame
}

/// Uniform gray throughout a frame; advance one gray level per frame.
pub fn sweep_frame(frame_index: usize) -> Frame {
    [[GRAY_SWEEP[frame_index % GRAY_SWEEP.len()]; CHANNELS]; ROWS]
}

#[derive(Clone, Copy, Debug)]
pub enum Pattern {
    Diagonal,
    Normal,
}

/// Logical row/channel masks; only SIN data changes, never scan timing.
pub fn pattern_frame(pattern: Pattern, gray: u16) -> Frame {
    pattern_frame_range(pattern, [gray; 2])
}

/// Normal rotates the OUT0..OUT15 gradient right by one channel per row.
/// Diagonal keeps only OUT(row % 16), fading from gray[1] to gray[0].
/// The pattern repeats every 16 rows and on each cascaded chip.
/// Integer interpolation rounds to the nearest gray and preserves endpoints.
pub fn pattern_frame_range(pattern: Pattern, gray: [u16; 2]) -> Frame {
    assert!(gray.iter().all(|&value| value <= MAX_GRAY_INPUT));
    let mut frame = [[0; CHANNELS]; ROWS];
    for (row, values) in frame.iter_mut().enumerate() {
        for (out, value) in values.iter_mut().enumerate() {
            let position = if matches!(pattern, Pattern::Diagonal) {
                // One nonzero-capable channel per row permits zero repeats.
                // Top-level gray controls the diagonal as well as full pixels.
                let diagonal_row = row % CHANNELS;
                if out != diagonal_row {
                    continue;
                }
                CHANNELS - 1 - diagonal_row
            } else {
                (out + CHANNELS - row % CHANNELS) % CHANNELS
            } as u32;
            // Weighted endpoints also support descending ranges without
            // unsigned subtraction. Serial transfer still runs OUT15..OUT0.
            let last = (CHANNELS - 1) as u32;
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
pub fn encode_split(head: &mut [u32], tail: &mut [u32], frame: &Frame) {
    assert_eq!(DATA_END % 2, 0);
    assert_eq!(head.len(), HEAD_WORDS);
    assert_eq!(tail.len(), TAIL_WORDS);
    encode_wave(&mut Waveform::split(head, tail), frame, None);
}

fn encode_wave(wave: &mut Waveform<'_>, frame: &Frame, rotating: Option<[u16; 3]>) -> usize {
    // Exact rising-edge coordinates from the reference: falls 8, 31, 48.
    wave.command_at(5, VSYNC);
    wave.command_at(20, ENABLE);
    wave.command_at(34, PRE_ACT);
    wave.shift(0x00aa, WR_CFG, 9); // first WR_CFG falls at 57+TRANSFER_DCLKS.
    wave.shift(0x01aa, WR_CFG, 8);
    if let Some(value) = rotating {
        wave.shift_rgb(value, WR_CFG, 8);
    } else {
        for &(address, value) in REGISTERS {
            wave.shift_rgb(config_words(address, value), WR_CFG, 8);
        }
    }
    wave.shift(0x0055, WR_CFG, if rotating.is_some() { 9 } else { 8 });
    wave.shift(0x0155, WR_CFG, 8);
    if SHORT_DATA {
        wave.shift(0, 0, 0);
    }
    for (row, values) in frame.iter().enumerate() {
        if SHORT_DATA {
            let target = latch_target(row);
            for out in (0..CHANNELS).rev() {
                assert!(out == target || values[out] == 0);
                if out == target {
                    wave.shift(values[out], 1, 0);
                } else if out + 1 == target
                    || (!DIAGONAL_LATCH && target == 0 && out == CHANNELS - 1)
                {
                    // OUT0's next serial slot is next row's OUT15.
                    wave.shift(0, 1, 0);
                } else {
                    // SIN stays zero, so the cleared register remains zero.
                    wave.hold(PARK, REPEAT_LATCH_GAP);
                    wave.hold(PARK | LE, 1);
                }
            }
            if DIAGONAL_LATCH && target == 0 {
                // No channel remains in this row: clear without a latch,
                // so the next row starts with zero in the shift register.
                wave.shift(0, 0, 0);
            }
        } else {
            for out in (0..CHANNELS).rev() {
                // Reference has an extra DCLK between logical data rows.
                let gap = usize::from(row != 0 && out == CHANNELS - 1);
                wave.shift(values[out], 1, gap);
            }
        }
    }
    if DATA_SUFFIX_DCLKS != 0 {
        // Clear the last row's target without consuming a 257th pixel.
        wave.shift(0, 0, 0);
    }
    if rotating.is_none() {
        assert_eq!(wave.clocks, DATA_PAYLOAD_END);
        wave.hold(PARK, DATA_END - DATA_PAYLOAD_END);
        assert_eq!(wave.clocks, DATA_END);
    }
    wave.hold(PARK, FRAME_DCLKS - wave.clocks);

    // Overlay independently running ROW on the serialized command/data bus.
    // GROUPS complete sweeps x 54 rows with a fixed ROW_DCLKS cadence.
    let mut start = FIRST_ROW;
    for i in 0..GROUPS * ROWS {
        let period = row_period(i);
        for t in start - 8..start + period - 8 {
            let mut row_pins = blank(t < start);
            row_pins |= select_bit(i % ROWS == 0 && t < start + 2);
            if (start - 4..start - 2).contains(&t) {
                row_pins |= LCK;
            }
            // ROW is generated separately by SM1, not in SM0's pin word.
            let mask = BK | DIN | LCK;
            let word = wave.word_mut(t);
            *word = (*word & !mask) | row_pins;
        }
        start += period;
    }
    FRAME_WORDS
}

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

pub fn startup_reset(words: &mut [u32]) {
    // Diagnostic only: extend the first LE pulse from 3 to 9 rising edges.
    for t in 8..14 {
        words[t] |= LE;
    }
}

pub fn update_rotation(words: &mut [u32], value: [u16; 3]) {
    // First transfer starts at 57, with an 8-DCLK gap between transfers.
    // Derive the third word's position for the configured chain length.
    // Only SIN changes. Preserve LE and the overlapping ROW controls.
    let start = 57 + 2 * (TRANSFER_DCLKS + 8);
    for t in start..start + TRANSFER_DCLKS {
        let bit = 15 - (t - start) % 16;
        let mask = SIN;
        words[t] = (words[t] & !mask) | rgb_bit(value, bit);
    }
}
