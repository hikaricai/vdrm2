//! ICND2069 16-scan image and full configuration using fixed DCLK timing.
//! Each halfword describes pins for one DCLK rising-edge transfer.
//! Reference: dbg/nova_to_devboard_100m_2s.dsl and the startup summary.

pub const ROWS: usize = 16;
pub const CHANNELS: usize = 16;
/// Number of complete 16-row sweeps per frame; build-time variable, default 1.
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
        b"12.5" => 12_500_000,
        b"25" => 25_000_000,
        b"35" => 35_000_000,
        _ => panic!("ICND2069_DCLK_MHZ must be 12.5, 25 or 35"),
    },
    None => 35_000_000,
};
pub const PIO_CYCLES_PER_DCLK: u32 = if DCLK_HZ == 35_000_000 { 4 } else { 6 };
pub const SYS_HZ: u32 = if DCLK_HZ == 35_000_000 { 140_000_000 } else { 150_000_000 };
pub const ROW_DCLKS: usize = 81;
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
/// Send only the selected physical channel and its following zero word.
pub const SHORT_DATA: bool = match option_env!("ICND2069_SHORT_DATA") {
    Some(value) => {
        let bytes = value.as_bytes();
        assert!(bytes.len() == 1 && (bytes[0] == b'0' || bytes[0] == b'1'));
        bytes[0] == b'1'
    }
    None => true,
};
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

// Pin bits relative to GP10. GP15 is driven independently by side-set.
pub const LE: u16 = 1;
pub const SIN: u16 = 1 << 3;
pub const LCK: u16 = 1 << 6;
pub const BK: u16 = 1 << 8;
pub const DIN: u16 = 1 << 10;
pub const ROW: u16 = 1 << 12;
// The reference's six channels do not measure RT5990 DIN/LCK/BK.
// Preserve the existing board assumptions; do not claim measured parity.
pub const BK_ACTIVE_HIGH: bool = true;
pub const DIN_ONE_SELECTS: bool = true;
const fn blank(on: bool) -> u16 {
    if on == BK_ACTIVE_HIGH {
        BK
    } else {
        0
    }
}
const fn select_bit(selected: bool) -> u16 {
    if selected == DIN_ONE_SELECTS {
        DIN
    } else {
        0
    }
}
pub const PARK: u16 = blank(true) | select_bit(false);
pub const VSYNC: usize = 3;
pub const WR_CFG: usize = 5;
pub const ENABLE: usize = 11;
pub const PRE_ACT: usize = 14;

// Red/SIN values from all three 7-second NOVA reference captures.
// 00/01 are access controls, sent outside this list to keep the whole batch
// unlocked. The reference rotation includes them but they are not payload.
pub const REGISTERS: &[(u8, u8)] = &[
    (0x02, (ROWS - 1) as u8),
    (0x03, (GROUPS - 1) as u8),
    (0x04, 0x04),
    (0x05, 0x0a),
    (0x06, 0x01),
    (0x07, 0x20),
    (0x08, 0x20),
    (0x09, 0x08),
    (0x0a, 0x08),
    (0x0b, 0x00),
    (0x0c, 0x01),
    (0x0d, 0x01),
    (0x0e, 0x07),
    (0x0f, 0x01),
    (0x10, 0xc2),
    (0x11, 0x21),
    (0x12, 0x01),
    (0x13, 0x00),
    (0x14, 0x00),
    (0x15, 0x00),
    (0x16, 0x00),
    (0x17, 0xf0),
    (0x18, 0x1f),
    (0x19, 0x20),
    (0x1a, 0x1f),
    (0x1b, 0x10),
    (0x1c, 0x72),
    (0x1d, 0x82),
    (0x1e, 0x41),
    (0x1f, 0x04),
    (0x20, 0x48),
    (0x21, 0x01),
    (0x22, 0x1c),
    (0x70, 0x00),
    (0x71, 0x00),
    (0x72, 0x00),
    (0x73, 0x00),
    (0x74, 0x00),
    (0xf0, 0x00),
    (0xf1, 0x00),
    (0xf2, 0x00),
    (0xf3, 0x00),
    (0xf4, 0x00),
    (0xf5, 0x00),
    (0x23, 0x01),
];
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
// Keep VSYNC/EN_OP and their trailing PARK clock in a 17-word prefix.
// Skip the full configuration on even word boundaries. The final skipped
// clock is PARK at the beginning of the data gap, replaced by PARK at 33.
pub const COMMON_PREFIX_WORDS: usize = 17;
pub const CONFIG_END_WORDS: usize = (49 + CONFIG_WRITES * (8 + TRANSFER_DCLKS) + 1) / 2;
pub const CONFIG_SKIP_DCLKS: usize = 2 * (CONFIG_END_WORDS - COMMON_PREFIX_WORDS);
pub const DATA_ROW_DCLKS: usize = if SHORT_DATA {
    2 * (8 + TRANSFER_DCLKS) + (CHANNELS - 2) * (REPEAT_LATCH_GAP + 1)
} else {
    CHANNELS * (8 + TRANSFER_DCLKS)
};
// Configuration leaves arbitrary bits in the shift registers. Clear them
// before the first latch. OUT0 also needs an unlatched clear at frame end.
pub const DATA_PREFIX_DCLKS: usize = if SHORT_DATA { 8 + TRANSFER_DCLKS } else { 0 };
pub const DATA_SUFFIX_DCLKS: usize =
    if SHORT_DATA && TARGET_CHANNEL == 0 { 8 + TRANSFER_DCLKS } else { 0 };
pub const DATA_END: usize =
    49 + CONFIG_WRITES * (8 + TRANSFER_DCLKS)
        + DATA_PREFIX_DCLKS + ROWS * DATA_ROW_DCLKS + ROWS - 1 + DATA_SUFFIX_DCLKS;
// Load all data before the repeated display sweeps; next VSYNC follows the last
// complete row window. Displayed data may lag the transmitted data by a frame.
pub const FIRST_ROW: usize = DATA_END + 16;
pub const FRAME_DCLKS: usize = (FIRST_ROW + scan_period() + 1) & !1;
pub const FRAME_WORDS: usize = FRAME_DCLKS / 2;
pub const NORMAL_FRAME_DCLKS: usize = FRAME_DCLKS - CONFIG_SKIP_DCLKS;
pub const HEAD_WORDS: usize = DATA_END / 2;
pub const TAIL_WORDS: usize = FRAME_WORDS - HEAD_WORDS;
pub const ROW_RUN_COUNT: usize = GROUPS * ROWS * 2 + 1;
pub const VSYNC_FALL: usize = 8;
pub const fn row_period(_index: usize) -> usize {
    ROW_DCLKS
}

const fn scan_period() -> usize {
    GROUPS * ROWS * ROW_DCLKS
}

/// Black plus eight evenly spaced words from 511 to 8191, rounded to integers.
pub const GRAY_SWEEP: &[u16] = &[
    0, 511, 1608, 2705, 3802, 4900, 5997, 7094, 8191,
];

/// Patch only the gray-data SIN bits in a buffer not currently used by DMA.
pub fn update_white(words: &mut [u32], gray: u16) {
    assert!(gray <= MAX_GRAY_INPUT);
    if SHORT_DATA {
        let before_target = if TARGET_CHANNEL == 0 {
            8 + TRANSFER_DCLKS + (CHANNELS - 2) * (REPEAT_LATCH_GAP + 1)
        } else {
            (CHANNELS - 1 - TARGET_CHANNEL) * (REPEAT_LATCH_GAP + 1)
        };
        for row in 0..ROWS {
            let start = 49 + CONFIG_WRITES * (8 + TRANSFER_DCLKS)
                + DATA_PREFIX_DCLKS + row * (DATA_ROW_DCLKS + 1) + before_target + 8;
            for t in start..start + TRANSFER_DCLKS {
                let bit = 15 - (t - start) % 16;
                let mask = (SIN as u32) << ((t & 1) * 16);
                words[t / 2] = (words[t / 2] & !mask)
                    | if gray & (1 << bit) != 0 { mask } else { 0 };
            }
        }
        return;
    }
    let mut start = 49 + CONFIG_WRITES * (8 + TRANSFER_DCLKS);
    for row in 0..ROWS {
        for transfer in 0..CHANNELS {
            start += 8 + usize::from(row != 0 && transfer == 0);
            for t in start..start + TRANSFER_DCLKS {
                let bit = 15 - (t - start) % 16;
                let mask = (SIN as u32) << ((t & 1) * 16);
                words[t / 2] = (words[t / 2] & !mask)
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
    pub fn sample(&mut self, pins: u16) {
        let clock = self.clocks;
        let word = self.word_mut(clock / 2);
        if clock & 1 == 0 {
            *word = pins as u32;
        } else {
            *word |= (pins as u32) << 16;
        }
        self.clocks += 1;
    }
    pub fn hold(&mut self, pins: u16, clocks: usize) {
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
        self.hold(PARK, gap);
        for copy in 0..CHAIN_COPIES {
            for bit in (0..16).rev() {
                let data = if value & (1 << bit) != 0 { SIN } else { 0 };
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

/// Uniform gray throughout a frame; advance one gray level per frame.
pub fn sweep_frame(frame_index: usize) -> Frame {
    [[GRAY_SWEEP[frame_index % GRAY_SWEEP.len()]; CHANNELS]; ROWS]
}

#[derive(Clone, Copy, Debug)]
pub enum Pattern {
    Reference,
    White,
    Black,
    Out0,
    Out1,
    AlternatingRows,
    Checker,
    Split,
}

/// Logical row/channel masks; only SIN data changes, never scan timing.
pub fn pattern_frame(pattern: Pattern, gray: u16) -> Frame {
    assert!(gray <= MAX_GRAY_INPUT);
    let mut frame = [[0; CHANNELS]; ROWS];
    for (row, values) in frame.iter_mut().enumerate() {
        for (out, value) in values.iter_mut().enumerate() {
            let on = match pattern {
                Pattern::Reference => out != 15,
                Pattern::White => true,
                Pattern::Black => false,
                Pattern::Out0 => out == 0,
                Pattern::Out1 => out == 1,
                Pattern::AlternatingRows => row % 2 == 0,
                Pattern::Checker => (row + out) % 2 == 0,
                Pattern::Split => row < ROWS / 2,
            };
            *value = if on { gray } else { 0 };
        }
    }
    frame
}

pub fn demo_frame() -> Frame {
    pattern_frame(Pattern::Reference, DEMO_GRAY)
}

pub fn encode_frame(words: &mut [u32], frame: &Frame) -> usize {
    encode_with_config(words, frame, None)
}

pub fn encode_with_config(words: &mut [u32], frame: &Frame, rotating: Option<u16>) -> usize {
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

fn encode_wave(wave: &mut Waveform<'_>, frame: &Frame, rotating: Option<u16>) -> usize {
    // Exact rising-edge coordinates from the reference: falls 8, 31, 48.
    wave.command_at(5, VSYNC);
    wave.command_at(20, ENABLE);
    wave.command_at(34, PRE_ACT);
    wave.shift(0x00aa, WR_CFG, 9); // first WR_CFG falls at 57+TRANSFER_DCLKS.
    wave.shift(0x01aa, WR_CFG, 8);
    if let Some(value) = rotating {
        wave.shift(value, WR_CFG, 8);
    } else {
        for &(address, value) in REGISTERS {
            wave.shift((address as u16) << 8 | value as u16, WR_CFG, 8);
        }
    }
    wave.shift(0x0055, WR_CFG, if rotating.is_some() { 9 } else { 8 });
    wave.shift(0x0155, WR_CFG, 8);
    if SHORT_DATA {
        wave.shift(0, 0, 8);
    }
    for (row, values) in frame.iter().enumerate() {
        if SHORT_DATA {
            wave.hold(PARK, usize::from(row != 0));
            for out in (0..CHANNELS).rev() {
                if out == TARGET_CHANNEL {
                    wave.shift(values[out], 1, 8);
                } else if out + 1 == TARGET_CHANNEL
                    || (TARGET_CHANNEL == 0 && out == CHANNELS - 1)
                {
                    // OUT0's next serial slot is next row's OUT15.
                    wave.shift(0, 1, 8);
                } else {
                    // SIN stays zero, so the cleared register remains zero.
                    wave.hold(PARK, REPEAT_LATCH_GAP);
                    wave.hold(PARK | LE, 1);
                }
            }
        } else {
            for out in (0..CHANNELS).rev() {
                // Reference has an extra DCLK between logical data rows.
                let gap = 8 + usize::from(row != 0 && out == CHANNELS - 1);
                wave.shift(values[out], 1, gap);
            }
        }
    }
    if SHORT_DATA && TARGET_CHANNEL == 0 {
        // Clear the last row's target without consuming a 257th pixel.
        wave.shift(0, 0, 8);
    }
    if rotating.is_none() {
        assert_eq!(wave.clocks, DATA_END);
    }
    wave.hold(PARK, FRAME_DCLKS - wave.clocks);

    // Overlay independently running ROW on the serialized command/data bus.
    // GROUPS complete sweeps x 16 rows with a fixed ROW_DCLKS cadence.
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
            let shift = (t & 1) * 16;
            let mask = ((BK | DIN | LCK | ROW) as u32) << shift;
            let word = wave.word_mut(t / 2);
            *word = (*word & !mask) | (row_pins as u32) << shift;
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

pub fn rotation_word(index: usize) -> u16 {
    match index % (REGISTERS.len() + 2) {
        0 => 0x0055,
        1 => 0x0155,
        i => {
            let (a, v) = REGISTERS[i - 2];
            (a as u16) << 8 | v as u16
        }
    }
}

pub fn startup_reset(words: &mut [u32]) {
    // Diagnostic only: extend the first LE pulse from 3 to 9 rising edges.
    for t in 8..14 {
        words[t / 2] |= (LE as u32) << ((t & 1) * 16);
    }
}

pub fn update_rotation(words: &mut [u32], value: u16) {
    // First transfer starts at 57, with an 8-DCLK gap between transfers.
    // Derive the third word's position for the configured chain length.
    // Only SIN changes. Preserve LE and the overlapping ROW controls.
    let start = 57 + 2 * (TRANSFER_DCLKS + 8);
    for t in start..start + TRANSFER_DCLKS {
        let bit = 15 - (t - start) % 16;
        let shift = (t & 1) * 16;
        let mask = (SIN as u32) << shift;
        words[t / 2] = (words[t / 2] & !mask) | (if value & (1 << bit) != 0 { mask } else { 0 });
    }
}
