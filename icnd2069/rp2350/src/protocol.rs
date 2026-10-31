//! ICND2069 reference timing, adapted to 16 scans and uniform gray 255.
//! Each halfword describes pins for one DCLK rising-edge transfer.
//! Reference: dbg/nova_to_devboard_100m_2s.dsl and the startup summary.

pub const ROWS: usize = 16;
pub const CHANNELS: usize = 16;
pub const GROUPS: usize = 64;
pub const DCLK_HZ: u32 = 12_500_000;
pub const ROW_DCLKS: usize = 81;
pub const GRAY_K: u8 = 32;
pub const GRAY_CAPACITY: u16 = (GROUPS * GRAY_K as usize * 4) as u16;
pub const DEMO_GRAY: u16 = 255;
pub const CHAIN_COPIES: usize = 8;

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

// Red/SIN values from the actual receiving-card captures. Only 02 changes.
// 00/01 are access controls, sent outside this list to keep the whole batch
// unlocked. The reference rotation includes them but they are not payload.
pub const REGISTERS: &[(u8, u8)] = &[
    (0x02, (ROWS - 1) as u8),
    (0x03, 0x3f),
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
pub const FRAME_DCLKS: usize = 208_334; // ~60 Hz at 12.5 MHz.
pub const FRAME_WORDS: usize = FRAME_DCLKS / 2;
pub const VSYNC_FALL: usize = 8;
pub const FIRST_ROW: usize = VSYNC_FALL + 25;

pub struct Waveform<'a> {
    words: &'a mut [u32],
    clocks: usize,
}
impl<'a> Waveform<'a> {
    pub fn new(words: &'a mut [u32]) -> Self {
        Self { words, clocks: 0 }
    }
    pub fn sample(&mut self, pins: u16) {
        let word = &mut self.words[self.clocks / 2];
        if self.clocks & 1 == 0 {
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
pub fn demo_frame() -> Frame {
    [[DEMO_GRAY; CHANNELS]; ROWS]
}

pub fn encode_frame(words: &mut [u32], frame: &Frame) -> usize {
    assert_eq!(words.len(), FRAME_WORDS);
    let mut wave = Waveform::new(words);
    // Exact rising-edge coordinates from the reference: falls 8, 31, 48.
    wave.command_at(5, VSYNC);
    wave.command_at(20, ENABLE);
    wave.command_at(34, PRE_ACT);
    wave.shift(0x00aa, WR_CFG, 9); // first WR_CFG falls at 185.
    wave.shift(0x01aa, WR_CFG, 8);
    for &(address, value) in REGISTERS {
        wave.shift((address as u16) << 8 | value as u16, WR_CFG, 8);
    }
    wave.shift(0x0055, WR_CFG, 8);
    wave.shift(0x0155, WR_CFG, 8);
    for (row, values) in frame.iter().enumerate() {
        for out in (0..CHANNELS).rev() {
            // Reference has an extra DCLK between logical data rows.
            let gap = 8 + usize::from(row != 0 && out == CHANNELS - 1);
            wave.shift(values[out], 1, gap);
        }
    }
    assert!(wave.clocks < FIRST_ROW + GROUPS * ROWS * ROW_DCLKS);
    wave.hold(PARK, FRAME_DCLKS - wave.clocks);

    // Overlay independently running ROW on the serialized command/data bus.
    // 64 groups x 16 scans; reference local width/cadence and VSYNC phase.
    for i in 0..GROUPS * ROWS {
        let start = FIRST_ROW + i * ROW_DCLKS;
        let width = if i == 0 { 12 } else { 4 };
        for t in start - 8..start + ROW_DCLKS - 8 {
            let mut row_pins = blank(t < start);
            row_pins |= select_bit(i % ROWS == 0 && t < start + 2);
            if (start - 4..start - 2).contains(&t) {
                row_pins |= LCK;
            }
            if (start..start + width).contains(&t) {
                row_pins |= ROW;
            }
            let shift = (t & 1) * 16;
            let mask = ((BK | DIN | LCK | ROW) as u32) << shift;
            words[t / 2] = (words[t / 2] & !mask) | (row_pins as u32) << shift;
        }
    }
    FRAME_WORDS
}
