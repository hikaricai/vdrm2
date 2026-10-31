//! Reference receiver flow with the captured 40-scan protocol.
//! Logical scan rows do not imply 40 physical rows on the attached board.
//! SDR: LE length counts DCLK rising edges. Two pin samples per DMA word.

pub const ROWS: usize = 40;
pub const CHANNELS: usize = 16;
pub const GROUPS: usize = 64;
pub const DCLK_HZ: u32 = 12_500_000;
// Reference measured 80..83 DCLK, predominantly 81, per row.
pub const ROW_DCLKS: usize = 81;
pub const GRAY_K: u8 = 32;
pub const GRAY_CAPACITY: u16 = (GROUPS * GRAY_K as usize * 4) as u16;
pub const DEMO_GRAY: u16 = 255;

pub const LE: u16 = 1;
pub const SIN: u16 = 1 << 3;
pub const LCK: u16 = 1 << 6;
pub const BK: u16 = 1 << 8;
pub const DIN: u16 = 1 << 10;
pub const ROW: u16 = 1 << 12;
// RT5990 pin identities in the reference capture remain unconfirmed.
pub const BK_ACTIVE_HIGH: bool = true;
pub const DIN_ONE_SELECTS: bool = true;
const fn blank(on: bool) -> u16 {
    if on == BK_ACTIVE_HIGH { BK } else { 0 }
}
const fn select_bit(selected: bool) -> u16 {
    if selected == DIN_ONE_SELECTS { DIN } else { 0 }
}
pub const PARK: u16 = blank(true) | select_bit(false);
pub const PARK_WORD: u32 = PARK as u32 | ((PARK as u32) << 16);

pub const VSYNC: usize = 3;
pub const WR_CFG: usize = 5;
pub const ENABLE: usize = 11;
pub const PRE_ACT: usize = 14;

// Captured reference R-chain values, including 02=27 for 40 scan rows.
// 70..74/F0..F5 writes reproduce observed traffic, not documented semantics.
// The measured PLL tuple is reproduced without claiming a verified N mapping.
pub const REGISTERS: &[(u8, u8)] = &[
    (0x00, 0x55), (0x01, 0x55),
    (0x02, (ROWS - 1) as u8), (0x03, 0x3f),
    (0x04, 0x04), (0x05, 0x0a), (0x06, 0x01),
    (0x07, GRAY_K), (0x08, GRAY_K), (0x09, 0x08), (0x0a, 0x08),
    (0x0b, 0x00), (0x0c, 0x01), (0x0d, 0x01), (0x0e, 0x07),
    (0x0f, 0x01), (0x10, 0xc2), (0x11, 0x21), (0x12, 0x01),
    (0x13, 0x00), (0x14, 0x00), (0x15, 0x00), (0x16, 0x00),
    (0x17, 0xf0), (0x18, 0x1f), (0x19, 0x20), (0x1a, 0x1f),
    (0x1b, 0x10), (0x1c, 0x72), (0x1d, 0x82), (0x1e, 0x41),
    (0x1f, 0x04), (0x20, 0x48), (0x21, 0x01), (0x22, 0x1c),
    (0x70, 0x00), (0x71, 0x00), (0x72, 0x00), (0x73, 0x00),
    (0x74, 0x00), (0xf0, 0x00), (0xf1, 0x00), (0xf2, 0x00),
    (0xf3, 0x00), (0xf4, 0x00), (0xf5, 0x00), (0x23, 0x01),
];

// Clear the row shift register with ROWS low bits before VSYNC.
pub const VSYNC_START: usize = ROWS * 8;
pub const VSYNC_FALL: usize = VSYNC_START + 4;
pub const SCAN_START: usize = VSYNC_FALL + 25;
// Approximately 60 Hz including blank tail; DCLK also runs during that tail.
pub const FRAME_DCLKS: usize = 208_334;
pub const FRAME_WORDS: usize = FRAME_DCLKS / 2;
const PRE_FALL: usize = VSYNC_FALL + 40;
const SLOT_DCLKS: usize = 136;
const FIRST_CFG_FALL: usize = PRE_FALL + 137;
const ROTATING_CFG_FALL: usize = FIRST_CFG_FALL + 2 * SLOT_DCLKS;

pub type Frame = [[u16; CHANNELS]; ROWS];
pub fn demo_frame() -> Frame {
    [[DEMO_GRAY; CHANNELS]; ROWS]
}

fn set_sample(words: &mut [u32], t: usize, pins: u16) {
    let shift = (t & 1) * 16;
    words[t / 2] = (words[t / 2] & !(0xffffu32 << shift)) | ((pins as u32) << shift);
}

fn serial_sample(words: &mut [u32], t: usize, pins: u16) {
    let shift = (t & 1) * 16;
    let mask = ((LE | SIN) as u32) << shift;
    words[t / 2] = (words[t / 2] & !mask) | ((pins as u32) << shift);
}

fn command(words: &mut [u32], fall: usize, clocks: usize) {
    for t in fall - clocks..fall {
        serial_sample(words, t, LE);
    }
    serial_sample(words, fall, 0);
}

/// Same eight repeated 16-bit words as the reference; one chip retains the last.
/// Only the last word carries LE, covering its final le_clocks rising edges.
fn transfer(words: &mut [u32], fall: usize, value: u16, le_clocks: usize) {
    for i in 0..128 {
        let sin = if value & (1 << (15 - i % 16)) != 0 { SIN } else { 0 };
        serial_sample(words, fall - 128 + i, sin | if i >= 128 - le_clocks { LE } else { 0 });
    }
    serial_sample(words, fall, 0);
}

/// Patch only the rotating register slot after send() has drained the DMA/FIFO.
pub fn update_register(words: &mut [u32], index: usize) {
    let (address, value) = REGISTERS[index % REGISTERS.len()];
    transfer(words, ROTATING_CFG_FALL, ((address as u16) << 8) | value as u16, WR_CFG);
}

pub fn encode_frame(words: &mut [u32], frame: &Frame) -> usize {
    assert_eq!(words.len(), FRAME_WORDS);
    words.fill(PARK_WORD);
    for row in 0..ROWS {
        for phase in 4..6 {
            set_sample(words, row * 8 + phase, PARK | LCK);
        }
    }
    // Row controls retain the board assumptions; source protocol follows ROW.
    for i in 0..GROUPS * ROWS {
        let start = SCAN_START + i * ROW_DCLKS;
        let width = if i == 0 { 12 } else { 4 };
        for phase in 0..ROW_DCLKS {
            // Prepare DIN/LCK/BK eight clocks ahead of the ROW edge.
            let t = start - 8 + phase;
            let mut pins = blank(phase < 8);
            pins |= select_bit(i % ROWS == 0 && phase < 10);
            if (4..6).contains(&phase) { pins |= LCK; }
            if (8..8 + width).contains(&phase) { pins |= ROW; }
            set_sample(words, t, pins);
        }
    }
    command(words, VSYNC_FALL, VSYNC);
    command(words, VSYNC_FALL + 23, ENABLE);
    command(words, PRE_FALL, PRE_ACT);
    for (i, value) in [0x00aa, 0x01aa, 0x0055, 0x0055, 0x0155].into_iter().enumerate() {
        transfer(words, FIRST_CFG_FALL + i * SLOT_DCLKS, value, WR_CFG);
    }
    update_register(words, 0);
    for (i, value) in frame.iter().flat_map(|line| line.iter().rev()).enumerate() {
        transfer(words, FIRST_CFG_FALL + (5 + i) * SLOT_DCLKS, *value, 1);
    }
    const {
        assert!(FIRST_CFG_FALL + (5 + ROWS * CHANNELS) * SLOT_DCLKS < SCAN_START + GROUPS * ROWS * ROW_DCLKS);
        assert!(SCAN_START + GROUPS * ROWS * ROW_DCLKS < FRAME_DCLKS);
    }
    FRAME_WORDS
}
