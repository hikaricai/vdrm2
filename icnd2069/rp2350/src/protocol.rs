//! Pure waveform encoder, also usable on the host without Embassy.
//! One sample = one rising-edge DCLK transfer, NOT one DDR half-cycle.

pub const ROWS: usize = 16;
pub const CHANNELS: usize = 16;
pub const GROUPS: usize = 64;
pub const DCLK_HZ: u32 = 10_000_000;
pub const ROW_DCLKS: usize = 160;
pub const GRAY_K: u8 = 32;
pub const GRAY_CAPACITY: u16 = (GROUPS * GRAY_K as usize * 4) as u16;
pub const DEMO_GRAY: u16 = 255;

// Pin bits relative to OUT_BASE = GP10. DCLK (GP15) is driven by side-set.
pub const LE: u16 = 1;
pub const SIN: u16 = 1 << 3;
pub const LCK: u16 = 1 << 6;
pub const BK: u16 = 1 << 8;
pub const DIN: u16 = 1 << 10;
pub const ROW: u16 = 1 << 12;

// Board assumptions inherited from the RT5960 demo, not verified RT5990 timing.
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
pub const SOFT_RESET: usize = 9;
pub const ENABLE: usize = 11;
pub const DISABLE: usize = 12;
pub const PRE_ACT: usize = 14;

// Guide V4.2, red-channel defaults, with explicit board overrides:
// 16 scan; 64 groups; 10 MHz DCLK; M=2, P=8.
// N code 2 is tentatively treated as /4 => GCLK/DCLK=1.
// The guide does NOT specify this code mapping; verify on hardware.
// Even for /1 or /2 the conservative 160-DCLK row still fits.
// 0x1C=0x40 selects a low current gain (~33.6% by the guide formula).
pub const REGISTERS: &[(u8, u8)] = &[
    (0x02, (ROWS - 1) as u8),
    (0x03, (GROUPS - 1) as u8),
    (0x04, 0x02),
    (0x05, 0x08),
    (0x06, 0x02),
    (0x07, GRAY_K),
    (0x08, GRAY_K),
    (0x09, 0x08),
    (0x0a, 0x08),
    (0x0b, 0x00),
    (0x0c, 0x20),
    (0x0d, 0x01),
    (0x0e, 0x04),
    (0x0f, 0x01),
    (0x10, 0x82),
    (0x11, 0x21),
    (0x12, 0x01),
    (0x13, 0x00),
    (0x14, 0x00),
    (0x15, 0x00),
    (0x16, 0x00),
    (0x17, 0xf0),
    (0x18, 0x1f),
    (0x19, 0x00),
    (0x1a, 0x1f),
    (0x1b, 0x10),
    (0x1c, 0x40),
    (0x1d, 0xca),
    (0x1e, 0x42),
    (0x1f, 0x04),
    (0x20, 0x08),
    (0x21, 0x01),
    (0x22, 0x1c),
    (0x23, 0x00),
];

pub const GRAY_DCLKS: usize = ROWS * CHANNELS * 17;
pub const SCAN_START: usize = ROWS * 8 + (VSYNC + 2) + (ENABLE + 2) + 16;
pub const FRAME_DCLKS: usize = SCAN_START + GROUPS * ROWS * ROW_DCLKS + 16;
pub const FRAME_WORDS: usize = FRAME_DCLKS.div_ceil(2);
pub const INIT_WORDS: usize = 4096;

/// Each u32 carries two successive 16-bit pin samples, low half first.
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

    pub fn command(&mut self, clocks: usize) {
        self.sample(PARK);
        self.hold(PARK | LE, clocks);
        self.sample(PARK); // LE falling edge completes the command.
    }

    pub fn write_register(&mut self, address: u8, value: u8) {
        self.shift16(((address as u16) << 8) | value as u16, WR_CFG);
    }

    fn shift16(&mut self, value: u16, le_clocks: usize) {
        for bit in (0..16).rev() {
            let data = if value & (1 << bit) != 0 { SIN } else { 0 };
            let latch = if bit < le_clocks { LE } else { 0 };
            self.sample(PARK | data | latch);
        }
        self.sample(PARK);
    }

    fn clear_rows(&mut self) {
        // Flush all 16 row stages with inactive bits while BK blanks outputs.
        for _ in 0..ROWS {
            self.hold(PARK, 4);
            self.hold(PARK | LCK, 2);
            self.hold(PARK, 2);
        }
    }

    pub fn finish(mut self) -> usize {
        if self.clocks & 1 != 0 {
            self.sample(PARK);
        }
        self.clocks / 2
    }
}

pub fn encode_init(words: &mut [u32]) -> usize {
    let mut wave = Waveform::new(words);
    wave.hold(PARK, 32);
    wave.clear_rows();
    wave.command(SOFT_RESET);
    wave.command(DISABLE);
    wave.command(PRE_ACT);
    wave.write_register(0x00, 0xaa);
    wave.write_register(0x01, 0xaa);
    for &(address, value) in REGISTERS {
        wave.write_register(address, value);
    }
    wave.write_register(0x00, 0x55);
    wave.write_register(0x01, 0x55);
    wave.hold(PARK, 2048); // Allow PLL settling with DCLK running (~205 us).
    wave.command(VSYNC); // Reset the SRAM write address before the black frame.
    for _ in 0..ROWS * CHANNELS {
        wave.shift16(0, 1);
    }
    wave.hold(PARK, 16);
    wave.finish()
}

/// Indexed by logical scan row and ICND2069 OUT number.
pub type Frame = [[u16; CHANNELS]; ROWS];

pub fn demo_frame() -> Frame {
    // Uniform raw 16-bit gray value: compare all rows and OUT channels.
    [[DEMO_GRAY; CHANNELS]; ROWS]
}

pub fn encode_frame(words: &mut [u32], frame: &Frame) -> usize {
    assert!(GRAY_DCLKS < GROUPS * ROWS * ROW_DCLKS);
    let mut wave = Waveform::new(words);
    wave.clear_rows();
    wave.command(VSYNC);
    wave.command(ENABLE);
    wave.hold(PARK, 16);

    // The current SRAM frame is displayed while the next frame is written.
    for t in 0..GROUPS * ROWS * ROW_DCLKS {
        let scan_row = (t / ROW_DCLKS) % ROWS;
        let phase = t % ROW_DCLKS;
        let first = t / ROW_DCLKS == 0;
        let row_width = if first { 12 } else { 4 };
        // BK leads LCK; LCK has 4 DCLK setup and 2 DCLK high time.
        // Unblank at ROW rise. DIN remains stable across both LCK edges.
        let mut pins = blank(phase < 8);
        pins |= select_bit(scan_row == 0 && phase < 10);
        if (4..6).contains(&phase) {
            pins |= LCK;
        }
        if (8..8 + row_width).contains(&phase) {
            pins |= ROW;
        }
        if t < GRAY_DCLKS {
            let pixel = t / 17;
            let bit = t % 17;
            if bit < 16 {
                let value = frame[pixel / CHANNELS][CHANNELS - 1 - pixel % CHANNELS];
                if value & (1 << (15 - bit)) != 0 {
                    pins |= SIN;
                }
                if bit == 15 {
                    pins |= LE;
                }
            }
        }
        wave.sample(pins);
    }
    // The SM may pause here between DMA transfers: never leave a row enabled.
    wave.hold(PARK, 16);
    wave.finish()
}
