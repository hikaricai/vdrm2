//! Check the actual fixed PIO program and combined GPIO waveform.
#![allow(dead_code)]
#[path = "../src/dclk_program.rs"]
mod dclk_program;
#[path = "../src/protocol.rs"]
mod protocol;
use protocol::*;

fn commands(words: &[u32]) -> Vec<usize> {
    let mut result = Vec::new();
    let mut width = 0;
    for &word in words {
        if word & LE != 0 {
            width += 1;
        } else if width != 0 {
            result.push(width);
            width = 0;
        }
    }
    assert_eq!(width, 0, "block must finish with LE low");
    result
}

fn main() {
    let program = dclk_program::fixed_rate(PIO_CYCLES_PER_DCLK);
    assert_eq!(program.code.len(), 4);
    assert_eq!(program.code[0], 0xe020, "initialize idle X=0 with DCLK LOW");
    assert_eq!(program.wrap.target, 1);
    assert_eq!(program.wrap.source, 3);
    let half = PIO_CYCLES_PER_DCLK / 2;
    let pull = program.code[1];
    let out = program.code[2];
    let high = program.code[3];
    assert_eq!(pull >> 13, 4); // PULL
    assert_ne!(pull & 0x80, 0);
    assert_eq!(pull & 0x20, 0, "empty FIFO must copy X=0 without stopping DCLK");
    assert_eq!(pull & 0x1000, 0x1000, "prepare data with DCLK high");
    assert_eq!(((pull >> 8) & 15) + 1, half as u16 - 1);
    assert_eq!(out >> 13, 3);
    assert_eq!(out & 31, 23);
    assert_eq!(out & 0x1000, 0x1000);
    assert_eq!(high & 0x1000, 0);
    assert_eq!(((high >> 8) & 15) + 1, half as u16);
    for (hz, divider) in [
        (DCLK_HZ, PIO_CLKDIV_INT << 8),
        (CFG_DCLK_HZ, CFG_CLKDIV_BITS),
    ] {
        assert_eq!(u64::from(hz) * u64::from(divider) * u64::from(PIO_CYCLES_PER_DCLK),
            u64::from(SYS_HZ) * 256);
    }
    let image = pattern_frame_range(Pattern::Diagonal, [0, MAX_GRAY_INPUT.min(255)]);
    let mut full = vec![0; FRAME_WORDS];
    encode_frame(&mut full, &image);
    let connected = 0x7fff | (1 << 16) | (1 << 18) | (1 << 20) | (1 << 22);
    assert_eq!(SIN | SEL | LE | LCK | BK | DIN | ROW, connected);
    assert!(full.iter().all(|&w| w & !connected == 0),
        "GPIO words may only drive the schematic's data/control pins");
    for row in 0..ROWS * SCAN_REPEATS {
        let start = row_start(row);
        let width = if row == 0 { 12 } else { 4 };
        assert_eq!(full[start - 1] & ROW, 0);
        assert!(full[start..start + width].iter().all(|&w| w & ROW != 0));
        assert_eq!(full[start + width] & ROW, 0);
    }
    let mut normal = full[..COMMON_PREFIX_WORDS].to_vec();
    normal.extend_from_slice(&full[CONFIG_END_WORDS..]);
    assert_eq!(normal.len(), NORMAL_FRAME_DCLKS);
    let normal_commands = commands(&normal);
    assert_eq!(normal_commands.iter().filter(|&&n| n == 3).count(), 1);
    assert_eq!(normal_commands.iter().filter(|&&n| n == 1).count(), ROWS * CHANNELS);
    assert!(!normal_commands.iter().any(|&n| n == 5 || n == 11 || n == 14));
    if !ROW_ON_CFG {
        full[CPLD_SELECT_DCLKS..COMMON_PREFIX_WORDS].fill(PARK);
        full.iter_mut().for_each(|w| *w &= !ROW);
        assert!(full.iter().all(|&w| w & ROW == 0));
    }
    let cfg_commands = commands(&full[..CONFIG_END_WORDS]);
    let clear_start = DATA_START - CFG_POST_CLR_DCLKS;
    assert!(full[clear_start..DATA_START].iter().all(|&w| w & (SIN | LE | ROW) == 0),
        "post-configuration clear must not latch data, issue VSYNC or scan");
    assert!(CFG_POST_CLR_DCLKS >= TRANSFER_DCLKS
        && CFG_POST_CLR_DCLKS >= CFG_TRANSFER_DCLKS);
    assert!(!commands(&full[CONFIG_END_WORDS..]).contains(&VSYNC),
        "write the complete grayscale frame before the next normal-frame VSYNC");
    assert_eq!(cfg_commands.iter().filter(|&&n| n == 5).count(), CONFIG_WRITES);
    assert_eq!(cfg_commands.iter().filter(|&&n| n == 11).count(), 1);
    assert_eq!(cfg_commands.iter().filter(|&&n| n == 3).count(), usize::from(ROW_ON_CFG));
    assert_eq!(full[CONFIG_END_WORDS - 1] & (ROW | LE), 0);
    assert_eq!(full[FRAME_WORDS - 1] & (ROW | LE), 0);
    println!("PASS single PIO: idle-zero continuous clocks, SET outside 3-instruction wrap, divider changes, combined ROW/LE, 49 config writes; DCLK={}, cfg={}", DCLK_HZ, CFG_DCLK_HZ);
}
