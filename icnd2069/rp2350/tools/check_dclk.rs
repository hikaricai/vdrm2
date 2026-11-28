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
    let low = program.code[3];
    assert_eq!(pull >> 13, 4); // PULL
    assert_ne!(pull & 0x80, 0);
    assert_eq!(pull & 0x20, 0, "empty FIFO must copy X=0 without stopping DCLK");
    assert_eq!(pull & 0x1000, 0x1000, "PULL holds DCLK high for half a period");
    assert_eq!(((pull >> 8) & 15) + 1, half as u16);
    assert_eq!(out >> 13, 3);
    assert_eq!(out & 31, if DIRECT_BOARD { 19 } else { 23 });
    assert_eq!(out & 0x1000, 0, "DATA must update on the falling edge");
    assert_eq!((out >> 8) & 15, 0);
    assert_eq!(low & 0x1000, 0);
    assert_eq!(((low >> 8) & 15) + 1, half as u16 - 1);
    // Derive the sample window from the emitted instruction delays.
    let instruction_cycles = |word: u16| u32::from((word >> 8) & 15) + 1;
    let data_phase = instruction_cycles(pull);
    let period = program.code[1..].iter().map(|&word| instruction_cycles(word)).sum::<u32>();
    assert_eq!(period, PIO_CYCLES_PER_DCLK);
    assert_eq!(data_phase * 2, period, "rising edge must center each DATA/LAT word");
    for (hz, divider) in [
        (DCLK_HZ, PIO_CLKDIV_INT << 8),
        (CFG_DCLK_HZ, CFG_CLKDIV_BITS),
    ] {
        assert_eq!(u64::from(hz) * u64::from(divider) * u64::from(PIO_CYCLES_PER_DCLK),
            u64::from(SYS_HZ) * 256);
        let pio_ns = f64::from(divider) * 1e9 / (f64::from(SYS_HZ) * 256.0);
        println!(
            "DIRECT timing nominal: DCLK={} Hz, DATA update={:.3} ns after rise, \
             setup={:.3} ns, hold={:.3} ns, rise at {:.1}% of one bit interval, centered={}",
            hz, f64::from(data_phase) * pio_ns, f64::from(period - data_phase) * pio_ns,
            f64::from(data_phase) * pio_ns, f64::from(period - data_phase) * 100.0 / f64::from(period),
            data_phase * 2 == period,
        );
    }
    let image = pattern_frame_range(Pattern::Diagonal, [0, MAX_GRAY_INPUT.min(255)]);
    let mut full = vec![0; FRAME_WORDS];
    encode_frame(&mut full, &image);
    let connected = if DIRECT_BOARD {
        0xfff | (1 << 13) | (1 << 15) | (1 << 16) | (1 << 17) | (1 << 18)
    } else {
        (if DEV_BOARD { (1 << 13) | (1 << 10) } else { 0x7fff })
            | (1 << 16) | (1 << 18) | (1 << 20) | (1 << 22)
    };
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
