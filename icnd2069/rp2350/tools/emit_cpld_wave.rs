//! Export the actual DMA buffers with independently specified route expectations.
#![allow(dead_code)]
#[path = "../src/protocol.rs"]
mod protocol;
use protocol::*;

fn emit(words: &[u32], addresses: [u8; 3], hz: u32) {
    for &word in words {
        let mut expected = 0u64;
        for bank in 0..3 {
            let selected = if addresses[bank] < 6 { 1u64 << addresses[bank] } else { 63 };
            for color in 0..3 {
                if word & (1 << (bank * 4 + color)) != 0 {
                    expected |= selected << (bank * 18 + color * 6);
                }
            }
        }
        println!("{word:08x} {expected:014x} {hz}");
    }
}

fn main() {
    let mut configuration = vec![0; FRAME_WORDS + CFG_POST_CLR_DCLKS];
    let mut normal = vec![0; NORMAL_FRAME_DCLKS];
    // Different chip/column values expose any one-bit phase or chain-order error.
    let image = pattern_frame_range(
        if COLUMN_PATTERN { Pattern::Column(TARGET_CHANNEL) } else { Pattern::Diagonal },
        [0, 255]);
    prepare_frames(&mut configuration, &mut normal, &image);
    for _ in 0..2 {
        emit(&configuration[..CONFIG_END_WORDS], CPLD_ADDRESSES, CFG_DCLK_HZ);
        emit(&[PARK; 17], CPLD_ADDRESSES, CFG_DCLK_HZ);
        emit(&configuration[CONFIG_END_WORDS..], CPLD_ADDRESSES, DCLK_HZ);
        emit(&[PARK; 19], CPLD_ADDRESSES, DCLK_HZ);
        for _ in 0..2 {
            emit(&normal, CPLD_ADDRESSES, DCLK_HZ);
            emit(&[PARK; 23], CPLD_ADDRESSES, DCLK_HZ);
        }
    }
}
