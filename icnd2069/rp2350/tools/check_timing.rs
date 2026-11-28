//! Decode actual prepared DMA buffers, with independent JSON-derived expectations.
#![allow(dead_code)]
#[path = "../src/protocol.rs"]
mod protocol;
use protocol::*;

fn setting(name: &str) -> usize {
    std::env::var(format!("ICND2069_TIMING_{name}")).unwrap().parse().unwrap()
}

fn main() {
    let pattern = if COLUMN_PATTERN { Pattern::Column(TARGET_CHANNEL) }
        else if REVERSE_DIAGONAL { Pattern::DiagonalReverse } else { Pattern::Diagonal };
    let image = pattern_frame_range(pattern, [255, 31]);
    let mut config = vec![0; FRAME_WORDS + CFG_POST_CLR_DCLKS];
    let mut normal = vec![0; NORMAL_FRAME_DCLKS];
    prepare_frames(&mut config, &mut normal, &image);
    let mut commands = Vec::new();
    let mut width = 0;
    let mut shift = [0u128; 3];
    let mut row_edges = Vec::new();
    let mut row_widths = Vec::new();
    let mut row_high = 0;
    // Independent physical RGB mapping, not firmware-generated masks.
    let pins: [Option<usize>; 3] = if DEV_BOARD { [Some(13), None, None] }
        else if DIRECT_BOARD { [Some(5), Some(4), Some(3)] }
        else { [Some(0), Some(1), Some(2)] };
    for (t, &word) in normal.iter().enumerate() {
        if word & LE == 0 && width != 0 {
            commands.push((t - width, t, width, shift));
            width = 0;
        }
        for (color, pin) in pins.iter().enumerate() {
            if let Some(pin) = pin {
                shift[color] = shift[color].wrapping_shl(1) | u128::from(word & (1 << pin) != 0);
            }
        }
        if word & LE != 0 { width += 1; }
        if word & ROW != 0 {
            if row_high == 0 { row_edges.push(t); }
            row_high += 1;
        } else if row_high != 0 {
            row_widths.push(row_high);
            row_high = 0;
        }
    }
    assert_eq!((width, row_high), (0, 0));
    assert_eq!(commands[0].2, 3);
    let sync = commands[0].0;
    let en = setting("EN_START");
    assert_eq!((commands[1].0 - sync, commands[1].2), (en, 11));
    assert_eq!(commands.len(), 2 + ROWS * 16, "normal frame must contain VSYNC, EN and grayscale");
    let data = &commands[2..];
    let start = sync + setting("GRAY_START");
    assert!(normal[commands[1].1..start].iter().all(|w| w & (LE | SIN) == 0));
    for (i, command) in data.iter().enumerate() {
        assert_eq!(command.2, 1);
        let row = i / 16;
        let out = 15 - i % 16;
        if !SHORT_DATA {
            let bits = CHAIN_COPIES * 16;
            let first = start + i * (bits + setting("GRAY_WORD_GAP"))
                + row * setting("GRAY_ROW_GAP");
            assert_eq!(command.0, first + bits - 1);
            assert_eq!(command.1, first + bits);
        }
        for (color, pin) in pins.iter().enumerate() {
            if let Some(pin) = pin {
                for chip in 0..CHAIN_COPIES {
                    let expected = if GRAY_SIN & (1 << pin) != 0 && out < LED_CHANNELS {
                        image[row][chip * LED_CHANNELS + out]
                    } else { 0 };
                    assert_eq!((command.3[color] >> (16 * chip)) as u16, expected,
                        "row={row} out={out} chip={chip} color={color}");
                }
            }
        }
    }
    // Short mode's first transfer may be a zero repeated latch or a gray byte.
    // The first source bit is anchored even when all leading SIN values are zero.
    if SHORT_DATA {
        let expected_first = if latch_target(0) == 15 {
            start + 8 + 16 * latch_chip(0)
        } else {
            start + REPEAT_LATCH_GAP + 1
        };
        assert_eq!(data[0].1, expected_first);
    }
    let count = ROWS * GROUPS;
    assert_eq!(row_edges.len(), count);
    assert_eq!(row_widths[0], 12);
    assert!(row_widths[1..].iter().all(|w| *w == 4));
    let first_row = sync + setting("ROW_START");
    let scan = if KEEP_SCAN_TIME { NORMAL_DATA_END - first_row }
        else { count * setting("ROW_PERIOD") };
    for (i, &t) in row_edges.iter().enumerate() {
        assert_eq!(t, first_row + i * scan / count);
        if !DISABLE_RT5990 {
            assert!(normal[t - 4..t - 2].iter().all(|w| w & LCK != 0));
            assert!(normal[t - 2..t].iter().all(|w| w & LCK == 0));
        }
    }
    if DISABLE_RT5990 {
        assert!(normal.iter().all(|w| w & (LCK | DIN) == 0));
    }
    let last_row_fall = row_edges[count - 1] + row_widths[count - 1];
    let tail = normal.len() + sync - last_row_fall;
    assert!(tail >= setting("FRAME_TAIL"));
    assert!(normal.len() >= data.last().unwrap().1 + 16);
    assert_eq!(normal.last().unwrap() & (LE | ROW), 0);
    println!("PASS {BOARD}: EN={}, ROW={}, gray={}, first_latch={}, last_latch={}, rows={}, tail={}, normal={} DCLK, repeat={}",
        commands[1].0 - sync, row_edges[0] - sync, start - sync,
        data[0].1 - sync, data.last().unwrap().1 - sync, count, tail, normal.len(), SHORT_DATA);
}
