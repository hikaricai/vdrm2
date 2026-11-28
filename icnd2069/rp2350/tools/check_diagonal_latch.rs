//! Host verification; compile with rustc, using the firmware's build-time env.
#![allow(dead_code)]
#[path = "../src/protocol.rs"]
mod protocol;
use protocol::*;

fn decode(words: &[u32], expected: &Frame) {
    let data_start = RT5990_CLEAR_DCLKS + 63 + CONFIG_WRITES * (8 + CFG_PRE_CLR_DCLKS + CFG_TRANSFER_DCLKS);
    // Deliberately dirty the chain: the initial full zero transfer must
    // clear it before relying on omitted leading gray bits.
    let mut bits = vec![true; TRANSFER_DCLKS];
    let mut width = 0;
    let mut saved = bits.clone();
    let mut pixels = 0;
    let mut previous_latch = None;
    for t in data_start..=DATA_END {
        let pins = words[t];
        assert!(pins & SIN == 0 || pins & SIN == GRAY_SIN,
            "gray must drive only selected colors at clock {t}");
        // Model all cascaded shift registers, then sample at the latch clock.
        bits.rotate_left(1);
        bits[TRANSFER_DCLKS - 1] = pins & SIN != 0;
        if pins & LE != 0 {
            width += 1;
            saved.clone_from(&bits);
        } else if width != 0 {
            assert_eq!(width, 1, "merged LAT command at clock {t}");
            assert!(pixels < ROWS * CHANNELS);
            let row = pixels / CHANNELS;
            let out = CHANNELS - 1 - pixels % CHANNELS;
            if let Some(previous) = previous_latch {
                if !CONTINUOUS_LATCH {
                    assert_eq!(t - previous, TRANSFER_DCLKS + usize::from(out == CHANNELS - 1));
                } else if out == latch_target(row) {
                    // Dev may start a row with lit OUT15 immediately after
                    // the previous row's OUT0 and its unlatched zero flush.
                    let row_clear = if out == CHANNELS - 1 && row > 0 && latch_target(row - 1) == 0 {
                        latch_clear_dclks(row - 1)
                    } else { 0 };
                    assert_eq!(t - previous, row_clear + 16 * (1 + latch_pixel(row) / LED_CHANNELS),
                        "16-bit gray word must advance to only the selected chip");
                } else if latch_target(row) > 0 && out + 1 == latch_target(row) {
                    assert_eq!(t - previous, 16 * (CHAIN_COPIES - latch_pixel(row) / LED_CHANNELS),
                        "clear must flush the selected gray out of the chain");
                }
            }
            previous_latch = Some(t);
            for chip in 0..CHAIN_COPIES {
                let serial = CHAIN_COPIES - 1 - chip;
                let value = saved[serial * 16..(serial + 1) * 16]
                    .iter().fold(0u16, |value, bit| (value << 1) | u16::from(*bit));
                let expected = if GRAY_SIN == 0 || out >= LED_CHANNELS { 0 }
                    else { expected[row][chip * LED_CHANNELS + out] };
                assert_eq!(value, expected, "row={row} OUT{out} chip={chip}");
            }
            pixels += 1;
            width = 0;
        }
    }
    assert_eq!(width, 0);
    assert_eq!(pixels, ROWS * CHANNELS);
}

fn main() {
    let width = if DEV_BOARD { 16 } else { 15 };
    assert_eq!((CHANNELS, LED_CHANNELS, TOTAL_CHANNELS), (16, width, CHAIN_COPIES * width));
    if DEV_BOARD {
        assert_eq!((SIN, LE, SEL, RT5990_CLEAR_ROWS), (1 << 13, 1 << 10, 0, 16));
        assert_eq!(RGB_SIN, [1 << 13, 0, 0]);
    } else if DIRECT_BOARD {
        assert_eq!((SIN, LE, SEL, RT5990_CLEAR_ROWS), (0xfff, 1 << 18, 0, 160));
        assert_eq!(RGB_SIN, [0x924, 0x492, 0x249]);
        assert_eq!((ROW, DIN, LCK, BK, PIO_OUT_COUNT),
            (1 << 13, 1 << 17, 1 << 16, 1 << 15, 19));
    } else {
        assert_eq!((SIN, LE, SEL, RT5990_CLEAR_ROWS), (0x777, 0x7000, 0x888, 160));
    }
    let pattern = if COLUMN_PATTERN { Pattern::Column(TARGET_CHANNEL) }
        else if REVERSE_DIAGONAL { Pattern::DiagonalReverse }
        else if DIAGONAL_LATCH { Pattern::Diagonal } else { Pattern::Normal };
    let image = pattern_frame_range(pattern, [0, 255]);
    if DIAGONAL_LATCH {
        for row in 0..ROWS {
            let step = row % TOTAL_CHANNELS;
            let column = if REVERSE_DIAGONAL { TOTAL_CHANNELS - 1 - step } else { step };
            let mut expected = [0; TOTAL_CHANNELS];
            expected[column] = (255.0 * (TOTAL_CHANNELS - 1 - step) as f64
                / (TOTAL_CHANNELS - 1) as f64).round() as u16;
            assert_eq!(image[row], expected, "diagonal gray range row={row}");
        }
        for range in [[255, 255], [255, 0], [16, 64], [0, 0]] {
            let forward = pattern_frame_range(Pattern::Diagonal, range);
            let reverse = pattern_frame_range(Pattern::DiagonalReverse, range);
            for row in 0..ROWS {
                for column in 0..TOTAL_CHANNELS {
                    assert_eq!(reverse[row][column], forward[row][TOTAL_CHANNELS - 1 - column]);
                }
            }
            let image = if REVERSE_DIAGONAL { reverse } else { forward };
            let mut words = vec![0; FRAME_WORDS];
            encode_frame(&mut words, &image);
            decode(&words, &image);
        }
    } else if COLUMN_PATTERN {
        for (row, values) in image.iter().enumerate() {
            for (column, &value) in values.iter().enumerate() {
                let expected = if column == TARGET_CHANNEL {
                    if ROWS == 1 { 255 } else {
                        (255.0 * (ROWS - 1 - row) as f64 / (ROWS - 1) as f64).round() as u16
                    }
                } else { 0 };
                assert_eq!(value, expected, "fixed column row={row} column={column}");
            }
        }
    } else {
        let mut expected = std::array::from_fn::<_, TOTAL_CHANNELS, _>(|column|
            (255.0 * column as f64 / (TOTAL_CHANNELS - 1) as f64).round() as u16);
        for (row, values) in image.iter().enumerate() {
            assert_eq!(*values, expected, "normal gradient row={row}");
            expected.rotate_right(1);
        }
    }
    let image = if CONTINUOUS_LATCH { target_frame(image) } else { image };
    let mut words = vec![0; FRAME_WORDS];
    encode_frame(&mut words, &image);
    let mut configuration = vec![0; FRAME_WORDS + CFG_POST_CLR_DCLKS];
    let mut normal = vec![0; NORMAL_FRAME_DCLKS];
    prepare_frames(&mut configuration, &mut normal, &image);
    let allowed = if DIRECT_BOARD {
        0xfff | (1 << 13) | (1 << 15) | (1 << 16) | (1 << 17) | (1 << 18)
    } else {
        (if DEV_BOARD { (1 << 10) | (1 << 13) } else { 0x7fff })
            | (1 << 16) | (1 << 18) | (1 << 20) | (1 << 22)
    };
    assert!(configuration.iter().chain(&normal).all(|word| word & !allowed == 0),
        "actual DMA must only drive the selected board's signal pins");
    if !HDL_BOARD {
        assert!(configuration[..CPLD_SELECT_DCLKS].iter().all(|&word| word == PARK));
        assert!(normal[..CPLD_SELECT_DCLKS].iter().all(|&word| word == PARK));
        assert!(CONFIG_IDLE_PREFIX.iter().all(|&word| word == PARK));
    }
    let bk_clocks: Vec<_> = (0..FRAME_DCLKS).filter(|&t|
        ((words[t] & BK) != 0) == BK_ACTIVE_HIGH
    ).collect();
    assert_eq!(bk_clocks, (COMMON_PREFIX_WORDS..COMMON_PREFIX_WORDS + RT5990_CLEAR_DCLKS).chain((0..ROWS * SCAN_REPEATS)
        .flat_map(|i| {
            let start = row_start(i);
            start - 8..start
        })).collect::<Vec<_>>(),
        "BK must blank the row-clear sequence and pulse once per scanned row");
    decode(&words, &image);
    let mut head = vec![0; HEAD_WORDS];
    let mut tail = vec![0; TAIL_WORDS];
    encode_split(&mut head, &mut tail, &image);
    head.extend_from_slice(&tail);
    assert_eq!(words, head, "DMA head/tail packing");
    update_white(&mut words, 127);
    let mut patched = [[127; TOTAL_CHANNELS]; ROWS];
    if CONTINUOUS_LATCH {
        patched = target_frame(patched);
    }
    decode(&words, &patched);
    if CONTINUOUS_LATCH {
        for gray in [0, 1, 0x8000, 0xa55a, u16::MAX] {
            let expected = target_frame([[gray; TOTAL_CHANNELS]; ROWS]);
            let mut rebuilt = vec![0; FRAME_WORDS];
            encode_frame(&mut rebuilt, &expected);
            update_white(&mut words, gray);
            assert_eq!(words, rebuilt, "16-bit gray patch must match a fresh encoding");
            decode(&words, &expected);
        }
    }
    let mut runs = [0; ROW_RUN_COUNT];
    encode_row_runs(&mut runs);
    assert_eq!(runs.iter().filter(|run| **run & 1 != 0).count(), ROWS * SCAN_REPEATS);
    assert_eq!(runs.iter().map(|run| (run >> 1) as usize + 1).sum::<usize>(), FRAME_DCLKS);
    let count = ROWS * SCAN_REPEATS;
    let duration = if KEEP_SCAN_TIME { DATA_END - FIRST_ROW } else { count * BASE_ROW_DCLKS };
    let mut elapsed = 0;
    for i in 0..count {
        let period = row_period(i);
        assert!(period == duration / count || period == duration.div_ceil(count));
        elapsed += period;
        // Each boundary stays within one DCLK of the ideal uniform cadence.
        assert!(elapsed * count <= (i + 1) * duration);
        assert!((i + 1) * duration - elapsed * count < count);
    }
    assert_eq!(elapsed, duration);
    assert_eq!(FIRST_ROW + elapsed, SCAN_END);
    if KEEP_SCAN_TIME {
        assert_eq!(SCAN_END, DATA_END, "all sweeps must end with gray loading");
    }
    assert!(FIRST_ROW < DATA_PAYLOAD_END, "scan must overlap data loading");
    assert!(FRAME_DCLKS >= SCAN_END && FRAME_DCLKS >= DATA_END + 16);
    let mut position = 0;
    let mut overlapping_rows = 0;
    for (i, run) in runs.iter().enumerate() {
        if run & 1 != 0 {
            let start = row_start(i / 2);
            assert_eq!(position, start + 1);
            let pins = |t: usize| head[t];
            assert_eq!(pins(start - 4) & LCK, LCK);
            assert_eq!(pins(start) & BK != 0, !BK_ACTIVE_HIGH);
            assert_eq!(pins(start) & DIN != 0, i / 2 % ROWS == 0);
            overlapping_rows += usize::from(start < DATA_PAYLOAD_END);
        }
        position += (run >> 1) as usize + 1;
    }
    // The scheduler removes only configuration clocks from ordinary frames.
    let mut normal = head[..COMMON_PREFIX_WORDS].to_vec();
    normal.extend_from_slice(&head[CONFIG_END_WORDS..]);
    assert_eq!(normal.len(), NORMAL_FRAME_DCLKS);
    assert!(FIRST_ROW - CONFIG_SKIP_DCLKS > 31);
    for t in FIRST_ROW - 8..SCAN_END {
        let original = head[t];
        let shifted = t - CONFIG_SKIP_DCLKS;
        let pins = normal[shifted];
        assert_eq!(pins, original, "normal frame lost data/scan alignment");
    }
    if COLUMN_PATTERN {
        for range in [[255, 0], [0x8001, 0xa55a], [u16::MAX; 2], [0, 0]] {
            let image = pattern_frame_range(Pattern::Column(TARGET_CHANNEL), range);
            for (row, values) in image.iter().enumerate() {
                for (column, &value) in values.iter().enumerate() {
                    let expected = if column == TARGET_CHANNEL {
                        if ROWS == 1 { range[1] } else {
                            (f64::from(range[1]) + (f64::from(range[0]) - f64::from(range[1]))
                                * row as f64 / (ROWS - 1) as f64).round() as u16
                        }
                    } else { 0 };
                    assert_eq!(value, expected);
                }
            }
            encode_frame(&mut words, &image);
            decode(&words, &image);
        }
    }
    if !CONTINUOUS_LATCH {
        for range in [[255, 0], [0, u16::MAX], [u16::MAX; 2], [0, 0]] {
            let image = pattern_frame_range(Pattern::Normal, range);
            let mut expected = std::array::from_fn::<_, TOTAL_CHANNELS, _>(|column|
                (f64::from(range[0]) + (f64::from(range[1]) - f64::from(range[0]))
                    * column as f64 / (TOTAL_CHANNELS - 1) as f64).round() as u16);
            for row in &image {
                assert_eq!(*row, expected);
                expected.rotate_right(1);
            }
            encode_frame(&mut words, &image);
            decode(&words, &image);
        }
        assert_eq!(pattern_frame(Pattern::Normal, 255),
            pattern_frame_range(Pattern::Normal, [255, 255]));
        // Distinct chip/OUT/row values catch reversed chips and broadcasting.
        let image = std::array::from_fn(|row|
            std::array::from_fn(|column| ((row * TOTAL_CHANNELS + column) % 8193) as u16));
        encode_frame(&mut words, &image);
        decode(&words, &image);
    }
    println!("scan starts at normal clock {}, {} ROW pulses overlap data",
        FIRST_ROW - CONFIG_SKIP_DCLKS, overlapping_rows);
    println!(
        "OK: diagonal={}, latch={}, chain={}, gap={}, ROW={}, pixels={}, data_end={}, normal_frame={}",
        DIAGONAL_LATCH, CONTINUOUS_LATCH, CHAIN_COPIES, REPEAT_LATCH_GAP,
        ROWS * SCAN_REPEATS, ROWS * CHANNELS, DATA_END, NORMAL_FRAME_DCLKS,
    );
}
