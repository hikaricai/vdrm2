//! Host verification: rustc --edition=2021 tools/check_config_interval.rs -o /tmp/icnd2069-interval-check
#![allow(dead_code)]
#[path = "../src/protocol.rs"]
mod protocol;
#[path = "../src/scheduler.rs"]
mod scheduler;
use protocol::*;

fn sample(words: &[u32], t: usize) -> u32 {
    words[t]
}

fn check_reference_registers() {
    let preclear: usize = option_env!("ICND2069_CFG_PRE_CLR_CHAINS")
        .unwrap_or("0").parse().unwrap();
    assert_eq!(CFG_PRE_CLR_DCLKS, preclear * 16);
    assert_eq!(ROW_ON_CFG, option_env!("ICND2069_ROW_ON_CFG").unwrap_or("0") == "1");
    // Independent oracle: the captured RGB table, with only the current
    // scan/PLL/PWM settings substituted. Do not derive expected RGB from masks.
    let requested_gclk: u64 = option_env!("ICND2069_GCLK_HZ")
        .unwrap_or("0").parse().unwrap();
    let expected_pwm: u8 = option_env!("ICND2069_PWM_WIDTH")
        .unwrap_or("16").parse().unwrap();
    let expected_rows: usize = option_env!("ICND2069_LINES")
        .unwrap_or("54").parse().unwrap();
    assert_eq!(ROWS, expected_rows);
    let (expected_m, expected_p) = if DCLK_HZ == 12_500_000 && requested_gclk == 31_250_000 {
        assert_eq!(GCLK_HZ_ASSUMING_N1, 31_250_000);
        (4, 10) // Exact register values observed in the original capture.
    } else if requested_gclk != 0 {
        let (m, p) = (1..=31u64).flat_map(|m| (1..=31u64).map(move |p| (m, p)))
            .find(|&(m, p)| 4_000_000 * m < u64::from(DCLK_HZ)
                && u64::from(DCLK_HZ) < 10_000_000 * m
                && u64::from(DCLK_HZ) * p == requested_gclk * m)
            .expect("requested GCLK has no exact PLL candidate");
        assert_eq!(u64::from(GCLK_HZ_ASSUMING_N1), requested_gclk);
        (m as u8, p as u8)
    } else if DCLK_HZ < 20_000_000 {
        if DCLK_HZ <= 6_000_000 { (1, 10) } else { (2, 12) }
    } else {
        // Independently solve the requested 5x ratio, choosing the smallest
        // M with a valid reference input and a five-bit P.
        let m = (1..=31u32).find(|&m|
            4_000_000 * m < DCLK_HZ && DCLK_HZ < 10_000_000 * m
                && 5 * m <= 31
        ).expect("no 5x PLL candidate");
        assert_eq!(GCLK_HZ_ASSUMING_N1, DCLK_HZ * 5);
        (m as u8, (m * 5) as u8)
    };
    let reference = include_str!("../../../dsview/icnd2069/ICND2069-抓包寄存器汇总.md");
    let mut count = 0;
    let mut differences = 0;
    for line in reference.lines() {
        let cells: Vec<_> = line.split('|').map(str::trim).collect();
        if cells.len() != 7 { continue; }
        let parsed: Vec<_> = cells[1..5].iter()
            .filter_map(|s| if s.len() == 2 { u8::from_str_radix(s, 16).ok() } else { None })
            .collect();
        if parsed.len() != 4 { continue; }
        let address = parsed[0];
        let expected = match address {
            0x02 => [(expected_rows - 1) as u8; 3],
            0x03 => [(SCAN_REPEATS - 1) as u8; 3],
            0x04 => [expected_m; 3],
            0x05 => [expected_p; 3],
            0x07 | 0x08 => [expected_pwm; 3],
            _ => [parsed[1], parsed[2], parsed[3]],
        };
        assert!(REGISTERS.contains(&(address, expected)), "capture mismatch at {address:02x}");
        count += 1;
        differences += usize::from(expected[0] != expected[1] || expected[0] != expected[2]);
    }
    assert_eq!(count, 45);
    assert_eq!(REGISTERS.len(), count);
    assert_eq!(differences, 8);
    let coupling = REGISTERS.iter().find(|(a, _)| *a == 0x0c).unwrap().1;
    assert!(coupling.iter().all(|&v| u32::from(v) < 4 * u32::from(expected_pwm)),
        "guide requires reg0C < reg07*4");
    // Register changes do not alter the external ROW period.
    assert_eq!(BASE_ROW_DCLKS, 60);
    if KEEP_SCAN_TIME {
        assert_eq!(SCAN_END, DATA_END);
    } else {
        assert_eq!(ROW_DCLKS, 60);
    }
}

fn check_transfer(words: &[u32], start: usize, expected: [u16; 3]) {
    for gpio in [0, 1, 2, 4, 5, 6, 8, 9, 10] {
        for chip in 0..CFG_CHAIN_COPIES {
            let value = words[start + chip * 16..start + (chip + 1) * 16]
                .iter().fold(0u16, |v, pins| (v << 1) | u16::from(pins & (1 << gpio) != 0));
            assert_eq!(value, expected[gpio % 4], "GP{gpio}, chip={chip}");
        }
    }
}

fn check_rotation() {
    let image = pattern_frame(Pattern::Normal, 0);
    let mut words = vec![0; FRAME_WORDS];
    let start = RT5990_CLEAR_DCLKS + 71 + 3 * CFG_PRE_CLR_DCLKS + 2 * (CFG_TRANSFER_DCLKS + 8);
    for index in 0..REGISTERS.len() + 2 {
        let expected = match index {
            0 => [0x0055; 3],
            1 => [0x0155; 3],
            i => {
                let (a, v) = REGISTERS[i - 2];
                v.map(|value| (a as u16) << 8 | value as u16)
            }
        };
        let value = rotation_word(index);
        assert_eq!(value, expected);
        encode_with_config(&mut words, &image, Some(value));
        check_transfer(&words, start, expected);
        let mut patched = vec![0; FRAME_WORDS];
        encode_with_config(&mut patched, &image, Some([0; 3]));
        update_rotation(&mut patched, value);
        assert_eq!(patched, words, "rotation patch must preserve timing and controls");
    }
}

fn check_rt5990(words: &[u32], full: bool) {
    let scan_start = FIRST_ROW - if full { 0 } else { CONFIG_SKIP_DCLKS };
    let mut selected = [true; RT5990_CLEAR_ROWS];
    let mut clear_edges = Vec::new();
    let mut total_edges = 0;
    for t in 1..words.len() {
        if words[t - 1] & LCK == 0 && words[t] & LCK != 0 {
            total_edges += 1;
            if t < scan_start - 8 {
                assert!(full, "ordinary frame must not clear the row chain");
                assert_eq!(words[t] & DIN, 0);
                selected.rotate_right(1);
                selected[0] = false;
                clear_edges.push(t);
                assert_ne!(words[t + 1] & LCK, 0);
                assert_eq!(words[t + 2] & LCK, 0);
            } else if t == scan_start - 4 {
                assert_ne!(words[t] & DIN, 0, "first scan must insert a row selection");
            }
        }
    }
    assert_eq!(total_edges, GROUPS * ROWS + if full { RT5990_CLEAR_ROWS } else { 0 });
    if full {
        assert_eq!(clear_edges.len(), RT5990_CLEAR_ROWS);
        assert!(selected.iter().all(|&on| !on), "all physical row bits must be cleared");
        assert_eq!(clear_edges[0], COMMON_PREFIX_WORDS + 2);
        assert!(clear_edges.windows(2).all(|pair| pair[1] - pair[0] == 4));
        let enable_start = COMMON_PREFIX_WORDS + RT5990_CLEAR_DCLKS;
        assert!(clear_edges[RT5990_CLEAR_ROWS - 1] + 2 < enable_start);
        for pins in &words[COMMON_PREFIX_WORDS..enable_start] {
            assert_eq!(pins & (SIN | LE | DIN), 0);
            assert_eq!(pins & BK != 0, BK_ACTIVE_HIGH, "blank while clearing");
        }
        assert!(words[enable_start..scan_start - 8].iter()
            .all(|pins| pins & (LCK | DIN) == 0), "keep row chain empty during configuration");
    }
}

fn check_frame(gray: u16, full: bool) {
    let mut head = vec![0; HEAD_WORDS];
    let mut tail = vec![0; TAIL_WORDS];
    let image = pattern_frame(Pattern::Normal, gray);
    let image = if CONTINUOUS_LATCH { target_frame(image) } else { image };
    encode_split(&mut head, &mut tail, &image);
    let offset = if full { COMMON_PREFIX_WORDS } else { CONFIG_END_WORDS };
    let mut words = if full && !ROW_ON_CFG {
        CONFIG_IDLE_PREFIX.to_vec()
    } else {
        head[..COMMON_PREFIX_WORDS].to_vec()
    };
    words.extend_from_slice(&head[offset..]);
    words.extend_from_slice(&tail);
    if full && !ROW_ON_CFG {
        words.iter_mut().for_each(|pins| *pins &= !ROW);
    }
    let skip = if full { 0 } else { CONFIG_SKIP_DCLKS };
    assert_eq!(words.len(), FRAME_DCLKS - skip);
    check_rt5990(&words, full);
    assert!(words.iter().all(|word| word >> 23 == 0),
        "single-PIO GPIO words must not contain obsolete rate/PC tags");
    let mut runs = [0; ROW_RUN_COUNT];
    encode_row_runs(&mut runs);
    let mut config_runs = vec![0; CONFIG_ROW_RUN_COUNT];
    encode_config_row_runs(&mut config_runs, &runs);
    if full && !ROW_ON_CFG {
        assert_eq!(config_runs.len(), runs.len());
        assert!(config_runs.iter().all(|run| run & 1 == 0),
            "ROW must stay low for the entire configuration-only frame");
        assert!(config_runs.iter().zip(&runs).all(|(a, b)| a >> 1 == b >> 1),
            "configuration ROW gating must preserve DMA run durations");
        runs.copy_from_slice(&config_runs);
    }
    runs[0] -= (skip as u32) << 1;
    assert_eq!(runs.iter().filter(|run| **run & 1 != 0).count(),
        if full && !ROW_ON_CFG { 0 } else { GROUPS * ROWS });
    assert_eq!(runs.iter().map(|r| (r >> 1) as usize + 1).sum::<usize>(), words.len());
    assert_eq!((runs[0] >> 1) as usize + 1, FIRST_ROW + 1 - skip);
    let mut position = 0;
    for (i, run) in runs.iter().enumerate() {
        if run & 1 != 0 {
            let expected = row_start(i / 2) + 1 - skip;
            assert_eq!(position, expected);
            // Serialized LCK and BK move with the independent ROW edge.
            assert!(sample(&words, expected - 4) & LCK != 0);
            assert!(sample(&words, expected - 2) & LCK == 0);
        }
        position += (run >> 1) as usize + 1;
    }
    let mut shift = [u128::MAX; 23];
    let mut width = 0;
    let mut commands = Vec::new();
    for t in 0..words.len() {
        let pins = sample(&words, t);
        if pins & LE == 0 && width != 0 {
            commands.push((t, width, shift));
            width = 0;
        }
        for gpio in 0..23 {
            shift[gpio] = shift[gpio].wrapping_shl(1) | u128::from(pins & (1 << gpio) != 0);
        }
        if pins & LE != 0 { width += 1; }
    }
    assert_eq!(width, 0);
    let vsync = !full || ROW_ON_CFG;
    assert_eq!(commands.iter().filter(|c| c.1 == VSYNC).count(), usize::from(vsync));
    if vsync {
        assert_eq!(commands[0].0, VSYNC_FALL);
        assert_eq!(commands[0].1, VSYNC);
    } else {
        assert!(words[..COMMON_PREFIX_WORDS].iter().all(|pins| pins & LE == 0));
    }
    assert!(commands.iter().all(|c| matches!(c.1, 1 | 3 | 5 | 11 | 14)),
        "unexpected command, including DIS_OP");
    assert!(words[VSYNC_FALL..COMMON_PREFIX_WORDS].iter().all(|pins| pins & LE == 0),
        "the former EN_OP slot must remain idle");
    let writes: Vec<_> = commands.iter().filter(|c| c.1 == WR_CFG).collect();
    assert_eq!(writes.len(), if full { CONFIG_WRITES } else { 0 });
    let expected_cfg: Vec<[u16; 3]> = [[0x00aa; 3], [0x01aa; 3]].into_iter()
        .chain(REGISTERS.iter().map(|&(a,v)| v.map(|value| (a as u16) << 8 | value as u16)))
        .chain([[0x0055; 3], [0x0155; 3]]).collect();
    for (index, (write, &expected)) in writes.iter().zip(&expected_cfg).enumerate() {
        let bits = 16 * CFG_CHAIN_COPIES;
        assert_eq!(write.0, RT5990_CLEAR_DCLKS + 71 + CFG_PRE_CLR_DCLKS + bits
            + index * (8 + CFG_PRE_CLR_DCLKS + bits));
        // Every write, including unlock/lock, gets its own extra zero bits.
        let clear_end = write.0 - bits;
        let clear_start = clear_end - CFG_PRE_CLR_DCLKS;
        for &pins in &words[clear_start - 8..clear_end] {
            assert_eq!(pins & (SIN | LE), 0, "WR_CFG gap and extra preclear {index}");
        }
        for gpio in [0, 1, 2, 4, 5, 6, 8, 9, 10] {
            for copy in 0..CFG_CHAIN_COPIES {
                assert_eq!((write.2[gpio] >> (16 * copy)) as u16, expected[gpio % 4],
                    "WR_CFG GP{gpio}, chip={copy}");
            }
        }
    }
    assert_eq!(commands.iter().filter(|c| c.1 == PRE_ACT).count(), usize::from(full));
    assert_eq!(commands.iter().filter(|c| c.1 == EN_OP).count(), usize::from(full));
    if full {
        let first = usize::from(vsync);
        assert_eq!((commands[first].0, commands[first].1), (RT5990_CLEAR_DCLKS + 45, EN_OP));
        assert_eq!((commands[first + 1].0, commands[first + 1].1), (RT5990_CLEAR_DCLKS + 62, PRE_ACT));
        let clear_start = commands[first + 1].0;
        let clear_end = clear_start + CFG_PRE_CLR_DCLKS;
        assert!(words[clear_start..clear_end].iter().all(|pins| pins & (SIN | LE) == 0));
        assert_eq!(writes[0].0 - CFG_TRANSFER_DCLKS, clear_end + 9,
            "first configuration word must follow its preclear and original gap");
        let post_clear_start = writes.last().unwrap().0;
        assert_eq!(DATA_START - post_clear_start, CFG_POST_CLR_DCLKS);
        assert!(words[post_clear_start..DATA_START].iter()
            .all(|pins| pins & (SIN | LE | ROW) == 0),
            "flush every shift register after lock and before any grayscale latch");
    }
    let data: Vec<_> = commands.iter().filter(|c| c.1 == 1).collect();
    assert_eq!(data.len(), ROWS * CHANNELS);
    for (i, command) in data.iter().enumerate() {
        let out = CHANNELS - 1 - i % CHANNELS;
        if !CONTINUOUS_LATCH {
            // Every slot must contain a complete transfer, including zero words.
            let first_fall = RT5990_CLEAR_DCLKS + 63 + CONFIG_WRITES * (8 + CFG_PRE_CLR_DCLKS + CFG_TRANSFER_DCLKS)
                + CFG_POST_CLR_DCLKS + TRANSFER_DCLKS - skip;
            assert_eq!(command.0,
                first_fall + i * TRANSFER_DCLKS + i / CHANNELS);
            if i % CHANNELS != 0 {
                assert_eq!(command.0 - data[i - 1].0, TRANSFER_DCLKS);
            }
        }
        for gpio in [0, 1, 2, 4, 5, 6, 8, 9, 10] {
            for copy in 0..CHAIN_COPIES {
                let expected = if GRAY_SIN & (1 << gpio) != 0 && out < LED_CHANNELS {
                    image[i / CHANNELS][copy * LED_CHANNELS + out]
                } else { 0 };
                assert_eq!((command.2[gpio] >> (16 * copy)) as u16, expected,
                    "DATA_LATCH GP{gpio}, chip={copy}");
            }
        }
    }
    let baseline = head.clone();
    update_white(&mut head, 127);
    let updated = pattern_frame(Pattern::Normal, 127);
    let updated = if CONTINUOUS_LATCH { target_frame(updated) } else { updated };
    let mut rebuilt = vec![0; HEAD_WORDS];
    encode_split(&mut rebuilt, &mut tail, &updated);
    assert_eq!(head, rebuilt, "gray patch must preserve RGB configuration");
    let data_start = RT5990_CLEAR_DCLKS + 63 + CONFIG_WRITES * (8 + CFG_PRE_CLR_DCLKS + CFG_TRANSFER_DCLKS)
        + CFG_POST_CLR_DCLKS;
    assert_eq!(head[..data_start], baseline[..data_start]);
}

fn main() {
    check_reference_registers();
    check_rotation();
    if !CONTINUOUS_LATCH {
        assert_eq!(GRAY_SWEEP, &[0, 15, 31, 63, 127, 255, 511, 1023, 2047, 4095, 8191]);
        for i in 0..GRAY_SWEEP.len() * 2 + 1 {
            let frame = sweep_frame(i);
            assert!(frame.iter().flatten().all(|&value| value == GRAY_SWEEP[i % 11]));
        }
    }
    // Simulate the actual two-bank schedule, including gray/config periods
    // that do not divide each other. Check 0, N, 2N without counter overflow.
    for interval in [1, 2, 7, 9, 10, 10000, u32::MAX] {
        for grays in [1, 4, 9, 11] {
            let mut cursor = scheduler::Cursor::new(interval, grays);
            let mut banks = [cursor.next(), cursor.next()];
            for frame in 0..100_003u64 {
                let bank = (frame % 2) as usize;
                assert_eq!(banks[bank], (frame % u64::from(interval) == 0, frame as usize % grays));
                banks[bank] = cursor.next();
            }
        }
    }
    for &gray in GRAY_SWEEP {
        check_frame(gray, true);
        check_frame(gray, false);
    }
    println!("PASS 9 GPIO RGB config/data, capture table (45 registers, 8 RGB differences), rotation/gray patch; two-bank schedule: intervals 1/2/7/9/10/10000/u32::MAX; gray continuity; 49/0 WR_CFG; {} latches; ROW alignment. Full/normal={} / {} clocks, OUT{}, chain={}, groups={}, continuous_latch={}, row input={} clocks, sweep={:?}",
        ROWS * CHANNELS,
        FRAME_DCLKS, NORMAL_FRAME_DCLKS, TARGET_CHANNEL, CHAIN_COPIES, GROUPS,
        CONTINUOUS_LATCH, DATA_ROW_DCLKS, GRAY_SWEEP);
}
