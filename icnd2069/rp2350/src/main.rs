#![no_std]
#![no_main]

mod driver;
mod dclk_program;
mod protocol;
mod scheduler;
include!(concat!(env!("OUT_DIR"), "/pattern_config.rs"));

use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_time::Timer;
use panic_rtt_target as _;
use static_cell::ConstStaticCell;

// Both latch modes use one static gradient, shared by the two DMA banks.
static HEADS: ConstStaticCell<[u32; protocol::HEAD_WORDS]> =
    ConstStaticCell::new([0; protocol::HEAD_WORDS]);
static TAIL: ConstStaticCell<[u32; protocol::TAIL_WORDS]> =
    ConstStaticCell::new([0; protocol::TAIL_WORDS]);
static ROW_RUNS: ConstStaticCell<[u32; protocol::ROW_RUN_COUNT]> =
    ConstStaticCell::new([0; protocol::ROW_RUN_COUNT]);
static DESCRIPTORS: ConstStaticCell<[u32; 24]> = ConstStaticCell::new([0; 24]);
static ROW_DESCRIPTORS: ConstStaticCell<[u32; 16]> = ConstStaticCell::new([0; 16]);
static FIRST_RUNS: ConstStaticCell<[u32; 2]> = ConstStaticCell::new([0; 2]);
static RESTART: ConstStaticCell<[u32; 2]> = ConstStaticCell::new([0; 2]);

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    rtt_target::rtt_init_print!(rtt_target::ChannelMode::NoBlockSkip, 2048);
    let mut config = embassy_rp::config::Config::default();
    config.clocks.xosc.as_mut().unwrap().sys_pll =
        Some(embassy_rp::clocks::PllConfig {
            refdiv: 1,
            fbdiv: protocol::SYS_PLL_FBDIV,
            post_div1: protocol::SYS_PLL_POST_DIV1,
            post_div2: protocol::SYS_PLL_POST_DIV2,
        });
    let p = embassy_rp::init(config);
    assert_eq!(embassy_rp::clocks::clk_sys_freq(), protocol::SYS_HZ);
    let mut led = Output::new(p.PIN_25, Level::Low);
    let mut driver = driver::Driver::new(
        p.PIO0,
        p.DMA_CH0,
        p.DMA_CH1,
        driver::Pins {
            // 开发板接线见 hardware/dev_sch.png，编号为 GPIO。
            r1: p.PIN_5,
            g1: p.PIN_4,
            b1: p.PIN_3,
            r2: p.PIN_11,
            g2: p.PIN_10,
            b2: p.PIN_9,
            r3: p.PIN_2,
            g3: p.PIN_1,
            b3: p.PIN_0,
            r4: p.PIN_8,
            g4: p.PIN_7,
            b4: p.PIN_6,
            // icnd2069
            dclk: p.PIN_19,
            le: p.PIN_18,
            row: p.PIN_13,
            // rt5990
            lck: p.PIN_16,
            lbk: p.PIN_15,
            ldata: p.PIN_17,
        },
    );
    let heads = HEADS.take();
    let tail = TAIL.take();
    let row_runs = ROW_RUNS.take();
    protocol::encode_row_runs(row_runs);
    let row_pulses = row_runs.iter().filter(|run| **run & 1 != 0).count();
    assert_eq!(row_pulses, protocol::ROWS * protocol::SCAN_REPEATS);
    let image = protocol::pattern_frame_range(PATTERN, GRAY_RANGE);
    protocol::encode_split(heads, tail, &image);
    rtt_target::rprintln!(
        "ICND2069: chain={}, {}x{}, groups={}, DCLK={} Hz, frame={} clocks",
        protocol::CHAIN_COPIES, protocol::TOTAL_CHANNELS, protocol::ROWS, protocol::GROUPS,
        protocol::DCLK_HZ, protocol::FRAME_DCLKS,
    );
    rtt_target::rprintln!("wave RAM={} bytes", heads.len() * 4 + tail.len() * 4 + row_runs.len() * 4);
    rtt_target::rprintln!("configuration DCLK={} Hz, data/scan DCLK={} Hz",
        protocol::CFG_DCLK_HZ, protocol::DCLK_HZ);
    rtt_target::rprintln!("RGB1=GP5/4/3, RGB2=GP11/10/9, RGB3=GP2/1/0, RGB4=GP8/7/6 (RGB-specific config, mirrored gray)");
    rtt_target::rprintln!("ICND CLK/LAT/ROW=GP19/18/13, RT LCLK/LBK/LDATA=GP16/15/17");
    rtt_target::rprintln!(
        "ROW pulses/frame={} ({} rows x {} sweeps), reg03=0x{:02x}",
        row_pulses, protocol::ROWS, protocol::SCAN_REPEATS,
        protocol::GROUPS - 1,
    );
    rtt_target::rprintln!(
        "continuous_latch={}, fixed column={}, repeated LAT-low gap={} clocks, base row input={} clocks",
        protocol::CONTINUOUS_LATCH, protocol::TARGET_CHANNEL, protocol::REPEAT_LATCH_GAP,
        protocol::DATA_ROW_DCLKS,
    );
    if protocol::CONTINUOUS_LATCH && protocol::DIAGONAL_LATCH {
        rtt_target::rprintln!("diagonal latch: column=row%{}, chip=column/16 (0 nearest MCU), OUT=column%16",
            protocol::TOTAL_CHANNELS);
    }
    if protocol::CONTINUOUS_LATCH {
        rtt_target::rprintln!(
            "8-bit latch gray: 8+16*chip clocks, clear=16*(chains-chip), total={} clocks/row",
            protocol::TRANSFER_DCLKS + 8,
        );
    }
    rtt_target::rprintln!(
        "config.json: image={}, gray range={:?}, config interval={} ({} writes)",
        PATTERN_NAME, GRAY_RANGE, protocol::CONFIG_INTERVAL,
        protocol::CONFIG_WRITES,
    );
    rtt_target::rprintln!("normal frame={} clocks, first frame has full configuration",
        protocol::NORMAL_FRAME_DCLKS);
    rtt_target::rprintln!("data/ROW overlap: first ROW at normal clock {}, VSYNC waits for both",
        protocol::FIRST_ROW - protocol::CONFIG_SKIP_DCLKS);
    rtt_target::rprintln!(
        "keep_scan_time={}, scan={} DCLK, ROW period={}..{} DCLK (base={}), scan/data end={}/{}",
        protocol::KEEP_SCAN_TIME, protocol::SCAN_DCLKS,
        protocol::ROW_DCLKS,
        protocol::SCAN_DCLKS.div_ceil(protocol::ROWS * protocol::GROUPS),
        protocol::BASE_ROW_DCLKS,
        protocol::SCAN_END - protocol::CONFIG_SKIP_DCLKS,
        protocol::DATA_END - protocol::CONFIG_SKIP_DCLKS,
    );
    rtt_target::rprintln!("system clock={} Hz, PIO cycles/DCLK={}, divider={}+0/256",
        protocol::SYS_HZ, protocol::PIO_CYCLES_PER_DCLK, protocol::PIO_CLKDIV_INT);
    rtt_target::rprintln!(
        "ICND2069 PLL: M={}, P={}, reg06=0x{:02x}, GCLK={} Hz (assuming N=1), ROW={} DCLK",
        protocol::PLL_M, protocol::PLL_P, protocol::PLL_N,
        protocol::GCLK_HZ_ASSUMING_N1, protocol::ROW_DCLKS,
    );
    rtt_target::rprintln!(
        "PWM: reg07=reg08={}, nominal accelerate=256/{}x, display={} GCLK/group, theoretical gray capacity={}",
        protocol::PWM_WIDTH, protocol::PWM_WIDTH,
        4 * u32::from(protocol::PWM_WIDTH),
        protocol::GROUPS * 4 * usize::from(protocol::PWM_WIDTH),
    );
    rtt_target::rprintln!("ideal PWM window={} ns/group (assuming N=1)",
        4_000_000_000u64 * u64::from(protocol::PWM_WIDTH)
            / u64::from(protocol::GCLK_HZ_ASSUMING_N1));
    if protocol::GCLK_HZ_ASSUMING_N1 == 100_000_000 {
        rtt_target::rprintln!("100 MHz PLL boundary experiment: guide specifies strictly <100 MHz");
    } else if protocol::GCLK_HZ_ASSUMING_N1 > 100_000_000 {
        rtt_target::rprintln!("PLL experiment: calculated frequency exceeds guide's <100 MHz limit");
    }
    // Ideal frame rate from PIO cycles, including slow configuration. Split the
    // division before scaling to avoid overflow at config_interval=u32::MAX.
    let fps_x100 = |frames: u64, clocks: u64| {
        let ticks = u64::from(protocol::SYS_HZ / protocol::PIO_CLKDIV_INT) * frames;
        (ticks / clocks) * 100 + ((ticks % clocks) * 100 + clocks / 2) / clocks
    };
    let normal_clocks = protocol::NORMAL_FRAME_PIO_CYCLES;
    let config_clocks = protocol::CONFIG_FRAME_PIO_CYCLES;
    let interval = u64::from(protocol::CONFIG_INTERVAL);
    let normal_fps = fps_x100(1, normal_clocks);
    let config_fps = fps_x100(1, config_clocks);
    let average_fps = fps_x100(interval, (interval - 1) * normal_clocks + config_clocks);
    rtt_target::rprintln!(
        "theoretical FPS: avg={}.{:02}, normal={}.{:02}, config={}.{:02}",
        average_fps / 100, average_fps % 100,
        normal_fps / 100, normal_fps % 100,
        config_fps / 100, config_fps % 100,
    );
    if protocol::DCLK_HZ < 4_000_000 {
        rtt_target::rprintln!("1/3 MHz timing experiment: outside documented ICND2069 PLL input range");
    }
    Timer::after_millis(100).await;
    driver.start_scheduled(
        heads, protocol::HEAD_WORDS, tail, row_runs, DESCRIPTORS.take(),
        ROW_DESCRIPTORS.take(), FIRST_RUNS.take(), RESTART.take(),
        p.DMA_CH2, p.DMA_CH3, p.DMA_CH4, p.DMA_CH5,
    );
    let mut dclk_stall_windows = 0u32;
    loop {
        // DMA_IRQ_1 prepares the inactive descriptor bank each frame.
        Timer::after_secs(1).await;
        if driver.take_dclk_stall() {
            dclk_stall_windows = dclk_stall_windows.saturating_add(1);
            rtt_target::rprintln!(
                "WARNING: DCLK TXSTALL detected, affected polling windows={}; clock period was extended",
                dclk_stall_windows,
            );
        }
        led.toggle();
    }
}
