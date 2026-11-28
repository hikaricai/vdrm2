#![no_std]
#![no_main]

mod driver;
mod dclk_program;
mod protocol;
mod scheduler;
include!(concat!(env!("OUT_DIR"), "/pattern_config.rs"));

use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_time::{Instant, Timer};
use panic_rtt_target as _;
use static_cell::ConstStaticCell;

static CONFIG_FRAME: ConstStaticCell<[u32; protocol::CONFIG_LAYOUT.stored()]> =
    ConstStaticCell::new([0; protocol::CONFIG_LAYOUT.stored()]);
static NORMAL_FRAME: ConstStaticCell<[u32; protocol::NORMAL_LAYOUT.stored()]> =
    ConstStaticCell::new([0; protocol::NORMAL_LAYOUT.stored()]);

/// Hundredths of FPS, without floating point or scaling the full numerator.
fn fps_x100(clock_hz: u64, frames: u64, ticks: u64) -> u64 {
    let count = clock_hz * frames;
    (count / ticks) * 100 + ((count % ticks) * 100 + ticks / 2) / ticks
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    rtt_target::rtt_init_print!(rtt_target::ChannelMode::NoBlockSkip, 2048);
    let mut config = embassy_rp::config::Config::default();
    config.clocks.xosc.as_mut().unwrap().sys_pll = Some(embassy_rp::clocks::PllConfig {
        refdiv: 1,
        fbdiv: protocol::SYS_PLL_FBDIV,
        post_div1: protocol::SYS_PLL_POST_DIV1,
        post_div2: protocol::SYS_PLL_POST_DIV2,
    });
    let p = embassy_rp::init(config);
    assert_eq!(embassy_rp::clocks::clk_sys_freq(), protocol::SYS_HZ);
    let mut led = Output::new(p.PIN_25, Level::Low);
    #[cfg(icnd2069_hdl)]
    let pins = driver::Pins {
        r1: p.PIN_0, g1: p.PIN_1, b1: p.PIN_2, sel1: p.PIN_3,
        r2: p.PIN_4, g2: p.PIN_5, b2: p.PIN_6, sel2: p.PIN_7,
        r3: p.PIN_8, g3: p.PIN_9, b3: p.PIN_10, sel3: p.PIN_11,
        lat1: p.PIN_12, lat2: p.PIN_13, lat3: p.PIN_14,
        dclk: p.PIN_15, dclk2: p.PIN_17, row: p.PIN_22,
        lck: p.PIN_16, lbk: p.PIN_18, ldata: p.PIN_20,
    };
    #[cfg(icnd2069_dev)]
    let pins = driver::Pins {
        sin: p.PIN_13, le: p.PIN_10, dclk: p.PIN_15, row: p.PIN_22,
        lck: p.PIN_16, lbk: p.PIN_18, ldata: p.PIN_20,
    };
    #[cfg(icnd2069_direct)]
    let pins = driver::Pins {
        r1: p.PIN_5, g1: p.PIN_4, b1: p.PIN_3,
        r2: p.PIN_11, g2: p.PIN_10, b2: p.PIN_9,
        r3: p.PIN_2, g3: p.PIN_1, b3: p.PIN_0,
        r4: p.PIN_8, g4: p.PIN_7, b4: p.PIN_6,
        dclk: p.PIN_19, le: p.PIN_18, row: p.PIN_13,
        lck: p.PIN_16, lbk: p.PIN_15, ldata: p.PIN_17,
    };
    let mut driver = driver::Driver::new(p.PIO0, p.DMA_CH0, p.DMA_CH1, pins);
    let configuration = CONFIG_FRAME.take();
    let normal = NORMAL_FRAME.take();
    protocol::prepare_stored_frames(configuration, normal, &protocol::pattern_frame_range(PATTERN, GRAY_RANGE));
    let slow = driver::Transfer::new(configuration, protocol::CONFIG_LAYOUT, 0..protocol::CONFIG_END_WORDS);
    let fast = driver::Transfer::new(configuration, protocol::CONFIG_LAYOUT,
        protocol::CONFIG_END_WORDS..protocol::CONFIG_LAYOUT.total);
    let display = driver::Transfer::new(normal, protocol::NORMAL_LAYOUT, 0..protocol::NORMAL_FRAME_DCLKS);
    rtt_target::rprintln!("ICND2069 board={}: PIO SM0 + DMA0/1, SM1 clock copy={}; continuous idle-zero clocks",
        protocol::BOARD, protocol::HDL_BOARD);
    rtt_target::rprintln!("chains={}, cfg_chains={}, lines={}, scan_repeats={}, row_on_cfg={}",
        protocol::CHAIN_COPIES, protocol::CFG_CHAIN_COPIES, protocol::ROWS,
        protocol::SCAN_REPEATS, protocol::ROW_ON_CFG);
    rtt_target::rprintln!("disable_rt5990={}; BK blank level={}, DIN/LCK held LOW when disabled",
        protocol::DISABLE_RT5990, u8::from(protocol::BK_ACTIVE_HIGH));
    rtt_target::rprintln!("DCLK={} Hz, cfg DCLK={} Hz; normal/config dividers={}/256, {}/256",
        protocol::DCLK_HZ, protocol::CFG_DCLK_HZ,
        protocol::PIO_CLKDIV_INT << 8, protocol::CFG_CLKDIV_BITS);
    rtt_target::rprintln!("pattern={}, color={}, gray={:?}, config_interval={}",
        PATTERN_NAME, protocol::COLOR, GRAY_RANGE, protocol::CONFIG_INTERVAL);
    rtt_target::rprintln!("normal={} data clocks, ROW={} pulses at {} minimum clocks/row; idle-zero clocks between frames",
        protocol::NORMAL_FRAME_DCLKS, protocol::ROWS * protocol::SCAN_REPEATS,
        protocol::NORMAL_SCAN_DCLKS / (protocol::ROWS * protocol::SCAN_REPEATS));
    if protocol::TIMING_ENABLED {
        rtt_target::rprintln!("timing relative to VSYNC: EN={}, gray={}, ROW={}; full-write gaps={}/{}, minimum tail={}",
            protocol::TIMING_EN_START, protocol::TIMING_GRAY_START, protocol::TIMING_ROW_START,
            protocol::TIMING_GRAY_WORD_GAP, protocol::TIMING_GRAY_ROW_GAP, protocol::TIMING_FRAME_TAIL);
    }
    rtt_target::rprintln!("configuration={} slow clocks; WR_CFG={}, preclear={} clocks/write, RT5990 clear={} rows",
        protocol::CONFIG_END_WORDS, protocol::CONFIG_WRITES,
        protocol::CFG_PRE_CLR_DCLKS, protocol::RT5990_CLEAR_ROWS);
    rtt_target::rprintln!("after configuration: {} slow zero clocks, {} fast zero clocks, complete grayscale frame, then VSYNC; clear LE/ROW LOW",
        protocol::CFG_POST_CLR_DCLKS, protocol::CFG_POST_CLR_DCLKS);
    if protocol::DEV_BOARD {
        rtt_target::rprintln!("GPIO SIN/LE/CLK/ROW=13/10/15/22; RT LCK/BK/DIN=16/18/20; single R data line, no CPLD");
    } else if protocol::DIRECT_BOARD {
        rtt_target::rprintln!("GPIO RGB1=5/4/3 RGB2=11/10/9 RGB3=2/1/0 RGB4=8/7/6");
        rtt_target::rprintln!("GPIO CLK/LE/ROW=19/18/13; RT LCK/BK/DIN=16/15/17; direct, no CPLD");
    } else {
        rtt_target::rprintln!("CPLD configuration/gray addresses={:?}; both use the same destination",
            protocol::CPLD_ADDRESSES);
        rtt_target::rprintln!("GPIO CLK/CLK2/LAT1/LAT2/LAT3/ROW=15/17/12/13/14/22; RT LCK/BK/DIN=16/18/20");
    }
    rtt_target::rprintln!("LED channels/chip={}; stored waveform RAM={} bytes",
        protocol::LED_CHANNELS, (configuration.len() + normal.len()) * 4);
    let fast_cycles = u64::from(protocol::SYS_HZ / protocol::DCLK_HZ);
    let normal_cycles = protocol::NORMAL_FRAME_DCLKS as u64 * fast_cycles;
    let config_cycles = protocol::CONFIG_END_WORDS as u64
        * u64::from(protocol::SYS_HZ / protocol::CFG_DCLK_HZ)
        + (protocol::CONFIG_LAYOUT.total - protocol::CONFIG_END_WORDS) as u64 * fast_cycles;
    let interval = u64::from(protocol::CONFIG_INTERVAL);
    let normal_fps = fps_x100(u64::from(protocol::SYS_HZ), 1, normal_cycles);
    let config_fps = fps_x100(u64::from(protocol::SYS_HZ), 1, config_cycles);
    let average_fps = fps_x100(u64::from(protocol::SYS_HZ), interval,
        (interval - 1) * normal_cycles + config_cycles);
    rtt_target::rprintln!(
        "theoretical FPS: avg={}.{:02}, normal={}.{:02}, config={}.{:02}",
        average_fps / 100, average_fps % 100,
        normal_fps / 100, normal_fps % 100,
        config_fps / 100, config_fps % 100,
    );
    rtt_target::rprintln!("normal FPS excludes configuration frames; theoretical FPS excludes software gaps");
    if !protocol::RAW_REGS.is_empty() {
        rtt_target::rprintln!("raw regs: 45 RGB register values from config.json; internal GCLK not inferred");
    } else if protocol::GCLK_HZ_ASSUMING_N1 >= 100_000_000 {
        rtt_target::rprintln!("PLL experiment: GCLK={} Hz exceeds guide's strict <100 MHz limit",
            protocol::GCLK_HZ_ASSUMING_N1);
    }
    Timer::after_millis(100).await;
    let mut cursor = scheduler::Cursor::new(protocol::CONFIG_INTERVAL, 1);
    let mut frames = 0u32;
    let mut report = Instant::now();
    let mut report_frames = 0u32;
    let mut previous_start = report;
    let mut previous_normal = false;
    let mut normal_frames = 0u64;
    let mut normal_ticks = 0u64;
    loop {
        // One timestamp per frame. Account the preceding complete frame,
        // including software gaps, only when it was not a configuration frame.
        let start = Instant::now();
        if previous_normal {
            normal_frames += 1;
            normal_ticks += start.duration_since(previous_start).as_ticks();
        }
        previous_start = start;
        let report_elapsed = start.duration_since(report);
        if report_elapsed.as_secs() >= 5 {
            #[cfg(feature = "dma-diagnostics")]
            driver::diagnose_rtt_mode();
            let average_fps = fps_x100(embassy_time::TICK_HZ,
                u64::from(frames.wrapping_sub(report_frames)), report_elapsed.as_ticks());
            if normal_frames != 0 && normal_ticks != 0 {
                let normal_fps = fps_x100(embassy_time::TICK_HZ, normal_frames, normal_ticks);
                rtt_target::rprintln!(
                    "measured FPS: normal={}.{:02} (config frames excluded), avg={}.{:02}, frames={}",
                    normal_fps / 100, normal_fps % 100,
                    average_fps / 100, average_fps % 100, frames,
                );
            } else {
                rtt_target::rprintln!(
                    "measured FPS: normal=N/A (no normal frames), avg={}.{:02}, frames={}",
                    average_fps / 100, average_fps % 100, frames,
                );
            }
            normal_frames = 0;
            normal_ticks = 0;
            report_frames = frames;
            report = start;
            led.toggle();
        }
        let configure = cursor.next().0;
        previous_normal = !configure;
        if configure {
            driver.send(&slow, true);
            driver.send(&fast, false);
        } else {
            driver.send(&display, false);
        }
        frames = frames.wrapping_add(1);
    }
}
