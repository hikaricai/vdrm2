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

static CONFIG_FRAME: ConstStaticCell<[u32; protocol::FRAME_WORDS + protocol::CFG_POST_CLR_DCLKS]> =
    ConstStaticCell::new([0; protocol::FRAME_WORDS + protocol::CFG_POST_CLR_DCLKS]);
static NORMAL_FRAME: ConstStaticCell<[u32; protocol::NORMAL_FRAME_DCLKS]> =
    ConstStaticCell::new([0; protocol::NORMAL_FRAME_DCLKS]);

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
    let mut driver = driver::Driver::new(
        p.PIO0, p.DMA_CH0,
        driver::Pins {
            r1: p.PIN_0, g1: p.PIN_1, b1: p.PIN_2, sel1: p.PIN_3,
            r2: p.PIN_4, g2: p.PIN_5, b2: p.PIN_6, sel2: p.PIN_7,
            r3: p.PIN_8, g3: p.PIN_9, b3: p.PIN_10, sel3: p.PIN_11,
            lat1: p.PIN_12, lat2: p.PIN_13, lat3: p.PIN_14,
            dclk: p.PIN_15, dclk2: p.PIN_17, row: p.PIN_22,
            lck: p.PIN_16, lbk: p.PIN_18, ldata: p.PIN_20,
        },
    );
    let configuration = CONFIG_FRAME.take();
    let normal = NORMAL_FRAME.take();
    {
        // Keep the temporary image out of the async task arena.
        let image = protocol::pattern_frame_range(PATTERN, GRAY_RANGE);
        protocol::prepare_frames(configuration, normal, &image);
    }
    rtt_target::rprintln!("ICND2069: PIO SM0 + DMA0, synchronized SM1 DCLK copy; continuous idle-zero clocks, stop LOW for divider/drive changes");
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
    rtt_target::rprintln!("normal={} data clocks, ROW={} pulses at {} clocks/row; idle-zero clocks between frames",
        normal.len(), protocol::ROWS * protocol::SCAN_REPEATS, protocol::ROW_DCLKS);
    rtt_target::rprintln!("configuration={} slow clocks; WR_CFG={}, preclear={} clocks/write, RT5990 clear={} rows",
        protocol::CONFIG_END_WORDS, protocol::CONFIG_WRITES,
        protocol::CFG_PRE_CLR_DCLKS, protocol::RT5990_CLEAR_ROWS);
    rtt_target::rprintln!("after configuration: {} slow zero clocks, {} fast zero clocks, complete grayscale frame, then VSYNC; clear LE/ROW LOW",
        protocol::CFG_POST_CLR_DCLKS, protocol::CFG_POST_CLR_DCLKS);
    rtt_target::rprintln!("CPLD configuration/gray addresses={:?}; both use the same destination",
        protocol::CPLD_ADDRESSES);
    rtt_target::rprintln!("GPIO CLK/CLK2/LAT1/LAT2/LAT3/ROW=15/17/12/13/14/22; RT LCK/BK/DIN=16/18/20; wave RAM={} bytes",
        (configuration.len() + normal.len()) * 4);
    let fast_cycles = u64::from(protocol::SYS_HZ / protocol::DCLK_HZ);
    let normal_cycles = normal.len() as u64 * fast_cycles;
    let config_cycles = protocol::CONFIG_END_WORDS as u64
        * u64::from(protocol::SYS_HZ / protocol::CFG_DCLK_HZ)
        + (configuration.len() - protocol::CONFIG_END_WORDS) as u64 * fast_cycles;
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
    if protocol::GCLK_HZ_ASSUMING_N1 >= 100_000_000 {
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
            driver.send(&configuration[..protocol::CONFIG_END_WORDS], true).await;
            driver.send(&configuration[protocol::CONFIG_END_WORDS..], false).await;
        } else {
            driver.send(normal, false).await;
        }
        frames = frames.wrapping_add(1);
    }
}
