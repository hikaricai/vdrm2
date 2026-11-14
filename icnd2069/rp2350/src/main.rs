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

static CONFIG_FRAME: ConstStaticCell<[u32; protocol::FRAME_WORDS]> =
    ConstStaticCell::new([0; protocol::FRAME_WORDS]);
static NORMAL_FRAME: ConstStaticCell<[u32; protocol::NORMAL_FRAME_DCLKS]> =
    ConstStaticCell::new([0; protocol::NORMAL_FRAME_DCLKS]);

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
            r1: p.PIN_5, g1: p.PIN_4, b1: p.PIN_3,
            r2: p.PIN_11, g2: p.PIN_10, b2: p.PIN_9,
            r3: p.PIN_2, g3: p.PIN_1, b3: p.PIN_0,
            r4: p.PIN_8, g4: p.PIN_7, b4: p.PIN_6,
            dclk: p.PIN_19, le: p.PIN_18, row: p.PIN_13,
            lck: p.PIN_16, lbk: p.PIN_15, ldata: p.PIN_17,
        },
    );
    let configuration = CONFIG_FRAME.take();
    let normal = NORMAL_FRAME.take();
    {
        // Keep the temporary image out of the async task arena.
        let image = protocol::pattern_frame_range(PATTERN, GRAY_RANGE);
        protocol::encode_frame(configuration, &image);
    }
    normal[..protocol::COMMON_PREFIX_WORDS]
        .copy_from_slice(&configuration[..protocol::COMMON_PREFIX_WORDS]);
    normal[protocol::COMMON_PREFIX_WORDS..]
        .copy_from_slice(&configuration[protocol::CONFIG_END_WORDS..]);
    if !protocol::ROW_ON_CFG {
        configuration[..protocol::COMMON_PREFIX_WORDS].fill(protocol::PARK);
        for word in configuration.iter_mut() {
            *word &= !protocol::ROW;
        }
    }
    rtt_target::rprintln!("ICND2069: single PIO SM0 + DMA0; stop LOW between blocks, change divider while stopped");
    rtt_target::rprintln!("chains={}, cfg_chains={}, lines={}, scan_repeats={}, row_on_cfg={}",
        protocol::CHAIN_COPIES, protocol::CFG_CHAIN_COPIES, protocol::ROWS,
        protocol::SCAN_REPEATS, protocol::ROW_ON_CFG);
    rtt_target::rprintln!("DCLK={} Hz, cfg DCLK={} Hz; normal/config dividers={}/256, {}/256",
        protocol::DCLK_HZ, protocol::CFG_DCLK_HZ,
        protocol::PIO_CLKDIV_INT << 8, protocol::CFG_CLKDIV_BITS);
    rtt_target::rprintln!("pattern={}, color={}, gray={:?}, config_interval={}",
        PATTERN_NAME, protocol::COLOR, GRAY_RANGE, protocol::CONFIG_INTERVAL);
    rtt_target::rprintln!("normal={} clocks, ROW={} pulses at {} clocks/row; software gaps between blocks",
        normal.len(), protocol::ROWS * protocol::SCAN_REPEATS, protocol::ROW_DCLKS);
    rtt_target::rprintln!("configuration={} slow clocks; WR_CFG={}, preclear={} clocks/write, RT5990 clear={} rows",
        protocol::CONFIG_END_WORDS, protocol::CONFIG_WRITES,
        protocol::CFG_PRE_CLR_DCLKS, protocol::RT5990_CLEAR_ROWS);
    rtt_target::rprintln!("GPIO DCLK/LE/ROW=19/18/13; RT LCK/BK/DIN=16/15/17; wave RAM={} bytes",
        (configuration.len() + normal.len()) * 4);
    if protocol::GCLK_HZ_ASSUMING_N1 >= 100_000_000 {
        rtt_target::rprintln!("PLL experiment: GCLK={} Hz exceeds guide's strict <100 MHz limit",
            protocol::GCLK_HZ_ASSUMING_N1);
    }
    Timer::after_millis(100).await;
    let mut cursor = scheduler::Cursor::new(protocol::CONFIG_INTERVAL, 1);
    let mut frames = 0u32;
    let mut report = Instant::now();
    loop {
        if cursor.next().0 {
            driver.send(&configuration[..protocol::CONFIG_END_WORDS], true).await;
            driver.send(&configuration[protocol::CONFIG_END_WORDS..], false).await;
        } else {
            driver.send(normal, false).await;
        }
        frames = frames.wrapping_add(1);
        if report.elapsed().as_secs() >= 5 {
            rtt_target::rprintln!("running: frames={}, single DMA; PIO stopped LOW at block boundaries", frames);
            report = Instant::now();
            led.toggle();
        }
    }
}
