#![no_std]
#![no_main]

mod driver;
mod protocol;
include!(concat!(env!("OUT_DIR"), "/pattern_config.rs"));

use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_time::{Instant, Timer};
use panic_rtt_target as _;
use static_cell::ConstStaticCell;

// Large waveforms live in BSS, never on the stack or in an executor task frame.
static FRAME: ConstStaticCell<[u32; protocol::FRAME_WORDS]> =
    ConstStaticCell::new([0; protocol::FRAME_WORDS]);
static ROW_RUNS: ConstStaticCell<[u32; protocol::ROW_RUN_COUNT]> =
    ConstStaticCell::new([0; protocol::ROW_RUN_COUNT]);
// Each frame sends the complete register batch once, then all 16x16 gray words.
const FULL_CONFIG_EVERY_FRAME: bool = true;
const SOFT_RESET_ON_START: bool = false; // Tested separately; did not restore OUT1.

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    rtt_target::rtt_init_print!(rtt_target::ChannelMode::NoBlockSkip, 2048);
    let p = embassy_rp::init(Default::default());
    let mut led = Output::new(p.PIN_25, Level::Low);
    let mut driver = driver::Driver::new(
        p.PIO0,
        p.DMA_CH0,
        p.DMA_CH1,
        driver::Pins {
            le: p.PIN_10,
            sin: p.PIN_13,
            dclk: p.PIN_15,
            lck: p.PIN_16,
            bk: p.PIN_18,
            din: p.PIN_20,
            row: p.PIN_22,
        },
    );
    Timer::after_millis(100).await;

    let frame = FRAME.take();
    let row_runs = ROW_RUNS.take();
    protocol::encode_row_runs(row_runs);
    let image = protocol::pattern_frame(PATTERN, GRAY);
    let frame_len = protocol::encode_frame(frame, &image);
    assert_eq!(frame_len, protocol::FRAME_WORDS);
    assert!(GRAY <= protocol::MAX_GRAY_INPUT);
    rtt_target::rprintln!(
        "ICND2069: PIO0/SM0, DCLK={} Hz, {}x{}, {} groups, {} clocks/row",
        protocol::DCLK_HZ,
        protocol::CHANNELS,
        protocol::ROWS,
        protocol::GROUPS,
        protocol::ROW_DCLKS
    );
    rtt_target::rprintln!(
        "frame={} bytes; startup {} WR_CFG; reference PLL 04/05/06=04/0A/01",
        frame_len * 4,
        protocol::CONFIG_WRITES
    );
    rtt_target::rprintln!(
        "pattern={}, gray={}, input_limit={}, ICND2069 chain={}",
        PATTERN_NAME, GRAY, protocol::MAX_GRAY_INPUT, protocol::CHAIN_COPIES
    );
    if SOFT_RESET_ON_START {
        protocol::startup_reset(frame);
        driver.send(&frame[..frame_len], row_runs).await;
        protocol::encode_frame(frame, &image);
    }
    if !FULL_CONFIG_EVERY_FRAME {
        driver.send(&frame[..frame_len], row_runs).await;
        protocol::encode_with_config(
            frame,
            &image,
            Some(protocol::rotation_word(0)),
        );
    }
    rtt_target::rprintln!(
        "WR_CFG/frame={}, soft_reset={}; {} scans, saved local ROW cadence",
        if FULL_CONFIG_EVERY_FRAME {
            protocol::CONFIG_WRITES
        } else {
            5
        },
        SOFT_RESET_ON_START,
        protocol::ROWS
    );

    let mut frames = 0u32;
    let mut last = Instant::now();
    loop {
        // Repeat the selected image. DCLK runs across DMA boundaries.
        driver.send(&frame[..frame_len], row_runs).await;
        frames = frames.wrapping_add(1);
        if !FULL_CONFIG_EVERY_FRAME {
            protocol::update_rotation(frame, protocol::rotation_word(frames as usize));
        }
        if frames % 60 == 0 {
            let now = Instant::now();
            rtt_target::rprintln!("60 frames / {} us", (now - last).as_micros());
            last = now;
            led.toggle();
        }
    }
}
