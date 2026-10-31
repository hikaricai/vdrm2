#![no_std]
#![no_main]

mod driver;
mod protocol;

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
// Keep both the requested full-write mode and exact NOVA rotation available.
const FULL_CONFIG_EVERY_FRAME: bool = false;
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
    let frame_len = protocol::encode_frame(frame, &protocol::demo_frame());
    assert_eq!(frame_len, protocol::FRAME_WORDS);
    assert!(protocol::DEMO_GRAY <= protocol::GRAY_CAPACITY);
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
    rtt_target::rprintln!("pattern=NOVA, OUT15=0, OUT14..0={}", protocol::DEMO_GRAY);
    if SOFT_RESET_ON_START {
        protocol::startup_reset(frame);
        driver.send(&frame[..frame_len], row_runs).await;
        protocol::encode_frame(frame, &protocol::demo_frame());
    }
    if !FULL_CONFIG_EVERY_FRAME {
        driver.send(&frame[..frame_len], row_runs).await;
        protocol::encode_with_config(
            frame,
            &protocol::demo_frame(),
            Some(protocol::rotation_word(0)),
        );
    }
    rtt_target::rprintln!(
        "WR_CFG/frame={}, soft_reset={}; exact captured ROW schedule",
        if FULL_CONFIG_EVERY_FRAME {
            protocol::CONFIG_WRITES
        } else {
            5
        },
        SOFT_RESET_ON_START
    );

    let mut frames = 0u32;
    let mut last = Instant::now();
    loop {
        // Repeat uniform data. Driver keeps DCLK running across DMA boundaries.
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
