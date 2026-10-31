#![no_std]
#![no_main]

mod driver;
mod protocol;
include!(concat!(env!("OUT_DIR"), "/pattern_config.rs"));

use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_time::Timer;
use panic_rtt_target as _;
use static_cell::ConstStaticCell;

// Only gray-specific input heads are duplicated. Every frame shares one
// scan tail and one ROW table, so RAM no longer scales by repeats * grays.
// Larger cascades use four representative levels to stay within 512 KiB.
const CYCLE_FRAMES: usize = if !GRAY_SWEEP { 1 } else if protocol::CHAIN_COPIES <= 3 {
    protocol::GRAY_SWEEP.len()
} else { 4 };
static HEADS: ConstStaticCell<[u32; protocol::HEAD_WORDS * CYCLE_FRAMES]> =
    ConstStaticCell::new([0; protocol::HEAD_WORDS * CYCLE_FRAMES]);
static TAIL: ConstStaticCell<[u32; protocol::TAIL_WORDS]> =
    ConstStaticCell::new([0; protocol::TAIL_WORDS]);
static ROW_RUNS: ConstStaticCell<[u32; protocol::ROW_RUN_COUNT]> =
    ConstStaticCell::new([0; protocol::ROW_RUN_COUNT]);
static DESCRIPTORS: ConstStaticCell<[u32; CYCLE_FRAMES * 2 * 4]> =
    ConstStaticCell::new([0; CYCLE_FRAMES * 2 * 4]);
static RESTART: ConstStaticCell<[u32; 2]> = ConstStaticCell::new([0; 2]);
const FULL_CONFIG_EVERY_FRAME: bool = true;

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
    let heads = HEADS.take();
    let tail = TAIL.take();
    let row_runs = ROW_RUNS.take();
    protocol::encode_row_runs(row_runs);
    for i in 0..CYCLE_FRAMES {
        let image = if GRAY_SWEEP {
            if CYCLE_FRAMES == protocol::GRAY_SWEEP.len() {
                protocol::sweep_frame(i)
            } else {
                protocol::pattern_frame(protocol::Pattern::White, [0, 1, 127, 255][i])
            }
        } else {
            protocol::pattern_frame(PATTERN, GRAY)
        };
        protocol::encode_split(
            &mut heads[i * protocol::HEAD_WORDS..(i + 1) * protocol::HEAD_WORDS],
            tail,
            &image,
        );
    }
    rtt_target::rprintln!(
        "ICND2069: chain={}, {}x{}, groups={}, DCLK={} Hz, frame={} clocks",
        protocol::CHAIN_COPIES, protocol::CHANNELS, protocol::ROWS, protocol::GROUPS,
        protocol::DCLK_HZ, protocol::FRAME_DCLKS,
    );
    rtt_target::rprintln!("wave RAM={} bytes", heads.len() * 4 + tail.len() * 4 + row_runs.len() * 4);
    rtt_target::rprintln!(
        "short_data={}, target=OUT{}, repeated LAT-low gap={} clocks, row input={} clocks",
        protocol::SHORT_DATA, protocol::TARGET_CHANNEL, protocol::REPEAT_LATCH_GAP,
        protocol::DATA_ROW_DCLKS,
    );
    rtt_target::rprintln!(
        "pattern={}, static gray={}, sweep={}, cycle={} frames, full_config={} ({} writes)",
        PATTERN_NAME, GRAY, GRAY_SWEEP, CYCLE_FRAMES, FULL_CONFIG_EVERY_FRAME,
        protocol::CONFIG_WRITES,
    );
    Timer::after_millis(100).await;
    driver.start_segments(
        heads, protocol::HEAD_WORDS, tail, row_runs, DESCRIPTORS.take(),
        RESTART.take(), p.DMA_CH2, p.DMA_CH3, p.DMA_CH4,
    );
    loop {
        // Only heartbeat work on the CPU; hardware changes gray every frame.
        Timer::after_secs(1).await;
        led.toggle();
    }
}
