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

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    rtt_target::rtt_init_print!(rtt_target::ChannelMode::NoBlockSkip, 2048);
    let p = embassy_rp::init(Default::default());
    let mut led = Output::new(p.PIN_25, Level::Low);
    let mut driver = driver::Driver::new(
        p.PIO0,
        p.DMA_CH0,
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
    let frame_len = protocol::encode_frame(frame, &protocol::demo_frame());
    assert_eq!(frame_len, protocol::FRAME_WORDS);
    assert!(protocol::DEMO_GRAY < protocol::GRAY_CAPACITY);
    rtt_target::rprintln!(
        "ICND2069: PIO0/SM0, DCLK={} Hz, {}x{}, {} groups, {} clocks/row",
        protocol::DCLK_HZ,
        protocol::CHANNELS,
        protocol::ROWS,
        protocol::GROUPS,
        protocol::ROW_DCLKS
    );
    rtt_target::rprintln!(
        "reference flow, frame={} bytes; 47-register rotation; RT5990 timing unverified",
        frame_len * 4
    );
    rtt_target::rprintln!("pattern=uniform, raw_gray={}", protocol::DEMO_GRAY);

    let mut frames = 0u32;
    let mut last = Instant::now();
    loop {
        // Only the 128-sample register slot changes; the rest stays immutable.
        // PIO keeps DCLK running with blank pins during CPU/DMA boundaries.
        protocol::update_register(frame, frames as usize % protocol::REGISTERS.len());
        driver.send(&frame[..frame_len]).await;
        frames = frames.wrapping_add(1);
        if frames.is_multiple_of(60) {
            let now = Instant::now();
            rtt_target::rprintln!("60 frames / {} us", (now - last).as_micros());
            last = now;
            led.toggle();
        }
    }
}
