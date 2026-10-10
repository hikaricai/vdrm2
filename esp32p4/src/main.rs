#![no_std]
#![no_main]

use esp_hal::{
    delay::Delay,
    gpio::Level,
    main,
    rmt::{PulseCode, Rmt, TxChannelConfig, TxChannelCreator},
    time::Rate,
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let delay = Delay::new();

    // LED1 is an addressable RGB LED driven by GPIO51. Send an all-zero frame
    // and keep its data input low so it cannot latch a random color at startup.
    let rmt = Rmt::new(peripherals.RMT, Rate::from_mhz(10)).unwrap();
    let tx_config = TxChannelConfig::default()
        .with_clk_divider(1)
        .with_idle_output_level(Level::Low)
        .with_idle_output(true);
    let rgb_led = rmt
        .channel0
        .configure_tx(&tx_config)
        .unwrap()
        .with_pin(peripherals.GPIO51);
    let mut off_frame = [PulseCode::new(Level::High, 4, Level::Low, 9); 25];
    off_frame[24] = PulseCode::end_marker();
    let _rgb_led = rgb_led.transmit(&off_frame).unwrap().wait().unwrap();
    delay.delay_micros(80);

    println!("ESP32-P4 Rust Hello World started!");
    println!("GPIO51 RGB LED1 forced off; hard-wired power LED2 is not software-controlled.");
    let mut count = 0_u32;
    loop {
        println!("Hello, world! ESP32-P4 Rust demo count={}", count);
        count = count.wrapping_add(1);
        delay.delay_millis(1_000);
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo<'_>) -> ! {
    println!("PANIC: {}", info);
    loop {
        core::hint::spin_loop();
    }
}
