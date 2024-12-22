//! Displays an animated Nyan cat
#![no_std]
#![no_main]
mod clocks;
use core::cell::RefCell;

use bsp::entry;
use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac, pwm,
    sio::Sio,
    watchdog::Watchdog,
};
use clocks::LineClock;
use critical_section::Mutex;
use embedded_hal::digital::OutputPin;
use embedded_hal::pwm::SetDutyCycle;
use panic_rtt_target as _;
use rp2040_hal::dma::DMAExt;
use rp2040_hal::gpio::{DynPinId, DynPullType, FunctionSioOutput, Pin};
use rp2040_hal::pac::interrupt;
use rp_pico as bsp;
use rtt_target::{rprintln, rtt_init};
static GLOBAL_LINE_CLOCK: Mutex<RefCell<Option<LineClock>>> = Mutex::new(RefCell::new(None));
type Outpin = Pin<DynPinId, FunctionSioOutput, DynPullType>;

#[repr(packed)]
pub struct Transaction {
    // 命令
    pub cmd: u8,
    // 9个串联芯片
    pub regs: [[u16; 3]; 9],
}

impl Transaction {
    fn mock(cmd: u8) -> Self {
        Self {
            cmd,
            regs: [[0xff00 + cmd as u16; 3]; 9],
        }
    }
}

#[entry]
fn main() -> ! {
    let mut rtt = rtt_init! {
        up: {
            0: {
                size: 1024,
                name: "Terminal"
            }
        }
        down: {
            0: {
                size: 1024,
                name: "Terminal"
            }
        }
    };
    rtt_target::set_print_channel(rtt.up.0);
    // rtt_init_print!();
    rprintln!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);
    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let cmd_pins = clocks::CmdClockPins {
        clk_pin: pins.gpio6,
        le_pin: pins.gpio7,
        r0_pin: pins.gpio8,
        g0_pin: pins.gpio9,
        b0_pin: pins.gpio10,
    };

    let mut cmd_pio = clocks::CmdClock::new(cmd_pins, pac.PIO0, pac.DMA, &mut pac.RESETS);

    let mut led_pin: Outpin = pins
        .led
        .into_push_pull_output()
        .into_dyn_pin()
        .into_pull_type();
    // let clk_pin: Outpin = pins
    //     .gpio4
    //     .into_push_pull_output()
    //     .into_dyn_pin()
    //     .into_pull_type();
    // let scan_pin: Outpin = pins
    //     .gpio0
    //     .into_push_pull_output()
    //     .into_dyn_pin()
    //     .into_pull_type();
    // let a_pin: Outpin = pins
    //     .gpio1
    //     .into_push_pull_output()
    //     .into_dyn_pin()
    //     .into_pull_type();
    // let b_pin: Outpin = pins
    //     .gpio2
    //     .into_push_pull_output()
    //     .into_dyn_pin()
    //     .into_pull_type();
    // let c_pin: Outpin = pins
    //     .gpio3
    //     .into_push_pull_output()
    //     .into_dyn_pin()
    //     .into_pull_type();

    let pwm_slices = pwm::Slices::new(pac.PWM, &mut pac.RESETS);
    let mut line_clock = LineClock::new(pwm_slices, pins.gpio0, pins.gpio2, pins.gpio4, pins.gpio5);

    line_clock.start();
    critical_section::with(move |cs| {
        GLOBAL_LINE_CLOCK.replace(cs, Some(line_clock));
    });
    // let a_b_c_pin = (a_pin, b_pin, c_pin);
    // let _app = clocks::ClockApp::new(
    //     clk_pin,
    //     scan_pin,
    //     a_b_c_pin,
    //     sio.fifo,
    //     &mut pac.PSM,
    //     &mut pac.PPB,
    // );
    let mut read_buf = [0; 1024];
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::PWM_IRQ_WRAP);
    }
    let mut mock_cmd = 1;
    loop {
        delay.delay_ms(10);
        cmd_pio.refresh(Transaction::mock(mock_cmd));
        cmd_pio.commit();
        mock_cmd += 1;
        if mock_cmd == 143 {
            mock_cmd = 1
        }
        led_pin.set_low().unwrap();
        delay.delay_ms(10);
        led_pin.set_high().unwrap();
        critical_section::with(move |cs| {
            GLOBAL_LINE_CLOCK
                .borrow_ref_mut(cs)
                .as_mut()
                .unwrap()
                .start();
        });
        loop {
            let read_len = rtt.down.0.read(&mut read_buf);
            if read_len == 0 {
                break;
            }
            rprintln!("read {}", read_len);
        }
    }
}

#[allow(static_mut_refs)] // See https://github.com/rust-embedded/cortex-m/pull/561
#[interrupt]
fn PWM_IRQ_WRAP() {
    static mut CNT: u32 = 0;
    critical_section::with(move |cs| {
        *CNT += 1;
        let mut guard = GLOBAL_LINE_CLOCK.borrow_ref_mut(cs);
        let line_clk = guard.as_mut().unwrap();
        line_clk.clear_interupte();
        if *CNT == 32 {
            *CNT = 0;
            line_clk.stop();
        }
    });
}
