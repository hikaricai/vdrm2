//! Displays an animated Nyan cat
#![no_std]
#![no_main]
mod clocks;
use core::cell::RefCell;
use core::fmt::Write;

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
pub struct Command {
    // 命令
    pub cmd: u8,
    // 9个串联芯片
    pub regs: [[u16; 3]; 9],
}

impl Command {
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
            1: {
                size: 1024,
                name: "Cmd"
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

    let pwm_slices = pwm::Slices::new(pac.PWM, &mut pac.RESETS);
    let line_clock = LineClock::new(pwm_slices, pins.gpio0, pins.gpio2, pins.gpio4, pins.gpio5);
    critical_section::with(move |cs| {
        GLOBAL_LINE_CLOCK.replace(cs, Some(line_clock));
    });
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::PWM_IRQ_WRAP);
    }
    let mut cmd_rx = CmdRx::new(rtt.down.0, rtt.up.1);
    loop {
        delay.delay_ms(10);
        // cmd_pio.refresh(Transaction::mock(mock_cmd));
        if let Some(cmd) = cmd_rx.try_recv() {
            cmd_pio.refresh_color(cmd);
            cmd_pio.commit();
            cmd_rx.ack();
        }
        critical_section::with(move |cs| {
            GLOBAL_LINE_CLOCK
                .borrow_ref_mut(cs)
                .as_mut()
                .unwrap()
                .start();
        });
        // rprintln!("mock_cmd {}", mock_cmd);
        led_pin.set_low().unwrap();
        delay.delay_ms(10);
        led_pin.set_high().unwrap();
    }
}

struct CmdRx {
    down: rtt_target::DownChannel,
    up: rtt_target::UpChannel,
    read_buf: [u8; 1024],
}

impl CmdRx {
    fn new(down: rtt_target::DownChannel, up: rtt_target::UpChannel) -> Self {
        Self {
            down,
            up,
            read_buf: [0; 1024],
        }
    }
    fn try_recv(&mut self) -> Option<&Command> {
        let read_len = self.down.read(&mut self.read_buf);
        if read_len != core::mem::size_of::<Command>() {
            if read_len != 0 {
                rprintln!("invalid size {}", read_len);
            }
            return None;
        }
        let trans: &Command = unsafe { &*(self.read_buf.as_ptr() as *const Command) };
        rprintln!("trans cmd {}", trans.cmd);
        Some(trans)
    }

    fn ack(&mut self) {
        self.up.write(b"\n");
    }
}

#[allow(static_mut_refs)] // See https://github.com/rust-embedded/cortex-m/pull/561
#[interrupt]
fn PWM_IRQ_WRAP() {
    critical_section::with(move |cs| {
        let mut guard = GLOBAL_LINE_CLOCK.borrow_ref_mut(cs);
        let line_clk = guard.as_mut().unwrap();
        line_clk.handle_interrupt();
    });
}
