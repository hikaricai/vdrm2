//! Displays an animated Nyan cat
#![no_std]
#![no_main]
mod clocks;
mod clocks2;
use core::cell::RefCell;
use core::fmt::Write;

use bsp::entry;
use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac, pwm,
    sio::Sio,
    watchdog::Watchdog,
};
use clocks::{gen_raw_buf, LineClock, CMD_BUF_SIZE};
use critical_section::Mutex;
use defmt::info;
use embedded_hal::digital::{OutputPin, StatefulOutputPin};
use rp2040_hal::gpio::{DynPinId, DynPullType, FunctionSioOutput, Pin};
use rp2040_hal::pac::interrupt;
use rp_pico as bsp;
use {defmt_rtt as _, panic_probe as _};
static GLOBAL_LINE_CLOCK: Mutex<RefCell<Option<LineClock>>> = Mutex::new(RefCell::new(None));
type Outpin = Pin<DynPinId, FunctionSioOutput, DynPullType>;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::USB;
use embassy_usb::driver::{Endpoint, EndpointIn, EndpointOut};
use embassy_usb::msos::{self, windows_version};
use embassy_usb::{Builder, Config};

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

    fn new(cmd: u8, param: u16) -> Self {
        Self {
            cmd,
            regs: [[param; 3]; 9],
        }
    }

    fn new_sync() -> Self {
        Self {
            cmd: 2,
            regs: [[0; 3]; 9],
        }
    }

    fn new_confirm() -> Self {
        Self {
            cmd: 14,
            regs: [[0; 3]; 9],
        }
    }
}
fn gen_colors() -> [Command; 1024] {
    let mut cmds = unsafe { core::mem::MaybeUninit::<[Command; 1024]>::zeroed().assume_init() };
    for y in 0..64u16 {
        for x in 0..16u16 {
            let regs = [[(y * 16 + x) << 2; 3]; 9];
            // let regs = [[0x1555; 3]; 9];
            let cmd = Command { cmd: 1, regs };
            cmds[(y * 16 + x) as usize] = cmd;
        }
    }
    cmds
}

const COLOR_RAW_BUF: [[u16; clocks::CMD_BUF_SIZE]; 1024] = clocks::gen_colors_raw_buf();
const UMINI_CMDS: &[(mbi5264_common::CMD, u16)] = &mbi5264_common::unimi_cmds();

#[entry]
fn entry() -> ! {
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

    let mut dbg_pin: Outpin = pins
        .gpio11
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
    // let colors = gen_colors();
    let sync_cmd = Command::new_sync();
    let confirm_cmd = Command::new_confirm();
    let mut cnt = 0;
    let mut do_latch = false;
    let mut raw_color_buf1: [u16; clocks::CMD_BUF_SIZE] = [0; clocks::CMD_BUF_SIZE];
    let mut raw_color_buf2: [u16; clocks::CMD_BUF_SIZE] = [0; clocks::CMD_BUF_SIZE];
    let mut using_raw_color_buf1 = true;
    let mut cmd_iter = UMINI_CMDS.iter();
    // put buf in ram, flash is tooooooo slow
    let palette: [[u16; clocks::CMD_BUF_SIZE]; 4] = [0u16, 1, 1, 1].map(|idx| {
        let color = idx * 0xffff;
        gen_raw_buf([color; 3])
    });
    let mut frame = 0usize;
    loop {
        cnt += 1;
        cmd_pio.refresh(&sync_cmd);
        cmd_pio.commit();
        if let Some(&(cmd, param)) = cmd_iter.next() {
            cmd_pio.refresh(&confirm_cmd);
            cmd_pio.commit();
            cmd_pio.refresh(&Command::new(cmd as u8, param));
            cmd_pio.commit();
        }
        // vsync
        critical_section::with(move |cs| {
            GLOBAL_LINE_CLOCK
                .borrow_ref_mut(cs)
                .as_mut()
                .unwrap()
                .start();
        });

        for (idx, _raw) in COLOR_RAW_BUF.iter().enumerate() {
            // dbg_pin.set_high().unwrap();
            let coloum_idx = idx % 16;
            let empty_coloum_idx = (frame + 1) % 16;
            if coloum_idx == frame {
                cmd_pio.refresh_raw_buf(&palette[3]);
            } else if coloum_idx == empty_coloum_idx {
                cmd_pio.refresh_raw_buf(&palette[0]);
            } else {
                cmd_pio.refresh_empty_buf();
            }
            // match coloum_idx {
            //     0 => {
            //         cmd_pio.refresh_raw_buf(&palette[3]);
            //     }
            //     1 => {
            //         cmd_pio.refresh_raw_buf(&palette[0]);
            //     }
            //     // 5..15 => {
            //     //     // cmd_pio.refresh_raw_buf(&palette[0]);
            //     //     cmd_pio.refresh_empty_buf();
            //     // }
            //     // 10 => {
            //     //     cmd_pio.refresh_raw_buf(&palette[3]);
            //     // }
            //     _ => {
            //         cmd_pio.refresh_empty_buf();
            //     }
            // };
            cmd_pio.commit();
            // dbg_pin.set_low().unwrap();
        }
        // cmd_pio.commit();
        loop {
            let sync_end = critical_section::with(move |cs| {
                !GLOBAL_LINE_CLOCK
                    .borrow_ref_mut(cs)
                    .as_mut()
                    .unwrap()
                    .running
            });
            if sync_end {
                break;
            }
        }
        frame += 1;
        frame %= 16;
        // delay.delay_ms(1);
        // rprintln!("mock_cmd {}", mock_cmd);

        if cnt >= 10 {
            // critical_section::with(move |cs| {
            //     GLOBAL_LINE_CLOCK
            //         .borrow_ref_mut(cs)
            //         .as_mut()
            //         .unwrap()
            //         .start();
            // });
            cnt = 0;
            led_pin.toggle().unwrap();
        }
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

use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::{InterruptHandler, Pio};
embassy_rp::bind_interrupts!(struct PioIrqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Hello there!");
    let p = embassy_rp::init(Default::default());
    let pio0 = Pio::new(p.PIO0, PioIrqs);
    let pins = clocks2::CmdClockPins {
        clk_pin: p.PIN_6,
        le_pin: p.PIN_7,
        r0_pin: p.PIN_8,
        g0_pin: p.PIN_9,
        b0_pin: p.PIN_10,
    };
    let data_ch = p.DMA_CH0;
    let clock2 = clocks2::CmdClock::new(pio0, pins, data_ch);
}
