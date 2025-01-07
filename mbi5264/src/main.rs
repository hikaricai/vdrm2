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
use embedded_hal::digital::{OutputPin, StatefulOutputPin};
use panic_rtt_target as _;
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
                size: 10240,
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
    let mut cmd_rx = CmdRx::new(rtt.down.0);
    let mut cmd_rx_ack = CmdRxAck::new(rtt.up.1);
    let mut cnt = 0;
    let mut do_latch = false;
    let mut raw_color_buf1: [u16; clocks::CMD_BUF_SIZE] = [0; clocks::CMD_BUF_SIZE];
    let mut raw_color_buf2: [u16; clocks::CMD_BUF_SIZE] = [0; clocks::CMD_BUF_SIZE];
    let mut using_raw_color_buf1 = true;
    let mut cmd_iter = UMINI_CMDS.iter();
    loop {
        cnt += 1;
        cmd_pio.refresh(&sync_cmd);
        cmd_pio.commit();
        // if let Some(cmd) = cmd_rx.try_recv_one() {
        //     // rprintln!("cmd {}", cmd.cmd);
        //     cmd_pio.refresh(&confirm_cmd);
        //     cmd_pio.commit();
        //     delay.delay_us(14);
        //     critical_section::with(move |cs| {
        //         GLOBAL_LINE_CLOCK
        //             .borrow_ref_mut(cs)
        //             .as_mut()
        //             .unwrap()
        //             .set_gclk(rp2040_hal::gpio::PinState::High);
        //     });
        //     cmd_pio.refresh(cmd);
        //     cmd_pio.commit();
        //     cmd_rx_ack.ack();
        // }
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
        using_raw_color_buf1 = true;
        raw_color_buf1 = *COLOR_RAW_BUF.first().unwrap();
        cmd_pio.refresh_raw_buf(&raw_color_buf1);
        for raw in COLOR_RAW_BUF.iter() {
            dbg_pin.set_high().unwrap();
            // let next_buf = if using_raw_color_buf1 {
            //     &mut raw_color_buf2
            // } else {
            //     &mut raw_color_buf1
            // };
            // using_raw_color_buf1 = !using_raw_color_buf1;
            // *next_buf = *raw;
            let next_buf = &raw_color_buf1;
            cmd_pio.commit();
            cmd_pio.refresh_raw_buf(next_buf);
            dbg_pin.set_low().unwrap();
        }
        cmd_pio.commit();
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
        delay.delay_ms(1);
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

struct CmdRx {
    down: rtt_target::DownChannel,
    read_buf: [u8; 10240],
    cnt: usize,
    iter: usize,
}

struct CmdRxAck {
    up: rtt_target::UpChannel,
}

impl CmdRxAck {
    pub fn new(up: rtt_target::UpChannel) -> Self {
        Self { up }
    }

    fn ack(&mut self) {
        self.up.write(b"\n");
    }
}

impl CmdRx {
    fn new(down: rtt_target::DownChannel) -> Self {
        Self {
            down,
            read_buf: [0; 10240],
            cnt: 0,
            iter: 0,
        }
    }
    fn try_recv(&mut self) -> Option<&[Command]> {
        let read_len = self.down.read(&mut self.read_buf);
        let len = read_len / core::mem::size_of::<Command>();
        if read_len % core::mem::size_of::<Command>() != 0 {
            rprintln!("invalid size {}", read_len);
        }
        self.cnt = len;
        self.iter = 0;
        if len == 0 {
            return None;
        }
        let trans: &[Command] =
            unsafe { core::slice::from_raw_parts(self.read_buf.as_ptr() as *const Command, len) };
        // rprintln!("trans cmd {}", trans.cmd);
        Some(trans)
    }

    fn try_recv_one(&mut self) -> Option<&Command> {
        if self.iter == self.cnt {
            self.try_recv();
        }
        if self.cnt == 0 {
            return None;
        }
        if self.iter == self.cnt {
            return None;
        }
        let trans: &[Command] = unsafe {
            core::slice::from_raw_parts(self.read_buf.as_ptr() as *const Command, self.cnt)
        };
        let ret = &trans[self.iter];
        self.iter += 1;
        Some(ret)
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
