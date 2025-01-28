#![no_std]
#![no_main]
mod clocks2;
mod core1;
use clocks2::gen_raw_buf;
use embassy_executor::Spawner;
use embassy_rp::{gpio, multicore::spawn_core1};
use {defmt_rtt as _, panic_probe as _};
// use panic_probe as _;
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

// const COLOR_RAW_BUF: [[u16; clocks2::CMD_BUF_SIZE]; 1024] = clocks2::gen_colors_raw_buf();
const UMINI_CMDS: &[(mbi5264_common::CMD, u16)] = &mbi5264_common::unimi_cmds();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // defmt::info!("Hello there!");
    let core_num = embassy_rp::pac::SIO.cpuid().read();
    defmt::info!("main core {}", core_num);
    let p = embassy_rp::init(Default::default());
    embassy_rp::pac::BUSCTRL.bus_priority().write(|w| {
        w.set_dma_r(true);
        w.set_dma_w(true);
    });

    let usb = p.USB;
    spawn_core1(
        p.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(core1::CORE1_STACK) },
        move || {
            core1::run(usb);
        },
    );

    let mut led_pin = gpio::Output::new(p.PIN_25, gpio::Level::Low);
    let mut dbg_pin = gpio::Output::new(p.PIN_11, gpio::Level::Low);
    let mut dbg_pin2 = gpio::Output::new(p.PIN_12, gpio::Level::Low);
    let pins = clocks2::CmdClockPins {
        clk_pin: p.PIN_6,
        le_pin: p.PIN_7,
        r0_pin: p.PIN_8,
        g0_pin: p.PIN_9,
        b0_pin: p.PIN_10,
    };
    let data_ch = p.DMA_CH0;
    let le_ch = p.DMA_CH1;
    let mut line = clocks2::LineClock::new(
        p.PWM_SLICE0,
        p.PWM_SLICE1,
        p.PWM_SLICE2,
        p.PIN_0,
        p.PIN_2,
        p.PIN_4,
        p.PIN_5,
    );
    let mut cmd_pio = clocks2::CmdClock::new(p.PIO0, pins, data_ch, le_ch);

    let sync_cmd = Command::new_sync();
    let confirm_cmd = Command::new_confirm();
    let mut cnt = 0;
    let mut cmd_iter = UMINI_CMDS.iter();
    // put buf in ram, flash is tooooooo slow
    let palette: [[u16; clocks2::CMD_BUF_SIZE]; 4] = [0u16, 1, 1, 1].map(|idx| {
        let color = idx * 0xffff;
        gen_raw_buf([color; 3])
    });
    let mut frame = 0usize;
    loop {
        cnt += 1;
        cmd_pio.refresh(&sync_cmd);
        if let Some(&(cmd, param)) = cmd_iter.next() {
            cmd_pio.refresh(&confirm_cmd);
            cmd_pio.refresh(&Command::new(cmd as u8, param));
        }
        // vsync
        line.start();
        update_frame(&mut cmd_pio, &palette, frame);
        line.wait_stop().await;
        frame += 1;
        frame %= 16;
        if cnt >= 10 {
            cnt = 0;
            led_pin.toggle();
        }
    }
}

#[link_section = ".data"]
#[inline(never)]
fn update_frame(
    cmd_pio: &mut clocks2::CmdClock,
    palette: &[[u16; clocks2::CMD_BUF_SIZE]; 4],
    frame: usize,
) {
    let frame = frame % 16;
    if frame > 0 {
        cmd_pio.refresh_empty_buf(frame);
    }
    let mut cnt = frame;
    loop {
        if cnt >= 1024 {
            break;
        }
        cmd_pio.refresh_raw_buf(&palette[3]);
        cnt += 1;
        if cnt >= 1024 {
            break;
        }
        cmd_pio.refresh_raw_buf(&palette[0]);
        cnt += 1;
        let remain = 1024 - cnt;
        if remain == 0 {
            break;
        }
        if remain >= 14 {
            cmd_pio.refresh_empty_buf(14);
            cnt += 14;
        } else {
            cmd_pio.refresh_empty_buf(remain);
            cnt += remain;
        }
    }
}
