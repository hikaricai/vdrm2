#![no_std]
#![no_main]
mod clocks2;
use clocks2::gen_raw_buf;
use embassy_executor::Spawner;
use embassy_rp::gpio;
use {defmt_rtt as _, panic_probe as _};

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
    defmt::info!("Hello there!");
    let p = embassy_rp::init(Default::default());
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
    let mut line = clocks2::LineClock::new(
        p.PWM_SLICE0,
        p.PWM_SLICE1,
        p.PWM_SLICE2,
        p.PIN_0,
        p.PIN_2,
        p.PIN_4,
        p.PIN_5,
    );
    let mut cmd_pio = clocks2::CmdClock::new(p.PIO0, pins, data_ch);

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
        cmd_pio.refresh(&sync_cmd).await;
        if let Some(&(cmd, param)) = cmd_iter.next() {
            cmd_pio.refresh(&confirm_cmd).await;
            cmd_pio.refresh(&Command::new(cmd as u8, param)).await;
        }
        // vsync
        line.start();
        for idx in 0..1024 {
            dbg_pin.set_high();
            let coloum_idx = idx % 16;
            let empty_coloum_idx = (frame + 1) % 16;
            let task = if coloum_idx == frame {
                cmd_pio.refresh_raw_buf(&palette[3])
            } else if coloum_idx == empty_coloum_idx {
                cmd_pio.refresh_raw_buf(&palette[0])
            } else {
                cmd_pio.refresh_empty_buf()
            };
            task.wait().await;
            dbg_pin.set_low();
        }
        line.wait_stop().await;
        frame += 1;
        frame %= 16;
        if cnt >= 10 {
            cnt = 0;
            led_pin.toggle();
        }
    }
}
