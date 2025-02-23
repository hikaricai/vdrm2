#![no_std]
#![no_main]
mod clocks2;
mod core1;
use clocks2::gen_raw_buf;
use core::ptr::NonNull;
use embassy_executor::Spawner;
use embassy_rp::{gpio, multicore::spawn_core1, Peripherals};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, zerocopy_channel};
use static_cell::{ConstStaticCell, StaticCell};
use {defmt_rtt as _, panic_probe as _};
// use panic_probe as _;
const IMG_WIDTH: usize = mbi5264_common::IMG_WIDTH;
const IMG_HEIGHT: usize = mbi5264_common::IMG_HEIGHT;
const IMG_SIZE: usize = mbi5264_common::IMG_WIDTH * mbi5264_common::IMG_HEIGHT;
type RGBH = [u8; 4];
type ImageBuffer = [RGBH; IMG_SIZE];
// static mut BUF2: [ImageBuffer; 2] = [[[0; 4]; IMG_SIZE]; 2];
// static IMG_CHANNEL2: ConstStaticCell<
//     zerocopy_channel::Channel<'_, CriticalSectionRawMutex, ImageBuffer>,
// > = ConstStaticCell::new(zerocopy_channel::Channel::new(&mut BUF2));

static BUF: ConstStaticCell<[ImageBuffer; 2]> = ConstStaticCell::new([[[0; 4]; IMG_SIZE]; 2]);
static IMG_CHANNEL: StaticCell<
    zerocopy_channel::Channel<'_, CriticalSectionRawMutex, ImageBuffer>,
> = StaticCell::new();

struct SafeSender<T: 'static> {
    sender: zerocopy_channel::Sender<'static, CriticalSectionRawMutex, T>,
}

unsafe impl<T: 'static> Send for SafeSender<T> {}

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

    let channel = IMG_CHANNEL.init(zerocopy_channel::Channel::new(BUF.take()));
    let (sender, mut receiver) = channel.split();
    let safe_sender = SafeSender { sender };
    // core1::spawn_usb_core1(p.CORE1, p.USB, safe_sender);

    let mut led_pin = gpio::Output::new(p.PIN_25, gpio::Level::Low);
    // let mut dbg_pin = gpio::Output::new(p.PIN_11, gpio::Level::Low);
    // let mut dbg_pin2 = gpio::Output::new(p.PIN_12, gpio::Level::Low);
    let pins = clocks2::CmdClockPins {
        clk_pin: p.PIN_1,
        r0_pin: p.PIN_2,
        g0_pin: p.PIN_3,
        b0_pin: p.PIN_4,
        le0_pin: p.PIN_5,
        r1_pin: p.PIN_6,
        g1_pin: p.PIN_7,
        b1_pin: p.PIN_8,
        le1_pin: p.PIN_9,
        r2_pin: p.PIN_10,
        g2_pin: p.PIN_11,
        b2_pin: p.PIN_12,
        le2_pin: p.PIN_13,
    };
    let data_ch = p.DMA_CH0;
    let le_ch = p.DMA_CH1;
    let mut line = clocks2::LineClock::new(
        p.PWM_SLICE7,
        p.PWM_SLICE0,
        p.PWM_SLICE1,
        p.PIN_14,
        p.PIN_16,
        p.PIN_17,
        p.PIN_0,
        p.PIN_18,
    );
    let mut cmd_pio = clocks2::CmdClock::new(p.PIO0, pins, data_ch, le_ch);

    let sync_cmd = Command::new_sync();
    let confirm_cmd = Command::new_confirm();
    let mut cnt: usize = 0;
    let mut cmd_iter = UMINI_CMDS.iter();
    // put buf in ram, flash is tooooooo slow
    // let palette: [[u16; clocks2::CMD_BUF_SIZE]; 4] = [0u16, 1, 1, 1].map(|idx| {
    //     let color = idx * 0xffff;
    //     gen_raw_buf([color; 3])
    // });
    let mut frame = 0usize;
    // for &(cmd, param) in cmd_iter {
    //     cmd_pio.refresh2(&confirm_cmd);
    //     cmd_pio.refresh2(&Command::new(cmd as u8, param));
    // }
    let mut cmd_iter = core::iter::repeat(UMINI_CMDS.iter()).flatten();
    let mut coloum: [RGBH; IMG_HEIGHT] = [[255, 255, 255, 0]; IMG_HEIGHT];
    // let mut buf = [0u16; 8192];
    // let mut parser = clocks2::ColorParser::new(&mut buf);
    loop {
        let &(cmd, param) = cmd_iter.next().unwrap();
        // cmd_pio.refresh2(&confirm_cmd);
        // cmd_pio.refresh2(&Command::new(cmd as u8, param));

        // cmd_pio.refresh(&sync_cmd);
        // vsync
        // line.start();
        // update_frame(&mut cmd_pio, &palette, frame);
        // frame += 1;
        // frame %= 16;
        // defmt::info!("main loop");
        // let img = receiver.receive().await;
        // for p in img.iter_mut() {
        //     p[3] = core::cmp::min(p[3], 143);
        // }
        // defmt::info!("acquire qoi recv buffer");
        // for (idx, coloum) in img.chunks(IMG_HEIGHT).enumerate() {
        //     defmt::info!("update_frame coloum {}", idx);
        //     let coloum = unsafe { core::mem::transmute(coloum.as_ptr()) };
        //     cmd_pio.refresh(&sync_cmd);
        //     // vsync
        //     line.start();
        //     // defmt::info!("[begin] update_frame2");
        //     update_frame2(&mut cmd_pio, coloum);
        //     // defmt::info!("[end] update_frame2");
        //     line.wait_stop().await;
        // }
        // receiver.receive_done();
        // defmt::info!("release qoi recv buffer");
        // line.wait_stop().await;
        //
        let h = cnt as u8 % 144;
        for (idx, c) in coloum.iter_mut().enumerate() {
            c[3] = (0 + (idx as u8 / 64) << 4) % 144;
            // c[3] = 0;
        }
        for _idx in 0..IMG_WIDTH {
            // cmd_pio.refresh(&sync_cmd);
            // vsync
            line.start();
            // defmt::info!("[begin] update_frame2");
            update_frame3(&mut cmd_pio, &coloum);
            // defmt::info!("[end] update_frame2");
            line.wait_stop().await;
        }

        // let color = [[255u8; 3]; 3];
        // let idx = cnt as usize % 9;
        // // cmd_pio.refresh_color(color, idx);

        // let buf_ptr = buf.as_ptr();
        // let mut parser = clocks2::ColorParser::new(&mut buf);
        // parser.add_color2(&pixel_slot.buf, 0, 0);
        // parser.add_color_end(&pixel_slot.buf, 8, 1);
        // // parser.add_color(color, idx as u32);
        // // parser.add_empty_les(idx as u32);
        // // parser.add_color(color, idx as u32);
        // // parser.add_empty_les(idx as u32);
        // if cnt == 0 {
        //     let buf_t: &[u16; 20] = unsafe { core::mem::transmute(parser.buf_ori) };
        //     defmt::info!("buf_t {:?}", buf_t);
        //     defmt::info!(
        //         "buf {} buf_ori {} buf_t {} loop {}",
        //         buf_ptr,
        //         parser.buf_ori,
        //         parser.buf,
        //         parser.loops as *mut u32
        //     );
        // }
        // parser.run(&mut cmd_pio);
        // // clear the buf
        // buf = [0u16; 1024];

        if cnt & 0x10 != 0 {
            led_pin.toggle();
        }
        cnt += 1;
    }
}

struct RGBMeta {
    rgbh: [u8; 4],
    h_div: u8,
    h_mod: u8,
    region: u16,
}

impl RGBMeta {
    #[inline]
    fn new(rgbh: [u8; 4], region: u16) -> Self {
        let h = rgbh[3];
        let h_div = (h >> 4) & 0x0F;
        let h_mod = h & 0x0F;
        Self {
            rgbh,
            h_div,
            h_mod,
            region,
        }
    }
}

fn update_frame3(
    // parser: &mut clocks2::ColorParser,
    cmd_pio: &mut clocks2::CmdClock,
    rgbh_coloum: &[RGBH; IMG_HEIGHT],
) {
    let mut buf = [0u16; 8192];
    let mut parser = clocks2::ColorParser::new(&mut buf);
    let region0 = &rgbh_coloum[0..64];
    let region1 = &rgbh_coloum[64..128];
    let region2 = &rgbh_coloum[128..];
    // init last_h_mod with 15, so the first line's "empty" is first h_mod
    let mut last_h_mod = 15;
    for line in 0..64usize {
        // defmt::info!("line {}", line);
        // TODO optimize speed
        let p0 = RGBMeta::new(region0[line], 0);
        let p1 = RGBMeta::new(region1[line], 1);
        let p2 = RGBMeta::new(region2[line], 2);
        let mut pixels = [p0, p1, p2];
        bubble_rgbh2(&mut pixels);
        let mut pixel_iter = pixels.iter_mut();
        let last_pixel = pixel_iter.next().unwrap();
        let mut last_solt = PixelSlot2::new(last_pixel, 0);

        let empty = if last_solt.h_mod > last_h_mod {
            15 + last_solt.h_mod - last_h_mod
        } else {
            15 - (last_h_mod - last_solt.h_mod)
        };
        last_h_mod = last_solt.h_mod;
        parser.add_empty_les(empty as u32);

        for rgbh_meta in pixel_iter {
            last_h_mod = rgbh_meta.h_mod;
            if rgbh_meta.h_mod == last_solt.h_mod {
                if rgbh_meta.h_div == last_solt.h_div {
                    last_solt.update(&rgbh_meta);
                } else {
                    // new chip_idx
                    let last_chip_idx = last_solt.h_div as u32;
                    parser.add_color2(&last_solt.buf, last_chip_idx, last_solt.last_chip_idx);
                    last_solt = PixelSlot2::new(rgbh_meta, last_chip_idx + 1);
                }
                continue;
            }

            parser.add_color_end(
                &last_solt.buf,
                last_solt.h_div as u32,
                last_solt.last_chip_idx,
            );

            last_solt = PixelSlot2::new(rgbh_meta, 0);
            let empty = rgbh_meta.h_mod - last_h_mod;
            parser.add_empty_les(empty as u32);
        }

        parser.add_color_end(
            &last_solt.buf,
            last_solt.h_div as u32,
            last_solt.last_chip_idx,
        );
    }
    parser.add_empty_les(15 - last_h_mod as u32);
    parser.run(cmd_pio);
}

#[inline]
fn bubble_rgbh2(slice: &mut [RGBMeta]) {
    let len = slice.len();
    for i in 0..len {
        for j in 0..len - 1 - i {
            let (l, r) = (&slice[j], &slice[j + 1]);
            if (l.h_mod, l.h_div) > (r.h_mod, r.h_div) {
                // Swap elements
                slice.swap(j, j + 1);
            }
        }
    }
}

struct PixelSlot2 {
    buf: [u16; 8],
    h: u8,
    h_div: u8,
    h_mod: u8,
    last_chip_idx: u32,
}

impl PixelSlot2 {
    #[inline]
    fn new(rgbh_meta: &RGBMeta, last_chip_idx: u32) -> Self {
        let mut buf = [0u16; 8];
        let &RGBMeta {
            rgbh,
            region,
            h_div,
            h_mod,
        } = rgbh_meta;
        let [r, g, b, h] = rgbh;
        for (i, buf) in (0..8).rev().zip(buf.iter_mut()) {
            let r = (r >> i) & 1;
            let g = (g >> i) & 1;
            let b = (b >> i) & 1;
            let rgb = (r | (g << 1) | (b << 2)) as u16;
            *buf |= rgb << (4 * region);
        }
        Self {
            buf,
            h,
            h_div,
            h_mod,
            last_chip_idx,
        }
    }

    #[inline]
    fn update(&mut self, rgbh_meta: &RGBMeta) {
        let region = rgbh_meta.region;
        let [r, g, b, _] = rgbh_meta.rgbh;
        for (i, buf) in (0..8).rev().zip(self.buf.iter_mut()) {
            let r = (r >> i) & 1;
            let g = (g >> i) & 1;
            let b = (b >> i) & 1;
            let rgb = (r | (g << 1) | (b << 2)) as u16;
            *buf |= rgb << (4 * region);
        }
    }
}
