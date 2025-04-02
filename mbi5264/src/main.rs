#![no_std]
#![no_main]
mod clocks2;
mod core1;
use embassy_executor::Spawner;
use embassy_rp::gpio::{self, Input};
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    zerocopy_channel,
};
use embassy_time::{block_for, Duration, Instant, Timer};
use static_cell::{ConstStaticCell, StaticCell};
use {defmt_rtt as _, panic_probe as _};
// use panic_probe as _;
const IMG_WIDTH: usize = mbi5264_common::IMG_WIDTH;
const IMG_HEIGHT: usize = mbi5264_common::IMG_HEIGHT;
const IMG_SIZE: usize = mbi5264_common::IMG_WIDTH * mbi5264_common::IMG_HEIGHT;
type RGBH = [u8; 4];
type ImageBuffer = [mbi5264_common::AngleImage; mbi5264_common::IMG_HEIGHT];
const TOTAL_ANGLES: u32 = 180;

static MBI_BUF: ConstStaticCell<[MbiBuf2; 2]> =
    ConstStaticCell::new([MbiBuf2::new(0), MbiBuf2::new(0)]);
static MBI_CHANNEL: StaticCell<zerocopy_channel::Channel<'_, NoopRawMutex, MbiBuf2>> =
    StaticCell::new();
// static mut BUF2: [ImageBuffer; 2] = [[[0; 4]; IMG_SIZE]; 2];
// static IMG_CHANNEL2: ConstStaticCell<
//     zerocopy_channel::Channel<'_, CriticalSectionRawMutex, ImageBuffer>,
// > = ConstStaticCell::new(zerocopy_channel::Channel::new(&mut BUF2));

// static BUF: ConstStaticCell<[ImageBuffer; 1]> =
//     ConstStaticCell::new([[mbi5264_common::AngleImage::new(0); mbi5264_common::IMG_HEIGHT]; 1]);
// static IMG_CHANNEL: StaticCell<
//     zerocopy_channel::Channel<'_, CriticalSectionRawMutex, ImageBuffer>,
// > = StaticCell::new();

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

const UMINI_CMDS: &[(mbi5264_common::CMD, u16)] = &mbi5264_common::unimi_cmds();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // defmt::info!("Hello there!");
    let core_num = embassy_rp::pac::SIO.cpuid().read();
    defmt::info!("main core {}", core_num);
    let mut config = embassy_rp::config::Config::default();
    let sys_pll = config
        .clocks
        .xosc
        .as_mut()
        .unwrap()
        .sys_pll
        .as_mut()
        .unwrap();
    sys_pll.fbdiv = 125;
    // overclock
    // sys_pll.post_div1 = 4;
    sys_pll.post_div1 = 6;
    let p = embassy_rp::init(config);
    embassy_rp::pac::BUSCTRL.bus_priority().write(|w| {
        w.set_dma_r(true);
        w.set_dma_w(true);
    });

    // let channel = IMG_CHANNEL.init(zerocopy_channel::Channel::new(BUF.take()));
    // let (sender, img_rx) = channel.split();
    // let img_tx = SafeSender { sender };
    // core1::spawn_usb_core1(p.CORE1, p.USB, img_tx);
    let mbi_channel = MBI_CHANNEL.init(zerocopy_channel::Channel::new(MBI_BUF.take()));
    let (mbi_tx, mut mbi_rx) = mbi_channel.split();
    // spawner.spawn(encode_mbi(mbi_tx, img_rx)).unwrap();
    spawner.spawn(encode_mbi_vdrm(mbi_tx)).unwrap();
    // spawner.spawn(encode_mbi2(mbi_tx)).unwrap();

    let mut led_pin = gpio::Output::new(p.PIN_25, gpio::Level::Low);
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
    let mut cmd_pio = clocks2::CmdClock::new(p.PIO0, pins, data_ch);
    let sync_cmd = Command::new_sync();
    let confirm_cmd = Command::new_confirm();
    let mut cnt: usize = 0;
    let cmd_iter = UMINI_CMDS.iter();

    for &(cmd, param) in cmd_iter {
        cmd_pio.refresh2(&confirm_cmd);
        cmd_pio.refresh2(&Command::new(cmd as u8, param));
    }

    test_screen_vdrm(&mut cmd_pio, &mut line, &mut mbi_rx, &mut led_pin).await;

    let start_angle = TOTAL_ANGLES / 4;
    let end_angle = start_angle + TOTAL_ANGLES / 2 - 1;
    let mut sync_signal = Input::new(p.PIN_22, gpio::Pull::None);
    sync_signal.wait_for_falling_edge().await;
    let mut last_sync_tick = Instant::now();

    let mut angle = start_angle;
    let mut cmd_iter = core::iter::repeat(UMINI_CMDS.iter()).flatten();
    loop {
        let &(cmd, param) = cmd_iter.next().unwrap();
        cmd_pio.refresh2(&confirm_cmd);
        cmd_pio.refresh2(&Command::new(cmd as u8, param));

        sync_signal.wait_for_falling_edge().await;
        let now = Instant::now();
        let ticks_per_angle = (now - last_sync_tick).as_ticks() as u32 / TOTAL_ANGLES;
        last_sync_tick = now;
        Timer::at(now + Duration::from_ticks((angle * ticks_per_angle) as u64)).await;

        loop {
            if angle >= end_angle {
                break;
            }
            cmd_pio.refresh2(&sync_cmd);
            // vsync
            line.start();
            // need sleep at least 15 micros , or there will be emi problem
            Timer::after_micros(15).await;
            let buf = mbi_rx.receive().await;
            cmd_pio.refresh_ptr(buf.buf.as_ptr() as u32, buf.len);
            cmd_pio.wait().await;
            mbi_rx.receive_done();
            line.wait_stop().await;
            angle += 1;
            Timer::at(now + Duration::from_ticks((angle * ticks_per_angle) as u64)).await;
        }
        angle = start_angle;
        if cnt & 0x10 != 0 {
            led_pin.toggle();
        }
        cnt += 1;
    }
}

async fn test_screen(
    cmd_pio: &mut clocks2::CmdClock,
    line: &mut clocks2::LineClockHdl,
    mbi_rx: &mut zerocopy_channel::Receiver<'static, NoopRawMutex, MbiBuf2>,
    led_pin: &mut gpio::Output<'_>,
) {
    let sync_cmd = Command::new_sync();
    let confirm_cmd = Command::new_confirm();
    let mut cnt: usize = 0;
    let cmd_iter = UMINI_CMDS.iter();
    for &(cmd, param) in cmd_iter {
        cmd_pio.refresh2(&confirm_cmd);
        cmd_pio.refresh2(&Command::new(cmd as u8, param));
    }
    let mut cmd_iter = core::iter::repeat(UMINI_CMDS.iter()).flatten();

    let mut buf = [0u16; 16384];
    let mut coloum: [RGBH; IMG_HEIGHT] = [[255, 255, 255, 0]; IMG_HEIGHT];
    let mut last = Instant::now();
    let mut frame_cnt = 0u32;
    loop {
        if cnt & 0x10 != 0 {
            // let &(cmd, param) = cmd_iter.next().unwrap();
            // cmd_pio.refresh2(&confirm_cmd);
            // cmd_pio.refresh2(&Command::new(cmd as u8, param));
        }
        // let h = cnt % 16;
        // for c in coloum.iter_mut() {
        //     c[3] = h as u8;
        // }
        {
            // vsync
            // block_for(Duration::from_micros(50));
            // cmd_pio.refresh2(&sync_cmd);
            // block_for(Duration::from_micros(10));
            line.start();
            // need sleep at least 15 micros , or there will be emi problem
            // Timer::after_micros(5).await;
            // block_for(Duration::from_micros(250)); // no emi with 710fps
            // block_for(Duration::from_micros(5)); // no emi with 718fps

            // let &(cmd, param) = cmd_iter.next().unwrap();
            // cmd_pio.refresh2(&confirm_cmd);
            // cmd_pio.refresh2(&Command::new(cmd as u8, param));
            // block_for(Duration::from_micros(120));

            // defmt::info!("[begin] update_frame2");
            // let mut parser = clocks2::ColorParser::new(&mut buf);
            // block_update_frame(&mut parser, cmd_pio, &coloum);
            // async_update_frame(&mut parser, cmd_pio, &coloum).await;
            async_update_frame2(cmd_pio, mbi_rx).await;
            // cmd_pio.refresh2(&sync_cmd);
            // defmt::info!("[end] update_frame2");
            line.wait_stop().await;
            // frame_cnt += 1;
            // frame_cnt %= 16;
        }
        if cnt & 0x10 != 0 {
            led_pin.toggle();
        }
        cnt += 1;
        if cnt % 5_000 == 0 {
            let now = Instant::now();
            let duration_ms = (now - last).as_millis();
            last = now;
            let fps = 5_000 * 1000 / duration_ms;
            defmt::info!("5000 frames in {} ms, {} fps", duration_ms, fps);
        }
    }
}

async fn test_screen_vdrm(
    cmd_pio: &mut clocks2::CmdClock,
    line: &mut clocks2::LineClockHdl,
    mbi_rx: &mut zerocopy_channel::Receiver<'static, NoopRawMutex, MbiBuf2>,
    led_pin: &mut gpio::Output<'_>,
) {
    let mut cnt: usize = 0;
    let mut last = Instant::now();
    loop {
        {
            line.start();
            async_update_frame2(cmd_pio, mbi_rx).await;
            line.wait_stop().await;
        }
        if cnt & 0x10 != 0 {
            led_pin.toggle();
        }
        cnt += 1;
        if cnt % 5_000 == 0 {
            let now = Instant::now();
            let duration_ms = (now - last).as_millis();
            last = now;
            let fps = 5_000 * 1000 / duration_ms;
            defmt::info!("5000 frames in {} ms, {} fps", duration_ms, fps);
        }
    }
}

#[embassy_executor::task]
async fn encode_mbi_vdrm(mut mbi_tx: zerocopy_channel::Sender<'static, NoopRawMutex, MbiBuf2>) {
    const IMG_BIN: &[u8] = include_bytes!("../img.bin");
    assert!(IMG_BIN.len() >= core::mem::size_of::<mbi5264_common::AngleImage>());
    let img = unsafe {
        core::slice::from_raw_parts(
            IMG_BIN.as_ptr() as *const mbi5264_common::AngleImage,
            IMG_BIN.len() / core::mem::size_of::<mbi5264_common::AngleImage>(),
        )
    };
    let mut sync_buf = MbiBuf2::new(0);
    {
        const EMPTY_BUF: [RGBH; IMG_HEIGHT] = [[0, 0, 0, 0]; IMG_HEIGHT];
        let mut parser = clocks2::ColorParser::new(&mut sync_buf.buf);
        let len = update_frame(&mut parser, &EMPTY_BUF);
        sync_buf.len = len;
    }
    let sync_len = sync_buf.len as usize * 2;

    let mut last_angle = 0u32;
    loop {
        for (idx, angle_line) in img.iter().enumerate() {
            let angle = if idx == 0 { u32::MAX } else { last_angle };
            last_angle = angle_line.angle;
            let mbi_buf = mbi_tx.send().await;
            let mut parser = clocks2::ColorParser::new(&mut mbi_buf.buf);
            let len = update_frame(&mut parser, &angle_line.coloum);
            mbi_buf.len = len;
            mbi_buf.angle = angle;
            mbi_tx.send_done();
        }

        let mbi_buf = mbi_tx.send().await;
        mbi_buf.buf[..sync_len].copy_from_slice(&sync_buf.buf[..sync_len]);
        mbi_buf.len = sync_buf.len;
        mbi_buf.angle = last_angle;
        mbi_tx.send_done();
    }
}

#[embassy_executor::task]
async fn encode_mbi_vdrm2(mut mbi_tx: zerocopy_channel::Sender<'static, NoopRawMutex, MbiBuf2>) {
    const IMG_BIN: &[u8] = include_bytes!("../img.bin");
    assert!(IMG_BIN.len() >= core::mem::size_of::<mbi5264_common::AngleImage>());
    let img = unsafe {
        core::slice::from_raw_parts(
            IMG_BIN.as_ptr() as *const mbi5264_common::AngleImage,
            IMG_BIN.len() / core::mem::size_of::<mbi5264_common::AngleImage>(),
        )
    };
    let mut sync_buf = MbiBuf2::new(0);

    let mut coloum = img[32].coloum.clone();
    defmt::info!("fmt coloum");
    for rgbh in coloum.iter_mut() {
        // defmt::info!("{}", rgbh[3]);
        // rgbh[0] = 255;
        // rgbh[1] = 255;
        // rgbh[2] = 255;
        // rgbh[3] = 0;
    }
    defmt::info!("fmt coloum end");
    let mut parser = clocks2::ColorParser::new(&mut sync_buf.buf);
    let len = update_frame(&mut parser, &coloum);
    sync_buf.len = len;

    let sync_len = sync_buf.len as usize * 2;

    let mut last_angle = 0u32;
    loop {
        let mbi_buf = mbi_tx.send().await;
        mbi_buf.buf[..sync_len].copy_from_slice(&sync_buf.buf[..sync_len]);
        mbi_buf.len = sync_buf.len;
        //
        // let mut parser = clocks2::ColorParser::new(&mut mbi_buf.buf);
        // let len = update_frame(&mut parser, &coloum);
        // mbi_buf.len = len;
        mbi_buf.angle = 0;
        mbi_tx.send_done();
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

fn block_update_frame(
    parser: &mut clocks2::ColorParser,
    cmd_pio: &mut clocks2::CmdClock,
    rgbh_coloum: &[RGBH; IMG_HEIGHT],
) {
    let _len = update_frame(parser, rgbh_coloum);
    parser.run(cmd_pio);
}

async fn async_update_frame(
    parser: &mut clocks2::ColorParser<'_>,
    cmd_pio: &mut clocks2::CmdClock,
    rgbh_coloum: &[RGBH; IMG_HEIGHT],
) {
    let len = update_frame(parser, rgbh_coloum);
    cmd_pio.refresh_ptr(parser.buf_ori as u32, len);
    cmd_pio.wait().await;
}

async fn async_update_frame2(
    cmd_pio: &mut clocks2::CmdClock,
    mbi_rx: &mut zerocopy_channel::Receiver<'static, NoopRawMutex, MbiBuf2>,
) {
    let buf = mbi_rx.receive().await;
    cmd_pio.refresh_ptr(buf.buf.as_ptr() as u32, buf.len);
    cmd_pio.wait().await;
    mbi_rx.receive_done();
}

fn update_frame(parser: &mut clocks2::ColorParser, rgbh_coloum: &[RGBH; IMG_HEIGHT]) -> u32 {
    let region0 = &rgbh_coloum[0..64];
    let region1 = &rgbh_coloum[64..128];
    let region2 = &rgbh_coloum[128..];
    // init last_h_mod with 15, so the first line's "empty" is first h_mod
    let mut last_h_mod = 15;
    // FIXME maybe gclk share same sram
    // will cause image brocken if too small
    parser.add_empty(50);
    for line in 0..64usize {
        // defmt::info!("line {}", line);
        // TODO optimize speed
        let p0 = RGBMeta::new(region0[line], 0);
        let p1 = RGBMeta::new(region1[line], 1);
        let p2 = RGBMeta::new(region2[line], 2);
        let mut pixels = [p0, p1, p2];
        bubble_rgbh2(&mut pixels);
        let mut pixel_iter = pixels.iter();
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
            let empty = rgbh_meta.h_mod - last_h_mod - 1;
            parser.add_empty_les(empty as u32);
            last_h_mod = rgbh_meta.h_mod;
        }

        parser.add_color_end(
            &last_solt.buf,
            last_solt.h_div as u32,
            last_solt.last_chip_idx,
        );
    }
    parser.add_empty_les(15 - last_h_mod as u32);
    parser.add_sync(8);
    parser.add_empty(8);
    parser.encode()
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
        let [r, g, b, _h] = rgbh;
        for (i, buf) in (0..8).rev().zip(buf.iter_mut()) {
            let r = (r >> i) & 1;
            let g = (g >> i) & 1;
            let b = (b >> i) & 1;
            let rgb = (r | (g << 1) | (b << 2)) as u16;
            *buf |= rgb << (4 * region);
        }
        Self {
            buf,
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

struct MbiBuf2 {
    angle: u32,
    len: u32,
    buf: [u16; 16384],
}

impl MbiBuf2 {
    const fn new(angle: u32) -> Self {
        Self {
            angle,
            len: 0,
            buf: [0u16; 16384],
        }
    }
}

#[embassy_executor::task]
async fn encode_mbi(
    mut mbi_tx: zerocopy_channel::Sender<'static, NoopRawMutex, MbiBuf2>,
    mut img_rx: zerocopy_channel::Receiver<'static, CriticalSectionRawMutex, ImageBuffer>,
) {
    loop {
        let img_buf = img_rx.receive().await;
        for coloum in img_buf.chunks_exact(IMG_HEIGHT) {
            let buf = mbi_tx.send().await;
            let mut parser = clocks2::ColorParser::new(&mut buf.buf);
            let coloum: &[RGBH; IMG_HEIGHT] = unsafe { core::mem::transmute(coloum.as_ptr()) };
            let len = update_frame(&mut parser, &coloum);
            buf.len = len;
            mbi_tx.send_done();
        }
        img_rx.receive_done();
    }
}

#[embassy_executor::task]
async fn encode_mbi2(mut mbi_tx: zerocopy_channel::Sender<'static, NoopRawMutex, MbiBuf2>) {
    let gray = 0b11111_111_0_00000_00;
    let gray = 255u8;
    let mut img_buf = [[gray, gray, gray, 1]; IMG_SIZE];
    // for (idx, coloum) in img_buf.chunks_exact_mut(IMG_HEIGHT).enumerate() {
    //     for (col, p) in coloum.iter_mut().enumerate() {
    //         let h = (col / 16 + idx) as u8;
    //         let gray = 16 * ((idx as u8) / 3);
    //         // let gray = 128;
    //         *p = [gray, gray, gray, h];
    //     }
    // }
    loop {
        for (angle, coloum) in img_buf.chunks_exact(IMG_HEIGHT).enumerate() {
            let buf = mbi_tx.send().await;
            let mut parser = clocks2::ColorParser::new(&mut buf.buf);
            let coloum: &[RGBH; IMG_HEIGHT] = unsafe { core::mem::transmute(coloum.as_ptr()) };
            let len = update_frame(&mut parser, &coloum);
            buf.len = len;
            buf.angle = angle as u32;
            mbi_tx.send_done();
        }
    }
}
