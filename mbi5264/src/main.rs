#![no_std]
#![no_main]
mod clocks2;
mod consts;
mod core1;
use embassy_executor::Spawner;
use embassy_rp::gpio::{self, Input};
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    zerocopy_channel,
};
use embassy_time::{block_for, Duration, Instant, Timer};
use panic_rtt_target as _;
use static_cell::{ConstStaticCell, StaticCell};
// use panic_probe as _;
const FRAME_SYNC_ANGLE: u32 = u32::MAX;
const IMG_WIDTH: usize = mbi5264_common::IMG_WIDTH;
const IMG_HEIGHT: usize = mbi5264_common::IMG_HEIGHT;
const IMG_SIZE: usize = mbi5264_common::IMG_WIDTH * mbi5264_common::IMG_HEIGHT;
type RGBH = [u8; 4];
type ImageBuffer = [mbi5264_common::AngleImage; mbi5264_common::IMG_HEIGHT];
const TOTAL_ANGLES: u32 = consts::TOTAL_ANGLES as u32;
const TOTAL_MIRRORS: u32 = 8;
const ANGLES_PER_MIRROR: u32 = TOTAL_ANGLES / TOTAL_MIRRORS;
const DBG_INTREVAL: usize = 20;

static MBI_BUF: ConstStaticCell<[MbiBuf2; 2]> =
    ConstStaticCell::new([MbiBuf2::new(0), MbiBuf2::new(0)]);
static MBI_CHANNEL: StaticCell<zerocopy_channel::Channel<'_, NoopRawMutex, MbiBuf2>> =
    StaticCell::new();
static MOTOR_SYNC_SIGNAL: StaticCell<embassy_sync::signal::Signal<NoopRawMutex, SyncState>> =
    StaticCell::new();
// static MOTOR_SYNC_SIGNAL: embassy_sync::signal::Signal<NoopRawMutex, ()> =
//     embassy_sync::signal::Signal::new();

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

struct SyncSignal {
    pin: Input<'static>,
    is_mock: bool,
}

impl SyncSignal {
    async fn wait_sync(&mut self) {
        if self.is_mock {
            Timer::after(Duration::from_millis(1000 / 8)).await;
            return;
        }
        self.pin.wait_for_any_edge().await;
    }
}

struct SyncState {
    last_tick: Instant,
    ticks_per_angle: u32,
    angle_offset: i32,
}

#[embassy_executor::task]
async fn motor_input_sync(
    mut sync_signal: SyncSignal,
    mut rtt_down: rtt_target::DownChannel,
    motor_sync_sinal: &'static embassy_sync::signal::Signal<NoopRawMutex, SyncState>,
) {
    let mut cnt = 0usize;
    sync_signal.wait_sync().await;
    let mut last_sync_tick = Instant::now();
    let mut rtt_read_buf = [0u8; 16];
    let mut angle_offset = 33i32;
    loop {
        if cnt % DBG_INTREVAL == 0 {
            let read_len = rtt_down.read(&mut rtt_read_buf);
            for &c in rtt_read_buf.iter().take(read_len) {
                let deta = match c {
                    b'1' => -3,
                    b'2' => 3,
                    _ => 0,
                };
                angle_offset += deta;
            }
            rtt_target::rprintln!("angle_offset {}", angle_offset);
        }

        sync_signal.wait_sync().await;
        let now = Instant::now();
        let elapsed = now - last_sync_tick;
        let ticks_per_angle = elapsed.as_ticks() as u32 / TOTAL_ANGLES;
        motor_sync_sinal.signal(SyncState {
            last_tick: now,
            ticks_per_angle,
            angle_offset,
        });
        if cnt % DBG_INTREVAL == 0 {
            let fps = 10000 / elapsed.as_millis() as u32;
            rtt_target::rprintln!(
                "sync_signal ticks_per_angle {} fps {}.{}",
                ticks_per_angle,
                fps / 10,
                fps % 10
            );
        }
        last_sync_tick = now;
        cnt += 1;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let rtt = rtt_target::rtt_init_default!();
    let mut rtt_down: rtt_target::DownChannel = rtt.down.0;
    rtt_target::set_print_channel(rtt.up.0);
    rtt_target::rprintln!("Hello there!");
    let core_num = embassy_rp::pac::SIO.cpuid().read();
    rtt_target::rprintln!("main core {}", core_num);
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

    let motor_sync_sinal = MOTOR_SYNC_SIGNAL.init(embassy_sync::signal::Signal::new());
    let motor_sync_sinal = &*motor_sync_sinal;
    let sync_signal = SyncSignal {
        pin: Input::new(p.PIN_28, gpio::Pull::None),
        is_mock: true,
    };
    spawner
        .spawn(motor_input_sync(sync_signal, rtt_down, motor_sync_sinal))
        .unwrap();
    // let channel = IMG_CHANNEL.init(zerocopy_channel::Channel::new(BUF.take()));
    // let (sender, img_rx) = channel.split();
    // let img_tx = SafeSender { sender };
    // core1::spawn_usb_core1(p.CORE1, p.USB, img_tx);
    let mbi_channel = MBI_CHANNEL.init(zerocopy_channel::Channel::new(MBI_BUF.take()));
    let (mbi_tx, mut mbi_rx) = mbi_channel.split();
    // spawner.spawn(encode_mbi(mbi_tx, img_rx)).unwrap();
    // spawner.spawn(encode_mbi_vdrm(mbi_tx)).unwrap();
    spawner.spawn(encode_mbi_vdrm(mbi_tx)).unwrap();
    // spawner.spawn(encode_mbi3(mbi_tx)).unwrap();

    let mut led_pin = gpio::Output::new(p.PIN_25, gpio::Level::Low);
    let pins = clocks2::CmdClockPins {
        r0_pin: p.PIN_0,
        g0_pin: p.PIN_1,
        b0_pin: p.PIN_2,
        r1_pin: p.PIN_3,
        g1_pin: p.PIN_4,
        b1_pin: p.PIN_5,
        r2_pin: p.PIN_6,
        g2_pin: p.PIN_7,
        b2_pin: p.PIN_8,
        le_pin: p.PIN_9,
        clk_pin: p.PIN_10,
    };
    let data_ch = p.DMA_CH0;
    let mut line = clocks2::LineClock::new(
        p.PWM_SLICE6,
        p.PWM_SLICE7,
        p.PWM_SLICE0,
        p.PWM_SLICE1,
        p.PIN_12,
        p.PIN_14,
        p.PIN_16,
        p.PIN_18,
    );
    let mut cmd_pio = clocks2::CmdClock::new(p.PIO0, pins, data_ch);
    let confirm_cmd = Command::new_confirm();
    let mut cnt: usize = 0;
    let cmd_iter = UMINI_CMDS.iter();

    for &(cmd, param) in cmd_iter {
        cmd_pio.refresh2(&confirm_cmd);
        cmd_pio.refresh2(&Command::new(cmd as u8, param));
    }

    // test_screen_normal(&mut cmd_pio, &mut line, &mut led_pin);
    // test_screen_vdrm(&mut cmd_pio, &mut line, &mut mbi_rx, &mut led_pin).await;
    // rtt_target::rprintln!("first sync_signal");
    // let mut cmd_iter = core::iter::repeat(UMINI_CMDS.iter()).flatten();
    let mut dbg_pin = gpio::Output::new(p.PIN_22, gpio::Level::Low);
    loop {
        // let &(cmd, param) = cmd_iter.next().unwrap();
        // cmd_pio.refresh2(&confirm_cmd);
        // cmd_pio.refresh2(&Command::new(cmd as u8, param));
        let mut total_frames = 0u32;
        let mut overflow_frames = 0u32;
        let mut late_frames = 0u32;
        let mut fast_frames = 0u32;
        let mut fresh_frames = 0u32;
        let mut fast_angles = 0u32;
        let SyncState {
            last_tick,
            ticks_per_angle,
            angle_offset,
        } = motor_sync_sinal.wait().await;
        let mut mbi_frames = 0u32;
        let dbg = cnt % DBG_INTREVAL == 0;
        loop {
            total_frames += 1;
            dbg_pin.set_high();
            let mbi_buf = mbi_rx.receive().await;
            dbg_pin.set_low();
            if mbi_buf.angle == FRAME_SYNC_ANGLE {
                mbi_frames += 1;
                line.start();
                cmd_pio.refresh_ptr(mbi_buf.buf.as_ptr() as u32, mbi_buf.len as u32);
                cmd_pio.wait().await;
                mbi_rx.receive_done();
                line.wait_stop().await;
                break;
            }
            let angle_offset = 0;
            let img_angle = (mbi_buf.angle as i32 + angle_offset) as u32;

            let now = Instant::now();
            let cur_angle = (now.as_ticks() - last_tick.as_ticks()) as u32 / ticks_per_angle;

            let offset = ANGLES_PER_MIRROR / 2;
            let show_angle = cur_angle + (TOTAL_ANGLES / 4) - offset;
            let show_angle = cur_angle + 190;
            let show_angle_max = ANGLES_PER_MIRROR + 190;

            if cur_angle >= ANGLES_PER_MIRROR || img_angle >= show_angle_max {
                // rtt_target::rprintln!("break");
                overflow_frames += 1;
                mbi_rx.receive_done();
                continue;
            }
            // if img_angle < show_angle || img_angle > show_angle + ANGLES_PER_MIRROR / 2 {
            if img_angle < show_angle {
                if dbg {
                    rtt_target::rprintln!("late img {} show {}", img_angle, show_angle);
                }
                late_frames += 1;
                mbi_rx.receive_done();
                continue;
            }
            let inc_angles = img_angle - show_angle;
            if dbg {
                rtt_target::rprintln!("fresh img {} show {}", img_angle, show_angle);
            }
            if inc_angles > 0 {
                fast_frames += 1;
                fast_angles += inc_angles;
                let target_angle = cur_angle + inc_angles;
                let expires =
                    last_tick + Duration::from_ticks((target_angle * ticks_per_angle) as u64);
                Timer::at(expires).await;
            }

            fresh_frames += 1;
            line.start();
            cmd_pio.refresh_ptr(mbi_buf.buf.as_ptr() as u32, mbi_buf.len as u32);
            cmd_pio.wait().await;
            mbi_rx.receive_done();
            line.wait_stop().await;
        }
        if dbg {
            rtt_target::rprintln!("mbi_frames {}", mbi_frames);
            rtt_target::rprintln!("late_frames {}", late_frames);
            rtt_target::rprintln!("overflow_frames {}", overflow_frames);
            rtt_target::rprintln!("fast_frames {}", fast_frames);
            rtt_target::rprintln!("fresh_frames {}", fresh_frames);
            rtt_target::rprintln!("fast_angles {}", fast_angles);
            rtt_target::rprintln!("total_frames {}\n", total_frames);
        }
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

            // rtt_target::rprintln!("[begin] update_frame2");
            // let mut parser = clocks2::ColorParser::new(&mut buf);
            // block_update_frame(&mut parser, cmd_pio, &coloum);
            // async_update_frame(&mut parser, cmd_pio, &coloum).await;
            async_update_frame2(cmd_pio, mbi_rx).await;
            // cmd_pio.refresh2(&sync_cmd);
            // rtt_target::rprintln!("[end] update_frame2");
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
            rtt_target::rprintln!("5000 frames in {} ms, {} fps", duration_ms, fps);
        }
    }
}

fn test_screen_normal(
    cmd_pio: &mut clocks2::CmdClock,
    line: &mut clocks2::LineClockHdl,
    led_pin: &mut gpio::Output<'_>,
) {
    let mut cnt: usize = 0;
    let mut last = Instant::now();
    let gray = 0xff00u16;
    let mut color_latch = Command::new(1, 0);
    color_latch.regs[0] = [0, 0, 0];
    let mut color_buf: [u16; clocks2::CMD_BUF_SIZE] = [0; clocks2::CMD_BUF_SIZE];
    clocks2::CmdClock::encode_cmd(&color_latch, &mut color_buf);

    color_latch.regs[0] = [0x0000, gray, 0];
    let mut low_color_buf: [u16; clocks2::CMD_BUF_SIZE] = [0; clocks2::CMD_BUF_SIZE];
    clocks2::CmdClock::encode_cmd(&color_latch, &mut low_color_buf);

    let empty_latch = Command::new(1, 0);
    let mut empty_buf: [u16; clocks2::CMD_BUF_SIZE] = [0; clocks2::CMD_BUF_SIZE];
    clocks2::CmdClock::encode_cmd(&empty_latch, &mut empty_buf);

    let sync_cmd = Command::new_sync();

    let u32_buf_len = clocks2::CMD_BUF_SIZE as u32 / 2;
    loop {
        {
            cmd_pio.refresh2(&sync_cmd);
            for i in 0..16 {
                line.start();
                line.block_wait_stop();
            }
            for i in 0..64 {
                for j in 0..16 {
                    let ptr = if i == 0 && j == 0 {
                        color_buf.as_ptr() as u32
                    } else {
                        empty_buf.as_ptr() as u32
                    };
                    cmd_pio.refresh_ptr(ptr, u32_buf_len);
                    cmd_pio.block_wait();
                }
            }
        }
        if cnt & 0x10 != 0 {
            led_pin.toggle();
        }
        cnt += 1;
        if cnt % 5_00 == 0 {
            let now = Instant::now();
            let duration_ms = (now - last).as_millis();
            last = now;
            let fps = 5_00 * 1000 / duration_ms;
            rtt_target::rprintln!("500 frames in {} ms, {} fps", duration_ms, fps);
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
            rtt_target::rprintln!("5000 frames in {} ms, {} fps", duration_ms, fps);
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
    rtt_target::rprintln!("total angles {}", img.len());
    rtt_target::rprintln!("first angle {}", img[0].angle);
    let mut sync_buf = MbiBuf2::new(0);
    {
        const EMPTY_BUF: [RGBH; IMG_HEIGHT] = [[0, 0, 0, 0]; IMG_HEIGHT];
        let mut parser = clocks2::ColorParser::new(&mut sync_buf.buf);
        let len = update_frame(&mut parser, &EMPTY_BUF);
        sync_buf.len = len;
    }
    let sync_len = sync_buf.len as usize * 2;

    let mut last_angle = 0u32;
    const INDEX_MOD: usize = 4;
    let mut index_mod = 0usize;

    let p = unsafe { embassy_rp::Peripherals::steal() };
    let mut dbg_pin20 = gpio::Output::new(p.PIN_20, gpio::Level::Low);
    let mut dbg_pin21 = gpio::Output::new(p.PIN_21, gpio::Level::Low);
    loop {
        index_mod += 1;
        index_mod %= INDEX_MOD;
        let iter = img.iter().enumerate().filter_map(|(idx, angle_line)| {
            (index_mod == idx % INDEX_MOD).then(|| (idx / INDEX_MOD, angle_line))
        });
        for (idx, angle_line) in iter {
            let angle = if idx == 0 {
                FRAME_SYNC_ANGLE
            } else {
                last_angle
            };
            last_angle = angle_line.angle;
            dbg_pin20.set_high();
            dbg_pin21.set_high();
            let mbi_buf = mbi_tx.send().await;
            dbg_pin20.set_low();
            let mut parser = clocks2::ColorParser::new(&mut mbi_buf.buf);
            let len = update_frame(&mut parser, &angle_line.coloum);
            dbg_pin21.set_low();
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
    // parser.add_empty(5000);
    for line in 0..64usize {
        // rtt_target::rprintln!("line {}", line);
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
            // assume data is optimized
            unreachable!();

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
    //
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
            *buf |= rgb << (3 * region);
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
            *buf |= rgb << (3 * region);
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
    for (idx, coloum) in img_buf.chunks_exact_mut(IMG_HEIGHT).enumerate() {
        for (col, p) in coloum.iter_mut().enumerate() {
            let h = (col / 16 + idx) as u8;
            let gray = 16 * ((idx as u8) / 3);
            // let gray = 128;
            *p = [gray, gray, gray, h];
        }
    }
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
