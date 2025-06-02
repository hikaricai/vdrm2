#![no_std]
#![no_main]
mod clocks;
mod consts;
mod core1;
mod encoder;

use embassy_executor::InterruptExecutor;
use embassy_executor::Spawner;
use embassy_rp::gpio::{self, Input};
use embassy_rp::interrupt;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, zerocopy_channel};
use embassy_time::{Duration, Instant, Timer};
use panic_rtt_target as _;
use static_cell::StaticCell;

// use panic_probe as _;
const IMG_HEIGHT: usize = mbi5264_common::IMG_HEIGHT;
type RGBH = [u8; 4];
type ImageBuffer = [mbi5264_common::AngleImage; mbi5264_common::IMG_HEIGHT];
const TOTAL_ANGLES: u32 = consts::TOTAL_ANGLES as u32;
const TOTAL_MIRRORS: u32 = 8;
const ANGLES_PER_MIRROR: u32 = TOTAL_ANGLES / TOTAL_MIRRORS;
const DBG_INTREVAL: usize = 20;
// const SHOW_ANGLE_MAX: u32 = ANGLES_PER_MIRROR + 190;

static MOTOR_SYNC_SIGNAL: StaticCell<
    embassy_sync::signal::Signal<CriticalSectionRawMutex, SyncState>,
> = StaticCell::new();
// static CUR_ANGLE: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
static DBG: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

static SW1_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

#[interrupt]
unsafe fn SWI_IRQ_1() {
    SW1_EXECUTOR.on_interrupt()
}

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
    last: Instant,
}

impl SyncSignal {
    fn new(pin: Input<'static>, is_mock: bool) -> Self {
        Self {
            pin,
            is_mock,
            last: Instant::now(),
        }
    }
    async fn wait_sync(&mut self) {
        if self.is_mock {
            Timer::at(self.last + Duration::from_millis(1000 / 32)).await;
            self.last = Instant::now();
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
    motor_sync_sinal: &'static embassy_sync::signal::Signal<CriticalSectionRawMutex, SyncState>,
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
        let ticks_per_angle = elapsed.as_ticks() as u32 * TOTAL_MIRRORS / TOTAL_ANGLES;
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
    let rtt = rtt_target::rtt_init! {
        up: {
            0: {
                size: 1024 * 8,
                name: "Terminal"
            }
        }
        down: {
            0: {
                size: 16,
                name: "Terminal"
            }
        }
    };
    let rtt_down: rtt_target::DownChannel = rtt.down.0;
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
    let sync_signal = SyncSignal::new(Input::new(p.PIN_28, gpio::Pull::None), true);

    let spawner1 = SW1_EXECUTOR.start(interrupt::SWI_IRQ_1);
    spawner1
        .spawn(motor_input_sync(sync_signal, rtt_down, motor_sync_sinal))
        .unwrap();

    // let channel = IMG_CHANNEL.init(zerocopy_channel::Channel::new(BUF.take()));
    // let (sender, img_rx) = channel.split();
    // let img_tx = SafeSender { sender };
    // core1::spawn_usb_core1(p.CORE1, p.USB, img_tx);
    // let mbi_channel = MBI_CHANNEL.init(zerocopy_channel::Channel::new(MBI_BUF.take()));
    // let (mbi_tx, mut mbi_rx) = mbi_channel.split();
    // spawner.spawn(encode_mbi(mbi_tx, img_rx)).unwrap();
    // spawner.spawn(encode_mbi_vdrm(mbi_tx)).unwrap();
    // let encode_sinal = ENCODE_SIGNAL.init(embassy_sync::signal::Signal::new());
    // let encode_sinal = &*encode_sinal;

    // spawner
    //     .spawn(encoder::encode_mbi_vdrm(mbi_tx, encode_sinal))
    //     .unwrap();
    // spawner.spawn(encode_mbi3(mbi_tx)).unwrap();

    let mut led_pin = gpio::Output::new(p.PIN_25, gpio::Level::Low);
    let pins = clocks::CmdClockPins {
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
    let mut line = clocks::LineClock::new(
        p.PWM_SLICE6,
        p.PWM_SLICE7,
        p.PWM_SLICE0,
        p.PWM_SLICE1,
        p.PIN_12,
        p.PIN_14,
        p.PIN_16,
        p.PIN_18,
    );
    let mut cmd_pio = clocks::CmdClock::new(p.PIO0, pins, data_ch);
    let confirm_cmd = Command::new_confirm();
    let mut cnt: usize = 0;
    let cmd_iter = UMINI_CMDS.iter();

    for &(cmd, param) in cmd_iter {
        cmd_pio.refresh2(&confirm_cmd);
        cmd_pio.refresh2(&Command::new(cmd as u8, param));
    }

    // test_screen(&mut cmd_pio, &mut line, &mut led_pin).await;
    // return;
    // rtt_target::rprintln!("first sync_signal");
    // let mut cmd_iter = core::iter::repeat(UMINI_CMDS.iter()).flatten();
    let mut dbg_pin = gpio::Output::new(p.PIN_22, gpio::Level::Low);
    // encode_sinal.signal(0);
    let mut encoder = encoder::Encoder::new();
    let mut dma_buf = encoder.encode_next(0);
    loop {
        // let &(cmd, param) = cmd_iter.next().unwrap();
        // cmd_pio.refresh2(&confirm_cmd);
        // cmd_pio.refresh2(&Command::new(cmd as u8, param));
        let mut total_frames = 0u32;
        let mut late_frames = 0u32;
        let mut fast_frames = 0u32;
        let mut fast_angles = 0u32;
        let SyncState {
            last_tick,
            ticks_per_angle,
            angle_offset,
        } = motor_sync_sinal.wait().await;
        let dbg = cnt % DBG_INTREVAL == 0;
        DBG.store(dbg, core::sync::atomic::Ordering::Relaxed);

        loop {
            total_frames += 1;
            dbg_pin.set_high();
            line.start();
            cmd_pio.refresh_ptr(dma_buf.ptr, dma_buf.len);

            let now = Instant::now();
            let cur_angle = (now.as_ticks() - last_tick.as_ticks()) as u32 / ticks_per_angle;

            if cur_angle >= ANGLES_PER_MIRROR {
                dma_buf = encoder.encode_next(0);
                cmd_pio.wait().await;
                line.wait_stop().await;
                break;
            }

            // let offset = ANGLES_PER_MIRROR / 2;
            // let show_angle = cur_angle + (TOTAL_ANGLES / 4) - offset;
            let show_angle = cur_angle + 190;
            dma_buf = encoder.encode_next(show_angle);
            let img_angle = (dma_buf.img_angle as i32 + angle_offset) as u32;
            if dbg {
                // rtt_target::rprintln!("encode img {} show {}", img_angle, show_angle);
            }
            dbg_pin.set_low();

            let now = Instant::now();
            let cur_angle = (now.as_ticks() - last_tick.as_ticks()) as u32 / ticks_per_angle;
            let show_angle = cur_angle + 190;

            if img_angle < show_angle {
                // if dbg {
                //     rtt_target::rprintln!("late img {} show {}", img_angle, show_angle);
                // }
                late_frames += 1;
            }
            let inc_angles = img_angle - show_angle;
            // if dbg {
            //     rtt_target::rprintln!("fresh img {} show {}", img_angle, show_angle);
            // }
            if img_angle > show_angle {
                fast_frames += 1;
                fast_angles += inc_angles;
                // let target_angle = cur_angle + inc_angles;
                // let expires =
                //     last_tick + Duration::from_ticks((target_angle * ticks_per_angle) as u64);
                // Timer::at(expires).await;
            }
            cmd_pio.wait().await;
            line.wait_stop().await;
        }
        if dbg {
            rtt_target::rprintln!("late_frames {}", late_frames);
            rtt_target::rprintln!("fast_frames {}", fast_frames);
            rtt_target::rprintln!("fast_angles {}", fast_angles);
            rtt_target::rprintln!("total_frames {}\n", total_frames);
        }
        if cnt & 0x1 == 0 {
            led_pin.toggle();
        }
        cnt += 1;
    }
}

#[allow(unused)]
async fn test_screen(
    cmd_pio: &mut clocks::CmdClock,
    line: &mut clocks::LineClockHdl,
    led_pin: &mut gpio::Output<'static>,
) {
    let mut encoder = encoder::Encoder::new();
    let mut next_angle = 0u32;
    let mut dma_buf = encoder.encode_next(next_angle);
    let mut cnt = 0usize;
    let mut last = Instant::now();
    loop {
        line.start();
        cmd_pio.refresh_ptr(dma_buf.ptr, dma_buf.len);
        next_angle = if next_angle == dma_buf.img_angle {
            0
        } else {
            dma_buf.img_angle
        };
        dma_buf = encoder.encode_next(next_angle);
        cmd_pio.wait().await;
        line.wait_stop().await;
        cnt += 1;
        if cnt & 0xFFF == 0 {
            let now = Instant::now();
            let ms = (now - last).as_millis() as u32;
            last = now;
            let fps = 0xFFF * 1024 / ms;
            rtt_target::rprintln!("fps {}", fps);
            led_pin.toggle();
        }
    }
}
