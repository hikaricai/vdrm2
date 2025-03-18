use core::cell::RefCell;
use embassy_rp::dma::Channel;
use embassy_rp::interrupt::typelevel::{Handler, Interrupt, DMA_IRQ_0, DMA_IRQ_1, PWM_IRQ_WRAP};
use embassy_rp::peripherals::{self};
use embassy_rp::peripherals::{DMA_CH0, DMA_CH1, PIO0};
use embassy_rp::pio::{self, Pio, ShiftConfig, StateMachine};
use embassy_rp::pwm::{self, Pwm, PwmBatch};
use embassy_rp::{gpio, interrupt, pac, Peripheral, PeripheralRef, Peripherals};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_sync::waitqueue::AtomicWaker;

static LINE_CLOCK: Mutex<CriticalSectionRawMutex, RefCell<Option<LineClock>>> =
    Mutex::new(RefCell::new(None));

// TODO delete this?
embassy_rp::bind_interrupts!(struct PioIrqs {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
});

embassy_rp::bind_interrupts!(struct Dma1Irq {
    DMA_IRQ_1 => Dma1InterruptHandler;
});

static DMA_DATA_CH0_WAKER: AtomicWaker = AtomicWaker::new();

struct Dma1InterruptHandler {}

impl Handler<DMA_IRQ_1> for Dma1InterruptHandler {
    unsafe fn on_interrupt() {
        let ints0 = pac::DMA.ints(0).read();
        pac::DMA.ints(0).write_value(ints0);
        DMA_DATA_CH0_WAKER.wake();
    }
}

unsafe fn init_dma1() {
    DMA_IRQ_1::disable();
    DMA_IRQ_1::set_priority(interrupt::Priority::P3);
    embassy_rp::pac::DMA.inte(1).write_value(0xFFFF);
    DMA_IRQ_1::enable();
}

static PWM_OFF_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

embassy_rp::bind_interrupts!(struct PwmIrq {
    PWM_IRQ_WRAP => PwmInterruptHandler;
});

struct DataTransfer {
    ch: pac::dma::Channel,
}

impl DataTransfer {
    fn new(ch: pac::dma::Channel) -> Self {
        Self { ch }
    }
}

impl core::future::Future for DataTransfer {
    type Output = ();
    fn poll(
        self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Self::Output> {
        // We need to register/re-register the waker for each poll because any
        // calls to wake will deregister the waker.
        DMA_DATA_CH0_WAKER.register(cx.waker());

        if self.ch.ctrl_trig().read().busy() {
            core::task::Poll::Pending
        } else {
            core::task::Poll::Ready(())
        }
    }
}

struct PwmInterruptHandler {}

impl Handler<PWM_IRQ_WRAP> for PwmInterruptHandler {
    unsafe fn on_interrupt() {
        critical_section::with(|cs| {
            let mut guard = LINE_CLOCK.borrow(cs).borrow_mut();
            let line = guard.as_mut().unwrap();
            line.stop();
            line.set_pwm_ba_high();
            line.pwm_c.clear_wrapped();
        });
        PWM_OFF_SIGNAL.signal(());
    }
}

pub struct LineClock {
    pwm_gclk: Pwm<'static>,
    pwm_c: Pwm<'static>,
    pwm_ba: Pwm<'static>,
    started: bool,
}

impl LineClock {
    pub fn new(
        pwm7: peripherals::PWM_SLICE7,
        pwm8: peripherals::PWM_SLICE0,
        pwm1: peripherals::PWM_SLICE1,
        c_pin: peripherals::PIN_14,
        b_pin: peripherals::PIN_16,
        a_pin: peripherals::PIN_17,
        wrong_gclk_pin: peripherals::PIN_0,
        gclk_pin: peripherals::PIN_18,
    ) -> LineClockHdl {
        let _wrong_gclk_pin =
            embassy_rp::gpio::Input::new(wrong_gclk_pin, embassy_rp::gpio::Pull::None);
        let pwm_div = 5.into();
        PWM_IRQ_WRAP::unpend();
        unsafe {
            PWM_IRQ_WRAP::enable();
            DMA_IRQ_0::disable();
            init_dma1();
        };
        let w = 100u16;
        let mut gclk_cfg = pwm::Config::default();
        gclk_cfg.divider = pwm_div;
        gclk_cfg.top = w * 2 - 1;
        gclk_cfg.compare_a = w;

        let pwm_gclk = Pwm::new_output_a(pwm1, gclk_pin, gclk_cfg);
        let mut c_cfg = pwm::Config::default();
        c_cfg.divider = pwm_div;
        // let c_ount = 64;
        let c_ount = 64;
        c_cfg.top = w * c_ount - 50 - 1;
        c_cfg.compare_a = w;
        let pwm_c = Pwm::new_output_a(pwm7, c_pin, c_cfg);
        embassy_rp::pac::PWM.inte().modify(|w| w.set_ch7(true));

        let mut ba_cfg = pwm::Config::default();
        ba_cfg.divider = pwm_div;
        ba_cfg.top = w - 1;
        ba_cfg.compare_a = 3;
        ba_cfg.compare_b = 1;
        let pwm_ba = Pwm::new_output_ab(pwm8, b_pin, a_pin, ba_cfg);

        let this = Self {
            pwm_gclk,
            pwm_c,
            pwm_ba,
            started: false,
        };
        LINE_CLOCK.lock(|v| v.borrow_mut().replace(this));
        LineClockHdl { started: false }
    }

    pub fn start(&mut self) {
        self.stop();
        PWM_OFF_SIGNAL.reset();
        PwmBatch::set_enabled(true, |batch| {
            batch.enable(&self.pwm_gclk);
            batch.enable(&self.pwm_c);
            batch.enable(&self.pwm_ba);
        });
        self.pwm_ba.phase_retard();
        self.started = true;
    }

    pub fn stop(&mut self) {
        self.started = false;
        PwmBatch::set_enabled(false, |batch| {
            batch.enable(&self.pwm_gclk);
            batch.enable(&self.pwm_c);
            batch.enable(&self.pwm_ba);
        });
        self.pwm_gclk.set_counter(0);
        self.pwm_c.set_counter(0);
        self.pwm_ba.set_counter(0);
    }
    fn set_pwm_ba_high(&self) {
        let p = unsafe { Peripherals::steal() };
        let pin_b = gpio::Output::new(p.PIN_16, gpio::Level::High);
        let pin_a = gpio::Output::new(p.PIN_17, gpio::Level::Low);
        PwmBatch::set_enabled(true, |batch| {
            batch.enable(&self.pwm_ba);
        });
        PwmBatch::set_enabled(false, |batch| {
            batch.enable(&self.pwm_ba);
        });
        self.pwm_ba.set_counter(0);

        core::mem::drop(pin_b);
        core::mem::drop(pin_a);
        pac::IO_BANK0.gpio(16).ctrl().write(|w| w.set_funcsel(4));
        pac::IO_BANK0.gpio(17).ctrl().write(|w| w.set_funcsel(4));
    }
}

pub struct LineClockHdl {
    started: bool,
}

impl LineClockHdl {
    pub fn start(&mut self) {
        self.started = true;
        LINE_CLOCK.lock(|v| {
            v.borrow_mut().as_mut().unwrap().start();
        });
        PWM_OFF_SIGNAL.reset();
    }

    pub async fn wait_stop(&mut self) {
        if !self.started {
            return;
        }
        PWM_OFF_SIGNAL.wait().await;
        self.started = false;
    }
}

// 2(loop_size) + 2(empty_size) + 2(cmd_size) + 16(chip bits) x 9(chips)
pub const CMD_BUF_SIZE: usize = 2 + 2 + 2 + 16 * 9;

pub struct CmdClockPins {
    pub clk_pin: peripherals::PIN_1,

    pub r0_pin: peripherals::PIN_2,
    pub g0_pin: peripherals::PIN_3,
    pub b0_pin: peripherals::PIN_4,
    pub le0_pin: peripherals::PIN_5,

    pub r1_pin: peripherals::PIN_6,
    pub g1_pin: peripherals::PIN_7,
    pub b1_pin: peripherals::PIN_8,
    pub le1_pin: peripherals::PIN_9,

    pub r2_pin: peripherals::PIN_10,
    pub g2_pin: peripherals::PIN_11,
    pub b2_pin: peripherals::PIN_12,
    pub le2_pin: peripherals::PIN_13,
}

pub struct CmdClock {
    _clk_sm: StateMachine<'static, PIO0, 0>,
    _data_sm: StateMachine<'static, PIO0, 1>,
    data_ch: PeripheralRef<'static, DMA_CH0>,
    pub data_buf: [u16; CMD_BUF_SIZE],
}

impl CmdClock {
    pub fn new(pio0: PIO0, pins: CmdClockPins, data_ch: DMA_CH0) -> Self {
        let pio0 = Pio::new(pio0, PioIrqs);
        let Pio {
            mut common,
            sm0: mut clk_sm,
            sm1: mut data_sm,
            ..
        } = pio0;

        let clk_program_data = pio_proc::pio_asm!(
            ".side_set 1",
            ".wrap_target",
            "irq 4           side 0b0 [3]", // increase the delay if something get wrong
            "irq 4           side 0b1 [3]",
            ".wrap",
        );
        let clk_prog = common.load_program(&clk_program_data.program);
        let clk_pin = common.make_pio_pin(pins.clk_pin);
        clk_sm.set_pin_dirs(pio::Direction::Out, &[&clk_pin]);

        let mut cfg = pio::Config::default();
        cfg.use_program(&clk_prog, &[&clk_pin]);
        clk_sm.set_config(&cfg);

        let data_program_data = pio_proc::pio_asm!(
            ".define public DELAY 2"
            ".wrap_target",
            "mov pins null [DELAY]"
            "out y, 32",    // save loop_cnt to y
            "wait 1 irq 4",
            "wait 1 irq 4", // sync irq
            "loop:",
            "mov pins null [DELAY]"
            "out x, 32", // save empty_cnt to x, at least 3 empty
            "loop_empty:",
            "mov pins null [DELAY]",
            "jmp x-- loop_empty",
            //
            "mov pins null [DELAY]"
            "out x, 32", // save data_cnt to x, at least 2 data
            "loop_data:",
            "out pins, 16 [DELAY]",
            "jmp x-- loop_data",
            "out pins, 16 [DELAY]", // more one data
            "jmp y-- loop",
            ".wrap",
        );
        let data_prog = common.load_program(&data_program_data.program);
        let r0_pin = common.make_pio_pin(pins.r0_pin);
        let g0_pin = common.make_pio_pin(pins.g0_pin);
        let b0_pin = common.make_pio_pin(pins.b0_pin);
        let le0_pin = common.make_pio_pin(pins.le0_pin);

        let r1_pin = common.make_pio_pin(pins.r1_pin);
        let g1_pin = common.make_pio_pin(pins.g1_pin);
        let b1_pin = common.make_pio_pin(pins.b1_pin);
        let le1_pin = common.make_pio_pin(pins.le1_pin);

        let r2_pin = common.make_pio_pin(pins.r2_pin);
        let g2_pin = common.make_pio_pin(pins.g2_pin);
        let b2_pin = common.make_pio_pin(pins.b2_pin);
        let le2_pin = common.make_pio_pin(pins.le2_pin);
        data_sm.set_pin_dirs(
            pio::Direction::Out,
            &[
                &r0_pin, &g0_pin, &b0_pin, &le0_pin, &r1_pin, &g1_pin, &b1_pin, &r2_pin, &g2_pin,
                &b2_pin,
            ],
        );

        // defmt::info!("data_sm addr {}", data_prog.origin);
        let mut cfg = pio::Config::default();
        cfg.use_program(&data_prog, &[]);
        cfg.set_out_pins(&[
            &r0_pin, &g0_pin, &b0_pin, &le0_pin, &r1_pin, &g1_pin, &b1_pin, &le1_pin, &r2_pin,
            &g2_pin, &b2_pin, &le2_pin,
        ]);
        cfg.fifo_join = pio::FifoJoin::TxOnly;
        cfg.shift_out = ShiftConfig {
            auto_fill: true,
            threshold: 32,
            direction: pio::ShiftDirection::Right,
        };
        data_sm.set_config(&cfg);

        clk_sm.set_enable(true);
        data_sm.set_enable(true);

        let p = data_ch.regs();

        let pio_no = 0;
        let ch_no = 0;
        let data_sm_no = 1usize;
        p.write_addr()
            .write_value(embassy_rp::pac::PIO0.txf(data_sm_no).as_ptr() as u32);
        p.al1_ctrl().write(|w| {
            let mut reg = embassy_rp::pac::dma::regs::CtrlTrig::default();
            // Set TX DREQ for this statemachine
            reg.set_treq_sel(embassy_rp::pac::dma::vals::TreqSel::from(
                pio_no * 8 + data_sm_no as u8,
            ));
            reg.set_data_size(embassy_rp::pac::dma::vals::DataSize::SIZE_WORD);
            reg.set_chain_to(ch_no);
            reg.set_incr_read(true);
            reg.set_incr_write(false);
            reg.set_en(true);
            reg.set_irq_quiet(false);
            *w = reg.0;
        });

        Self {
            _clk_sm: clk_sm,
            _data_sm: data_sm,
            data_ch: data_ch.into_ref(),
            data_buf: [0; CMD_BUF_SIZE],
        }
    }

    pub fn refresh2(&mut self, transaction: &super::Command) {
        let mut buf_iter = self.data_buf.iter_mut().skip(6);
        for [r, g, b] in transaction.regs {
            for i in (0..16).rev() {
                let buf = buf_iter.next().unwrap();
                let r = (r >> i) & 1;
                let g = (g >> i) & 1;
                let b = (b >> i) & 1;
                let rgb = r | (g << 1) | (b << 2);
                *buf = rgb | (rgb << 4) | (rgb << 8);
            }
        }
        // set le
        for b in self
            .data_buf
            .iter_mut()
            .rev()
            .take(transaction.cmd as usize)
        {
            *b |= 0x8;
        }
        let buf: &mut [u32; CMD_BUF_SIZE / 2] = unsafe { core::mem::transmute(&mut self.data_buf) };
        buf[0] = 0; // loop cnt
        buf[1] = 3 - 3; // empty cnt
        buf[2] = 16 * 9 - 2;

        let p = self.data_ch.regs();
        p.trans_count().write_value(buf.len() as u32);
        p.al3_read_addr_trig().write_value(buf.as_ptr() as u32);
        while p.ctrl_trig().read().busy() {}
    }

    pub fn refresh_ptr(&mut self, ptr: u32, len: u32) {
        let p = self.data_ch.regs();
        p.trans_count().write_value(len);
        p.al3_read_addr_trig().write_value(ptr);
    }

    pub fn block_wait(&self) {
        let p = self.data_ch.regs();
        while p.ctrl_trig().read().busy() {}
    }

    pub async fn wait(&mut self) {
        let p = self.data_ch.regs();
        DataTransfer::new(p).await;
    }
}

#[repr(C)]
struct ColorTranser {
    empty_loops: u32,
    data_loops: u32,
    buf: [u16; 8],
}

#[repr(C)]
struct ColorTranserTail {
    empty_loops: u32,
    data_loops: u32,
    buf: [u16; 2],
}

#[repr(C)]
struct TranserMeta {
    empty_loops: u32,
    data_loops: u32,
}

pub struct ColorParser<'a> {
    pub loops: &'a mut u32,
    pub buf: *mut u16,
    pub buf_ori: *mut u16,
    pub cnt: usize,
}

impl<'a> ColorParser<'a> {
    pub fn new(buf: &'a mut [u16]) -> Self {
        let buf_ori = buf.as_mut_ptr();
        let buf = unsafe { buf.as_mut_ptr().add(2) };
        let loops: &mut u32 = unsafe { core::mem::transmute(buf_ori) };
        *loops = 0;
        Self {
            loops,
            buf,
            buf_ori,
            cnt: 0,
        }
    }

    #[inline]
    fn start(&mut self, cmd_pio: &mut CmdClock) {
        let ptr = self.buf_ori as u32;
        let len = unsafe { self.buf.offset_from(self.buf_ori) } as u32 / 2;
        cmd_pio.refresh_ptr(ptr, len);
    }

    #[inline]
    fn reinit(&mut self) {
        *self.loops = 0;
        self.buf = unsafe { self.buf_ori.add(2) };
        self.cnt += 1;
    }

    pub fn run(&mut self, cmd_pio: &mut CmdClock) {
        self.start(cmd_pio);
        cmd_pio.block_wait();
        self.reinit();
    }

    pub fn encode(&mut self) -> u32 {
        *self.loops -= 1;
        let len = unsafe { self.buf.offset_from(self.buf_ori) } as u32 / 2;
        len
    }

    pub async fn async_run(&mut self, cmd_pio: &mut CmdClock) {
        self.start(cmd_pio);
        cmd_pio.wait().await;
        self.reinit();
    }

    pub fn add_empty_les(&mut self, empty_size: u32) {
        if empty_size == 0 {
            return;
        }
        unsafe {
            *self.loops += 1;

            // empty with le
            let meta: &mut ColorTranserTail = add_buf_ptr(&mut self.buf);
            meta.empty_loops = 16 * 9 - 3;
            meta.data_loops = 2 - 2;
            meta.buf = [0, 8];

            // many le
            if empty_size <= 1 {
                return;
            }
            *self.loops += 1;
            let empty_size = empty_size - 1;
            let meta: &mut TranserMeta = add_buf_ptr(&mut self.buf);
            meta.empty_loops = 0;
            meta.data_loops = empty_size * 2 - 2;
            for _ in 0..empty_size {
                let u32_buf: &mut u32 = add_buf_ptr(&mut self.buf);
                *u32_buf = 0x0008_0000;
            }
        }
    }

    pub fn add_color2(&mut self, buf: &[u16; 8], chip_index: u32, last_chip_idx: u32) {
        let le = chip_index == 8;
        let chip_inc_index = chip_index - last_chip_idx;
        let empty_loops = chip_inc_index * 16 + 8 - 3;
        unsafe {
            *self.loops += 1;

            let transfer: &mut ColorTranser = add_buf_ptr(&mut self.buf);
            // -1
            transfer.empty_loops = empty_loops;
            transfer.data_loops = 8 - 2;
            transfer.buf = *buf;
            if le {
                transfer.data_loops += 8;
                let le_buf: &mut [u16; 8] = add_buf_ptr(&mut self.buf);
                *le_buf = [0; 8];
                le_buf[7] = 0x8;
            }
        }
    }

    fn add_empty_le(&mut self, chip_inc_index: u32) {
        unsafe {
            *self.loops += 1;
            let tail: &mut ColorTranserTail = add_buf_ptr(&mut self.buf);
            tail.empty_loops = chip_inc_index * 16 + 8 - 2 - 3;
            tail.data_loops = 2 - 2;
            // LE
            tail.buf[0] = 0;
            tail.buf[1] = 8;
        }
    }

    pub fn add_color_end(&mut self, buf: &[u16; 8], chip_index: u32, last_chip_idx: u32) {
        let le = chip_index == 8;
        self.add_color2(buf, chip_index, last_chip_idx);
        if !le {
            self.add_empty_le(8 - chip_index as u32);
        }
    }

    pub fn add_sync(&mut self, empty_loops: u32) {
        unsafe {
            *self.loops += 1;
            let tail: &mut ColorTranserTail = add_buf_ptr(&mut self.buf);
            tail.empty_loops = empty_loops;
            tail.data_loops = 2 - 2;
            // LE
            tail.buf[0] = 8;
            tail.buf[1] = 8;
        }
    }

    pub fn add_empty(&mut self, empty_loops: u32) {
        if empty_loops == 0 {
            return;
        }
        unsafe {
            *self.loops += 1;
            let tail: &mut ColorTranserTail = add_buf_ptr(&mut self.buf);
            tail.empty_loops = empty_loops + 8 * 1 - 3;
            tail.data_loops = 2 - 2;
            // LE
            tail.buf[0] = 0;
            tail.buf[1] = 0;
        }
    }
}

unsafe fn add_buf_ptr<B, T>(buf: &mut *mut B) -> &mut T {
    let t: &mut T = core::mem::transmute(*buf);
    *buf = buf.add(core::mem::size_of::<T>() / core::mem::size_of::<B>());
    t
}
