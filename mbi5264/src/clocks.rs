use core::cell::RefCell;
use embassy_rp::dma::Channel;
use embassy_rp::interrupt::typelevel::{Handler, Interrupt, DMA_IRQ_0, DMA_IRQ_1, PWM_IRQ_WRAP_0};
use embassy_rp::peripherals::{self};
use embassy_rp::peripherals::{DMA_CH0, PIO0, PIO1};
use embassy_rp::pio::program::pio_asm;
use embassy_rp::pio::{self, Pio, ShiftConfig, StateMachine};
use embassy_rp::pwm::{self, Pwm, PwmBatch};
use embassy_rp::{gpio, interrupt, pac, Peripheral, PeripheralRef, Peripherals};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_sync::waitqueue::AtomicWaker;

pub const LE_HIGH: u16 = 1 << 9;

static LINE_CLOCK: Mutex<ThreadModeRawMutex, RefCell<Option<LineClock>>> =
    Mutex::new(RefCell::new(None));

// TODO delete this?
embassy_rp::bind_interrupts!(struct PioIrqs {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
});

embassy_rp::bind_interrupts!(struct PioIrqs1 {
    PIO1_IRQ_0 => pio::InterruptHandler<PIO1>;
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
    PWM_IRQ_WRAP_0 => PwmInterruptHandler;
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

impl Handler<PWM_IRQ_WRAP_0> for PwmInterruptHandler {
    #[link_section = ".data"]
    #[inline(never)]
    unsafe fn on_interrupt() {
        let p = unsafe { Peripherals::steal() };
        let mut dbg_pin = gpio::Output::new(p.PIN_19, gpio::Level::High);
        let line_clock = &LINE_CLOCK as *const Mutex<ThreadModeRawMutex, RefCell<Option<LineClock>>>
            as *mut Mutex<ThreadModeRawMutex, RefCell<Option<LineClock>>>;

        let line_clock: &mut Mutex<ThreadModeRawMutex, RefCell<Option<LineClock>>> =
            core::mem::transmute(line_clock);
        let line = line_clock.get_mut();
        let line = line.get_mut().as_mut().unwrap();
        line.handle_interupt();
        PWM_OFF_SIGNAL.signal(());
        dbg_pin.set_low();
    }
}

enum PwmState {
    Idle,
    Freshing,
    Ending,
}

struct LinePwmConfig {
    ba_cfg: pwm::Config,
}

pub struct LineClock {
    pwm_gclk: Pwm<'static>,
    pwm_c: Pwm<'static>,
    pwm_b: Pwm<'static>,
    pwm_a: Pwm<'static>,
    pwm_cfg: LinePwmConfig,
    started: bool,
    state: PwmState,
    all_batch: PwmBatch,
    // pio_ba: PwmPio,
}
const W: u16 = 160;
impl LineClock {
    pub fn new(
        pwm6: peripherals::PWM_SLICE6,
        pwm7: peripherals::PWM_SLICE7,
        pwm8: peripherals::PWM_SLICE0,
        pwm9: peripherals::PWM_SLICE1,
        gclk_pin: peripherals::PIN_12,
        a_pin: peripherals::PIN_14,
        b_pin: peripherals::PIN_16,
        c_pin: peripherals::PIN_18,
    ) -> LineClockHdl {
        PWM_IRQ_WRAP_0::unpend();
        unsafe {
            PWM_IRQ_WRAP_0::enable();
            DMA_IRQ_0::disable();
            init_dma1();
        };
        // for umini div is 5 w is 100 to get 125khz glck
        let pwm_div = 10.into();

        let w = W;
        let first_line_comp_cnt = 0u16;
        let first_line_comp = w * first_line_comp_cnt;
        let mut gclk_cfg = pwm::Config::default();
        gclk_cfg.divider = pwm_div * 2;
        gclk_cfg.top = w + first_line_comp / 2 - 1;
        gclk_cfg.compare_a = w / 2 + first_line_comp / 2;
        gclk_cfg.enable = false;

        let pwm_gclk = Pwm::new_output_a(pwm6, gclk_pin, gclk_cfg);
        let mut c_cfg = pwm::Config::default();
        c_cfg.divider = pwm_div;
        // let c_ount = 64;
        // must be odd
        let c_ount = 64;
        c_cfg.top = w * c_ount + first_line_comp - w / 2 - 1;
        c_cfg.compare_a = w / 2;
        c_cfg.enable = false;
        let pwm_c = Pwm::new_output_a(pwm9, c_pin, c_cfg);
        embassy_rp::pac::PWM.irq0_inte().modify(|w| w.set_ch1(true));

        let mut ba_cfg = pwm::Config::default();
        ba_cfg.divider = pwm_div;
        ba_cfg.top = w + first_line_comp - 1;
        ba_cfg.compare_a = 3;
        ba_cfg.enable = false;
        let pwm_b = Pwm::new_output_a(pwm8, b_pin, ba_cfg.clone());
        let pwm_a = Pwm::new_output_a(pwm7, a_pin, ba_cfg.clone());

        let pwm_cfg = LinePwmConfig { ba_cfg };
        let mut all_batch: PwmBatch = unsafe { core::mem::transmute(0u32) };

        all_batch.enable(&pwm_gclk);
        all_batch.enable(&pwm_c);
        all_batch.enable(&pwm_b);
        all_batch.enable(&pwm_a);
        let this = Self {
            pwm_gclk,
            pwm_c,
            pwm_b,
            pwm_a,
            pwm_cfg,
            started: false,
            state: PwmState::Idle,
            all_batch,
        };
        LINE_CLOCK.lock(|v| v.borrow_mut().replace(this));
        LineClockHdl { started: false }
    }

    pub fn start(&mut self) {
        self.stop();
        PWM_OFF_SIGNAL.reset();
        self.pwm_b.set_config(&self.pwm_cfg.ba_cfg);

        PwmBatch::set_enabled(true, |batch| {
            *batch = unsafe { core::mem::transmute_copy(&self.all_batch) };
        });

        self.pwm_a.phase_retard();
        self.pwm_a.phase_retard();
        self.started = true;
        self.state = PwmState::Freshing;
    }

    #[inline]
    pub fn stop(&mut self) {
        PwmBatch::set_enabled(false, |batch| {
            *batch = unsafe { core::mem::transmute_copy(&self.all_batch) };
        });
        // self.pio_ba.stop();
        self.started = false;
        self.pwm_gclk.set_counter(0);
        self.pwm_c.set_counter(0);
        self.pwm_b.set_counter(0);
        self.pwm_a.set_counter(0);
    }

    #[inline]
    fn set_pwm_b_high(&mut self) {
        let pwm_div = 5.into();
        let mut ba_cfg = pwm::Config::default();
        ba_cfg.divider = pwm_div;
        let top = u16::MAX;
        ba_cfg.top = top;
        ba_cfg.compare_a = top;
        ba_cfg.enable = true;
        self.pwm_b.set_config(&ba_cfg);
        PwmBatch::set_enabled(false, |batch| {
            batch.enable(&self.pwm_b);
        });
        self.pwm_b.set_counter(0);
    }

    #[inline]
    fn handle_interupt(&mut self) {
        self.stop();
        self.pwm_c.clear_wrapped();
        match self.state {
            PwmState::Idle => {}
            PwmState::Freshing => {
                self.state = PwmState::Ending;
                self.set_pwm_b_high();
            }
            PwmState::Ending => {
                self.state = PwmState::Idle;
            }
        }
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

    #[allow(dead_code)]
    pub fn block_wait_stop(&mut self) {
        while !PWM_OFF_SIGNAL.signaled() {}
    }
}

// 2(loop_size) + 2(empty_size) + 2(cmd_size) + 16(chip bits) x 9(chips)
pub const CMD_BUF_SIZE: usize = 2 + 2 + 2 + 16 * 9;

pub struct CmdClockPins {
    pub r0_pin: peripherals::PIN_0,
    pub g0_pin: peripherals::PIN_1,
    pub b0_pin: peripherals::PIN_2,
    pub r1_pin: peripherals::PIN_3,
    pub g1_pin: peripherals::PIN_4,
    pub b1_pin: peripherals::PIN_5,
    pub r2_pin: peripherals::PIN_6,
    pub g2_pin: peripherals::PIN_7,
    pub b2_pin: peripherals::PIN_8,
    pub le_pin: peripherals::PIN_9,
    pub clk_pin: peripherals::PIN_10,
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

        // delay param
        // clk    data   irq
        // 3      2      2
        // 4      3      2
        // 5      4      2
        // 6      5      3
        let clk_program_data = pio_asm!(
            ".define public DELAY 3",
            ".side_set 1",
            ".wrap_target",
            "nop             side 0b0 [DELAY - 1]", // increase the delay if something get wrong
            "irq 4           side 0b0 [0]",
            "nop             side 0b1 [DELAY]"
            ".wrap",
        );
        let clk_prog = common.load_program(&clk_program_data.program);
        let clk_pin = common.make_pio_pin(pins.clk_pin);
        clk_sm.set_pin_dirs(pio::Direction::Out, &[&clk_pin]);

        let mut cfg = pio::Config::default();
        cfg.clock_divider = 1u8.into();
        let clk_div = cfg.clock_divider;
        cfg.use_program(&clk_prog, &[&clk_pin]);
        clk_sm.set_config(&cfg);

        let data_program_data = pio_asm!(
            ".define public DELAY 2"
            ".define public IRQ_DELAY 2"
            ".wrap_target",
            "mov pins null [DELAY]"
            "out y, 32",    // save loop_cnt to y
            "wait 1 irq 4",
            "wait 1 irq 4 [IRQ_DELAY]", // sync irq
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

        let r1_pin = common.make_pio_pin(pins.r1_pin);
        let g1_pin = common.make_pio_pin(pins.g1_pin);
        let b1_pin = common.make_pio_pin(pins.b1_pin);

        let r2_pin = common.make_pio_pin(pins.r2_pin);
        let g2_pin = common.make_pio_pin(pins.g2_pin);
        let b2_pin = common.make_pio_pin(pins.b2_pin);
        let le_pin = common.make_pio_pin(pins.le_pin);
        data_sm.set_pin_dirs(
            pio::Direction::Out,
            &[
                &r0_pin, &g0_pin, &b0_pin, &r1_pin, &g1_pin, &b1_pin, &r2_pin, &g2_pin, &b2_pin,
                &le_pin,
            ],
        );

        let mut cfg = pio::Config::default();
        cfg.clock_divider = clk_div;
        cfg.use_program(&data_prog, &[]);
        cfg.set_out_pins(&[
            &r0_pin, &g0_pin, &b0_pin, &r1_pin, &g1_pin, &b1_pin, &r2_pin, &g2_pin, &b2_pin,
            &le_pin,
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

    #[inline]
    pub fn encode_cmd(transaction: &super::Command, data_buf: &mut [u16; CMD_BUF_SIZE]) {
        let mut buf_iter = data_buf.iter_mut().skip(6);
        for [r, g, b] in transaction.regs {
            for i in (0..16).rev() {
                let buf = buf_iter.next().unwrap();
                let r = (r >> i) & 1;
                let g = (g >> i) & 1;
                let b = (b >> i) & 1;
                let rgb = r | (g << 1) | (b << 2);
                *buf = rgb | (rgb << 3) | (rgb << 6);
            }
        }
        // set le
        for b in data_buf.iter_mut().rev().take(transaction.cmd as usize) {
            *b |= LE_HIGH;
        }
        let buf: &mut [u32; CMD_BUF_SIZE / 2] = unsafe { core::mem::transmute(data_buf) };
        buf[0] = 0; // loop cnt
        buf[1] = 3 - 3; // empty cnt
        buf[2] = 16 * 9 - 2;
    }

    pub fn refresh2(&mut self, transaction: &super::Command) {
        Self::encode_cmd(transaction, &mut self.data_buf);
        let p = self.data_ch.regs();
        let buf: &[u32; CMD_BUF_SIZE / 2] = unsafe { core::mem::transmute(&self.data_buf) };
        p.trans_count()
            .write_value(pac::dma::regs::ChTransCount(buf.len() as u32));
        p.al3_read_addr_trig().write_value(buf.as_ptr() as u32);
        while p.ctrl_trig().read().busy() {}
    }

    pub fn refresh_ptr(&mut self, ptr: u32, len: u32) {
        let p = self.data_ch.regs();
        p.trans_count()
            .write_value(pac::dma::regs::ChTransCount(len));
        p.al3_read_addr_trig().write_value(ptr);
    }

    #[allow(dead_code)]
    pub fn block_wait(&self) {
        let p = self.data_ch.regs();
        while p.ctrl_trig().read().busy() {}
    }

    pub async fn wait(&mut self) {
        let p = self.data_ch.regs();
        DataTransfer::new(p).await;
    }
}
