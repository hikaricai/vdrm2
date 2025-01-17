use embassy_rp::gpio::Level;
use embassy_rp::interrupt::typelevel::{Handler, Interrupt, PWM_IRQ_WRAP};
use embassy_rp::pac::PIO0;
use embassy_rp::peripherals::{
    DMA_CH0, PIN_0, PIN_2, PIN_4, PIN_5, PIO0, PWM_SLICE0, PWM_SLICE1, PWM_SLICE2,
};
use embassy_rp::peripherals::{PIN_10, PIN_6, PIN_7, PIN_8, PIN_9};
use embassy_rp::pio::{self, Instance, Pio, StateMachine};
use embassy_rp::pwm::{self, Pwm, PwmBatch};
use embassy_rp::{Peripheral, PeripheralRef};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;

embassy_rp::bind_interrupts!(struct PioIrqs {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
});

embassy_rp::bind_interrupts!(struct PwmIrq {
    PWM_IRQ_WRAP => PwmInterruptHandler;
});

static PWM_CMD_SIGNAL: Signal<CriticalSectionRawMutex, PwmCmd> = Signal::new();
static PWM_OFF_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

enum PwmCmd {
    On,
    Off,
}
struct PwmInterruptHandler {}

impl Handler<PWM_IRQ_WRAP> for PwmInterruptHandler {
    unsafe fn on_interrupt() {
        PWM_IRQ_WRAP::unpend();
        PWM_CMD_SIGNAL.signal(PwmCmd::Off);
        PWM_OFF_SIGNAL.signal(());
    }
}

pub struct LineClock {
    pwm_gclk: Pwm<'static>,
    pwm_a: Pwm<'static>,
    pwm_bc: Pwm<'static>,
    started: bool,
    // pub running: bool,
    // cnt: u32,
    // pwm: rp_pico::hal::pwm::Slices,
    // gclk_pin: Option<Pin<Gpio0, FunctionPwm, PullDown>>,
    // gpio0_pin: Option<Pin<Gpio0, FunctionSioOutput, PullDown>>,
    // a_pin: Pin<Gpio2, FunctionPwm, PullDown>,
    // b_pin: Pin<Gpio4, FunctionPwm, PullDown>,
    // c_pin: Pin<Gpio5, FunctionPwm, PullDown>,
}

impl LineClock {
    pub fn new(
        pwm0: PWM_SLICE0,
        pwm1: PWM_SLICE1,
        pwm2: PWM_SLICE2,
        gclk_pin: PIN_0,
        a_pin: PIN_2,
        b_pin: PIN_4,
        c_pin: PIN_5,
    ) -> Self {
        let pwm_div = 5.into();
        PWM_IRQ_WRAP::unpend();
        unsafe {
            PWM_IRQ_WRAP::enable();
        };
        let mut gclk_cfg = pwm::Config::default();
        gclk_cfg.divider = pwm_div;
        gclk_cfg.top = 200 - 1;
        gclk_cfg.compare_a = 100;
        let pwm_gclk = Pwm::new_output_a(pwm0, gclk_pin, gclk_cfg);

        let mut a_cfg = pwm::Config::default();
        a_cfg.divider = pwm_div;
        a_cfg.top = 100 * 64 - 1;
        a_cfg.compare_a = 100;
        let pwm_a = Pwm::new_output_a(pwm1, a_pin, a_cfg);

        let mut bc_cfg = pwm::Config::default();
        bc_cfg.divider = pwm_div;
        bc_cfg.top = 100 - 1;
        bc_cfg.compare_a = 3;
        bc_cfg.compare_a = 1;
        let pwm_bc = Pwm::new_output_ab(pwm2, b_pin, c_pin, bc_cfg);

        Self {
            pwm_gclk,
            pwm_a,
            pwm_bc,
            started: false,
        }
    }

    pub fn start(&mut self) {
        self.stop();
        PWM_OFF_SIGNAL.reset();
        PwmBatch::set_enabled(true, |batch| {
            batch.enable(&self.pwm_gclk);
            batch.enable(&self.pwm_a);
            batch.enable(&self.pwm_bc);
        });
        self.pwm_bc.phase_retard();
        self.started = true;
    }

    pub fn stop(&mut self) {
        self.started = false;
        PwmBatch::set_enabled(false, |batch| {
            batch.enable(&self.pwm_gclk);
            batch.enable(&self.pwm_a);
            batch.enable(&self.pwm_bc);
        });
        self.pwm_gclk.set_counter(0);
        self.pwm_a.set_counter(0);
        self.pwm_bc.set_counter(0);
    }

    pub async fn wait(&mut self) {
        if !self.started {
            return;
        }
        PWM_OFF_SIGNAL.wait().await;
        self.stop();
    }

    pub async fn run_bg(mut self) {
        loop {
            let cmd = PWM_CMD_SIGNAL.wait().await;
            match cmd {
                PwmCmd::On => {
                    self.start();
                }
                PwmCmd::Off => {
                    self.stop();
                }
            }
        }
    }
}

pub struct LineClockHdl {
    started: bool,
}

impl LineClockHdl {
    pub fn start(&mut self) {
        PWM_CMD_SIGNAL.signal(PwmCmd::On);
        self.started = true;
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

//16(chip bits) x 9(chips) + 2(empty)
pub const CMD_BUF_SIZE: usize = 16 * 9 + 2;
const COLOR_BUF_SIZE: usize = 16 * 9;
const ONE_COMMAND_LOOPS: u32 = CMD_BUF_SIZE as u32 - 1; // 多2个0u16保证数据都触发

pub struct CmdClockPins {
    pub clk_pin: PIN_6,
    pub le_pin: PIN_7,
    pub r0_pin: PIN_8,
    pub g0_pin: PIN_9,
    pub b0_pin: PIN_10,
}

pub struct CmdClock {
    clk_sm: StateMachine<'static, PIO0, 0>,
    data_sm: StateMachine<'static, PIO0, 1>,
    data_ch: PeripheralRef<'static, DMA_CH0>,
    le_prog_offset: u8,
    le_sm: StateMachine<'static, PIO0, 3>,
    data_buf: [u16; CMD_BUF_SIZE],
    // le_sm_tx: rp2040_hal::pio::Tx<(PIO0, rp2040_hal::pio::SM3)>,
    // color_prog_offset: u32,
    // color_sm: rp2040_hal::pio::StateMachine<(PIO0, rp2040_hal::pio::SM2), Stopped>,
    // data_sm_tx: rp2040_hal::pio::Tx<(PIO0, rp2040_hal::pio::SM1)>,
    // data_ch: rp2040_hal::dma::Channel<rp2040_hal::dma::CH0>,
    // color_ch: rp2040_hal::dma::Channel<rp2040_hal::dma::CH1>,
    // data_buf: [u16; CMD_BUF_SIZE],
    // color_buf: [u16; COLOR_BUF_SIZE],
}

impl CmdClock {
    pub fn new(pio0: PIO0, pins: CmdClockPins, data_ch: DMA_CH0) -> Self {
        let pio0 = Pio::new(pio0, PioIrqs);
        let Pio {
            mut common,
            sm0: mut clk_sm,
            sm1: mut data_sm,
            sm2: _,
            sm3: mut le_sm,
            ..
        } = pio0;

        let clk_program_data = pio_proc::pio_asm!(
            ".side_set 1",
            ".wrap_target",
            "irq 5           side 0b0",     // 5 first to be faster
            "irq 4           side 0b0 [5]", // increase the delay if something get wrong
            "irq 5           side 0b1",
            "irq 4           side 0b1 [5]",
            ".wrap",
        );
        let clk_prog = common.load_program(&clk_program_data.program);
        let clk_pin = common.make_pio_pin(pins.clk_pin);
        clk_sm.set_pin_dirs(pio::Direction::Out, &[&clk_pin]);

        let mut cfg = pio::Config::default();
        cfg.use_program(&clk_prog, &[&clk_pin]);
        clk_sm.set_config(&cfg);

        let data_program_data = pio_proc::pio_asm!(
            ".wrap_target",
            "out x, 32", // save loop_cnt to isr
            "irq wait 6" // sync clk sm0
            "wait 1 irq 4", // pre wait to clear old irq
            "loop:",
            "wait 1 irq 4",
            "out pins, 16",
            // TODO try loop with !OSRE
            "jmp x-- loop",
            // "wait 1 irq 4", // 再wait一个edge将数据写入 由app多填充一个0u32实现
            // "out pins, 32",
            ".wrap",
        );
        let data_prog = common.load_program(&data_program_data.program);
        let r0_pin = common.make_pio_pin(pins.r0_pin);
        let g0_pin = common.make_pio_pin(pins.g0_pin);
        let b0_pin = common.make_pio_pin(pins.b0_pin);
        data_sm.set_pin_dirs(pio::Direction::Out, &[&r0_pin, &g0_pin, &b0_pin]);

        let mut cfg = pio::Config::default();
        cfg.use_program(&data_prog, &[]);
        cfg.set_out_pins(&[&r0_pin, &g0_pin, &b0_pin]);
        data_sm.set_config(&cfg);

        let le_program_data = pio_proc::pio_asm!(
            ".side_set 2",
            ".wrap_target"
            "out x, 16 side 0",
            "out y, 16 side 0",
            "wait 1 irq 6 side 0b0", // sync
            "wait 1 irq 5 side 0b0",
            "loop0:",
            "wait 1 irq 5 side 0b0",
            "jmp x-- loop0 side 0b0",
            "loop1:",
            "wait 1 irq 5 side 0b1",
            "jmp y-- loop1 side 0b1",
            ".wrap",
        );
        let le_prog = common.load_program(&le_program_data.program);
        let le_prog_offset = le_prog.origin;
        let le_pin = common.make_pio_pin(pins.le_pin);
        // le_sm.set_pins(Level::High, &[&le_pin]);
        le_sm.set_pin_dirs(pio::Direction::Out, &[&le_pin]);

        let mut cfg = pio::Config::default();
        cfg.use_program(&le_prog, &[&le_pin]);
        le_sm.set_config(&cfg);

        Self {
            clk_sm,
            data_sm,
            data_ch: data_ch.into_ref(),
            le_prog_offset,
            le_sm,
            data_buf: [0; CMD_BUF_SIZE],
        }
    }

    pub async fn refresh(&mut self, transaction: &super::Command) {
        let mut buf_iter = self.data_buf.iter_mut();
        for [r, g, b] in transaction.regs {
            for i in (0..16).rev() {
                let buf = buf_iter.next().unwrap();
                let r = (r >> i) & 1;
                let g = (g >> i) & 1;
                let b = (b >> i) & 1;
                *buf = r + (g << 1) + (b << 2);
                // *buf = 0x03;
            }
        }

        let data_tx = self.data_sm.tx();
        data_tx.push(ONE_COMMAND_LOOPS);
        let data_ch = self.data_ch.reborrow();
        let dma_task = data_tx.dma_push(data_ch, &self.data_buf);

        let le_high_cnt = 1;
        let le_low_cnt: u32 = 16 * 9 + 1 - le_high_cnt;
        let le_low_cnt = le_low_cnt - 1;
        let le_high_cnt = le_high_cnt - 1;
        let le_data = (le_high_cnt << 16) | le_low_cnt;
        let le_tx = self.le_sm.tx();
        le_tx.push(le_data);
        dma_task.await;
        self.wait_le_end();
    }

    pub async fn refresh_raw_buf(&mut self, buf: &[u16; CMD_BUF_SIZE]) {
        let data_tx = self.data_sm.tx();
        data_tx.push(ONE_COMMAND_LOOPS);
        let data_ch = self.data_ch.reborrow();
        let dma_task = data_tx.dma_push(data_ch, buf);

        let le_high_cnt = 1;
        let le_low_cnt: u32 = 16 * 9 + 1 - le_high_cnt;
        let le_low_cnt = le_low_cnt - 1;
        let le_high_cnt = le_high_cnt - 1;
        let le_data = (le_high_cnt << 16) | le_low_cnt;
        let le_tx = self.le_sm.tx();
        le_tx.push(le_data);
        dma_task.await;
        self.wait_le_end();
    }
    pub async fn refresh_empty_buf(&mut self) {
        static BUF: [u16; 2] = [0; 2];
        let data_tx = self.data_sm.tx();
        data_tx.push(2 - 1);
        let data_ch = self.data_ch.reborrow();
        let dma_task = data_tx.dma_push(data_ch, &BUF);

        let le_high_cnt = 1;
        let le_low_cnt: u32 = 1 + 1 - le_high_cnt;
        let le_low_cnt = le_low_cnt - 1;
        let le_high_cnt = le_high_cnt - 1;
        let le_data = (le_high_cnt << 16) | le_low_cnt;
        let le_tx = self.le_sm.tx();
        le_tx.push(le_data);

        dma_task.await;
        self.wait_le_end();
    }
    fn wait_le_end(&self) {
        while self.le_end() {}
    }
    fn le_end(&self) -> bool {
        let addr = embassy_rp::pac::PIO0.sm(3).addr().read().addr();
        if addr <= self.le_prog_offset + 1 {
            return true;
        }
        false
    }
}

pub const fn gen_colors_raw_buf() -> [[u16; CMD_BUF_SIZE]; 1024] {
    let mut data_buf: [[u16; CMD_BUF_SIZE]; 1024] = [[0; CMD_BUF_SIZE]; 1024];
    let mut idx = 0usize;
    let mut x = 0u16;
    let mut y = 0u16;
    let mut i = 0usize;
    let mut j = 0usize;
    loop {
        // for y in 0..64
        x = 0;
        loop {
            // for x in 0..16
            let color = (x % 4) * 0x155;
            // let [r, g, b] = [(y * 16 + x) << 2; 3];
            let [r, g, b] = [color; 3];
            let buf = &mut data_buf[idx];
            let mut pixel_idx = 0usize;
            j = 0;
            loop {
                // for j in 0..9
                i = 15;
                loop {
                    // for i in (0..16).rev
                    let r = (r >> i) & 1;
                    let g = (g >> i) & 1;
                    let b = (b >> i) & 1;
                    let color = r + (g << 1) + (b << 2);
                    buf[pixel_idx] = color;
                    pixel_idx += 1;
                    if i == 0 {
                        break;
                    }
                    i -= 1;
                }
                j += 1;
                if j >= 9 {
                    break;
                }
            }
            idx += 1;
            x += 1;
            if x >= 16 {
                break;
            }
        }
        y += 1;
        if y >= 64 {
            break;
        }
    }
    return data_buf;
}

pub fn gen_raw_buf(regs: [u16; 3]) -> [u16; CMD_BUF_SIZE] {
    let mut buf = [0; CMD_BUF_SIZE];
    let mut buf_iter = buf.iter_mut();
    let [r, g, b] = regs;
    for _chip in 0..9 {
        for i in (0..16).rev() {
            let buf = buf_iter.next().unwrap();
            let r = (r >> i) & 1;
            let g = (g >> i) & 1;
            let b = (b >> i) & 1;
            *buf = r + (g << 1) + (b << 2);
        }
    }
    buf
}
