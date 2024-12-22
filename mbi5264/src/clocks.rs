use critical_section::Mutex;
use embedded_hal::digital::OutputPin;
use embedded_hal::pwm::SetDutyCycle;
use rp2040_hal::dma::{ChannelIndex, DMAExt, SingleChannel};
use rp2040_hal::gpio::bank0::{self, Gpio0, Gpio2, Gpio4, Gpio5};
use rp2040_hal::gpio::{
    DefaultTypeState, DynFunction, DynPinId, DynPullType, FunctionNull, FunctionPio0, FunctionPwm,
    FunctionSioOutput, Pin, PullDown, PullType,
};
use rp2040_hal::multicore::{Multicore, Stack};
use rp2040_hal::pac::{PPB, PSM};
use rp2040_hal::pio::{PIOBuilder, PIOExt, PinDir, SM0};
use rp2040_hal::sio::SioFifo;
use rp2040_hal::{pac, Sio};
use rp_pico::pac::{DMA, PIO0, RESETS};
const CMD_STOP_GCLK: u32 = 0x42;
const CMD_RESTART_GCLK: u32 = 0x43;
const RESP_ACK: u32 = 0x11;
const RESP_ONLINE: u32 = 0x12;
use crate::{Outpin, Transaction};
use rtt_target::rprintln;
static mut CORE1_STACK: Stack<4096> = Stack::new();
pub struct ClockApp {
    clock: Option<Clock>,
    fifo: SioFifo,
}
impl ClockApp {
    pub fn new(
        clk_pin: Outpin,
        scan_pin: Outpin,
        a_b_c_pin: (Outpin, Outpin, Outpin),
        fifo: SioFifo,
        psm: &mut PSM,
        ppb: &mut PPB,
    ) -> ClockApp {
        let clock = Clock {
            fifo: None,
            clk_pin,
            scan_pin,
            a_b_c_pin,
        };
        let mut this = Self {
            clock: Some(clock),
            fifo,
        };
        this.spawn_second_core(psm, ppb);
        this
    }
    fn spawn_second_core(&mut self, psm: &mut PSM, ppb: &mut PPB) {
        let clock = self.clock.take().unwrap();
        let mut mc = Multicore::new(psm, ppb, &mut self.fifo);
        mc.cores()[1]
            .spawn(unsafe { &mut CORE1_STACK.mem }, move || core1_main(clock))
            .unwrap();
        assert_eq!(self.fifo.read_blocking(), RESP_ONLINE);
    }
}

struct Clock {
    fifo: Option<SioFifo>,
    clk_pin: Outpin,
    scan_pin: Outpin,
    a_b_c_pin: (Outpin, Outpin, Outpin),
}

impl Clock {
    fn run(&mut self) {
        if let Some(fifo) = self.fifo.as_mut() {
            fifo.write_blocking(RESP_ONLINE);
        }
        let mut loop_cnt = 0u32;
        let clk_duty = 10u32;
        let scan_duty = 36u32;
        loop {
            loop_cnt += 1;
            let clk_edges = loop_cnt / clk_duty;
            let clk_pin = clk_edges & 1;
            let clk_cycles = clk_edges >> 1;

            let scan_edges = clk_cycles / scan_duty;
            let scan_pin = scan_edges & 1;

            let a_pin = (scan_edges & 0x3f == 0) as u32;
            let clk_cycles_in_scan = clk_cycles - scan_edges * scan_duty;
            let b_pin = (clk_cycles_in_scan == 2 || clk_cycles_in_scan == 3) as u32;
            let c_pin = (clk_cycles_in_scan == 3) as u32;
            self.clk_pin.set_state((clk_pin > 0).into()).unwrap();
            self.scan_pin.set_state((scan_pin > 0).into()).unwrap();
            self.a_b_c_pin.0.set_state((a_pin > 0).into()).unwrap();
            self.a_b_c_pin.1.set_state((b_pin > 0).into()).unwrap();
            self.a_b_c_pin.2.set_state((c_pin > 0).into()).unwrap();
        }
    }
}

fn core1_main(mut clock: Clock) {
    rprintln!("core1_main");
    let pac = unsafe { pac::Peripherals::steal() };
    let _core = unsafe { pac::CorePeripherals::steal() };
    let sio = Sio::new(pac.SIO);
    // FIXME(eta): why can't I initialise the pins here??
    clock.fifo = Some(sio.fifo);
    clock.run();
    unreachable!();
}

pub struct LineClock {
    pwm: rp_pico::hal::pwm::Slices,
    gclk_pin: Pin<Gpio0, FunctionPwm, PullDown>,
    a_pin: Pin<Gpio2, FunctionPwm, PullDown>,
    b_pin: Pin<Gpio4, FunctionPwm, PullDown>,
    c_pin: Pin<Gpio5, FunctionPwm, PullDown>,
}

impl LineClock {
    pub fn new(
        mut pwm: rp_pico::hal::pwm::Slices,
        gclk_pin: Pin<Gpio0, FunctionNull, PullDown>,
        a_pin: Pin<Gpio2, FunctionNull, PullDown>,
        b_pin: Pin<Gpio4, FunctionNull, PullDown>,
        c_pin: Pin<Gpio5, FunctionNull, PullDown>,
    ) -> Self {
        let gclk_pin = pwm.pwm0.channel_a.output_to(gclk_pin);
        let a_pin = pwm.pwm1.channel_a.output_to(a_pin);
        let b_pin = pwm.pwm2.channel_a.output_to(b_pin);
        let c_pin = pwm.pwm2.channel_b.output_to(c_pin);
        pwm.pwm0.set_div_int(5);
        pwm.pwm0.set_top(200 - 1);
        pwm.pwm0.channel_a.set_duty_cycle(100).unwrap();
        pwm.pwm0.enable_interrupt();

        pwm.pwm1.set_div_int(5);
        pwm.pwm1.set_top(100 * 64 - 1);
        pwm.pwm1.channel_a.set_duty_cycle(100).unwrap();

        pwm.pwm2.set_div_int(5);
        pwm.pwm2.set_top(100 - 1);
        pwm.pwm2.channel_a.set_duty_cycle(3).unwrap();
        pwm.pwm2.channel_b.set_duty_cycle(1).unwrap();
        Self {
            pwm,
            gclk_pin,
            a_pin,
            b_pin,
            c_pin,
        }
    }

    pub fn start(&mut self) {
        self.pwm.pwm2.retard_phase();
        self.pwm.enable_simultaneous(0x07);
    }

    pub fn clear_interupte(&mut self) {
        self.pwm.pwm0.clear_interrupt();
    }

    pub fn stop(&mut self) {
        // self.pwm.enable_simultaneous(0xf8);
        self.pwm.pwm0.disable();
        self.pwm.pwm1.disable();
        self.pwm.pwm2.disable();
        self.pwm.pwm0.set_counter(0);
        self.pwm.pwm1.set_counter(0);
        self.pwm.pwm2.set_counter(0);
    }
}

pub struct CmdClockPins {
    pub clk_pin: Pin<bank0::Gpio6, FunctionNull, PullDown>,
    pub le_pin: Pin<bank0::Gpio7, FunctionNull, PullDown>,
    pub r0_pin: Pin<bank0::Gpio8, FunctionNull, PullDown>,
    pub g0_pin: Pin<bank0::Gpio9, FunctionNull, PullDown>,
    pub b0_pin: Pin<bank0::Gpio10, FunctionNull, PullDown>,
}

impl CmdClockPins {
    fn into_pio0_pins(self) -> [Pin<DynPinId, FunctionPio0, PullDown>; 5] {
        [
            self.clk_pin.into_function::<FunctionPio0>().into_dyn_pin(),
            self.le_pin.into_function::<FunctionPio0>().into_dyn_pin(),
            self.r0_pin.into_function::<FunctionPio0>().into_dyn_pin(),
            self.g0_pin.into_function::<FunctionPio0>().into_dyn_pin(),
            self.b0_pin.into_function::<FunctionPio0>().into_dyn_pin(),
        ]
    }
}
//16(chip bits) x 9(chips) + 2(empty data)
const CMD_BUF_SIZE: usize = 16 * 9 + 2;
pub struct CmdClock {
    clk_sm_tx: rp2040_hal::pio::Tx<(PIO0, SM0)>,
    cmd_ch: rp2040_hal::dma::Channel<rp2040_hal::dma::CH0>,
    buf: [u16; CMD_BUF_SIZE],
}

impl CmdClock {
    pub fn new(pins: CmdClockPins, pio0: PIO0, dma: DMA, resets: &mut RESETS) -> Self {
        let dma = dma.dyn_split(resets);
        let mut cmd_ch = dma.ch0.unwrap();
        let (mut pio, sm0, sm1, _, _) = pio0.split(resets);
        let (clk_sm, clk_sm_tx) = {
            let program_data = pio_proc::pio_asm!(
                ".side_set 1",
                "clk:",
                ".wrap_target",
                "irq 4           side 0b0",
                "jmp !osre le    side 0b0",
                "nop             side 0b0",
                "clk_1:"
                "irq 4           side 0b1",
                "nop             side 0b1",
                "jmp clk         side 0b1",

                "le:",
                "out x, 16       side 0b0",
                "irq 4           side 0b1",
                "out y, 16       side 0b1",
                "wait 1 irq 5    side 0b1", // should not block

                "cmd_clk:"
                "irq 4           side 0b0",
                "jmp x-- ext0    side 0b0",
                "jmp le_up1      side 0b0",
                "ext0:"
                "nop             side 0b0"

                "irq 4           side 0b1",
                "jmp x-- ext1    side 0b1",
                "jmp le_up0      side 0b1",
                "ext1:"
                "jmp cmd_clk     side 0b1",

                "le_up1:"
                "irq 4           side 0b1",
                "set pins 1      side 0b1",
                "jmp y-- le_up0  side 0b1",
                "irq 4           side 0b0",
                "set pins 0      side 0b0",
                "jmp clk_1       side 0b0"

                "le_up0:"
                "irq 4           side 0b0",
                "set pins 1      side 0b0",
                "jmp y-- le_up1  side 0b0",
                "irq 4           side 0b1",
                "set pins 0      side 0b1",
                "nop             side 0b1",
                ".wrap",
            );
            let installed = pio.install(&program_data.program).unwrap();
            let (mut sm, _, tx) = PIOBuilder::from_installed_program(installed)
                .side_set_pin_base(pins.clk_pin.id().num)
                .clock_divisor_fixed_point(3, 0)
                .autopull(false)
                .build(sm0);
            sm.set_pindirs([(pins.clk_pin.id().num, PinDir::Output)]);
            (sm, tx)
        };

        let (cmd_sm, cmd_sm_tx) = {
            let program_data = pio_proc::pio_asm!(
                "out isr, 32", // save loop_cnt to isr
                ".wrap_target",
                "mov x isr ",   // load loop_cnt
                "irq wait 5" // sync clk sm0
                "loop:",
                "wait 1 irq 4",
                "out pins, 16",
                "jmp x-- loop",
                "wait 1 irq 4", // 再wait一个edge将数据写入 由app多填充一个0u32实现
                "out pins, 32",
                ".wrap",
            );
            let installed = pio.install(&program_data.program).unwrap();
            let (mut sm, _, mut tx) = PIOBuilder::from_installed_program(installed)
                .out_pins(pins.r0_pin.id().num, 3)
                .side_set_pin_base(pins.le_pin.id().num)
                .clock_divisor_fixed_point(1, 0)
                .autopull(true)
                .buffers(rp2040_hal::pio::Buffers::OnlyTx)
                .build(sm1);
            sm.set_pindirs([
                (pins.le_pin.id().num, PinDir::Output),
                (pins.r0_pin.id().num, PinDir::Output),
                (pins.g0_pin.id().num, PinDir::Output),
                (pins.b0_pin.id().num, PinDir::Output),
            ]);
            // Configure the width of the screen
            // let one_cmd_loops: u32 = 16 * 9 - 1
            let one_cmd_loops: u32 = 16 * 9 - 1; // 多一个0u16保证数据都触发
            tx.write(one_cmd_loops);
            (sm, tx)
        };

        pins.into_pio0_pins();
        clk_sm.start();
        cmd_sm.start();
        cmd_ch.ch().ch_al1_ctrl().write(|w| unsafe {
            w
                // Increase the read addr as we progress through the buffer
                .incr_read()
                .bit(true)
                // Do not increase the write addr because we always want to write to PIO FIFO
                .incr_write()
                .bit(false)
                // Read 32 bits at a time
                .data_size()
                .size_word()
                // Setup PIO FIFO as data request trigger
                .treq_sel()
                .bits(cmd_sm_tx.dreq_value())
                // Turn off interrupts
                .irq_quiet()
                .bit(true)
                // Chain to the channel selecting the framebuffers
                .chain_to()
                .bits(rp2040_hal::dma::CH0::id())
                // Enable the channel
                .en()
                .bit(true)
        });
        cmd_ch
            .ch()
            .ch_write_addr()
            .write(|w| unsafe { w.bits(cmd_sm_tx.fifo_address() as u32) });
        let buf = [0; CMD_BUF_SIZE];
        Self {
            clk_sm_tx,
            cmd_ch,
            buf,
        }
    }

    pub fn cmd_busy(&self) -> bool {
        self.cmd_ch.ch().ch_ctrl_trig().read().busy().bit_is_set()
    }

    pub fn commit(&mut self) -> bool {
        let busy = self.cmd_busy();
        while self.cmd_busy() {}
        busy
    }

    pub fn refresh(&mut self, transaction: Transaction) {
        let mut buf_iter = self.buf.iter_mut();
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
        let pixels_cnt = self.buf.len() as u32 / 2;
        let pixels_addr = self.buf.as_ptr() as u32;

        self.cmd_ch
            .ch()
            .ch_trans_count()
            .write(|w| unsafe { w.bits(pixels_cnt) });
        self.cmd_ch
            .ch()
            .ch_al3_read_addr_trig()
            .write(|w| unsafe { w.bits(pixels_addr) });
        let clk_total: u32 = 16 * 9;
        let cmd_cnt = transaction.cmd as u32;
        let clk_data = (clk_total << 16) | cmd_cnt;
        self.clk_sm_tx.write(clk_data);
    }
}
