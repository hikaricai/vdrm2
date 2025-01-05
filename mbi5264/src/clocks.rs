use critical_section::Mutex;
use embedded_hal::digital::OutputPin;
use embedded_hal::pwm::SetDutyCycle;
use rp2040_hal::dma::{ChannelIndex, DMAExt, SingleChannel};
use rp2040_hal::gpio::bank0::{self, Gpio0, Gpio12, Gpio2, Gpio4, Gpio5};
use rp2040_hal::gpio::{
    DefaultTypeState, DynFunction, DynPinId, DynPullType, FunctionNull, FunctionPio0, FunctionPwm,
    FunctionSioOutput, OutputDriveStrength, Pin, PullDown, PullType,
};
use rp2040_hal::multicore::{Multicore, Stack};
use rp2040_hal::pac::{PPB, PSM};
use rp2040_hal::pio::{PIOBuilder, PIOExt, PinDir, Running, Stopped};
use rp2040_hal::sio::SioFifo;
use rp2040_hal::{pac, Sio};
use rp_pico::pac::{DMA, PIO0, RESETS};
const CMD_STOP_GCLK: u32 = 0x42;
const CMD_RESTART_GCLK: u32 = 0x43;
const RESP_ACK: u32 = 0x11;
const RESP_ONLINE: u32 = 0x12;
use crate::{Command, Outpin};
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
    pub running: bool,
    cnt: u32,
    pwm: rp_pico::hal::pwm::Slices,
    gclk_pin: Option<Pin<Gpio0, FunctionPwm, PullDown>>,
    gpio0_pin: Option<Pin<Gpio0, FunctionSioOutput, PullDown>>,
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
        let pwm_div = 5;
        pwm.pwm0.set_div_int(pwm_div);
        pwm.pwm0.set_top(200 - 1);
        pwm.pwm0.channel_a.set_duty_cycle(100).unwrap();
        // pwm.pwm0.disable_interrupt();

        pwm.pwm1.set_div_int(pwm_div);
        pwm.pwm1.set_top(100 * 64 - 1);
        pwm.pwm1.channel_a.set_duty_cycle(100).unwrap();
        pwm.pwm1.enable_interrupt();

        pwm.pwm2.set_div_int(pwm_div);
        pwm.pwm2.set_top(100 - 1);
        pwm.pwm2.channel_a.set_duty_cycle(3).unwrap();
        pwm.pwm2.channel_b.set_duty_cycle(1).unwrap();

        Self {
            running: false,
            cnt: 0,
            pwm,
            gclk_pin: Some(gclk_pin),
            gpio0_pin: None,
            a_pin,
            b_pin,
            c_pin,
        }
    }

    pub fn start(&mut self) {
        if let Some(gpio0_pin) = self.gpio0_pin.take() {
            self.gclk_pin = Some(gpio0_pin.into_function());
        }
        self.stop();
        self.running = true;
        self.pwm.pwm1.set_top(100 * 64 - 1);
        self.pwm.pwm2.retard_phase();
        self.pwm.enable_simultaneous(0x07);
    }

    pub fn set_gclk(&mut self, state: rp2040_hal::gpio::PinState) {
        let mut pin: Pin<Gpio0, FunctionSioOutput, PullDown> =
            self.gclk_pin.take().unwrap().into_function();
        pin.set_state(state).unwrap();
        self.gpio0_pin = Some(pin);
    }

    pub fn clear_interupte(&mut self) {
        self.pwm.pwm1.clear_interrupt();
    }

    pub fn handle_interrupt(&mut self) {
        // self.pwm.pwm0.clear_interrupt();
        // if !self.pwm.pwm1.has_overflown() {
        //     return;
        // }
        self.pwm.pwm1.clear_interrupt();
        self.cnt += 1;
        // self.pwm.pwm2.clear_interrupt();
        if self.cnt == 32 * 2 - 2 {
            self.pwm.pwm1.set_top(100 * 64 - 1 - 50);
        }
        if self.cnt == 32 * 2 {
            self.cnt = 0;
            self.stop();
        }
    }

    pub fn stop(&mut self) {
        self.running = false;
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
//16(chip bits) x 9(chips) + 2(empty)
pub const CMD_BUF_SIZE: usize = 16 * 9 + 2;
const COLOR_BUF_SIZE: usize = 16 * 9;
pub struct CmdClock {
    le_prog_offset: u32,
    le_sm: rp2040_hal::pio::StateMachine<(PIO0, rp2040_hal::pio::SM3), Running>,
    le_sm_tx: rp2040_hal::pio::Tx<(PIO0, rp2040_hal::pio::SM3)>,
    color_prog_offset: u32,
    color_sm: rp2040_hal::pio::StateMachine<(PIO0, rp2040_hal::pio::SM2), Stopped>,
    data_ch: rp2040_hal::dma::Channel<rp2040_hal::dma::CH0>,
    color_ch: rp2040_hal::dma::Channel<rp2040_hal::dma::CH1>,
    data_buf: [u16; CMD_BUF_SIZE],
    color_buf: [u16; COLOR_BUF_SIZE],
}

impl CmdClock {
    pub fn new(mut pins: CmdClockPins, pio0: PIO0, dma: DMA, resets: &mut RESETS) -> Self {
        let default_strength = pins.r0_pin.get_drive_strength();
        let default_strength_v = match default_strength {
            OutputDriveStrength::TwoMilliAmps => 2,
            OutputDriveStrength::FourMilliAmps => 4,
            OutputDriveStrength::EightMilliAmps => 8,
            OutputDriveStrength::TwelveMilliAmps => 12,
        };
        rprintln!("default_strength {}ma", default_strength_v);
        pins.r0_pin
            .set_drive_strength(OutputDriveStrength::TwelveMilliAmps);
        pins.g0_pin
            .set_drive_strength(OutputDriveStrength::TwelveMilliAmps);
        pins.b0_pin
            .set_drive_strength(OutputDriveStrength::TwelveMilliAmps);
        let dma = dma.dyn_split(resets);
        let data_ch = dma.ch0.unwrap();
        let color_ch = dma.ch1.unwrap();
        let (mut pio, sm0, sm1, sm2, sm3) = pio0.split(resets);
        let (clk_sm, _clk_sm_tx) = {
            let program_data = pio_proc::pio_asm!(
                ".side_set 1",
                ".wrap_target",
                "irq 5           side 0b0",     // 5 first to be faster
                "irq 4           side 0b0 [5]", // increase the delay if something get wrong
                "irq 5           side 0b1",
                "irq 4           side 0b1 [5]",
                ".wrap",
            );
            let installed = pio.install(&program_data.program).unwrap();
            let (mut sm, _, tx) = PIOBuilder::from_installed_program(installed)
                .side_set_pin_base(pins.clk_pin.id().num)
                .clock_divisor_fixed_point(1, 0)
                .autopull(false)
                .build(sm0);
            sm.set_pindirs([(pins.clk_pin.id().num, PinDir::Output)]);
            (sm, tx)
        };

        let (data_sm, data_sm_tx) = {
            let program_data = pio_proc::pio_asm!(
                "out isr, 32", // save loop_cnt to isr
                ".wrap_target",
                "mov x isr ",   // load loop_cnt
                "irq wait 6" // sync clk sm0
                "wait 1 irq 4", // pre wait to clear old irq
                "loop:",
                "wait 1 irq 4",
                "out pins, 16",
                "jmp x-- loop",
                // "wait 1 irq 4", // 再wait一个edge将数据写入 由app多填充一个0u32实现
                // "out pins, 32",
                ".wrap",
            );
            let installed = pio.install(&program_data.program).unwrap();
            let (mut sm, _, mut tx) = PIOBuilder::from_installed_program(installed)
                .out_pins(pins.r0_pin.id().num, 3)
                .clock_divisor_fixed_point(1, 0)
                .autopull(true)
                .buffers(rp2040_hal::pio::Buffers::OnlyTx)
                .build(sm1);
            sm.set_pindirs([
                (pins.r0_pin.id().num, PinDir::Output),
                (pins.g0_pin.id().num, PinDir::Output),
                (pins.b0_pin.id().num, PinDir::Output),
            ]);
            // Configure the width of the screen
            // let one_cmd_loops: u32 = 16 * 9 - 1
            let one_cmd_loops: u32 = 16 * 9 + 2 - 1; // 多2个0u16保证数据都触发
            tx.write(one_cmd_loops);
            (sm, tx)
        };

        let (color_sm, color_sm_tx, color_prog_offset) = {
            let program_data = pio_proc::pio_asm!(
                ".wrap_target",
                "set y 8",   // bigloop cnt is chip_num -1
                "irq wait 6", // sync clk sm0
                "wait 1 irq 4", // pre wait to clear old irq
                "bigloop:"
                "set x 7",
                "loop:",
                "wait 1 irq 4",
                "out pins, 16",
                "jmp x-- loop",
                "set x 7",
                "empty:"
                "wait 1 irq 4",
                "mov pins null",
                "jmp x-- empty",
                "jmp y-- bigloop",
                ".wrap",
            );
            let installed = pio.install(&program_data.program).unwrap();
            let offset = installed.offset() as u32;
            let (mut sm, _, tx) = PIOBuilder::from_installed_program(installed)
                .out_pins(pins.r0_pin.id().num, 3)
                .clock_divisor_fixed_point(1, 0)
                .autopull(true)
                .buffers(rp2040_hal::pio::Buffers::OnlyTx)
                .build(sm2);
            sm.set_pindirs([
                (pins.r0_pin.id().num, PinDir::Output),
                (pins.g0_pin.id().num, PinDir::Output),
                (pins.b0_pin.id().num, PinDir::Output),
            ]);
            (sm, tx, offset)
        };

        let (le_sm, le_sm_tx, le_prog_offset) = {
            let program_data = pio_proc::pio_asm!(
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
            let installed = pio.install(&program_data.program).unwrap();
            let offset = installed.offset() as u32;
            let (mut sm, _, tx) = PIOBuilder::from_installed_program(installed)
                .out_pins(pins.r0_pin.id().num, 3)
                .side_set_pin_base(pins.le_pin.id().num)
                .clock_divisor_fixed_point(1, 0)
                .autopull(true)
                .buffers(rp2040_hal::pio::Buffers::OnlyTx)
                .build(sm3);
            sm.set_pindirs([(pins.le_pin.id().num, PinDir::Output)]);
            (sm, tx, offset)
        };

        pins.into_pio0_pins();
        clk_sm.start();
        data_sm.start();
        // let color_sm = color_sm.start();
        let le_sm = le_sm.start();

        data_ch.ch().ch_al1_ctrl().write(|w| unsafe {
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
                .bits(data_sm_tx.dreq_value())
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
        data_ch
            .ch()
            .ch_write_addr()
            .write(|w| unsafe { w.bits(data_sm_tx.fifo_address() as u32) });

        color_ch.ch().ch_al1_ctrl().write(|w| unsafe {
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
                .bits(color_sm_tx.dreq_value())
                // Turn off interrupts
                .irq_quiet()
                .bit(true)
                // Chain to the channel selecting the framebuffers
                .chain_to()
                .bits(rp2040_hal::dma::CH1::id())
                // Enable the channel
                .en()
                .bit(true)
        });
        color_ch
            .ch()
            .ch_write_addr()
            .write(|w| unsafe { w.bits(color_sm_tx.fifo_address() as u32) });
        let buf = [0; CMD_BUF_SIZE];
        rprintln!(
            "le_prog_offset {} color_prog_offset {}",
            le_prog_offset,
            color_prog_offset
        );
        Self {
            le_prog_offset,
            le_sm,
            le_sm_tx,
            color_sm,
            color_prog_offset,
            data_ch,
            color_ch,
            data_buf: buf,
            color_buf: [0; COLOR_BUF_SIZE],
        }
    }

    pub fn le_end(&self) -> bool {
        let addr = self.le_sm.instruction_address();
        if addr <= self.le_prog_offset + 1 {
            return true;
        }
        false
    }

    pub fn color_end(&self) -> bool {
        // TODO use color sm
        return true;
        let addr = self.color_sm.instruction_address();
        // wait irq 1 command may add pc
        if addr <= self.color_prog_offset + 2 {
            return true;
        }
        false
    }

    pub fn cmd_busy(&self) -> bool {
        self.data_ch.ch().ch_ctrl_trig().read().busy().bit_is_set()
            // | self.color_ch.ch().ch_ctrl_trig().read().busy().bit_is_set()
            // | !self.color_end()
            | !self.le_end()
    }

    pub fn commit(&mut self) -> bool {
        let busy = self.cmd_busy();
        while self.cmd_busy() {}
        busy
    }

    pub fn refresh(&mut self, transaction: &Command) {
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
        let pixels_cnt = self.data_buf.len() as u32 / 2;
        let pixels_addr = self.data_buf.as_ptr() as u32;

        self.data_ch
            .ch()
            .ch_trans_count()
            .write(|w| unsafe { w.bits(pixels_cnt) });
        self.data_ch
            .ch()
            .ch_al3_read_addr_trig()
            .write(|w| unsafe { w.bits(pixels_addr) });

        let le_high_cnt = transaction.cmd as u32;
        let le_low_cnt: u32 = 16 * 9 + 1 - le_high_cnt;
        let le_low_cnt = le_low_cnt - 1;
        let le_high_cnt = le_high_cnt - 1;
        let le_data = (le_high_cnt << 16) | le_low_cnt;
        self.le_sm_tx.write(le_data);
    }

    pub fn refresh_raw_buf(&mut self, buf: &[u16; CMD_BUF_SIZE]) {
        let pixels_cnt = buf.len() as u32 / 2;
        let pixels_addr = buf.as_ptr() as u32;

        self.data_ch
            .ch()
            .ch_trans_count()
            .write(|w| unsafe { w.bits(pixels_cnt) });
        self.data_ch
            .ch()
            .ch_al3_read_addr_trig()
            .write(|w| unsafe { w.bits(pixels_addr) });

        let le_high_cnt = 1;
        let le_low_cnt: u32 = 16 * 9 + 1 - le_high_cnt;
        let le_low_cnt = le_low_cnt - 1;
        let le_high_cnt = le_high_cnt - 1;
        let le_data = (le_high_cnt << 16) | le_low_cnt;
        self.le_sm_tx.write(le_data);
    }

    pub fn refresh_color(&mut self, cmd: &Command) {
        let mut buf_iter = self.color_buf.iter_mut();
        for [r, g, b] in cmd.regs {
            for i in (0..8).rev() {
                let buf = buf_iter.next().unwrap();
                let r = (r >> i) & 1;
                let g = (g >> i) & 1;
                let b = (b >> i) & 1;
                *buf = r + (g << 1) + (b << 2);
                // *buf = 0x03;
            }
        }
        let pixels_cnt = self.color_buf.len() as u32 / 4;
        let pixels_addr = self.color_buf.as_ptr() as u32;

        self.color_ch
            .ch()
            .ch_trans_count()
            .write(|w| unsafe { w.bits(pixels_cnt) });
        self.color_ch
            .ch()
            .ch_al3_read_addr_trig()
            .write(|w| unsafe { w.bits(pixels_addr) });

        let total_cnt = 16 * 9 + 1 - 8; // so color will remain zero
        let le_high_cnt = cmd.cmd as u32;
        let le_low_cnt: u32 = total_cnt - le_high_cnt;
        let le_low_cnt = le_low_cnt - 1;
        let le_high_cnt = le_high_cnt - 1;
        let le_data = (le_high_cnt << 16) | le_low_cnt;
        self.le_sm_tx.write(le_data);
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
            let [r, g, b] = [(y * 16 + x) << 2; 3];
            let [r, g, b] = [0x1555; 3];
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
