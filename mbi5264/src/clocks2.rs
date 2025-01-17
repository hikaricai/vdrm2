use embassy_rp::gpio::Level;
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_rp::peripherals::{PIN_10, PIN_6, PIN_7, PIN_8, PIN_9};
use embassy_rp::pio::{self, InterruptHandler, Pio, StateMachine};
use embassy_rp::{Peripheral, PeripheralRef};
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
    le_prog_offset: u32,
    le_sm: StateMachine<'static, PIO0, 3>,
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
    pub fn new(pio0: Pio<'static, PIO0>, pins: CmdClockPins, data_ch: DMA_CH0) -> Self {
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
        let le_prog_offset = le_prog.origin as u32;
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
        }
    }

    pub async fn refresh_raw_buf(&mut self, buf: &[u16; CMD_BUF_SIZE]) {
        let data_tx = self.data_sm.tx();
        data_tx.push(ONE_COMMAND_LOOPS);
        let data_ch = self.data_ch.reborrow();
        let dam_task = data_tx.dma_push(data_ch, buf);

        let le_high_cnt = 1;
        let le_low_cnt: u32 = 16 * 9 + 1 - le_high_cnt;
        let le_low_cnt = le_low_cnt - 1;
        let le_high_cnt = le_high_cnt - 1;
        let le_data = (le_high_cnt << 16) | le_low_cnt;
        let le_tx = self.le_sm.tx();
        le_tx.push(le_data);
        dam_task.await
    }
}
