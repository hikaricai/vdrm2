use embassy_rp::{
    bind_interrupts,
    gpio::Level,
    peripherals::{DMA_CH0, DMA_CH1, PIN_10, PIN_13, PIN_15, PIN_16, PIN_18, PIN_20, PIN_22, PIO0},
    pio::{self, program::pio_asm, Common, Pio, ShiftConfig, StateMachine},
    Peripheral, PeripheralRef,
};
use fixed::types::U24F8;

use crate::protocol::{BK_ACTIVE_HIGH, DCLK_HZ, DIN_ONE_SELECTS};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
});

pub struct Pins {
    pub le: PIN_10,
    pub sin: PIN_13,
    pub dclk: PIN_15,
    pub lck: PIN_16,
    pub bk: PIN_18,
    pub din: PIN_20,
    pub row: PIN_22,
}

/// SM0 drives CLK/LAT/DATA; SM1 advances OE using SM0's per-clock IRQ.
pub struct Driver {
    sm: StateMachine<'static, PIO0, 0>,
    row_sm: StateMachine<'static, PIO0, 1>,
    dma: PeripheralRef<'static, DMA_CH0>,
    row_dma: PeripheralRef<'static, DMA_CH1>,
    started: bool,
    _common: Common<'static, PIO0>,
}

impl Driver {
    pub fn new(pio: PIO0, dma: DMA_CH0, row_dma: DMA_CH1, pins: Pins) -> Self {
        let Pio {
            mut common,
            sm0: mut sm,
            sm1: mut row_sm,
            ..
        } = Pio::new(pio, Irqs);
        let le = common.make_pio_pin(pins.le);
        let sin = common.make_pio_pin(pins.sin);
        let dclk = common.make_pio_pin(pins.dclk);
        let lck = common.make_pio_pin(pins.lck);
        let bk = common.make_pio_pin(pins.bk);
        let din = common.make_pio_pin(pins.din);
        let row = common.make_pio_pin(pins.row);

        // Twelve 150 MHz cycles per ordinary DCLK; captured extensions add
        // a programmable number of low cycles. IRQ4 synchronizes SM1.
        let program = pio_asm!(
            ".side_set 1",
            ".wrap_target",
            "clock:",
            "nop side 1 [5]",
            "pull ifempty block side 0",
            "irq 4 side 0",
            "out pins, 12 side 0",
            "out y, 4 side 0",
            "delay:",
            "jmp y-- delay side 0",
            "nop side 0",
            ".wrap",
        );
        let loaded = common.load_program(&program.program);
        let mut cfg = pio::Config::default();
        cfg.use_program(&loaded, &[&dclk]);
        // Convert the ratio, not 150 MHz itself (which exceeds U24F8's range).
        // 12 PIO cycles/DCLK: high 6, low 6. At 150 MHz, LAT/SIN
        // change 53.33 ns after rising; IRQ4 is asserted at 46.67 ns.
        let divider = (embassy_rp::clocks::clk_sys_freq() as u64 * 256) / (12 * DCLK_HZ as u64);
        cfg.clock_divider = U24F8::from_bits(divider as u32);
        cfg.fifo_join = pio::FifoJoin::TxOnly;
        cfg.shift_out = ShiftConfig {
            auto_fill: false,
            threshold: 32,
            direction: pio::ShiftDirection::Right,
        };
        let mut pin_cfg = cfg.get_pins();
        pin_cfg.out_base = 10;
        pin_cfg.out_count = 12;
        // SAFETY: OUT covers GP10..21. Only the owned pins above are
        // muxed to PIO and enabled as outputs. Gap pins retain their GPIO mux
        // and direction. SM1 uses a disjoint OUT mapping at GP22.
        // GP15 is also inside
        // OUT's range; hardware side-set has priority over OUT on that pin.
        unsafe { cfg.set_pins(pin_cfg) };
        sm.set_config(&cfg);
        let all = [&le, &sin, &dclk, &lck, &bk, &din];
        sm.set_pins(Level::Low, &all);
        if BK_ACTIVE_HIGH {
            sm.set_pins(Level::High, &[&bk]);
        }
        if !DIN_ONE_SELECTS {
            sm.set_pins(Level::High, &[&din]);
        }
        sm.set_pin_dirs(pio::Direction::Out, &all);
        let row_program = pio_asm!(
            ".wrap_target",
            "pull block",
            "out pins, 1",
            "out x, 31",
            "count:",
            "wait 1 irq 4",
            "jmp x-- count",
            ".wrap",
        );
        let row_loaded = common.load_program(&row_program.program);
        let mut row_cfg = pio::Config::default();
        row_cfg.use_program(&row_loaded, &[]);
        row_cfg.set_out_pins(&[&row]);
        row_cfg.fifo_join = pio::FifoJoin::TxOnly;
        row_cfg.shift_out = ShiftConfig {
            auto_fill: false,
            threshold: 32,
            direction: pio::ShiftDirection::Right,
        };
        row_sm.set_config(&row_cfg);
        row_sm.set_pins(Level::Low, &[&row]);
        row_sm.set_pin_dirs(pio::Direction::Out, &[&row]);
        Self {
            sm,
            row_sm,
            dma: dma.into_ref(),
            row_dma: row_dma.into_ref(),
            started: false,
            _common: common,
        }
    }

    /// Both streams count the same DCLK events, including any FIFO stall.
    pub async fn send(&mut self, words: &[u32], row_runs: &[u32]) {
        assert!(!words.is_empty());
        let skip = if self.started { 0 } else { 4 };
        if !self.started {
            for &word in &words[..skip] { self.sm.tx().push(word); }
            for &word in &row_runs[..skip] { self.row_sm.tx().push(word); }
            self._common.apply_sm_batch(|b| {
                b.restart(&mut self.sm);
                b.restart(&mut self.row_sm);
                b.set_enable(&mut self.row_sm, true);
                b.set_enable(&mut self.sm, true);
            });
            self.started = true;
        }
        embassy_futures::join::join(
            self.sm.tx().dma_push(self.dma.reborrow(), &words[skip..], false),
            self.row_sm.tx().dma_push(self.row_dma.reborrow(), &row_runs[skip..], false),
        ).await;
    }
}
