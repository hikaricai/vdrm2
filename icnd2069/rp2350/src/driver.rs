use embassy_rp::{
    bind_interrupts,
    gpio::Level,
    peripherals::{DMA_CH0, PIN_10, PIN_13, PIN_15, PIN_16, PIN_18, PIN_20, PIN_22, PIO0},
    pio::{self, program::pio_asm, Common, Pio, ShiftConfig, StateMachine},
    Peripheral, PeripheralRef,
};
use embassy_time::Timer;
use fixed::types::U24F8;

use crate::protocol::{BK_ACTIVE_HIGH, DCLK_HZ, DIN_ONE_SELECTS, PARK};

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

/// PIO0 is exclusively owned by this driver; only SM0 is enabled.
pub struct Driver {
    sm: StateMachine<'static, PIO0, 0>,
    dma: PeripheralRef<'static, DMA_CH0>,
    _common: Common<'static, PIO0>,
}

impl Driver {
    pub fn new(pio: PIO0, dma: DMA_CH0, pins: Pins) -> Self {
        let Pio {
            mut common,
            sm0: mut sm,
            ..
        } = Pio::new(pio, Irqs);
        let le = common.make_pio_pin(pins.le);
        let sin = common.make_pio_pin(pins.sin);
        let dclk = common.make_pio_pin(pins.dclk);
        let lck = common.make_pio_pin(pins.lck);
        let bk = common.make_pio_pin(pins.bk);
        let din = common.make_pio_pin(pins.din);
        let row = common.make_pio_pin(pins.row);

        // Empty FIFO repeats two blank samples, keeping PLL input continuous.
        let program = pio_asm!(
            ".side_set 1",
            ".wrap_target",
            "pull noblock side 0",
            "out pins, 16 side 0",
            "nop          side 1 [1]",
            "out pins, 16 side 0 [1]",
            "nop          side 1 [1]",
            ".wrap",
        );
        let loaded = common.load_program(&program.program);
        let mut cfg = pio::Config::default();
        cfg.use_program(&loaded, &[&dclk]);
        // Convert the ratio, not 150 MHz itself (which exceeds U24F8's range).
        let divider = (embassy_rp::clocks::clk_sys_freq() as u64 * 256) / (4 * DCLK_HZ as u64);
        cfg.clock_divider = U24F8::from_bits(divider as u32);
        cfg.fifo_join = pio::FifoJoin::TxOnly;
        cfg.shift_out = ShiftConfig {
            auto_fill: false,
            threshold: 32,
            direction: pio::ShiftDirection::Right,
        };
        let mut pin_cfg = cfg.get_pins();
        pin_cfg.out_base = 10;
        pin_cfg.out_count = 13;
        // SAFETY: OUT covers GP10..22. Only the seven owned pins above are
        // muxed to PIO and enabled as outputs. Gap pins retain their GPIO mux
        // and direction. No other SM uses this PIO block. GP15 is also inside
        // OUT's range; hardware side-set has priority over OUT on that pin.
        unsafe { cfg.set_pins(pin_cfg) };
        sm.set_config(&cfg);
        let all = [&le, &sin, &dclk, &lck, &bk, &din, &row];
        sm.set_pins(Level::Low, &all);
        if BK_ACTIVE_HIGH {
            sm.set_pins(Level::High, &[&bk]);
        }
        if !DIN_ONE_SELECTS {
            sm.set_pins(Level::High, &[&din]);
        }
        sm.set_pin_dirs(pio::Direction::Out, &all);
        sm.tx().push(PARK as u32 * 0x10001);
        // SAFETY: disabled SM, one owned FIFO word; PULL then MOV X, OSR.
        unsafe {
            sm.exec_instr(0x80a0);
            sm.exec_instr(0xa027);
        }
        sm.set_enable(true);
        Self {
            sm,
            dma: dma.into_ref(),
            _common: common,
        }
    }

    /// Waveform must terminate in PARK (LE/ROW/LCK low, row outputs blanked).
    /// DMA completion is NOT wire completion: drain FIFO + OSR before return.
    pub async fn send(&mut self, words: &[u32]) {
        assert!(!words.is_empty());
        self.sm
            .tx()
            .dma_push(self.dma.reborrow(), words, false)
            .await;
        // Eight FIFO words + one OSR word = at most 18 DCLK = 1.8 us.
        // Round up from DCLK_HZ, so changing that constant retains the drain.
        let drain_us = (18_000_000u64).div_ceil(DCLK_HZ as u64) + 2;
        Timer::after_micros(drain_us).await;
        assert!(self.sm.tx().empty());
    }
}
