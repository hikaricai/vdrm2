use embassy_rp::{
    bind_interrupts,
    gpio::Level,
    peripherals::{
        DMA_CH0, PIN_0, PIN_1, PIN_2, PIN_3, PIN_4, PIN_5, PIN_6, PIN_7, PIN_8,
        PIN_9, PIN_10, PIN_11, PIN_13, PIN_15, PIN_16, PIN_17, PIN_18, PIN_19, PIO0,
    },
    pio::{self, Common, Pio, ShiftConfig, StateMachine},
    Peripheral, PeripheralRef,
};
use fixed::types::U24F8;

use crate::protocol;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
});

pub struct Pins {
    pub r1: PIN_5,
    pub g1: PIN_4,
    pub b1: PIN_3,
    pub r2: PIN_11,
    pub g2: PIN_10,
    pub b2: PIN_9,
    pub r3: PIN_2,
    pub g3: PIN_1,
    pub b3: PIN_0,
    pub r4: PIN_8,
    pub g4: PIN_7,
    pub b4: PIN_6,
    pub dclk: PIN_19,
    pub le: PIN_18,
    pub row: PIN_13,
    pub lck: PIN_16,
    pub lbk: PIN_15,
    pub ldata: PIN_17,
}

/// One pin word drives data, LE, ROW and RT5990; side-set drives DCLK.
pub struct Driver {
    sm: StateMachine<'static, PIO0, 0>,
    dma: PeripheralRef<'static, DMA_CH0>,
    _common: Common<'static, PIO0>,
    divider: u32,
    running: bool,
    program_origin: u8,
}

impl Driver {
    pub fn new(pio: PIO0, dma: DMA_CH0, pins: Pins) -> Self {
        let Pio { mut common, sm0: mut sm, .. } = Pio::new(pio, Irqs);
        let r1 = common.make_pio_pin(pins.r1);
        let g1 = common.make_pio_pin(pins.g1);
        let b1 = common.make_pio_pin(pins.b1);
        let r2 = common.make_pio_pin(pins.r2);
        let g2 = common.make_pio_pin(pins.g2);
        let b2 = common.make_pio_pin(pins.b2);
        let r3 = common.make_pio_pin(pins.r3);
        let g3 = common.make_pio_pin(pins.g3);
        let b3 = common.make_pio_pin(pins.b3);
        let r4 = common.make_pio_pin(pins.r4);
        let g4 = common.make_pio_pin(pins.g4);
        let b4 = common.make_pio_pin(pins.b4);
        let le = common.make_pio_pin(pins.le);
        let row = common.make_pio_pin(pins.row);
        let dclk = common.make_pio_pin(pins.dclk);
        let lck = common.make_pio_pin(pins.lck);
        let bk = common.make_pio_pin(pins.lbk);
        let din = common.make_pio_pin(pins.ldata);
        let loaded = common.load_program(&crate::dclk_program::fixed_rate(
            protocol::PIO_CYCLES_PER_DCLK));
        let mut cfg = pio::Config::default();
        cfg.use_program(&loaded, &[&dclk]);
        cfg.clock_divider = U24F8::from_bits(protocol::PIO_CLKDIV_INT << 8);
        cfg.fifo_join = pio::FifoJoin::TxOnly;
        cfg.shift_out = ShiftConfig {
            auto_fill: false,
            threshold: protocol::PIO_OUT_COUNT,
            direction: pio::ShiftDirection::Right,
        };
        let mut pin_cfg = cfg.get_pins();
        pin_cfg.out_base = 0;
        pin_cfg.out_count = protocol::PIO_OUT_COUNT;
        // Only the owned pins are muxed; GP12/14 remain unused.
        unsafe { cfg.set_pins(pin_cfg); }
        sm.set_config(&cfg);
        let all = [
            &r1, &g1, &b1, &r2, &g2, &b2, &r3, &g3, &b3, &r4, &g4, &b4,
            &le, &row, &lck, &bk, &din, &dclk,
        ];
        sm.set_pins(Level::Low, &all);
        if !protocol::DIN_ONE_SELECTS {
            sm.set_pins(Level::High, &[&din]);
        }
        sm.set_pin_dirs(pio::Direction::Out, &all);
        Self {
            sm, dma: dma.into_ref(), _common: common,
            divider: protocol::PIO_CLKDIV_INT << 8,
            running: false, program_origin: loaded.origin,
        }
    }

    /// Same-rate blocks keep DCLK running; empty FIFO emits PARK words.
    /// Only a rate change drains the parking tail and stops the SM.
    pub async fn send(&mut self, words: &[u32], configuration: bool) {
        assert!(!words.is_empty());
        assert_eq!(words[words.len() - 1] & (protocol::LE | protocol::ROW), 0);
        let divider = if configuration {
            protocol::CFG_CLKDIV_BITS
        } else {
            protocol::PIO_CLKDIV_INT << 8
        };
        let start = !self.running || self.divider != divider;
        if start {
            if self.running {
                while !self.sm.tx().empty() {
                    core::hint::spin_loop();
                }
                // All queued trailing words are PARK. No active latch/ROW
                // can be cut short while stopping for the divider change.
                self.sm.set_enable(false);
            }
            self.sm.restart();
            // JMP to the one-time SET X,0, with side-set LOW. Forced
            // instructions execute while disabled, parking DCLK before
            // changing divider; startup then reinitializes the idle word.
            unsafe { self.sm.exec_instr(self.program_origin as u16); }
            self.sm.set_clock_divider(U24F8::from_bits(divider));
            self.sm.clkdiv_restart();
            self.divider = divider;
        }
        let transfer = self.sm.tx().dma_push(self.dma.reborrow(), words, false);
        // The transfer borrows TX. CTRL/FLEVEL are independent registers;
        // this driver exclusively owns SM0 and DMA0.
        if start {
            while embassy_rp::pac::PIO0.flevel().read().0 & 0xf < words.len().min(8) as u32 {
                core::hint::spin_loop();
            }
            embassy_rp::pac::PIO0.ctrl().write(|w| w.set_sm_enable(1));
            self.running = true;
        }
        transfer.await;
    }
}
