use embassy_rp::{
    bind_interrupts,
    gpio::{Drive, Level, Output, SlewRate},
    peripherals::{
        DMA_CH0, PIN_0, PIN_1, PIN_2, PIN_3, PIN_4, PIN_5, PIN_6, PIN_7, PIN_8,
        PIN_9, PIN_10, PIN_11, PIN_12, PIN_13, PIN_14, PIN_15, PIN_16,
        PIN_18, PIN_20, PIN_22, PIO0,
    },
    pio::{self, Common, Pio, ShiftConfig, StateMachine},
    Peripheral, PeripheralRef,
};
use fixed::types::U24F8;

use crate::protocol;
include!(concat!(env!("OUT_DIR"), "/pin_config.rs"));

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
});

pub struct Pins {
    pub r1: PIN_0,
    pub g1: PIN_1,
    pub b1: PIN_2,
    pub sel1: PIN_3,
    pub r2: PIN_4,
    pub g2: PIN_5,
    pub b2: PIN_6,
    pub sel2: PIN_7,
    pub r3: PIN_8,
    pub g3: PIN_9,
    pub b3: PIN_10,
    pub sel3: PIN_11,
    pub lat1: PIN_12,
    pub lat2: PIN_13,
    pub lat3: PIN_14,
    pub dclk: PIN_15,
    pub row: PIN_22,
    pub lck: PIN_16,
    pub lbk: PIN_18,
    pub ldata: PIN_20,
}

/// One pin word drives data, LE, ROW and RT5990; side-set drives DCLK.
pub struct Driver {
    sm: StateMachine<'static, PIO0, 0>,
    dma: PeripheralRef<'static, DMA_CH0>,
    _common: Common<'static, PIO0>,
    divider: u32,
    running: bool,
    program_origin: u8,
    pins: [pio::Pin<'static, PIO0>; 19],
    bk: Option<pio::Pin<'static, PIO0>>,
    drive_strength: Drive,
    _blank: Option<Output<'static>>,
}

impl Driver {
    pub fn new(pio: PIO0, dma: DMA_CH0, pins: Pins) -> Self {
        // Keep BK asserted even while PIO emits idle zeros or changes divider.
        let (mut blank, bk_pin) = if protocol::DISABLE_RT5990 {
            let level = if protocol::BK_ACTIVE_HIGH { Level::High } else { Level::Low };
            (Some(Output::new(pins.lbk, level)), None)
        } else {
            (None, Some(pins.lbk))
        };
        if let Some(blank) = blank.as_mut() {
            blank.set_drive_strength(CFG_PIN_DRIVE_STRENGTH);
            blank.set_slew_rate(PIN_SLEW_RATE);
        }
        let Pio { mut common, sm0: mut sm, .. } = Pio::new(pio, Irqs);
        let mut r1 = common.make_pio_pin(pins.r1);
        let mut g1 = common.make_pio_pin(pins.g1);
        let mut b1 = common.make_pio_pin(pins.b1);
        let mut r2 = common.make_pio_pin(pins.r2);
        let mut g2 = common.make_pio_pin(pins.g2);
        let mut b2 = common.make_pio_pin(pins.b2);
        let mut r3 = common.make_pio_pin(pins.r3);
        let mut g3 = common.make_pio_pin(pins.g3);
        let mut b3 = common.make_pio_pin(pins.b3);
        let mut sel1 = common.make_pio_pin(pins.sel1);
        let mut sel2 = common.make_pio_pin(pins.sel2);
        let mut sel3 = common.make_pio_pin(pins.sel3);
        let mut lat1 = common.make_pio_pin(pins.lat1);
        let mut lat2 = common.make_pio_pin(pins.lat2);
        let mut lat3 = common.make_pio_pin(pins.lat3);
        let mut row = common.make_pio_pin(pins.row);
        let mut dclk = common.make_pio_pin(pins.dclk);
        let mut lck = common.make_pio_pin(pins.lck);
        let mut bk = bk_pin.map(|pin| common.make_pio_pin(pin));
        let mut din = common.make_pio_pin(pins.ldata);
        // make_pio_pin resets pad settings, so configure after registration
        // and before enabling output directions or starting the SM.
        for pin in [
            &mut r1, &mut g1, &mut b1, &mut r2, &mut g2, &mut b2,
            &mut r3, &mut g3, &mut b3, &mut sel1, &mut sel2, &mut sel3,
            &mut lat1, &mut lat2, &mut lat3, &mut row, &mut lck, &mut din, &mut dclk,
        ].into_iter().chain(bk.as_mut()) {
            pin.set_drive_strength(CFG_PIN_DRIVE_STRENGTH);
            pin.set_slew_rate(PIN_SLEW_RATE);
        }
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
        // GP17/19/21 remain unowned despite the contiguous OUT range.
        unsafe { cfg.set_pins(pin_cfg); }
        sm.set_config(&cfg);
        let all = [
            &r1, &g1, &b1, &r2, &g2, &b2, &r3, &g3, &b3,
            &sel1, &sel2, &sel3, &lat1, &lat2, &lat3, &row, &lck, &din, &dclk,
        ];
        sm.set_pins(Level::Low, &all);
        if let Some(bk) = &bk {
            sm.set_pins(Level::Low, &[bk]);
            sm.set_pin_dirs(pio::Direction::Out, &[bk]);
        }
        if !protocol::DISABLE_RT5990 && !protocol::DIN_ONE_SELECTS {
            sm.set_pins(Level::High, &[&din]);
        }
        sm.set_pin_dirs(pio::Direction::Out, &all);
        Self {
            sm, dma: dma.into_ref(), _common: common,
            divider: protocol::PIO_CLKDIV_INT << 8,
            running: false, program_origin: loaded.origin, _blank: blank,
            pins: [
                r1, g1, b1, r2, g2, b2, r3, g3, b3,
                sel1, sel2, sel3, lat1, lat2, lat3, row, lck, din, dclk,
            ],
            bk,
            drive_strength: CFG_PIN_DRIVE_STRENGTH,
        }
    }

    /// Blocks with unchanged rate and drive keep DCLK running.
    /// Rate or drive changes drain the parking tail and stop the SM.
    pub async fn send(&mut self, words: &[u32], configuration: bool) {
        assert!(!words.is_empty());
        assert_eq!(words[words.len() - 1] & (protocol::LE | protocol::ROW), 0);
        let divider = if configuration {
            protocol::CFG_CLKDIV_BITS
        } else {
            protocol::PIO_CLKDIV_INT << 8
        };
        let drive_strength = if configuration {
            CFG_PIN_DRIVE_STRENGTH
        } else {
            PIN_DRIVE_STRENGTH
        };
        let drive_changed = self.drive_strength != drive_strength;
        let start = !self.running || self.divider != divider || drive_changed;
        if start {
            if self.running {
                while !self.sm.tx().empty() {
                    core::hint::spin_loop();
                }
                // All queued trailing words are PARK. No active latch/ROW
                // can be cut short while stopping for a rate/drive change.
                self.sm.set_enable(false);
            }
            self.sm.restart();
            // JMP to the one-time SET X,0, with side-set LOW. Forced
            // instructions execute while disabled, parking DCLK before
            // changing pads/divider; startup reinitializes the idle word.
            unsafe { self.sm.exec_instr(self.program_origin as u16); }
            if drive_changed {
                for pin in self.pins.iter_mut().chain(self.bk.as_mut()) {
                    pin.set_drive_strength(if configuration {
                        CFG_PIN_DRIVE_STRENGTH
                    } else {
                        PIN_DRIVE_STRENGTH
                    });
                }
                if let Some(blank) = self._blank.as_mut() {
                    blank.set_drive_strength(if configuration {
                        CFG_PIN_DRIVE_STRENGTH
                    } else {
                        PIN_DRIVE_STRENGTH
                    });
                }
                self.drive_strength = drive_strength;
            }
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
