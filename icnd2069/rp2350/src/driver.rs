use embassy_rp::{
    bind_interrupts,
    gpio::{Drive, Level, Output, SlewRate},
    peripherals::{
        DMA_CH0, DMA_CH1, PIN_10, PIN_13, PIN_15, PIN_16, PIN_18, PIO0,
    },
    pio::{self, Common, Pio, ShiftConfig, StateMachine},
    Peripheral, PeripheralRef,
};
#[cfg(any(icnd2069_dev, icnd2069_hdl))]
use embassy_rp::peripherals::{PIN_20, PIN_22};
#[cfg(icnd2069_hdl)]
use embassy_rp::peripherals::{
    PIN_0, PIN_1, PIN_2, PIN_3, PIN_4, PIN_5, PIN_6, PIN_7, PIN_8,
    PIN_9, PIN_11, PIN_12, PIN_14, PIN_17,
};
#[cfg(icnd2069_direct)]
use embassy_rp::peripherals::{
    PIN_0, PIN_1, PIN_2, PIN_3, PIN_4, PIN_5, PIN_6, PIN_7, PIN_8,
    PIN_9, PIN_11, PIN_17, PIN_19,
};
use fixed::types::U24F8;

use crate::protocol;
include!(concat!(env!("OUT_DIR"), "/pin_config.rs"));
pub struct Transfer<'a> {
    first: &'a [u32],
    blocks: [[u32; 2]; protocol::GROUPS + 3],
    count: usize,
}

impl<'a> Transfer<'a> {
    pub fn new(words: &'a [u32], layout: protocol::FrameLayout,
        range: core::ops::Range<usize>) -> Self
    {
        assert!(range.start < range.end && words.len() == layout.stored());
        let mut parts = layout.ranges(range.start, range.end);
        let first = &words[parts.next().unwrap()];
        let mut transfer = Self { first, blocks: [[0; 2]; protocol::GROUPS + 3], count: 0 };
        for part in core::iter::once(first).chain(parts.map(|part| &words[part])) {
            transfer.blocks[transfer.count] = [part.len() as u32, part.as_ptr() as u32];
            transfer.count += 1;
        }
        // Zero count/address sentinel terminates the device-side chain.
        transfer.count += 1;
        transfer
    }
}

#[cfg(feature = "dma-diagnostics")]
pub fn diagnose_rtt_mode() {
    let changed = core::cell::Cell::new(false);
    rtt_target::with_terminal_channel(|terminal| {
        if terminal.mode() == rtt_target::ChannelMode::BlockIfFull {
            terminal.set_mode(rtt_target::ChannelMode::NoBlockSkip);
            changed.set(true);
        }
    });
    if changed.get() {
        rtt_target::rprintln!("RTT_DIAG host_mode=2 changed_to=0 before_runtime_print");
    }
}

#[cfg(feature = "dma-diagnostics")]
fn report_stall(stage: &str, elapsed_ms: u64, words: &[u32]) {
    diagnose_rtt_mode();
    let dma = embassy_rp::pac::DMA;
    let pio = embassy_rp::pac::PIO0;
    rtt_target::rprintln!(
        "DMA_DIAG stage={} elapsed_ms={} words={} source={:#010x} head={:#010x} \
         dma_ctrl={:#010x} count={:#010x} read={:#010x} write={:#010x} \
         irq_raw={:#010x} irq_pending={:#010x} irq_enable={:#010x} \
         pio_ctrl={:#010x} fifo={:#010x} fdebug={:#010x} pc={}",
        stage, elapsed_ms, words.len(), words.as_ptr() as usize, words[0],
        dma.ch(0).ctrl_trig().read().0, dma.ch(0).trans_count().read().0,
        dma.ch(0).read_addr().read(), dma.ch(0).write_addr().read(),
        dma.intr(0).read(), dma.ints(0).read(), dma.inte(0).read(),
        pio.ctrl().read().0, pio.flevel().read().0, pio.fdebug().read().0,
        pio.sm(0).addr().read().0,
    );
}

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
});

#[cfg(icnd2069_hdl)]
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
    pub dclk2: PIN_17,
    pub row: PIN_22,
    pub lck: PIN_16,
    pub lbk: PIN_18,
    pub ldata: PIN_20,
}

#[cfg(icnd2069_dev)]
pub struct Pins {
    pub sin: PIN_13,
    pub le: PIN_10,
    pub dclk: PIN_15,
    pub row: PIN_22,
    pub lck: PIN_16,
    pub lbk: PIN_18,
    pub ldata: PIN_20,
}

#[cfg(icnd2069_direct)]
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

const SM0_PIN_COUNT: usize = if protocol::DEV_BOARD { 6 }
    else if protocol::DIRECT_BOARD { 17 } else { 19 };
const PIN_COUNT: usize = SM0_PIN_COUNT + protocol::HDL_BOARD as usize;
const DCLK_INDEX: usize = SM0_PIN_COUNT - 1;
const DIN_INDEX: usize = SM0_PIN_COUNT - 2;
const SM_MASK: u8 = if protocol::HDL_BOARD { 0b11 } else { 0b01 };

/// SM0 drives data/control and DCLK; HDL also uses SM1 for the GP17 clock copy.
pub struct Driver {
    sm: StateMachine<'static, PIO0, 0>,
    #[cfg(icnd2069_hdl)]
    clock_sm: StateMachine<'static, PIO0, 1>,
    _dma: PeripheralRef<'static, DMA_CH0>,
    _control_dma: PeripheralRef<'static, DMA_CH1>,
    _common: Common<'static, PIO0>,
    divider: u32,
    running: bool,
    program_origin: u8,
    pins: [pio::Pin<'static, PIO0>; PIN_COUNT],
    bk: Option<pio::Pin<'static, PIO0>>,
    drive_strength: Drive,
    slew_rate: SlewRate,
    _blank: Option<Output<'static>>,
}

impl Driver {
    pub fn new(pio: PIO0, dma: DMA_CH0, control_dma: DMA_CH1, pins: Pins) -> Self {
        // Keep BK asserted even while PIO emits idle zeros or changes divider.
        let (mut blank, bk_pin) = if protocol::DISABLE_RT5990 {
            let level = if protocol::BK_ACTIVE_HIGH { Level::High } else { Level::Low };
            (Some(Output::new(pins.lbk, level)), None)
        } else {
            (None, Some(pins.lbk))
        };
        if let Some(blank) = blank.as_mut() {
            let selected = PIN_DRIVE_MASK & protocol::BK != 0;
            blank.set_drive_strength(if selected { CFG_PIN_DRIVE_STRENGTH } else { Drive::_4mA });
            blank.set_slew_rate(if selected { CFG_PIN_SLEW_RATE } else { SlewRate::Slow });
        }
        let pio = Pio::new(pio, Irqs);
        let mut common = pio.common;
        let mut sm = pio.sm0;
        #[cfg(icnd2069_hdl)]
        let mut clock_sm = pio.sm1;
        #[cfg(icnd2069_dev)]
        let mut owned_pins = [
            common.make_pio_pin(pins.sin), common.make_pio_pin(pins.le),
            common.make_pio_pin(pins.row), common.make_pio_pin(pins.lck),
            common.make_pio_pin(pins.ldata), common.make_pio_pin(pins.dclk),
        ];
        #[cfg(icnd2069_direct)]
        let mut owned_pins = [
            common.make_pio_pin(pins.r1), common.make_pio_pin(pins.g1), common.make_pio_pin(pins.b1),
            common.make_pio_pin(pins.r2), common.make_pio_pin(pins.g2), common.make_pio_pin(pins.b2),
            common.make_pio_pin(pins.r3), common.make_pio_pin(pins.g3), common.make_pio_pin(pins.b3),
            common.make_pio_pin(pins.r4), common.make_pio_pin(pins.g4), common.make_pio_pin(pins.b4),
            common.make_pio_pin(pins.le), common.make_pio_pin(pins.row),
            common.make_pio_pin(pins.lck), common.make_pio_pin(pins.ldata), common.make_pio_pin(pins.dclk),
        ];
        #[cfg(icnd2069_hdl)]
        let mut owned_pins = [
            common.make_pio_pin(pins.r1), common.make_pio_pin(pins.g1), common.make_pio_pin(pins.b1),
            common.make_pio_pin(pins.r2), common.make_pio_pin(pins.g2), common.make_pio_pin(pins.b2),
            common.make_pio_pin(pins.r3), common.make_pio_pin(pins.g3), common.make_pio_pin(pins.b3),
            common.make_pio_pin(pins.sel1), common.make_pio_pin(pins.sel2), common.make_pio_pin(pins.sel3),
            common.make_pio_pin(pins.lat1), common.make_pio_pin(pins.lat2), common.make_pio_pin(pins.lat3),
            common.make_pio_pin(pins.row), common.make_pio_pin(pins.lck), common.make_pio_pin(pins.ldata),
            common.make_pio_pin(pins.dclk), common.make_pio_pin(pins.dclk2),
        ];
        let mut bk = bk_pin.map(|pin| common.make_pio_pin(pin));
        // make_pio_pin resets pad settings, so configure after registration
        // and before enabling output directions or starting the SM.
        for pin in owned_pins.iter_mut().chain(bk.as_mut()) {
            let selected = PIN_DRIVE_MASK & (1 << pin.pin()) != 0;
            pin.set_drive_strength(if selected { CFG_PIN_DRIVE_STRENGTH } else { Drive::_4mA });
            pin.set_slew_rate(if selected { CFG_PIN_SLEW_RATE } else { SlewRate::Slow });
        }
        let loaded = common.load_program(&crate::dclk_program::fixed_rate(
            protocol::PIO_CYCLES_PER_DCLK));
        let mut cfg = pio::Config::default();
        cfg.use_program(&loaded, &[&owned_pins[DCLK_INDEX]]);
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
        // Only board-connected pins are muxed. On HDL, SM1 overrides GP17
        // at each SM0 instruction boundary; direct uses GP17 as DIN.
        unsafe { cfg.set_pins(pin_cfg); }
        sm.set_config(&cfg);
        // Reuse the same program: empty FIFO PULL is nonblocking, and
        // OUT_COUNT=0 disables data writes. Only GP17 side-set is driven.
        #[cfg(icnd2069_hdl)]
        {
            let dclk2 = &owned_pins[SM0_PIN_COUNT];
            let mut clock_cfg = pio::Config::default();
            clock_cfg.use_program(&loaded, &[dclk2]);
            clock_cfg.clock_divider = cfg.clock_divider;
            clock_sm.set_config(&clock_cfg);
            clock_sm.set_pins(Level::Low, &[dclk2]);
            clock_sm.set_pin_dirs(pio::Direction::Out, &[dclk2]);
        }
        let all = owned_pins.each_ref();
        sm.set_pins(Level::Low, &all[..SM0_PIN_COUNT]);
        if let Some(bk) = &bk {
            sm.set_pins(Level::Low, &[bk]);
            sm.set_pin_dirs(pio::Direction::Out, &[bk]);
        }
        if !protocol::DISABLE_RT5990 && !protocol::DIN_ONE_SELECTS {
            sm.set_pins(Level::High, &[&owned_pins[DIN_INDEX]]);
        }
        sm.set_pin_dirs(pio::Direction::Out, &all[..SM0_PIN_COUNT]);
        Self {
            sm,
            #[cfg(icnd2069_hdl)]
            clock_sm,
            _dma: dma.into_ref(), _control_dma: control_dma.into_ref(), _common: common,
            divider: protocol::PIO_CLKDIV_INT << 8,
            running: false, program_origin: loaded.origin, _blank: blank,
            pins: owned_pins,
            bk,
            drive_strength: CFG_PIN_DRIVE_STRENGTH,
            slew_rate: CFG_PIN_SLEW_RATE,
        }
    }

    /// Blocks with unchanged rate and pads keep DCLK running.
    /// Rate or selected pad changes drain PARK and stop both SMs.
    pub fn send(&mut self, transfer: &Transfer<'_>, configuration: bool) {
        let words = transfer.first;
        #[cfg(feature = "dma-diagnostics")]
        let started = embassy_time::Instant::now();
        #[cfg(feature = "dma-diagnostics")]
        let mut next_report = 100;
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
        let slew_rate = if configuration { CFG_PIN_SLEW_RATE } else { PIN_SLEW_RATE };
        let pads_changed = PIN_DRIVE_MASK != 0
            && (self.drive_strength != drive_strength || self.slew_rate != slew_rate);
        let start = !self.running || self.divider != divider || pads_changed;
        if start {
            if self.running {
                while !self.sm.tx().empty() {
                    #[cfg(feature = "dma-diagnostics")]
                    {
                        let elapsed = started.elapsed().as_millis();
                        if elapsed >= next_report {
                            report_stall("drain_fifo", elapsed, words);
                            next_report = elapsed + 100;
                        }
                    }
                    core::hint::spin_loop();
                }
                // All queued trailing words are PARK. No active latch/ROW
                // can be cut short while stopping for a rate/drive change.
                embassy_rp::pac::PIO0.ctrl().modify(|w| {
                    w.set_sm_enable(w.sm_enable() & !SM_MASK);
                });
            }
            self.sm.restart();
            #[cfg(icnd2069_hdl)]
            self.clock_sm.restart();
            // JMP to the one-time SET X,0, with side-set LOW. Forced
            // instructions execute while disabled, parking DCLK before
            // changing pads/divider; startup reinitializes the idle word.
            unsafe { self.sm.exec_instr(self.program_origin as u16); }
            #[cfg(icnd2069_hdl)]
            unsafe { self.clock_sm.exec_instr(self.program_origin as u16); }
            if pads_changed {
                for pin in self.pins.iter_mut().chain(self.bk.as_mut())
                    .filter(|pin| PIN_DRIVE_MASK & (1 << pin.pin()) != 0) {
                    pin.set_drive_strength(if configuration {
                        CFG_PIN_DRIVE_STRENGTH
                    } else {
                        PIN_DRIVE_STRENGTH
                    });
                    pin.set_slew_rate(if configuration { CFG_PIN_SLEW_RATE } else { PIN_SLEW_RATE });
                }
                if let Some(blank) = self._blank.as_mut().filter(|_| PIN_DRIVE_MASK & protocol::BK != 0) {
                    blank.set_drive_strength(if configuration {
                        CFG_PIN_DRIVE_STRENGTH
                    } else {
                        PIN_DRIVE_STRENGTH
                    });
                    blank.set_slew_rate(if configuration { CFG_PIN_SLEW_RATE } else { PIN_SLEW_RATE });
                }
                self.drive_strength = drive_strength;
                self.slew_rate = slew_rate;
            }
            self.sm.set_clock_divider(U24F8::from_bits(divider));
            #[cfg(icnd2069_hdl)]
            self.clock_sm.set_clock_divider(U24F8::from_bits(divider));
            self.divider = divider;
        }
        // DMA1 loads the next descriptor when DMA0 finishes. The complete
        // descriptor list and all waveform words stay valid without CPU work.
        let dma = embassy_rp::pac::DMA;
        assert!(!dma.ch(0).ctrl_trig().read().busy() && !dma.ch(1).ctrl_trig().read().busy());
        dma.inte(0).modify(|bits| *bits &= !3);
        dma.intr(0).write_value(3);
        let descriptors = &transfer.blocks[..transfer.count];
        start_chain(descriptors);
        if start {
            while embassy_rp::pac::PIO0.flevel().read().0 & 0xf < words.len().min(8) as u32 {
                #[cfg(feature = "dma-diagnostics")]
                {
                    let elapsed = started.elapsed().as_millis();
                    if elapsed >= next_report {
                        report_stall("prefill_fifo", elapsed, words);
                        next_report = elapsed + 100;
                    }
                }
                core::hint::spin_loop();
            }
            // Reset both divider phases and enable both SMs in one write.
            embassy_rp::pac::PIO0.ctrl().modify(|w| {
                w.set_clkdiv_restart(SM_MASK);
                w.set_sm_enable(w.sm_enable() | SM_MASK);
            });
            self.running = true;
        }
        let end = descriptors.as_ptr_range().end as u32;
        while dma.ch(1).read_addr().read() != end
            || dma.ch(1).ctrl_trig().read().busy() || dma.ch(0).ctrl_trig().read().busy()
        {
            assert!(!dma.ch(0).ctrl_trig().read().ahb_error()
                && !dma.ch(1).ctrl_trig().read().ahb_error(), "chained DMA bus error");
            core::hint::spin_loop();
        }
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        dma.intr(0).write_value(3);
    }
}

fn start_chain(descriptors: &[[u32; 2]]) {
    use embassy_rp::pac::{dma::{regs::CtrlTrig, vals::{DataSize, TreqSel}}, DMA, PIO0};
    let data = DMA.ch(0);
    let control = DMA.ch(1);
    data.write_addr().write_value(PIO0.txf(0).as_ptr() as u32);
    let mut cfg = CtrlTrig::default();
    cfg.set_en(true);
    cfg.set_data_size(DataSize::SIZE_WORD);
    cfg.set_incr_read(true);
    cfg.set_treq_sel(TreqSel::PIO0_TX0);
    cfg.set_chain_to(1);
    cfg.set_irq_quiet(true);
    data.al1_ctrl().write_value(cfg.0);
    control.read_addr().write_value(descriptors.as_ptr() as u32);
    control.write_addr().write_value(data.al3_trans_count().as_ptr() as u32);
    control.trans_count().write(|w| { w.set_mode(0.into()); w.set_count(2); });
    let mut cfg = CtrlTrig::default();
    cfg.set_en(true);
    cfg.set_data_size(DataSize::SIZE_WORD);
    cfg.set_incr_read(true);
    cfg.set_incr_write(true);
    cfg.set_ring_sel(true);
    cfg.set_ring_size(3);
    cfg.set_treq_sel(TreqSel::PERMANENT);
    cfg.set_chain_to(1);
    cfg.set_irq_quiet(true);
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    control.ctrl_trig().write_value(cfg);
}
