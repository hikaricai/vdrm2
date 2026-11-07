use embassy_rp::{
    bind_interrupts,
    gpio::Level,
    peripherals::{
        DMA_CH0, DMA_CH1, DMA_CH2, DMA_CH3, DMA_CH4, DMA_CH5,
        PIN_0, PIN_1, PIN_2, PIN_3, PIN_4, PIN_5, PIN_6, PIN_7, PIN_8,
        PIN_9, PIN_10, PIN_11, PIN_12, PIN_13, PIN_14, PIN_15, PIN_16, PIN_17, PIO0,
    },
    pio::{self, program::pio_asm, Common, Pio, ShiftConfig, StateMachine},
    Peripheral, PeripheralRef,
};
use fixed::types::U24F8;

use crate::protocol::DIN_ONE_SELECTS;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
    DMA_IRQ_1 => crate::scheduler::Handler;
});

pub struct Pins {
    pub r1: PIN_0,
    pub g1: PIN_1,
    pub b1: PIN_2,
    pub r2: PIN_3,
    pub g2: PIN_4,
    pub b2: PIN_5,
    pub r3: PIN_6,
    pub g3: PIN_7,
    pub b3: PIN_8,
    pub r4: PIN_9,
    pub g4: PIN_10,
    pub b4: PIN_11,
    // ICND2069
    pub dclk: PIN_12,
    pub le: PIN_13,
    pub row: PIN_14,
    // RT5990
    pub lck: PIN_15,
    pub bk: PIN_16,
    pub din: PIN_17,
}

/// SM0 drives RGB1..4 and LE/LCK/BK/DIN, with DCLK on side-set.
/// SM1 advances ROW using SM0's per-clock IRQ.
pub struct Driver {
    sm: StateMachine<'static, PIO0, 0>,
    row_sm: StateMachine<'static, PIO0, 1>,
    dma: PeripheralRef<'static, DMA_CH0>,
    row_dma: PeripheralRef<'static, DMA_CH1>,
    started: bool,
    _common: Common<'static, PIO0>,
}

impl Driver {
    pub fn start_scheduled(
        &mut self,
        heads: &'static [u32],
        head_words: usize,
        tail: &'static [u32],
        row_runs: &'static [u32],
        descriptors: &'static mut [u32; 24],
        row_descriptors: &'static mut [u32; 16],
        first_runs: &'static mut [u32; 2],
        restart: &'static mut [u32; 2],
        _control0: DMA_CH2,
        _control1: DMA_CH3,
        _rewind0: DMA_CH4,
        _rewind1: DMA_CH5,
    ) {
        assert!(!self.started);
        // These six channels are exclusively owned. The scheduler owns all
        // mutable descriptors after this call; waveform samples stay immutable.
        unsafe {
            crate::scheduler::start(heads, head_words, tail, row_runs,
                descriptors, row_descriptors, first_runs, restart);
        }
        // Clear setup history before streaming; later TXSTALL means DCLK
        // was stretched while SM0 waited for data. Read/clear once per poll.
        let _ = self.sm.tx().stalled();
        self._common.apply_sm_batch(|b| {
            b.restart(&mut self.sm);
            b.restart(&mut self.row_sm);
            b.set_enable(&mut self.row_sm, true);
            b.set_enable(&mut self.sm, true);
        });
        self.started = true;
    }

    pub fn take_dclk_stall(&mut self) -> bool {
        self.sm.tx().stalled()
    }

    /// Alternate a gray-specific input head and the shared scan tail. DMA2
    /// writes four AL3 registers on DMA0 per segment. The final tail chains to
    /// DMA4, which rewinds the descriptor list. DMA1/3 repeat one ROW table.
    pub fn start_segments(
        &mut self,
        heads: &'static [u32],
        head_words: usize,
        tail: &'static [u32],
        row_runs: &'static [u32],
        descriptors: &'static mut [u32],
        restart_addresses: &'static mut [u32; 2],
        _control0: DMA_CH2,
        _control1: DMA_CH3,
        _rewind: DMA_CH4,
    ) {
        use embassy_rp::pac::{self, dma::vals::{DataSize, TreqSel}};
        assert!(!self.started && head_words != 0 && !tail.is_empty() && !row_runs.is_empty());
        assert_eq!(heads.len() % head_words, 0);
        let frames = heads.len() / head_words;
        assert!(frames > 0);
        assert_eq!(descriptors.len(), frames * 2 * 4);
        let mut stream_ctrl = pac::dma::regs::CtrlTrig::default();
        stream_ctrl.set_data_size(DataSize::SIZE_WORD);
        stream_ctrl.set_treq_sel(TreqSel::from(0));
        stream_ctrl.set_incr_read(true);
        stream_ctrl.set_irq_quiet(true);
        stream_ctrl.set_en(true);
        for i in 0..frames * 2 {
            let is_head = i % 2 == 0;
            let (data, count) = if is_head {
                (heads[i / 2 * head_words..].as_ptr(), head_words)
            } else {
                (tail.as_ptr(), tail.len())
            };
            stream_ctrl.set_chain_to(if i == frames * 2 - 1 { 4 } else { 2 });
            // AL3: CTRL, WRITE_ADDR, TRANS_COUNT, READ_ADDR_TRIG. The final
            // write triggers DMA0 only after all segment registers are valid.
            descriptors[i * 4..i * 4 + 4].copy_from_slice(&[
                stream_ctrl.0, pac::PIO0.txf(0).as_ptr() as u32,
                count as u32, data as u32,
            ]);
        }
        restart_addresses[0] = row_runs.as_ptr() as u32;
        restart_addresses[1] = descriptors.as_ptr() as u32;
        pac::DMA.inte(0).modify(|v| *v &= !0x1f);
        // Channels and buffers are exclusively owned; no HAL Transfer future
        // accesses these channels. All DMA sources have static lifetime and
        // become immutable before the stream starts.
        for (reset_id, target_id, address) in [
            (3, 1, &restart_addresses[0]),
            (4, 2, &restart_addresses[1]),
        ] {
            let reset = pac::DMA.ch(reset_id);
            reset.read_addr().write_value(address as *const u32 as u32);
            reset.write_addr().write_value(pac::DMA.ch(target_id).al3_read_addr_trig().as_ptr() as u32);
            reset.trans_count().write(|v| { v.set_mode(0.into()); v.set_count(1); });
            let mut ctrl = pac::dma::regs::CtrlTrig::default();
            ctrl.set_data_size(DataSize::SIZE_WORD);
            ctrl.set_treq_sel(TreqSel::PERMANENT);
            ctrl.set_chain_to(reset_id as u8);
            ctrl.set_irq_quiet(true);
            ctrl.set_en(true);
            reset.al1_ctrl().write_value(ctrl.0);
        }
        let row = pac::DMA.ch(1);
        row.read_addr().write_value(row_runs.as_ptr() as u32);
        row.write_addr().write_value(pac::PIO0.txf(1).as_ptr() as u32);
        row.trans_count().write(|v| { v.set_mode(0.into()); v.set_count(row_runs.len() as u32); });
        let control = pac::DMA.ch(2);
        control.read_addr().write_value(descriptors.as_ptr() as u32);
        // 16-byte write ring is aligned on DMA0's AL3_CTRL (offset 0x30).
        control.write_addr().write_value(pac::DMA.ch(0).al3_ctrl().as_ptr() as u32);
        control.trans_count().write(|v| { v.set_mode(0.into()); v.set_count(4); });
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        row.ctrl_trig().write(|v| {
            v.set_data_size(DataSize::SIZE_WORD);
            v.set_treq_sel(TreqSel::from(1));
            v.set_incr_read(true);
            v.set_chain_to(3);
            v.set_irq_quiet(true);
            v.set_en(true);
        });
        control.ctrl_trig().write(|v| {
            v.set_data_size(DataSize::SIZE_WORD);
            v.set_treq_sel(TreqSel::PERMANENT);
            v.set_incr_read(true);
            v.set_incr_write(true);
            v.set_ring_sel(true);
            v.set_ring_size(4);
            v.set_chain_to(2);
            v.set_irq_quiet(true);
            v.set_en(true);
        });
        self._common.apply_sm_batch(|b| {
            b.restart(&mut self.sm);
            b.restart(&mut self.row_sm);
            b.set_enable(&mut self.row_sm, true);
            b.set_enable(&mut self.sm, true);
        });
        self.started = true;
    }

    /// DMA0/1 stream immutable cycles. DMA2/3 reset their read pointers via
    /// AL3_READ_ADDR_TRIG, which reloads the original transfer count. No CPU
    /// wakeup is needed at either a frame boundary or a cycle boundary.
    pub fn start_loop(
        &mut self,
        words: &'static [u32],
        row_runs: &'static [u32],
        restart_addresses: &'static mut [u32; 2],
        _control0: DMA_CH2,
        _control1: DMA_CH3,
    ) {
        use embassy_rp::pac::{self, dma::vals::{DataSize, TreqSel}};
        assert!(!self.started && !words.is_empty() && !row_runs.is_empty());
        restart_addresses[0] = words.as_ptr() as u32;
        restart_addresses[1] = row_runs.as_ptr() as u32;
        // These four DMA channels are owned by this driver/call. Buffers have
        // static lifetime and are immutable after arming. Disable HAL IRQs for
        // the hardware loop, which has no Transfer future or channel waker.
        pac::DMA.inte(0).modify(|v| *v &= !0x0f);
        for (i, data, count) in [(0, words.as_ptr(), words.len()),
                                 (1, row_runs.as_ptr(), row_runs.len())] {
            let stream = pac::DMA.ch(i);
            let reset = pac::DMA.ch(i + 2);
            reset.read_addr().write_value((&restart_addresses[i]) as *const u32 as u32);
            reset.write_addr().write_value(stream.al3_read_addr_trig().as_ptr() as u32);
            reset.trans_count().write(|v| { v.set_mode(0.into()); v.set_count(1); });
            let mut ctrl = pac::dma::regs::CtrlTrig::default();
            ctrl.set_data_size(DataSize::SIZE_WORD);
            ctrl.set_treq_sel(TreqSel::PERMANENT);
            ctrl.set_chain_to((i + 2) as u8);
            ctrl.set_irq_quiet(true);
            ctrl.set_en(true);
            // Non-triggering alias: the stream completion starts this channel.
            reset.al1_ctrl().write_value(ctrl.0);
            stream.read_addr().write_value(data as u32);
            stream.write_addr().write_value(pac::PIO0.txf(i).as_ptr() as u32);
            stream.trans_count().write(|v| { v.set_mode(0.into()); v.set_count(count as u32); });
            core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
            stream.ctrl_trig().write(|v| {
                v.set_data_size(DataSize::SIZE_WORD);
                v.set_treq_sel(TreqSel::from(i as u8));
                v.set_incr_read(true);
                v.set_chain_to((i + 2) as u8);
                v.set_irq_quiet(true);
                v.set_en(true);
            });
        }
        self._common.apply_sm_batch(|b| {
            b.restart(&mut self.sm);
            b.restart(&mut self.row_sm);
            b.set_enable(&mut self.row_sm, true);
            b.set_enable(&mut self.sm, true);
        });
        self.started = true;
    }

    pub fn new(pio: PIO0, dma: DMA_CH0, row_dma: DMA_CH1, pins: Pins) -> Self {
        let Pio {
            mut common,
            sm0: mut sm,
            sm1: mut row_sm,
            ..
        } = Pio::new(pio, Irqs);
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
        let dclk = common.make_pio_pin(pins.dclk);
        let lck = common.make_pio_pin(pins.lck);
        let bk = common.make_pio_pin(pins.bk);
        let din = common.make_pio_pin(pins.din);
        let row = common.make_pio_pin(pins.row);

        // >=30 MHz: four cycles, high 2 / low 2. Pull during the high phase
        // removes a separate NOP. Lower presets retain the six-cycle program.
        // IRQ4 synchronizes SM1; no captured delay replay.
        let program = if crate::protocol::PIO_CYCLES_PER_DCLK == 4 {
            pio_asm!(
                ".side_set 1",
                ".wrap_target",
                "pull ifempty block side 1 [1]",
                "irq 4 side 0",
                "out pins, 32 side 0",
                ".wrap",
            ).program
        } else {
            pio_asm!(
                ".side_set 1",
                ".wrap_target",
                "nop side 1 [2]",
                "pull ifempty block side 0",
                "irq 4 side 0",
                "out pins, 32 side 0",
                ".wrap",
            ).program
        };
        let loaded = common.load_program(&program);
        let mut cfg = pio::Config::default();
        cfg.use_program(&loaded, &[&dclk]);
        // Convert the ratio, not 150 MHz itself (which exceeds U24F8's range).
        // One 32-bit word per DCLK covers GP0..17; only 18 bits drive pins.
        // Side-set owns DCLK (GP12); sticky SM1 output owns ROW (GP14).
        // The low 8 fractional bits are always zero: never approximate a
        // requested DCLK with a fractional divider (which adds long cycles).
        assert_eq!(embassy_rp::clocks::clk_sys_freq(), crate::protocol::SYS_HZ);
        cfg.clock_divider = U24F8::from_bits(crate::protocol::PIO_CLKDIV_INT << 8);
        cfg.fifo_join = pio::FifoJoin::TxOnly;
        cfg.shift_out = ShiftConfig {
            auto_fill: false,
            threshold: 32,
            direction: pio::ShiftDirection::Right,
        };
        cfg.set_out_pins(&[
            &r1, &g1, &b1, &r2, &g2, &b2, &r3, &g3, &b3, &r4, &g4, &b4,
            &dclk, &le, &row, &lck, &bk, &din,
        ]);
        sm.set_config(&cfg);
        let all = [
            &r1, &g1, &b1, &r2, &g2, &b2, &r3, &g3, &b3, &r4, &g4, &b4,
            &le, &lck, &bk, &din, &dclk,
        ];
        sm.set_pins(Level::Low, &all);
        // BK starts inactive; waveform rows generate the complete 8-DCLK pulse.
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
        // GP14 lies inside SM0's contiguous OUT range. SM1 has higher pin
        // priority; continuously reassert its last ROW value so SM0 cannot
        // clear it between the sparse ROW transitions.
        row_cfg.out_sticky = true;
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
