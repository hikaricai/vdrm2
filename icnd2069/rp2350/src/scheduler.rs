//! Two pre-armed DMA banks keep RAM independent of the configuration interval.
//! The tail IRQ prepares the completed bank for reuse one frame later.
#[cfg(target_os = "none")]
use crate::protocol;

pub struct Cursor {
    phase: u32,
    gray: usize,
    interval: u32,
    grays: usize,
}

impl Cursor {
    pub fn new(interval: u32, grays: usize) -> Self {
        assert!(interval > 0 && grays > 0);
        Self { phase: 0, gray: 0, interval, grays }
    }

    pub fn next(&mut self) -> (bool, usize) {
        let result = (self.phase == 0, self.gray);
        self.phase = if self.phase == self.interval - 1 { 0 } else { self.phase + 1 };
        self.gray = if self.gray == self.grays - 1 { 0 } else { self.gray + 1 };
        result
    }
}

#[cfg(target_os = "none")]
mod hardware {
    use super::*;
    use embassy_rp::{
        interrupt::{self, InterruptExt},
        pac::{self, dma::vals::{DataSize, TreqSel}},
    };
    use core::ptr::{write_volatile, addr_of_mut};
    use core::sync::atomic::{compiler_fence, Ordering};

    struct State {
        cursor: Cursor,
        bank: usize,
        heads: *const u32,
        head_words: usize,
        descriptors: *mut u32,
        first_runs: *mut u32,
        full_first_run: u32,
    }
    // Initialized before IRQ enable, then accessed only by DMA_IRQ_1.
    static mut STATE: Option<State> = None;

    impl State {
        unsafe fn prepare(&mut self, bank: usize) {
            let (full, gray) = self.cursor.next();
            let offset = if full {
                protocol::COMMON_PREFIX_WORDS
            } else {
                protocol::CONFIG_END_WORDS
            };
            let middle = self.descriptors.add(bank * 12 + 4);
            write_volatile(middle.add(2), (self.head_words - offset) as u32);
            write_volatile(middle.add(3), self.heads.add(gray * self.head_words + offset) as u32);
            write_volatile(self.first_runs.add(bank), self.full_first_run
                - if full { 0 } else { (protocol::CONFIG_SKIP_DCLKS as u32) << 1 });
            compiler_fence(Ordering::SeqCst);
            cortex_m::asm::dmb();
        }
    }

    pub struct Handler;
    impl interrupt::typelevel::Handler<interrupt::typelevel::DMA_IRQ_1> for Handler {
        unsafe fn on_interrupt() {
            if pac::DMA.ints(1).read() & 1 == 0 {
                return;
            }
            pac::DMA.ints(1).write_value(1);
            let state = (&mut *addr_of_mut!(STATE)).as_mut().unwrap();
            // DMA0 has already chained to the other bank. Its complete data
            // head + 16-row tail gives us at least one frame to update this
            // bank. ROW DMA can prefetch into the other bank, but its 33+
            // run words prevent prefetch reaching this bank yet.
            state.prepare(state.bank);
            state.bank ^= 1;
        }
    }

    pub unsafe fn start(
        heads: &'static [u32],
        head_words: usize,
        tail: &'static [u32],
        row_runs: &'static [u32],
        descriptors: &'static mut [u32; 24],
        row_descriptors: &'static mut [u32; 16],
        first_runs: &'static mut [u32; 2],
        restart: &'static mut [u32; 2],
    ) {
        assert!(heads.len() % head_words == 0);
        assert!(head_words > protocol::CONFIG_END_WORDS);
        assert!(row_runs.len() == protocol::ROW_RUN_COUNT && !tail.is_empty());
        pac::DMA.inte(0).modify(|v| *v &= !0x3f);
        pac::DMA.inte(1).modify(|v| *v &= !0x3f);
        pac::DMA.ints(1).write_value(0x3f);
        for bank in 0..2 {
            for segment in 0..3 {
                let mut ctrl = pac::dma::regs::CtrlTrig::default();
                ctrl.set_data_size(DataSize::SIZE_WORD);
                ctrl.set_treq_sel(TreqSel::from(0));
                ctrl.set_incr_read(true);
                ctrl.set_irq_quiet(segment != 2);
                ctrl.set_en(true);
                ctrl.set_chain_to(if bank == 1 && segment == 2 { 4 } else { 2 });
                let (data, count) = if segment == 0 {
                    (heads.as_ptr(), protocol::COMMON_PREFIX_WORDS)
                } else if segment == 1 {
                    (heads.as_ptr(), head_words)
                } else {
                    (tail.as_ptr(), tail.len())
                };
                let i = bank * 12 + segment * 4;
                descriptors[i..i + 4].copy_from_slice(&[
                    ctrl.0, pac::PIO0.txf(0).as_ptr() as u32,
                    count as u32, data as u32,
                ]);
            }
            for segment in 0..2 {
                let mut ctrl = pac::dma::regs::CtrlTrig::default();
                ctrl.set_data_size(DataSize::SIZE_WORD);
                ctrl.set_treq_sel(TreqSel::from(1));
                ctrl.set_incr_read(true);
                ctrl.set_irq_quiet(true);
                ctrl.set_en(true);
                ctrl.set_chain_to(if bank == 1 && segment == 1 { 5 } else { 3 });
                let (data, count) = if segment == 0 {
                    (first_runs.as_ptr().add(bank), 1)
                } else {
                    (row_runs.as_ptr().add(1), row_runs.len() - 1)
                };
                let i = bank * 8 + segment * 4;
                row_descriptors[i..i + 4].copy_from_slice(&[
                    ctrl.0, pac::PIO0.txf(1).as_ptr() as u32,
                    count as u32, data as u32,
                ]);
            }
        }
        let mut state = State {
            cursor: Cursor::new(protocol::CONFIG_INTERVAL, heads.len() / head_words),
            bank: 0, heads: heads.as_ptr(), head_words,
            descriptors: descriptors.as_mut_ptr(),
            first_runs: first_runs.as_mut_ptr(), full_first_run: row_runs[0],
        };
        state.prepare(0);
        state.prepare(1);
        *addr_of_mut!(STATE) = Some(state);
        restart[0] = descriptors.as_ptr() as u32;
        restart[1] = row_descriptors.as_ptr() as u32;
        for (reset_id, target_id, address) in [(4, 2, &restart[0]), (5, 3, &restart[1])] {
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
        // Each control channel copies four words through the AL3 write ring.
        for (id, target, source) in [
            (2, 0, descriptors.as_ptr()), (3, 1, row_descriptors.as_ptr())
        ] {
            let control = pac::DMA.ch(id);
            control.read_addr().write_value(source as u32);
            control.write_addr().write_value(pac::DMA.ch(target).al3_ctrl().as_ptr() as u32);
            control.trans_count().write(|v| { v.set_mode(0.into()); v.set_count(4); });
        }
        compiler_fence(Ordering::SeqCst);
        cortex_m::asm::dmb();
        interrupt::DMA_IRQ_1.set_priority(interrupt::Priority::P1);
        interrupt::DMA_IRQ_1.unpend();
        interrupt::DMA_IRQ_1.enable();
        pac::DMA.inte(1).modify(|v| *v |= 1);
        for id in [3, 2] {
            pac::DMA.ch(id).ctrl_trig().write(|v| {
                v.set_data_size(DataSize::SIZE_WORD);
                v.set_treq_sel(TreqSel::PERMANENT);
                v.set_incr_read(true);
                v.set_incr_write(true);
                v.set_ring_sel(true);
                v.set_ring_size(4);
                v.set_chain_to(id as u8);
                v.set_irq_quiet(true);
                v.set_en(true);
            });
        }
    }
}

#[cfg(target_os = "none")]
pub use hardware::{start, Handler};
