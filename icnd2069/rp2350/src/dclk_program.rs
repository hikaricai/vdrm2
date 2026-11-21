//! Fixed waveform; changing the SM clock divider changes DCLK frequency.
#[cfg(target_os = "none")]
use embassy_rp::pio::program as pio;
use pio::{Assembler, OutDestination, Program, SetDestination, SideSet};

pub fn fixed_rate(cycles: u32) -> Program<32> {
    assert!(cycles == 4 || cycles == 6);
    let half = cycles / 2;
    let mut asm = Assembler::<32>::new_with_side_set(SideSet::new(false, 1, false));
    // PULL NOBLOCK copies X when FIFO is empty. Initialize X to PARK=0
    // once, outside the wrap, so idle clocks cannot repeat a latch/ROW word.
    asm.set_with_side_set(SetDestination::X, 0, 0);
    let mut target = asm.label();
    let mut source = asm.label();
    asm.bind(&mut target);
    // Prepare RGB/SEL/LAT during HIGH. CPLD publishes RGB at the falling
    // edge; the next rising edge clocks that same word into ICND2069.
    asm.pull_with_delay_and_side_set(false, false, (half - 2) as u8, 1);
    asm.out_with_side_set(OutDestination::PINS, crate::protocol::PIO_OUT_COUNT, 1);
    asm.nop_with_delay_and_side_set((half - 1) as u8, 0);
    asm.bind(&mut source);
    asm.assemble_with_wrap(source, target)
}
