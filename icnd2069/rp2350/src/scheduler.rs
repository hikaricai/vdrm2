//! Frame counter shared by the main loop and host configuration checks.
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
