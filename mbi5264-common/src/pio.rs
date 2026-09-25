use crate::{RGBH, SERIAL_CHIPS};

/// Maximum number of 32-bit words accepted by the PIO DMA for one angle frame.
pub const MAX_FRAME_WORDS: usize = 8192;
pub const LE_HIGH: u16 = 1 << 12;

const LAST_CHIP_IDX: u32 = SERIAL_CHIPS - 1;
const SEL_PAIR: u32 = 0x0008_0008;
const SEL_LAT_PAIR: u32 = 0x4000_4000;
const SEL_DATA_MASK_PAIR: u32 = 0x0888_0888;

struct RGBMeta {
    rgbh: RGBH,
    h_div: u8,
    h_mod: u8,
    region: u16,
}

impl RGBMeta {
    #[inline(always)]
    fn new(rgbh: RGBH, region: u16) -> Self {
        let h = rgbh.h();
        Self {
            rgbh,
            h_div: ((h >> 4) & 0x0f) % SERIAL_CHIPS as u8,
            h_mod: h & 0x0f,
            region,
        }
    }
}

#[inline(always)]
fn bubble_rgbh(slice: &mut [RGBMeta]) {
    let len = slice.len();
    for i in 0..len {
        for j in 0..len - 1 - i {
            let (left, right) = (&slice[j], &slice[j + 1]);
            if (left.h_mod, left.h_div) > (right.h_mod, right.h_div) {
                slice.swap(j, j + 1);
            }
        }
    }
}

const fn make_nibble_planes() -> [[u32; 2]; 16] {
    let mut planes = [[0; 2]; 16];
    let mut nibble = 0;
    while nibble < planes.len() {
        planes[nibble][0] = ((nibble as u32 >> 3) & 1) | (((nibble as u32 >> 2) & 1) << 16);
        planes[nibble][1] = ((nibble as u32 >> 1) & 1) | ((nibble as u32 & 1) << 16);
        nibble += 1;
    }
    planes
}

static NIBBLE_PLANES: [[u32; 2]; 16] = make_nibble_planes();

#[inline(always)]
fn pixel_planes(rgbh: RGBH) -> [u32; 4] {
    let [r, g, b] = rgbh.rgb();
    let r_hi = NIBBLE_PLANES[(r >> 4) as usize];
    let g_hi = NIBBLE_PLANES[(g >> 4) as usize];
    let b_hi = NIBBLE_PLANES[(b >> 4) as usize];
    let r_lo = NIBBLE_PLANES[(r & 0x0f) as usize];
    let g_lo = NIBBLE_PLANES[(g & 0x0f) as usize];
    let b_lo = NIBBLE_PLANES[(b & 0x0f) as usize];
    let h_idx = rgbh.h_idx() as u32;

    [
        r_hi[0] | (g_hi[0] << 1) | (b_hi[0] << 2),
        r_hi[1] | (g_hi[1] << 1) | (b_hi[1] << 2) | (((h_idx >> 2) & 1) * SEL_PAIR),
        r_lo[0] | (g_lo[0] << 1) | (b_lo[0] << 2) | (((h_idx >> 1) & 1) * SEL_PAIR),
        r_lo[1] | (g_lo[1] << 1) | (b_lo[1] << 2) | ((h_idx & 1) * SEL_PAIR),
    ]
}

struct PixelSlot {
    buf: [u32; 4],
    h_div: u8,
    h_mod: u8,
    last_chip_idx: u32,
}

impl PixelSlot {
    #[inline(always)]
    fn new(rgbh_meta: &RGBMeta, last_chip_idx: u32, last_buf: Option<&[u32; 4]>) -> Self {
        let mut buf = [0u32; 4];
        if let Some(last_buf) = last_buf {
            for (value, last) in buf.iter_mut().zip(last_buf) {
                *value = *last & SEL_DATA_MASK_PAIR;
            }
        }

        let &RGBMeta {
            rgbh,
            region,
            h_div,
            h_mod,
        } = rgbh_meta;
        let planes = pixel_planes(rgbh);
        let shift = 4 * region;
        for (value, plane) in buf.iter_mut().zip(planes) {
            *value |= (plane << shift) | SEL_LAT_PAIR;
        }
        Self {
            buf,
            h_div,
            h_mod,
            last_chip_idx,
        }
    }

    #[inline(always)]
    fn update(&mut self, rgbh_meta: &RGBMeta) {
        let planes = pixel_planes(rgbh_meta.rgbh);
        let shift = 4 * rgbh_meta.region;
        for (value, plane) in self.buf.iter_mut().zip(planes) {
            *value |= plane << shift;
        }
    }

    #[inline(always)]
    fn clear_sel_lat(&mut self) {
        for value in &mut self.buf {
            *value &= !SEL_LAT_PAIR;
        }
    }
}

#[repr(C)]
struct ColorTransfer {
    empty_loops: u32,
    data_loops: u32,
    buf: [u32; 4],
}

#[repr(C)]
struct ColorTransferTail {
    empty_loops: u32,
    data_loops: u32,
    buf: [u16; 2],
}

struct ColorParser {
    loops: u32,
    buf: *mut u16,
    buf_origin: *mut u16,
    buf_end: *mut u16,
    last_empties: u32,
    new_line: bool,
}

impl ColorParser {
    #[inline(always)]
    fn new(buf: &mut [u32]) -> Self {
        let buf_origin = buf.as_mut_ptr().cast::<u16>();
        Self {
            loops: 0,
            buf: unsafe { buf_origin.add(2) },
            buf_origin,
            buf_end: unsafe { buf_origin.add(buf.len() * 2) },
            last_empties: 0,
            new_line: false,
        }
    }

    #[inline(always)]
    fn encode(&mut self) -> usize {
        assert!(self.loops > 0);
        unsafe {
            self.buf_origin
                .cast::<u32>()
                .write_unaligned(self.loops - 1);
            self.buf.offset_from(self.buf_origin) as usize / 2
        }
    }

    #[inline(always)]
    fn reduce_empty_loops(last_empties: u32, required_empty_loops: u32) -> u32 {
        required_empty_loops.saturating_sub(last_empties + 3)
    }

    #[inline(always)]
    unsafe fn reserve<T>(&mut self) -> *mut T {
        let ptr = self.buf.cast::<T>();
        let words = core::mem::size_of::<T>() / core::mem::size_of::<u16>();
        let next = unsafe { self.buf.add(words) };
        assert!(next <= self.buf_end);
        self.buf = next;
        ptr
    }

    #[inline(always)]
    fn push_tail(&mut self, empty_loops: u32, data_loops: u32, buf: [u16; 2]) {
        unsafe {
            self.reserve::<ColorTransferTail>()
                .write(ColorTransferTail {
                    empty_loops,
                    data_loops,
                    buf,
                });
        }
        self.loops += 1;
    }

    #[inline(always)]
    fn push_color(&mut self, empty_loops: u32, data_loops: u32, buf: [u32; 4]) {
        unsafe {
            self.reserve::<ColorTransfer>().write(ColorTransfer {
                empty_loops,
                data_loops,
                buf,
            });
        }
        self.loops += 1;
    }

    #[inline(always)]
    fn push_words(&mut self, words: [u32; 4]) {
        unsafe {
            self.reserve::<[u32; 4]>().write(words);
        }
    }

    #[inline(always)]
    fn add_empty_les(&mut self, empty_size: u32) {
        const EMPTY_LEN_U32_CYCLES: u32 = 3;
        if empty_size == 0 {
            return;
        }
        let empty_loops = 16 * SERIAL_CHIPS;
        self.push_tail(
            Self::reduce_empty_loops(self.last_empties, empty_loops),
            0,
            [0, LE_HIGH],
        );
        for _ in 1..empty_size {
            self.push_tail(EMPTY_LEN_U32_CYCLES - 3, 0, [0, LE_HIGH]);
        }
        self.last_empties = empty_loops;
    }

    #[inline(always)]
    fn add_color(&mut self, buf: &[u32; 4], chip_index: u32, last_chip_idx: u32) {
        let le = chip_index == LAST_CHIP_IDX;
        let chip_inc_index = chip_index - last_chip_idx;
        let empty_loops = chip_inc_index * 16 + 8 * !self.new_line as u32;
        self.new_line = false;
        self.push_color(
            Self::reduce_empty_loops(self.last_empties, empty_loops),
            if le { 14 } else { 6 },
            *buf,
        );
        if le {
            self.push_words([0, 0, 0, (LE_HIGH as u32) << 16]);
        }
        self.last_empties = 0;
    }

    #[inline(always)]
    fn add_empty_le(&mut self, chip_inc_index: u32) {
        let empty_loops = chip_inc_index * 16 + 8;
        self.push_tail(empty_loops - 5, 0, [0, LE_HIGH]);
        self.last_empties = empty_loops;
    }

    #[inline(always)]
    fn add_color_end(&mut self, buf: &[u32; 4], chip_index: u32, last_chip_idx: u32) {
        let le = chip_index == LAST_CHIP_IDX;
        self.add_color(buf, chip_index, last_chip_idx);
        if !le {
            self.add_empty_le(LAST_CHIP_IDX - chip_index);
        }
    }

    #[inline(always)]
    fn add_sync(&mut self, empty_loops: u32) {
        self.push_tail(empty_loops, 0, [LE_HIGH, LE_HIGH]);
    }

    #[inline(always)]
    fn add_empty(&mut self, empty_loops: u32) {
        if empty_loops > 0 {
            self.push_tail(empty_loops + 5, 0, [0, 0]);
        }
    }
}

/// Encodes one RGBH column directly into the word stream consumed by the PIO DMA.
pub fn encode_frame(rgbh_column: &[RGBH; crate::IMG_HEIGHT], output: &mut [u32]) -> usize {
    let region0 = &rgbh_column[0..64];
    let region1 = &rgbh_column[64..128];
    let region2 = &rgbh_column[128..];
    let mut parser = ColorParser::new(output);
    let mut last_h_mod = 15;

    parser.add_empty(10);
    for line in 0..64 {
        let mut pixels = [
            RGBMeta::new(region0[line], 0),
            RGBMeta::new(region1[line], 1),
            RGBMeta::new(region2[line], 2),
        ];
        bubble_rgbh(&mut pixels);
        let mut pixel_iter = pixels.iter();
        let first_pixel = pixel_iter.next().unwrap();
        let mut last_slot = PixelSlot::new(first_pixel, 0, None);

        let empty = if last_slot.h_mod > last_h_mod {
            15 + last_slot.h_mod - last_h_mod
        } else {
            15 - (last_h_mod - last_slot.h_mod)
        };
        last_h_mod = last_slot.h_mod;
        parser.add_empty_les(empty as u32);
        parser.new_line = true;

        for rgbh_meta in pixel_iter {
            if rgbh_meta.h_mod == last_slot.h_mod {
                if rgbh_meta.h_div == last_slot.h_div {
                    last_slot.update(rgbh_meta);
                } else {
                    let last_chip_idx = last_slot.h_div as u32;
                    last_slot.clear_sel_lat();
                    parser.add_color(&last_slot.buf, last_chip_idx, last_slot.last_chip_idx);
                    last_slot = PixelSlot::new(rgbh_meta, last_chip_idx + 1, Some(&last_slot.buf));
                }
                continue;
            }

            last_slot.clear_sel_lat();
            parser.add_color_end(
                &last_slot.buf,
                last_slot.h_div as u32,
                last_slot.last_chip_idx,
            );
            last_slot = PixelSlot::new(rgbh_meta, 0, Some(&last_slot.buf));
            parser.add_empty_les((rgbh_meta.h_mod - last_h_mod - 1) as u32);
            last_h_mod = rgbh_meta.h_mod;
        }

        parser.add_color_end(
            &last_slot.buf,
            last_slot.h_div as u32,
            last_slot.last_chip_idx,
        );
    }
    parser.add_empty_les(15 - last_h_mod as u32);
    parser.add_sync(8);
    parser.add_empty(8);
    parser.encode()
}
