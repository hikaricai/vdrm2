const IMG_BIN: &[u8] = include_bytes!("../img.bin");

const INDEX_MOD: usize = 4;

pub struct DmaBuf {
    pub img_angle: u32,
    pub ptr: u32,
    pub len: u32,
}

struct EncoderCtx {
    img: &'static [mbi5264_common::AngleImage],
    idx_mod: usize,
    img_idx: usize,
    last_angle: u32,
    max_img_angle: u32,
}

impl EncoderCtx {
    fn new() -> Self {
        assert!(IMG_BIN.len() >= core::mem::size_of::<mbi5264_common::AngleImage>());
        let img: &'static [mbi5264_common::AngleImage] = unsafe {
            core::slice::from_raw_parts(
                IMG_BIN.as_ptr() as *const mbi5264_common::AngleImage,
                IMG_BIN.len() / core::mem::size_of::<mbi5264_common::AngleImage>(),
            )
        };
        rtt_target::rprintln!("total angles {}", img.len());
        rtt_target::rprintln!("first angle {}", img[0].angle);

        Self {
            img,
            idx_mod: 0,
            img_idx: 0,
            last_angle: 0,
            max_img_angle: img.last().unwrap().angle,
        }
    }

    fn next_img_line(&mut self, angle: u32) -> &mbi5264_common::AngleImage {
        if angle < self.last_angle || self.img_idx >= self.img.len() {
            self.img_idx = self.idx_mod;
            self.idx_mod += 1;
            self.idx_mod %= INDEX_MOD;
        }
        self.last_angle = angle;
        if angle >= self.max_img_angle {
            return self.img.last().unwrap();
        }
        loop {
            let angle_line = &self.img[self.img_idx];
            self.img_idx += INDEX_MOD;

            if angle_line.angle >= angle {
                return angle_line;
            }
        }
    }
}

pub struct Encoder {
    ctx: EncoderCtx,
    buf_idx: usize,
    buf0: [u16; 16384],
    buf1: [u16; 16384],
}

impl Encoder {
    pub fn new() -> Self {
        Self {
            ctx: EncoderCtx::new(),
            buf_idx: 0,
            buf0: [0; 16384],
            buf1: [0; 16384],
        }
    }
    pub fn encode_next(&mut self, angle: u32) -> DmaBuf {
        let angle_line = self.ctx.next_img_line(angle);
        self.buf_idx += 1;
        let buf = if self.buf_idx & 1 > 0 {
            &mut self.buf1
        } else {
            &mut self.buf0
        };
        let mut parser = ColorParser::new(buf);
        let len = update_frame(&mut parser, &angle_line.coloum);
        DmaBuf {
            img_angle: angle_line.angle,
            ptr: buf.as_ptr() as u32,
            len,
        }
    }
}

struct RGBMeta {
    rgbh: [u8; 4],
    h_div: u8,
    h_mod: u8,
    region: u16,
}

impl RGBMeta {
    #[inline]
    fn new(rgbh: [u8; 4], region: u16) -> Self {
        let h = rgbh[3];
        let h_div = (h >> 4) & 0x0F;
        let h_mod = h & 0x0F;
        Self {
            rgbh,
            h_div,
            h_mod,
            region,
        }
    }
}

fn update_frame(parser: &mut ColorParser, rgbh_coloum: &[crate::RGBH; crate::IMG_HEIGHT]) -> u32 {
    let region0 = &rgbh_coloum[0..64];
    let region1 = &rgbh_coloum[64..128];
    let region2 = &rgbh_coloum[128..];
    // init last_h_mod with 15, so the first line's "empty" is first h_mod
    let mut last_h_mod = 15;
    // FIXME maybe gclk share same sram
    // will cause image brocken if too small
    parser.add_empty(50);
    // parser.add_empty(5000);
    for line in 0..64usize {
        // rtt_target::rprintln!("line {}", line);
        // TODO optimize speed
        let p0 = RGBMeta::new(region0[line], 0);
        let p1 = RGBMeta::new(region1[line], 1);
        let p2 = RGBMeta::new(region2[line], 2);
        let mut pixels = [p0, p1, p2];
        bubble_rgbh2(&mut pixels);
        let mut pixel_iter = pixels.iter();
        let last_pixel = pixel_iter.next().unwrap();
        let mut last_solt = PixelSlot2::new(last_pixel, 0);

        let empty = if last_solt.h_mod > last_h_mod {
            15 + last_solt.h_mod - last_h_mod
        } else {
            15 - (last_h_mod - last_solt.h_mod)
        };
        last_h_mod = last_solt.h_mod;
        parser.add_empty_les(empty as u32);

        for rgbh_meta in pixel_iter {
            if rgbh_meta.h_mod == last_solt.h_mod {
                if rgbh_meta.h_div == last_solt.h_div {
                    last_solt.update(&rgbh_meta);
                } else {
                    // new chip_idx
                    let last_chip_idx = last_solt.h_div as u32;
                    parser.add_color2(&last_solt.buf, last_chip_idx, last_solt.last_chip_idx);
                    last_solt = PixelSlot2::new(rgbh_meta, last_chip_idx + 1);
                }
                continue;
            }
            // assume data is optimized
            unreachable!();

            #[allow(unreachable_code)]
            parser.add_color_end(
                &last_solt.buf,
                last_solt.h_div as u32,
                last_solt.last_chip_idx,
            );

            last_solt = PixelSlot2::new(rgbh_meta, 0);
            let empty = rgbh_meta.h_mod - last_h_mod - 1;
            parser.add_empty_les(empty as u32);
            last_h_mod = rgbh_meta.h_mod;
        }

        parser.add_color_end(
            &last_solt.buf,
            last_solt.h_div as u32,
            last_solt.last_chip_idx,
        );
    }
    parser.add_empty_les(15 - last_h_mod as u32);
    //
    parser.add_sync(8);
    parser.add_empty(8);
    parser.encode()
}

#[inline]
fn bubble_rgbh2(slice: &mut [RGBMeta]) {
    let len = slice.len();
    for i in 0..len {
        for j in 0..len - 1 - i {
            let (l, r) = (&slice[j], &slice[j + 1]);
            if (l.h_mod, l.h_div) > (r.h_mod, r.h_div) {
                // Swap elements
                slice.swap(j, j + 1);
            }
        }
    }
}

struct PixelSlot2 {
    buf: [u16; 8],
    h_div: u8,
    h_mod: u8,
    last_chip_idx: u32,
}

impl PixelSlot2 {
    #[inline]
    fn new(rgbh_meta: &RGBMeta, last_chip_idx: u32) -> Self {
        let mut buf = [0u16; 8];
        let &RGBMeta {
            rgbh,
            region,
            h_div,
            h_mod,
        } = rgbh_meta;
        let [r, g, b, _h] = rgbh;
        for (i, buf) in (0..8).rev().zip(buf.iter_mut()) {
            let r = (r >> i) & 1;
            let g = (g >> i) & 1;
            let b = (b >> i) & 1;
            let rgb = (r | (g << 1) | (b << 2)) as u16;
            *buf |= rgb << (3 * region);
        }
        Self {
            buf,
            h_div,
            h_mod,
            last_chip_idx,
        }
    }

    #[inline]
    fn update(&mut self, rgbh_meta: &RGBMeta) {
        let region = rgbh_meta.region;
        let [r, g, b, _] = rgbh_meta.rgbh;
        for (i, buf) in (0..8).rev().zip(self.buf.iter_mut()) {
            let r = (r >> i) & 1;
            let g = (g >> i) & 1;
            let b = (b >> i) & 1;
            let rgb = (r | (g << 1) | (b << 2)) as u16;
            *buf |= rgb << (3 * region);
        }
    }
}

#[repr(C)]
struct ColorTranser {
    empty_loops: u32,
    data_loops: u32,
    buf: [u16; 8],
}

#[repr(C)]
struct ColorTranserTail {
    empty_loops: u32,
    data_loops: u32,
    buf: [u16; 2],
}

pub struct ColorParser<'a> {
    pub loops: &'a mut u32,
    pub buf: *mut u16,
    pub buf_ori: *mut u16,
    pub last_empties: u32,
}

impl<'a> ColorParser<'a> {
    pub fn new(buf: &'a mut [u16]) -> Self {
        let buf_ori = buf.as_mut_ptr();
        let buf = unsafe { buf.as_mut_ptr().add(2) };
        let loops: &mut u32 = unsafe { core::mem::transmute(buf_ori) };
        *loops = 0;
        Self {
            loops,
            buf,
            buf_ori,
            last_empties: 0,
        }
    }

    pub fn encode(&mut self) -> u32 {
        *self.loops -= 1;
        let len = unsafe { self.buf.offset_from(self.buf_ori) } as u32 / 2;
        len
    }

    #[inline]
    fn reduce_empty_loops(last_empties: u32, required_empty_loops: u32) -> u32 {
        let mut empty_loops = if last_empties > required_empty_loops {
            0u32
        } else {
            required_empty_loops - last_empties
        };
        if empty_loops >= 3 {
            empty_loops -= 3;
        }
        empty_loops
    }
    pub fn add_empty_les(&mut self, empty_size: u32) {
        const EMPTY_LEN_U32_CYCLES: usize = 8;
        if empty_size == 0 {
            return;
        }
        unsafe {
            *self.loops += 1;

            // empty with le
            let meta: &mut ColorTranserTail = add_buf_ptr(&mut self.buf);
            let empty_loops: u32 = 16 * 9;
            meta.empty_loops = Self::reduce_empty_loops(self.last_empties, empty_loops);
            meta.data_loops = 2 - 2;
            meta.buf = [0, crate::clocks::LE_HIGH];

            // many le
            if empty_size <= 1 {
                return;
            }
            for _i in 1..empty_size {
                *self.loops += 1;
                let meta: &mut ColorTranserTail = add_buf_ptr(&mut self.buf);
                meta.empty_loops = EMPTY_LEN_U32_CYCLES as u32 * 2 - 3;
                meta.data_loops = 2 - 2;
                meta.buf = [0, crate::clocks::LE_HIGH];
            }
        }
        self.last_empties = 16 * 9;
    }

    pub fn add_color2(&mut self, buf: &[u16; 8], chip_index: u32, last_chip_idx: u32) {
        let le = chip_index == 8;
        let chip_inc_index = chip_index - last_chip_idx;
        let empty_loops = chip_inc_index * 16 + 8;
        unsafe {
            *self.loops += 1;

            let transfer: &mut ColorTranser = add_buf_ptr(&mut self.buf);
            // -1
            transfer.empty_loops = Self::reduce_empty_loops(self.last_empties, empty_loops);
            transfer.data_loops = 8 - 2;
            transfer.buf = *buf;
            if le {
                transfer.data_loops += 8;
                let le_buf: &mut [u16; 8] = add_buf_ptr(&mut self.buf);
                *le_buf = [0; 8];
                le_buf[7] = crate::clocks::LE_HIGH;
            }
        }
        self.last_empties = 0;
    }

    fn add_empty_le(&mut self, chip_inc_index: u32) {
        let empty_loops = chip_inc_index * 16 + 8 - 2;
        unsafe {
            *self.loops += 1;
            let tail: &mut ColorTranserTail = add_buf_ptr(&mut self.buf);
            tail.empty_loops = empty_loops - 3;
            tail.data_loops = 2 - 2;
            // LE
            tail.buf[0] = 0;
            tail.buf[1] = crate::clocks::LE_HIGH;
        }
        self.last_empties = empty_loops;
    }

    pub fn add_color_end(&mut self, buf: &[u16; 8], chip_index: u32, last_chip_idx: u32) {
        let le = chip_index == 8;
        self.add_color2(buf, chip_index, last_chip_idx);
        if !le {
            self.add_empty_le(8 - chip_index as u32);
        }
    }

    pub fn add_sync(&mut self, empty_loops: u32) {
        unsafe {
            *self.loops += 1;
            let tail: &mut ColorTranserTail = add_buf_ptr(&mut self.buf);
            tail.empty_loops = empty_loops;
            tail.data_loops = 2 - 2;
            // LE
            tail.buf[0] = crate::clocks::LE_HIGH;
            tail.buf[1] = crate::clocks::LE_HIGH;
        }
    }

    pub fn add_empty(&mut self, empty_loops: u32) {
        if empty_loops == 0 {
            return;
        }
        unsafe {
            *self.loops += 1;
            let tail: &mut ColorTranserTail = add_buf_ptr(&mut self.buf);
            tail.empty_loops = empty_loops + 8 * 1 - 3;
            tail.data_loops = 2 - 2;
            // LE
            tail.buf[0] = 0;
            tail.buf[1] = 0;
        }
    }
}

unsafe fn add_buf_ptr<B, T>(buf: &mut *mut B) -> &mut T {
    let t: &mut T = core::mem::transmute(*buf);
    *buf = buf.add(core::mem::size_of::<T>() / core::mem::size_of::<B>());
    t
}
