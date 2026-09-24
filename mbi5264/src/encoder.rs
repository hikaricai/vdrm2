use mbi5264_common::SERIAL_CHIPS;

const INDEX_MOD: usize = 1;

const LAST_CHIP_IDX: u32 = SERIAL_CHIPS - 1;
pub struct DmaBuf {
    pub img_angle: u32,
    pub ptr: u32,
    pub len: u32,
}

// 再大就异常了
const RAM_IMG_SIZE: usize = 300;
static mut IMG_RAM: [mbi5264_common::AngleImage; RAM_IMG_SIZE] =
    [mbi5264_common::AngleImage::new(0); RAM_IMG_SIZE];

fn load_img(img_addr_offset: usize) -> (&'static [mbi5264_common::AngleImage], u32) {
    let (img, init_angle): (&'static [mbi5264_common::AngleImage], u32) = unsafe {
        let img_base_addr = crate::env::IMGS_LEN_ADDR + img_addr_offset;
        let img_len_addr = img_base_addr + core::mem::size_of::<u32>();

        let init_angle = *(img_base_addr as *const u32);

        let img_addr = img_len_addr + core::mem::size_of::<u32>();
        let len = *(img_len_addr as *const u32);
        let img_ref: &'static [mbi5264_common::AngleImage] = core::slice::from_raw_parts(
            img_addr as *const mbi5264_common::AngleImage,
            len as usize,
        );
        let len = core::cmp::min(RAM_IMG_SIZE, len as usize);
        let img_ram = core::slice::from_raw_parts_mut(
            IMG_RAM.as_ptr() as *mut mbi5264_common::AngleImage,
            len,
        );
        for (line_ram, line) in img_ram.iter_mut().zip(img_ref) {
            // rtt_target::rprintln!("iter line {}", line.angle);
            *line_ram = *line;
            // rtt_target::rprintln!("line_ram {}", line_ram.angle);
        }
        (img_ram, init_angle)
    };
    (img, init_angle)
}
struct EncoderCtx {
    imgs_addr_list: &'static [u32],
    img_idx: usize,
    init_angle: u32,
    img: &'static [mbi5264_common::AngleImage],
    idx_mod: usize,
    line_idx: usize,
    last_angle: u32,
    max_img_angle: u32,
}

impl EncoderCtx {
    fn new() -> Self {
        let imgs_addr_list = unsafe {
            let len = *(crate::env::IMGS_LEN_ADDR as *const u32);
            rtt_target::rprintln!("total imgs {}", len);
            let imgs_addr_list: &'static [u32] =
                core::slice::from_raw_parts(crate::env::IMGS_LIST_ADDR as *const u32, len as usize);
            for imgs_addr in imgs_addr_list {
                rtt_target::rprintln!("imgs_addr {}", *imgs_addr);
            }
            imgs_addr_list
        };

        let (img, init_angle) = load_img(imgs_addr_list[0] as usize);

        rtt_target::rprintln!("total angles {}", img.len());
        rtt_target::rprintln!("first line_angle {}", img[0].angle);
        rtt_target::rprintln!("init_angle {}", init_angle);

        Self {
            init_angle,
            imgs_addr_list,
            img_idx: 0,
            img,
            idx_mod: 0,
            line_idx: 0,
            last_angle: 0,
            max_img_angle: img.last().unwrap().angle,
        }
    }

    pub fn update_to_next_image(&mut self) {
        let img_idx = (self.img_idx + 1) % self.imgs_addr_list.len();
        let (img, init_angle) = load_img(self.imgs_addr_list[img_idx] as usize);
        self.img_idx = img_idx;
        self.init_angle = init_angle;
        self.img = img;
        self.idx_mod = 0;
        self.line_idx = 0;
        self.last_angle = 0;
        self.max_img_angle = img.last().unwrap().angle;
    }

    #[inline(always)]
    fn next_img_line(&mut self, angle: u32) -> Option<&mbi5264_common::AngleImage> {
        if angle < self.last_angle {
            self.line_idx = self.idx_mod;
            self.idx_mod += 1;
            self.idx_mod %= INDEX_MOD;
        }
        if self.line_idx >= self.img.len() {
            self.line_idx = self.idx_mod;
        }
        self.last_angle = angle;
        if angle > self.max_img_angle {
            return None;
        }
        loop {
            let angle_line = &self.img[self.line_idx];
            self.line_idx += INDEX_MOD;
            if self.line_idx >= self.img.len() {
                self.line_idx = self.idx_mod;
                return None;
            }
            if angle_line.angle >= angle {
                return Some(angle_line);
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
    pub fn update_to_next_image(&mut self) {
        self.ctx.update_to_next_image();
    }
    pub fn init_angle(&self) -> u32 {
        self.ctx.init_angle
    }
    // Keep the complete hot path in SRAM. cortex-m-rt copies .data.* from
    // flash to RAM before main, and the callees below are forced inline.
    #[link_section = ".data.ram_code"]
    #[inline(never)]
    pub fn encode_next(&mut self, angle: u32) -> Option<DmaBuf> {
        let angle_line = self.ctx.next_img_line(angle)?;
        self.buf_idx += 1;
        let buf = if self.buf_idx & 1 > 0 {
            &mut self.buf1
        } else {
            &mut self.buf0
        };
        let mut parser = ColorParser::new(buf);
        let len = update_frame(&mut parser, &angle_line.coloum);
        Some(DmaBuf {
            img_angle: angle_line.angle,
            ptr: buf.as_ptr() as u32,
            len,
        })
    }
}

struct RGBMeta {
    rgbh: mbi5264_common::RGBH,
    h_div: u8,
    h_mod: u8,
    region: u16,
}

impl RGBMeta {
    #[inline(always)]
    fn new(rgbh: mbi5264_common::RGBH, region: u16) -> Self {
        let h = rgbh.h();
        let h_div = (h >> 4) & 0x0F;
        let h_div = h_div % SERIAL_CHIPS as u8;
        let h_mod = h & 0x0F;
        Self {
            rgbh,
            h_div,
            h_mod,
            region,
        }
    }
}

#[inline(always)]
pub fn update_frame(
    parser: &mut ColorParser,
    rgbh_coloum: &[mbi5264_common::RGBH; crate::IMG_HEIGHT],
) -> u32 {
    let region0 = &rgbh_coloum[0..64];
    let region1 = &rgbh_coloum[64..128];
    let region2 = &rgbh_coloum[128..];
    // init last_h_mod with 15, so the first line's "empty" is first h_mod
    let mut last_h_mod = 15;
    // FIXME maybe gclk share same sram
    // will cause image brocken if too small
    parser.add_empty(10);
    // parser.add_empty(5000);
    for line in 0..64usize {
        // rtt_target::rprintln!("line {}", line);
        // TODO optimize speed
        let p0 = RGBMeta::new(region0[line], 0);
        let p1 = RGBMeta::new(region1[line], 1);
        let p2 = RGBMeta::new(region2[line], 2);
        let mut pixels = [p0, p1, p2];
        bubble_rgbh(&mut pixels);
        let mut pixel_iter = pixels.iter();
        let last_pixel = pixel_iter.next().unwrap();
        let mut last_solt = PixelSlot::new(last_pixel, 0, None);

        let empty = if last_solt.h_mod > last_h_mod {
            15 + last_solt.h_mod - last_h_mod
        } else {
            15 - (last_h_mod - last_solt.h_mod)
        };
        last_h_mod = last_solt.h_mod;
        parser.add_empty_les(empty as u32);
        parser.new_line = true;

        for rgbh_meta in pixel_iter {
            if rgbh_meta.h_mod == last_solt.h_mod {
                if rgbh_meta.h_div == last_solt.h_div {
                    last_solt.update(&rgbh_meta);
                } else {
                    // new chip_idx
                    let last_chip_idx = last_solt.h_div as u32;
                    last_solt.clear_sel_lat();
                    parser.add_color(&last_solt.buf, last_chip_idx, last_solt.last_chip_idx);
                    last_solt = PixelSlot::new(rgbh_meta, last_chip_idx + 1, Some(&last_solt.buf));
                }
                continue;
            }
            // assume data is optimized
            // unreachable!();

            last_solt.clear_sel_lat();
            #[allow(unreachable_code)]
            parser.add_color_end(
                &last_solt.buf,
                last_solt.h_div as u32,
                last_solt.last_chip_idx,
            );
            last_solt = PixelSlot::new(rgbh_meta, 0, Some(&last_solt.buf));
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

#[inline(always)]
fn bubble_rgbh(slice: &mut [RGBMeta]) {
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

struct PixelSlot {
    buf: [u32; 4],
    h_div: u8,
    h_mod: u8,
    last_chip_idx: u32,
}

const SEL_PAIR: u32 = 0x0008_0008;
const SEL_LAT_PAIR: u32 = 0x4000_4000;
const SEL_DATA_MASK_PAIR: u32 = 0x0888_0888;

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

#[link_section = ".data.ram_code"]
static NIBBLE_PLANES: [[u32; 2]; 16] = make_nibble_planes();

#[inline(always)]
fn pixel_planes(rgbh: mbi5264_common::RGBH) -> [u32; 4] {
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

impl PixelSlot {
    #[inline(always)]
    fn new(rgbh_meta: &RGBMeta, last_chip_idx: u32, last_buf: Option<&[u32; 4]>) -> Self {
        let mut buf = [0u32; 4];
        if let Some(last_buf) = last_buf {
            for (b, last_b) in buf.iter_mut().zip(last_buf) {
                *b = *last_b & SEL_DATA_MASK_PAIR;
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
        for (buf, plane) in buf.iter_mut().zip(planes) {
            *buf |= (plane << shift) | SEL_LAT_PAIR;
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
        for (buf, plane) in self.buf.iter_mut().zip(planes) {
            *buf |= plane << shift;
        }
    }
    #[inline(always)]
    fn clear_sel_lat(&mut self) {
        for b in self.buf.iter_mut() {
            *b &= !SEL_LAT_PAIR;
        }
    }
}

#[repr(C)]
struct ColorTranser {
    empty_loops: u32,
    data_loops: u32,
    buf: [u32; 4],
}

#[repr(C)]
struct ColorTranserTail {
    empty_loops: u32,
    data_loops: u32,
    buf: [u16; 2],
}

// state machine 先写empty loops 再写data
// 对于每个pixel 先发8bit empty 再补8bit data 但是mbi5264是16bit msb 此时的8bit无法点亮屏幕
// 最后le时会补8bit empty
pub struct ColorParser<'a> {
    loops: u32,
    buf: *mut u16,
    buf_ori: *mut u16,
    last_empties: u32,
    pub new_line: bool,
    _buf: core::marker::PhantomData<&'a mut [u16]>,
}

impl<'a> ColorParser<'a> {
    #[inline(always)]
    pub fn new(buf: &'a mut [u16]) -> Self {
        let buf_ori = buf.as_mut_ptr();
        let buf = unsafe { buf.as_mut_ptr().add(2) };
        Self {
            loops: 0,
            buf,
            buf_ori,
            last_empties: 0,
            new_line: false,
            _buf: core::marker::PhantomData,
        }
    }

    #[inline(always)]
    pub fn encode(&mut self) -> u32 {
        unsafe {
            self.buf_ori.cast::<u32>().write_unaligned(self.loops - 1);
            self.buf.offset_from(self.buf_ori) as u32 / 2
        }
    }

    #[inline(always)]
    fn reduce_empty_loops(last_empties: u32, required_empty_loops: u32) -> u32 {
        required_empty_loops.saturating_sub(last_empties + 3)
    }

    #[inline(always)]
    unsafe fn push_tail(&mut self, empty_loops: u32, data_loops: u32, buf: [u16; 2]) {
        self.buf.cast::<ColorTranserTail>().write(ColorTranserTail {
            empty_loops,
            data_loops,
            buf,
        });
        self.buf = self
            .buf
            .add(core::mem::size_of::<ColorTranserTail>() / core::mem::size_of::<u16>());
        self.loops += 1;
    }

    #[inline(always)]
    unsafe fn push_color(&mut self, empty_loops: u32, data_loops: u32, buf: [u32; 4]) {
        self.buf.cast::<ColorTranser>().write(ColorTranser {
            empty_loops,
            data_loops,
            buf,
        });
        self.buf = self
            .buf
            .add(core::mem::size_of::<ColorTranser>() / core::mem::size_of::<u16>());
        self.loops += 1;
    }

    #[inline(always)]
    unsafe fn push_words(&mut self, words: [u32; 4]) {
        self.buf.cast::<[u32; 4]>().write(words);
        self.buf = self.buf.add(8);
    }

    #[inline(always)]
    pub fn add_empty_les(&mut self, empty_size: u32) {
        // 缩减latch的时钟 看起来只是让画面的行偏移了
        const EMPTY_LEN_U32_CYCLES: u32 = 3;
        if empty_size == 0 {
            return;
        }
        let empty_loops: u32 = 16 * SERIAL_CHIPS;
        unsafe {
            self.push_tail(
                Self::reduce_empty_loops(self.last_empties, empty_loops),
                0,
                [0, crate::clocks::LE_HIGH],
            );
            for _ in 1..empty_size {
                self.push_tail(EMPTY_LEN_U32_CYCLES - 3, 0, [0, crate::clocks::LE_HIGH]);
            }
        }
        self.last_empties = empty_loops;
    }

    #[inline(always)]
    pub fn add_color(&mut self, buf: &[u32; 4], chip_index: u32, last_chip_idx: u32) {
        let le = chip_index == LAST_CHIP_IDX;
        let chip_inc_index = chip_index - last_chip_idx;
        let empty_loops = chip_inc_index * 16 + 8 * !self.new_line as u32;
        self.new_line = false;
        unsafe {
            let data_loops = if le { 14 } else { 6 };
            self.push_color(
                Self::reduce_empty_loops(self.last_empties, empty_loops),
                data_loops,
                *buf,
            );
            if le {
                self.push_words([0, 0, 0, (crate::clocks::LE_HIGH as u32) << 16]);
            }
        }
        self.last_empties = 0;
    }

    #[inline(always)]
    fn add_empty_le(&mut self, chip_inc_index: u32) {
        let empty_loops = chip_inc_index * 16 + 8;
        unsafe {
            self.push_tail(empty_loops - 5, 0, [0, crate::clocks::LE_HIGH]);
        }
        self.last_empties = empty_loops;
    }

    #[inline(always)]
    fn add_color_end(&mut self, buf: &[u32; 4], chip_index: u32, last_chip_idx: u32) {
        let le = chip_index == LAST_CHIP_IDX;
        self.add_color(buf, chip_index, last_chip_idx);
        if !le {
            self.add_empty_le(LAST_CHIP_IDX - chip_index as u32);
        }
    }

    #[inline(always)]
    pub fn add_sync(&mut self, empty_loops: u32) {
        unsafe {
            self.push_tail(
                empty_loops,
                0,
                [crate::clocks::LE_HIGH, crate::clocks::LE_HIGH],
            );
        }
    }

    #[inline(always)]
    pub fn add_empty(&mut self, empty_loops: u32) {
        if empty_loops == 0 {
            return;
        }
        unsafe {
            self.push_tail(empty_loops + 5, 0, [0, 0]);
        }
    }
}
