use embassy_rp::gpio::{self, Input};
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    zerocopy_channel,
};
use embassy_time::{Duration, Instant, Timer};
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
            self.img_idx = 0;
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
        let mut parser = crate::clocks::ColorParser::new(buf);
        let len = update_frame(&mut parser, &angle_line.coloum);
        DmaBuf {
            img_angle: angle_line.angle,
            ptr: buf.as_ptr() as u32,
            len,
        }
    }
}

#[embassy_executor::task]
pub async fn encode_mbi_vdrm(
    mut mbi_tx: zerocopy_channel::Sender<'static, NoopRawMutex, MbiBuf2>,
    encode_sinal: &'static embassy_sync::signal::Signal<NoopRawMutex, u32>,
) {
    const IMG_BIN: &[u8] = include_bytes!("../img.bin");
    assert!(IMG_BIN.len() >= core::mem::size_of::<mbi5264_common::AngleImage>());
    let img = unsafe {
        core::slice::from_raw_parts(
            IMG_BIN.as_ptr() as *const mbi5264_common::AngleImage,
            IMG_BIN.len() / core::mem::size_of::<mbi5264_common::AngleImage>(),
        )
    };
    rtt_target::rprintln!("total angles {}", img.len());
    rtt_target::rprintln!("first angle {}", img[0].angle);

    const INDEX_MOD: usize = 4;

    let p = unsafe { embassy_rp::Peripherals::steal() };
    let mut dbg_pin20 = gpio::Output::new(p.PIN_20, gpio::Level::Low);
    let mut dbg_pin21 = gpio::Output::new(p.PIN_21, gpio::Level::Low);

    let mut idx_mod = 0usize;
    let mut img_idx = 0usize;
    let mut find_angle_line = |angle: u32| {
        let dbg = crate::DBG.load(core::sync::atomic::Ordering::Relaxed);
        loop {
            if img_idx >= img.len() {
                idx_mod += 1;
                idx_mod %= INDEX_MOD;
                img_idx = idx_mod;
                // rtt_target::rprintln!("idx_mod {} img_idx {}", idx_mod, img_idx);
            }
            let angle_line = &img[img_idx];
            img_idx += INDEX_MOD;
            if angle_line.angle > crate::SHOW_ANGLE_MAX {
                continue;
            }
            if angle_line.angle >= angle {
                if dbg {
                    rtt_target::rprintln!("pars img_idx {} img {}", img_idx, angle_line.angle,);
                }
                return angle_line;
            }
        }
    };
    loop {
        // rtt_target::rprintln!("encode loop {}", index_mod);
        dbg_pin20.set_high();
        dbg_pin21.set_high();
        let mbi_buf = mbi_tx.send().await;
        let exp_angle = encode_sinal.wait().await;
        let angle_line = find_angle_line(exp_angle);
        dbg_pin20.set_low();
        let mut parser = crate::clocks::ColorParser::new(&mut mbi_buf.buf);
        let len = update_frame(&mut parser, &angle_line.coloum);
        dbg_pin21.set_low();
        mbi_buf.len = len;
        mbi_buf.angle = angle_line.angle;
        mbi_tx.send_done();
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

fn block_update_frame(
    parser: &mut crate::clocks::ColorParser,
    cmd_pio: &mut crate::clocks::CmdClock,
    rgbh_coloum: &[crate::RGBH; crate::IMG_HEIGHT],
) {
    let _len = update_frame(parser, rgbh_coloum);
    parser.run(cmd_pio);
}

async fn async_update_frame(
    parser: &mut crate::clocks::ColorParser<'_>,
    cmd_pio: &mut crate::clocks::CmdClock,
    rgbh_coloum: &[crate::RGBH; crate::IMG_HEIGHT],
) {
    let len = update_frame(parser, rgbh_coloum);
    cmd_pio.refresh_ptr(parser.buf_ori as u32, len);
    cmd_pio.wait().await;
}

fn update_frame(
    parser: &mut crate::clocks::ColorParser,
    rgbh_coloum: &[crate::RGBH; crate::IMG_HEIGHT],
) -> u32 {
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

pub struct MbiBuf2 {
    pub angle: u32,
    pub len: u32,
    pub buf: [u16; 16384],
    pub is_last: bool,
}

impl MbiBuf2 {
    pub const fn new(angle: u32, is_last: bool) -> Self {
        Self {
            angle,
            len: 0,
            buf: [0u16; 16384],
            is_last,
        }
    }
}
