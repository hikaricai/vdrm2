use mbi5264_common::pio::MAX_FRAME_WORDS;
use mbi5264_common::preencoded::{
    FrameEntry, Header, FRAME_ENTRY_SIZE, HEADER_SIZE, MAX_CACHED_IMAGE_BYTES,
};

pub struct DmaBuf {
    pub img_angle: u32,
    pub ptr: u32,
    pub len: u32,
}

static mut IMAGE_CACHE: [u8; MAX_CACHED_IMAGE_BYTES] = [0; MAX_CACHED_IMAGE_BYTES];

#[inline]
fn align4(value: usize) -> usize {
    (value + 3) & !3
}

#[inline]
unsafe fn flash_slice(addr: usize, len: usize) -> &'static [u8] {
    unsafe { core::slice::from_raw_parts(addr as *const u8, len) }
}

#[inline]
fn cached_image(len: usize) -> &'static [u8] {
    unsafe { core::slice::from_raw_parts(core::ptr::addr_of!(IMAGE_CACHE).cast::<u8>(), len) }
}

struct EncoderCtx {
    image_count: usize,
    image_idx: usize,
    image_len: usize,
    frame_count: usize,
    frame_idx: usize,
    init_angle: u32,
    last_angle: u32,
    max_img_angle: u32,
}

impl EncoderCtx {
    fn new() -> Self {
        let image_count =
            unsafe { (crate::env::IMGS_LEN_ADDR as *const u32).read_unaligned() as usize };
        assert!(image_count > 0);
        rtt_target::rprintln!("total imgs {}", image_count);

        let mut ctx = Self {
            image_count,
            image_idx: 0,
            image_len: 0,
            frame_count: 0,
            frame_idx: 0,
            init_angle: 0,
            last_angle: 0,
            max_img_angle: 0,
        };
        ctx.load_image(0);
        ctx
    }

    #[inline]
    fn image_offset(index: usize) -> usize {
        unsafe {
            (crate::env::IMGS_LIST_ADDR as *const u32)
                .add(index)
                .read_unaligned() as usize
        }
    }

    fn load_image(&mut self, image_idx: usize) {
        let image_addr = crate::env::IMGS_LEN_ADDR + Self::image_offset(image_idx);
        let header_data = unsafe { flash_slice(image_addr, HEADER_SIZE) };
        let header = Header::parse(header_data).expect("invalid PIO image header");
        let frame_count = header.frame_count as usize;
        assert!(frame_count > 0);
        assert!(header.max_frame_words as usize <= MAX_FRAME_WORDS);

        let table_len = HEADER_SIZE + frame_count * FRAME_ENTRY_SIZE;
        let metadata = unsafe { flash_slice(image_addr, table_len) };
        let last_frame =
            FrameEntry::parse(metadata, frame_count - 1).expect("invalid PIO frame table");
        let image_len = align4(
            last_frame.data_offset as usize + last_frame.instruction_len as usize,
        );
        assert!(image_len <= MAX_CACHED_IMAGE_BYTES);

        unsafe {
            core::ptr::copy_nonoverlapping(
                image_addr as *const u8,
                core::ptr::addr_of_mut!(IMAGE_CACHE).cast::<u8>(),
                image_len,
            );
        }

        let image = cached_image(image_len);
        let header = Header::parse(image).unwrap();
        let last_frame = FrameEntry::parse(image, frame_count - 1).unwrap();
        self.image_idx = image_idx;
        self.image_len = image_len;
        self.frame_count = frame_count;
        self.frame_idx = 0;
        self.init_angle = header.init_angle;
        self.last_angle = 0;
        self.max_img_angle = last_frame.angle;

        rtt_target::rprintln!(
            "image {} frames {} program {} max_frame {}",
            image_idx,
            frame_count,
            image_len,
            header.max_frame_words * 4
        );
    }

    fn update_to_next_image(&mut self) {
        self.load_image((self.image_idx + 1) % self.image_count);
    }

    #[inline(always)]
    fn next_frame(&mut self, angle: u32) -> Option<FrameEntry> {
        if angle < self.last_angle || self.frame_idx >= self.frame_count {
            self.frame_idx = 0;
        }
        self.last_angle = angle;
        if angle > self.max_img_angle {
            return None;
        }

        let image = cached_image(self.image_len);
        while self.frame_idx < self.frame_count {
            let frame = FrameEntry::parse(image, self.frame_idx).unwrap();
            self.frame_idx += 1;
            if frame.angle >= angle {
                return Some(frame);
            }
        }
        None
    }
}

pub struct Encoder {
    ctx: EncoderCtx,
    buf_idx: usize,
    buf0: [u32; MAX_FRAME_WORDS],
    buf1: [u32; MAX_FRAME_WORDS],
}

impl Encoder {
    pub fn new() -> Self {
        Self {
            ctx: EncoderCtx::new(),
            buf_idx: 0,
            buf0: [0; MAX_FRAME_WORDS],
            buf1: [0; MAX_FRAME_WORDS],
        }
    }

    pub fn update_to_next_image(&mut self) {
        self.ctx.update_to_next_image();
    }

    pub fn init_angle(&self) -> u32 {
        self.ctx.init_angle
    }

    #[link_section = ".data.ram_code"]
    #[inline(never)]
    pub fn encode_next(&mut self, angle: u32) -> Option<DmaBuf> {
        let frame = self.ctx.next_frame(angle)?;
        let image = cached_image(self.ctx.image_len);
        let start = frame.data_offset as usize;
        let end = start.checked_add(frame.instruction_len as usize)?;
        let program = image.get(start..end)?;
        let dma_words = frame.dma_words as usize;
        if dma_words > MAX_FRAME_WORDS {
            return None;
        }

        self.buf_idx ^= 1;
        let output = if self.buf_idx == 0 {
            &mut self.buf0
        } else {
            &mut self.buf1
        };
        let decoded = unsafe {
            mbi5264_common::pio::decode_frame_program_unchecked(
                program.as_ptr(),
                program.len(),
                output.as_mut_ptr(),
            )
        };
        if decoded != dma_words {
            return None;
        }

        Some(DmaBuf {
            img_angle: frame.angle,
            ptr: output.as_ptr() as u32,
            len: frame.dma_words,
        })
    }
}
