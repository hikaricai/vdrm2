use crate::{RGBH, SERIAL_CHIPS};

/// Maximum number of 32-bit words accepted by the PIO DMA for one angle frame.
pub const MAX_FRAME_WORDS: usize = 8192;
pub const MAX_FRAME_PROGRAM_BYTES: usize = MAX_FRAME_WORDS * 4;
pub const LE_HIGH: u16 = 1 << 12;

const LAST_CHIP_IDX: u32 = SERIAL_CHIPS - 1;
const SEL_PAIR: u32 = 0x0008_0008;
const SEL_LAT_PAIR: u32 = 0x4000_4000;
const SEL_DATA_MASK_PAIR: u32 = 0x0888_0888;
const PIO_DATA_MASK: u32 = 0x7fff_7fff;
const TAIL_RUN_MAX: usize = 32;
const TAIL_PATTERNS: [[u32; 3]; 7] = [
    [0, 0, (LE_HIGH as u32) << 16],
    [5, 0, (LE_HIGH as u32) << 16],
    [8, 0, LE_HIGH as u32 | (LE_HIGH as u32) << 16],
    [13, 0, 0],
    [15, 0, 0],
    [19, 0, (LE_HIGH as u32) << 16],
    [29, 0, (LE_HIGH as u32) << 16],
];
const COLOR_EMPTY_LOOPS: [u32; 3] = [0, 5, 13];

#[cfg_attr(target_os = "none", link_section = ".data.ram_code")]
static DECODE_TAIL_WORD0: [u32; 7] = [0, 5, 8, 13, 15, 19, 29];
#[cfg_attr(target_os = "none", link_section = ".data.ram_code")]
static DECODE_TAIL_WORD2: [u32; 7] = [
    (LE_HIGH as u32) << 16,
    (LE_HIGH as u32) << 16,
    LE_HIGH as u32 | (LE_HIGH as u32) << 16,
    0,
    0,
    (LE_HIGH as u32) << 16,
    (LE_HIGH as u32) << 16,
];
#[cfg_attr(target_os = "none", link_section = ".data.ram_code")]
static DECODE_COLOR_EMPTY_LOOPS: [u32; 3] = COLOR_EMPTY_LOOPS;

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

struct ProgramWriter<'a> {
    output: &'a mut [u8],
    offset: usize,
    bitmap_offset: usize,
    command_slot: u8,
}

impl<'a> ProgramWriter<'a> {
    fn new(output: &'a mut [u8]) -> Self {
        Self {
            output,
            offset: 0,
            bitmap_offset: 0,
            command_slot: 0,
        }
    }

    #[inline]
    fn begin_command(&mut self, color: bool) {
        if self.command_slot == 0 {
            self.bitmap_offset = self.offset;
            self.output[self.offset] = 0;
            self.offset += 1;
        }
        if color {
            self.output[self.bitmap_offset] |= 1 << self.command_slot;
        }
        self.command_slot = (self.command_slot + 1) & 7;
    }

    #[inline]
    fn push_tail(&mut self, kind: usize, repeat: usize) {
        assert!(kind < TAIL_PATTERNS.len());
        assert!((1..=TAIL_RUN_MAX).contains(&repeat));
        self.begin_command(false);
        self.output[self.offset] = ((kind * TAIL_RUN_MAX) + repeat - 1) as u8;
        self.offset += 1;
    }

    #[inline]
    fn push_color(&mut self, empty_loops: u32, le: bool, words: &[u32]) {
        let empty_kind = COLOR_EMPTY_LOOPS
            .iter()
            .position(|&value| value == empty_loops)
            .expect("unsupported color empty loops");
        let metadata = (empty_kind << 1) | le as usize;

        self.begin_command(true);
        for (index, &word) in words.iter().enumerate() {
            assert_eq!(word & !PIO_DATA_MASK, 0);
            let metadata_lo = ((metadata >> (index * 2)) & 1) as u32;
            let metadata_hi = ((metadata >> (index * 2 + 1)) & 1) as u32;
            let encoded = word | (metadata_lo << 15) | (metadata_hi << 31);
            let end = self.offset + 4;
            self.output[self.offset..end].copy_from_slice(&encoded.to_le_bytes());
            self.offset = end;
        }
    }
}

/// Converts a DMA word stream into compact, PIO-specific expansion instructions.
pub fn encode_frame_program(words: &[u32], output: &mut [u8]) -> usize {
    assert!(!words.is_empty());
    let loop_count = words[0] as usize + 1;
    let mut writer = ProgramWriter::new(output);
    let mut word_offset = 1usize;
    let mut loops = 0usize;

    while loops < loop_count {
        let empty_loops = words[word_offset];
        let data_loops = words[word_offset + 1];
        if data_loops == 0 {
            let tail = &words[word_offset..word_offset + 3];
            let kind = TAIL_PATTERNS
                .iter()
                .position(|pattern| pattern == tail)
                .expect("unsupported tail pattern");
            let mut repeat = 1usize;
            while loops + repeat < loop_count {
                let next_offset = word_offset + repeat * 3;
                if words.get(next_offset..next_offset + 3) != Some(tail) {
                    break;
                }
                repeat += 1;
            }

            let mut remaining = repeat;
            while remaining > 0 {
                let chunk = remaining.min(TAIL_RUN_MAX);
                writer.push_tail(kind, chunk);
                remaining -= chunk;
            }
            word_offset += repeat * 3;
            loops += repeat;
            continue;
        }

        assert!(data_loops == 6 || data_loops == 14);
        let le = data_loops == 14;
        writer.push_color(empty_loops, le, &words[word_offset + 2..word_offset + 6]);
        word_offset += if le { 10 } else { 6 };
        loops += 1;
    }

    assert_eq!(word_offset, words.len());
    writer.offset
}

/// Expands one compact frame program into the exact word stream consumed by PIO DMA.
#[inline]
pub fn decode_frame_program(program: &[u8], output: &mut [u32]) -> Option<usize> {
    let mut input_offset = 0usize;
    let mut output_offset = 1usize;
    let mut loops = 0usize;

    while input_offset < program.len() {
        let bitmap = *program.get(input_offset)?;
        input_offset += 1;
        for command_slot in 0..8 {
            if input_offset == program.len() {
                break;
            }

            if bitmap & (1 << command_slot) == 0 {
                let command = *program.get(input_offset)? as usize;
                input_offset += 1;
                let kind = command / TAIL_RUN_MAX;
                let repeat = command % TAIL_RUN_MAX + 1;
                let pattern = TAIL_PATTERNS.get(kind)?;
                let end = output_offset.checked_add(repeat * pattern.len())?;
                let target = output.get_mut(output_offset..end)?;
                for chunk in target.chunks_exact_mut(pattern.len()) {
                    chunk.copy_from_slice(pattern);
                }
                output_offset = end;
                loops += repeat;
                continue;
            }

            let end = input_offset.checked_add(16)?;
            let payload = program.get(input_offset..end)?;
            let mut color = [0u32; 4];
            for (word, bytes) in color.iter_mut().zip(payload.chunks_exact(4)) {
                *word = u32::from_le_bytes(bytes.try_into().ok()?);
            }
            input_offset = end;

            let metadata = ((color[0] >> 15) & 1)
                | (((color[0] >> 31) & 1) << 1)
                | (((color[1] >> 15) & 1) << 2);
            let empty_kind = (metadata >> 1) as usize;
            let le = metadata & 1 != 0;
            for word in &mut color {
                *word &= PIO_DATA_MASK;
            }

            let color_words = if le { 10 } else { 6 };
            let target = output.get_mut(output_offset..output_offset + color_words)?;
            target[0] = *COLOR_EMPTY_LOOPS.get(empty_kind)?;
            target[1] = if le { 14 } else { 6 };
            target[2..6].copy_from_slice(&color);
            if le {
                target[6..10].copy_from_slice(&[0, 0, 0, (LE_HIGH as u32) << 16]);
            }
            output_offset += color_words;
            loops += 1;
        }
    }

    output[0] = loops.checked_sub(1)?.try_into().ok()?;
    Some(output_offset)
}

/// Expands a build-time validated frame program without per-command bounds checks.
///
/// # Safety
///
/// `program` must contain a valid complete PIO2 frame program. `output` must
/// point to writable storage large enough for the frame's declared DMA word
/// count, and it must not overlap `program`.
#[inline(always)]
pub unsafe fn decode_frame_program_unchecked(
    program: *const u8,
    program_len: usize,
    output: *mut u32,
) -> usize {
    let input_end = program.add(program_len);
    let mut input = program;
    let mut out = output.add(1);
    let mut loops = 0usize;

    while input < input_end {
        let mut bitmap = input.read();
        input = input.add(1);

        for _ in 0..8 {
            if input == input_end {
                break;
            }

            if bitmap & 1 == 0 {
                let command = input.read() as usize;
                input = input.add(1);
                let repeat = (command & (TAIL_RUN_MAX - 1)) + 1;
                let kind = command >> 5;
                let word0 = *DECODE_TAIL_WORD0.get_unchecked(kind);
                let word2 = *DECODE_TAIL_WORD2.get_unchecked(kind);

                for _ in 0..repeat {
                    out.write(word0);
                    out.add(1).write(0);
                    out.add(2).write(word2);
                    out = out.add(3);
                }
                loops += repeat;
            } else {
                let color0 = input.cast::<u32>().read_unaligned();
                let color1 = input.add(4).cast::<u32>().read_unaligned();
                let color2 = input.add(8).cast::<u32>().read_unaligned();
                let color3 = input.add(12).cast::<u32>().read_unaligned();
                input = input.add(16);

                let metadata = ((color0 >> 15) & 1)
                    | (((color0 >> 31) & 1) << 1)
                    | (((color1 >> 15) & 1) << 2);
                let empty_loops = *DECODE_COLOR_EMPTY_LOOPS.get_unchecked((metadata >> 1) as usize);
                let le = metadata & 1 != 0;

                out.write(empty_loops);
                out.add(1).write(if le { 14 } else { 6 });
                out.add(2).write(color0 & PIO_DATA_MASK);
                out.add(3).write(color1 & PIO_DATA_MASK);
                out.add(4).write(color2 & PIO_DATA_MASK);
                out.add(5).write(color3 & PIO_DATA_MASK);
                out = out.add(6);
                if le {
                    out.write(0);
                    out.add(1).write(0);
                    out.add(2).write(0);
                    out.add(3).write((LE_HIGH as u32) << 16);
                    out = out.add(4);
                }
                loops += 1;
            }

            bitmap >>= 1;
        }
    }

    output.write((loops - 1) as u32);
    out.offset_from(output) as usize
}
