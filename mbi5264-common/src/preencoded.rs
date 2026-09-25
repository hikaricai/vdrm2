//! Little-endian, independently compressed PIO frame format.
//!
//! Header: magic, init angle, frame count, maximum decoded frame words.
//! Frame table: angle, payload offset, compressed bytes, decoded DMA words.

pub const MAGIC: [u8; 4] = *b"PIO1";
pub const HEADER_SIZE: usize = 16;
pub const FRAME_ENTRY_SIZE: usize = 16;
pub const MAX_CACHED_IMAGE_BYTES: usize = 192 * 1024;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Header {
    pub init_angle: u32,
    pub frame_count: u32,
    pub max_frame_words: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FrameEntry {
    pub angle: u32,
    pub data_offset: u32,
    pub compressed_len: u32,
    pub dma_words: u32,
}

#[inline]
fn read_u32(data: &[u8], offset: usize) -> Option<u32> {
    Some(u32::from_le_bytes(
        data.get(offset..offset + 4)?.try_into().ok()?,
    ))
}

impl Header {
    pub fn parse(data: &[u8]) -> Option<Self> {
        if data.get(..4)? != MAGIC {
            return None;
        }
        Some(Self {
            init_angle: read_u32(data, 4)?,
            frame_count: read_u32(data, 8)?,
            max_frame_words: read_u32(data, 12)?,
        })
    }

    pub fn to_bytes(self) -> [u8; HEADER_SIZE] {
        let mut output = [0; HEADER_SIZE];
        output[..4].copy_from_slice(&MAGIC);
        output[4..8].copy_from_slice(&self.init_angle.to_le_bytes());
        output[8..12].copy_from_slice(&self.frame_count.to_le_bytes());
        output[12..16].copy_from_slice(&self.max_frame_words.to_le_bytes());
        output
    }
}

impl FrameEntry {
    pub fn parse(data: &[u8], index: usize) -> Option<Self> {
        let offset = HEADER_SIZE.checked_add(index.checked_mul(FRAME_ENTRY_SIZE)?)?;
        Some(Self {
            angle: read_u32(data, offset)?,
            data_offset: read_u32(data, offset + 4)?,
            compressed_len: read_u32(data, offset + 8)?,
            dma_words: read_u32(data, offset + 12)?,
        })
    }

    pub fn to_bytes(self) -> [u8; FRAME_ENTRY_SIZE] {
        let mut output = [0; FRAME_ENTRY_SIZE];
        output[..4].copy_from_slice(&self.angle.to_le_bytes());
        output[4..8].copy_from_slice(&self.data_offset.to_le_bytes());
        output[8..12].copy_from_slice(&self.compressed_len.to_le_bytes());
        output[12..16].copy_from_slice(&self.dma_words.to_le_bytes());
        output
    }
}
