pub const FALSH_ADDR: usize = 0x10000000;
// 0x10100000
pub const EXT_ADDR: usize = FALSH_ADDR + 1024 * 1024;
pub const IMAGE_LEN_ADDR: usize = EXT_ADDR + core::mem::size_of::<u32>();
pub const IMAGE_ADDR: usize = IMAGE_LEN_ADDR + core::mem::size_of::<u32>();
