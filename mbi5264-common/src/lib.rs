#![no_std]
use bitfield_struct::bitfield;
type MbiWave = u8;
type Reg = u16;

const fn assert_eq_const(a: u16, b: u16, msg: &'static str) {
    if a != b {
        panic!("{}", msg);
    }
}

#[derive(Clone, Copy)]
#[repr(u8)]
pub enum CMD {
    Latch = 1,
    VSync = 2,
    Unknown3 = 3,
    WriteCfg1 = 4,
    WriteCfg7 = 6,
    ErrDetect = 7,
    WriteCfg2 = 8,
    ResetSoft = 10,  // A
    EnableAll = 11,  // B
    DisableAll = 12, // C
    WriteCfg5 = 13,  // D
    Confirm = 14,    // E
    WriteCfg6 = 15,  // F
    WriteCfg3 = 16,  // 10
    WriteCfg4 = 18,  // 12
    WriteCfg8 = 23,  // 17
    WriteCfg9 = 24,  // 18
    WriteCfg10 = 25, // 19
    WriteCfg11 = 26, // 1A
    WriteCfg12 = 27, // 1B
    WriteCfg13 = 28, // 1C
    WriteCfg14 = 29, // 1D
    WriteCfg15 = 30, // 1E
    WriteCfg16 = 31, // 1F
}

pub struct Unknown3(u16);
impl Unknown3 {
    const fn umini_default() -> u16 {
        // 也可以是4a78
        0x4A7C;
        0x4A78
    }
}

pub struct ErrDetect(u16);
impl ErrDetect {
    const fn umini_default() -> u16 {
        0
    }
}

#[bitfield(u16)]
struct RegCfg1 {
    #[bits(6, default = 0b111_111)]
    current_gain: u8,
    #[bits(1)]
    _reserved0: u8,
    spwm_mode_low: bool,
    #[bits(5)]
    scan_lines_low: u8,
    _cross_rm: bool,
    _strip_rm: bool,
    #[bits(default = true)]
    _reserved1: bool,
}

impl RegCfg1 {
    const fn umini_default() -> u16 {
        let exp = 0x9FA5;
        let exp2 = 0b1001_1111_1010_0101;
        let exp = 0x9F2D;
        let exp2 = 0b1001_1111_0010_1101;
        assert_eq_const(exp, exp2, concat!("line", line!()));
        let v = Self::new()
            .with_current_gain(0x2D)
            .with_scan_lines_low(31)
            .0;
        assert_eq_const(v, exp, concat!("line", line!()));
        v
    }
}

#[bitfield(u16)]
struct RegCfg2 {
    #[bits(10, default = 0x201)]
    _reserved0: u16,
    #[bits(default = true)]
    double_fresh: bool,
    #[bits(3)]
    color_compensation: u8,
    spwm_mode_high: bool,
    #[bits(default = true)]
    _reserved1: bool,
}

impl RegCfg2 {
    const fn umini_default() -> u16 {
        let exp = 0xC601;
        let exp2 = 0b1100_0110_0000_0001;
        assert_eq_const(exp, exp2, concat!("line", line!()));
        let v = Self::new().with_spwm_mode_high(true).0;
        assert_eq_const(v, exp, concat!("line", line!()));
        v
    }
}

#[bitfield(u16)]
struct RegCfg3 {
    #[bits(5, default = 0x1F)]
    dark_compensation3: u8,
    #[bits(5, default = 0x1F)]
    dark_compensation2: u8,
    #[bits(5, default = 0x1F)]
    dark_compensation1: u8,
    #[bits(default = true)]
    _reserved1: bool,
}

impl RegCfg3 {
    const fn umini_default() -> u16 {
        let exp = 0xFEF9;
        let exp2 = 0b1111_1110_1111_1001;
        assert_eq_const(exp, exp2, concat!("line", line!()));
        let v = Self::new()
            .with_dark_compensation3(0x19)
            .with_dark_compensation2(0x17)
            .0;
        assert_eq_const(v, exp, concat!("line", line!()));
        v
    }
}

#[bitfield(u16)]
struct RegCfg4 {
    #[bits(5, default = 0x1F)]
    dimm_compensation3: u8,
    #[bits(5, default = 0x1F)]
    dimm_compensation2: u8,
    #[bits(5, default = 0x1F)]
    dimm_compensation1: u8,
    #[bits(default = true)]
    _reserved1: bool,
}

impl RegCfg4 {
    const fn umini_default() -> u16 {
        let exp = 0xCC1F;
        let exp2 = 0b1100_1100_0001_1111;

        let exp = 0xAC1F;
        let exp2 = 0b1010_1100_0001_1111;
        assert_eq_const(exp, exp2, concat!("line", line!()));
        let v = Self::new()
            .with_dimm_compensation2(0x00)
            .with_dimm_compensation1(0x0B)
            .0;
        assert_eq_const(v, exp, concat!("line", line!()));
        v
    }
}

#[bitfield(u16)]
struct RegCfg5 {
    #[bits(5, default = 0x1A)]
    de_ghosting: u8,
    #[bits(3)]
    cross_removal: u8,
    #[bits(8, default = 0b1110010)]
    _reserved0: u8,
}

impl RegCfg5 {
    const fn umini_default() -> u16 {
        let exp = 0x721F;
        let exp2 = 0b0111_0010_0001_1111;
        assert_eq_const(exp, exp2, concat!("line", line!()));
        let v = Self::new().with_de_ghosting(0x1F).0;
        assert_eq_const(v, exp, concat!("line", line!()));
        v
    }
}

struct RegCfg6(u16);

impl RegCfg6 {
    const fn umini_default() -> u16 {
        0x4000
    }
}

#[bitfield(u16)]
struct RegCfg7 {
    #[bits(2)]
    _reserved0: u8,
    #[bits(default = true)]
    scan_lines_high: bool,
    #[bits(3)]
    _reserved1: u8,
    #[bits()]
    reset: bool,
    #[bits(9, default = 0x100)]
    reserved2: u16,
}

impl RegCfg7 {
    const fn umini_default() -> u16 {
        let exp = 0x8A04;
        let exp2 = 0b1000_1010_0000_0100;
        let exp = 0x8C04;
        let exp2 = 0b1000_1100_0000_0100;

        assert_eq_const(exp, exp2, concat!("line", line!()));
        let v = Self::new().with_reserved2(0x118).0;
        assert_eq_const(v, exp, concat!("line", line!()));
        v
    }
}

#[bitfield(u16)]
struct RegCfg8 {
    #[bits(5)]
    gclk_change: u8,
    #[bits(7)]
    _reserved0: u8,
    #[bits()]
    gradient_compensation: bool,
    #[bits(3)]
    reserved1: u16,
}

impl RegCfg8 {
    const fn umini_default() -> u16 {
        let exp = 0x9001;
        let exp2 = 0b1001_0000_0000_0001;
        assert_eq_const(exp, exp2, concat!("line", line!()));
        let v = Self::new()
            .with_gclk_change(1)
            .with_gradient_compensation(true)
            .with_reserved1(4)
            .0;
        assert_eq_const(v, exp, concat!("line", line!()));
        v
    }
}

struct RegCfg9(u16);

impl RegCfg9 {
    const fn umini_default() -> u16 {
        // datasheet default is 0x121
        // dsveiw is 0x643 or 0x 3A1
        // 0b0000_0011_1010_0001
        0x321
    }
}

struct RegCfg10(u16);

impl RegCfg10 {
    const fn umini_default() -> u16 {
        0x00
    }
}

#[bitfield(u16)]
struct RegCfg11 {
    #[bits(10, default = 215)]
    gclk_change: u16,
    #[bits(6, default = 1)]
    _reserved0: u8,
}

impl RegCfg11 {
    const fn umini_default() -> u16 {
        // dsview is 844F or 89F or 44F
        let exp = 0x044F;
        let exp2 = 0b0000_0100_0100_1111;
        assert_eq_const(exp, exp2, concat!("line", line!()));
        let v = Self::new().with_gclk_change(0x4F).0;
        assert_eq_const(v, exp, concat!("line", line!()));
        v
    }
}

#[bitfield(u16)]
struct RegCfg12 {
    #[bits(10, default = 133)]
    dummy_gclk_period: u16,
    #[bits(6)]
    color_compensation: u8,
}

impl RegCfg12 {
    const fn umini_default() -> u16 {
        let exp = 0x4F;
        let exp2 = 0b0000_0000_0100_1111;
        assert_eq_const(exp, exp2, concat!("line", line!()));
        let v = Self::new().with_dummy_gclk_period(0x4F).0;
        assert_eq_const(v, exp, concat!("line", line!()));
        v
    }
}

#[bitfield(u16)]
struct RegCfg13 {
    #[bits(2, default = 0b11)]
    pll_div: u8,
    #[bits(5, default = 0b00110)]
    pll_n: u8,
    #[bits(7, default = 0b1100100)]
    pll_m: u8,
    #[bits(2, default = 2)]
    _reserve0: u8,
}

impl RegCfg13 {
    const fn umini_default() -> u16 {
        // clk = 10m
        // gclk_freq >= 125k * (spwm_mod 0b11) 512 == 64m
        // ? pll_multi == (79 + 1) / (7 + 1) = 10
        // ? gclk == 9m * 10 > 64m
        // dsview is A79F or A797
        // let exp = 0xA79F;
        // let exp2 = 0b1010_0111_1001_1111;
        // assert_eq_const(exp, exp2, concat!("line", line!()));

        let exp = 0xA797;
        let exp2 = 0b1010_0111_1001_0111;
        assert_eq_const(exp, exp2, concat!("line", line!()));
        let v = Self::new().with_pll_n(5).with_pll_m(79).0;
        assert_eq_const(v, exp, concat!("line", line!()));
        v
    }
}

struct RegCfg14(u16);

impl RegCfg14 {
    const fn umini_default() -> u16 {
        0x49
    }
}

struct RegCfg15(u16);

impl RegCfg15 {
    const fn umini_default() -> u16 {
        0x00
    }
}

struct RegCfg16(u16);

impl RegCfg16 {
    const fn umini_default() -> u16 {
        0x00
    }
}

pub const fn unimi_cmds() -> [(CMD, u16); 17] {
    [
        (CMD::Unknown3, Unknown3::umini_default()),
        (CMD::WriteCfg1, RegCfg1::umini_default()),
        (CMD::WriteCfg7, RegCfg7::umini_default()),
        (CMD::WriteCfg2, RegCfg2::umini_default()),
        (CMD::WriteCfg5, RegCfg5::umini_default()),
        (CMD::WriteCfg6, RegCfg6::umini_default()),
        (CMD::WriteCfg3, RegCfg3::umini_default()),
        (CMD::WriteCfg4, RegCfg4::umini_default()),
        (CMD::WriteCfg8, RegCfg8::umini_default()),
        (CMD::WriteCfg9, RegCfg9::umini_default()),
        (CMD::WriteCfg10, RegCfg10::umini_default()),
        (CMD::WriteCfg11, RegCfg11::umini_default()),
        (CMD::WriteCfg12, RegCfg12::umini_default()),
        (CMD::WriteCfg13, RegCfg13::umini_default()),
        (CMD::WriteCfg14, RegCfg14::umini_default()),
        (CMD::WriteCfg15, RegCfg15::umini_default()),
        (CMD::WriteCfg16, RegCfg16::umini_default()),
    ]
    // return vec![(CMD::EnableAll, 0)];
}

#[derive(Clone, Copy)]
#[repr(u32)]
pub enum UsbCmd {
    QOI = 1,
}

impl UsbCmd {
    pub fn from_u8(v: u8) -> Option<Self> {
        match v {
            1 => Some(UsbCmd::QOI),
            _ => None,
        }
    }
}

#[derive(Clone, Copy)]
#[repr(packed, C)]
pub struct UsbDataHead {
    pub cmd: UsbCmd,
    pub payload_len: u32,
}

impl UsbDataHead {
    pub fn to_buf(&self) -> &[u8] {
        unsafe {
            core::slice::from_raw_parts(
                self as *const Self as *const u8,
                core::mem::size_of::<UsbDataHead>(),
            )
        }
    }
}

pub const IMG_HEIGHT: usize = 192;
pub const IMG_WIDTH: usize = IMG_HEIGHT / 4;
pub const QOI_BUF_SIZE: usize = IMG_HEIGHT * IMG_WIDTH;

#[repr(C)]
pub struct UsbData<'a> {
    pub hdr: &'a UsbDataHead,
    pub payload: &'a [u8],
}

impl<'a> UsbData<'a> {
    pub fn ref_from_buf(buf: &'a [u8]) -> Option<Self> {
        let hdr_len = core::mem::size_of::<UsbDataHead>();
        if buf.len() < hdr_len {
            return None;
        }
        let payload = &buf[hdr_len..];
        let hdr: &UsbDataHead = unsafe { core::mem::transmute(buf.as_ptr()) };
        if hdr.payload_len > payload.len() as u32 {
            return None;
        }
        let payload = &payload[0..hdr.payload_len as usize];
        Some(Self { hdr, payload })
    }
}
