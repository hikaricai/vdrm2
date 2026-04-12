#![no_std]
use bitfield_struct::bitfield;
type MbiWave = u8;
type Reg = u16;

pub type RGBH = [u8; 4];
#[derive(Clone, Copy)]
pub enum CmdParam {
    Comm(u16),
    RGB((u16, u16, u16)),
}

impl From<u16> for CmdParam {
    fn from(value: u16) -> Self {
        Self::Comm(value)
    }
}

impl From<(u16, u16, u16)> for CmdParam {
    fn from(value: (u16, u16, u16)) -> Self {
        Self::RGB(value)
    }
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct AngleImage {
    pub angle: u32,
    // rgbh
    pub coloum: [RGBH; IMG_HEIGHT],
}

impl AngleImage {
    pub const fn new(angle: u32) -> Self {
        Self {
            angle,
            coloum: [[0; 4]; IMG_HEIGHT],
        }
    }
}

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
    WriteCfg1_4 = 4,
    WriteCfg7_6 = 6,
    ErrDetect = 7,
    WriteCfg2_8 = 8,
    ResetSoft = 10,     // A
    EnableAll = 11,     // B
    DisableAll = 12,    // C
    WriteCfg5_0D = 13,  // D
    Confirm = 14,       // E
    WriteCfg6_0F = 15,  // F
    WriteCfg3_10 = 16,  // 10
    WriteCfg4_12 = 18,  // 12
    WriteCfg8_17 = 23,  // 17
    WriteCfg9_18 = 24,  // 18
    WriteCfg10_19 = 25, // 19
    WriteCfg11_1A = 26, // 1A
    WriteCfg12_1B = 27, // 1B
    WriteCfg13_1C = 28, // 1C
    WriteCfg14_1D = 29, // 1D
    WriteCfg15_1E = 30, // 1E
    WriteCfg16_1F = 31, // 1F
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
    #[bits(6, default = 0b11_1111)]
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
        // let exp = 0x9F3F;
        // let exp2 = 0b1001_1111_0011_1111;
        // assert_eq_const(exp, exp2, concat!("line", line!()));
        // let v = Self::new()
        //     .with_current_gain(0x3F)
        //     .with_scan_lines_low(31)
        //     .0;
        // assert_eq_const(v, exp, concat!("line", line!()));
        // v

        // spwm mode 0b11 512 gclks in 16 loops
        let exp = 0x9FBF;
        let exp2 = 0b1001_1111_1011_1111;
        assert_eq_const(exp, exp2, concat!("line", line!()));
        let v = Self::new()
            .with_current_gain(0x3F)
            .with_spwm_mode_low(true)
            .with_scan_lines_low(31)
            .0;
        assert_eq_const(v, exp, concat!("line", line!()));
        v
    }

    const fn umini_rgbs() -> (u16, u16, u16) {
        // (0x9FBA, 0x9FBC, 0x9FBF)
        let v = Self::new()
            .with_current_gain(0x3F)
            .with_spwm_mode_low(true)
            .with_scan_lines_low(31)
            .0;
        (v, v, v)
    }
}

#[bitfield(u16)]
struct RegCfg4 {
    #[bits(5, default = 0x1F)]
    dimm_compensation3: u8,
    #[bits(5, default = 0x1F)]
    dimm_compensation1: u8,
    #[bits(5, default = 0x1F)]
    dimm_compensation_first_line: u8,
    #[bits(default = true)]
    _reserved1: bool,
}

impl RegCfg4 {
    const fn umini_default() -> u16 {
        let exp = 0xAC1F;
        let exp2 = 0b1010_1100_0001_1111;
        assert_eq_const(exp, exp2, concat!("line", line!()));
        let v = Self::new()
            .with_dimm_compensation1(0x00)
            .with_dimm_compensation_first_line(0x0B)
            .0;
        // let v = Self::new().0; //only show red
        // let exp = 0xffff;
        assert_eq_const(v, exp, concat!("line", line!()));
        v
    }
    const fn umini_rgbs() -> (u16, u16, u16) {
        // (0xCC1F, 0xAC1F, 0x9C1F)
        let v = Self::new()
            .with_dimm_compensation1(31)
            .with_dimm_compensation3(31)
            .0;
        let v = Self::new()
            .with_dimm_compensation1(0x00)
            .with_dimm_compensation_first_line(0x0B)
            .0;
        (v, v, v)
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
    const fn umini_rgbs() -> (u16, u16, u16) {
        (0x8A04, 0x8C04, 0x8E04)
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
        // v
        // 这个有效果 显著改善亮度
        v
    }

    const fn umini_rgbs() -> (u16, u16, u16) {
        let v = Self::new()
            .with_dark_compensation3(0)
            .with_dark_compensation2(0)
            // .with_dark_compensation1(0x10)
            .0;
        let vb = Self::new()
            .with_dark_compensation3(0x19)
            .with_dark_compensation2(0x17)
            // .with_dark_compensation1(0x10)
            .0;

        (v, v, vb)
    }
}

#[bitfield(u16)]
struct RegCfg2 {
    #[bits(10, default = 0x201)]
    _reserved0: u16,
    #[bits(default = true)]
    double_fresh: bool,
    #[bits(3)]
    color_compensation2: u8,
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
struct RegCfg8 {
    #[bits(5)]
    gclk_change: u8,
    #[bits(7)]
    _reserved0: u8,
    #[bits(1)]
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
        // let exp2 = 0b0000_0100_0100_1111;
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
        // let exp = 0x4F;
        // let exp2 = 0b0000_0000_0100_1111;
        // assert_eq_const(exp, exp2, concat!("line", line!()));
        let v = Self::new().with_dummy_gclk_period(0x4F).0;
        // assert_eq_const(v, exp, concat!("line", line!()));
        // v
        Self::new()
            .with_dummy_gclk_period(0x1)
            .with_color_compensation(63)
            .0
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
        // gclk = x / div
        // gclk = x / n
        // gclk = x * m

        // clk = 9m
        // gclk_freq >= 125k * (spwm_mod 0b11) 512 == 64m
        // ? pll_multi == (79 + 1) / (5) / 1 = 16
        // ? gclk == 9m * 16 > 64m
        // dsview is A797
        // let exp = 0xA797;
        // let exp2 = 0b1010_0111_1001_0111;
        // assert_eq_const(exp, exp2, concat!("line", line!()));

        // let exp = 0xA797;
        // let exp2 = 0b1010_0111_1001_0111;
        // assert_eq_const(exp, exp2, concat!("line", line!()));
        // pll_m from 19 to 39 to 79
        let v = Self::new().with_pll_n(5).with_pll_m(86).0;
        // assert_eq_const(v, exp, concat!("line", line!()));
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

pub fn unimi_cmds() -> [(CMD, CmdParam); 16] {
    [
        // (CMD::Unknown3, Unknown3::umini_default()),
        (CMD::WriteCfg1_4, RegCfg1::umini_rgbs().into()),
        (CMD::WriteCfg7_6, RegCfg7::umini_rgbs().into()),
        (CMD::WriteCfg4_12, RegCfg4::umini_rgbs().into()),
        (CMD::WriteCfg3_10, RegCfg3::umini_rgbs().into()),
        (CMD::WriteCfg2_8, RegCfg2::umini_default().into()),
        (CMD::WriteCfg5_0D, RegCfg5::umini_default().into()),
        (CMD::WriteCfg6_0F, RegCfg6::umini_default().into()),
        (CMD::WriteCfg8_17, RegCfg8::umini_default().into()),
        (CMD::WriteCfg9_18, RegCfg9::umini_default().into()),
        (CMD::WriteCfg10_19, RegCfg10::umini_default().into()),
        (CMD::WriteCfg11_1A, RegCfg11::umini_default().into()),
        (CMD::WriteCfg12_1B, RegCfg12::umini_default().into()),
        (CMD::WriteCfg13_1C, RegCfg13::umini_default().into()),
        (CMD::WriteCfg14_1D, RegCfg14::umini_default().into()),
        (CMD::WriteCfg15_1E, RegCfg15::umini_default().into()),
        (CMD::WriteCfg16_1F, RegCfg16::umini_default().into()),
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
