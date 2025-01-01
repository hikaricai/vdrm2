use bitfield_struct::bitfield;
type MbiWave = u8;
type Reg = u16;

trait UminiDefault {
    fn umini_default() -> u16;
}

#[derive(Clone, Copy)]
#[repr(u8)]
pub enum CMD {
    Latch = 1,
    VSync = 2,
    Unknown3 = 3,
    WriteCfg1 = 4,
    WriteCfg7 = 6,
    WriteCfg2 = 8,
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
impl UminiDefault for Unknown3 {
    fn umini_default() -> u16 {
        0x4A7C
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

impl UminiDefault for RegCfg1 {
    fn umini_default() -> u16 {
        let exp = 0x9FA5;
        let exp2 = 0b1001_1111_1010_0101;
        assert_eq!(exp, exp2);
        let v = Self::new()
            .with_current_gain(0x25)
            .with_spwm_mode_low(true)
            .with_scan_lines_low(31)
            .0;
        assert_eq!(v, exp);
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

impl UminiDefault for RegCfg2 {
    fn umini_default() -> u16 {
        let exp = 0xC601;
        let exp2 = 0b1100_0110_0000_0001;
        assert_eq!(exp, exp2);
        let v = Self::new().with_spwm_mode_high(true).0;
        assert_eq!(v, exp);
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

impl UminiDefault for RegCfg3 {
    fn umini_default() -> u16 {
        let exp = 0xFEF9;
        let exp2 = 0b1111_1110_1111_1001;
        assert_eq!(exp, exp2);
        let v = Self::new()
            .with_dark_compensation3(0x19)
            .with_dark_compensation2(0x17)
            .0;
        assert_eq!(v, exp);
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

impl UminiDefault for RegCfg4 {
    fn umini_default() -> u16 {
        let exp = 0xCC1F;
        let exp2 = 0b1100_1100_0001_1111;
        assert_eq!(exp, exp2);
        let v = Self::new()
            .with_dimm_compensation2(0x00)
            .with_dimm_compensation1(0x13)
            .0;
        assert_eq!(v, exp);
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

impl UminiDefault for RegCfg5 {
    fn umini_default() -> u16 {
        let exp = 0x721F;
        let exp2 = 0b0111_0010_0001_1111;
        assert_eq!(exp, exp2);
        let v = Self::new().with_de_ghosting(0x1F).0;
        assert_eq!(v, exp);
        v
    }
}

struct RegCfg6(u16);

impl UminiDefault for RegCfg6 {
    fn umini_default() -> u16 {
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

impl UminiDefault for RegCfg7 {
    fn umini_default() -> u16 {
        let exp = 0x8A04;
        let exp2 = 0b1000_1010_0000_0100;
        assert_eq!(exp, exp2);
        let v = Self::new().with_reserved2(0x114).0;
        assert_eq!(v, exp);
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

impl UminiDefault for RegCfg8 {
    fn umini_default() -> u16 {
        let exp = 0x9001;
        let exp2 = 0b1001_0000_0000_0001;
        assert_eq!(exp, exp2);
        let v = Self::new()
            .with_gclk_change(1)
            .with_gradient_compensation(true)
            .with_reserved1(4)
            .0;
        assert_eq!(v, exp);
        v
    }
}

struct RegCfg9(u16);

impl UminiDefault for RegCfg9 {
    fn umini_default() -> u16 {
        // datasheet default is 0x121
        // dsveiw is 0x643 or 0x 3A1
        // 0b0000_0011_1010_0001
        0x3A1
    }
}

struct RegCfg10(u16);

impl UminiDefault for RegCfg10 {
    fn umini_default() -> u16 {
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

impl UminiDefault for RegCfg11 {
    fn umini_default() -> u16 {
        // dsview is 844F or 89F or 44F
        let exp = 0x044F;
        let exp2 = 0b0000_0100_0100_1111;
        assert_eq!(exp, exp2);
        let v = Self::new().with_gclk_change(0x4F).0;
        assert_eq!(v, exp);
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

impl UminiDefault for RegCfg12 {
    fn umini_default() -> u16 {
        let exp = 0x4F;
        let exp2 = 0b0000_0000_0100_1111;
        assert_eq!(exp, exp2);
        let v = Self::new().with_dummy_gclk_period(0x4F).0;
        assert_eq!(v, exp);
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

impl UminiDefault for RegCfg13 {
    fn umini_default() -> u16 {
        // clk = 10m
        // gclk_freq >= 125k * (spwm_mod 0b11) 512 == 64m
        // ? pll_multi == (79 + 1) / (7 + 1) = 10
        // ? gclk == 9m * 10 > 64m
        // dsview is A79F or A797
        let exp = 0xA79F;
        let exp2 = 0b1010_0111_1001_1111;
        assert_eq!(exp, exp2);
        let v = Self::new().with_pll_n(7).with_pll_m(79).0;
        assert_eq!(v, exp);
        v
    }
}

struct RegCfg14(u16);

impl UminiDefault for RegCfg14 {
    fn umini_default() -> u16 {
        0x49
    }
}

struct RegCfg15(u16);

impl UminiDefault for RegCfg15 {
    fn umini_default() -> u16 {
        0x00
    }
}

struct RegCfg16(u16);

impl UminiDefault for RegCfg16 {
    fn umini_default() -> u16 {
        0x00
    }
}

pub fn unimi_cmds() -> Vec<(CMD, u16)> {
    vec![
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
}

// 一个transaction完成串联的全部芯片的一次命令写入
// 只控制le和data
#[repr(packed)]
pub struct Command {
    // 命令
    pub cmd: u8,
    // 9个串联芯片
    pub regs: [[u16; 3]; 9],
}

impl Command {
    pub fn new_cmd(cmd: CMD, reg: u16) -> Self {
        Self {
            regs: [[reg; 3]; 9],
            cmd: cmd as u8,
        }
    }
    pub fn new_rgb(regs: [[u16; 3]; 9]) -> Self {
        Self { regs, cmd: 1 }
    }

    pub fn to_buf(&self) -> &[u8; 55] {
        assert_eq!(std::mem::size_of::<Self>(), 55);
        let ptr = self as *const Self as *const u8;
        unsafe { &*(ptr as *const [u8; 55]) }
    }
}
