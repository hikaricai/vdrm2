type MbiWave = u8;
type Reg = u16;

// 一个transaction完成串联的全部芯片的一次命令写入
// 只控制le和data
#[repr(packed)]
pub struct Transaction {
    // 命令
    pub cmd: u8,
    // 9个串联芯片
    pub regs: [[u16; 3]; 9],
}

impl Transaction {
    pub fn new_cmd(cmd: u8, reg: u16) -> Self {
        Self {
            regs: [[reg; 3]; 9],
            cmd,
        }
    }
    pub fn new_rgb(regs: [[u16; 3]; 9]) -> Self {
        Self { regs, cmd: 1 }
    }

    pub fn to_buf(&self) -> &[u8; 55] {
        assert_eq!(std::mem::size_of::<Self>(), 55);
        unsafe { std::mem::transmute(&self) }
    }
}
