# JTAGENB 与当前 PCB 的烧录补救方案

## 已确认结论

实板为 MachXO2-2000、TQFP144，当前构建目标为 `LCMXO2-2000HC-4TG144C`。本地
[封装迁移表](../MachXO2144-PinTQFPPackageMigrationFile.CSV) 的 **LCMXO2-2000**
列确认：120 脚 `PT20C` 的复用功能是 `JTAGENB`。
官方编程指南明确支持用它切换 JTAG 与用户 IO。

下表描述配置完成、进入 **user mode** 后的状态：

| 非易失配置 `JTAG_PORT` | 120 脚 | 四个 JTAG 管脚的功能 |
| --- | --- | --- |
| `ENABLE`（默认） | 普通用户 IO，其电平不控制 JTAG | 始终为 JTAG |
| `DISABLE` | JTAGENB = 0 | 用户 IO：B2、SEL2、B3、SEL3 |
| `DISABLE` | JTAGENB = 1 | JTAG：TDO、TDI、TCK、TMS |

**高电平启用 JTAG，低电平交给业务逻辑**，不能按名称末尾的 B 猜成低有效。
这里的 `DISABLE` 是启用由 JTAGENB 控制的复用，不是永久销毁 JTAG。
复用启用后，120 脚成为专用输入，不能再分配给普通 RTL 端口。
该功能由芯片配置硬件实现，不需要在 Verilog 中写切换逻辑。

空白器件 / Feature Row HW Default Mode 下 JTAG 默认可用。
JTAGENB 的复用控制只在 user mode 生效，不能将上表无条件套用于整个上电过程。
官方表 5.12 记载 JTAGENB 默认带弱下拉；工程上仍建议给它明确电平和可操作的入口。

## 当前板的接法

当前 [原理图](../LCMXO21200hc_SCH_Schematic3_2026-09-30.pdf) 中，120 脚没有画出
外接网络；这不等于已经检查了实际 PCB 铜皮或焊接。若实板该脚确实空闲，
可从芯片 120 脚飞线到一个跳线或测试点。

建议补救电路：120 脚经 **10 kΩ 下拉到 GND**，另设可拆跳线接 **VCCIO0**。
10 kΩ 是本方案建议值，不是官方规定值；VCCIO0 在本板设计中应为 3.3 V。
平时断开跳线，进入 user mode 后用作业务 IO；烧录时闭合跳线，切回 JTAG。
不要把 120 脚直接永久焊死到 GND，否则会失去这个切回入口。

已有 H24 排针可接下载器：

| H24 脚号 | FPGA 脚号 | 业务网络 | JTAG | 默认 Pico DirtyJTAG |
| --- | --- | --- | --- | --- |
| 2 | 137 | B2 | TDO | GP17 |
| 3 | 136 | SEL2 | TDI | GP16 |
| 6 | 131 | B3 | TCK | GP18 |
| 7 | 130 | SEL3 | TMS | GP19 |

另外连接共地，FPGA 板独立供电。

建议操作顺序：

1. 断开 MCU 对这四根业务信号的驱动，或确认其输出已进入高阻。
2. 将 JTAGENB 拉高，再连接下载器；从上电开始保持高电平即可覆盖配置前后状态。
3. 执行检测、写入和校验；写入的端口设置应为 `JTAG_PORT=DISABLE`。
4. 完成校验与退出编程后，断开下载器对共享信号的驱动。
5. 将 JTAGENB 拉低，再恢复 MCU 业务信号。

JTAGENB 只切换 FPGA 内部功能，**不会隔离外部 MCU 与下载器**。
不能让两者同时驱动共享线路；TDO 在扫描移位状态还会由 FPGA 主动输出。

## 当前工具链还需处理的部分

`JTAG_PORT` 属于非易失 Feature Row 端口配置，不是给 120 脚加一条 LPF LOC 就能实现。
官方指南第 7.1 节说明，这类配置保持到 Feature Row 被擦除。

已核对 openFPGALoader **v1.0.0** 的 `src/lattice.cpp`：
MachXO2 `.bit` Flash 路径固定使用 `featuresRow = 0`、`feabits = 0x460`；
该代码按 FEABITS bit 8 = 0 解码为 JTAG 启用。
所以现有 `openFPGALoader ... -f led.bit` 流程不会实现上述复用，
还可能把此前写入的端口设置改回默认值。
本地 nextpnr 的 LPF 解析器也未接受 `JTAG_PORT` 这个 sysCONFIG 项。

后续可以选择：

- 使用 Diamond 正确生成含端口配置的 JEDEC / 编程流程；
- 或为当前开源流程补齐经过验证的 Feature Row / FEABITS 写入和回读支持。

不能只在文档或 LPF 中写 `JTAG_PORT=DISABLE` 就当作完成。
本次仅研究和记录方案，没有改写 FPGA 的 Feature Row，也没有执行烧录。
除了 JTAG，现有 R32/INITN 等复用脚也应一起核对端口配置。

## 官方依据

- [MachXO2 Programming and Configuration User Guide，FPGA-TN-02155 v5.0](https://www.latticesemi.com/view_document?document_id=39085)：
  第 26～28 页 §5.14.4、表 5.12、图 5.9；第 42 页 §7.1.1。
  第 46 页 §7.2.8 还说明了关闭所有配置端口时的 `MUX_CONFIGURATION_PORTS` 设置，
  以及将 JTAGENB 硬接地可能导致无法再编程的后果。
- [Lattice FAQ 2117：JTAGENB 是否可以直接接 VCC 或 GND](https://www.latticesemi.com/en/Support/AnswerDatabase/2/1/1/2117)。
- [官方 TQFP144 封装迁移表（本地副本）](../MachXO2144-PinTQFPPackageMigrationFile.CSV)：
  查 `LCMXO2-2000` 组的 `Pin Number` 与 `Dual Function` 列。
- [openFPGALoader v1.0.0 lattice.cpp](https://github.com/trabucayre/openFPGALoader/blob/v1.0.0/src/lattice.cpp)：
  `program_intFlash()` 与 `displayFeabits()`。
