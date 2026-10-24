# ICND2069 开源驱动项目调查

调查日期：2026-10-23。目标是寻找能够由 MCU/FPGA 直接产生 `SIN/DCLK/LE/ROW` 波形、初始化 ICND2069 并刷新 LED 模组的开源代码，而不是只在产品说明或接收卡支持列表中出现 `ICND2069` 名称的项目。

## 结论

**截至本次检索，没有找到经过验证、明确点名支持 ICND2069 的开源底层驱动。** GitHub 仓库名搜索、GitLab/Gitee 项目搜索、Sourcegraph 代码搜索，以及普通网页搜索均未发现完整实现。这个结论表示“公开索引范围内未找到”，不等于能证明私有仓库或未被索引的代码不存在。

最接近且确实包含底层波形代码的项目是 [`board707/DMD_STM32`](https://github.com/board707/DMD_STM32)。它实现的是 **ICND2055**，不是 ICND2069；代码包括寄存器解锁、配置写入、16 bit 灰度装载、时钟包和换行控制。[cite:1][cite:2] ICND2055 与 ICND2069 使用同一份集创编程指南中的命令框架，因此它适合作为移植骨架，但不能不改代码直接驱动当前 `ICND2069AP + RT5990` 共阴板。[cite:3]

主要障碍不是 ICND2069 命令完全未知，而是：

- ICND2055 是共阳恒流驱动，官方产品页标称最高 1/32 扫；ICND2069 是共阴恒流源驱动，最高 1/64 扫，电流方向和应用拓扑不同。[cite:4][cite:5]
- ICND2069 的寄存器值必须匹配板子的扫描数、时钟、RGB 电流、消隐和鬼影补偿，不能照抄 ICND2055 表。
- 板上的 RT5990 行管协议和引脚尚无公开完整手册。即使 ICND2069 灰度数据正确，RT5990 没有同步选行也不能正常显示。
- 当前模组接口是否把 `SIN/DCLK/LE/ROW` 和 RT5990 控制信号直接引出，仍需追线确认。

## 检索结果分类

| 项目或资料 | 是否开源 | 是否直接支持 ICND2069 | 实际价值 |
|---|---:|---:|---|
| `board707/DMD_STM32` | 是，GPL-3.0 | 否，只明确支持 ICND2055 等 | 最接近的 MCU 底层波形和刷新实现 |
| `sarakusha/novastar` | 是，MIT | 未发现 ICND2069 实现 | NovaStar 设备协议和部分芯片配置结构，不是 GPIO 直驱 |
| 集创 2055/2065/2059/2069 编程指南 | 文档可下载，不是软件项目 | 是 | 给出 ICND2069 命令、寄存器写法、灰度顺序和 ROW 时序 |
| NovaStar/COEX 固件 | 否 | 支持 ICND2069 | 可点亮商用模组，但固件和芯片驱动闭源 |
| 普通 HUB75 库 | 多数开源 | 否 | 只适合传统移位寄存器面板，不能直接驱动带 SRAM 的 ICND2069 |

## DMD_STM32

### 项目状态

项目主页明确称其为实验平台，并在 SPWM 驱动列表中列出 `ICND2055`，未列出 `ICND2069`。[cite:1] 本次核对的 `dev-V2` 分支版本为 `1.2.18`，提交：

```text
398ec1d8636bce866709fbe9c3f0035c90645e7a
```

项目许可证是 GPL-3.0。若复制或改写其代码并对外分发，需要按 GPL-3.0 履行相应源代码和许可证义务。[cite:6]

支持平台包括：

- RP2040 Arduino Core；
- STM32F1、STM32F4 的 Roger Clark Arduino Core；
- README 称 RP2350 “应该可以工作，但未充分测试”；
- STM32duino 路径目前不支持 SPWM 驱动。

### 已实现的底层功能

核心文件 [`DMD_SPWM_Driver.h`](https://github.com/board707/DMD_STM32/blob/dev-V2/DMD_SPWM_Driver.h) 中存在 `DMD_RGB_ICN2055` 类和公共 SPWM 基类。[cite:2] 代码不是型号枚举占位，而是实际实现了：

- GPIO 快速写入 `DATA/CLK/LAT`；
- `LE` 保持指定 DCLK 个数的命令编码；
- `VSYNC`；
- `PRE_ACT`；
- `0x00AA/0x01AA` 解锁和 `0x0055/0x0155` 结束序列；
- 逐项配置寄存器写入；
- 每像素 16 bit 灰度数据装载；
- 定时器生成内部 PWM 所需时钟；
- 按行切换和双缓冲。

这些结构与集创联合编程指南公开的 ICND2069 高层协议明显同源：

| LE 高电平包含的 DCLK 上升沿 | 官方命令 |
|---:|---|
| 1 | DATA_LATCH |
| 3 | VSYNC |
| 5 | WR_CFG |
| 7 | RD_CFG |
| 9 | SOFT_RST |
| 11 | EN_OP |
| 12 | DIS_OP |
| 14 | PRE_ACT |

因此，DMD_STM32 的“命令发送器 + 灰度数据装载器”有较高复用价值。[cite:2][cite:3]

源码注释不能全部照信：公共基类中有注释把 12 个 LE 时钟写成“enable all output”，DP3264 基类又有注释把 11 个时钟写成“pre-active”；集创官方定义实际是 `11=EN_OP`、`12=DIS_OP`、`14=PRE_ACT`。ICND2055 路径的实际序列包含 11 和 14，但移植时应依据脉冲数和官方指南核对行为，不应依据这两处注释重命名命令。[cite:2][cite:3]

### P0.93 配置样本

仓库包含一个值得注意的文件：

[`register_test/profiles/icnd2055/icnd2055_scan_46.h`](https://github.com/board707/DMD_STM32/blob/dev-V2/register_test/profiles/icnd2055/icnd2055_scan_46.h)

其中有两组注释为：

```text
P0.93 - ICND2055 - 5958 - 1/46
```

这与当前 P0.93 板在点间距上接近，但芯片组合不同：

| 项目 | 开源样本 | 当前板 |
|---|---|---|
| 恒流驱动 | ICND2055 | ICND2069AP |
| 行管 | 5958 | RT5990 |
| 扫描 | 标注 1/46 | 未确认 |
| 阳极/阴极架构 | ICND2055 为共阳 | ICND2069 为共阴 |

该样本可用于理解不同 RGB 通道如何保存独立寄存器表，但**不能作为当前板的初始化值**。另外，官方 ICND2055 产品页标称最高 1/32 扫，而样本标注 1/46，二者存在表面冲突；在没有该样本来源和实物验证说明的情况下，应把它视为实验/导入数据，而不是经官方确认的工作配置。[cite:4][cite:7]

### 不能直接使用的原因

1. `DMD_RGB_ICN2055` 类中的默认寄存器数组是 ICND2055 配置。
2. 其行选择抽象主要面向常见 HUB75 `ABCDE/MUX` 或项目已知的行管，未实现 RT5990。
3. ICND2069 编程指南要求 `ROW` 与外部行管选择同步；当前板上的同步路径还未追清。
4. DMD_STM32 只输出 1 bit 或 4 bit 色彩缓冲，再扩展进芯片的高灰度字。它适合验证点亮，不等于商用控制卡的 14/16 bit 画质。
5. 代码本身标记为实验性，时钟参数和初始化序列仍需示波器验证。

## novastar.js

[`sarakusha/novastar`](https://github.com/sarakusha/novastar) 是一个通过逆向实现 NovaStar 设备 API 的 TypeScript 项目，许可证为 MIT。[cite:8] 它主要处理：

- 上位机到 NovaStar 发送卡、接收卡的串口或网络协议；
- 屏体配置数据结构；
- 从反编译 NovaStar .NET 库生成的类型；
- Wireshark 协议解析。

本次在固定提交 `0d2930e7ac5cd1da7fc289c51c8ae1c0afc00167` 中搜索 `ICND2069` 和 `ICN2069`，未发现对应文件或符号。仓库存在 ICND2055 等芯片配置结构，但这些结构描述的是 NovaStar 配置层，不是 MCU GPIO 直接输出 ICND2069 波形。

因此它的价值是后续解析原接收卡配置、研究寄存器参数，不是直接拿来连接裸模组。

## 新发现的 ICND2069 配置资源

后续公开网页检索找到了两类明确点名 ICND2069 的工程配置。它们证明网上存在实际配置样本，但目前公开页面只展示文件目录，文件本体需要登录或付费下载，因此**尚未核实具体寄存器值**。

### FPGA 驱动工程

CSDN 资源 `icnd2069+icnd2019驱动` 上传于 2022 年，标注为 11.11 MB 的 RAR 文件，说明可用于 Xilinx 平台，移植到 Intel FPGA 时需要替换 BRAM。公开目录包含以下与配置直接相关的文件：[cite:10]

```text
init_reg.coe
register_init.mif
bram_reg_config_rgb.mif
bram_reg_config_rgb.xci
bram_reg_config_rgb.xml
```

另一个 2026 年上传的资源名为 `ICND2069与ICND2019双芯片LED显示驱动FPGA工程包`，公开目录比前者更完整，包含 20 个文件：[cite:11]

```text
RTL/send_data.v
RTL/rgb_config_order.v
RTL/decode_unit/decode_icnd2019.v
RTL/config_reg/config_cmd.v
RTL/config_reg/ic_config_reg.v
RTL/config_reg/config_reg.v
RTL/config_reg/create_config_add.v
RTL/g_clk_config.v
bram_reg_config_rgb.xcix
testbench.v
```

从文件名可以确认该工程至少包含：

- ICND2069 寄存器配置发送逻辑；
- ICND2019 行译码逻辑；
- RGB 三路配置表或 BRAM 初始化数据；
- 配置地址生成、命令发送和 GCLK 配置；
- 仿真入口。

不过，CSDN 页面中的长篇功能描述可能是平台自动生成内容，不能替代源码核验。第二个资源页面还显示下载量为 0，因此“开箱即用”“所有配置均已验证”等宣传性表述不能视为已确认事实。只有取得 `ic_config_reg.v`、`config_reg.v` 和 BRAM 初始化文件后，才能提取可信的 16 bit 寄存器表。

### 卡莱特接收卡配置

CSDN 的“利亚德渠道卡莱特配置文件”资源公开列出了 164 个文件，其中至少两份接收卡配置明确使用 `ICN2069 + ICN2019`：[cite:12]

```text
ESV4-20S-F10;ICN2069+ICN2019-YF000381.rcvbp   17 KB
ESV3.076S-13S-F10;ICN2069+ICN2019.rcvbp       9 KB
```

搜索摘要还出现了另一份：

```text
ESV2-40S-F10;ICN2069-ICN2019-160X640.rcvbp
```

这些 `.rcvbp` 是卡莱特接收卡工程配置，不是可直接编译的 MCU 驱动，但可能包含 ICND2069 的实际寄存器表、扫描数、RGB 增益、消隐和 ICND2019 行译码参数。它们比 ICND2055 默认值更接近当前共阴板，是目前公开网上找到的最有价值配置样本。

仍需注意，样本使用的是 **ICND2019 行译码芯片**，当前板使用 **RT5990**。即使 ICND2069 寄存器值可以参考，行扫描时序和译码配置也不能直接照搬；`20S`、`13S`、`40S` 等不同扫描配置之间也不能混用。

### 当前证据边界

| 问题 | 结论 |
|---|---|
| 网上是否存在点名 ICND2069 的驱动工程 | `[CONFIRMED]`：存在 FPGA 工程资源及公开文件目录 |
| 工程是否包含寄存器初始化数据 | `[MAJORITY]`：目录含 `init_reg.coe`、`register_init.mif` 和配置 RTL，但文件内容未公开 |
| 网上是否存在实际接收卡配置 | `[CONFIRMED]`：公开目录列出多份 `ICN2069+ICN2019 .rcvbp` |
| 具体 ICND2069 寄存器值是否已取得 | `[INSUFFICIENT DATA]`：公开网页未展示文件内容 |
| 这些配置能否直接用于 `ICND2069AP + RT5990` | `[DISPROVED]`：行管型号及扫描参数不匹配 |

## 可行的 ICND2069 移植路线

### 第一阶段：只验证 ICND2069 命令链

以 RP2040/RP2350 PIO 或 STM32 定时器实现以下最小原语：

```text
shift16(word)              发送 16 bit，MSB first
le_command(clock_count)    LE=1 时输出指定数量 DCLK
write_reg(addr, value)     PRE_ACT + 解锁 + WR_CFG + 结束
load_gray(word)            shift16 + DATA_LATCH
send_vsync()               LE 宽 3 DCLK
send_row(first)            首行 12 DCLK，其余行 4 DCLK
```

建议先只接一颗芯片或一条级联链，使用 `SDO` 验证移位，再做寄存器读回。不要在行管状态未知时直接执行全亮。

### 第二阶段：建立板级配置

需要从以下来源获得正确寄存器值，优先级从高到低：

1. 原接收卡 `.rcfgx`/`.ncp` 或软件配置；
2. 能点亮状态下抓取 `SIN/DCLK/LE/ROW` 波形；
3. 按官方默认值起步，逐项配置扫描数、组数、时钟分频和低电流增益；
4. 参考 DMD_STM32 的结构，但不复制 ICND2055 数值。

### 第三阶段：解决 RT5990

RT5990 是整板驱动的关键未知项。需要找出：

- 串行输入、时钟、锁存、消隐和串行输出脚；
- 16 路 NMOS 输出脚；
- 上电默认状态；
- 一帧中所需的行选择位序；
- 与 ICND2069 `ROW` 的建立、保持和消隐关系。

最佳办法是保留原接收卡，在低亮单色图案下同时抓取：

```text
ICND2069: SIN, DCLK, LE, ROW
RT5990:   疑似 DIN, CLK, LAT/BK
```

单独把 ICND2069 驱动移植成功，只代表灰度 SRAM 能写入；没有 RT5990 的正确行选择，整板仍不会形成正常二维图像。

## RP2350 实现建议

RP2350 的 PIO 很适合生成确定性时序。建议拆成两部分：

- PIO 状态机负责多路 RGB 数据、DCLK、LE 和可选 ROW 的逐周期输出；
- CPU/DMA 负责生成命令描述符、寄存器序列和帧缓冲；
- 另一状态机或 GPIO/DMA 通道负责 RT5990，直到确认它能否与 `ROW` 共用时钟；
- 初期把 DCLK 控制在官方编程指南建议的 4～16 MHz 范围内，先用约 4 MHz 留足探测裕量；
- 所有输出默认保持关闭状态，验证 `DIS_OP`、行消隐和电源限流后再发送 `EN_OP`。

若目标只是尽快点亮，RP2040 + DMD_STM32 是较短的实验入口；若目标是形成可维护、可扩展的驱动，RP2350 PIO 上按 ICND2069 官方协议重写一个小型驱动，比长期修改依赖 STM32 定时器抽象的 GPL 工程更干净。

## 最终判断

| 判断 | 状态 |
|---|---|
| 存在明确支持 ICND2069 的开源直接驱动 | `[INSUFFICIENT DATA]`：公开检索未找到 |
| 存在可复用的同协议族底层实现 | `[CONFIRMED]`：DMD_STM32 的 ICND2055 驱动 |
| DMD_STM32 可不修改直接点亮当前板 | `[DISPROVED]`：型号、极性、扫描配置和行管均不匹配 |
| 根据公开资料自行实现 ICND2069 驱动 | `[CONFIRMED]`：核心命令和寄存器接口已公开 |
| 仅实现 ICND2069 就能点亮整板 | `[DISPROVED]`：还需要 RT5990 行选择和板级映射 |

## Sources

- [cite:1] board707, DMD_STM32 项目主页：<https://github.com/board707/DMD_STM32>
- [cite:2] DMD_STM32, `DMD_SPWM_Driver.h`：<https://github.com/board707/DMD_STM32/blob/dev-V2/DMD_SPWM_Driver.h>
- [cite:3] Chipone, *ICND2055-ICND2065-ICND2059-ICND2069 Programming Guide*, V4.2：<https://downloads.olympianled.com/PDF/Driver%20ICs/ICND2055-ICND2065-ICND2059-ICND2069-Programming-Guide-EN-2019-V4.2-Olympian.pdf>
- [cite:4] Chipone, ICND2055 官方产品页：<https://www.chiponeic.com/en/pwmsg/180>
- [cite:5] Chipone, ICND2069 官方产品页：<https://www.chiponeic.com/pwmsg/178>
- [cite:6] DMD_STM32 GPL-3.0 许可证：<https://github.com/board707/DMD_STM32/blob/dev-V2/LICENSE>
- [cite:7] DMD_STM32, ICND2055 1/46 扫配置样本：<https://github.com/board707/DMD_STM32/blob/dev-V2/register_test/profiles/icnd2055/icnd2055_scan_46.h>
- [cite:8] sarakusha, novastar.js：<https://github.com/sarakusha/novastar>
- [cite:9] Pixelmatix 论坛，DMD_STM32 的 ICND2055 初始化说明：<https://community.pixelmatix.com/t/rp2040-hub75-dma-driver/1254/11>
- [cite:10] CSDN，`icnd2069+icnd2019驱动`（2022 年上传）：<https://download.csdn.net/download/weixin_42332976/85426842>
- [cite:11] CSDN，`ICND2069与ICND2019双芯片LED显示驱动FPGA工程包`：<https://download.csdn.net/download/t1u2v/92833560>
- [cite:12] CSDN，`利亚德与卡莱特LED显示系统渠道配置文件技术解析`：<https://wenku.csdn.net/doc/3r6ny9tr7k>
