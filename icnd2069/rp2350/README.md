# ICND2069 / RP2350：HDL、dev 与 direct

## 先烧录

在 `icnd2069/rp2350` 目录执行，一条命令完成编译、烧录、运行并查看 RTT 日志：

```sh
cargo run --release
```

编译参数全部读取本目录的 [`config.json`](config.json)，修改后 Cargo 自动重新构建。`bash build.sh` 执行同一条命令；只编译使用 `cargo build --release`。

`Ctrl+C` 可以直接退出 RTT，固件继续运行。probe-rs 清理断点时可能短暂停住 CPU；整帧由硬件 DMA 串接，不需要 CPU 在帧内补块。`row_on_cfg=false` 时，周期配置帧不发 ROW，因此仍可能按 `config_interval` 短闪一次。

要烧录后独立运行，不挂 RTT：

```sh
cargo build --release &&
probe-rs download --chip RP235x --verify target/thumbv8m.main-none-eabihf/release/icnd2069-demo &&
probe-rs reset --chip RP235x
```

## 选择板型

`board` 接受 `"hdl"`、`"dev"` 和 `"direct"`，省略时保持原来的 HDL 行为。dev 接线依据 `icnd2069/debug-devboard` 分支 `384f2d7`；direct 接线依据 `icnd2069/debug16-uniform255` 分支 `551dc02`。均核对了 `src/driver.rs`、`src/main.rs` 和 `src/protocol.rs`；参考分支旧 RTT/文档可能有不同引脚号，本实现以实际代码为准。

| 项目 | `hdl` | `dev` | `direct` |
| --- | --- | --- | --- |
| SIN | 三组 RGB：GP0/1/2、4/5/6、8/9/10 | GP13，单路，使用 R 寄存器配置 | 四组 RGB：GP5/4/3、11/10/9、2/1/0、8/7/6 |
| LE / LAT | GP12、13、14 | GP10 | GP18 |
| DCLK | GP15，SM1 在 GP17 输出时钟副本 | GP15，仅 SM0 | GP19，仅 SM0 |
| ROW | GP22 | GP22 | GP13 |
| RT5990 DIN / LCK / BK | GP20 / 16 / 18 | GP20 / 16 / 18 | GP17 / 16 / 15 |
| CPLD SEL | GP3、7、11 | 无；选址位置输出空闲字 | 无；RP2350 直接输出到面板 |
| 每颗有效 LED 通道 | OUT0～OUT14，OUT15 写零 | OUT0～OUT15 | OUT0～OUT14，OUT15 写零 |
| 灰度级联 `chains` | 固定 2 | 支持 1～8，按实际连接填写 | 支持 1～8，按实际连接填写 |
| RT5990 配置前清空 | 160 个行位 | 16 个行位 | 160 个行位 |
| PIO 数据字覆盖 | GP0～22 | GP0～22，仅接管七根线 | GP0～18，GP12/14 不接管；GP19 由 side-set 输出 |

`board` 不覆盖行数、`chains`、`cfg_chains`、频率或灰度。启用 `regs` 时，单颗 16×16 dev 开发板使用 **`regs["0x02"]="0x0F"、chains=1、cfg_chains=1`**；未启用 `regs` 的旧配置使用 `lines=16`。dev 的 `color="r"` 与 `"rgb"` 输出相同，`g`/`b`/`gb` 不输出灰度；direct 支持四组 RGB 分色，寄存器使用对应 R/G/B 值，同色四路镜像同一图案。`cpld_addresses` 在 dev/direct 上不产生选址信号。

`pin_drive.pins` 也随板型校验：dev 接受 `sin/le/dclk/row/lck/lbk/ldata`；direct 接受 `r1/g1/b1/r2/g2/b2/r3/g3/b3/r4/g4/b4/dclk/le/row/lck/lbk/ldata`；HDL 保持原字段名。direct 的 `dclk` 指 GP19，`lbk` 指 GP15，包括 `disable_rt5990=true` 时的 SIO 消隐。三块板共用频率、灰度、配置刷新和图案编码；GPIO 波形按板型直接生成，不在 DMA 数据路径中增加逐字转换。

2026-11-28 验证：两板 release 构建、14 组通道/配置逐字解码、dev PIO 指令检查通过；覆盖反向斜线、OUT15、单颗/双颗、重复锁存/完整写入。dev 单颗关闭行驱动、配置/普通阶段使用不同驱动强度的构建通过。相同 HDL 配置与图案下，配置帧和普通帧共 47,400 字节与修改前逐字一致（SHA-256 `5403c90e2d427f282e866e1b0f00fd5c17f901cf07cc59520b3034e51c033cbe`）。未烧录或验证实物显示。

2026-11-28 direct 验证：三板 release 构建、25 组像素/配置逐字解码及 direct 实际 PIO 指令检查通过；包含四组 RGB 分色、正反斜线、单颗/双颗、完整写入/重复锁存。direct 关闭行驱动并分别设置配置/普通阶段驱动强度的构建通过，生成的 DCLK/LE/BK 驱动掩码为 GP19/18/15。HDL/dev 配置帧和普通帧分别共 47,400/45,096 字节，与添加 direct 前逐字一致；当前构建产物为 JSON 选择的 direct，未烧录。

复用核验：`python3 tools/check_chain_width.py --boards-only` 检查三板及当前配置；`python3 tools/check_dclk.py` 检查当前板型 PIO；`python3 tools/check_cpld.py` 仅检查 HDL 的 CPLD，当前选 dev/direct 时运行固定 HDL 回归案例。

HDL 驱动使用 **PIO0/SM0，SM1 输出独立时钟副本**；dev/direct 只使用 SM0。启动时生成波形和 DMA 描述符，DMA0 送数据到 PIO，DMA1 在每段结束时装载下一段。LE、SIN、ROW 和 RT5990 引脚来自同一个 GPIO 数据字，SM0 用 side-set 输出 DCLK；HDL 的 SM1 同步输出 GP17 DCLK2，独立接到 CPLD。频率及驱动强度均不变时不停 PIO，FIFO 空时补零并保持时钟；切换分频或驱动强度时停用当前板型占用的状态机。配置帧仍分为慢速配置部分和高速灰度部分。

启动及周期配置帧依次执行 RT5990 清空、`EN_OP(11)`、PRE_ACT、49 条带额外清零的 WR_CFG 和灰度 DATA_LATCH。每条 WR_CFG 的配置数据前均发送 `cfg_pre_clr_chains×16` 个额外零。RT5990 清空时 DIN=0，LCK 高/低各 2 DCLK，BK 保持消隐；HDL 清空 160 位、共 644 DCLK，dev 清空 16 位、共 68 DCLK。清空长度按板型选择，独立于 `lines`、`chains`、`cfg_pre_clr_chains` 和 `cfg_chains`。`row_on_cfg=false`（默认）时，上一帧写完后进入配置帧，不发 VSYNC，整帧 ROW 保持低；下一普通帧才恢复 VSYNC 和扫描。true 时配置帧保留 VSYNC 和配置完成后的扫描。没有发送 `DIS_OP(12)`；板上表现待实测。

此前 HDL 参考配置：**每路灰度 2 颗、配置发送 4 份、54 行、每颗 15 列、1 遍扫描、DCLK/配置均为 20 MHz、GCLK 80 MHz（按 N=1）、PWM_WIDTH=32、Accelerate Rate 8×**。实际构建值以 `config.json` 和启动 RTT 为准。`07h=08h=20h` 时单 group 的理论灰阶容量为 128。顶层 `gray` 设置原始灰度字，两种锁存方式都支持 16 位。旧的 `ICND2069_*` 环境变量不再覆盖 JSON。

## config.json 参数

当前采用 `regs` 直接配置，按现有硬件保留 **54 扫、2 颗×15 通道、OUT15 写零、配置发送 4 份**。`02h=35h` 为板型适配，其余 44 项来自 [261024 抓包寄存器表](../../dsview/icnd2069/ICND2069-抓包寄存器汇总.md)，包括 `03h=3Fh`（64 组）及八项 RGB 差异。扫描数量随 02h/03h 同步调整，当前每普通帧 3456 个 ROW。`_lines`、`_scan_repeats` 等前缀下划线字段只是备忘，不生效。

`regs` 必须列出全部 45 个常规地址。`"0x1C": "0x72,0x76,0x64"` 表示 R/G/B 分别设置，`"0x07": "0x20"` 表示 RGB 共用；`_...` 可作说明。00/01 是访问控制，由协议自动发送 AA 解锁、55 锁回，不作为固定配置开放。启用 `regs` 时，`lines`、`scan_repeats`、`gclk_mhz`、`pwm_width`、`keep_scan_time` 不能同时存在，避免隐式覆盖；移除 `regs` 后仍兼容旧模式。

`regs_high_level` 可选择覆盖下方 `regs` 的字段：`lines` → 02h，`scan_repeats` → 03h，`pwm_width` → 07h/08h，`gclk` 和 `gclk_n_code` → 04h/05h/06h。扫描逻辑使用覆盖后的 02h/03h。`"gclk": "10/(4*1)"` 按 `P/(M*N)` 填，GCLK = DCLK × 此值；`gclk_n_code` 是 06h 低两位，必须单独填写，因为指南未给数学 N 与寄存器编码的映射。删除某项即用下方 `regs` 的原值；PLL 两项一起删。

当前波形缓冲占 **110,412 字节**；完整逐时钟展开需 1,968,228 字节。仅复用灰度传输结束后的完整扫描轮，首轮 ROW12、灰度数据和帧尾独立保存。关闭连续 latch 后当前硬件的波形缓冲为 323,604 字节，同样已构建通过。DMA 串接和显示效果尚未做板上抓包验证。

复核：`python3 tools/check_registers.py` 核对配置解析与实际串行 RGB 字；`python3 tools/check_timing.py` 核对压缩前后逐 DCLK 一致及扫描/灰度时序。[本次验证记录](../docs/debug/20261128_参考寄存器适配54扫.md)。

| 字段 | 当前值 | 可选值 / 含义 |
|---|---|---|
| `board` | `"direct"` | `dev` / `hdl` / `direct`，省略默认 `hdl`；板型差异见上表 |
| `regs` | 45 项 | 原始 RGB 配置；02h=35h、03h=3Fh，其余参考商业抓包 |
| `scan_repeats` | 已注释，实际 64 | 仅旧模式：整数 1～64，03h 为 `遍数 - 1`；regs 模式由 03h 推导 |
| `lines` | 已注释，实际 54 | 仅旧模式：整数 1～64，02h 为 `lines - 1`；regs 模式由 02h 推导 |
| `keep_scan_time` | 已注释 | 仅旧模式：开启时将全部 ROW 分配到灰度发送窗口；当前按参考行周期扫描 |
| `config_interval` | `10000` | 整数 1～4294967295；第 0、N、2N…帧写完整配置 |
| `chains` | `2` | HDL 固定两颗；dev/direct 允许 1～8 颗。每颗有效列数：HDL/direct 15，dev 16 |
| `cfg_pre_clr_chains` | `0` | 可选整数 0～64，省略为 0；每条 WR_CFG 前额外清零的等效级联数量，每颗 16 个 DCLK；0 表示不额外清零 |
| `cfg_chains` | `4` | 可选整数 1～8，独立于灰度 `chains`；每条 WR_CFG 发送该数量的 16-bit 配置字，省略跟随 `chains` |
| `timing` | 参考时序对象 | 可选，普通显示帧的 DCLK 起点和间隔，详见下节；省略保持旧波形 |
| `pin_drive.pins` | `"dclk"` | 对象内必填，`driver::Pins` 字段名字符串或列表，如 `["dclk","dclk2"]`；仅指定引脚应用配置，未知名字报错 |
| `pin_drive.strength` / `cfg_strength` | `"12mA"` / `"12mA"` | `2mA` / `4mA` / `8mA` / `12mA`；普通档省略默认 4mA，配置档省略跟随普通档 |
| `pin_drive.slew_rate` / `cfg_slew_rate` | `"fast"` / `"fast"` | `slow` / `fast`；普通档省略默认 slow，配置档省略跟随普通档 |
| `cpld_addresses` | `[3, 3, 3]` | 可选，配置和灰度共用的三组独立地址；0～5 单路、6/7 本组广播，省略默认 `[7,7,7]`；配置不再强制广播 |
| `row_on_cfg` | `false` | 可选布尔值，省略为 false；false 表示整个配置帧不发 VSYNC 和 ROW，true 保留配置帧 VSYNC 及配置完成后的扫描；普通帧照常发送 |
| `pattern` | `"diagonal"` | 总宽 W 为 `chains×每颗有效通道数`。`col_x` 点亮指定列，范围 `0..W-1`，沿行按 `gray[1] → gray[0]` 渐变，行数=1 时取 `gray[1]`。`diagonal` 为正向斜线（默认），列 0→W-1 循环；`diagonal_rev` 为反向，列 W-1→0 循环，逐行灰度与正向相同。均支持两种锁存方式 |
| `color` | `"rgb"` | 可选，`r`/`g`/`b` 分别仅亮红/绿/蓝；`rg`/`rb`/`gb` 亮对应两色；`rgb` 三色全亮。省略默认 `rgb`，非法值编译报错 |
| `dclk_mhz` | `12.5` | 数值 1、3、6、10、12.5、20、25、26、28、30、31、32、33、34、35、40、45；1/3 为低速时序实验档，40/45 为超频实验档 |
| `cfg_dclk_mhz` | `12.5` | 可选，配置块 DCLK MHz；省略跟随 `dclk_mhz`。正数且不高于 `dclk_mhz`，要求对称半周期和精确的 16.8 PIO 分频，分频不超过 65535 |
| `gclk_mhz` | 已注释 | 仅旧模式按 N=1 自动选择 M/P；当前直接发送 04/05/06，不推断内部频率 |
| `pwm_width` | 已注释 | 仅旧模式自动写 07h/08h；当前两项均由 regs 配为 20h |
| `_dclk_mhz_doc` | 频率说明 | 注释字段示例，按未知字段忽略，不参与编译参数选择 |
| `gray` | `"0xffff,0xffff"` | 必填；支持单个整数、两个整数的数组、`"0xffff"` 或 `"0x0000,0xffff"`；连续 latch 和完整写入都支持 0～65535，越界编译报错 |
| `repeat_latch` | 当前启用 | 可选对象；固定列和斜线均支持连续 latch，省略时完整写入相同图案 |
| `repeat_latch.repeat_latch_gap` | 备用值为 `1` | 整数 1～64，省略为 1；连续重复锁存之间 LAT 低电平的完整 DCLK 数 |

`keep_scan_time=true` 时，全部扫描均匀分配到首个 ROW 至灰度结束的窗口；不能整除的 DCLK 均匀分配，各行周期最多相差 1 DCLK。ROW 高电平仍为首行 12 DCLK、后续 4 DCLK，BK/LCK 脉宽保持 8/2 DCLK。最短行周期必须达到 14 DCLK，遍数过多会编译报错。该模式覆盖普通帧的 `timing.row_period`，不修改 GCLK 或 PWM_WIDTH。false 时普通帧使用 `timing.row_period`；省略 `timing` 的旧波形及配置帧仍用 60 DCLK。

必填字段为 `config_interval`、`chains`、`dclk_mhz`、`gray`。启用 `regs` 时必须给全 45 项；旧模式省略 `scan_repeats/lines` 时分别取 1/54。`repeat_latch: {}` 也会开启连续 latch，间隔默认为 1；已知参数的类型和范围仍会校验。

`pin_drive` 仅作用于 `pins` 指定的字段，HDL 字段名称为
`r1/g1/b1/sel1/r2/g2/b2/sel2/r3/g3/b3/sel3/lat1/lat2/lat3/dclk/dclk2/row/lck/lbk/ldata`。
未指定的引脚固定 4mA、slow；省略整个对象或 `pins: []` 时所有引脚均使用此基线。
`"dclk"` 在当前 direct 板设置 GP19；HDL/dev 为 GP15。HDL 的 GP17 DCLK2 需要显式加入列表。
输出启动前使用配置档位，进入高速清零和灰度块时恢复普通档位。
指定 `lbk` 时，禁用 RT5990 后由 SIO 控制的 BK 同样应用配置。
分频或选定引脚的电流/边沿档位变化时，先排空 PARK 尾部并停车，再更新后恢复。
两阶段电流和边沿均相同时不额外停车或重写 GPIO。原顶层三个引脚配置字段已由 `pin_drive` 替代。
驱动强度是 GPIO 输出能力档位，不是 ICND2069 的 LED 恒流设置；
slew rate 控制上升/下降沿速度，不是多引脚时钟偏斜（skew）。

顶层和 `repeat_latch` 内的未知字段均忽略，可用于注释、备用配置和调试记录。例如把 `repeat_latch` 改名为 `_repeat_latch` 即可关闭连续 latch 并保留原配置。`pattern` 为正式配置，仅接受 `col_x` / `diagonal` / `diagonal_rev`；旧 `line`、非法格式和越界列号会编译报错。`pattern_doc` / `_pattern` 仍是忽略的说明字段。文件仍需符合 JSON 语法，说明内容写成字段即可。

regs 模式的行数为 `(02h & 3Fh)+1`、组数为 `(03h & 7Fh)+1`；旧模式由 `lines/scan_repeats` 设置。每颗芯片协议通道数 `CHANNELS=16` 固定，有效 LED 列数 `LED_CHANNELS` 为 HDL/direct 15、dev 16，图像宽度为 `chains×LED_CHANNELS`。HDL/direct 的 OUT15 保留零灰度锁存槽，dev 的 OUT15 正常参与图案；协议按 OUT15→OUT0 发送。普通帧外部行周期由 `timing.row_period` 控制，PWM/PLL 不参与计算。图像大小、灰度长度、ROW 数和 DMA 缓冲随行数调整，修改后需重新编译烧录；组合参数仍需满足 512 KiB RAM 容量。

### 普通帧参考时序 `timing`

所有数值单位为 **DCLK 上升沿个数**，`*_start` 以本帧 VSYNC 的 LE 拉高为 0。[参考抓包及测量口径](../docs/debug/20261128_产品抓包DCLK时序.md)。

| 字段 | 已填参考值 | 控制含义 |
|---|---:|---|
| `en_start` | 省略 | 可选，普通帧 EN_OP 的 LE 拉高；命令宽固定 11，省略或 `null` 不发 |
| `gray_start` | 732 | 首个灰度块的串行起点；重复锁存依赖的整链清零提前完成 |
| `row_start` | 28 | 首个 ROW 上升沿；首脉冲宽 12，其余宽 4 |
| `gray_word_gap` | 8 | 完整写入时两个 16×chains 位数据块间的空闲 DCLK |
| `gray_row_gap` | 1 | 完整写入跨数据行时额外增加的 DCLK |
| `row_period` | 81 | 普通帧固定 ROW 周期；取参考主值，原包实际为 80～83 |
| `frame_tail` | 430 | 末 ROW 下降沿到下一 VSYNC 的最小 DCLK 数；取参考首帧，原包为 380～465 |

整个对象可省略；出现时除 `en_start` 外其余六项须填写，额外备注字段会忽略。

普通帧由这组参数直接生成。普通帧的 EN_OP 关闭时，启动及周期配置帧仍发送 EN_OP。`repeat_latch` 启用时，灰度字/行间隔由重复锁存算法决定，`gray_word_gap` 和 `gray_row_gap` 仅作用于完整写入。以两颗级联、`gray_start=732` 为例：完整写入首次锁存在 `732+32=764`；连续锁存的首个零字锁存在 `732+1+1=734`。参考包八颗级联的首次锁存是 `732+128=860`。

`timing` 直接用于实验波形生成，不检查协议顺序、清零余量和最小 ROW 周期。VSYNC、EN_OP、DATA_LATCH 共用 LE，重叠时高电平会连成一段；连续 latch 若起得太早，帧头零位也可能不足以清空级联。ROW 周期为零时不输出扫描；周期太短时，窄行窗口会截断部分脉冲。

`frame_tail` 是波形缓冲内的下限：若灰度或最后扫描窗口尚未结束，则等待它们完成；帧与帧之间的软件空闲时钟还会增加实测间隔。当前 54 行、64 组、两颗、重复锁存的普通帧为 **280317 DCLK**，ROW 起点 28、末次灰度锁存 4412；12.5 MHz 下缓冲时长 **22.42536 ms**。参考八颗/40行/64组的主机核验得到首次/末次锁存 **860/87803**，该八颗完整波形组合仍超出板上 RAM，只作为主机参考核验。

启动和周期配置帧仍执行完整配置、慢速清零及 `row_on_cfg` 策略；参考产品的“每帧五次 WR_CFG 轮转”没有替换当前 `config_interval` 流程。该对象用于调整普通显示帧时序，不是逐周期重放整份产品抓包。省略 `timing` 恢复此前波形。

验证：`python3 tools/check_timing.py` 读取实际 `build.rs` 产生的编译参数，解码最终 DMA 缓冲；覆盖三板、两种写入、参考八颗时序、修改全部参数、`keep_scan_time`、无效参数，以及省略对象时与 HEAD 的逐字节比较。`check_chain_width.py` 在有 `timing` 时也调用此核验。

2026-11-28 结果：三板 release 构建、25 组原波形回归、12 组新时序解码、无效时序拒绝及 5 组 PIO 检查通过。省略 `timing` 时 HDL/direct 各 47,400 字节、dev 45,096 字节的配置/普通 DMA 波形与修改前完全一致。启用新时序的 HDL CPLD 仿真三组共验证 266,932 个 DCLK。当前构建产物已核实为 direct；未烧录或采集实物波形。

### DCLK 精确分频

完整高速档位为 **1、3、6、10、12.5、20、25、26、28、30、31、32、33、34、
35、40、45 MHz**，原档位均保留。40/45 MHz 对应 RP2350 系统
160/180 MHz 超频预设；其余档位系统时钟不超过 150 MHz。
`config.json` 的 `_dclk_mhz_doc` 已恢复完整列表。
`python3 tools/check_dclk.py --presets` 可核验全部档位，分别检查
配置速度跟随高速和 1 MHz 配置；省略显式 GCLK 时采用原 PLL 预设。

高速档位与配置速度需要配对。例如 **30/20 MHz** 使用系统 120 MHz，
配置 PIO 分频 1.5，半周期正好 3 个系统周期；**25/20 MHz** 使用
系统 150 MHz，半周期为 3.75 个系统周期，不符合均匀时钟约束。
25 MHz 高速可用 12.5 或 25 MHz 配置；要求 20 MHz 配置可用
20 或 30 MHz 高速。显式 `gclk_mhz` 还需满足相应 M/P 的精确选参条件。

26/28 MHz 分别使用系统 104/112 MHz，无需超频；配置速度可跟随高速，
或分别使用 13/14 MHz。对应 GCLK 可选 78/84 MHz（M=3、P=9）。

`cfg_pre_clr_chains` 控制每条小配置（WR_CFG）前的额外清零级联数量：每次先保持 SIN/LE 为 0，发送 `16×cfg_pre_clr_chains` 个零，再发送 `16×cfg_chains` bit 配置数据。配置解锁、45 项寄存器写入和配置锁定均适用，共 49 次额外清零。原有间隔仍保留，首条 9 DCLK、其余 8 DCLK；不再在整组开头单独额外清零。例如两个字段均为 2：每条先清零 32 CLK，再写两颗共 32 bit，常规 WR_CFG 间隔为 `8+32+32=72 DCLK`，1 MHz 下为 72 µs。RT5990 的 160 行清空仍独立保留在 EN_OP 前。

`chains=2、cfg_chains=4` 时，灰度数据仍按两颗生成 30 列图像；
每条配置发送四份相同的 16-bit 字，共 64 DCLK。
配置后的慢速、高速清零各为 `16×max(chains,cfg_chains)=64` DCLK，
普通帧灰度长度不变。

`row_on_cfg` 控制配置帧的 VSYNC 和 ICND2069 ROW（GP22），不取消 RT5990 清空脉冲。false 时，保留前缀按 `cpld_addresses` 发送的八时钟选址，将其余前缀填 PARK，并清除整帧 ROW；true 时保留配置帧 VSYNC 和配置完成后的 ROW。普通帧缓冲始终保留 VSYNC 和扫描。若 `config_interval=1` 且 `row_on_cfg=false`，每帧都是配置帧，因此持续不发 VSYNC 和 ROW。

`cfg_dclk_mhz` 用于配置帧的前缀、RT5990 清空及寄存器配置段（EN_OP、PRE_ACT、解锁、寄存器写入、锁定及对齐时钟）。配置块完成后停 PIO，切回 `dclk_mhz` 发送灰度。普通帧全部使用 `dclk_mhz`。ICND2069 的 M/P 配置仍按正常数据阶段的 `dclk_mhz` 与 `gclk_mhz` 计算。

固定 PIO 程序在启动时执行一次 `SET X,0`，随后只循环
`PULL NOBLOCK、OUT PINS、NOP` 三条指令。FIFO 空时 PULL 取 X=0，
保持 SIN/SEL/LE/ROW 低，DCLK 继续运行；OUT 在下降沿更新管脚，
下一上升沿位于相邻两次 OUT 更新的正中间。4/6 周期程序均保持高低各半。
DATA、SEL、LAT、ROW 共用这一相位；GP17 时钟副本同步跟随。
30 MHz 下源端名义建立/保持时间均为 16.67 ns，配置 10 MHz 下均为 50 ns。
接收端实际窗口仍需扣除引脚边沿、飞线和器件延迟；上述数值不代表实测波形。

例如当前 30 MHz / 10 MHz：系统时钟 120 MHz，同一程序每 DCLK 4 个 PIO 周期；普通分频为 1，配置分频为 3。频率切换只改状态机分频，不再使用 GPIO 字高位或 OUT PC 跳转。

配置速度要求半周期对称且分频能精确表示；不可精确生成的组合会编译报错。RTT 打印两种速度和分频，每五秒打印帧计数。1 MHz 配置阶段低于芯片指南的 PLL 输入范围，切回正常速度后的锁定与显示效果需实测。

验证命令：`cargo build --release --locked`，随后运行 `python3 tools/check_dclk.py`、`python3 tools/check_chain_width.py` 和 `python3 tools/check_cpld.py`。最后一个需要 `iverilog`/`vvp`，将固件实际 DMA 缓冲送入 CPLD RTL，在 ICND2069 上升沿采样时刻逐位核对全部 54 路 SIN，覆盖广播、独立地址、周期配置、空闲补零及两种灰度写入。逻辑仿真不包含电气延迟，实际帧边界及输出需抓包验证。

所有档位均满足 `DCLK = 系统时钟 / PIO 整数分频 / 每 DCLK 指令周期数`。PIO 分频器的小数部分固定为 0，高低电平各占一半周期；编译期校验系统 PLL 的 VCO 范围、输出频率和 PIO 整除关系，不允许用近似分数分频凑频率。

| DCLK（MHz） | 系统时钟（MHz） | PIO 周期/DCLK | PIO 整数分频 |
|---|---|---|---|
| 1 | 150 | 6 | 25 |
| 3 | 144 | 6 | 8 |
| 6 | 144 | 6 | 4 |
| 10 | 120 | 6 | 2 |
| 12.5 | 150 | 6 | 2 |
| 20 | 120 | 6 | 1 |
| 25 | 150 | 6 | 1 |
| 26 | 104 | 4 | 1 |
| 28 | 112 | 4 | 1 |
| 30 | 120 | 4 | 1 |
| 31 | 124 | 4 | 1 |
| 32 | 128 | 4 | 1 |
| 33 | 132 | 4 | 1 |
| 34 | 136 | 4 | 1 |
| 35 | 140 | 4 | 1 |
| 40 | 160 | 4 | 1 |
| 45 | 180 | 4 | 1 |

26/28 MHz 使用系统 PLL `12 MHz × 78/84 / 3 / 3 = 104/112 MHz`，VCO 分别为 936/1008 MHz。31 MHz 使用系统 PLL `12 MHz × 124 / 4 / 3 = 124 MHz`，VCO 为 1488 MHz，避免使用低于合法范围的 744 MHz VCO。32/33/34 MHz 分别使用 `12 MHz × 64/66/68 / 3 / 2`。

绝对频率仍受晶振和 PLL 影响。频率及驱动强度均相同的发送只等 DMA 写入完成，
不排空 FIFO、不停 PIO、不重置分频；下一帧继续使用同一 DMA。
CPU 尚未提交下一帧时，PIO 输出零字补齐空闲时间，CLK 不会断开。
实际切换分频或 GPIO 驱动强度时等待 FIFO 清空，在 PARK 尾部停状态机并强制 DCLK 低，
跳回一次性初始化入口、更新驱动强度及分频、预填 FIFO 后恢复。
不再依赖阻塞 PULL 的 TXSTALL 标志判断发送完成。
普通帧仍含相同数量的数据字，但帧间可能多出零时钟，
理论 FPS 不计这段软件空闲时间，实测 FPS 会计入。

2026-11-07 全档位核验：15 个 JSON 档位全部通过主机 PLL/PIO 精确比例、分频小数部分为零及 RGB/灰度/ROW 波形检查；31/32/33/34 MHz 和当前 30 MHz 的 RP2350 release 构建通过。本轮未烧录，TXSTALL 检测和实际边沿均待上板验证。

10/20 MHz 每 DCLK 使用 6 个 PIO 周期，整数分频为 2/1；30 MHz 使用 4 个 PIO 周期，整数分频为 1，均保持高低各半且不超频。10 MHz 的 ICND2069 PLL 配置为 `04h=02h, 05h=0Ch`；20 MHz 及以上使用下表的新参数，06h 均保留 `01h`。

2026-11-07 新增档位验证：10/20/30 MHz、6 颗级联、diagonal 均通过 release 构建、整数分频检查和 RGB/像素/ROW 主机波形核验；尚未烧录或测量实际 DCLK。

1/3/6 MHz 均使用每 DCLK 6 个 PIO 周期，整数分频分别为 25/8/4，高低各占 3 个 PIO 周期。6 MHz 配套使用 ICND2069 `04h=01h`、`05h=0Ah`，06h 保留 `01h`；1/3 MHz 沿用这组低频配置，但低于指南的 PLL 输入范围，只作为时序实验档，不保证 LED 正常输出。GPIO、每帧 DCLK 数及 ROW 脉冲数不变，实际时间随 DCLK 缩放。

2026-11-07 低频档验证：1/3/6 MHz、6 颗级联、diagonal 均通过 release 构建、整数分频检查和 RGB/像素/ROW 主机波形核验；尚未烧录或测量实际 DCLK。

### ICND2069 PLL：独立设置 GCLK

本节为**未启用 regs 的旧模式**。当前 regs 模式直接发送 04/05/06，改变 DCLK 不会自动改写 PLL；以下公式和预设均不生效。

在 `config.json` 修改 `gclk_mhz`，无需再改源码中的 PLL 常量。06h 保留 `01h`，暂按 N=1 计算 `GCLK=DCLK×P/M`。自动选择满足 `4 MHz < DCLK/M < 10 MHz`、M/P 均为 1～31 且精确达到目标频率的最小 M；不做近似取整。当前 DCLK=30 MHz、GCLK=75 MHz 对应 M=4、P=10。30 MHz DCLK 下可尝试 GCLK=30、45、60、75、90、120、150 MHz 等；例如 76 MHz 无解，会明确报错。1/3 MHz DCLK 无法满足 PLL 输入范围，需省略 `gclk_mhz` 才能使用原来的低速实验预设。

**以下表格仅在省略 `gclk_mhz` 时生效**，行周期按 `keep_scan_time=false` 列示，与 PWM_WIDTH 无关。旧预设在 20 MHz 及以上使用 `P=5×M`；低频预设为 1/3/6 MHz → M=1、P=10，10/12.5 MHz → M=2、P=12。

| DCLK（MHz） | M / 04h（十进制） | P / 05h（十进制） | GCLK/DCLK（按 N=1） | 计算 GCLK（MHz） | 行周期（DCLK） |
|---|---|---|---|---|---|
| 20 | 3 | 15 | 5 | 100 | 60 |
| 25 | 3 | 15 | 5 | 125 | 60 |
| 26 | 3 | 15 | 5 | 130 | 60 |
| 28 | 3 | 15 | 5 | 140 | 60 |
| 30 | 4 | 20 | 5 | 150 | 60 |
| 31 | 4 | 20 | 5 | 155 | 60 |
| 32 | 4 | 20 | 5 | 160 | 60 |
| 33 | 4 | 20 | 5 | 165 | 60 |
| 34 | 4 | 20 | 5 | 170 | 60 |
| 35 | 4 | 20 | 5 | 175 | 60 |
| 40 | 5 | 25 | 5 | 200 | 60 |
| 45 | 5 | 25 | 5 | 225 | 60 |

**以上为实验配置的计算值。** 指南 p.13 明确要求 `DCLK×P/M < 100 MHz`：20 MHz 档处于边界，25 MHz 及以上档位超过该限制，本次按要求允许这些实验配置。06h 的 N 编码映射未明确，且超范围时 PLL 是否锁定、实际输出频率及显示效果均待硬件验证。

指南 p.14～15 的行时间公式为 `ceil((2×0E + 4×07 + 2×0F + 2×0D) / R) + 3`，其中 `R=P/M` 暂按 N=1。此公式保留为芯片显示窗口的参考，固件已按要求取消自动延长外部 ROW 的逻辑。当前 DCLK=30 MHz 时，60 DCLK 为 2 µs；PWM_WIDTH=64、GCLK=90 MHz 的理想显示段约 2.844 µs，会长于外部行周期，实际输出需实测。首个 ROW 高 12 DCLK，后续高 4 DCLK。

启动 RTT 会打印 M、P、06h 写入值、按 N=1 计算的 GCLK 和行周期；100 MHz 提示边界实验，高于 100 MHz 提示超出指南范围。

此前 PLL 档位验证：20～45 MHz 的全部 10 个支持档位、两种锁存模式均通过主机核验，覆盖固定 5 倍 PLL、60 DCLK 行周期、整数 PIO 分频、12 路 RGB 配置与像素、ROW 对齐及配置帧调度。当时 30 MHz／1 颗／1 遍／连续锁存配置通过 release 构建；未烧录或实测 GCLK。

### PWM 宽度与亮度（2026-11-08）

2026-11-14 更新：`pwm_width` 只修改三色 07h/08h 和对应日志；不改变外部扫描时序。PWM=4/16/32/64/255、`keep_scan_time` 开/关的逐字波形比较均通过，变化仅限这两个寄存器的数据位。当前配置普通帧为 3762 DCLK，即 125.4 µs（30 MHz）。以下亮度实验与自动行周期数值为旧版本记录。

使用指南 p.14 的 manual settings，`pwm_width` 同时写入三色 07h/08h。名义倍率为 `256/pwm_width`，显示段为 `4*pwm_width` GCLK/group。此前尝试值 4 对应 64×；当前值 16 对应 16×，值 32 对应 8×。指南手动值范围为 1～255，本固件保留 RGB `0Ch=1/12/13`，要求 `0Ch < 4×07h`，因此只接受 4～255。

理想显示窗口为 `4*pwm_width/GCLK`。PWM 从 32 改为 16 而 GCLK 保持 150 MHz，窗口从约 853 ns 缩至 427 ns，可能造成观察到的亮度下降。当前将 GCLK 一并降到 75 MHz，窗口回到约 853 ns；这只匹配理想显示时间，灰度编码、补偿时序与实际输出仍需实测。当前两颗级联的数据装载是帧率瓶颈，普通帧仍为 3762 DCLK，理论平均约 7970.30 FPS。

输入 `gray=[0,255]` 仍是原始串行编码范围，不能视为 256 个实际亮度等级。当前单 group 的公式灰阶容量为 64，也不能直接推断原始满亮编码为 63。启动 RTT 打印 PLL、PWM 寄存器值、名义倍率、每组显示 GCLK 数、理想窗口 ns 及理论灰阶容量。

本次验证：当前配置通过 `cargo build --release --locked` 和 `python3 tools/check_chain_width.py` 的 16 组主机波形检查，覆盖 1/2/3/8 颗级联、两种写入方式、ROW 对齐与配置调度。另外验证 GCLK=150 MHz 的 PWM=32/4、GCLK=30 MHz 的 PWM=16（91 DCLK 行周期）、DCLK=6 MHz/GCLK=30 MHz/PWM=255（212 DCLK 行周期）、省略字段的旧预设，以及频率无解/零值/PWM 越界的构建错误。主机核验脚本可用 `--config <JSON路径>` 检查其他配置。未烧录。

## 连续 latch 与完整像素

`pattern: "col_x"` 仅点亮第 x 列，其余列全黑。x 从 0 开始，范围为 `0..chains*15-1`；单颗允许 `col_0`～`col_14`，两颗级联允许 `col_0`～`col_29`，`col_15` 是第二颗芯片的 OUT0。选中列从第 0 行到第 `lines-1` 行按 `gray[1] → gray[0]` 渐变：`[0,255]` 为 255→0，`[255,0]` 为 0→255，单值为整列同一灰度。固定列每行只有一个像素，支持现有连续 latch 优化。以下斜线示例针对 `pattern: "diagonal"`。

图像为 `W=chains×15` 列、`lines` 行。行号 r 从 0 开始，斜线列号 `x=r%W`；芯片号 `c=x/15`、物理通道 `OUT=x%15`。芯片 0 最靠近 RP2350 的 SIN。有 `repeat_latch` 时，每行只向这个芯片的这个 OUT 写灰度，整条链的其余像素为零。斜线灰度在 W 列内按顶层范围的终点→起点渐变，四舍五入到整数，每 W 行重复，逐颗跳过 OUT15：

```json
"repeat_latch": {
    "repeat_latch_gap": 4
},
"gray": [0, 255]
```

连续 latch 首次写数据前发送 `16×chains` 个零，清除配置残留。每行从全零移位链开始，发送 `gray[7:0]`，再发送 `16×c` 个零，将灰度移到目标芯片；只在最后一个时钟拉高 LE 锁存。之后发送 `16×(chains−c)` 个零，将残留灰度移出整条链。灰度与清零合计固定为 `16×chains+8` DCLK。

目标 OUT>0 时，清零末尾锁存下一个 OUT，其余 14 个零通道用连续 latch；目标 OUT0 时，行末清零不锁存，之前的 15 个零通道用连续 latch。每行仍为 16 次 DATA_LATCH，同时作用于所有芯片；每帧总计 `lines×16` 次（当前为 864 次），ROW 为 `lines×scan_repeats` 次。配置仍向每颗芯片写完整的 16 bit。

此前每颗 16 列时，以 **30 MHz／2 颗／1 遍／gap=1／config_interval=1000** 为例，相对更早各芯片复制斜线的 8 bit 实现，每行省 16 DCLK，每帧省 864 DCLK；普通帧 **4626→3762 DCLK**，配置帧 **6602→5738 DCLK**，理论平均 FPS **6482.32→7970.30（+22.95%）**。该对比保留为历史记录；当前帧周期以启动 RTT 为准。多遍扫描可能由 ROW 时长限制 FPS。

省略或改名为 `_repeat_latch` 时，显示相同的整条链斜线，`chains×15` 个有效像素及每颗 OUT15 的零灰度槽均完整写入：每个 OUT 槽先发送最远芯片的 16 bit，逐颗向近端发送，最后一个 bit 拉高 LE 锁存。每行数据仍为 `16×16×chains` DCLK：

```json
"gray": [0, 255]
```

两颗级联、`gray=[0,255]` 时，两种锁存方式的斜线均为：

```text
第  1 行：列  0 → 芯片0 OUT0， gray=255
第 15 行：列 14 → 芯片0 OUT14，gray=188
第 16 行：列 15 → 芯片1 OUT0， gray=183
第 30 行：列 29 → 芯片1 OUT14，gray=115
第 31 行：列  0 → 芯片0 OUT0， gray=111
第 54 行：列 23 → 芯片1 OUT8， gray=0
```

`[255, 0]` 使斜线灰度从首行 0 递增到末行 255；`255` 或 `[255, 255]` 表示斜线上每个选中像素都为 255，其余像素仍为零。列绕回不重算灰度，单行取 `gray[1]`。三组 RGB 中由 `color` 选中的颜色同步显示同一幅宽图，未选颜色写零灰度；链内各芯片分别显示自己的 15 列。串行锁存仍按 OUT15→OUT0；改变灰度不改变帧时序，切换锁存方式会改变输入时长及理论 FPS。

图案由 `pattern` 选择固定列或斜线。`gray=0` 或 `[0, 0]` 为全黑。两种锁存方式都发送完整 16 位原始灰度字（0～65535）；连续 latch 仍复用清零后的锁存，只在点亮的像素插入完整灰度字。相比旧 8 位实现，灰度传输每行增加 8 个 DCLK。串行字的上限不等于实测满亮编码。

链宽恢复检查：`python3 tools/check_chain_width.py`，覆盖 1/2/3/8 颗、两种锁存模式、跨芯片边界、近端/远端芯片、灰度原地更新、RGB 配置、ROW 对齐和帧调度。检查不访问硬件。

2026-11-07 新结构验证：30 MHz／6 颗下，存在 `repeat_latch`（gap=4）和省略该字段均通过 release 构建及 RGB/像素/ROW 主机核验；顶层灰度在两种模式下均生效，空对象默认 gap=1，非法对象/范围会报错。本轮未烧录。

此前 2026-11-07 的 `pattern` JSON 构建验证采用旧配置结构；当前结构和使用方法以上方说明为准。

<details>
<summary>手动烧录（需要指定 SWD 速度时）</summary>

```sh
cargo build --release --locked &&
probe-rs download --chip RP235x --protocol swd --speed 1000 --verify \
  target/thumbv8m.main-none-eabihf/release/icnd2069-demo &&
probe-rs reset --chip RP235x --protocol swd --speed 1000
```

也可执行 `bash tools/flash.sh` 完成同样的步骤，参数同样读取 `config.json`。

</details>

## Pico 2 引脚接线

接线依据 [RP2350 板原理图](../docs/SCH_Schematic4_2026-11-21.pdf) 和
[CPLD/面板原理图](../../lcmxo2/project/icnd2069/SCH_Schematic5_2026-11-21.pdf)。
三组 RGB/SEL 输入 CPLD，三组 LAT 直接送面板。按 2026-11-22 新接线，GP15 送面板，GP17 独立送 CPLD；两路时钟同步：

| 信号 | GPIO（R / G / B 顺序） | Pico 2 排针脚号 |
|---|---|---|
| R1 / G1 / B1 | GP0 / GP1 / GP2 | 1 / 2 / 4 |
| SEL1 | GP3 | 5 |
| R2 / G2 / B2 | GP4 / GP5 / GP6 | 6 / 7 / 9 |
| SEL2 | GP7 | 10 |
| R3 / G3 / B3 | GP8 / GP9 / GP10 | 11 / 12 / 14 |
| SEL3 | GP11 | 15 |
| LAT1 / LAT2 / LAT3 | GP12 / GP13 / GP14 | 16 / 17 / 19 |
| ICND2069 DCLK | GP15 | 20 |
| CPLD CLK / DCLK2 | GP17 | 22 |
| RT5990 LCK | GP16 | 21 |
| RT5990 BK | GP18 | 24 |
| RT5990 DIN / LDATA | GP20 | 26 |
| ICND2069 ROW（旧 GCLK 网络） | GP22 | 29 |

GP25 保留给 Pico 2 板载 LED；共地可接排针 3、8、13、18、23、28 或 38 脚。**旧接线需要按此表重接。**

GP19/21 未被 PIO 占用，GP28 SYNC 暂未使用。LAT1～3 同步输出
同一命令；三组 RGB 共享 30 列图像，`color` 只控制灰度颜色，配置仍分色
写入选定地址的全部 RGB 颜色。每条 SIN 固定两颗 ICND2069，CPLD 只扩展 IO。

SEL 每组发送 `a2 a1 a0 0 1 1 1 1`，MSB 先发，总共 8 CLK。
地址 0～5 对应后缀 1/3/5/7/9/11，6/7 广播。原理图 U3→面板 U2
使用每组地址 0 和 3，即 R11/R17、R21/R27、R31/R37 及对应 G/B。
默认 `[7,7,7]` 同时驱动这些输出；例如 `[0,3,0]` 分别选第 1/2/3 组
的地址 0/3/0。当前没有逐地址独立图像调度。

配置帧与普通帧前八个 CLK 均设置 `cpld_addresses`，配置和灰度使用同一目的地。
原有 34 CLK 前缀不变，VSYNC 移到索引 13～15（零基）。
配置后的高速 32 CLK 清零段最后八个 CLK 再次发送相同地址，RGB/LAT 保持零。
SEL 在数据和空闲期间为零，因此地址不会误锁存；普通帧长度不增加。

### RGB 配置

采用[抓包寄存器汇总](../../dsview/icnd2069/ICND2069-抓包寄存器汇总.md)中 B/C 两份长包的颜色参数；字段含义参见[寄存器配置整理](../../dsview/icnd2069/ICND2069-寄存器配置整理.md)。以下均为十六进制，源码表在 `src/protocol.rs` 的 `REGISTERS`，数组顺序为 `[R, G, B]`。

| 地址 | R | G | B | 用途 |
|---|---|---|---|---|
| 0C | 01 | 0C | 0D | 耦合优化 1 |
| 0E | 07 | 06 | 0A | 低灰首行补偿时间 |
| 18 | 1F | 1F | 2F | 低灰首行补偿组合字段 |
| 19 | 20 | 80 | A0 | 低灰首行补偿组合字段 |
| 1C | 72 | 76 | 64 | 电流增益 |
| 1E | 41 | 46 | 48 | 消影 |
| 1F | 04 | 20 | 20 | 耦合优化 2 |
| 23 | 01 | 01 | 05 | 低灰偏移补偿 |

其余配置三色一致；当前全部取自 `config.json.regs`，02h=35h 适配 54 扫。每条配置在同一组 DCLK/LE 上并行发送 RGB 字，仍为 49 条 WR_CFG。启动和周期刷新均使用分色配置。

这些值是参考板的实测写入值，当前 microLED 板的绿蓝点亮效果仍需烧录后验证。

2026-11-07 主机验证：`tools/check_config_interval.rs` 逐路解码 GP0～11，直接读取上述抓包表核对 45 个配置地址及 8 项颜色差异，并检查全级联、配置刷新、轮转配置和灰度更新。它与 `tools/check_diagonal_latch.rs` 均通过默认配置、25 MHz／6 颗（连续 latch 开/关）、8 颗／64 遍扫描的核验。默认及 25 MHz／6 颗 static 固件 release 构建通过；本轮未烧录。

PIO 每个 DCLK 使用一个 32 位 DMA 样本，OUT 宽度为 23 位，覆盖 GP0～22；
GP15 DCLK 使用 SM0 side-set，GP17 DCLK2 使用 SM1 side-set，GP19/21 未复用到 PIO。两路复用同一份 PIO 程序、相同分频，并同步复位分频器及使能；SM1 的 OUT_COUNT=0，不写数据管脚，空 FIFO 的 PULL 不阻塞。GP17 位于 SM0 的 OUT 范围内，SM1 同时 side-set 按较高状态机编号优先级覆盖其零位。GP16 LCK 仍只由 SM0 驱动。

2026-11-22 DCLK2 适配：release 构建及共享程序的 `check_dclk.py --presets` 指令/分频检查通过；未烧录，板上同步和接面板后的波形质量待测。停车时两路一起禁止运行，随后分别执行低电平指令；有效传输从同步重启后开始。

两种锁存方式都在启动时生成普通帧和配置帧的存储缓冲，重复扫描段由 DMA0/1 串接发送。

## 当前时序概览

2026-11-21 新板适配通过 release 构建和实际 DMA→CPLD RTL 联动仿真：
四组场景共核对 257,056 个 DCLK 的全部 54 路输出，覆盖地址
`[7,7,7]`、`[0,3,7]`、`[3,0,6]`、`[5,2,1]`，两颗级联、
两种写入、选色、ROW 门控、周期配置和空闲时钟。本轮未烧录。
逻辑通过不代表飞线在 30 MHz 下满足电气时序；CPLD 原构建的内部频率
报告也不能替代上述两个外部半周期路径的验证。
下面的硬件抓包数值来自此前直接连接版本。

当前 JSON 启用 `timing` 和 64 组 regs，普通帧为 **280317 DCLK / 22.42536 ms**，
理论普通帧 **44.59 FPS**，不包含软件间隔。配置帧及加权平均帧率由启动日志计算。
此前 30 MHz 单组版本为 4755 DCLK / 158.5 µs，未配置 `timing` 时为 3762 DCLK / 125.4 µs。

2026-11-14 22:43 起，普通帧之间不再停 PIO；FIFO 空时补零并继续 DCLK，
仅实际切换分频时停车。实际检查 382 个完整帧，
帧尾到下一 VSYNC 的周期为采样量化后的 32/40 ns，无原约 5.47 µs 停顿。
详见 [连续帧实测](../captures/experiments/20261114_continuous_frames/README.md)。

配置锁回后，以 `cfg_dclk_mhz` 再发送
`max(chains, cfg_chains) × 16` 个零时钟，LE/ROW 均低。
随后切换到 `dclk_mhz`，在同一个连续 SRAM/DMA 高速块中，
先发同样数量的零时钟，再发送完整灰度帧，中间不停止 PIO；
下一普通帧的 VSYNC 才切换显示缓冲。
偶数块边界的 PARK 补齐和原有灰度前导清零仍保留。
普通帧不发送这两个新增清零块；配置理论 FPS 已计入两段清零。

启动时打印 `theoretical FPS: avg=..., normal=..., config=...`。`normal`
仅按普通帧时钟数计算，排除配置帧；`config` 包含慢速配置块和灰度块，
`avg` 按 `config_interval` 加权。理论值均不包含块间软件停顿。
每五秒打印 `measured FPS`：`normal` 用普通帧的完整主循环时间累计，
剔除整个配置帧，包含软件停顿；`avg` 包含所有帧及配置耗时。
若 `config_interval=1`，没有普通帧样本，实测 `normal` 显示 `N/A`。

2026-11-14，20 MHz 数据 / 1 MHz 配置、`cfg_chains=2`、
`cfg_pre_clr_chains=0`、54 行、1 遍、`config_interval=1000`：
启动理论 `normal=5316.32、config=350.34、avg=5242.02 FPS`；
一次五秒窗口的实测 `normal=5171.29、avg=5099.37 FPS`。

2026-11-14 单 PIO/单 DMA 最终版实测：30 MHz 普通帧发送 3762 个时钟，活动时长 125.4 µs；约 5.14 µs 的块间边沿间隔使完整普通帧周期为 130.498～130.508 µs。配置时 1 MHz，每个配置块 2668 clocks；配置帧无 VSYNC/ROW，下一普通帧恢复。30 秒 RTT 运行中及退出 RTT 后抓包均通过，详见 [本轮实测](../captures/experiments/20261114_single_pio/README.md)。

BK（当前 direct 为 GP15，HDL/dev 为 GP18）使用原有 DCLK/PIO 指令时序。每个 ROW 前 BK 只拉高 **8 个完整 DCLK**，当前 12.5 MHz 下为 640 ns；RT5990 清空期间也保持消隐。其他时段为低。

当前 `02h=35h` 对应 54 扫。正向斜线令 `x = r % W`（`W=30`，`r` 从 0 起），第 x 列写入 `round((gray[0]×r + gray[1]×(53-r))/53)`，其余列及 OUT15 为 0。反向斜线只镜像 x；灰度在 54 行内渐变一次。

## 历史调试与验证记录

以下保留不同阶段的实验记录，包含旧的环境变量配置、16 行、8/16/32/64 遍扫描、不同图案及行周期；其中“当前”“默认”“最新”等措辞描述记录当时的状态。**当前烧录方法与 JSON 参数以上方说明为准，历史命令及抓包结论只适用于对应固件。**

### 2026-11-01 单遍扫描灰度目视验证

用户实测运行命令：`ICND2069_SCAN_REPEATS=1 cargo run --release`，反馈“可以分清楚8个gray”。

记录时源码为上述对角线图案、`v=round(i×32/15)`、`gray=v<<1`，发送灰度范围 0～64；扫描为每帧 1 遍×16 行，共 16 个 ROW 脉冲，03h=00h。默认 DCLK 为 12.5 MHz，07h/08h=20h。此结果是用户肉眼辨认出约 8 个层级，未记录具体可区分的灰度字及是否包含黑色，未进行定量亮度标定；不据此推断芯片最大灰度能力为 8 级。记录当时源码，未通过本轮 RTT 或抓包独立核验运行固件。

当前默认**启用连续 latch**，保留上述斜向灰度线。每行目标为 OUTrow，完整移入一次 gray、一次零，其余通道仅连续锁存零；目标之前也用零锁存定位。串行顺序仍为 OUT15→OUT0。第 0 行目标在行尾，gray 后的零字不锁存，随后下一行从零开始；每帧仍恰好 256 次 DATA_LATCH。数据开始前另有一次无锁存清零，消除配置残留。

DCLK 默认 **12.5 MHz**，系统时钟 **150 MHz（不超频）**。当前为 54 扫，默认每帧扫描 1 遍，共 **54 个 ROW 脉冲**；数据写入与 ROW/RT5990 控制重叠执行。ROW 高电平仍为 4 DCLK，BK 为 8 DCLK，LCK 为 2 DCLK；仅将每行低电平空闲段缩短，行周期从 81 改为 60 DCLK。连续 latch 的配置帧为 4506 DCLK、普通帧为 3314 DCLK（265.12 µs，约 3771.9 Hz），ROW 控制段与灰度写入在约 3.3k DCLK 附近同时结束。需要多遍扫描可显式设置 `ICND2069_SCAN_REPEATS=2..64`。本次仅完成编译和主机编码核验，未烧录。`ICND2069_DCLK_MHZ` 可选择 **12.5 / 25 / 35 / 40 / 45**，40/45 为超频实验档。

为适配缩短后的行时间，默认 12.5 MHz DCLK 使用 `04h=02h`、`05h=0Ch`、`06h=01h`：按官方 Programming Guide V4.2 p.13-14 的 `R=P/(M×N)`，内部 GCLK 从旧的 31.25 MHz（`M=4,P=10,N=1,R=2.5`）提高到 **75 MHz**（`M=2,P=12,N=1,R=6`）。`07h/08h` 保持 `20h`，所以灰度相关的内部 GCLK 计数不变、每个 DCLK 行窗口内的显示阶段缩短。25/35/40/45 MHz 实验档保留原 PLL 系数；这次未作硬件抓包或亮度标定。

2026-11-01 最新状态：用户反馈 16 遍效果与此前差不多，随后 8 遍固件已烧录、Flash 校验通过；RTT 确认 `groups=8`、`pattern=gradient`、`cycle=1`，配置帧 17768 DCLK、普通帧 16576 DCLK。未使用 DSView。已移除 build.rs 的扫描次数默认注入：未设置 `ICND2069_SCAN_REPEATS` 时，默认值只取 protocol.rs 的 `None => 8`；显式环境变量仍优先。修改后需重新编译、烧录。每帧 256 个灰度字与 ROW 数量分别计数；ROW=16 行×遍数，16 列在一行内并行输出。原 GCLK 网络实际连接 ICND2069 的 ROW，芯片内部 PWM GCLK 不在该引脚输出。

2026-11-01 最新反馈：32 遍也有灰度变化，但肉眼看变化主要在前半部分；尚无定量亮度或饱和阈值测量。随后改为默认 16 遍并烧录，通过 Flash 校验；RTT 确认 `groups=16`、`pattern=gradient`、`sweep=false`、`cycle=1`，波形 RAM 58324 字节，配置帧 28136 DCLK，普通帧 26944 DCLK。已退出 RTT 并重新复位，未使用 DSView；16 遍实际效果待目视确认。

2026-11-01 此前记录：用户目视确认 64 遍扫描时已经能看出灰度变化。随后 32 遍固件烧录通过 Flash 校验；RTT 确认 `groups=32`、`pattern=gradient`、`sweep=false`、`cycle=1`，波形 RAM 101844 字节，配置帧 48872 DCLK，普通帧 47680 DCLK。此前 64 遍主机编码核验为 02h=0Fh、03h=3Fh、每帧 1024 个 ROW 脉冲；32 遍为 02h=0Fh、03h=1Fh、每帧 512 个 ROW 脉冲。下文此前单组的抓包及周期数值仅对应 `ICND2069_SCAN_REPEATS=1`。

另支持 `ICND2069_DCLK_MHZ=40` 超频实验档：系统时钟 160 MHz，PIO 每 CLK 4 周期，高低各 2（均为 12.5 ns，1:1 占空比）。默认完整写入模式的普通帧理论周期 187.60 µs；此前仅编译、烧录并重启，未抓包验证。

`ICND2069_DCLK_MHZ=45` 使用系统时钟 180 MHz，高低各约 11.11 ns（1:1），默认完整写入模式的普通帧理论周期约 166.76 µs。仅 40/45 MHz 档超频；12.5/25 MHz 档为系统 150 MHz，35 MHz 档为系统 140 MHz。45 MHz 此前仅编译、烧录并重启，未抓包验证。

配置仍为单颗 ICND2069、16 行 × 16 通道，启动首帧及每隔 10000 帧完整发送 49 条配置（`ICND2069_CONFIG_INTERVAL` 可调）；每帧 256 次 DATA_LATCH 与 8 遍 16 行扫描重叠执行（03h=07h）。`ICND2069_SCAN_REPEATS` 默认 8，允许 1～64。历史整屏 `gradient` 图案按像素序号 i=0..255 使用 `round(i×8192/255)`，可通过 `ICND2069_PATTERN=gradient ICND2069_CONTINUOUS_LATCH=0` 恢复；整屏渐变必须完整写入。

[周期配置实测与原始抓包](../captures/experiments/20261031_config_interval/README.md)：N=7 的 44 个完整帧验证配置间隔、灰度连续及 ROW 对齐；恢复 N=10000 后 46 个完整普通帧约 209.92 µs，每帧 256 次锁存及 16 行扫描。10 ms 抓包未直接覆盖 N=10000 的完整刷新间隔。

[此前单遍扫描实测与原始抓包](../captures/experiments/20261031_single_sweep_gray/README.md)：两份独立 100 MHz / 50 ms / 0.4 V 抓包，共 142 个完整帧，配置、灰度、单颗传输和 16 次扫描均通过核验。该历史配置帧周期约 **697.70 µs，1433.28 Hz**。64 遍扫描的[灰度分析](../captures/experiments/20261031_analysis_gray/README.md)测得约 **7246.47 µs，138 Hz**；[此前 13 档分析](../captures/experiments/20261031_analysis_gray2/README.md)观察到 511 及以上已测字的数字输出近似饱和。当前单组模式的连续亮度分级仍待验证。

### 时序和 DMA（历史）

- `ROWS=16`、`CHANNELS=16`、`GROUPS=8`；02h=0Fh，03h=07h。
- 每帧保留 VSYNC、EN_OP。以首帧为 frame 0，仅 frame 0、N、2N…发送 PRE_ACT、解锁、全部寄存器、锁定，合计 49 条 WR_CFG；普通帧直接装载灰度，跳过配置时钟。
- ROW 在配置段结束后 16 DCLK 开始；普通帧首个 ROW 位于第 50 DCLK。数据 DMA 写入下一帧灰度，独立 ROW DMA 扫描当前显示帧；BK/DIN/LCK 与 ROW 同步叠加在数据波形上，不改变 SIN/LE。下一次 VSYNC 等扫描完整结束且数据写完后再发送，保留至少 16 DCLK 数据末尾余量。主机核验包含配置段跳过后的两路对齐；实际显示缓冲切换和输出效果待上板确认。
- `ICND2069_CONTINUOUS_LATCH=1`：开启连续锁存，按 OUT15→OUT0 顺序锁存，只完整移入目标灰度和紧随目标的零字，其余通道直接锁存已经清零的移位寄存器。SIN 始终保持零，直到下一次目标传输。
- `ICND2069_CONTINUOUS_LATCH=0`：每行 16 个通道分别完整移入灰度字（每颗芯片 16 bit）再锁存。static 模式保留完整图案；可选 sweep 模式帧内使用统一灰度，帧间按 11 档循环；此模式不使用 `TARGET_CHANNEL` 掩码。
- 旧参数 `ICND2069_SHORT_DATA` 保留为兼容别名；同时设置时，以 `ICND2069_CONTINUOUS_LATCH` 为准。
- 固定目标为 OUT0 时，主动零字对应下一行的 OUT15；斜线模式第 0 行 OUT0 后直接无锁存移入零。数据开始前先无锁存地移入零，避免配置残留；不产生第 257 次 DATA_LATCH。
- `ICND2069_REPEAT_LATCH_GAP=1`（默认，范围 1～64）：重复锁存间 LAT 低电平保持 1 个完整 DCLK，LAT 高电平为 1 个 DCLK。连续重复锁存的上升沿间距因此为 2 个 DCLK，防止相邻脉冲合成多时钟命令。连续 latch 模式取消旧的行间额外低电平，重复锁存跨行也保持 `0,1` 交替；灰度/零字自身的 16 位串行移入仍会保持 LAT 低。
- 每个灰度/零字移入 16 CLK（单颗），末位同时 DATA_LATCH，取消之前额外的 8 CLK。完整写入每行数据段为 256 DCLK；连续 latch 为 60 DCLK，斜线第 0 行额外 2 DCLK。跨行额外间隔仍保留 1 DCLK；数据末尾按需补一个 PARK 时钟以对齐 DMA 字。WR_CFG 配置传输的间隔保持原有时序。
- 关闭连续锁存时，默认单颗配置帧为 8696 DCLK，普通帧为 7504 DCLK；12.5 MHz 下分别为 695.68 µs 和 600.32 µs（普通帧约 1665.8 Hz）。两种模式均为每帧 256 次 DATA_LATCH、16 行扫描。
- 第一行扫描在数据发送结束 16 DCLK 后开始；首个 ROW 高 12 DCLK，后续高 4 DCLK，所有行周期固定为 81 DCLK。
- 帧长度根据配置、级联数、数据和最后一行窗口推导；开启连续 latch 时，单颗/OUT0 的配置帧为 3816 DCLK（12.5 MHz 下 305.28 µs），普通帧为 2624 DCLK（209.92 µs，4763.7 Hz）。最后一行窗口结束即进入下帧。
- 当前仅 SM0 输出所有引脚，DMA0 顺序发送普通帧或配置帧；每个 DCLK 使用固定 4/6 PIO 周期，GPIO 为 GP0～18。
- 每块最后一个 DCLK 完成后，PIO 在低电平停下；配置块使用慢速分频，灰度和普通帧使用正常分频。
- 普通帧及配置帧缓冲在启动时生成并保持不变，无 DMA 描述符链和 ROW IRQ 同步。
- 未发送 SOFT_RST；没有 SDO 读回。

### Flash 大小与烧录耗时（历史）

波形在启动时由代码生成到 RAM；HEADS、TAIL、ROW_RUNS 等静态零初始化缓冲位于 `.bss`，不作为 Flash 数据烧录。ELF 中的调试段也不参与烧录。

原 `include_bytes!` 完整嵌入了 208094 字节的 NOVA 时钟延迟表。现已删除表文件、提取工具、编译期提取、波形延迟字段及 PIO 可变延迟逻辑；抓包行周期表也已删除，改为固定 81 DCLK。

周期配置调度版本的可烧录段约 30 KiB，原完整延迟表版本为 238796 字节（约 233.2 KiB）。此前裁剪前缀版本在 1000 kHz SWD、带校验烧录时约 2.48 秒，完整表版本约 14.69 秒；具体耗时仍受探针和连接状态影响。

### 灰度验证结论（历史）

OUT0/OUT1 对应**上一帧发送的灰度**。验证必须错开一帧：发送 0 的那一帧仍显示前一帧，随后发送 1 的帧才显示全黑。

当前配置和 0.4 V 门限下，灰度 0 无输出，所测非零灰度 1～255 每路每帧均有 16 个主脉冲，脉宽约 4.66/4.68 µs。灰度数据变化、开关响应和扫描次数已验证，**连续亮度分级尚未验证**。不能沿用旧 64 组扫描中靠开启组数体现灰度的标定；8192 仍只是软件输入上限。

### 主机编码核验（历史）

本次主机核验：`rustc --edition=2021 tools/check_diagonal_latch.rs -o /tmp/icnd2069-diagonal-latch-check && /tmp/icnd2069-diagonal-latch-check`。按 DCLK 模拟串行移位并核验全部 256 个锁存值、LAT 高期间只有 1 个上升沿、DMA 头尾拼接和 ROW 数量；覆盖 1/8 颗级联、LAT-low 间隔 1/2/3/64、固定目标 OUT0/7/15，以及关闭连续 latch 的完整写入。

## DSView 与验证工具（历史接线）

以下抓包使用旧 GPIO 分配。新接线 CH0～3 应接 GP15（DCLK）、
面板端的 CPLD R11（SIN，观察 CPLD 后的数据）、GP12（LAT1）、GP22（ROW）。
要分析选址，可另外采集 GP3/7/11；解码器的分析仪通道编号不变。

2026-11-01 当前波形输入核验：[原始抓包](../captures/reference/dsview/DSLogic%20U2Pro16-la-261101-001023.dsl)，100 MHz、实际时长 10.00448 ms、门限 1 V，仅分析 CH0～3。16 个完整普通帧，每帧 256 个灰度字一致，覆盖完整 11 档循环；周期 600.31～600.32 µs。每帧同行相邻 DATA_LATCH 间隔 24 CLK（240 次），跨行间隔 25 CLK（15 次）；LAT 高期间均只有 1 个 CLK 上升沿。ROW 每帧 16 次，周期通常 81 CLK，少数边沿的采样位置造成计数 80/82，未据此增加时序补偿。

历史抓包的 `wave.shift(value, 1, 8)` 使用 8 个空闲 CLK 加 16 位数据，共 24 CLK。现按用户要求改为 `wave.shift(value, 1, 0)`，只按 D15→D0 发送 16 位，最后一位时 LAT 高；连续 latch 的 gray 和零字均如此。指南第 6 页定义 DATA_LATCH 将移位寄存器的最后 16 位写入 SRAM。灰度 63 历史实测发送 `0x003F`，255 为 `0x00FF`，未左移到高 8 位。不能从 16 位串行接口推断低 8 位可以丢弃；指南第 20 页要求 Gamma 编码匹配组数和行宽。历史 03h=00h、07h=08h=20h 的手册灰度容量公式为 `1×32×4=128`，测试发送值 0～8191 不等同于已标定的线性亮度。

35 MHz 下，当前硬件的 0.4 V 门限抓包会漏计部分 DCLK 上升沿；输入协议核验使用 1.4 V，OUT 观察使用 0.4 V。本轮已将现场门限恢复为 0.4 V。两个门限的输出跳变数不可直接比较。

| 分析仪 | 信号 | GPIO |
|---|---|---|
| CH0 | CLK | GP15 |
| CH1 | DATA/SIN | CPLD 输出 R11 |
| CH2 | LAT1/LE | GP12 |
| CH3 | OE/ROW（旧 GCLK 网络） | GP22 |
| CH4 | OUT0 | — |
| CH5 | OUT1 | — |

DSView：100 MHz、50 ms、0.4 V。`capture.swift --save-only` 只保存已有波形；正常运行会采集 50 ms。均拒绝覆盖已有抓包。

```sh
# 原始单颗抓包解码
/tmp/icnd2069-analysis-venv/bin/python tools/inspect_dsl.py capture.dsl raw.json --chain-copies 1

# 可选输入去毛刺：只作用于 CH0～3，OUT0/1 完全不滤波
/tmp/icnd2069-analysis-venv/bin/python tools/inspect_dsl.py capture.dsl filtered.json --chain-copies 1 --deglitch-samples 5

# 专用于默认单颗、十灰度循环：目录含 capture.dsl 和匹配 firmware.elf
/tmp/icnd2069-analysis-venv/bin/python tools/verify_sweep_capture.py ../new_sweep_capture

# 固定图案验证工具显式使用 static 模式，会烧录并采集
/tmp/icnd2069-analysis-venv/bin/python tools/validate_pattern_capture.py white 255 ../new_static_capture

# 历史完整传输模式的主机编码核验，不访问硬件
ICND2069_SHORT_DATA=0 rustc --edition=2021 tools/check_waveform.rs -o /tmp/icnd2069-check
/tmp/icnd2069-check --white-only

# 周期配置、灰度连续性、普通/配置波形和 ROW 对齐核验
rustc --edition=2021 tools/check_config_interval.rs -o /tmp/icnd2069-interval-check
/tmp/icnd2069-interval-check

# 关闭连续 latch 的完整传输核验，同样检查每行 16 字及通道内容
ICND2069_CONTINUOUS_LATCH=0 rustc --edition=2021 tools/check_config_interval.rs -o /tmp/icnd2069-latch-off-check
/tmp/icnd2069-latch-off-check
```

连续 latch 开关的两种 release 构建及主机编码核验均通过：覆盖全部 9 档灰度、配置帧/普通帧、256 次锁存、ROW 对齐及配置调度。关闭模式还核验每个灰度字的完整传输间隔；本次未新增硬件抓包。

默认独立解码器仍按历史 8 颗处理，新单颗抓包必须加 `--chain-copies 1`。五采样点居中多数滤波在 100 MHz 下剔除/填补 10～20 ns 短跨越，保留原 DSL；报告同时记录改变的采样点数量。本轮两份最终抓包 CH0～3 改变点均为 0，原始和滤波后解码一致，不能因此推断其他抓包也没有毛刺。

旧 `summarize_pattern_validation.py` 的行掩码和阈值针对历史多组模式，不用于本轮单组循环验证。

## 历史版本

- [原始 0.4 V 恢复快照](../../icnd2069_20261031_0p4v_snapshot/README.md)：40 扫、8 颗；保持原样。
- [40 扫灰度与图案实测](../captures/experiments/20261031_pattern_validation/README.md)。
- [16 扫、8 颗、64 组全量配置](../captures/experiments/20261031_scan16_full_config/README.md)。
- [16 扫、单颗、64 组固定全白](../captures/experiments/20261031_single_icnd2069_white/README.md)。
