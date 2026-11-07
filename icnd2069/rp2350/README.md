# ICND2069 / RP2350：JSON 配置、54 行

## 先烧录

在 `icnd2069/rp2350` 目录执行，一条命令完成编译、烧录、运行并查看 RTT 日志：

```sh
cargo run --release
```

编译参数全部读取本目录的 [`config.json`](config.json)，修改后 Cargo 自动重新构建。`bash build.sh` 执行同一条命令；只编译使用 `cargo build --release`。

当前配置：**每路 1 颗、54 行、1 遍扫描、30 MHz、连续锁存斜线渐变**。顶层 `gray` 设置灰度范围，`repeat_latch` 是否存在只决定锁存方式，两种方式显示相同图案；不再需要 `pattern`。旧的 `ICND2069_*` 环境变量不再覆盖 JSON。

## config.json 参数

| 字段 | 当前值 | 可选值 / 含义 |
|---|---|---|
| `scan_repeats` | `1` | 整数 1～64；每帧 ROW 数为 `54 × 遍数`，03h 为 `遍数 - 1` |
| `config_interval` | `1000` | 整数 1～4294967295；第 0、N、2N…帧写完整配置 |
| `chains` | `1` | 整数 1～8；每条数据线的级联芯片数，同色链内使用相同配置和灰度 |
| `dclk_mhz` | `30` | 数值 1、3、6、10、12.5、20、25、30、31、32、33、34、35、40、45；20 及以上按 N=1 固定 GCLK=5×DCLK，详见 PLL 表；1/3 为低速时序实验档，40/45 为超频实验档 |
| `_dclk_mhz_doc` | 频率说明 | 注释字段示例，按未知字段忽略，不参与编译参数选择 |
| `gray` | `[0, 256]` | 必填；两个整数分别为渐变起点、终点，均为 0～8192；也接受单个整数表示固定灰度 |
| `repeat_latch` | 当前启用 | 可选对象；存在时开启连续 latch，省略或改名时完整写入所有像素，显示同一斜线 |
| `repeat_latch.repeat_latch_gap` | 备用值为 `1` | 整数 1～64，省略为 1；连续重复锁存之间 LAT 低电平的完整 DCLK 数 |

除 `repeat_latch`、其中的 `repeat_latch_gap` 和注释字段外，上表字段必须填写。`repeat_latch: {}` 也会开启连续 latch，间隔默认为 1；已知参数的类型和范围仍会校验。

顶层和 `repeat_latch` 内的未知字段均忽略，可用于注释、备用配置和调试记录。例如把 `repeat_latch` 改名为 `_repeat_latch` 即可关闭连续 latch 并保留原配置；只有准确名为 `repeat_latch` 的对象生效。旧的 `pattern` / `_pattern` 字段也会忽略。文件仍需符合 JSON 语法，说明内容写成字段即可。

行数 `ROWS=54`、通道数 `CHANNELS=16` 固定。当前寄存器下所有档位行周期均为 60 DCLK；20 MHz 及以上仍按 RGB 最长显示时序核算下限。02h=`0x35`，03h=`scan_repeats - 1`。组合参数仍需满足 512 KiB RAM 容量。

### DCLK 精确分频

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
| 30 | 120 | 4 | 1 |
| 31 | 124 | 4 | 1 |
| 32 | 128 | 4 | 1 |
| 33 | 132 | 4 | 1 |
| 34 | 136 | 4 | 1 |
| 35 | 140 | 4 | 1 |
| 40 | 160 | 4 | 1 |
| 45 | 180 | 4 | 1 |

31 MHz 使用系统 PLL `12 MHz × 124 / 4 / 3 = 124 MHz`，VCO 为 1488 MHz，避免使用低于合法范围的 744 MHz VCO。32/33/34 MHz 分别使用 `12 MHz × 64/66/68 / 3 / 2`。

整数分频排除了分数分频导致的长短周期；绝对频率仍受板载晶振误差和 PLL 抖动影响。若 DMA 未及时供数，阻塞的 PULL 仍会拉长 DCLK：固件每秒读取并清除 SM0 的 TXSTALL 标志，检测到后打印告警；`affected polling windows` 统计发现缺数的轮询窗口数，不是精确缺数次数。无告警不等于已完成电气波形验证，实际边沿需在芯片输入端测量。

2026-11-07 全档位核验：15 个 JSON 档位全部通过主机 PLL/PIO 精确比例、分频小数部分为零及 RGB/灰度/ROW 波形检查；31/32/33/34 MHz 和当前 30 MHz 的 RP2350 release 构建通过。本轮未烧录，TXSTALL 检测和实际边沿均待上板验证。

10/20 MHz 每 DCLK 使用 6 个 PIO 周期，整数分频为 2/1；30 MHz 使用 4 个 PIO 周期，整数分频为 1，均保持高低各半且不超频。10 MHz 的 ICND2069 PLL 配置为 `04h=02h, 05h=0Ch`；20 MHz 及以上使用下表的新参数，06h 均保留 `01h`。

2026-11-07 新增档位验证：10/20/30 MHz、6 颗级联、diagonal 均通过 release 构建、整数分频检查和 RGB/像素/ROW 主机波形核验；尚未烧录或测量实际 DCLK。

1/3/6 MHz 均使用每 DCLK 6 个 PIO 周期，整数分频分别为 25/8/4，高低各占 3 个 PIO 周期。6 MHz 配套使用 ICND2069 `04h=01h`、`05h=0Ah`，06h 保留 `01h`；1/3 MHz 沿用这组低频配置，但低于指南的 PLL 输入范围，只作为时序实验档，不保证 LED 正常输出。GPIO、每帧 DCLK 数及 ROW 脉冲数不变，实际时间随 DCLK 缩放。

2026-11-07 低频档验证：1/3/6 MHz、6 颗级联、diagonal 均通过 release 构建、整数分频检查和 RGB/像素/ROW 主机波形核验；尚未烧录或测量实际 DCLK。

### ICND2069 PLL：20 MHz 及以上固定 5 倍 GCLK

20 MHz 及以上固定 `P=5×M`，06h 保留 `01h`，暂按 N=1 计算 `GCLK=5×DCLK`。选择满足 `4 MHz < DCLK/M < 10 MHz` 且 M/P 均为非零 5 位整数的最小 M。低于 20 MHz 的配置保持原值。

| DCLK（MHz） | M / 04h（十进制） | P / 05h（十进制） | GCLK/DCLK（按 N=1） | 计算 GCLK（MHz） | 行周期（DCLK） |
|---|---|---|---|---|---|
| 20 | 3 | 15 | 5 | 100 | 60 |
| 25 | 3 | 15 | 5 | 125 | 60 |
| 30 | 4 | 20 | 5 | 150 | 60 |
| 31 | 4 | 20 | 5 | 155 | 60 |
| 32 | 4 | 20 | 5 | 160 | 60 |
| 33 | 4 | 20 | 5 | 165 | 60 |
| 34 | 4 | 20 | 5 | 170 | 60 |
| 35 | 4 | 20 | 5 | 175 | 60 |
| 40 | 5 | 25 | 5 | 200 | 60 |
| 45 | 5 | 25 | 5 | 225 | 60 |

**以上为实验配置的计算值。** 指南 p.13 明确要求 `DCLK×P/M < 100 MHz`：20 MHz 档处于边界，25 MHz 及以上档位超过该限制，本次按要求允许这些实验配置。06h 的 N 编码映射未明确，且超范围时 PLL 是否锁定、实际输出频率及显示效果均待硬件验证。

按指南 p.14～15，最小行周期为 `ceil((2×0E + 4×07 + 2×0F + 2×0D) / R) + 3`，其中寄存器按有效位取值、`R=P/M` 暂按 N=1，末尾包含显示段的 1 DCLK 和最小换行时间 2 DCLK。当前最长为 B 通道（分子 152），R=5 时需 34 DCLK，故所有高频档均采用原有的 60 DCLK 行周期，40/45 MHz 也恢复为 60。ROW 脉宽保留原值：首脉冲 12 DCLK，后续 4 DCLK。固定倍频比保持以 DCLK 计数的内部时序比例一致；绝对脉宽与刷新率仍随 DCLK 变化，超范围工作不保证显示一致。

启动 RTT 会打印 M、P、06h 写入值、按 N=1 计算的 GCLK 和行周期；100 MHz 提示边界实验，高于 100 MHz 提示超出指南范围。

验证：20～45 MHz 的全部 10 个支持档位、两种锁存模式均通过主机核验，覆盖固定 5 倍 PLL、60 DCLK 行周期、整数 PIO 分频、12 路 RGB 配置与像素、ROW 对齐及配置帧调度。当前 30 MHz／1 颗／1 遍／连续锁存配置通过 release 构建；尚未烧录或实测 GCLK。

## 连续 latch 与完整像素

有 `repeat_latch` 时，每行仅保留 `OUT(行号 % 16)`，其余通道为零；用完整的灰度/零传输加重复锁存零实现斜线。斜线灰度按顶层范围的终点→起点渐变，每 16 行重复。以下示例选中通道的灰度依次为 `255, 238, …, 17, 0`，不再固定为 64→0：

```json
"repeat_latch": {
    "repeat_latch_gap": 4
},
"gray": [0, 255]
```

省略或改名为 `_repeat_latch` 时，显示完全相同的斜线渐变，但每行 16 个像素都完整写入：选中通道写灰度，其余 15 个通道完整写零。每颗芯片每像素移入 16 DCLK：

```json
"gray": [0, 255]
```

两种锁存方式下，`[0, 255]` 的斜线如下（按物理 OUT0→OUT15），每 16 行重复：

```text
第 1 行：255,   0,   0,   0, …
第 2 行：  0, 238,   0,   0, …
第 3 行：  0,   0, 221,   0, …
第 4 行：  0,   0,   0, 204, …
```

`[255, 0]` 使斜线灰度从 0 递增到 255；`255` 或 `[255, 255]` 表示斜线上每个选中通道都为 255，其余通道仍为零。四组 RGB、每颗级联芯片显示同一图案。串行写入仍按 OUT15→OUT0；改变灰度不改变帧时序，切换锁存方式会改变输入时长及理论 FPS。

图案固定为斜线渐变。`gray=0` 或 `[0, 0]` 为全黑；8192 是软件允许的灰度字上限，不是经过亮度标定的满亮值。

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

四组 RGB 共 12 路数据，控制信号共用。所有信号都在 Pico 2 排针上，GPIO 编号与排针脚号对应如下：

| 信号 | GPIO（R / G / B 顺序） | Pico 2 排针脚号 |
|---|---|---|
| R1 / G1 / B1 | GP0 / GP1 / GP2 | 1 / 2 / 4 |
| R2 / G2 / B2 | GP3 / GP4 / GP5 | 5 / 6 / 7 |
| R3 / G3 / B3 | GP6 / GP7 / GP8 | 9 / 10 / 11 |
| R4 / G4 / B4 | GP9 / GP10 / GP11 | 12 / 14 / 15 |
| ICND2069 DCLK | GP12 | 16 |
| ICND2069 LE / LAT | GP13 | 17 |
| ICND2069 ROW（旧 GCLK 网络） | GP14 | 19 |
| RT5990 LCK | GP15 | 20 |
| RT5990 BK | GP16 | 21 |
| RT5990 DIN（行选择数据） | GP17 | 22 |

GP25 保留给 Pico 2 板载 LED；共地可接排针 3、8、13、18、23、28 或 38 脚。**旧接线需要按此表重接。**

配置按 R/G/B 分别发送，四组中同色使用相同配置；灰度数据仍在 12 路同步发送同一图案，尚未提供各组、各颜色独立图像。`chains` 表示每条数据线的级联芯片数，四组 RGB 不需要把它乘以 12。

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

其余配置三色一致；02h/03h 和 PLL 04h～06h 按当前行数、扫描遍数及 DCLK 设置。每条配置在同一组 DCLK/LE 上并行发送 RGB 字，仍为 49 条 WR_CFG，不增加时钟数或波形 RAM。启动和周期刷新均使用分色配置。

这些值是参考板的实测写入值，当前 microLED 板的绿蓝点亮效果仍需烧录后验证。

2026-11-07 主机验证：`tools/check_config_interval.rs` 逐路解码 GP0～11，直接读取上述抓包表核对 45 个配置地址及 8 项颜色差异，并检查全级联、配置刷新、轮转配置和灰度更新。它与 `tools/check_diagonal_latch.rs` 均通过默认配置、25 MHz／6 颗（连续 latch 开/关）、8 颗／64 遍扫描的核验。默认及 25 MHz／6 颗 static 固件 release 构建通过；本轮未烧录。

PIO 每个 DCLK 使用一个 32 位 DMA 样本，覆盖 GP0～GP17。DCLK 由 side-set 输出；ROW 由优先级更高的 SM1 持续保持，避免被 SM0 的输出覆盖。帧时序保持不变，波形缓冲占用和数据 DMA 带宽是此前两个 DCLK 打包一个字时的两倍。

两种锁存方式都只保存一帧图案，两个 DMA 描述符组共享该缓冲。完整像素模式的数据缓冲随 `chains` 增长；扫描尾缓冲随 `scan_repeats` 增长。

## 当前时序概览

启动 RTT 在系统时钟行后打印 `theoretical FPS`：`normal` 为普通帧、`config` 为完整写配置帧，均按 `DCLK_HZ / 帧内 DCLK 数` 计算；`avg` 按每 N 帧包含 1 个配置帧、N−1 个普通帧计算，N 为 `config_interval`。显示两位小数，是无 FIFO 停顿时的理论帧率，不是实测值。

BK（GP16）使用原有 DCLK/PIO 指令时序。配置、数据区与帧间为低电平；每个 ROW 前 BK 只拉高 **8 个完整 DCLK**，默认 12.5 MHz 下为 640 ns。启动也保持低，避免 DSView 中首个 BK 脉冲宽于其他行。

当前 `02h=0x35`，为 54 扫。有 `repeat_latch` 时令 `r = 行号 % 16`，OUTr 写入 `round((gray[0]×r + gray[1]×(15-r))/15)`，其余通道为 0。54 行包含 3 个完整周期及第 4 个周期的前 6 行。

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
- PIO0/SM0 驱动 CLK/LAT/DATA，SM1 驱动 ROW/OE。35 MHz 档自动使用 140 MHz 系统时钟，每周期 4 个 PIO 周期，高低各 2；另外两档沿用 150 MHz 系统时钟及 6 周期程序。三档均为整数分频。`OUT PINS,16` 消耗整个样本，OUT_COUNT=12 只驱动 GP10～21。无时钟延迟表、可变延迟分支或抓包行周期表。
- DMA0/2/4 播放两个交替帧描述符组，DMA1/3/5 同步播放 ROW 描述符。尾段完成后 DMA 自动进入下一组，DMA_IRQ_1 更新已完成组，为再下一帧选择灰度、是否配置及首个 ROW 等待长度。IRQ 优先级 P1，更新必须在一帧内完成；当前默认最短帧约 600 µs，无软件换帧等待。
- 各档输入段、扫描段和 ROW 表在启动时生成并保持不变；只更新尚未复用的描述符及两个 ROW 首段字。RAM 占用不随 N 增长，配置刷新不会重置灰度序列。
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

以下抓包使用旧 GPIO 分配。新接线采集时 CH0～3 应分别接 GP12（DCLK）、GP0（R1，或其他 RGB 数据脚）、GP13（LE）、GP14（ROW）；解码器的分析仪通道编号不变。

2026-11-01 当前波形输入核验：[原始抓包](../captures/reference/dsview/DSLogic%20U2Pro16-la-261101-001023.dsl)，100 MHz、实际时长 10.00448 ms、门限 1 V，仅分析 CH0～3。16 个完整普通帧，每帧 256 个灰度字一致，覆盖完整 11 档循环；周期 600.31～600.32 µs。每帧同行相邻 DATA_LATCH 间隔 24 CLK（240 次），跨行间隔 25 CLK（15 次）；LAT 高期间均只有 1 个 CLK 上升沿。ROW 每帧 16 次，周期通常 81 CLK，少数边沿的采样位置造成计数 80/82，未据此增加时序补偿。

历史抓包的 `wave.shift(value, 1, 8)` 使用 8 个空闲 CLK 加 16 位数据，共 24 CLK。现按用户要求改为 `wave.shift(value, 1, 0)`，只按 D15→D0 发送 16 位，最后一位时 LAT 高；连续 latch 的 gray 和零字均如此。指南第 6 页定义 DATA_LATCH 将移位寄存器的最后 16 位写入 SRAM。灰度 63 历史实测发送 `0x003F`，255 为 `0x00FF`，未左移到高 8 位。不能从 16 位串行接口推断低 8 位可以丢弃；指南第 20 页要求 Gamma 编码匹配组数和行宽。历史 03h=00h、07h=08h=20h 的手册灰度容量公式为 `1×32×4=128`，测试发送值 0～8191 不等同于已标定的线性亮度。

35 MHz 下，当前硬件的 0.4 V 门限抓包会漏计部分 DCLK 上升沿；输入协议核验使用 1.4 V，OUT 观察使用 0.4 V。本轮已将现场门限恢复为 0.4 V。两个门限的输出跳变数不可直接比较。

| 分析仪 | 信号 | GPIO |
|---|---|---|
| CH0 | CLK | GP15 |
| CH1 | DATA/SIN | GP13 |
| CH2 | LAT/LE | GP10 |
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
