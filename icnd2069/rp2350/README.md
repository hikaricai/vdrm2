# ICND2069 / RP2350：单颗、16 行、1 组扫描

当前 DCLK 默认 **35 MHz**，通过 `ICND2069_DCLK_MHZ` 选择 **12.5 / 25 / 35** 三档。普通帧约 **74.97 µs（13338.4 Hz）**；1.4 V 门限抓包的 133 个完整帧已验证灰度连续、256 次锁存和 16 行扫描。详见[35 MHz 实测记录](../20261031_config_interval/README.md#35-mhz-与可调档位)。

当前配置：单颗 ICND2069，16 行 × 16 通道，启动首帧及每隔 10000 帧完整发送 49 条配置（`ICND2069_CONFIG_INTERVAL` 可调）；每帧发送 256 次 DATA_LATCH，然后只扫描一遍 16 行（03h=00h），紧接下一帧。默认只让物理 OUT0 使用灰度，其他通道全部为零；目标可通过 `ICND2069_TARGET_CHANNEL=0..15` 调整。`ICND2069_SCAN_REPEATS` 默认 1，允许 1～64。16 行目标像素使用同一灰度，逐帧循环零灰度加上 511～8191 的 8 个等间距档位（取整），共 9 帧：`0→511→1608→2705→3802→4900→5997→7094→8191→0`。

[周期配置实测与原始抓包](../20261031_config_interval/README.md)：N=7 的 44 个完整帧验证配置间隔、灰度连续及 ROW 对齐；恢复 N=10000 后 46 个完整普通帧约 209.92 µs，每帧 256 次锁存及 16 行扫描。10 ms 抓包未直接覆盖 N=10000 的完整刷新间隔。

[此前单遍扫描实测与原始抓包](../20261031_single_sweep_gray/README.md)：两份独立 100 MHz / 50 ms / 0.4 V 抓包，共 142 个完整帧，配置、灰度、单颗传输和 16 次扫描均通过核验。该历史配置帧周期约 **697.70 µs，1433.28 Hz**。64 遍扫描的[灰度分析](../20261031_analysis_gray/README.md)测得约 **7246.47 µs，138 Hz**；[此前 13 档分析](../20261031_analysis_gray2/README.md)观察到 511 及以上已测字的数字输出近似饱和。当前单组模式的连续亮度分级仍待验证。

## 时序和 DMA

- `ROWS=16`、`CHANNELS=16`、`GROUPS=1`；02h=0Fh，03h=00h。
- 每帧保留 VSYNC、EN_OP。以首帧为 frame 0，仅 frame 0、N、2N…发送 PRE_ACT、解锁、全部寄存器、锁定，合计 49 条 WR_CFG；普通帧直接装载灰度，跳过配置时钟。
- `ICND2069_SHORT_DATA=1`（默认）：按 OUT15→OUT0 顺序锁存，只完整移入目标灰度和紧随目标的零字，其余通道直接锁存已经清零的移位寄存器。SIN 始终保持零，直到下一次目标传输。
- 目标为 OUT0 时，主动零字对应下一行的 OUT15。数据开始前先无锁存地移入零，避免配置残留；最后一行 OUT0 后无锁存地移入零，不产生第 257 次 DATA_LATCH。
- `ICND2069_REPEAT_LATCH_GAP=1`（默认，范围 1～64）：重复锁存间 LAT 低电平保持 1 个完整 DCLK，LAT 高电平为 1 个 DCLK。连续重复锁存的上升沿间距因此为 2 个 DCLK，防止相邻脉冲合成多时钟命令。
- 默认每行数据段为 76 DCLK，较全量移入模式的 384 DCLK 减少约 80.2%；跨行额外间隔仍保留 1 DCLK。
- 第一行扫描在数据发送结束 16 DCLK 后开始；首个 ROW 高 12 DCLK，后续高 4 DCLK，所有行周期固定为 81 DCLK。
- 帧长度根据配置、级联数、数据和最后一行窗口推导；默认单颗/OUT0 的配置帧为 3816 DCLK（35 MHz 下约 109.03 µs），普通帧为 2624 DCLK（约 74.97 µs，13338.4 Hz）。最后一行窗口结束即进入下帧。
- PIO0/SM0 驱动 CLK/LAT/DATA，SM1 驱动 ROW/OE。35 MHz 档自动使用 140 MHz 系统时钟，每周期 4 个 PIO 周期，高低各 2；另外两档沿用 150 MHz 系统时钟及 6 周期程序。三档均为整数分频。`OUT PINS,16` 消耗整个样本，OUT_COUNT=12 只驱动 GP10～21。无时钟延迟表、可变延迟分支或抓包行周期表。
- DMA0/2/4 播放两个交替帧描述符组，DMA1/3/5 同步播放 ROW 描述符。尾段完成后 DMA 自动进入下一组，DMA_IRQ_1 更新已完成组，为再下一帧选择灰度、是否配置及首个 ROW 等待长度。IRQ 优先级 P1，更新必须在一帧内完成；当前默认最短帧约 75 µs，无软件换帧等待。
- 各档输入段、扫描段和 ROW 表在启动时生成并保持不变；只更新尚未复用的描述符及两个 ROW 首段字。RAM 占用不随 N 增长，配置刷新不会重置灰度序列。
- 未发送 SOFT_RST；没有 SDO 读回。

## Flash 大小与烧录耗时

波形在启动时由代码生成到 RAM；HEADS、TAIL、ROW_RUNS 等静态零初始化缓冲位于 `.bss`，不作为 Flash 数据烧录。ELF 中的调试段也不参与烧录。

原 `include_bytes!` 完整嵌入了 208094 字节的 NOVA 时钟延迟表。现已删除表文件、提取工具、编译期提取、波形延迟字段及 PIO 可变延迟逻辑；抓包行周期表也已删除，改为固定 81 DCLK。

周期配置调度版本的可烧录段约 30 KiB，原完整延迟表版本为 238796 字节（约 233.2 KiB）。此前裁剪前缀版本在 1000 kHz SWD、带校验烧录时约 2.48 秒，完整表版本约 14.69 秒；具体耗时仍受探针和连接状态影响。

## 灰度验证结论

OUT0/OUT1 对应**上一帧发送的灰度**。验证必须错开一帧：发送 0 的那一帧仍显示前一帧，随后发送 1 的帧才显示全黑。

当前配置和 0.4 V 门限下，灰度 0 无输出，所测非零灰度 1～255 每路每帧均有 16 个主脉冲，脉宽约 4.66/4.68 µs。灰度数据变化、开关响应和扫描次数已验证，**连续亮度分级尚未验证**。不能沿用旧 64 组扫描中靠开启组数体现灰度的标定；8192 仍只是软件输入上限。

## 构建与运行

```sh
# 当前默认：单颗、全白、灰度逐帧循环
cargo build --release --locked
probe-rs download --chip RP235x --protocol swd --speed 1000 --verify target/thumbv8m.main-none-eabihf/release/icnd2069-demo
probe-rs reset --chip RP235x --protocol swd --speed 1000
swift tools/capture.swift

# 固定全白灰度（构建后同样需要 download/reset）
ICND2069_GRAY_MODE=static ICND2069_PATTERN=white ICND2069_GRAY=255 cargo build --release --locked

# 配置刷新间隔，默认 10000；1 恢复每帧完整配置
ICND2069_CONFIG_INTERVAL=10000 cargo build --release --locked

# CLK 档位，只接受 12.5、25、35，默认 35；构建后同样需要烧录
ICND2069_DCLK_MHZ=35 cargo build --release --locked

# 仅改变级联数量，允许 1～8，默认 1
ICND2069_CHAIN_COPIES=8 cargo build --release --locked

# 调整物理目标通道及重复锁存的 LAT-low 间隔
ICND2069_TARGET_CHANNEL=7 ICND2069_REPEAT_LATCH_GAP=3 cargo build --release --locked

# 恢复逐通道完整 16-bit 数据传输及完整图案
ICND2069_SHORT_DATA=0 cargo build --release --locked
```

`ICND2069_GRAY_MODE`：`sweep`（默认）或 `static`。sweep 要求 `white`；1～3 颗选择完整的 9 帧灰度序列，4～8 颗为控制 RAM 使用保留 `0,1,127,255` 四帧循环。当前硬件使用单颗配置；扩展级联须重新核对 RAM 占用。构建参数不改变当前运行的固件，烧录后才生效。

static 模式保留图案 `reference`、`white`、`black`、`out0`、`out1`、`alternating_rows`、`checker`、`split`，灰度 `ICND2069_GRAY=0..8192`。省略图案默认 white。short-data 模式只取所选物理目标通道的图案值，其他通道强制为零；完整图案需设置 `ICND2069_SHORT_DATA=0`。

## DSView 与验证工具

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
```

默认独立解码器仍按历史 8 颗处理，新单颗抓包必须加 `--chain-copies 1`。五采样点居中多数滤波在 100 MHz 下剔除/填补 10～20 ns 短跨越，保留原 DSL；报告同时记录改变的采样点数量。本轮两份最终抓包 CH0～3 改变点均为 0，原始和滤波后解码一致，不能因此推断其他抓包也没有毛刺。

旧 `summarize_pattern_validation.py` 的行掩码和阈值针对历史多组模式，不用于本轮单组循环验证。

## 历史版本

- [原始 0.4 V 恢复快照](../../icnd2069_20261031_0p4v_snapshot/README.md)：40 扫、8 颗；保持原样。
- [40 扫灰度与图案实测](../20261031_pattern_validation/README.md)。
- [16 扫、8 颗、64 组全量配置](../20261031_scan16_full_config/README.md)。
- [16 扫、单颗、64 组固定全白](../20261031_single_icnd2069_white/README.md)。
