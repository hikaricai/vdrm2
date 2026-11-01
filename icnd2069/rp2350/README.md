# ICND2069 / RP2350：单颗、16 行、8 组扫描

BK（GP18）使用原有 DCLK/PIO 指令时序。配置、数据区与帧间为低电平；每个 ROW 前 BK 只拉高 **8 个完整 DCLK**，默认 12.5 MHz 下为 640 ns。启动也保持低，避免 DSView 中首个 BK 脉冲宽于其他行。

当前默认图案为 `diagonal`：仅逻辑主对角线 `(行 i, OUTi)` 使用 `v=round(i×32/15)`、`gray=v<<1`，i=0..15，其余 240 个像素均为 0。v 序列为 `0,2,4,6,9,11,13,15,17,19,21,23,26,28,30,32`；实际灰度字为 `0,4,8,12,18,22,26,30,34,38,42,46,52,56,60,64`。首个对角像素灰度为 0，因此实际非零像素为 15 个；末像素为 64。每帧图案一致。diagonal 固定使用上述范围，不受 `ICND2069_GRAY` 影响；`ICND2069_PATTERN=gradient` 可恢复此前整屏渐变图；下文整屏渐变验证属于历史记录。

## 2026-11-01 单遍扫描灰度目视验证

用户实测运行命令：`ICND2069_SCAN_REPEATS=1 cargo run --release`，反馈“可以分清楚8个gray”。

记录时源码为上述对角线图案、`v=round(i×32/15)`、`gray=v<<1`，发送灰度范围 0～64；扫描为每帧 1 遍×16 行，共 16 个 ROW 脉冲，03h=00h。默认 DCLK 为 12.5 MHz，07h/08h=20h。此结果是用户肉眼辨认出约 8 个层级，未记录具体可区分的灰度字及是否包含黑色，未进行定量亮度标定；不据此推断芯片最大灰度能力为 8 级。记录当时源码，未通过本轮 RTT 或抓包独立核验运行固件。

当前默认**启用连续 latch**，保留上述斜向灰度线。每行目标为 OUTrow，完整移入一次 gray、一次零，其余通道仅连续锁存零；目标之前也用零锁存定位。串行顺序仍为 OUT15→OUT0。第 0 行目标在行尾，gray 后的零字不锁存，随后下一行从零开始；每帧仍恰好 256 次 DATA_LATCH。数据开始前另有一次无锁存清零，消除配置残留。

DCLK 默认 **12.5 MHz**，系统时钟 **150 MHz（不超频）**，默认每帧扫描 8 遍 16 行，共 **128 个 ROW 脉冲**。数据写入与 ROW 扫描现在重叠执行：连续 latch 的配置帧为 11610 DCLK、普通帧为 10418 DCLK，普通帧理论周期 **833.44 µs（约 1199.8 Hz）**。使用用户目视验证过的 `ICND2069_SCAN_REPEATS=1` 时，每帧 16 个 ROW，配置帧 2538 DCLK、普通帧 1346 DCLK（107.68 µs，约 9286.8 Hz）；相比此前顺序执行的 187.04 µs，理论刷新率提高约 1.74 倍。本次仅完成编译和主机编码核验，未烧录。`ICND2069_DCLK_MHZ` 可选择 **12.5 / 25 / 35 / 40 / 45**，40/45 为超频实验档。

2026-11-01 最新状态：用户反馈 16 遍效果与此前差不多，随后 8 遍固件已烧录、Flash 校验通过；RTT 确认 `groups=8`、`pattern=gradient`、`cycle=1`，配置帧 17768 DCLK、普通帧 16576 DCLK。未使用 DSView。已移除 build.rs 的扫描次数默认注入：未设置 `ICND2069_SCAN_REPEATS` 时，默认值只取 protocol.rs 的 `None => 8`；显式环境变量仍优先。修改后需重新编译、烧录。每帧 256 个灰度字与 ROW 数量分别计数；ROW=16 行×遍数，16 列在一行内并行输出。原 GCLK 网络实际连接 ICND2069 的 ROW，芯片内部 PWM GCLK 不在该引脚输出。

2026-11-01 最新反馈：32 遍也有灰度变化，但肉眼看变化主要在前半部分；尚无定量亮度或饱和阈值测量。随后改为默认 16 遍并烧录，通过 Flash 校验；RTT 确认 `groups=16`、`pattern=gradient`、`sweep=false`、`cycle=1`，波形 RAM 58324 字节，配置帧 28136 DCLK，普通帧 26944 DCLK。已退出 RTT 并重新复位，未使用 DSView；16 遍实际效果待目视确认。

2026-11-01 此前记录：用户目视确认 64 遍扫描时已经能看出灰度变化。随后 32 遍固件烧录通过 Flash 校验；RTT 确认 `groups=32`、`pattern=gradient`、`sweep=false`、`cycle=1`，波形 RAM 101844 字节，配置帧 48872 DCLK，普通帧 47680 DCLK。此前 64 遍主机编码核验为 02h=0Fh、03h=3Fh、每帧 1024 个 ROW 脉冲；32 遍为 02h=0Fh、03h=1Fh、每帧 512 个 ROW 脉冲。下文此前单组的抓包及周期数值仅对应 `ICND2069_SCAN_REPEATS=1`。

另支持 `ICND2069_DCLK_MHZ=40` 超频实验档：系统时钟 160 MHz，PIO 每 CLK 4 周期，高低各 2（均为 12.5 ns，1:1 占空比）。默认完整写入模式的普通帧理论周期 187.60 µs；此前仅编译、烧录并重启，未抓包验证。

`ICND2069_DCLK_MHZ=45` 使用系统时钟 180 MHz，高低各约 11.11 ns（1:1），默认完整写入模式的普通帧理论周期约 166.76 µs。仅 40/45 MHz 档超频；12.5/25 MHz 档为系统 150 MHz，35 MHz 档为系统 140 MHz。45 MHz 此前仅编译、烧录并重启，未抓包验证。

配置仍为单颗 ICND2069、16 行 × 16 通道，启动首帧及每隔 10000 帧完整发送 49 条配置（`ICND2069_CONFIG_INTERVAL` 可调）；每帧 256 次 DATA_LATCH 与 8 遍 16 行扫描重叠执行（03h=07h）。`ICND2069_SCAN_REPEATS` 默认 8，允许 1～64。历史整屏 `gradient` 图案按像素序号 i=0..255 使用 `round(i×8192/255)`，可通过 `ICND2069_PATTERN=gradient ICND2069_CONTINUOUS_LATCH=0` 恢复；整屏渐变必须完整写入。

[周期配置实测与原始抓包](../captures/experiments/20261031_config_interval/README.md)：N=7 的 44 个完整帧验证配置间隔、灰度连续及 ROW 对齐；恢复 N=10000 后 46 个完整普通帧约 209.92 µs，每帧 256 次锁存及 16 行扫描。10 ms 抓包未直接覆盖 N=10000 的完整刷新间隔。

[此前单遍扫描实测与原始抓包](../captures/experiments/20261031_single_sweep_gray/README.md)：两份独立 100 MHz / 50 ms / 0.4 V 抓包，共 142 个完整帧，配置、灰度、单颗传输和 16 次扫描均通过核验。该历史配置帧周期约 **697.70 µs，1433.28 Hz**。64 遍扫描的[灰度分析](../captures/experiments/20261031_analysis_gray/README.md)测得约 **7246.47 µs，138 Hz**；[此前 13 档分析](../captures/experiments/20261031_analysis_gray2/README.md)观察到 511 及以上已测字的数字输出近似饱和。当前单组模式的连续亮度分级仍待验证。

## 时序和 DMA

- `ROWS=16`、`CHANNELS=16`、`GROUPS=8`；02h=0Fh，03h=07h。
- 每帧保留 VSYNC、EN_OP。以首帧为 frame 0，仅 frame 0、N、2N…发送 PRE_ACT、解锁、全部寄存器、锁定，合计 49 条 WR_CFG；普通帧直接装载灰度，跳过配置时钟。
- ROW 在配置段结束后 16 DCLK 开始；普通帧首个 ROW 位于第 50 DCLK。数据 DMA 写入下一帧灰度，独立 ROW DMA 扫描当前显示帧；BK/DIN/LCK 与 ROW 同步叠加在数据波形上，不改变 SIN/LE。下一次 VSYNC 等扫描完整结束且数据写完后再发送，保留至少 16 DCLK 数据末尾余量。主机核验包含配置段跳过后的两路对齐；实际显示缓冲切换和输出效果待上板确认。
- `ICND2069_CONTINUOUS_LATCH=1`：开启连续锁存，按 OUT15→OUT0 顺序锁存，只完整移入目标灰度和紧随目标的零字，其余通道直接锁存已经清零的移位寄存器。SIN 始终保持零，直到下一次目标传输。
- `ICND2069_CONTINUOUS_LATCH=0`：每行 16 个通道分别完整移入灰度字（每颗芯片 16 bit）再锁存。static 模式保留完整图案；可选 sweep 模式帧内使用统一灰度，帧间按 11 档循环；此模式不使用 `TARGET_CHANNEL` 掩码。
- 旧参数 `ICND2069_SHORT_DATA` 保留为兼容别名；同时设置时，以 `ICND2069_CONTINUOUS_LATCH` 为准。
- 固定目标为 OUT0 时，主动零字对应下一行的 OUT15；斜线模式第 0 行 OUT0 后直接无锁存移入零。数据开始前先无锁存地移入零，避免配置残留；不产生第 257 次 DATA_LATCH。
- `ICND2069_REPEAT_LATCH_GAP=1`（默认，范围 1～64）：重复锁存间 LAT 低电平保持 1 个完整 DCLK，LAT 高电平为 1 个 DCLK。连续重复锁存的上升沿间距因此为 2 个 DCLK，防止相邻脉冲合成多时钟命令。
- 每个灰度/零字移入 16 CLK（单颗），末位同时 DATA_LATCH，取消之前额外的 8 CLK。完整写入每行数据段为 256 DCLK；连续 latch 为 60 DCLK，斜线第 0 行额外 2 DCLK。跨行额外间隔仍保留 1 DCLK；数据末尾按需补一个 PARK 时钟以对齐 DMA 字。WR_CFG 配置传输的间隔保持原有时序。
- 关闭连续锁存时，默认单颗配置帧为 8696 DCLK，普通帧为 7504 DCLK；12.5 MHz 下分别为 695.68 µs 和 600.32 µs（普通帧约 1665.8 Hz）。两种模式均为每帧 256 次 DATA_LATCH、16 行扫描。
- 第一行扫描在数据发送结束 16 DCLK 后开始；首个 ROW 高 12 DCLK，后续高 4 DCLK，所有行周期固定为 81 DCLK。
- 帧长度根据配置、级联数、数据和最后一行窗口推导；开启连续 latch 时，单颗/OUT0 的配置帧为 3816 DCLK（12.5 MHz 下 305.28 µs），普通帧为 2624 DCLK（209.92 µs，4763.7 Hz）。最后一行窗口结束即进入下帧。
- PIO0/SM0 驱动 CLK/LAT/DATA，SM1 驱动 ROW/OE。35 MHz 档自动使用 140 MHz 系统时钟，每周期 4 个 PIO 周期，高低各 2；另外两档沿用 150 MHz 系统时钟及 6 周期程序。三档均为整数分频。`OUT PINS,16` 消耗整个样本，OUT_COUNT=12 只驱动 GP10～21。无时钟延迟表、可变延迟分支或抓包行周期表。
- DMA0/2/4 播放两个交替帧描述符组，DMA1/3/5 同步播放 ROW 描述符。尾段完成后 DMA 自动进入下一组，DMA_IRQ_1 更新已完成组，为再下一帧选择灰度、是否配置及首个 ROW 等待长度。IRQ 优先级 P1，更新必须在一帧内完成；当前默认最短帧约 600 µs，无软件换帧等待。
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

修改扫描次数并立即烧录、复位（在本目录执行）：`bash tools/flash.sh 16`。
参数范围 1～64，例如 `bash tools/flash.sh 8` 为 128 个 ROW 脉冲/帧，`bash tools/flash.sh 16` 为 256 个。脚本将参数明确传入构建，构建成功后才烧录并校验、复位；单独 `cargo build` 不会更新板上固件。RTT 的 `ROW pulses/frame` 统计生成的 ROW 表，可确认该固件的扫描数量，不代表引脚波形实测。

```sh
# 当前默认：单颗、16 行×8 遍、连续 latch、斜向灰度线
cargo build --release --locked
probe-rs download --chip RP235x --protocol swd --speed 1000 --verify target/thumbv8m.main-none-eabihf/release/icnd2069-demo
probe-rs reset --chip RP235x --protocol swd --speed 1000
swift tools/capture.swift

# 固定全白灰度（构建后同样需要 download/reset）
ICND2069_GRAY_MODE=static ICND2069_PATTERN=white ICND2069_GRAY=255 cargo build --release --locked

# 配置刷新间隔，默认 10000；1 恢复每帧完整配置
ICND2069_CONFIG_INTERVAL=10000 cargo build --release --locked

# 常规 CLK 档位：12.5、25、35，默认 12.5；另支持实验档 40/45
ICND2069_DCLK_MHZ=12.5 cargo build --release --locked

# 超频实验档：系统 160 MHz，CLK 40 MHz，高低电平比例为 1:1
ICND2069_DCLK_MHZ=40 cargo build --release --locked

# 超频实验档：系统 180 MHz，CLK 45 MHz，高低电平比例为 1:1
ICND2069_DCLK_MHZ=45 cargo build --release --locked

# 仅改变级联数量，允许 1～8，默认 1
ICND2069_CHAIN_COPIES=8 cargo build --release --locked

# 调整物理目标通道及重复锁存的 LAT-low 间隔
ICND2069_TARGET_CHANNEL=7 ICND2069_REPEAT_LATCH_GAP=3 cargo build --release --locked

# 关闭连续 latch，每行完整写入 16 个灰度字
ICND2069_CONTINUOUS_LATCH=0 cargo build --release --locked

# 开启连续 latch，使用旧全白灰度循环
ICND2069_CONTINUOUS_LATCH=1 ICND2069_GRAY_MODE=sweep cargo build --release --locked
```

用户单遍扫描运行命令（现在默认启用连续 latch）：`ICND2069_SCAN_REPEATS=1 cargo run --release`。

`ICND2069_GRAY_MODE`：`static`（默认）或 `sweep`。默认 static 图案为 `diagonal`，固定上述灰度字；整屏 `gradient` 的 `ICND2069_GRAY` 控制渐变终点，默认 8192。sweep 默认图案为 `white`，也只允许 white；1～3 颗选择完整灰度序列（连续 latch 关闭为 11 帧、开启为 9 帧），4～8 颗沿用为控制 RAM 使用保留的 `0,1,127,255` 四帧循环。当前硬件使用单颗配置；扩展级联须重新核对 RAM 占用。构建参数不改变当前运行的固件，烧录后才生效。

static 模式支持 `diagonal`（默认）、`gradient`、`reference`、`white`、`black`、`out0`、`out1`、`alternating_rows`、`checker`、`split`。关闭连续 latch 时保留完整图案；开启时 diagonal 使用 OUTrow，其余图案只取 `ICND2069_TARGET_CHANNEL`，其他通道强制为零，例如 `out1` 图案需配合 `ICND2069_TARGET_CHANNEL=1`；gradient 与连续 latch 同时启用会在构建时报错。

本次主机核验：`rustc --edition=2021 tools/check_diagonal_latch.rs -o /tmp/icnd2069-diagonal-latch-check && /tmp/icnd2069-diagonal-latch-check`。按 DCLK 模拟串行移位并核验全部 256 个锁存值、LAT 高期间只有 1 个上升沿、DMA 头尾拼接和 ROW 数量；覆盖 1/8 颗级联、LAT-low 间隔 1/2/3/64、固定目标 OUT0/7/15，以及关闭连续 latch 的完整写入。

## DSView 与验证工具

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
