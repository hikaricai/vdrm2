# ICND2069 / RP2350：单颗、16 行、1 组扫描

当前配置：单颗 ICND2069，16 行 × 16 通道，每帧完整发送 49 条配置、256 次 DATA_LATCH，然后只扫描一遍 16 行（03h=00h），紧接下一帧。默认只让物理 OUT0 使用灰度，其他通道全部为零；目标可通过 `ICND2069_TARGET_CHANNEL=0..15` 调整。`ICND2069_SCAN_REPEATS` 默认 1，允许 1～64。16 行目标像素使用同一灰度，逐帧循环零灰度加上 511～8191 的 8 个等间距档位（取整），共 9 帧：`0→511→1608→2705→3802→4900→5997→7094→8191→0`。

[此前单遍扫描实测与原始抓包](../20261031_single_sweep_gray/README.md)：两份独立 100 MHz / 50 ms / 0.4 V 抓包，共 142 个完整帧，配置、灰度、单颗传输和 16 次扫描均通过核验。该历史配置帧周期约 **697.70 µs，1433.28 Hz**。64 遍扫描的[灰度分析](../20261031_analysis_gray/README.md)测得约 **7246.47 µs，138 Hz**；[此前 13 档分析](../20261031_analysis_gray2/README.md)观察到 511 及以上已测字的数字输出近似饱和。当前单组、9 帧循环尚待输出抓包验证。

## 时序和 DMA

- `ROWS=16`、`CHANNELS=16`、`GROUPS=1`；02h=0Fh，03h=00h。
- 每帧 VSYNC、EN_OP、PRE_ACT，再解锁、全部寄存器、锁定，合计 49 条 WR_CFG；之后装载灰度。
- `ICND2069_SHORT_DATA=1`（默认）：按 OUT15→OUT0 顺序锁存，只完整移入目标灰度和紧随目标的零字，其余通道直接锁存已经清零的移位寄存器。SIN 始终保持零，直到下一次目标传输。
- 目标为 OUT0 时，主动零字对应下一行的 OUT15。数据开始前先无锁存地移入零，避免配置残留；最后一行 OUT0 后无锁存地移入零，不产生第 257 次 DATA_LATCH。
- `ICND2069_REPEAT_LATCH_GAP=1`（默认，范围 1～64）：重复锁存间 LAT 低电平保持 1 个完整 DCLK，LAT 高电平为 1 个 DCLK。连续重复锁存的上升沿间距因此为 2 个 DCLK，防止相邻脉冲合成多时钟命令。
- 默认每行数据段为 76 DCLK，较全量移入模式的 384 DCLK 减少约 80.2%；跨行额外间隔仍保留 1 DCLK。
- 第一行扫描在数据发送结束 16 DCLK 后开始；首个 ROW 高 12 DCLK，后续 15 个高 4 DCLK，行周期保留原表前 16 项（80～82 DCLK）。
- 帧长度根据配置、级联数、数据和最后一行窗口推导；默认单颗/OUT0 为 3816 DCLK。按编码中的时钟延迟计算约 306.25 µs、3265 Hz，尚待实际抓包。最后一行窗口结束即进入下帧。
- PIO0/SM0 驱动 CLK/LAT/DATA，SM1 驱动 ROW/OE。名义 CLK 12.5 MHz，PIO 指令及原延迟表内容保留，紧凑帧仅使用延迟表前缀。
- DMA0/2 通过描述符串接各档输入段和共享扫描段，DMA4 重启描述符循环；DMA1/3 循环播放 ROW 表。CPU 不参与换帧，循环边界也没有软件等待。
- 单颗默认 9 帧循环，各档输入段独立、扫描段和 ROW 表共享，均在静态 RAM；播放后不修改 DMA 使用的缓冲区。
- 未发送 SOFT_RST；没有 SDO 读回。

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
```

默认独立解码器仍按历史 8 颗处理，新单颗抓包必须加 `--chain-copies 1`。五采样点居中多数滤波在 100 MHz 下剔除/填补 10～20 ns 短跨越，保留原 DSL；报告同时记录改变的采样点数量。本轮两份最终抓包 CH0～3 改变点均为 0，原始和滤波后解码一致，不能因此推断其他抓包也没有毛刺。

旧 `summarize_pattern_validation.py` 的行掩码和阈值针对历史多组模式，不用于本轮单组循环验证。

## 历史版本

- [原始 0.4 V 恢复快照](../../icnd2069_20261031_0p4v_snapshot/README.md)：40 扫、8 颗；保持原样。
- [40 扫灰度与图案实测](../20261031_pattern_validation/README.md)。
- [16 扫、8 颗、64 组全量配置](../20261031_scan16_full_config/README.md)。
- [16 扫、单颗、64 组固定全白](../20261031_single_icnd2069_white/README.md)。
