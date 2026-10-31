# ICND2069 / RP2350：默认单颗、16 扫、每帧全量配置

2026-10-31 用户确认 DSView 阈值改为 **0.4 V** 后，OUT0/OUT1 都符合当前观察预期，要求先保存版本。

完整恢复快照位于仓库根目录 `icnd2069_20261031_0p4v_snapshot/`，包含实际烧录 ELF、对应源码、原始 DSL、定量分析和恢复说明。以该快照为基线，后续调试不要覆盖它。

## 当前运行参数

- 16 扫、16 通道、64 组，每帧 1024 个有效 OE（ROW）、256 个灰度字。
- 当前已烧录：1 颗 ICND2069、全白、所有通道灰度 255；每个灰度字只发送 1 份。
- `ICND2069_CHAIN_COPIES` 配置级联颗数，默认 1，允许 1～8；配置寄存器字和灰度字的传输长度均随之调整。
- 寄存器 02h=0Fh（16-1），其他配置值保留。
- 每帧一次完整发送 49 条 WR_CFG：解锁 2 条、寄存器 45 条、锁定 2 条，然后发送灰度数据。
- `FULL_CONFIG_EVERY_FRAME=true`，不再运行逐帧单项轮转。
- `SOFT_RESET_ON_START=false`。
- PIO0/SM0 + DMA0：CLK/LAT/DATA；PIO0/SM1 + DMA1：OE。
- 名义 CLK 12.5 MHz，CLK 驱动及延迟表保持保存版。帧仍为 208094 个 DCLK，实测约 16.67 ms。
- OE 运行表由 16×64 推导；第 1024 个脉冲之后保持低电平，保留原帧长度。旧 40 扫 `nova_row_runs.bin` 仅作历史资料，当前不再加载。

当前单颗全白实测见 [单颗 ICND2069 验证记录](../20261031_single_icnd2069_white/README.md)。此前 [16 扫验证记录](../20261031_scan16_full_config/README.md) 是 8 颗重复发送版本；原快照仍是 40 扫版本。

## 接线与验证

| 分析仪 | 信号 | GPIO |
|---|---|---|
| CH0 | CLK | GP15 |
| CH1 | DATA/SIN | GP13 |
| CH2 | LAT/LE | GP10 |
| CH3 | OE/ROW（旧 GCLK 网络） | GP22 |
| CH4 | OUT0 | — |
| CH5 | OUT1 | — |

DSView：100 MHz、50 ms、0.4 V。本轮只测单颗、全白、灰度 255：每个完整帧均有 49 条 WR_CFG、256 个灰度字、1024 个有效 OE；OUT0/OUT1 每帧各 512 个主脉冲，脉宽中位数约 4.65/4.68 µs。

```sh
ICND2069_PATTERN=white ICND2069_GRAY=255 cargo build --release --locked
probe-rs download --chip RP235x --protocol swd --speed 1000 --verify target/thumbv8m.main-none-eabihf/release/icnd2069-demo
probe-rs reset --chip RP235x --protocol swd --speed 1000
swift tools/capture.swift
```

`tools/inspect_dsl.py` 需要 NumPy，支持完整六路 DSL 的配置、级联字和 OUT 脉宽分析。`capture.swift --save-only` 保存当前窗口波形，不重新采集。两种方式均保留当前门限，保存时拒绝覆盖同名抓包。

## 级联数量变量

`build.rs` 读取 `ICND2069_CHAIN_COPIES`，缺省为 1；`protocol::CHAIN_COPIES` 在编译时使用该值。改值之后重新构建、烧录和 reset。非法数量使构建失败。

```sh
# 默认单颗：省略 ICND2069_CHAIN_COPIES 即为 1
ICND2069_PATTERN=white ICND2069_GRAY=255 cargo build --release --locked

# 显式指定单颗
ICND2069_CHAIN_COPIES=1 ICND2069_PATTERN=white ICND2069_GRAY=255 cargo build --release --locked

# 需要恢复 8 颗传输时可改为 8；本轮硬件只测单颗全白
ICND2069_CHAIN_COPIES=8 ICND2069_PATTERN=white ICND2069_GRAY=255 cargo build --release --locked
```

每次传输的移位长度为 `16 × CHAIN_COPIES` 个 DCLK。单颗全白抓包中，配置锁存间隔为 24 DCLK；同一行灰度锁存间隔也是 24 DCLK，换行多 1 个 DCLK。原 8 颗模式常规间隔为 136 DCLK，因此这次确实去掉了 7 份重复字。

保存版只确认当前 0.4 V 条件下的数字波形效果；不同门限的跳变统计不可直接比较。冷启动及示波器幅度尚未在保存阶段重新验证。

## 可调灰度与图案（保存版之后的扩展）

通过构建环境变量选择图案及灰度，每次构建后重新烧录。省略变量仍使用 `reference / 8192`，OUT15 关闭，其他通道同灰度。图案是 SIN 链路上 16 行、16 通道的逻辑数据。

| ICND2069_PATTERN | 图案 |
|---|---|
| `reference` | OUT0～14 同灰度，OUT15 关闭 |
| `white` | 16 通道、16 行均为指定灰度 |
| `black` | 全部关闭，忽略灰度值 |
| `out0` / `out1` | 仅指定通道开启 |
| `alternating_rows` | 偶数逻辑行开启，两路同相 |
| `checker` | 行与通道交错，两路主脉冲互补 |
| `split` | 前 8 行开启，后 8 行关闭 |

```sh
# 当前最终烧录版本：全白、灰度 255
ICND2069_PATTERN=white ICND2069_GRAY=255 cargo build --release --locked
probe-rs download --chip RP235x --protocol swd --speed 1000 --verify target/thumbv8m.main-none-eabihf/release/icnd2069-demo
probe-rs reset --chip RP235x --protocol swd --speed 1000

# 修改为单通道；构建之后仍需执行上述 download 和 reset
ICND2069_PATTERN=out0 ICND2069_GRAY=128 cargo build --release --locked
```

`ICND2069_GRAY` 接受十进制整数 `0..8192`，非法输入直接使构建失败。**8192 是软件允许的输入上限，不代表硬件 PWM 满量程。** 16 扫全量配置下，本轮灰度 255 每路每帧有 512 个主脉冲；其他灰度尚未在新模式下实测。此前 40 扫、轮转配置模式的 64/128/255 结果为 320/640/1280 个主脉冲，512 及以上为 2560 个；这些旧数量不能直接套到 16 扫模式。

`driver.rs` 和 CLK 延迟表与保存快照逐字节一致。LAT 命令宽度及传输间隔保留，每帧命令序列改为全量配置；OE 数量已改为 1024。此前 40 扫结果见 [历史灰度与图案实测报告](../20261031_pattern_validation/README.md)。

## 自动验证

以下工具会构建、烧录、复位并采集 50 ms，保留 DSView 当前阈值。需要 DSView 已打开、100 MHz、50 ms、0.4 V，及有 NumPy 的 Python。目标证据目录必须是新目录。

```sh
/tmp/icnd2069-analysis-venv/bin/python tools/validate_pattern_capture.py white 255 ../new_single_white_run
/tmp/icnd2069-analysis-venv/bin/python tools/summarize_pattern_validation.py ../new_single_white_run
```

抓包工具的 `--chain-copies` 默认 1，可指定 1～8；它同步传给构建及解码，并核验 LAT 间隔证明实际传输长度。独立运行 `inspect_dsl.py` 分析新单颗抓包时加 `--chain-copies 1`；该工具为兼容历史抓包，独立命令默认仍为 8。

每个用例保存匹配 ELF、原始 DSL、操作日志、完整解码及主脉冲行掩码。行掩码核验去除 <200 ns 的 OE 跳变，以 ≥1 µs 的 OUT 主脉冲定位开启窗口；这个方法针对本轮约 4.65 µs 的主脉冲，不适合直接判断未经测试的极低灰度。原始统计保留在 `analysis.json`，不会删除短跳变证据。

单颗全白实测每帧完整 49 条配置、256 灰度字、每字 1 份及移位长度检查通过；每帧 **64 组×16 扫=1024 个有效 OE**。未接 SDO，因此不包含芯片内部寄存器读回。
