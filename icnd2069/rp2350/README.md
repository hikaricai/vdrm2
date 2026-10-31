# ICND2069 / RP2350：0.4 V 保存基线

2026-10-31 用户确认 DSView 阈值改为 **0.4 V** 后，OUT0/OUT1 都符合当前观察预期，要求先保存版本。

完整恢复快照位于仓库根目录 `icnd2069_20261031_0p4v_snapshot/`，包含实际烧录 ELF、对应源码、原始 DSL、定量分析和恢复说明。以该快照为基线，后续调试不要覆盖它。

## 当前运行参数

- 40 扫、64 组，每帧 2560 个 OE（ROW）、640 个灰度字。
- OUT15=0，OUT14～OUT0=8192；每个灰度字连续发送 8 份。
- 启动全量写配置，随后每帧按 NOVA 47 项配置表轮转，固定 5 条 WR_CFG。
- `FULL_CONFIG_EVERY_FRAME=false`；全量每帧模式仍可切换。
- `SOFT_RESET_ON_START=false`。
- PIO0/SM0 + DMA0：CLK/LAT/DATA；PIO0/SM1 + DMA1：OE。
- 名义 CLK 12.5 MHz。保存版保留已实际烧录的参考时序资源；撤回 CLK 延迟的计划尚未执行。后续改动另做版本。

分支名 `icnd2069/debug16-uniform255` 是历史名称，当前源码不是 16 扫灰度 255。

## 接线与验证

| 分析仪 | 信号 | GPIO |
|---|---|---|
| CH0 | CLK | GP15 |
| CH1 | DATA/SIN | GP13 |
| CH2 | LAT/LE | GP10 |
| CH3 | OE/ROW（旧 GCLK 网络） | GP22 |
| CH4 | OUT0 | — |
| CH5 | OUT1 | — |

DSView：100 MHz、50 ms、0.4 V。保存现场 OUT0/OUT1 主脉宽中位数约 4.65/4.67 µs，约 99.97% 的 OE 窗口检测到两路输出。

```sh
cargo build --release --locked
rustc --edition=2021 tools/check_waveform.rs -o /tmp/icnd2069-check-waveform
/tmp/icnd2069-check-waveform
swift tools/capture.swift
```

`tools/inspect_dsl.py` 需要 NumPy，支持完整六路 DSL 的配置、级联字和 OUT 脉宽分析。`capture.swift --save-only` 保存当前窗口波形，不重新采集。两种方式均保留当前门限，保存时拒绝覆盖同名抓包。

保存版只确认当前 0.4 V 条件下的数字波形效果；不同门限的跳变统计不可直接比较。冷启动及示波器幅度尚未在保存阶段重新验证。
