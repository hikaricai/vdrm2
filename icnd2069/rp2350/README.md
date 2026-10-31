# ICND2069 / RP2350：16 扫全通道 255 调试

当前分支 `icnd2069/debug16-uniform255`。目标是 16 扫、所有通道原始灰度 `255`，实测 OUT0/OUT1 在各逻辑行的响应与均匀图案一致。**目前尚未通过输出验证，不能把有跳变或输入正确当作完成。**

40 扫参考版本已单独提交到 `icnd2069/reference40-snapshot-20261031`，提交 `6f93362`，包括依赖锁文件。当前代码从此前 16 扫版本恢复后继续调试。恢复时生成 ELF 与备份逐字一致：SHA-256 `a79d01163547c0ab216cbbf5b33b4a2c405e4f44b6c50153ba975e8c205c0f2e`。

## 当前诊断状态

- 16×16 数据，每帧 256 次 DATA_LATCH，全部 `0x00FF`。
- 64 组×16 扫，每帧 1024 个 ROW，首个高宽 12 DCLK，其余 4 DCLK。
- DCLK 10 MHz，行周期 160 DCLK（16 µs），帧周期约 16.41 ms。
- PIO0/SM0 + DMA_CH0；`pull noblock` 在 FIFO 空时发送消隐样本，使 DCLK 连续。
- `src/main.rs` 的 `INITIALIZE_CHIP=false`：**跳过初始化、SOFT_RST、DIS_OP 和配置写入**。此状态用于对比 ICND2069 单独重上电行为，不是可交付的正常启动流程。芯片实际扫描配置未回读，发出 16 扫波形并不证明芯片内部也设置成 16 扫。
- `encode_init` 保留旧基线配置作为对照，当前没有发送；PLL `04/05/06=02/08/02`，`02=0F`、`03=3F`、`07/08=20`、`1C=40`。
- BK 恢复高电平消隐假设；DIN 一高位随 LCK 移动。RT5990 的实际极性、物理行选择仍需测量。

历史两路跳变发生在 ICND2069 单独补供电/重上电后；全 255 图案曾只有 OUT0 跳变，OUT1 低。SOFT_RST 不复位寄存器，MCU 重启不能替代 ICND2069 电源重启。本板 SDO 未接出，现有六路无法读回寄存器。

最新逐项实验、原始 DSL 和限制见 [调试记录](../20261031_RP2350_DSView实测与调试记录.md)。

## 接线

| GPIO | 信号 | 分析仪 |
|---|---|---|
| GP15 | ICND2069 DCLK | CH0 |
| GP13 | ICND2069 SIN | CH1 |
| GP10 | ICND2069 LE | CH2 |
| GP22 | ICND2069 ROW（旧网络名 GCLK） | CH3 |
| — | ICND2069 OUT0 | CH4 |
| — | ICND2069 OUT1 | CH5 |
| GP16 | RT5990 LCK | 未采集 |
| GP18 | RT5990 BK | 未采集 |
| GP20 | RT5990 DIN | 未采集 |
| GP25 | 每 60 帧翻转 LED | 未采集 |

## 构建与检查

在本目录运行：

```sh
cargo build --release --locked
cargo run --release --locked
rustc -O --edition=2021 tools/check_waveform.rs -o /tmp/icnd2069-check16
/tmp/icnd2069-check16
python3 tools/analyze_capture.py capture.dsl \
  --pattern uniform --gray 255 -o capture.analysis.json
```

RTT 必须确认 `16x16`、`pattern=uniform, raw_gray=255` 及 `initialize_chip` 状态。硬件原始抓包核对 256 个灰度字、1024 个 ROW、门限、采集时间；观察 OUT 的逐行响应和多帧稳定性。

`tools/check_reference_capture.py` 仅针对快照中的 40 扫参考流，不能用于当前 16 扫判定。当前 `check_waveform.rs` 检查实际 16 扫 DMA 数据；其中验证 `encode_init` 只说明编码正确，不表示当前运行时已发送初始化。

大波形缓冲在静态 BSS；DMA 完成后等待 FIFO/OSR 排空。输出端是恒流源，数字门限测得的电压边沿不能单独证明 LED 电流或亮度正确。
