# ICND2069 / RP2350 点亮调试

当前版本按 `dsview/icnd2069/261024_100m_20ms.dsl` 原接收卡的 **40 扫、每帧 2560 个 ROW**、R 链配置和每帧配置轮转流程实现。连接硬件仍为单颗 ICND2069、RT5990、16×16 单色阵列；40 个逻辑扫描行用于复现参考协议，不代表板上有 40 个物理行。全部灰度字为原始 **255（0x00FF）**，不做满量程映射。

原始差异及历史故障见 [实测记录](../20261031_RP2350_DSView实测与调试记录.md) 和 [修改前对比报告](../20261031_原接收卡与RP2350波形对比.md)。发送正确不等于已经验证 LED 电流和实际图案正确。

## 接线

| GPIO | 目标信号 | 本次分析仪通道 |
|---|---|---|
| GP10 | ICND2069 LE | CH2 |
| GP13 | ICND2069 SIN | CH1 |
| GP15 | ICND2069 DCLK | CH0 |
| GP22 | ICND2069 ROW，旧网络名 GCLK | CH3 |
| GP16 | RT5990 LCK | 未采集 |
| GP18 | RT5990 BK | 未采集 |
| GP20 | RT5990 DIN | 未采集 |
| GP25 | 每 60 帧翻转板载 LED | 未采集 |
| — | ICND2069 OUT0 / OUT1 | CH4 / CH5 |

PIO0 仅使用 SM0；OUT 覆盖 GP10～22，但只有上述七个控制脚切换到 PIO。GP15 的 side-set 优先于 OUT。DIN 有效高、BK 高消隐、LCK 上升沿移位仍是板级假设，原接收卡 CH4/CH7 身份未确认，不能据此声称行管协议也已复现。

## 构建、运行与验证

在本目录执行：

```sh
cargo build --release --locked
cargo run --release --locked
```

runner 为 `probe-rs run --chip RP235x`。RTT 会打印 `reference flow`、`47-register rotation` 和 `pattern=uniform, raw_gray=255`。烧录/重启会改变硬件状态，先保存需要对比的波形。

无硬件检查实际 DMA 数据：

```sh
rustc -O --edition=2021 tools/check_waveform.rs -o /tmp/icnd2069-check
/tmp/icnd2069-check
```

检查 47 个轮转项及值、每帧命令顺序、每字 8 个重复副本、640 个灰度字、2560 个 ROW 与 12/4 宽度、81 时钟行周期、配置 patch 不改变行管输出，以及帧尾消隐。

当前 40 扫实物抓包检查（六路映射如上，100 MHz，至少 1 秒覆盖配置轮转）：

```sh
python3 tools/check_reference_capture.py capture.dsl capture.analysis.json
```

输出文件不得已存在。使用紧凑时钟数组，并独立检查全部 47 项寄存器轮转。历史工具 `analyze_capture.py` 的帧检查仍针对 16 扫，不用于判断当前 40 扫版本通过与否。

## 每帧流程

上电等待 100 ms，生成一帧固定波形。没有单独的 SOFT_RST、DIS_OP 或集中写完整配置表阶段；每帧发送：

```text
消隐并发送 40 个零选择位清空行移位状态
VSYNC → EN_OP → PRE_ACT
00=AA → 01=AA → 本帧轮转项 → 00=55 → 01=55
640 次灰度锁存（全部 00FF）
64 组 × 40 行 = 2560 个 ROW，覆盖约 16.59 ms
短暂帧尾消隐，随后下一帧
```

ROW 在 VSYNC 结束后第 25 个 DCLK 开始，与配置/灰度发送并行。EN_OP 在 VSYNC 后第 23 个 DCLK 结束；PRE_ACT 在第 40 个 DCLK 结束。首个 WR_CFG 在 PRE_ACT 后 137 DCLK 结束，之后每条配置/灰度槽固定为 136 DCLK。每次传 8 个相同的 16 位字，LE 仅覆盖最后一个字的末尾；单颗芯片最终保留最后一个字。

轮转顺序共 47 项：

```text
00=55 → 01=55 → 02 → 03 → … → 22
→ 70 → 71 → 72 → 73 → 74
→ F0 → F1 → F2 → F3 → F4 → F5 → 23
```

数值取原接收卡 R 链，包含 `02=27`（40 扫）、PLL `04=04,05=0A,06=01`、电流 `1C=72`。对 `70～74/F0～F5` 的写入是复现实际捕获行为，不代表其语义已公开。N 编码与内部频率没有通过 SDO 或内部时钟测量确认。

## DCLK 连续输出

一个 u32 装两个 16 位引脚样本，低半字先发：

```pio
.side_set 1
.wrap_target
pull noblock side 0
out pins, 16 side 0
nop          side 1 [1]
out pins, 16 side 0 [1]
nop          side 1 [1]
.wrap
```

8 个 PIO 周期输出两个 DCLK。默认 150 MHz 系统时钟下 divider=3，PIO 为 50 MHz，DCLK 为 12.5 MHz。X 预置两个 PARK 样本；空 FIFO 时 `pull noblock` 自动取 X，继续输出低 LE/ROW/LCK、消隐 BK 和 DCLK。关闭 autopull，避免 FIFO 空时因隐式拉取停住时钟。

DMA 完成后等待 FIFO/OSR 排空，才更新下一帧的配置槽，防止修改尚未发送的数据。大缓冲置于静态 BSS，不放在栈上。

## 对原包的适配边界

| 项目 | 当前实现 |
|---|---|
| DCLK | 12.5 MHz，含 CPU/DMA 间隙也连续 |
| 行周期 | 固定 81 DCLK=6.48 µs；原包为 80～83、主要 81 |
| ROW | 每帧 2560 个，首个 12 DCLK，其余 4 |
| 帧波形 | 208334 DCLK，理论约 16.66672 ms；软件更新/重启 DMA 增加消隐时间 |
| 帧缓冲 | 416668 字节 |
| 每帧灰度 | 40×16=640 字，每字发 8 个相同副本 |
| 扫描数 | 与原包一致的 40 扫，02=27 |
| 配置一轮 | 47 帧，约 0.784 s |

这不是逐采样重放：灰度图案、固定 136 字槽、固定 81 行周期、行移位清空、帧尾消隐及未确认的 RT5990 时序仍有区别。已撤销上一版自行缩减到 16 扫的适配，解决 1024 ROW 后约 10 ms 长空白的问题。实物 16 行选择映射和实际 LED 输出仍待验证。
