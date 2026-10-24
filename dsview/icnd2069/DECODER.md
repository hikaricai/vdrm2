# ICND2069 DSL 解码器

独立 Rust 命令行程序，直接读取 DSView `.dsl`，解码 CLK、LAT 和 RGB 三路串行输入。无需安装 DSView 或先导出 CSV。要求 Rust 1.88 或更新版本。

## 运行

在本目录执行：

```bash
cargo run --release -- 261024_100m_20ms.dsl
```

在仓库根目录执行：

```bash
cargo run --release --manifest-path dsview/icnd2069/Cargo.toml -- \
  dsview/icnd2069/261024_100m_20ms.dsl
```

默认通道对应 `backgroud.md`：

| 参数 | 默认通道 | 信号 |
|---|---:|---|
| `--clk` | 0 | DCLK，上升沿采样 |
| `--lat` | 1 | LE/LAT，高电平内计数，下降沿结束命令 |
| `--red` / `--r` | 2 | 红色串行输入 |
| `--green` / `--g` | 5 | 绿色串行输入 |
| `--blue` / `--b` | 6 | 蓝色串行输入 |

通道参数是逻辑分析仪的实际 CH 编号，不是启用通道列表中的排序。CH3 的 ROW/OE 和 CH4/CH7 的 RT5990 信号不参与当前命令解码。

```bash
# 显式指定通道和级联长度
cargo run --release -- 261024_100m_20ms.dsl \
  --clk 0 --lat 1 --red 2 --green 5 --blue 6 --chips 8

# 保存可读文本
cargo run --release -- 261024_100m_20ms.dsl -o commands.txt

# 保存完整结构化数据及汇总
cargo run --release -- 261024_100m_20ms.dsl \
  --format jsonl -o commands.jsonl --summary-json summary.json

cargo run -- --help
```

输出默认到 stdout，简短统计到 stderr，可直接用管道。`-o` 和 `--summary-json` 只创建新文件；文件已存在时明确报错，避免覆盖原始抓包或旧结果。

## 解码规则

依据本目录编程指南 V4.2 的 PDF 第 5～9、18 页：

| LAT 高电平内 CLK 上升沿数 | 命令 | RGB 数据解释 |
|---:|---|---|
| 1 | DATA_LATCH | 16 位灰度值 |
| 3 | VSYNC | 帧同步；RGB 只作为原始移位历史显示 |
| 5 | WR_CFG | 高 8 位寄存器地址，低 8 位写入值 |
| 7 | RD_CFG | 低 8 位为待读取地址 |
| 9 | SOFT_RST | 软件复位；使行/OUT 推断失效，直到下次 VSYNC |
| 11 | EN_OP | 打开输出 |
| 12 | DIS_OP | 关闭输出 |
| 14 | PRE_ACT | 预激活 |
| 其他 | UNKNOWN | 原样保留计数及 RGB，附提示 |

只统计 CLK **上升沿**，并在上升沿按 **MSB first** 移入 RGB。LAT 高电平期间的数据位同样参与移位，因此在 LAT 下降沿处取最终数据。没有照搬 MBI5264 示例的双边沿计数。

RD_CFG 的响应位于 **SDO**，指南要求先等待 16 个 dummy DCLK，再取 16 位响应。当前只接收三路 SIN，故只能解出读请求地址，`value_hex` 为 `null`，不把输入数据当成读回值。

程序报告总线上观察到的写操作，不模拟 PRE_ACT/密码保护状态，也不保证芯片实际接受了每次写入。`0x72`、`0x73` 等地址保留原值，没有擅自命名寄存器功能。

补充核验：指南 PDF 第 19 页的开路检测回读流程规定 `0x72=待读取行号-1`，并给出 `0x73=0x42`（Mode A）、`0x73=0x82`（Mode B）。第 22 页列出两者默认值均为 `0x00`。因此本次 `0x72=0` 在该流程中表示选中第 1 行；`0x73=0` 的具体行为仍缺少完整定义，且本次未见 RD_CFG，不能据此认定发生了检测回读。

## 级联数据和输出字段

每条命令保留从上次 LAT 下降沿到本次下降沿之间的所有 CLK 上升沿数据；第一条从采样起点开始。窗口可能同时包含有效串行数据和空闲时钟。

- 未指定 `--chips`：从窗口末尾向前按 16 位分组，输出全部完整的**候选字**，开头不足 16 位的部分保存为 `prefix_bits`。不会凭总时钟数自动断言芯片数量。
- 指定 `--chips N`：对于 DATA_LATCH、WR_CFG、RD_CFG，仅把最后 N 个完整字作为候选数据，其余所有前导位仍保存为 `prefix_bits`，不会丢弃。
- 数组按**发送先后**排列。假设恰好 N 颗级联且传满 N 个字，第一个字最终在最远端芯片，最后一个字在最近端芯片；物理链路拓扑仍需实物确认。
- `tail16_hex` 是持续移位寄存器的最后 16 位，最近端芯片的数据最直接对应它。若整个采样尚不足 16 个 CLK 上升沿，该字段为 `null`。移位寄存器不会因为输出一条命令而清零。
- 文本中的 `[02B8*8]` 表示按顺序连续出现 8 个 `0x02B8`，属于无损重复压缩；JSONL 中保留完整数组。
- 控制命令也显示原始移位历史，但标为 `raw_shift_history_no_defined_payload`，没有把这些字解释为该控制命令的参数。

JSONL 每行一个对象，依次为：

1. `type=metadata`：采样数、采样率、DSL 版本、块数、通道数组 `[CLK,LAT,R,G,B]`、级联参数。
2. `type=command`：`event` 中包含每条命令及 RGB 三路数据。
3. `type=summary`：指令计数、配置写入计数、帧同步间隔等。

主要命令字段：

| 字段 | 含义 |
|---|---|
| `index` | 从 0 开始的命令/不完整脉冲序号 |
| `lat_start_sample` / `end_sample` | LAT 上升沿/下降沿采样位置 |
| `shift_start_sample` | 当前 RGB 移位窗口起点 |
| `time_us` | 从采样起点到命令结束的微秒时间，不是触发点相对时间 |
| `lat_clocks` | LAT 高电平内的 CLK 上升沿数 |
| `shift_clocks` | 整个移位窗口的 CLK 上升沿数 |
| `complete` | 是否看到了完整 LAT 高脉冲的两端 |
| `frame` | 最近一次 VSYNC 的全局序号；0 表示未同步或复位后未知 |
| `data_index` | 当前帧内 DATA_LATCH 序号，从 0 开始 |
| `line_1based` / `out` | 按每行 16 次锁存、OUT15→OUT0 顺序推断的位置 |
| `red` / `green` / `blue` | `prefix_bits`、`words_hex`、`tail16_hex`、`registers` |
| `warnings` | 不完整采样、未知命令、时序歧义、数据不足等 |

行号/OUT 是逻辑 SRAM 写入位置推断，不代表已还原 PCB 上的像素坐标、蛇形连线或 RT5990 扫描顺序。第一帧和末帧是否完整应结合 VSYNC 边界判断。

## 当前抓包实测

输入：`261024_100m_20ms.dsl`。100 MHz，2,000,896 个采样点，长度 **20.00896 ms**。

已保存：

- `261024_100m_20ms.decoded.txt`：完整可读结果。
- `261024_100m_20ms.decoded.jsonl`：完整结构化结果。
- `261024_100m_20ms.summary.json`：汇总。

上述结果使用默认通道，不预设级联数量。

| 命令 | 数量 |
|---|---:|
| DATA_LATCH | 861 |
| WR_CFG | 10 |
| VSYNC | 2 |
| EN_OP | 2 |
| PRE_ACT | 2 |
| 合计 | **877** |

没有不完整 LAT 脉冲、未知命令或解码告警。采样末尾仍有 **123 个尚未锁存的 CLK 上升沿**，汇总中单独记录，不伪造为完整命令。零告警不等于证明采样没有漏掉更窄的物理脉冲。

两个 VSYNC 的 LAT 下降沿位于第 **86,133**、**1,752,803** 个采样点，对应 **861.33 µs**、**17,528.03 µs**；间隔 **16,666.7 µs ≈ 60 Hz**。

两次 VSYNC 之间有 **640 次 DATA_LATCH**，按指南顺序对应 40 行 × 每芯片 16 个输出。第二次 VSYNC 后捕获到 **221 次 DATA_LATCH**，采样在该帧传输过程中结束；不能据此把第二帧解释为只有 221 个数据字。

第一组配置序列：

```text
VSYNC → EN_OP → PRE_ACT
WR_CFG 00=AA
WR_CFG 01=AA
WR_CFG 72=00
WR_CFG 00=55
WR_CFG 01=55
DATA_LATCH ...
```

第二组同样解锁、锁定，中间写入改为 `73=00`。这些配置写入在 R/G/B 三路一致，每个窗口都包含 **8 个重复配置字**。

871 个灰度/配置命令窗口中，817 个含 136 个 CLK 上升沿，54 个含 137 个；右对齐后均得到 **8 × 16 bit + 8/9 个前导位**。这支持“每路 8 颗级联”的判断，但前导时钟不能被算作第 9 颗芯片。本次结果保留前导位供核查。

## 文件格式与边界

- 支持当前抓包所用的 **DSView DSL version 3、logic mode**，ZIP 内 `header` 与 `L-<实际CH编号>/<块号>`。
- 支持多块，按数值块号读取并在块边界连续解码；校验通道长度、缺失块、采样率等。
- DSView 文件内每个字节按 bit0→bit7 表示递增采样时间；这与 ICND 协议字 MSB first 是两层不同的位序。
- 只处理 `total samples` 范围内数据；末字节/64 位对齐填充不作为真实采样。
- 旧版 DSL、模拟/示波器采样明确报错。当前实现将五路所选位平面读入内存，另暂存最长命令间隔的移位历史；超大采样需要相应内存。
- 采样开始或结束落在 LAT 高电平内部时输出 `INCOMPLETE`，不会把残缺的高脉冲命名为正常命令。
- CLK 上升沿与 LAT 边沿出现在同一采样点时给出告警。LAT 下降沿先结束上一条命令，该点 LE=0 的 CLK 归入下一窗口；LAT 上升沿同点的 CLK 按当前高电平计数。真实亚采样先后关系需要更高采样率确认。

DSL 格式与位平面顺序核对自 DreamSourceLab/DSView 的 `DSView/pv/storesession.cpp`、`DSView/pv/data/logicsnapshot.cpp`、`libsigrok4DSL/session_driver.c`。

## 验证记录

2026-10-24，在 Rust 1.98 上完成：

- `cargo build --release`。
- `cargo clippy --all-targets -- -D warnings`。
- 100 条合成命令验证：覆盖全部 8 类正常命令及未知计数，RGB 独立随机数据、位序、读写地址、前导位、超过 10 个乱序存放的数据块。
- 边界验证：级联长度不足、采样头尾不完整 LAT、短移位窗口、同采样点边沿、恒定时钟、SOFT_RST 后重新同步。
- 错误输入验证：缺失/截断通道、非法版本/采样率、重复/不存在的通道、级联数为零、输出路径冲突、阻止覆盖输入文件。
- 独立 Python 参考实现解包原始位平面，逐条核对真实采样的 **877 条命令时间戳、LAT 计数和全部 RGB 原始位**，结果一致。

验证夹具、脚本和构建缓存位于系统临时目录，没有混入解码结果。仓库原有 `.gitignore` 忽略整个 `/dsview/`；如需提交本程序，需自行调整忽略规则或显式添加文件。
