# LCMXO2-1200HC TG100 全 IO 分组方波测试

目标器件：`LCMXO2-1200HC-4TG100C`，与新 ICND2069 板原理图型号一致。

工程使用芯片内部 `OSCH`，不需要外部晶振。`OSCH` 标称 2.08 MHz，
76 个被测 IO被分成 16 组轮流输出高电平。前 12 组各 5 个 IO，后
4 组各 4 个 IO；任一时刻只有一组为高。每个 IO输出约 1.016 kHz、
1/16 占空比脉冲。
内部振荡器有频率误差，实测频率不必精确等于 1.016 kHz。

## 安全要求

- 所有 VCCIO Bank必须供电 3.3 V。
- 烧录或上电前，断开 MCU、外设及其他可能驱动这些网络的器件。
- 所有输出使用 `LVCMOS33`、4 mA、慢沿。不要将其短接到电源或地。
- 示波器地夹接板上 GND，不要接到任意 FPGA IO。

## 被测封装脚

共 76 个，`board.lpf` 中按封装脚号升序映射为 `io[0]` 到 `io[75]`。
已核对本地 MachXO2-1200 pinout CSV 的 TQFP100 列，
不包含电源、地或 NC。JTAG 封装脚 90（TMS）、91（TCK）、
94（TDI）和 95（TDO）未配置为普通输出。
DONE、INITN、PROGRAMN 分别为 76、77、81 脚，
属于被测 IO，烧录脚本会释放其配置功能。

## 构建与烧录

从仓库根目录执行：

```sh
bash lcmxo2/project/1200hc100iotest/build.sh
bash lcmxo2/project/1200hc100iotest/flash.sh
```

码流为 `build-open/io_test.bit`。`flash.sh` 会重新构建、检测
`LCMXO2-1200HC` 的 IDCODE `0x012ba043`，然后写入内部 Flash并刷新配置。

本地 `openFPGALoader` 增加了 `OPENFPGALOADER_MACHXO2_FEABITS` 覆盖项。
本工程烧录时使用 `0x620`：保持 JTAG 启用，同时关闭 I2C、Slave SPI、
Master SPI、DONE、INITN 和 PROGRAMN 的 user-mode 持久占用，使对应复用脚
可作为普通输出。JTAG 四脚仍不可用作普通 IO。

若修改过下载器源码，先重新构建：

```sh
cmake --build lcmxo2/toolchain/build-loader -j4
```

## 构建验证

已完成 1200HC-4TG100C 的综合、布局布线和压缩码流生成，
内部时钟通过 2.08 MHz 目标；76 个 IO 约束逐项匹配 TQFP100 封装表。
时钟缓冲由 nextpnr 自动选择本器件的可用位置。本次未烧录。
