# LCMXO2-2000HC TG144 全 IO 分组方波测试

目标器件：`LCMXO2-2000HC-4TG144I`。

工程使用芯片内部 `OSCH`，不需要外部晶振。`OSCH` 标称 2.08 MHz，
108 个被测 IO被分成 16 组轮流输出高电平。前 12 组各 7 个 IO，后
4 组各 6 个 IO；任一时刻只有一组为高。每个 IO输出约 1.016 kHz、
1/16 占空比脉冲。
内部振荡器有频率误差，实测频率不必精确等于 1.016 kHz。

## 安全要求

- 所有 VCCIO Bank必须供电 3.3 V。
- 烧录或上电前，断开 MCU、外设及其他可能驱动这些网络的器件。
- 所有输出使用 `LVCMOS33`、4 mA、慢沿。不要将其短接到电源或地。
- 示波器地夹接板上 GND，不要接到任意 FPGA IO。

## 被测封装脚

共 108 个，`board.lpf` 中按 `io[0]` 到 `io[107]` 映射。JTAG封装脚
130、131、136和 137未配置为普通输出。

## 构建与烧录

从仓库根目录执行：

```sh
bash lcmxo2/project/2000hc144iotest/build.sh
bash lcmxo2/project/2000hc144iotest/flash.sh
```

码流为 `build-open/io_test.bit`。`flash.sh` 会重新构建、检测
`LCMXO2-2000HC` 的 IDCODE `0x012bb043`，然后写入内部 Flash并刷新配置。

本地 `openFPGALoader` 增加了 `OPENFPGALOADER_MACHXO2_FEABITS` 覆盖项。
本工程烧录时使用 `0x620`：保持 JTAG 启用，同时关闭 I2C、Slave SPI、
Master SPI、DONE、INITN 和 PROGRAMN 的 user-mode 持久占用，使对应复用脚
可作为普通输出。JTAG 四脚仍不可用作普通 IO。

若修改过下载器源码，先重新构建：

```sh
cmake --build lcmxo2/toolchain/build-loader -j4
```
