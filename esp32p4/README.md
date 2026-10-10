# ESP32-P4 Rust Hello World

这是一个运行在 WT9932P4-TINY 上的裸机 Rust (`no_std`) 示例。
程序通过 ESP32-P4 原生 USB Serial/JTAG 每秒输出一次 Hello World。

已在以下硬件上验证：

- ESP32-P4 revision v3.2
- 40 MHz 晶振
- 16 MB Flash
- macOS Apple Silicon

## 快速开始

进入项目目录：

```sh
cd /Users/bytedance/rust/vdrm2/esp32p4
```

首次使用时安装项目所需的 Rust 工具链、编译目标、`espflash` 和 Python
串口依赖：

```sh
sh scripts/setup.sh
```

连接开发板后查看串口设备：

```sh
ls /dev/cu.usbmodem*
```

当前开发板对应的设备是 `/dev/cu.usbmodem1101`。重新插拔或更换 USB
接口后，设备名可能改变。

编译、烧录并打开串口监视器：

```sh
ESPFLASH_PORT=/dev/cu.usbmodem1101 \
  cargo run --release --locked
```

看到以下内容表示固件运行成功：

```text
ESP32-P4 Rust Hello World started!
Hello, world! ESP32-P4 Rust demo count=0
Hello, world! ESP32-P4 Rust demo count=1
Hello, world! ESP32-P4 Rust demo count=2
```

按 `Ctrl+C` 退出监视器，不会停止开发板上的程序。

## 常用命令

只编译：

```sh
cargo build --release --locked
```

只烧录，不打开监视器：

```sh
.tools/bin/espflash flash \
  --port /dev/cu.usbmodem1101 \
  --chip esp32p4 \
  --flash-size 16mb \
  --min-chip-rev 3.0 \
  --non-interactive \
  --skip-update-check \
  target/riscv32imafc-unknown-none-elf/release/esp32p4-hello-world
```

只读取正在运行的固件输出，不复位芯片：

```sh
.venv/bin/python scripts/monitor_serial.py \
  --port /dev/cu.usbmodem1101
```

该脚本只是打开串口，不连接 ROM bootloader，也不发送复位序列。因此它和
RP2350 上的普通串口监视器行为一致，只能看到打开串口之后产生的输出。

需要复位芯片并查看完整启动日志时使用：

```sh
.tools/bin/espflash monitor \
  --port /dev/cu.usbmodem1101 \
  --skip-update-check
```

`espflash monitor` 不是纯串口终端。它会连接 ESP32-P4 的 ROM 下载协议、
识别芯片并运行 flasher stub，最后复位回用户固件后再读取串口，所以执行时
会发生一次复位。

## 板载 LED

原理图中有两颗 LED：

- `LED1`：5 V 可编程 RGB LED，数据脚连接 `IO51`。当前固件启动后发送
  全零帧并保持数据脚为低电平，因此它会熄灭。
- `LED2`：3.3 V 电源指示灯，连接为
  `3.3V -> R13 (1 kΩ) -> LED2 -> GND`，没有连接 GPIO。只要开发板上电
  就会常亮，无法通过软件关闭或调光。

需要降低 `LED2` 亮度时，可将 `R13` 从 1 kΩ 更换为 4.7 kΩ 或 10 kΩ；
需要完全关闭时，可拆除 `R13` 或 `LED2`。不改板的情况下可使用不透明胶带
遮光。不要降低 3.3 V 电源电压，因为该电源同时给 ESP32-P4 供电。

## 排查

串口被占用：

```text
Device or resource busy
```

先退出其他串口监视器。一个串口不能同时被多个程序打开。

找不到 `/dev/cu.usbmodem1101` 时，重新查询实际设备名：

```sh
ls /dev/cu.usbmodem*
```

烧录一直停在连接阶段时，按住开发板 `BOOT`，短按 `RESET`，松开
`BOOT` 后重新执行烧录命令。

## 项目配置

- Rust：1.98.0
- 编译目标：`riscv32imafc-unknown-none-elf`
- `esp-hal`：1.2.2
- `esp-println`：0.18.0
- `esp-bootloader-esp-idf`：0.6.0
- `espflash`：4.6.0，位于 `.tools/bin/espflash`

本项目不需要 Xtensa 工具链或完整 ESP-IDF SDK。依赖版本由
`Cargo.lock` 固定，Rust 版本由 `rust-toolchain.toml` 固定。
