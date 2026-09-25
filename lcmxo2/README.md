# MachXO2 流水灯：编译与烧录操作手册

适用开发板：HSEDA LCMXO2-2000HC V1.1，芯片 `LCMXO2-2000HC-4TG100C`。
Mac 通过 RP2040/Pico（上游默认 DirtyJTAG V1.07）烧录 FPGA。
已跑通开源工具链和内部 Flash 写入、回读校验，无需 Lattice Diamond。

**本机环境已经安装好，日常直接执行下面的流程。**
换电脑或重装环境见 [工具链安装说明](OPEN_TOOLCHAIN.md)；
新 Pico 或恢复下载器固件见 [Pico 固件与接线说明](programmer/README.md)。

## 1. 进入项目根目录

本文和两个配套文档的终端命令均从项目根目录执行：

```sh
cd /Users/bytedance/rust/vdrm2
```

如果项目移动了，替换为新的根目录。不要再进入 `lcmxo2/` 子目录。

## 2. 连接硬件

当前可用接线保持不变。重新接线时，先将两板断电，按下表连接：

| Pico GPIO | 标准 Pico 物理脚号 | FPGA 板 J2 脚号 | JTAG 信号 | FPGA 封装脚号 |
| --- | --- | --- | --- | --- |
| GP16 | 21 | 5 | TDI，Pico → FPGA | 94 |
| GP17 | 22 | 7 | TDO，FPGA → Pico | 95 |
| GP18 | 24 | 1 | TCK | 91 |
| GP19 | 25 | 3 | TMS | 90 |
| GND | 23 或其他 GND | 4 或 8 | 共地 | — |

Pico 通过 USB 接 Mac，FPGA 板独立供电。J2 的 VCC 不接 Pico 电源，
GP20/21 不接线，不同时连接其他 JTAG 下载器。
物理脚号只适用于标准 Pico；其他 RP2040 板按 GPIO 名称接。

## 3. 编译流水灯

```sh
./lcmxo2/build_open.sh
```

脚本依次执行 Yosys 综合、nextpnr-machxo2 布局布线和 ecppack 打包，
自动设置本地工具路径。成功后末尾打印：

```text
Bitstream: /Users/bytedance/rust/vdrm2/lcmxo2/build-open/led_chaser.bit
```

输出文件为 `lcmxo2/build-open/led_chaser.bit`。构建报错时先解决错误，
不要拿目录里上一次留下的 `.bit` 当作本次结果烧录。

## 4. 检查 JTAG

```sh
openFPGALoader -c dirtyJtag --freq 100000 --detect
```

正常结果应包含：

```text
found 1 devices
index 0:
    idcode 0x12bb043
    manufacturer lattice
    family MachXO2
    model LCMXO2-2000HC
    irlength 8
```

`0x12bb043` 与 `0x012bb043` 是同一个 ID。`--detect` 不擦写 FPGA 配置。
此前链路曾出现不稳定，建议连续执行几次，确认都读到正确 ID 后再烧录；
最近成功烧录前连续检查了 5 次。

## 5. 写入 FPGA 内部 Flash 并校验

```sh
openFPGALoader -c dirtyJtag --freq 100000 -f --verify lcmxo2/build-open/led_chaser.bit
```

这会替换 FPGA 内部 Flash 中的程序。参数含义：

| 参数 | 含义 |
| --- | --- |
| `-c dirtyJtag` | 使用 Pico 上的 DirtyJTAG 下载器 |
| `--freq 100000` | JTAG 时钟 100 kHz，已验证可用 |
| `-f` | 写内部配置 Flash，断电后仍可加载 |
| `--verify` | 写入后回读校验 |

成功标准：写入进度 100%、校验进度 100%，配置刷新返回 `DONE`，
命令无错误退出。不能只看到写入 100% 就认定整个流程成功。

**`.bit` 必须带 `-f` 才写 Flash。** 省略时只配置 SRAM，
断电后恢复 Flash 中原有的程序。构建脚本已使用 `ecppack --compress`；
openFPGALoader 1.0.0 的 MachXO2 Flash 解析路径要求压缩码流。

## 6. 检查板上效果

预期 LED1 → LED2 → LED3 → LED4 循环，每约 250 ms 切换一次。
烧录并校验成功后，可重新给 FPGA 板上电，检查程序能否自动恢复。
LED5 是电源灯，LED6 是串口指示灯，不参与流水灯。

## 修改程序后怎么操作

修改 [led_chaser.v](led_chaser.v)，然后重新执行第 3～6 步。

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `LED_COUNT` | `4` | 本板四个 LED；改变数量时需同步修改引脚约束 |
| `ACTIVE_LOW` | `1` | 本板低电平点亮，保持为 1 |
| `STEP_CYCLES` | `520000` | 每次切换间隔的时钟周期数，至少为 1 |

使用内部 `OSCH`，标称频率 2.08 MHz：
`STEP_CYCLES ≈ 间隔秒数 × 2080000`。
例如 100 ms 为 `208000`，500 ms 为 `1040000`，1 秒为 `2080000`。
实际间隔随内部振荡器误差变化。若修改振荡器频率，还需同步修改
[board.lpf](board.lpf) 中的时钟约束。

管脚已依据 [原理图](LCMXO2-2000HCV11sch.pdf) 第 1 页配置：

| 顶层信号 | 板上 LED | FPGA 封装脚号 | FPGA 管脚名 |
| --- | --- | --- | --- |
| `led[0]` | LED1 | 58 | PR11A |
| `led[1]` | LED2 | 60 | PR10A |
| `led[2]` | LED3 | 61 | PR9A |
| `led[3]` | LED4 | 62 | PR7B |

LED 阳极经 RN1（1 kΩ）接 3.3 V，阴极接 FPGA；Bank 1 为 3.3 V，
约束使用 `LVCMOS33`。板载 50 MHz 时钟在 FPGA 63 脚，本程序不使用。

## 常见问题

| 现象 | 处理方法 |
| --- | --- |
| `Missing tool` 或 `command not found` | 按 [工具链安装说明](OPEN_TOOLCHAIN.md) 检查依赖和本地安装目录 |
| 找不到 DirtyJTAG USB 设备 | 检查 Pico USB 连接；正常 VID:PID 为 `1209:c0ca`。若出现 `RPI-RP2` 磁盘，按 [Pico 说明](programmer/README.md) 刷入默认 UF2 |
| ID 为 `0xfc006803`、`0xffffffc3` 或识别不到型号 | 暂停烧录，保持当前接线，重新插拔 Pico USB、给 FPGA 板断电重启，再以 100 kHz 执行 `--detect`；正确 ID 稳定后再烧录 |
| 重启后仍持续读错 ID | 核对是否使用默认 V1.07 UF2；需要时按 Pico 说明恢复。不要通过强制指定器件或反复擦写绕过错误 ID |
| 进入配置模式失败、写入或校验失败 | 该次烧录未成功。重启两板，重新检查 ID，再重试完整烧录命令 |
| 断电后恢复旧程序 | 确认烧录命令带 `-f`，且写入、校验及刷新均成功 |
| 编译成功但 LED 不符合预期 | 核对当前源码、`board.lpf` 和烧录文件路径，确认观察的是 LED1～4 |

此前确实出现过异常 ID，后来在默认固件和默认接线下恢复并烧录成功。
异常根因尚未确定，不能据此认定是接线、PIO 或特定工具版本导致。

## 文件位置与已验证记录

| 文件 | 用途 |
| --- | --- |
| `lcmxo2/led_chaser.v` | 流水灯源码 |
| `lcmxo2/board.lpf` | 引脚和时钟约束 |
| `lcmxo2/build_open.sh` | 编译脚本，只构建、不烧录 |
| `lcmxo2/build-open/led_chaser.bit` | 待烧录 FPGA 码流 |
| `lcmxo2/build-open/synthesis.log` | 综合日志 |
| `lcmxo2/build-open/nextpnr.log` | 布局布线日志 |
| `lcmxo2/build-open/program.log` | 本次已成功烧录的历史日志；上面的日常命令不会自动更新它 |
| `lcmxo2/programmer/pico-dirtyJtag-V1.07.uf2` | Pico 下载器固件，刷到 Pico，不是刷到 FPGA |

本次使用 Yosys 0.58、本地 nextpnr-machxo2 / Trellis、
openFPGALoader 1.0.0 和上游默认 DirtyJTAG V1.07，已完成：

- 综合、布局布线及 2.08 MHz 时序检查。
- 核对四路输出管脚与 LVCMOS33 电平约束。
- 反解码流，420 × 1272 个配置位与布局布线配置一致。
- 内部 Flash 写入和回读校验均达到 100%，配置刷新成功。

该次码流为 8,874 字节，SHA-256：
`bce6738d6aacf4ee21c9d0eacc43c5c742624744697626de3258b2da893be227`。
修改设计或工具版本后大小和哈希可能改变。板上视觉效果及断电保持未单独记录实测结果。

仓库保留了 `build_diamond.tcl` 作为备选，但尚未在 Diamond 中验证；
日常操作使用本文已验证的开源流程即可。
