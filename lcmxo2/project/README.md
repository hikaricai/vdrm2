# MachXO2 新工程目录

后续新工程统一放在 `lcmxo2/project/<工程名>/`，每个工程独立保存源码、
管脚约束、构建入口和说明文档。旧流水灯保留在 `lcmxo2/demo/`。

当前工程：[mbi5264](mbi5264/README.md)，三组 RGB × 九路独立选择，
目标 `LCMXO2-2000HC-4TG144C`，包含 RTL、完整 LPF、构建脚本和仿真。

推荐布局：

```text
lcmxo2/
├── project/
│   └── <工程名>/
│       ├── README.md       # 目标芯片、板卡、接口与编译烧录步骤
│       ├── rtl/            # Verilog 源码
│       ├── board.lpf       # 本工程管脚和时钟约束
│       ├── build.sh        # 本工程构建入口，只编译、不烧录
│       └── build-open/     # 生成文件与日志，不提交 Git
├── demo/                  # 旧流水灯参考工程
├── toolchain/             # 共用本地工具链
└── programmer/            # 共用 Pico 下载器固件
```

工程构建脚本应按脚本自身位置定位源码和 `lcmxo2/toolchain/install/bin/`，
将中间文件、码流和 `XDG_STATE_HOME` 写入本工程的 `build-open/`。
顶层模块、目标器件和 LPF 按各工程实际硬件配置，不能直接沿用旧板的管脚。

当前工程使用 `lcmxo2/project/mbi5264/build.sh` 构建。
现有本地 nextpnr 已启用 1200 和 2000 器件；
其他容量需先确认后端支持并重新构建相应器件数据库。

公共说明：[工具链安装](../OPEN_TOOLCHAIN.md)、
[Pico 固件与接线](../programmer/README.md)。
