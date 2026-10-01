# macOS 原生 MachXO2 工具链安装

流程：Verilog → Yosys → nextpnr-machxo2 → ecppack → openFPGALoader。
无需 Diamond 或 Windows/Linux 虚拟机。MachXO2 后端在上游仍标为实验性；
不能把简单流水灯的验证结果外推到 PLL、DDR、EBR 等全部器件功能。

本机已经安装完成。日常编译、烧录见 [操作手册](README.md)；
本文用于换电脑或重新准备本地工具链。

## 已验证版本

- Yosys 0.58（Homebrew）。
- nextpnr：`1aea87ab50cd89ede712003ef9003acce534856d`。
- Project Trellis：`65fe191a290cd947c125f011e9e78a2ddc90d7a4`。
- Trellis 数据库：`015e0330630d7c238c0e4f2cdd9c8157eb78c54a`。
- openFPGALoader 1.0.0（Homebrew，成功烧录使用此版本）。
- Apple Clang 17、CMake 4.0.3、Python 3.13.5、Boost 1.89、Eigen 5.0。

源码、数据库、编译产物保存在 `lcmxo2/toolchain/`，供 `project/` 下各工程共用；
它们属于本地安装内容，换电脑时需重新准备。
nextpnr 启用 `1200;2000`；当前 MBI5264 板使用 `LCMXO2-2000HC-4TG144C`，
旧流水灯板使用 `LCMXO2-2000HC-4TG100C`。
器件列表参数必须加引号，避免分号被 shell 当作命令分隔符。

## 1. 安装系统依赖

以下命令均从项目根目录执行，新电脑需替换项目路径。
需要先安装 [Homebrew](https://brew.sh/) 和 Apple Command Line Tools；
缺少后者时执行 `xcode-select --install` 并完成安装。

```sh
cd /Users/bytedance/rust/vdrm2
brew install yosys cmake ninja boost eigen python@3.13 openfpgaloader
```

Homebrew 安装的是届时可用版本，不保证等于上面的历史版本。
下面固定 nextpnr、Trellis 和器件数据库的提交。

## 2. 获取源码

适用于尚无 `lcmxo2/toolchain/src/` 源码的环境。已有目录时不要重复 clone，
也不要直接覆盖其中的本地修改；本机现有环境可直接跳到第 5 步。

```sh
mkdir -p lcmxo2/toolchain/src
git clone https://github.com/YosysHQ/prjtrellis.git lcmxo2/toolchain/src/prjtrellis
git -C lcmxo2/toolchain/src/prjtrellis checkout 65fe191a290cd947c125f011e9e78a2ddc90d7a4
git -C lcmxo2/toolchain/src/prjtrellis submodule update --init --recursive
git -C lcmxo2/toolchain/src/prjtrellis/database checkout 015e0330630d7c238c0e4f2cdd9c8157eb78c54a
git clone https://github.com/YosysHQ/nextpnr.git lcmxo2/toolchain/src/nextpnr
git -C lcmxo2/toolchain/src/nextpnr checkout 1aea87ab50cd89ede712003ef9003acce534856d
git -C lcmxo2/toolchain/src/nextpnr submodule update --init --recursive
```

## 3. 编译并安装 Trellis

```sh
cmake -S lcmxo2/toolchain/src/prjtrellis/libtrellis -B lcmxo2/toolchain/build-trellis -G Ninja \
  -DCMAKE_INSTALL_PREFIX="$PWD/lcmxo2/toolchain/install" \
  -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE="$(brew --prefix python@3.13)/bin/python3.13" \
  -DBUILD_ECPBRAM=OFF -DBUILD_ECPPLL=OFF -DBUILD_ECPMULTI=OFF
cmake --build lcmxo2/toolchain/build-trellis --parallel 8
cmake --install lcmxo2/toolchain/build-trellis
```

## 4. 编译并安装 nextpnr-machxo2

```sh
cmake -S lcmxo2/toolchain/src/nextpnr -B lcmxo2/toolchain/build-nextpnr -G Ninja \
  -DARCH=machxo2 '-DMACHXO2_DEVICES=1200;2000' \
  -DTRELLIS_INSTALL_PREFIX="$PWD/lcmxo2/toolchain/install" \
  -DCMAKE_INSTALL_PREFIX="$PWD/lcmxo2/toolchain/install" \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON=OFF -DBUILD_GUI=OFF \
  -DPython3_EXECUTABLE="$(brew --prefix python@3.13)/bin/python3.13"
cmake --build lcmxo2/toolchain/build-nextpnr --parallel 8
cmake --install lcmxo2/toolchain/build-nextpnr
```

`pytrellis` 的编译和 nextpnr 数据库生成必须使用同一个 Python 版本。
即使设置 `BUILD_PYTHON=OFF`，构建器件数据库仍需要 Python。
若内存不足，可将 `--parallel 8` 改为 `--parallel 2`。

工具安装在项目目录，不需要手动修改系统 PATH；
各工程的 `build.sh` 会配置本次构建的 PATH。

## 5. 验证安装

```sh
yosys -V
openFPGALoader --version
bash lcmxo2/project/mbi5264/build.sh
```

此处使用当前 MBI5264 工程验证工具链。新工程放在
[`lcmxo2/project/<工程名>/`](project/README.md)，使用各自的构建入口。

构建成功后输出 `lcmxo2/project/mbi5264/build-open/led.bit`。
上板前按 [工程说明](project/mbi5264/README.md) 核对引脚复用和烧录接线。
此处验证不需要连接硬件。

常见安装问题：

- 找不到 `nextpnr-machxo2` 或 `ecppack`：检查上述 `cmake --install` 是否成功，
  可执行文件应位于 `lcmxo2/toolchain/install/bin/`。
- 找不到 `pytrellis`：核对两次 CMake 的 Python 路径一致，并确认先安装了 Trellis。
- 提示不支持 1200 或 2000 器件：核对 `-DARCH=machxo2 '-DMACHXO2_DEVICES=1200;2000'`。
- 移动项目后 CMake 报旧目录错误：CMake 缓存含绝对路径，使用新的构建目录重新配置，
  并将安装前缀指向新项目路径。

## 上游依据

- [nextpnr MachXO2 后端说明](https://github.com/YosysHQ/nextpnr/tree/main/machxo2)
- [上游完整构建示例](https://github.com/YosysHQ/nextpnr/blob/main/machxo2/examples/demo.sh)
- [2000 型号封装数据库](https://github.com/YosysHQ/prjtrellis-db/blob/master/MachXO2/LCMXO2-2000/iodb.json)
