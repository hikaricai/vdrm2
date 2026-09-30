# MachXO2 DDR x1：原语接法与当前工具链阻塞

核实日期：2026-10-01。目标：`LCMXO2-1200HC-4TG144C`。

## 结论

芯片支持专用 DDR x1 输入和输出寄存器，分别为 `IDDRXE`、`ODDRXE`。
当前安装的 Yosys / nextpnr-machxo2 尚不能完成这些原语的综合到码流流程。
已经用单个 ODDRXE 复现后端拒绝，不只是根据 README 推测。

`rtl/main.sv` 仍为已验证的旧行为：前八路高半周期直通、低半周期保持，
第九路始终直通。**尚未恢复 DDR 半周期延迟，也没有生成 DDR 码流。**

## 硬件与管脚

官方资料：
[Implementing High-Speed Interfaces with MachXO2 Devices，FPGA-TN-02153-1.9](https://www.latticesemi.com/view_document?document_id=39084)。

- 第 9 页表 2.1、第 10 页 §2.2：基本 PIO 支持 DDR x1，覆盖所有密度、四边 IO。
- 第 27 页 §5.4.2：ODDRXE 使用主时钟网络 SCLK。
- 第 60 页：IDDRXE 内含双沿采样级，以及向 FPGA 核心传递数据的同步级。
- 第 62 页：ODDRXE 可用于四边 IO；内部结构见第 10 页图 2.1。

当前 board.lpf 的 81 个输出，对应官方 CSV 中的：

| 边 | 输出数 |
| --- | ---: |
| PB（下） | 25 |
| PL（左） | 24 |
| PR（右） | 26 |
| PT（上） | 6 |

所以 DDR x1 不要求把所有输出挪到某一边；x2/x4/7:1 的边限制不能套用于 x1。
这些结论不代替完整设计在 Diamond 中的管脚和时序检查。
配置功能复用管脚仍受 [JTAGENB.md](JTAGENB.md) 中的限制。

现有 CLK（58 脚，PB15B）不是 CSV 标注的专用 PCLK 输入。
现有设计经 DCCA 使用主时钟网络，但不能据此假定所有官方 IDDR 输入接口
规则都已满足。下面的方案在输出端使用 ODDRXE，不直接套用 IDDRXE 输入接口。

## 半周期延迟的正确接法

这里将“恢复最早目标”解释为：每个边沿采集当前解码结果，在下一个相反边沿输出。
占空比为 50% 时延迟 T/2；其他占空比下，延迟为相邻边沿的实际间隔。

| 边沿 | 本次采样 | 边沿后输出 |
| --- | --- | --- |
| 上升沿 1 | A | 前一个下降沿采样值 |
| 下降沿 1 | B | A |
| 上升沿 2 | C | B |
| 下降沿 2 | D | C |

ODDRXE 的 D0、D1 都在 SCLK 上升沿锁存；D0 用于随后的高半周期，
D1 经内部下降沿寄存器用于低半周期。它不是每个边沿分别采样当前 D0/D1。
因此将 D0、D1 都直接接 decoded 会重复同一个上升沿样本，丢失下降沿样本。

可使用下面的结构替换每个 channel 原来的 held / pos / neg 选择逻辑：

```systemverilog
logic [8:0] sampled_fall = '0;

always_ff @(negedge main_clock)
    sampled_fall <= decoded;

for (genvar destination = 0; destination < 9; destination++) begin : outputs_ddr
    ODDRXE output_register (
        .SCLK(main_clock),
        .RST(1'b0),
        .D0(sampled_fall[destination]),
        .D1(decoded[destination]),
        .Q(routed[bank][channel][destination])
    );
end
```

上升沿：ODDRXE 输出前一个下降沿的 sampled_fall，同时内部锁存当前 decoded。
下降沿：ODDRXE 输出它在上升沿锁存的 decoded，外部 sampled_fall 采集新的 decoded。
全部九路统一处理，第九路也不再直通。输出切换使用专用输出 DDR 电路。

这是根据官方结构图推导的接法，**尚未经过厂商原语仿真或目标器件布局布线验证**。
后续必须验证初始状态、连续双沿数据、边沿间输出保持，以及地址锁存切换时的数据归属。
输入与地址需要满足采样沿的建立/保持时间；如果 MCU 与 FPGA 在同沿改变/采样数据，
必须根据实际相位调整采样时钟或 MCU 输出时序。

“当沿采样并立即输出当前值”与上表不同，不能把它称为从采样沿起延迟半周期。

## 实测的工具链阻塞

环境：Yosys 0.58；nextpnr 源码提交
`1aea87ab50cd89ede712003ef9003acce534856d`。

实验文件和完整日志在 `build-open/ddr-research/`（忽略目录）：

- `oddr_probe.sv`：一个 ODDRXE，使用已有 CLK 58 脚、DCCA DCC0、输出 60 脚。
- `oddr_blackbox.v`：仅声明 ODDRXE 端口，用于区分前端和后端支持情况。
- `probe.lpf`：四个端口的实际封装管脚约束。
- `yosys-native.log`、`yosys-blackbox.log`、`nextpnr.log`：失败与成功阶段的证据。

直接综合最小实例时，Yosys 报错：

```text
ERROR: Module `\ODDRXE' referenced in module `\oddr_probe'
in cell `\output_ddr' is not part of the design.
```

加入黑盒声明后，综合通过；但黑盒声明只描述接口，不提供硬件实现。
将该 JSON 交给 nextpnr，结果为：

```text
ERROR: cell type 'ODDRXE' is unsupported (instantiated as 'output_ddr')
```

本地源码核对：

- `../../toolchain/src/nextpnr/machxo2/README.md` 将所有 DDR 功能列为待实现。
  该列表部分项目已落后于实现，因此只将其作为线索。
- `machxo2/pack.cc` 没有 ODDRXE/IDDRXE 到 IOLOGIC 的打包实现。
- `machxo2/bitstream.cc` 没有 IOLOGIC DDR 配置写出实现。
- prjtrellis 已有基本 DDR 配置位的 fuzz 数据；有数据库不代表 nextpnr 已能使用。

## 后续落地

专用 DDR 路线需要使用支持 MachXO2 的 Lattice Diamond 后端
（在受支持的 Windows/Linux 环境运行），或者补齐 nextpnr 的 DDR 打包、
时序模型、布线约束和配置写出支持。

在此之前，不将上述草案接入默认 main.sv，也不把黑盒综合成功报告为 DDR 实现成功。
恢复后的全通道行为必须重新验证，旧版的等价证明和 24,640 次相位检查
只能证明旧功能，不能作为新 DDR 行为的验证结果。
