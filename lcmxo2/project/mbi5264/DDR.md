# 双沿当前值采样：语义修正与原生 DDR 限制

## 必须实现的行为

用户指定的行为参考为：

```systemverilog
always @(posedge main_clock or negedge main_clock)
    held <= decoded[7:0];
assign routed[bank][channel] = {decoded[8], held};
```

上升沿输出当时的 decoded，下降沿也输出当时的 decoded，边沿之间保持；
第九路直通。没有额外的半周期流水线延迟。
此处双沿 `always` 是行为参考，不能直接映射到 MachXO2 的普通触发器。

此前 ODDRXE 接法输出的是上一沿的样本，不符合需求；对应的仿真和码流
检查验证了错误的目标，不能作为本需求完成的依据。

## 当前 RTL

`main.sv` 使用两个相反边沿的编码寄存器及 XOR：

```systemverilog
always_ff @(posedge main_clock)
    encoded_pos <= decoded[7:0] ^ encoded_neg;
always_ff @(negedge main_clock)
    encoded_neg <= decoded[7:0] ^ encoded_pos;
assign held = encoded_pos ^ encoded_neg;
```

上升沿后，`held = (decoded ^ encoded_neg) ^ encoded_neg = decoded`；
下降沿后，`held = encoded_pos ^ (decoded ^ encoded_pos) = decoded`。
两个寄存器都初始化为零。每个边沿只更新其中一个寄存器，输出不需要
由时钟控制的选择器，也不需要 `late_clk`。
时钟暂停时保持，不依赖固定频率或 50% 占空比。

**这是普通逻辑 FF 实现，原生 DDR 要求尚未完成。**
交叉反馈路径必须满足相邻边沿间的建立保持时间。物理输出仍有 clock-to-Q
和 XOR 传播延迟，“当沿输出”不是零纳秒延迟的承诺。

## 原生原语为什么不能直接替换

依据 [FPGA-TN-02153-1.9](https://www.latticesemi.com/view_document?document_id=39084)
第 10 页图 2.1、第 60、62 页：

- ODDRXE 的 D0/D1 都在上升沿采集。D1 接当前 decoded 也无法在下降沿
  采集那个时刻的新数据，之前添加下降沿寄存器的接法则额外延迟了一个边沿。
- IDDRXE 双沿采集后还有上升沿同步级，不能直接提供所需的单路双沿当前值输出。
- 用 PLL 倍频后采样需要额外约束时钟连续运行、相位、占空比及锁定过程；
  不能直接视为可暂停、不等长半周期的双沿行为等价实现。

此前的 `tools/nextpnr-machxo2-oddr.patch`、`prepare-ddr.sh`、
`rtl/machxo2_ddr_bb.v`、`sim/oddrxe_model.v` 和 `tools/check_ddr.py`
仅保留作原生 DDR 调研材料，当前构建不使用它们。
旧的 `build-open/led-ddr.bit` 不符合本需求。

## 验证

```sh
bash lcmxo2/project/mbi5264/sim.sh
bash lcmxo2/project/mbi5264/sim-synth.sh
bash lcmxo2/project/mbi5264/build.sh
```

测试参考已改为每个边沿的**当前** decoded。覆盖全部 16 个地址、
512 种九位 RGB 输入、三组独立选址、两种时钟电平下锁存地址、
相邻边沿不同数据、不等长半周期、暂停时钟和第九路直通。
事件级检查覆盖边沿间保持及同沿多次翻转。

`build.sh` 使用常规 nextpnr，输出 `build-open/led-current-edge.bit`，
成功后复制为 `build-open/led.bit`。仿真不证明板级无毛刺；
外部输入建立保持和实际输出波形仍需验证。未执行硬件烧录。
