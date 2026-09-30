# Debug Session: machxo2-jtag-id
- **Status**: [OPEN]
- **Issue**: 曾读到异常 IDCODE；最新重试已恢复正确 ID，并完成 FPGA Flash 烧录及回读校验。根因未确定，等待用户确认实际显示。
- **Debug Server**: 曾启动于 127.0.0.1:7777；用户中断后未重新启动
- **Log File**: .dbg/trae-debug-log-machxo2-jtag-id.ndjson

## Reproduction Steps
1. 使用当前默认 DirtyJTAG 引脚连接 Pico 与 MachXO2。
2. openFPGALoader -c dirtyJtag --freq 100000 --detect

## Hypotheses & Verification
| ID | Hypothesis | Likelihood | Effort | Expected evidence |
|---|---|---|---|---|
| A | 运行的固件与默认 UF2 或引脚不一致 | 中 | 低 | SWD 回读与默认 UF2、GPIO 功能配置不符 |
| B | Pico PIO 传输长度或状态切换异常 | 中 | 中 | 相同硬件接线下，改变传输实现后 IDCODE 恢复；PIO 寄存器或收发日志异常 |
| C | 板间 JTAG 信号或接线异常 | 中 | 中 | 不同实现下均异常，GPIO 或边界扫描读回暴露不一致 |

## Instrumentation
- A: SWD 固件字节校验、GPIO 功能寄存器。
- B: USB XFER/BYPASS/IDCODE 收发。
- C: 不同 TDI 数据与时钟频率下的 IDCODE。

## Log Evidence
- `.dbg/trae-debug-log-machxo2-jtag-id.ndjson` 保留此前异常 IDCODE 和 BYPASS 收发记录。
- 重刷上游默认 UF2 后回读校验通过，但随后仍出现异常 ID，不能认定恢复是重刷直接导致。
- 最新重试：连续 5 次 `0x012bb043`，退出码均为 0。
- `lcmxo2/build-open/program.log`：Enable configuration、Flash erase 成功，写入及回读校验均 100%，Refresh DONE；烧录进程退出码 0。

## Verification Conclusion
链路本次已恢复，已使用已校验码流完成内部 Flash 烧录。
未修改 PIO，未跳过 ID 校验；没有证据将根因归于 PIO、接线或特定工具版本。
等待用户确认新流水灯效果；确认前保留排查记录。
