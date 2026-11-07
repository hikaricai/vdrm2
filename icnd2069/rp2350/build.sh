#!/usr/bin/env bash
set -euo pipefail

# 在 icnd2069/rp2350 目录执行：bash build.sh
# 参数全部读取 config.json；修改后自动重新编译。
# 两种方式都显示同一斜线渐变；repeat_latch 仅控制锁存方式，顶层 gray 设置范围。
# 编译、烧录、运行并查看 RTT 日志；只编译请执行 cargo build --release。
cargo run --release
