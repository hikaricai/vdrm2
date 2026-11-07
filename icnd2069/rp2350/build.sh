#!/usr/bin/env bash
set -euo pipefail

# 在 icnd2069/rp2350 目录执行：bash build.sh
# 参数全部读取 config.json；修改后自动重新编译。
# pattern 只支持 diagonal（连续 latch）和 normal（逐像素完整写入 gray）。
# 编译、烧录、运行并查看 RTT 日志；只编译请执行 cargo build --release。
cargo run --release
