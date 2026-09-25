#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
screen=1
gamma=0.5
output="$script_dir/batchimgs.bin"
inputs=()

usage() {
    cat <<EOF
Usage: $(basename "$0") [options] <rgbd-image>...

Options:
  -s, --screen <0|1|2>  Select the screen output to merge (default: 1)
  -o, --output <path>   Output flashable image (default: $script_dir/batchimgs.bin)
      --gamma <value>   RGB gamma correction (default: 0.5)
  -h, --help            Show this help
EOF
}

while (($# > 0)); do
    case "$1" in
        -s|--screen)
            [[ $# -ge 2 ]] || { echo "missing value for $1" >&2; exit 2; }
            screen="$2"
            shift 2
            ;;
        -o|--output)
            [[ $# -ge 2 ]] || { echo "missing value for $1" >&2; exit 2; }
            output="$2"
            shift 2
            ;;
        --gamma)
            [[ $# -ge 2 ]] || { echo "missing value for $1" >&2; exit 2; }
            gamma="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        --)
            shift
            inputs+=("$@")
            break
            ;;
        -*)
            echo "unknown option: $1" >&2
            usage >&2
            exit 2
            ;;
        *)
            inputs+=("$1")
            shift
            ;;
    esac
done

[[ "$screen" =~ ^[012]$ ]] || { echo "screen must be 0, 1, or 2" >&2; exit 2; }
((${#inputs[@]} > 0)) || { usage >&2; exit 2; }

for input in "${inputs[@]}"; do
    [[ -f "$input" ]] || { echo "input does not exist: $input" >&2; exit 2; }
done

if [[ "$output" != /* ]]; then
    output="$PWD/$output"
fi

tmp_dir="$(mktemp -d "${TMPDIR:-/tmp}/vdrm-images.XXXXXX")"
trap 'rm -rf "$tmp_dir"' EXIT

cargo build \
    --manifest-path "$script_dir/Cargo.toml" \
    --release \
    -p img_buider \
    -p merge

builder="$script_dir/target/release/img_buider"
merger="$script_dir/target/release/merge"
encoded=()

for index in "${!inputs[@]}"; do
    image_dir="$tmp_dir/image-$index"
    mkdir -p "$image_dir"
    "$builder" \
        --input "${inputs[$index]}" \
        --out-dir "$image_dir" \
        --gamma "$gamma"

    shopt -s nullglob
    matches=("$image_dir"/img"${screen}"_*.bin)
    shopt -u nullglob
    ((${#matches[@]} == 1)) || {
        echo "expected one screen $screen image for ${inputs[$index]}, found ${#matches[@]}" >&2
        exit 1
    }
    encoded+=("${matches[0]}")
done

(
    cd "$tmp_dir"
    "$merger" "${encoded[@]}"
)

mkdir -p "$(dirname -- "$output")"
mv "$tmp_dir/batchimgs.bin" "$output"
echo "flashable image: $output"
