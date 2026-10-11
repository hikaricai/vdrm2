image="${1:?missing argument 1}"
# ~/.platformio/packages/tool-picotool-rp2040-earlephilhower/picotool load $image -o 0x10100000
probe-rs download $image --chip RP235x --base-address 0x10100000 --binary-format bin
probe-rs reset  --chip RP235x
