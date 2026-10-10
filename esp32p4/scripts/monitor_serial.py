"""Monitor the running firmware without resetting the ESP32-P4."""

import argparse
import sys

import serial

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--port", default="/dev/cu.usbmodem1101")
parser.add_argument("--baud", type=int, default=115200)
args = parser.parse_args()

print(
    f"Monitoring {args.port} at {args.baud} baud without reset; press Ctrl+C to exit.",
    file=sys.stderr,
)

try:
    with serial.Serial(args.port, args.baud, timeout=0.2) as port:
        while True:
            data = port.read(port.in_waiting or 1)
            if data:
                sys.stdout.buffer.write(data)
                sys.stdout.buffer.flush()
except KeyboardInterrupt:
    pass
