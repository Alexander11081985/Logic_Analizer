"""One-shot transport check for a flashed ESP32-S3 analyzer."""

from __future__ import annotations

import argparse
import time

import serial

from rx7500_protocol import PacketParser


def main() -> int:
    arguments = argparse.ArgumentParser()
    arguments.add_argument("port", help="ESP32-S3 serial port, for example COM14")
    arguments.add_argument("--timeout", type=float, default=4.0)
    options = arguments.parse_args()

    parser = PacketParser()
    captures = []
    with serial.Serial(options.port, 921600, timeout=0.1, write_timeout=1.0) as port:
        port.reset_input_buffer()
        port.write(b"R")
        port.flush()
        deadline = time.monotonic() + options.timeout
        while time.monotonic() < deadline and not captures:
            captures.extend(parser.feed(port.read(8192)))

    if not captures:
        print("NO_TRIGGER: transport opened, but no CLK rising edge was observed")
        return 2

    capture = captures[0]
    print(
        f"PACKET_OK capture={capture.number} samples={len(capture.samples)} "
        f"rate={capture.sample_rate_hz} crc_errors={parser.crc_errors}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
