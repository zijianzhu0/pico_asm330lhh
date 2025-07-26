#!/usr/bin/env python3
import argparse
import sys

try:
    import serial
except ImportError:
    sys.exit("Error: pyserial is required. Install with `pip install pyserial`.")

def is_valid_packet(hex_bytes: bytes) -> bool:
    """Return True if the last byte equals the XOR of all prior bytes."""
    checksum = 0
    for b in hex_bytes[:-1]:
        checksum ^= b
    return checksum == hex_bytes[-1]

def main():
    parser = argparse.ArgumentParser(
        description="Read 19-byte packets (starting with 0xAA) from a serial port"
    )
    parser.add_argument(
        "port",
        nargs="?",
        default="/dev/tty.usbserial-BG013RMQ",       # ← default port
        help="Serial port (e.g., COM3 or /dev/ttyUSB0)"
    )
    parser.add_argument(
        "baud",
        nargs="?",
        type=int,
        default=115200,               # ← default baud rate
        help="Baud rate (e.g., 115200)"
    )
    args = parser.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as e:
        sys.exit(f"Error opening serial port {args.port}: {e}")

    print(f"Listening on {args.port} at {args.baud} baud for 0xAA-start packets...")

    try:
        while True:
            # Read one byte at a time
            b = ser.read(1)
            if not b:
                continue

            # Found start byte?
            if b[0] == 0xAA:
                # Read the remaining 18 bytes
                rest = ser.read(18)
                if len(rest) == 18:
                    packet = b + rest
                    status = "OK" if is_valid_packet(packet) else "BAD"
                    # Print as hex, uppercase, no separators
                    print(packet.hex().upper() + ' ' + status)
                # else: timed out before full packet—just skip and continue
    except KeyboardInterrupt:
        print("\nInterrupted by user. Exiting.")
    finally:
        ser.close()

if __name__ == "__main__":
    main()