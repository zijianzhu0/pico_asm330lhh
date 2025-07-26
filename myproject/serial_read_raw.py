#!/usr/bin/env python3
import argparse
import sys
import struct

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

def decode_data(data: bytes):
    """Unpack bytes 6-17 into three floats and print them to stdout."""
    # bytes 2-6: timestamp (uint32), bytes 6-18: three floats
    timestamp = struct.unpack('<I', data[2:6])[0]
    x, y, z = struct.unpack('<fff', data[6:18])
    print(f"Timestamp: {timestamp}, Data[0]: {x:.6f}, Data[1]: {y:.6f}, Data[2]: {z:.6f}")

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
    parser.add_argument(
        "--decode",
        action="store_true",
        help="Also decode and print Data[0]–Data[2] for each packet"
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
                    hex_str = packet.hex().upper()
                    status = "OK" if is_valid_packet(packet) else "BAD"
                    # Print as hex, uppercase, no separators
                    print(f"{hex_str} {status}")
                    if args.decode and status == "OK":
                        decode_data(packet)
                # else: timed out before full packet—just skip and continue
    except KeyboardInterrupt:
        print("\nInterrupted by user. Exiting.")
    finally:
        ser.close()

if __name__ == "__main__":
    main()