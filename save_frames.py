"""
EPC901 Frame Capture — triggered mode.

Flow:
  1. Press Enter (or set --auto) to capture a row
  2. Python sends 0x54 ('T') over serial to receiver
  3. Receiver forwards as BLE write to transmitter CMD characteristic
  4. Transmitter captures one frame and sends it back as BLE notifications
  5. Python receives framed binary packets, reassembles, unpacks, saves

UART frame format from receiver:
  [0xAA] [0x55] [len_lo] [len_hi] [data...]

Usage:
  python3 save_frames.py
  python3 save_frames.py --auto --rows 100 --interval 0.5
"""

import serial
import numpy as np
import os
import shutil
import argparse
import time

# --- Config ---
PORT             = '/dev/tty.usbmodem0010507939193'  # receiver port
BAUD             = 115200
PIXELS_PER_FRAME = 1024
BYTES_PER_FRAME  = PIXELS_PER_FRAME * 10 // 8        # 1280 bytes
OUTPUT_DIR       = 'frames'

# Clear frames folder at start of every run
if os.path.exists(OUTPUT_DIR) and os.listdir(OUTPUT_DIR):
    confirm = input(f"Clear {len(os.listdir(OUTPUT_DIR))} existing frames? (y/n): ")
    if confirm.lower() == 'y':
        shutil.rmtree(OUTPUT_DIR)
os.makedirs(OUTPUT_DIR, exist_ok=True)


def unpack_10bit(packed: bytes) -> np.ndarray:
    """Unpack 10-bit packed pixel data into uint16 array."""
    pixels = []
    for i in range(0, len(packed) - 4, 5):
        b0, b1, b2, b3, b4 = packed[i:i+5]
        pixels.append( b0        | ((b1 & 0x03) << 8))
        pixels.append(((b1 >> 2) | ((b2 & 0x0F) << 6)) & 0x3FF)
        pixels.append(((b2 >> 4) | ((b3 & 0x3F) << 4)) & 0x3FF)
        pixels.append(((b3 >> 6) | ( b4          << 2)) & 0x3FF)
    return np.array(pixels[:PIXELS_PER_FRAME], dtype=np.uint16)


def read_packet(ser: serial.Serial, timeout_s: float = 5.0) -> bytes | None:
    """
    Scan stream for 0xAA 0x55 sync marker, read length-prefixed payload.
    Returns payload bytes or None on timeout/error.
    """
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        byte = ser.read(1)
        if not byte:
            continue
        if byte == b'\xAA':
            if ser.read(1) == b'\x55':
                length_bytes = ser.read(2)
                if len(length_bytes) < 2:
                    continue
                length = int.from_bytes(length_bytes, 'little')
                if 1 <= length <= 244:
                    payload = ser.read(length)
                    if len(payload) == length:
                        return payload
    return None


def collect_frame(ser: serial.Serial) -> np.ndarray | None:
    """Collect packets until a full frame (1280 bytes) is assembled."""
    frame_buffer = bytearray()
    while len(frame_buffer) < BYTES_PER_FRAME:
        packet = read_packet(ser)
        if packet is None:
            print("  [timeout waiting for packet]")
            return None
        frame_buffer.extend(packet)

    return unpack_10bit(bytes(frame_buffer[:BYTES_PER_FRAME]))


def trigger_capture(ser: serial.Serial):
    """Send trigger byte to receiver."""
    ser.write(b'\x54')  # 'T'
    ser.flush()


def save_frame(pixels: np.ndarray, row_num: int) -> str:
    path = os.path.join(OUTPUT_DIR, f"frame_{row_num:05d}.npy")
    np.save(path, pixels)
    return path


def main():
    parser = argparse.ArgumentParser(description='EPC901 frame capture')
    parser.add_argument('--auto',     action='store_true',
                        help='Auto-trigger at fixed interval')
    parser.add_argument('--rows',     type=int,   default=100,
                        help='Number of rows to capture in auto mode (default 100)')
    parser.add_argument('--interval', type=float, default=0.5,
                        help='Seconds between auto triggers (default 0.5)')
    parser.add_argument('--port',     type=str,   default=PORT,
                        help='Serial port')
    args = parser.parse_args()

    ser = serial.Serial(args.port, BAUD, rtscts=False, dsrdtr=False, timeout=1)
    print(f"Connected to {args.port}")
    print(f"Saving to ./{OUTPUT_DIR}/")
    print(f"Mode: {'AUTO (' + str(args.rows) + ' rows @ ' + str(args.interval) + 's)' if args.auto else 'MANUAL (press Enter per row)'}\n")

    row_num = 0

    if args.auto:
        for row_num in range(args.rows):
            print(f"Triggering row {row_num}...", end=' ', flush=True)
            trigger_capture(ser)

            pixels = collect_frame(ser)
            if pixels is None:
                print(f"FAILED — skipping row {row_num}")
                continue

            path = save_frame(pixels, row_num)
            print(f"min={pixels.min():4d}  max={pixels.max():4d}  "
                  f"mean={pixels.mean():6.1f}  → {path}")

            time.sleep(args.interval)

        print(f"\nDone. {args.rows} rows captured.")

    else:
        # Manual mode — press Enter to trigger each row
        print("Press Enter to capture each row. Ctrl+C to stop.\n")
        while True:
            try:
                input(f"Row {row_num} — press Enter to capture...")
            except KeyboardInterrupt:
                print(f"\nStopped. {row_num} rows saved.")
                break

            trigger_capture(ser)
            print(f"  Capturing...", end=' ', flush=True)

            pixels = collect_frame(ser)
            if pixels is None:
                print("FAILED — try again")
                continue

            path = save_frame(pixels, row_num)
            print(f"min={pixels.min():4d}  max={pixels.max():4d}  "
                  f"mean={pixels.mean():6.1f}  → {path}")
            row_num += 1


if __name__ == '__main__':
    main()