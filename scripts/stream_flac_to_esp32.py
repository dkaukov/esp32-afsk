#!/usr/bin/env python3
import argparse
import os
import subprocess
import sys
import tempfile
import time

try:
    import serial
except ImportError:
    print("Missing pyserial. Install with: pip install pyserial", file=sys.stderr)
    sys.exit(1)


def run_ffmpeg_to_raw(flac_path: str, raw_path: str) -> None:
    cmd = [
        "ffmpeg",
        "-y",
        "-i",
        flac_path,
        "-f",
        "s16le",
        "-ac",
        "1",
        "-ar",
        "48000",
        raw_path,
    ]
    proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if proc.returncode != 0:
        sys.stderr.write(proc.stderr.decode("utf-8", errors="ignore"))
        raise RuntimeError("ffmpeg failed")


def stream_raw(ser, raw_path: str, num_samples: int) -> None:
    ser.write(f"DECODE {num_samples}\n".encode("ascii"))
    ser.flush()

    line = ser.readline().decode("utf-8", errors="ignore").strip()
    if line != "READY":
        raise RuntimeError(f"Unexpected response: {line}")

    with open(raw_path, "rb") as f:
        while True:
            chunk = f.read(16384)
            if not chunk:
                break
            ser.write(chunk)
    ser.flush()


def read_until_done(ser):
    while True:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if not line:
            continue
        print(line)
        if line.startswith("DONE "):
            return


def main():
    parser = argparse.ArgumentParser(description="Stream FLAC to ESP32 decoder test")
    parser.add_argument("flac", help="Path to input FLAC")
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/tty.usbserial-XXXX")
    parser.add_argument("--baud", type=int, default=921600)
    args = parser.parse_args()

    if not os.path.exists(args.flac):
        raise SystemExit(f"FLAC not found: {args.flac}")

    with tempfile.TemporaryDirectory() as tmpdir:
        raw_path = os.path.join(tmpdir, "audio.raw")
        run_ffmpeg_to_raw(args.flac, raw_path)
        size = os.path.getsize(raw_path)
        num_samples = size // 2

        with serial.Serial(args.port, args.baud, timeout=5) as ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            # Wait for ESP32 banner/READY
            time.sleep(0.5)
            while ser.in_waiting:
                print(ser.readline().decode("utf-8", errors="ignore").strip())

            stream_raw(ser, raw_path, num_samples)
            read_until_done(ser)


if __name__ == "__main__":
    main()
