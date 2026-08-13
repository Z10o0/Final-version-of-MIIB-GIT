#!/usr/bin/env python3
import argparse
import os
import sys
import time
from pathlib import Path

try:
    import serial
except ImportError:
    print("ERROR: pyserial is not installed. Install with: pip install pyserial", file=sys.stderr)
    sys.exit(1)


def parse_args():
    p = argparse.ArgumentParser(description="High-speed RAW serial capture for MIIB stream")
    p.add_argument("--port", required=True, help="COM port, e.g. COM3")
    p.add_argument("--baud", type=int, default=12000000, help="Baud rate")
    p.add_argument("--duration", type=float, default=30.0, help="Capture duration in seconds")
    p.add_argument("--outfile", default="", help="Output .bin file path")
    p.add_argument("--frames-per-batch", type=int, default=16)
    p.add_argument("--frame-len", type=int, default=348)
    p.add_argument("--batches-per-sec", type=int, default=100)
    p.add_argument("--chunk", type=int, default=65536, help="Max bytes to read per iteration")
    p.add_argument("--poll", type=float, default=0.0005, help="Sleep when no data, seconds")
    return p.parse_args()


def main():
    args = parse_args()

    batch_bytes = args.frames_per_batch * args.frame_len
    expected_bps = batch_bytes * args.batches_per_sec

    if args.outfile:
        out_path = Path(args.outfile)
    else:
        ts = time.strftime("%Y%m%d_%H%M%S")
        out_path = Path(f"miib_raw_py_{ts}.bin")

    ser = serial.Serial()
    ser.port = args.port
    ser.baudrate = args.baud
    ser.bytesize = serial.EIGHTBITS
    ser.parity = serial.PARITY_NONE
    ser.stopbits = serial.STOPBITS_ONE
    ser.timeout = 0
    ser.write_timeout = 0
    ser.rtscts = False
    ser.dsrdtr = False
    ser.xonxoff = False

    ser.open()

    try:
        try:
            ser.set_buffer_size(rx_size=16 * 1024 * 1024, tx_size=64 * 1024)
        except Exception:
            pass

        ser.reset_input_buffer()

        total = 0
        max_waiting = 0
        max_chunk = 0
        last_total = 0
        t0 = time.perf_counter()
        t_last = t0

        with open(out_path, "wb", buffering=1024 * 1024) as f:
            print(f"[PY] COM: {args.port} @ {args.baud} baud")
            print(f"[PY] Expected: {expected_bps} bytes/s")
            print(f"[PY] Batch bytes: {batch_bytes}")
            print(f"[PY] Duration: {args.duration:.3f} s")
            print(f"[PY] Output: {out_path}")

            while True:
                now = time.perf_counter()
                elapsed = now - t0
                if elapsed >= args.duration:
                    break

                waiting = ser.in_waiting
                if waiting > 0:
                    max_waiting = max(max_waiting, waiting)
                    n = min(waiting, args.chunk)
                    data = ser.read(n)
                    if data:
                        f.write(data)
                        total += len(data)
                        if len(data) > max_chunk:
                            max_chunk = len(data)
                else:
                    time.sleep(args.poll)

                if now - t_last >= 1.0:
                    dt = now - t_last
                    dbytes = total - last_total
                    inst_kib = dbytes / dt / 1024.0
                    avg_kib = total / max(elapsed, 1e-9) / 1024.0
                    print(f"[PY] t={elapsed:6.2f}s | inst={inst_kib:8.1f} KiB/s | avg={avg_kib:8.1f} KiB/s | bytes={total}")
                    t_last = now
                    last_total = total

            time.sleep(0.05)
            while ser.in_waiting > 0:
                waiting = ser.in_waiting
                max_waiting = max(max_waiting, waiting)
                n = min(waiting, args.chunk)
                data = ser.read(n)
                if not data:
                    break
                f.write(data)
                total += len(data)
                if len(data) > max_chunk:
                    max_chunk = len(data)

            f.flush()
            os.fsync(f.fileno())

        elapsed = time.perf_counter() - t0
        bps = total / elapsed
        kib = bps / 1024.0
        ratio = bps / expected_bps if expected_bps > 0 else 0.0

        print(f"[PY] Done: {total} bytes in {elapsed:.3f} s = {kib:.1f} KiB/s ({ratio * 100:.2f}% expected)")
        print(f"[PY] max in_waiting: {max_waiting} | max chunk read: {max_chunk}")
        print(f"[PY] Raw saved: {out_path}")

    finally:
        try:
            ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
