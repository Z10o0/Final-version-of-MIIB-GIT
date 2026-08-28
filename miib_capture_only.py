#!/usr/bin/env python3
"""
miib_capture_only.py — только захват UART в .bin + JSON с метаданными.
Вызывается из MATLAB через system(). Парсинг и графики делает MATLAB.

Usage:
  python miib_capture_only.py --port COM3 --baud 12000000 --duration 30 --outfile out.bin
"""
import argparse, ctypes, gc, os, sys, time, json, threading, queue
from pathlib import Path
from dataclasses import dataclass

try:
    import serial
except ImportError:
    sys.exit("ERROR: pip install pyserial")

IS_WINDOWS = (os.name == "nt")
_REALTIME_PRIORITY_CLASS = 0x00000100
_HIGH_PRIORITY_CLASS = 0x00000080
_THREAD_PRIORITY_TIME_CRITICAL = 15
_THREAD_PRIORITY_HIGHEST = 2


def set_process_max_priority():
    if IS_WINDOWS:
        kernel32 = ctypes.windll.kernel32
        h = kernel32.GetCurrentProcess()
        if not kernel32.SetPriorityClass(h, _REALTIME_PRIORITY_CLASS):
            kernel32.SetPriorityClass(h, _HIGH_PRIORITY_CLASS)
    else:
        try:
            os.nice(-20)
        except PermissionError:
            try:
                os.nice(-10)
            except Exception:
                pass


def set_current_thread_time_critical():
    if IS_WINDOWS:
        kernel32 = ctypes.windll.kernel32
        h = kernel32.GetCurrentThread()
        if not kernel32.SetThreadPriority(h, _THREAD_PRIORITY_TIME_CRITICAL):
            kernel32.SetThreadPriority(h, _THREAD_PRIORITY_HIGHEST)


def pin_current_thread_to_cpu(cpu_index):
    if IS_WINDOWS:
        try:
            kernel32 = ctypes.windll.kernel32
            h = kernel32.GetCurrentThread()
            kernel32.SetThreadAffinityMask(h, ctypes.c_size_t(1 << cpu_index))
        except Exception:
            pass


def try_max_serial_buffer(ser, rx_candidates=(256,128,64,32,16,8,4,2,1), tx_mb=1):
    if not hasattr(ser, "set_buffer_size"):
        return 0
    for mb in rx_candidates:
        try:
            ser.set_buffer_size(rx_size=mb*1024*1024, tx_size=tx_mb*1024*1024)
            return mb
        except Exception:
            continue
    return 0


@dataclass
class CaptureStats:
    total_bytes: int = 0
    read_calls: int = 0
    zero_reads: int = 0
    read_errors: int = 0
    queue_high_watermark: int = 0


class _SerialReaderThread(threading.Thread):
    def __init__(self, ser, out_queue, stop_event, chunk_size, stats, lock, pin_cpu=None):
        super().__init__(name="MIIB-SerialReader", daemon=True)
        self.ser, self.out_queue, self.stop_event = ser, out_queue, stop_event
        self.chunk_size, self.stats, self.lock, self.pin_cpu = chunk_size, stats, lock, pin_cpu

    def run(self):
        set_current_thread_time_critical()
        if self.pin_cpu is not None:
            pin_current_thread_to_cpu(self.pin_cpu)
        buf = bytearray(self.chunk_size)
        mv = memoryview(buf)
        while not self.stop_event.is_set():
            try:
                n = self.ser.readinto(mv)
            except Exception:
                with self.lock:
                    self.stats.read_errors += 1
                continue
            with self.lock:
                self.stats.read_calls += 1
                if not n:
                    self.stats.zero_reads += 1
                    continue
                self.stats.total_bytes += n
            self.out_queue.put(bytes(mv[:n]))
            with self.lock:
                q = self.out_queue.qsize()
                if q > self.stats.queue_high_watermark:
                    self.stats.queue_high_watermark = q


def do_capture(args):
    ts = time.strftime("%Y%m%d_%H%M%S")
    out_path = Path(args.outfile) if args.outfile else Path(f"miib_raw_{ts}.bin")

    set_process_max_priority()
    gc_was_enabled = gc.isenabled()
    gc.disable()

    ser = serial.Serial()
    ser.port, ser.baudrate = args.port, args.baud
    ser.bytesize, ser.parity, ser.stopbits = serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE
    ser.timeout = args.read_timeout
    ser.rtscts = ser.dsrdtr = ser.xonxoff = False
    ser.open()

    stats = CaptureStats()
    lock = threading.Lock()
    stop_event = threading.Event()
    out_queue = queue.Queue(maxsize=args.queue_max)

    elapsed = 0.0
    reader = None
    try:
        buf_mb = try_max_serial_buffer(ser)
        ser.reset_input_buffer()

        cpu_count = os.cpu_count() or 1
        reader_cpu = (cpu_count - 1) if cpu_count > 1 else None
        main_cpu = (cpu_count - 2) if cpu_count > 2 else (0 if cpu_count > 1 else None)
        if main_cpu is not None:
            pin_current_thread_to_cpu(main_cpu)

        print(f"[MIIB] Port {args.port} @ {args.baud} baud, duration {args.duration}s, buf={buf_mb}MB")

        reader = _SerialReaderThread(ser, out_queue, stop_event, args.chunk, stats, lock, pin_cpu=reader_cpu)
        t0 = time.perf_counter()
        reader.start()

        with open(out_path, "wb", buffering=1024*1024) as f:
            while True:
                now = time.perf_counter()
                if now - t0 >= args.duration:
                    break
                try:
                    f.write(out_queue.get(timeout=0.05))
                except queue.Empty:
                    pass
            stop_event.set()
            reader.join(timeout=2.0)
            while True:
                try:
                    f.write(out_queue.get_nowait())
                except queue.Empty:
                    break
            f.flush()
            os.fsync(f.fileno())

        elapsed = time.perf_counter() - t0
        print(f"[MIIB] Done: {stats.total_bytes} B in {elapsed:.3f}s "
              f"({stats.total_bytes/elapsed/1024:.1f} KiB/s)")
    finally:
        stop_event.set()
        if reader is not None:
            reader.join(timeout=2.0)
        try:
            ser.close()
        except Exception:
            pass
        if gc_was_enabled:
            gc.enable()

    meta = {
        "outfile": str(out_path.resolve()),
        "elapsed_sec": elapsed,
        "total_bytes": stats.total_bytes,
        "read_calls": stats.read_calls,
        "read_errors": stats.read_errors,
        "queue_high_watermark": stats.queue_high_watermark,
        "port": args.port,
        "baud": args.baud,
    }
    meta_path = out_path.with_suffix(".json")
    meta_path.write_text(json.dumps(meta, indent=2))
    print(f"[MIIB] META: {meta_path}")
    return out_path, elapsed


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", required=True)
    p.add_argument("--baud", type=int, default=12_000_000)
    p.add_argument("--duration", type=float, default=30.0)
    p.add_argument("--outfile", required=True)
    p.add_argument("--chunk", type=int, default=1_048_576)
    p.add_argument("--read-timeout", type=float, default=0.002, dest="read_timeout")
    p.add_argument("--queue-max", type=int, default=20000, dest="queue_max")
    args = p.parse_args()
    do_capture(args)


if __name__ == "__main__":
    main()