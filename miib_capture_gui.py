#!/usr/bin/env python3
"""
miib_capture_gui.py — MIIB CAPTURE-ONLY tool с графическим окном выбора
COM-порта и длительности записи. Предназначен для сборки в один .exe
через PyInstaller и запуска "самостоятельно" (без консоли Python).

Логика захвата (MAX-PERF reader-поток, приоритеты, буфер драйвера,
GC disable) идентична miib_capture_only.py — просто обёрнута в Tkinter GUI.

Install (для сборки):
  pip install pyserial numpy pyinstaller
  (опционально: pip install psutil)

Собрать в exe (см. подробности и .bat в конце ответа):
  pyinstaller --onefile --noconsole --uac-admin --name MIIB_Capture miib_capture_gui.py
"""

import ctypes
import gc
import os
import sys
import time
import threading
import queue
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from pathlib import Path
from dataclasses import dataclass

try:
    import serial
    import serial.tools.list_ports as list_ports
except ImportError:
    serial = None
    list_ports = None


# =============================================================================
#  MAX-PERF: process/thread priority, CPU affinity, GC control
# =============================================================================
IS_WINDOWS = (os.name == "nt")
_REALTIME_PRIORITY_CLASS = 0x00000100
_HIGH_PRIORITY_CLASS = 0x00000080
_THREAD_PRIORITY_TIME_CRITICAL = 15
_THREAD_PRIORITY_HIGHEST = 2


def set_process_max_priority(log_fn=print):
    if IS_WINDOWS:
        kernel32 = ctypes.windll.kernel32
        handle = kernel32.GetCurrentProcess()
        ok = kernel32.SetPriorityClass(handle, _REALTIME_PRIORITY_CLASS)
        if ok:
            log_fn("[MIIB] Priority: process = REALTIME_PRIORITY_CLASS")
        else:
            kernel32.SetPriorityClass(handle, _HIGH_PRIORITY_CLASS)
            log_fn("[MIIB] WARN: REALTIME отклонён -> HIGH_PRIORITY_CLASS "
                   "(запусти exe от администратора для полного REALTIME)")
    else:
        try:
            os.nice(-20)
        except Exception:
            pass


def set_current_thread_time_critical():
    if IS_WINDOWS:
        kernel32 = ctypes.windll.kernel32
        handle = kernel32.GetCurrentThread()
        if not kernel32.SetThreadPriority(handle, _THREAD_PRIORITY_TIME_CRITICAL):
            kernel32.SetThreadPriority(handle, _THREAD_PRIORITY_HIGHEST)


def pin_current_thread_to_cpu(cpu_index):
    if IS_WINDOWS:
        try:
            kernel32 = ctypes.windll.kernel32
            handle = kernel32.GetCurrentThread()
            kernel32.SetThreadAffinityMask(handle, ctypes.c_size_t(1 << cpu_index))
        except Exception:
            pass


def try_max_serial_buffer(ser, rx_candidates=(256, 128, 64, 32, 16, 8, 4, 2, 1), tx_mb=1):
    if not hasattr(ser, "set_buffer_size"):
        return 0
    for mb in rx_candidates:
        try:
            ser.set_buffer_size(rx_size=mb * 1024 * 1024, tx_size=tx_mb * 1024 * 1024)
            return mb
        except Exception:
            continue
    return 0


# =============================================================================
#  Протокол (информационно)
# =============================================================================
FRAME_LEN = 690
N_SENSORS = 36
FRAMES_PER_BATCH = 16
BATCHES_PER_SEC = 100
NOMINAL_FPS = FRAMES_PER_BATCH * BATCHES_PER_SEC
EXPECTED_BPS = NOMINAL_FPS * FRAME_LEN


@dataclass
class CaptureStats:
    total_bytes: int = 0
    read_calls: int = 0
    zero_reads: int = 0
    max_single_read: int = 0
    read_errors: int = 0
    queue_high_watermark: int = 0
    start_perf: float = 0.0
    end_perf: float = 0.0


class _SerialReaderThread(threading.Thread):
    def __init__(self, ser, out_queue, stop_event, chunk_size, stats, stats_lock, pin_cpu=None):
        super().__init__(name="MIIB-SerialReader", daemon=True)
        self.ser = ser
        self.out_queue = out_queue
        self.stop_event = stop_event
        self.chunk_size = chunk_size
        self.stats = stats
        self.stats_lock = stats_lock
        self.pin_cpu = pin_cpu

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
                with self.stats_lock:
                    self.stats.read_errors += 1
                continue
            with self.stats_lock:
                self.stats.read_calls += 1
                if not n:
                    self.stats.zero_reads += 1
                    continue
                self.stats.total_bytes += n
                if n > self.stats.max_single_read:
                    self.stats.max_single_read = n
            self.out_queue.put(bytes(mv[:n]))
            qsize = self.out_queue.qsize()
            with self.stats_lock:
                if qsize > self.stats.queue_high_watermark:
                    self.stats.queue_high_watermark = qsize


def do_capture(port, baud, duration, outfile, chunk=1_048_576,
                read_timeout=0.002, queue_max=20000,
                log_fn=print, stop_flag=None):
    """stop_flag: threading.Event() для досрочной остановки из GUI (кнопка Stop)."""
    if serial is None:
        raise RuntimeError("pyserial не установлен")

    ts = time.strftime("%Y%m%d_%H%M%S")
    out_path = Path(outfile) if outfile else Path(f"miib_raw_{ts}.bin")

    set_process_max_priority(log_fn)
    gc_was_enabled = gc.isenabled()
    gc.disable()

    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.bytesize = serial.EIGHTBITS
    ser.parity = serial.PARITY_NONE
    ser.stopbits = serial.STOPBITS_ONE
    ser.timeout = read_timeout
    ser.rtscts = ser.dsrdtr = ser.xonxoff = False
    ser.open()

    stats = CaptureStats()
    stats_lock = threading.Lock()
    stop_event = threading.Event()
    out_queue = queue.Queue(maxsize=queue_max)
    reader = None
    drained = 0
    elapsed = 0.0

    try:
        buf_mb = try_max_serial_buffer(ser)
        if buf_mb:
            log_fn(f"[MIIB] Driver RX buffer set to {buf_mb} MiB")
        else:
            log_fn("[MIIB] WARN: буфер драйвера по умолчанию")
        ser.reset_input_buffer()

        cpu_count = os.cpu_count() or 1
        reader_cpu = (cpu_count - 1) if cpu_count > 1 else None
        main_cpu = (cpu_count - 2) if cpu_count > 2 else (0 if cpu_count > 1 else None)
        if main_cpu is not None:
            pin_current_thread_to_cpu(main_cpu)

        log_fn(f"[MIIB] Port    : {port} @ {baud} baud")
        log_fn(f"[MIIB] Duration: {duration:.1f} s  ->  Output: {out_path}")
        log_fn(f"[MIIB] Nominal : {NOMINAL_FPS} fps -> {EXPECTED_BPS/1024:.1f} KiB/s")

        reader = _SerialReaderThread(ser, out_queue, stop_event, chunk,
                                      stats, stats_lock, pin_cpu=reader_cpu)
        stats.start_perf = time.perf_counter()
        reader.start()

        t0 = t_last = time.perf_counter()
        last_total = 0

        with open(out_path, "wb", buffering=1024 * 1024) as f:
            while True:
                now = time.perf_counter()
                if now - t0 >= duration:
                    break
                if stop_flag is not None and stop_flag.is_set():
                    log_fn("[MIIB] Остановлено пользователем")
                    break
                try:
                    chunk_data = out_queue.get(timeout=0.05)
                    f.write(chunk_data)
                except queue.Empty:
                    pass

                if now - t_last >= 1.0:
                    with stats_lock:
                        total = stats.total_bytes
                    el = now - t0
                    dt = now - t_last
                    log_fn(f"[MIIB] t={el:6.2f}s | avg={total/max(el,1e-9)/1024:8.1f} KiB/s | "
                           f"total={total:10d} B | q_hwm={stats.queue_high_watermark} | "
                           f"rd_err={stats.read_errors}")
                    t_last, last_total = now, total

            stop_event.set()
            reader.join(timeout=2.0)

            while True:
                try:
                    chunk_data = out_queue.get_nowait()
                    f.write(chunk_data)
                    drained += len(chunk_data)
                except queue.Empty:
                    break
            f.flush()
            os.fsync(f.fileno())

        stats.end_perf = time.perf_counter()
        elapsed = stats.end_perf - t0
        with stats_lock:
            total = stats.total_bytes
        bps = total / max(elapsed, 1e-9)
        raw_pct = bps / EXPECTED_BPS * 100.0
        log_fn(f"[MIIB] Done    : {total} B in {elapsed:.3f} s = {bps/1024:.1f} KiB/s "
               f"({raw_pct:.2f}% of nominal)")
        log_fn(f"[MIIB] Saved   : {out_path.resolve()}")
        if raw_pct < 95.0:
            log_fn(f"[MIIB] !!! RAW throughput {raw_pct:.1f}% < 95% — возможна потеря данных")
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
            gc.collect()

    return out_path, elapsed, stats


# =============================================================================
#  GUI (Tkinter — входит в стандартную библиотеку, не требует доп. зависимостей)
# =============================================================================
class MiibGui(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("MIIB Capture")
        self.geometry("640x480")
        self.resizable(True, True)

        self.stop_flag = threading.Event()
        self.worker = None

        pad = dict(padx=6, pady=4)

        frm = ttk.Frame(self)
        frm.pack(fill="x", **pad)

        ttk.Label(frm, text="COM-порт:").grid(row=0, column=0, sticky="w")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(frm, textvariable=self.port_var, width=15, state="readonly")
        self.port_combo.grid(row=0, column=1, sticky="w")
        ttk.Button(frm, text="Обновить", command=self.refresh_ports).grid(row=0, column=2, padx=4)

        ttk.Label(frm, text="Baudrate:").grid(row=1, column=0, sticky="w")
        self.baud_var = tk.StringVar(value="12000000")
        ttk.Entry(frm, textvariable=self.baud_var, width=15).grid(row=1, column=1, sticky="w")

        ttk.Label(frm, text="Длительность (с):").grid(row=2, column=0, sticky="w")
        self.duration_var = tk.StringVar(value="30")
        ttk.Entry(frm, textvariable=self.duration_var, width=15).grid(row=2, column=1, sticky="w")

        ttk.Label(frm, text="Файл .bin:").grid(row=3, column=0, sticky="w")
        self.outfile_var = tk.StringVar(value="")
        ttk.Entry(frm, textvariable=self.outfile_var, width=40).grid(row=3, column=1, columnspan=2, sticky="w")
        ttk.Button(frm, text="Обзор...", command=self.choose_file).grid(row=3, column=3, padx=4)

        btn_frm = ttk.Frame(self)
        btn_frm.pack(fill="x", **pad)
        self.start_btn = ttk.Button(btn_frm, text="Начать запись", command=self.start_capture)
        self.start_btn.pack(side="left", padx=4)
        self.stop_btn = ttk.Button(btn_frm, text="Стоп", command=self.stop_capture, state="disabled")
        self.stop_btn.pack(side="left", padx=4)

        self.log_text = tk.Text(self, height=20, bg="#0d1117", fg="#e6edf3", font=("Consolas", 9))
        self.log_text.pack(fill="both", expand=True, **pad)

        self.refresh_ports()
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    def log(self, msg):
        self.log_text.insert("end", str(msg) + "\n")
        self.log_text.see("end")
        self.log_text.update_idletasks()

    def refresh_ports(self):
        if list_ports is None:
            self.log("[MIIB] pyserial не найден — установите pip install pyserial")
            return
        ports = [p.device for p in list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])
        self.log(f"[MIIB] Найдено портов: {len(ports)} -> {ports}")

    def choose_file(self):
        f = filedialog.asksaveasfilename(defaultextension=".bin",
                                          filetypes=[("Binary files", "*.bin")])
        if f:
            self.outfile_var.set(f)

    def start_capture(self):
        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("Ошибка", "Выберите COM-порт")
            return
        try:
            baud = int(self.baud_var.get())
            duration = float(self.duration_var.get())
        except ValueError:
            messagebox.showerror("Ошибка", "Baudrate и длительность должны быть числами")
            return

        outfile = self.outfile_var.get().strip()
        self.stop_flag = threading.Event()
        self.start_btn.config(state="disabled")
        self.stop_btn.config(state="normal")
        self.log_text.delete("1.0", "end")

        def worker():
            try:
                do_capture(port, baud, duration, outfile,
                           log_fn=lambda m: self.after(0, self.log, m),
                           stop_flag=self.stop_flag)
            except Exception as e:
                self.after(0, self.log, f"[MIIB] ERROR: {e}")
            finally:
                self.after(0, self._on_finished)

        self.worker = threading.Thread(target=worker, daemon=True)
        self.worker.start()

    def _on_finished(self):
        self.start_btn.config(state="normal")
        self.stop_btn.config(state="disabled")

    def stop_capture(self):
        self.stop_flag.set()
        self.log("[MIIB] Запрошена остановка...")

    def on_close(self):
        self.stop_flag.set()
        self.destroy()


if __name__ == "__main__":
    app = MiibGui()
    app.mainloop()
