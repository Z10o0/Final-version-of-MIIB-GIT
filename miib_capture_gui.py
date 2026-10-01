#!/usr/bin/env python3
"""
MIIB capture GUI.

Текущий wire protocol:
    690 bytes/frame.
    Header: AA 55.
    Counter: uint16 Little Endian, bytes [2:4].
    IMU payload: 36 blocks x 19 bytes, bytes [4:688].
    CRC: uint16 Little Endian, bytes [688:690].
    CRC16-CCITT-FALSE:
        poly=0x1021, init=0xFFFF, no reflect, xorout=0.
        CRC input: bytes [2:688].

Текущий acquisition/output mode:
    Sensor ODR: 3200 Hz.
    Averaging factor: 8.
    Output: 400 frames/s.
    Expected raw traffic: 276000 bytes/s.

Все полученные serial bytes сохраняются в .bin без фильтрации.
Проверка кадров используется только для диагностики.

Baudrate по умолчанию: 4000000.
Если MCU всё ещё настроен на 12000000, выберите 12000000 в GUI.

Dependencies:
    pip install pyserial

Build:
    pyinstaller --onefile --noconsole --name MIIB_Capture miib_capture_gui.py
"""

import binascii
import json
import math
import os
import queue
import threading
import time
import tkinter as tk

from dataclasses import asdict, dataclass
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

try:
    import serial
    import serial.tools.list_ports as list_ports
except ImportError:
    serial = None
    list_ports = None


# =============================================================================
# Protocol and acquisition configuration
# =============================================================================

HEADER = b"\xAA\x55"

FRAME_LEN = 690
N_SENSORS = 36
IMU_WIRE_BYTES = 19

COUNTER_OFFSET = 2
CRC_OFFSET = 688

SENSOR_ODR_HZ = 3200
OUTPUT_RATE_HZ = 400
AVERAGING_FACTOR = SENSOR_ODR_HZ // OUTPUT_RATE_HZ

FRAMES_PER_BATCH = 1
BATCHES_PER_SEC = OUTPUT_RATE_HZ
NOMINAL_FPS = FRAMES_PER_BATCH * BATCHES_PER_SEC

EXPECTED_BPS = NOMINAL_FPS * FRAME_LEN

DEFAULT_BAUD = 4_000_000
READ_CHUNK_BYTES = 65_536
READ_TIMEOUT_SEC = 0.02
RX_QUEUE_DEPTH = 256

assert FRAME_LEN == 2 + 2 + N_SENSORS * IMU_WIRE_BYTES + 2
assert CRC_OFFSET == FRAME_LEN - 2
assert SENSOR_ODR_HZ % OUTPUT_RATE_HZ == 0
assert AVERAGING_FACTOR == 8
assert EXPECTED_BPS == 276_000


# =============================================================================
# Capture statistics
# =============================================================================

@dataclass
class CaptureStats:
    total_bytes: int = 0
    saved_bytes: int = 0

    read_calls: int = 0
    zero_reads: int = 0
    max_single_read: int = 0
    read_errors: int = 0

    queue_high_watermark: int = 0
    queue_overflows: int = 0
    unsaved_queue_bytes: int = 0

    start_perf: float = 0.0
    end_perf: float = 0.0
    first_byte_latency_sec: float = -1.0

    reader_error: str = ""


# =============================================================================
# Diagnostic frame parser
# =============================================================================

class FrameDiagnostics:
    """
    Streaming diagnostic parser.

    Не меняет raw capture.
    При CRC error ищет следующий AA 55 со сдвигом на один байт.
    Поддерживает кадры, разделённые между serial reads.
    """

    def __init__(self):
        self.buffer = bytearray()

        self.valid_frames = 0
        self.crc_errors = 0
        self.discarded_bytes = 0

        self.counter_gaps = 0
        self.duplicates = 0
        self.counter_restarts = 0

        self.last_counter = None

    def feed(self, data):
        self.buffer.extend(data)
        pos = 0

        while True:
            header_pos = self.buffer.find(HEADER, pos)

            if header_pos < 0:
                # Одиночный AA сохраняем: следующий chunk может начаться с 55.
                keep = (
                    1
                    if self.buffer and self.buffer[-1] == HEADER[0]
                    else 0
                )

                self.discarded_bytes += (
                    len(self.buffer) - pos - keep
                )

                pos = len(self.buffer) - keep
                break

            self.discarded_bytes += header_pos - pos
            pos = header_pos

            if len(self.buffer) - pos < FRAME_LEN:
                break

            frame = self.buffer[pos:pos + FRAME_LEN]

            received_crc = int.from_bytes(
                frame[CRC_OFFSET:FRAME_LEN],
                byteorder="little",
                signed=False,
            )

            calculated_crc = binascii.crc_hqx(
                frame[COUNTER_OFFSET:CRC_OFFSET],
                0xFFFF,
            )

            if calculated_crc != received_crc:
                self.crc_errors += 1
                self.discarded_bytes += 1
                pos += 1
                continue

            counter = int.from_bytes(
                frame[COUNTER_OFFSET:COUNTER_OFFSET + 2],
                byteorder="little",
                signed=False,
            )

            if self.last_counter is not None:
                delta = (counter - self.last_counter) & 0xFFFF

                if delta == 0:
                    self.duplicates += 1

                elif delta < 0x8000:
                    self.counter_gaps += delta - 1

                else:
                    # Большой обратный скачок может быть reset MCU,
                    # а не измеримое количество потерянных кадров.
                    self.counter_restarts += 1

            self.last_counter = counter
            self.valid_frames += 1
            pos += FRAME_LEN

        if pos:
            del self.buffer[:pos]

    def snapshot(self):
        return {
            "valid_frames": self.valid_frames,
            "crc_errors": self.crc_errors,
            "discarded_bytes": self.discarded_bytes,
            "counter_gaps": self.counter_gaps,
            "duplicates": self.duplicates,
            "counter_restarts": self.counter_restarts,
            "last_counter": self.last_counter,
            "pending_parser_bytes": len(self.buffer),
        }


# =============================================================================
# Serial reader
# =============================================================================

def try_serial_buffer(ser):
    """
    Запрашивает умеренный RX buffer.
    Успех вызова означает, что драйвер принял запрос.
    """

    if not hasattr(ser, "set_buffer_size"):
        return 0

    for size in (4 * 1024 * 1024, 1024 * 1024, 256 * 1024):
        try:
            ser.set_buffer_size(
                rx_size=size,
                tx_size=64 * 1024,
            )
            return size
        except Exception:
            pass

    return 0


def cancel_pending_read(ser):
    try:
        if hasattr(ser, "cancel_read"):
            ser.cancel_read()
    except Exception:
        pass


class SerialReaderThread(threading.Thread):
    def __init__(
        self,
        ser,
        out_queue,
        stop_event,
        stats,
        stats_lock,
        chunk_size,
    ):
        super().__init__(
            name="MIIB-SerialReader",
            daemon=True,
        )

        self.ser = ser
        self.out_queue = out_queue
        self.stop_event = stop_event

        self.stats = stats
        self.stats_lock = stats_lock

        self.chunk_size = chunk_size
        self.done_event = threading.Event()

    def run(self):
        buf = bytearray(self.chunk_size)
        view = memoryview(buf)

        try:
            while not self.stop_event.is_set():
                try:
                    n = self.ser.readinto(view)

                except Exception as exc:
                    with self.stats_lock:
                        self.stats.read_errors += 1
                        self.stats.reader_error = (
                            f"{type(exc).__name__}: {exc}"
                        )

                    self.stop_event.set()
                    break

                read_time = time.perf_counter()

                with self.stats_lock:
                    self.stats.read_calls += 1

                    if not n:
                        self.stats.zero_reads += 1

                    else:
                        self.stats.total_bytes += n

                        if n > self.stats.max_single_read:
                            self.stats.max_single_read = n

                        if self.stats.first_byte_latency_sec < 0.0:
                            self.stats.first_byte_latency_sec = (
                                read_time - self.stats.start_perf
                            )

                if not n:
                    continue

                data = bytes(view[:n])

                try:
                    # Не отбрасываем байты молча.
                    # Writer продолжает разгружать очередь и при остановке.
                    self.out_queue.put(data, timeout=1.0)

                except queue.Full:
                    with self.stats_lock:
                        self.stats.queue_overflows += 1
                        self.stats.unsaved_queue_bytes += len(data)
                        self.stats.reader_error = (
                            "RX queue overflow: запись неполная"
                        )

                    self.stop_event.set()
                    break

                queue_size = self.out_queue.qsize()

                with self.stats_lock:
                    if queue_size > self.stats.queue_high_watermark:
                        self.stats.queue_high_watermark = queue_size

        finally:
            with self.stats_lock:
                self.stats.end_perf = time.perf_counter()

            self.done_event.set()


# =============================================================================
# Capture worker
# =============================================================================

def do_capture(
    port,
    baud,
    duration,
    outfile,
    chunk=READ_CHUNK_BYTES,
    read_timeout=READ_TIMEOUT_SEC,
    queue_max=RX_QUEUE_DEPTH,
    log_fn=print,
    stop_flag=None,
):
    if serial is None:
        raise RuntimeError(
            "pyserial не установлен. Требуется пакет pyserial."
        )

    if baud <= 0:
        raise ValueError("Baudrate должен быть положительным")

    if not math.isfinite(duration) or duration <= 0:
        raise ValueError(
            "Длительность должна быть положительным конечным числом"
        )

    if chunk <= 0 or queue_max <= 0:
        raise ValueError("Некорректный размер буфера или очереди")

    timestamp = time.strftime("%Y%m%d_%H%M%S")

    out_path = (
        Path(outfile).expanduser()
        if outfile
        else Path(f"miib_raw_{timestamp}.bin")
    )

    out_path.parent.mkdir(parents=True, exist_ok=True)

    metadata_path = out_path.with_suffix(
        out_path.suffix + ".json"
    )

    stats = CaptureStats()
    stats_lock = threading.Lock()

    diagnostics = FrameDiagnostics()

    stop_event = (
        stop_flag
        if stop_flag is not None
        else threading.Event()
    )

    out_queue = queue.Queue(maxsize=queue_max)

    ser = serial.Serial()
    reader = None

    first_bytes = bytearray()
    no_data_warning_sent = False
    capture_started_at = time.strftime("%Y-%m-%dT%H:%M:%S%z")

    try:
        ser.port = port
        ser.baudrate = baud

        ser.bytesize = serial.EIGHTBITS
        ser.parity = serial.PARITY_NONE
        ser.stopbits = serial.STOPBITS_ONE

        ser.timeout = read_timeout
        ser.write_timeout = 1.0

        ser.rtscts = False
        ser.dsrdtr = False
        ser.xonxoff = False

        ser.open()

        buffer_size = try_serial_buffer(ser)

        log_fn(f"[MIIB] Port: {port}")
        log_fn(
            f"[MIIB] Baud requested: {baud}; "
            f"pySerial setting: {ser.baudrate}"
        )
        log_fn(
            "[MIIB] Настройка pySerial не является измерением "
            "фактической скорости UART."
        )
        log_fn(
            f"[MIIB] Format: 8N1, no flow control; "
            f"read timeout={read_timeout:.3f} s"
        )

        if buffer_size:
            log_fn(
                f"[MIIB] Driver RX buffer requested: "
                f"{buffer_size / (1024 * 1024):.1f} MiB"
            )
        else:
            log_fn("[MIIB] Driver RX buffer: default")

        log_fn(
            f"[MIIB] Protocol: {FRAME_LEN} B/frame, "
            f"{N_SENSORS} IMU, {NOMINAL_FPS} frames/s"
        )
        log_fn(
            f"[MIIB] Sensor ODR={SENSOR_ODR_HZ} Hz; "
            f"averaging={AVERAGING_FACTOR}; "
            f"output Fs={OUTPUT_RATE_HZ} Hz"
        )
        log_fn(
            f"[MIIB] Expected: {EXPECTED_BPS} B/s "
            f"= {EXPECTED_BPS / 1024:.2f} KiB/s"
        )
        log_fn(f"[MIIB] Output: {out_path.resolve()}")

        # Удаляем только накопившиеся до начала записи serial bytes.
        ser.reset_input_buffer()

        with open(out_path, "wb", buffering=1024 * 1024) as output:
            stats.start_perf = time.perf_counter()

            reader = SerialReaderThread(
                ser=ser,
                out_queue=out_queue,
                stop_event=stop_event,
                stats=stats,
                stats_lock=stats_lock,
                chunk_size=chunk,
            )

            reader.start()

            deadline = stats.start_perf + duration
            last_report_time = stats.start_perf

            last_report_bytes = 0
            last_report_frames = 0

            while True:
                now = time.perf_counter()

                if now >= deadline and not stop_event.is_set():
                    stop_event.set()
                    cancel_pending_read(ser)

                if stop_event.is_set():
                    # Прерываем pending read, но сначала сохраняем
                    # уже полученные и помещённые в очередь данные.
                    if not reader.done_event.is_set():
                        cancel_pending_read(ser)

                try:
                    data = out_queue.get(timeout=0.05)

                except queue.Empty:
                    data = None

                if data:
                    output.write(data)

                    with stats_lock:
                        stats.saved_bytes += len(data)

                    if len(first_bytes) < 32:
                        needed = 32 - len(first_bytes)
                        first_bytes.extend(data[:needed])

                        if len(first_bytes) == 32:
                            log_fn(
                                "[MIIB] First 32 raw bytes: "
                                + first_bytes.hex(" ")
                            )

                    diagnostics.feed(data)

                now = time.perf_counter()
                elapsed_now = now - stats.start_perf

                if (
                    elapsed_now >= 2.0
                    and not no_data_warning_sent
                ):
                    with stats_lock:
                        no_bytes = stats.total_bytes == 0

                    if no_bytes:
                        no_data_warning_sent = True
                        log_fn(
                            "[MIIB] NO SERIAL BYTES: за первые "
                            "2 секунды не получено ни одного байта."
                        )

                if now - last_report_time >= 1.0:
                    with stats_lock:
                        total = stats.total_bytes
                        saved = stats.saved_bytes
                        read_errors = stats.read_errors
                        queue_hwm = stats.queue_high_watermark

                    interval = now - last_report_time

                    instant_bps = (
                        total - last_report_bytes
                    ) / max(interval, 1e-9)

                    instant_fps = (
                        diagnostics.valid_frames - last_report_frames
                    ) / max(interval, 1e-9)

                    average_bps = total / max(elapsed_now, 1e-9)

                    log_fn(
                        f"[MIIB] t={elapsed_now:6.2f}s | "
                        f"RX={instant_bps:9.0f} B/s | "
                        f"avg={average_bps:9.0f} B/s | "
                        f"valid={instant_fps:7.1f} fps | "
                        f"CRC={diagnostics.crc_errors} | "
                        f"gaps={diagnostics.counter_gaps} | "
                        f"RX_total={total} | saved={saved} | "
                        f"q_hwm={queue_hwm} | rd_err={read_errors}"
                    )

                    last_report_time = now
                    last_report_bytes = total
                    last_report_frames = diagnostics.valid_frames

                # После завершения reader новые chunks уже не появятся.
                if (
                    reader.done_event.is_set()
                    and out_queue.empty()
                ):
                    break

            reader.join(timeout=2.0)

            output.flush()
            os.fsync(output.fileno())

    finally:
        stop_event.set()

        if ser.is_open:
            cancel_pending_read(ser)

        if reader is not None:
            reader.join(timeout=2.0)

        if ser.is_open:
            ser.close()

    with stats_lock:
        stats_copy = asdict(stats)

    elapsed = max(
        stats.end_perf - stats.start_perf,
        1e-9,
    )

    received_bps = stats.total_bytes / elapsed
    valid_fps = diagnostics.valid_frames / elapsed
    raw_pct = received_bps / EXPECTED_BPS * 100.0

    if first_bytes and len(first_bytes) < 32:
        log_fn(
            "[MIIB] First raw bytes: "
            + first_bytes.hex(" ")
        )

    log_fn(
        f"[MIIB] Done: RX={stats.total_bytes} B, "
        f"saved={stats.saved_bytes} B, "
        f"elapsed={elapsed:.3f} s"
    )

    log_fn(
        f"[MIIB] RAW rate: {received_bps:.1f} B/s "
        f"({raw_pct:.2f}% of nominal)"
    )

    log_fn(
        f"[MIIB] Valid frames: {diagnostics.valid_frames}; "
        f"rate={valid_fps:.2f} fps"
    )

    log_fn(
        f"[MIIB] CRC errors={diagnostics.crc_errors}; "
        f"counter gaps={diagnostics.counter_gaps}; "
        f"duplicates={diagnostics.duplicates}; "
        f"counter restarts/backward jumps="
        f"{diagnostics.counter_restarts}"
    )

    log_fn(
        f"[MIIB] Diagnostic discarded bytes="
        f"{diagnostics.discarded_bytes}; "
        f"pending parser bytes={len(diagnostics.buffer)}"
    )

    if stats.reader_error:
        log_fn(
            f"[MIIB] SERIAL/READER ERROR: "
            f"{stats.reader_error}"
        )

    if stats.total_bytes == 0:
        result = "no_serial_bytes"

        log_fn(
            "[MIIB] RESULT: байты отсутствуют. "
            "Проверять выбранный COM-порт, работающий MCU, "
            "UART TX, RS-485 и подключение адаптера."
        )

    elif diagnostics.valid_frames == 0:
        result = "bytes_without_valid_frames"

        log_fn(
            "[MIIB] RESULT: байты получены, но валидных "
            "690-байтовых кадров нет. Проверять совпадение "
            "baudrate MCU/ПК, сигнал и соответствие протоколу."
        )

    elif (
        stats.read_errors
        or stats.queue_overflows
        or stats.saved_bytes != stats.total_bytes
    ):
        result = "capture_error"

        log_fn(
            "[MIIB] RESULT: ошибка serial/capture. "
            "Запись нельзя считать полной."
        )

    elif (
        diagnostics.crc_errors
        or diagnostics.counter_gaps
        or diagnostics.duplicates
        or diagnostics.counter_restarts
    ):
        result = "frames_with_errors"

        log_fn(
            "[MIIB] RESULT: кадры распознаются, "
            "но обнаружены ошибки CRC или счётчика."
        )

    else:
        result = "valid_frames"

        log_fn(
            "[MIIB] RESULT: валидные кадры без выявленных "
            "ошибок CRC и последовательности счётчика."
        )

    if stats.total_bytes and raw_pct < 95.0:
        log_fn(
            "[MIIB] WARN: скорость ниже 95% номинальной. "
            "Это может означать задержку запуска, меньшую "
            "частоту передачи или потери; один byte rate "
            "не устанавливает причину."
        )

    metadata = {
        "format": "MIIB_690_IMU19",
        "capture_started_at": capture_started_at,
        "raw_file": str(out_path.resolve()),
        "port": port,
        "baudrate_requested": baud,
        "uart_format": "8N1",
        "flow_control": False,

        "sensor_odr_hz": SENSOR_ODR_HZ,
        "output_rate_hz": OUTPUT_RATE_HZ,
        "averaging_factor": AVERAGING_FACTOR,

        "frame_len": FRAME_LEN,
        "sensor_count": N_SENSORS,
        "imu_wire_bytes": IMU_WIRE_BYTES,
        "frames_per_batch": FRAMES_PER_BATCH,

        "expected_bytes_per_second": EXPECTED_BPS,
        "requested_duration_sec": duration,
        "actual_capture_duration_sec": elapsed,

        "measured_bytes_per_second": received_bps,
        "measured_valid_frames_per_second": valid_fps,

        "result": result,
        "stats": stats_copy,
        "frame_diagnostics": diagnostics.snapshot(),
        "first_raw_bytes_hex": first_bytes.hex(" "),

        "timestamp_semantics": (
            "Last raw FIFO sample timestamp in averaging window; "
            "not the output sampling interval."
        ),
    }

    with open(metadata_path, "w", encoding="utf-8") as metadata_file:
        json.dump(
            metadata,
            metadata_file,
            ensure_ascii=False,
            indent=2,
        )

    log_fn(f"[MIIB] Saved RAW: {out_path.resolve()}")
    log_fn(f"[MIIB] Saved metadata: {metadata_path.resolve()}")

    return out_path, elapsed, stats


# =============================================================================
# GUI
# =============================================================================

class MiibGui(tk.Tk):
    def __init__(self):
        super().__init__()

        self.title("MIIB Capture — 36 IMU / 400 Hz")
        self.geometry("1000x650")
        self.minsize(800, 450)

        self.stop_flag = threading.Event()
        self.worker = None

        self.ui_queue = queue.Queue()
        self.closing = False

        self.input_widgets = []

        pad = {"padx": 8, "pady": 5}

        form = ttk.Frame(self)
        form.pack(fill="x", **pad)

        form.columnconfigure(1, weight=1)

        ttk.Label(
            form,
            text="COM-порт:",
        ).grid(row=0, column=0, sticky="w")

        self.port_var = tk.StringVar()

        self.port_combo = ttk.Combobox(
            form,
            textvariable=self.port_var,
            width=24,
            state="readonly",
        )

        self.port_combo.grid(
            row=0,
            column=1,
            sticky="w",
        )

        self.refresh_btn = ttk.Button(
            form,
            text="Обновить",
            command=self.refresh_ports,
        )

        self.refresh_btn.grid(
            row=0,
            column=2,
            padx=4,
        )

        ttk.Label(
            form,
            text="Baudrate:",
        ).grid(row=1, column=0, sticky="w")

        self.baud_var = tk.StringVar(
            value=str(DEFAULT_BAUD)
        )

        self.baud_combo = ttk.Combobox(
            form,
            textvariable=self.baud_var,
            values=("4000000", "12000000", "6000000", "3000000"),
            width=24,
            state="normal",
        )

        self.baud_combo.grid(
            row=1,
            column=1,
            sticky="w",
        )

        ttk.Label(
            form,
            text="Должен совпадать с USART1 в прошивке",
        ).grid(row=1, column=2, columnspan=2, sticky="w")

        ttk.Label(
            form,
            text="Длительность, с:",
        ).grid(row=2, column=0, sticky="w")

        self.duration_var = tk.StringVar(value="30")

        self.duration_entry = ttk.Entry(
            form,
            textvariable=self.duration_var,
            width=24,
        )

        self.duration_entry.grid(
            row=2,
            column=1,
            sticky="w",
        )

        ttk.Label(
            form,
            text="Raw файл .bin:",
        ).grid(row=3, column=0, sticky="w")

        self.outfile_var = tk.StringVar(value="")

        self.outfile_entry = ttk.Entry(
            form,
            textvariable=self.outfile_var,
        )

        self.outfile_entry.grid(
            row=3,
            column=1,
            columnspan=2,
            sticky="ew",
        )

        self.browse_btn = ttk.Button(
            form,
            text="Обзор...",
            command=self.choose_file,
        )

        self.browse_btn.grid(
            row=3,
            column=3,
            padx=4,
        )

        ttk.Label(
            form,
            text=(
                "690 байт × 400 кадров/с = 276000 байт/с; "
                "ODR датчиков 3200 Гц, усреднение ×8"
            ),
        ).grid(
            row=4,
            column=0,
            columnspan=4,
            sticky="w",
            pady=5,
        )

        buttons = ttk.Frame(self)
        buttons.pack(fill="x", **pad)

        self.start_btn = ttk.Button(
            buttons,
            text="Начать запись",
            command=self.start_capture,
        )

        self.start_btn.pack(side="left", padx=4)

        self.stop_btn = ttk.Button(
            buttons,
            text="Стоп",
            command=self.stop_capture,
            state="disabled",
        )

        self.stop_btn.pack(side="left", padx=4)

        log_frame = ttk.Frame(self)
        log_frame.pack(
            fill="both",
            expand=True,
            **pad,
        )

        self.log_text = tk.Text(
            log_frame,
            wrap="none",
            bg="#0d1117",
            fg="#e6edf3",
            font=("Consolas", 10),
        )

        vertical_scroll = ttk.Scrollbar(
            log_frame,
            orient="vertical",
            command=self.log_text.yview,
        )

        horizontal_scroll = ttk.Scrollbar(
            log_frame,
            orient="horizontal",
            command=self.log_text.xview,
        )

        self.log_text.configure(
            yscrollcommand=vertical_scroll.set,
            xscrollcommand=horizontal_scroll.set,
        )

        self.log_text.grid(row=0, column=0, sticky="nsew")
        vertical_scroll.grid(row=0, column=1, sticky="ns")
        horizontal_scroll.grid(row=1, column=0, sticky="ew")

        log_frame.rowconfigure(0, weight=1)
        log_frame.columnconfigure(0, weight=1)

        self.input_widgets = [
            self.port_combo,
            self.refresh_btn,
            self.baud_combo,
            self.duration_entry,
            self.outfile_entry,
            self.browse_btn,
        ]

        self.refresh_ports()

        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(100, self.poll_ui_queue)

    def log(self, message):
        # Вызывается только главным GUI-потоком.
        self.log_text.insert("end", str(message) + "\n")
        self.log_text.see("end")

    def poll_ui_queue(self):
        processed = 0

        while processed < 200:
            try:
                kind, payload = self.ui_queue.get_nowait()
            except queue.Empty:
                break

            processed += 1

            if kind == "log":
                self.log(payload)

            elif kind == "finished":
                self.on_finished()

        if self.closing:
            if self.worker is None or not self.worker.is_alive():
                self.destroy()
                return

        self.after(100, self.poll_ui_queue)

    def refresh_ports(self):
        if list_ports is None:
            self.log(
                "[MIIB] pyserial не найден. "
                "Требуется пакет pyserial."
            )
            return

        ports = sorted(
            list_ports.comports(),
            key=lambda item: item.device,
        )

        names = [item.device for item in ports]
        self.port_combo["values"] = names

        if self.port_var.get() not in names:
            self.port_var.set(names[0] if names else "")

        self.log(f"[MIIB] Найдено COM-портов: {len(ports)}")

        for item in ports:
            self.log(
                f"[MIIB] {item.device}: "
                f"{item.description}; {item.hwid}"
            )

    def choose_file(self):
        selected = filedialog.asksaveasfilename(
            defaultextension=".bin",
            filetypes=[("Binary files", "*.bin")],
        )

        if selected:
            self.outfile_var.set(selected)

    def set_busy(self, busy):
        for widget in self.input_widgets:
            if busy:
                widget.configure(state="disabled")

            elif widget is self.port_combo:
                widget.configure(state="readonly")

            else:
                widget.configure(state="normal")

        self.start_btn.configure(
            state="disabled" if busy else "normal"
        )

        self.stop_btn.configure(
            state="normal" if busy else "disabled"
        )

    def start_capture(self):
        if self.worker is not None and self.worker.is_alive():
            return

        if serial is None:
            messagebox.showerror(
                "Ошибка",
                "Не установлен pyserial.",
            )
            return

        port = self.port_var.get().strip()

        if not port:
            messagebox.showerror(
                "Ошибка",
                "Выберите COM-порт.",
            )
            return

        try:
            baud = int(self.baud_var.get().strip())

            duration = float(
                self.duration_var.get().strip().replace(",", ".")
            )

            if baud <= 0:
                raise ValueError

            if not math.isfinite(duration) or duration <= 0:
                raise ValueError

        except ValueError:
            messagebox.showerror(
                "Ошибка",
                "Baudrate и длительность должны быть "
                "положительными числами.",
            )
            return

        if baud < EXPECTED_BPS * 10:
            messagebox.showerror(
                "Недостаточная скорость UART",
                f"Для {NOMINAL_FPS} кадров/с по {FRAME_LEN} байт "
                f"при 8N1 требуется не менее "
                f"{EXPECTED_BPS * 10} baud без запаса.",
            )
            return

        outfile = self.outfile_var.get().strip()

        self.stop_flag = threading.Event()
        self.log_text.delete("1.0", "end")
        self.set_busy(True)

        def worker():
            try:
                do_capture(
                    port=port,
                    baud=baud,
                    duration=duration,
                    outfile=outfile,
                    log_fn=lambda message: self.ui_queue.put(
                        ("log", str(message))
                    ),
                    stop_flag=self.stop_flag,
                )

            except Exception as exc:
                self.ui_queue.put(
                    (
                        "log",
                        f"[MIIB] ERROR: "
                        f"{type(exc).__name__}: {exc}",
                    )
                )

            finally:
                self.ui_queue.put(("finished", None))

        self.worker = threading.Thread(
            target=worker,
            name="MIIB-CaptureWorker",
            daemon=True,
        )

        self.worker.start()

    def on_finished(self):
        self.set_busy(False)

    def stop_capture(self):
        self.stop_flag.set()

        self.stop_btn.configure(state="disabled")
        self.log("[MIIB] Запрошена остановка; сохранение очереди...")

    def on_close(self):
        if self.closing:
            return

        self.closing = True
        self.stop_flag.set()

        self.start_btn.configure(state="disabled")
        self.stop_btn.configure(state="disabled")

        if self.worker is not None and self.worker.is_alive():
            self.log(
                "[MIIB] Завершение записи перед закрытием окна..."
            )
        else:
            self.destroy()


if __name__ == "__main__":
    app = MiibGui()
    app.mainloop()