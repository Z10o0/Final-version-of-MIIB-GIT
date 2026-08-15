#!/usr/bin/env python3
"""
miib_capture.py — MIIB unified tool v5: надёжный capture → offline parse →
interactive Plotly plots, с полной диагностикой "Python bug vs line loss".

Wire format (690 bytes):
  [0..1]    Header 0xAA 0x55
  [2..3]    frame_counter uint16 LE
  [4..687]  36 x 19-byte HIRES IMU block
  [688..689] CRC16-CCITT LE  (covers bytes [2..687])

Прибор реально шлёт кадры на НОМИНАЛЬНОЙ частоте 1600 fps
(FRAMES_PER_BATCH=16 x BATCHES_PER_SEC=100).

[FIX v5] Полная переработка capture-слоя и диагностики парсера по итогам
разбора реального лога, где Python видел лишь ~70.6% nominal throughput,
хотя счётчики STM32 (g_uart_drop_count=0, g_uart_dma_te/dme/fe_count=0,
enqueue==dma_start==dma_tc) показывали полностью здоровый TX-тракт на
приборе. Раз аппаратный тракт чист, узкое место нужно искать и жёстко
идентифицировать на стороне ПК — этот файл даёт для этого инструментарий:

  1. CAPTURE — reader вынесен в отдельный поток с tight-loop blocking read
     (readinto() в preallocated bytearray), без polling-паттерна
     `if in_waiting: read() else: sleep()`, который отдаёт GIL/scheduler
     непредсказуемо на высоких скоростях. Producer (reader thread) и
     consumer (writer/print) развязаны через thread-safe очередь чанков,
     чтобы файловый I/O и печать статистики никогда не блокировали приём.
  2. Отдельно считается RAW throughput (сколько байт реально прочитано
     из порта) и VALID FRAME throughput (сколько байт разложилось в
     CRC-валидные кадры) — это два разных числа, и раньше скрипт печатал
     только производную от valid-frames величину, маскируя место потери.
  3. Печатается подробная CRC/header/resync статистика: сколько всего
     найдено header-кандидатов (0xAA 0x55), сколько из них провалили CRC,
     сколько байт "мусора" суммарно съедено при ресинхронизации, и
     распределение расстояний между соседними валидными кадрами.
  4. Жёсткий диагностический вывод: если raw_bytes от pyserial почти равны
     nominal (>=98%), а valid-frame throughput ощутимо ниже — это ГОВОРИТ
     о проблеме в самом потоке байт до сборки кадра, что чаще всего
     означает ресинк/потерю данных ДО пришедших в Python байт (то есть на
     линии/драйвере/прошивке). Если же raw_bytes от pyserial САМИ по себе
     заметно ниже nominal — winner виноват capture-слой Python
     (недостаточно быстрое считывание из ОС-буфера), и это НЕ аппаратная
     проблема STM32.

Install:
  pip install pyserial numpy plotly

Usage:
  python miib_capture.py capture --port COM11 --duration 30
  python miib_capture.py capture --port COM11 --duration 60 --no-parse
  python miib_capture.py parse   --infile miib_raw_20260814_200000.bin
"""

import argparse
import os
import sys
import time
import struct
import threading
import queue
import webbrowser
from pathlib import Path
from dataclasses import dataclass, field

try:
    import numpy as np
except ImportError:
    sys.exit("ERROR: pip install numpy")

try:
    import serial
except ImportError:
    serial = None

try:
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots
    HAS_PLOTLY = True
except ImportError:
    HAS_PLOTLY = False
    print("WARNING: pip install plotly", file=sys.stderr)


# =============================================================================
#  Протокол
# =============================================================================
FRAME_LEN        = 690
N_SENSORS        = 36
IMU_BYTES        = 19
OFF_COUNTER      = 2
OFF_SAMPLES      = 4
OFF_CRC          = 688
PAYLOAD_LEN      = 686

FRAMES_PER_BATCH = 16
BATCHES_PER_SEC  = 100
NOMINAL_FPS      = FRAMES_PER_BATCH * BATCHES_PER_SEC     # 1600 fps
EXPECTED_BPS     = NOMINAL_FPS * FRAME_LEN                 # 1 104 000 B/s

ACCEL_FS_G   = 32.0
GYRO_FS_DPS  = 4000.0
ACCEL_LSB    = ACCEL_FS_G  / 2**19
GYRO_LSB     = GYRO_FS_DPS / 2**19
TEMP_SCALE   = 1.0 / 128.0
TEMP_OFFSET  = 25.0

C_RED   = "#ef5350"
C_BLUE  = "#42a5f5"
C_GREEN = "#66bb6a"
C_ORG   = "#ffa726"
C_PURP  = "#ce93d8"
C_GRAY  = "#90a4ae"
BG      = "#0d1117"
BG2     = "#161b22"
GRID_C  = "#30363d"
TEXT_C  = "#e6edf3"


# =============================================================================
#  CRC-16/CCITT-FALSE
# =============================================================================
def _build_crc16_table():
    poly, tbl = 0x1021, []
    for i in range(256):
        crc = i << 8
        for _ in range(8):
            crc = ((crc << 1) ^ poly) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
        tbl.append(crc)
    return tbl

_CRC16_TABLE = _build_crc16_table()

def crc16(data) -> int:
    crc = 0xFFFF
    for b in data:
        crc = ((crc << 8) ^ _CRC16_TABLE[((crc >> 8) ^ b) & 0xFF]) & 0xFFFF
    return crc


# =============================================================================
#  CAPTURE — надёжный многопоточный reader
# =============================================================================
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
    """
    Выделенный поток чтения из порта.

    [FIX v5] Ключевое отличие от старого polling-цикла
    `if in_waiting: read() else: sleep(poll)`:
      - используется БЛОКИРУЮЩЕЕ чтение с коротким timeout на уровне
        pyserial (ser.timeout = read_timeout), а не активный опрос
        in_waiting из user-space с ручным sleep(). Это устраняет
        зависимость от точности time.sleep() и от GIL-scheduling
        под нагрузкой печати/записи в основном потоке.
      - каждый прочитанный чанк немедленно кладётся в Queue и поток
        сразу идёт читать дальше, не дожидаясь записи на диск —
        запись и статистика делаются в основном потоке параллельно.
      - размер чанка не ограничен искусственно (chunk_size — верхняя
        граница одного read(), а не единственный источник данных за
        цикл), поэтому при всплесках накопленных в буфере ОС данных
        поток вычитывает их за минимум итераций.
    """

    def __init__(self, ser: "serial.Serial", out_queue: "queue.Queue",
                 stop_event: threading.Event, chunk_size: int,
                 stats: CaptureStats, stats_lock: threading.Lock):
        super().__init__(name="MIIB-SerialReader", daemon=True)
        self.ser = ser
        self.out_queue = out_queue
        self.stop_event = stop_event
        self.chunk_size = chunk_size
        self.stats = stats
        self.stats_lock = stats_lock

    def run(self):
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


def do_capture(args) -> tuple[Path, float, CaptureStats]:
    """Возвращает (путь_к_файлу, реальное_время_захвата_сек, CaptureStats)."""
    if serial is None:
        sys.exit("ERROR: pip install pyserial")

    ts       = time.strftime("%Y%m%d_%H%M%S")
    out_path = Path(args.outfile) if args.outfile else Path(f"miib_raw_{ts}.bin")

    ser = serial.Serial()
    ser.port = args.port
    ser.baudrate = args.baud
    ser.bytesize = serial.EIGHTBITS
    ser.parity = serial.PARITY_NONE
    ser.stopbits = serial.STOPBITS_ONE
    # [FIX v5] Короткий блокирующий timeout вместо timeout=0 + busy poll.
    # readinto() вернётся либо когда придут данные, либо через read_timeout.
    ser.timeout = args.read_timeout
    ser.rtscts = ser.dsrdtr = ser.xonxoff = False
    ser.open()

    stats = CaptureStats()
    stats_lock = threading.Lock()
    stop_event = threading.Event()
    out_queue: "queue.Queue[bytes]" = queue.Queue(maxsize=args.queue_max)

    elapsed = 0.0
    reader = None
    try:
        try:
            ser.set_buffer_size(rx_size=16 * 1024 * 1024, tx_size=64 * 1024)
        except Exception:
            pass
        ser.reset_input_buffer()

        print(f"[MIIB] Port    : {args.port} @ {args.baud} baud")
        print(f"[MIIB] Frame   : {FRAME_LEN} bytes  ({N_SENSORS} sensors x {IMU_BYTES} B)")
        print(f"[MIIB] Nominal : {NOMINAL_FPS} fps  ->  {EXPECTED_BPS} B/s  ({EXPECTED_BPS/1024:.1f} KiB/s)")
        print(f"[MIIB] Duration: {args.duration:.1f} s  ->  Output: {out_path}")
        print(f"[MIIB] Reader  : threaded, chunk<={args.chunk} B, read_timeout={args.read_timeout*1000:.1f} ms")

        reader = _SerialReaderThread(ser, out_queue, stop_event, args.chunk,
                                      stats, stats_lock)
        stats.start_perf = time.perf_counter()
        reader.start()

        t0 = t_last = time.perf_counter()
        last_total = 0

        with open(out_path, "wb", buffering=1024 * 1024) as f:
            while True:
                now = time.perf_counter()
                if now - t0 >= args.duration:
                    break
                try:
                    chunk = out_queue.get(timeout=0.05)
                    f.write(chunk)
                except queue.Empty:
                    pass

                if now - t_last >= 1.0:
                    with stats_lock:
                        total = stats.total_bytes
                    el = now - t0
                    dt = now - t_last
                    print(f"[MIIB] t={el:6.2f}s | "
                          f"inst={(total - last_total)/dt/1024:8.1f} KiB/s | "
                          f"avg={total/max(el,1e-9)/1024:8.1f} KiB/s | "
                          f"total={total:10d} B | "
                          f"q_hwm={stats.queue_high_watermark:4d} | "
                          f"rd_err={stats.read_errors}")
                    t_last, last_total = now, total

            stop_event.set()
            reader.join(timeout=2.0)

            # Дренируем то, что осталось в очереди после остановки потока.
            drained = 0
            while True:
                try:
                    chunk = out_queue.get_nowait()
                    f.write(chunk)
                    drained += len(chunk)
                except queue.Empty:
                    break
            f.flush()
            os.fsync(f.fileno())

        stats.end_perf = time.perf_counter()
        elapsed = stats.end_perf - t0
        with stats_lock:
            total = stats.total_bytes
        bps = total / max(elapsed, 1e-9)

        print(f"[MIIB] Done    : {total} B in {elapsed:.3f} s = "
              f"{bps/1024:.1f} KiB/s ({bps/EXPECTED_BPS*100:.2f}% of nominal RAW)")
        print(f"[MIIB] Drained : {drained} B (post-stop residual)")
        print(f"[MIIB] Reader  : read_calls={stats.read_calls}  zero_reads={stats.zero_reads}  "
              f"max_single_read={stats.max_single_read} B  read_errors={stats.read_errors}  "
              f"queue_high_watermark={stats.queue_high_watermark}/{args.queue_max}")
        print(f"[MIIB] Saved   : {out_path}")

        # [FIX v5] Немедленный жёсткий вердикт по RAW throughput —
        # ещё до сборки кадров и CRC-анализа.
        raw_pct = bps / EXPECTED_BPS * 100.0
        if raw_pct < 95.0:
            print(f"[MIIB] !!! RAW throughput {raw_pct:.1f}% < 95% nominal — "
                  f"похоже на проблему CAPTURE-слоя Python/ОС/драйвера "
                  f"(поток не успевает вычитывать буфер порта), "
                  f"а не на аппаратный тракт STM32.")
        else:
            print(f"[MIIB] RAW throughput OK ({raw_pct:.1f}% nominal) — "
                  f"Python успевает считывать байты почти на полной скорости; "
                  f"любая дальнейшая потеря будет искаться на уровне "
                  f"кадров/CRC/линии, см. секцию VALID FRAME ниже.")
    finally:
        stop_event.set()
        if reader is not None:
            reader.join(timeout=2.0)
        try:
            ser.close()
        except Exception:
            pass

    return out_path, elapsed, stats


# =============================================================================
#  PARSE — с полной CRC/header/resync диагностикой
# =============================================================================
def _s20(u: int) -> int:
    return u - (1 << 20) if u >= (1 << 19) else u


@dataclass
class ParseDiag:
    total_bytes: int = 0
    header_candidates: int = 0
    crc_ok_frames: int = 0
    crc_fail_events: int = 0
    resync_gap_bytes: int = 0          # суммарно "съеденных" байт мусора
    resync_events: int = 0             # сколько раз пришлось искать следующий header
    gap_distances: list = field(default_factory=list)  # байт между началами соседних valid frames
    max_resync_gap: int = 0
    leading_garbage_bytes: int = 0
    trailing_garbage_bytes: int = 0


def do_parse(raw_path: Path, capture_elapsed: float | None = None) -> dict:
    """
    capture_elapsed — реальное время захвата в секундах (из do_capture()).
    Используется для построения честной временной оси t.
    Если None (офлайн-парсинг старого .bin без метаданных времени),
    используется НОМИНАЛЬНАЯ частота прибора NOMINAL_FPS=1600 как fallback.
    """
    raw = Path(raw_path).read_bytes()
    buf = np.frombuffer(raw, dtype=np.uint8)
    diag = ParseDiag(total_bytes=len(raw))

    is_hdr  = (buf[:-1] == 0xAA) & (buf[1:] == 0x55)
    hdr_pos = np.where(is_hdr)[0]
    hdr_pos = hdr_pos[hdr_pos + FRAME_LEN <= len(raw)]
    diag.header_candidates = int(len(hdr_pos))

    valid_starts, i = [], 0
    prev_end = None
    while i < len(hdr_pos):
        pos   = int(hdr_pos[i])
        frame = raw[pos: pos + FRAME_LEN]
        if crc16(frame[OFF_COUNTER: OFF_COUNTER + PAYLOAD_LEN]) == \
               struct.unpack_from("<H", frame, OFF_CRC)[0]:
            if prev_end is not None:
                gap = pos - prev_end
                if gap > 0:
                    diag.resync_gap_bytes += gap
                    diag.resync_events += 1
                    if gap > diag.max_resync_gap:
                        diag.max_resync_gap = gap
                diag.gap_distances.append(pos - (valid_starts[-1] if valid_starts else pos))
            else:
                diag.leading_garbage_bytes = pos

            valid_starts.append(pos)
            prev_end = pos + FRAME_LEN
            diag.crc_ok_frames += 1
            j = int(np.searchsorted(hdr_pos, pos + FRAME_LEN))
            if j >= len(hdr_pos):
                i = len(hdr_pos)
                break
            i = j
        else:
            diag.crc_fail_events += 1
            i += 1

    if valid_starts:
        diag.trailing_garbage_bytes = len(raw) - (valid_starts[-1] + FRAME_LEN)

    n_frames = len(valid_starts)
    assert n_frames > 0, f"No valid CRC frames in {raw_path}"
    print(f"[MIIB] Valid CRC frames: {n_frames}")

    counters = np.zeros(n_frames, dtype=np.uint16)
    accel    = np.full((n_frames, N_SENSORS, 3), np.nan)
    gyro     = np.full((n_frames, N_SENSORS, 3), np.nan)
    temp     = np.full((n_frames, N_SENSORS),    np.nan)
    tstamp   = np.full((n_frames, N_SENSORS),    np.nan)
    invalid  = np.zeros((n_frames, N_SENSORS),   dtype=bool)

    for fi, pos in enumerate(valid_starts):
        frame = raw[pos: pos + FRAME_LEN]
        counters[fi] = struct.unpack_from("<H", frame, OFF_COUNTER)[0]
        for k in range(N_SENSORS):
            off = OFF_SAMPLES + k * IMU_BYTES
            blk = frame[off: off + IMU_BYTES]
            if all(b == 0 for b in blk):
                invalid[fi, k] = True
                continue

            ax20 = (blk[0]<<12)|(blk[1]<<4)|((blk[16]&0xF0)>>4)
            ay20 = (blk[2]<<12)|(blk[3]<<4)|((blk[17]&0xF0)>>4)
            az20 = (blk[4]<<12)|(blk[5]<<4)|((blk[18]&0xF0)>>4)
            gx20 = (blk[6]<<12)|(blk[7]<<4)| (blk[16]&0x0F)
            gy20 = (blk[8]<<12)|(blk[9]<<4)| (blk[17]&0x0F)
            gz20 = (blk[10]<<12)|(blk[11]<<4)|(blk[18]&0x0F)

            accel[fi,k] = [_s20(ax20)*ACCEL_LSB, _s20(ay20)*ACCEL_LSB, _s20(az20)*ACCEL_LSB]
            gyro [fi,k] = [_s20(gx20)*GYRO_LSB,  _s20(gy20)*GYRO_LSB,  _s20(gz20)*GYRO_LSB]
            t_raw       = struct.unpack_from(">h", bytes(blk[12:14]))[0]
            temp[fi,k]  = t_raw * TEMP_SCALE + TEMP_OFFSET
            tstamp[fi,k]= (blk[14] << 8) | blk[15]

    if n_frames > 1:
        dcnt           = np.diff(counters.astype(np.int32)) % 65536
        counter_gaps   = int(np.sum(dcnt != 1))
        missing_frames = int(np.sum(np.maximum(dcnt - 1, 0)))
    else:
        counter_gaps = missing_frames = 0

    if capture_elapsed and capture_elapsed > 0:
        dt_frames    = capture_elapsed / n_frames
        time_source  = f"measured ({n_frames} frames / {capture_elapsed:.3f} s capture)"
    else:
        dt_frames    = 1.0 / NOMINAL_FPS
        time_source  = f"nominal ({NOMINAL_FPS} fps fallback - no capture time available)"

    t          = np.arange(n_frames) * dt_frames
    real_fps   = 1.0 / dt_frames
    valid_per  = (~invalid).sum(axis=0)
    inv_per    = invalid.sum(axis=0)

    # === Общий сводный вывод ===
    print(f"\n{'='*60}")
    print(f"MIIB SUMMARY")
    print(f"{'='*60}")
    print(f"  Valid frames   : {n_frames}")
    print(f"  Counter gaps   : {counter_gaps}   Missing: {missing_frames}")
    print(f"  Time source    : {time_source}")
    print(f"  Frame rate     : {real_fps:.2f} fps  (nominal: {NOMINAL_FPS} fps)")
    print(f"  Duration       : {t[-1]:.3f} s")
    print(f"{'-'*60}")
    print(f"  {'Sensor':<8} {'Valid':>8} {'Invalid':>8}  {'Status'}")
    for k in range(N_SENSORS):
        status = "OK" if inv_per[k] == 0 else ("DEAD" if valid_per[k] == 0 else "ERR")
        print(f"  S{k:02d}      {valid_per[k]:8d}   {inv_per[k]:8d}  {status}")
    print(f"{'='*60}\n")

    # === [FIX v5] CRC / header / resync диагностика ===
    valid_frame_bytes = n_frames * FRAME_LEN
    raw_frame_bytes_pct = (valid_frame_bytes / diag.total_bytes * 100.0) if diag.total_bytes else 0.0
    crc_fail_rate = (diag.crc_fail_events / diag.header_candidates * 100.0) if diag.header_candidates else 0.0
    mean_gap = float(np.mean(diag.gap_distances)) if diag.gap_distances else 0.0
    median_gap = float(np.median(diag.gap_distances)) if diag.gap_distances else 0.0

    print(f"{'='*60}")
    print(f"CRC / HEADER / RESYNC DIAGNOSTICS")
    print(f"{'='*60}")
    print(f"  Raw file bytes           : {diag.total_bytes}")
    print(f"  Header candidates (AA55) : {diag.header_candidates}")
    print(f"  CRC-OK frames            : {diag.crc_ok_frames}")
    print(f"  CRC-FAIL events          : {diag.crc_fail_events}  ({crc_fail_rate:.3f}% of candidates)")
    print(f"  Resync events            : {diag.resync_events}")
    print(f"  Resync garbage bytes     : {diag.resync_gap_bytes}  (max single gap: {diag.max_resync_gap} B)")
    print(f"  Leading garbage bytes    : {diag.leading_garbage_bytes}")
    print(f"  Trailing garbage bytes   : {diag.trailing_garbage_bytes}")
    print(f"  Valid-frame byte share   : {valid_frame_bytes} / {diag.total_bytes}  ({raw_frame_bytes_pct:.2f}%)")
    if diag.gap_distances:
        print(f"  Inter-frame gap (bytes)  : mean={mean_gap:.1f}  median={median_gap:.1f}  "
              f"expected={FRAME_LEN}")
    print(f"{'='*60}\n")

    # === [FIX v5] Жёсткий вердикт: где реально теряются данные ===
    print(f"{'='*60}")
    print(f"VERDICT: Python bug vs line/hardware loss")
    print(f"{'='*60}")
    if raw_frame_bytes_pct >= 99.5 and diag.crc_fail_events == 0 and diag.resync_events == 0:
        print(f"  -> Практически 100% сырых байт разложились в валидные кадры")
        print(f"     без единого CRC-fail/resync. Если итоговый fps всё равно")
        print(f"     ниже nominal — узкое место НЕ в парсере и НЕ в целостности")
        print(f"     потока, а в RAW throughput на этапе capture (см. вывод")
        print(f"     'RAW throughput' от do_capture) или в самом источнике данных.")
    elif diag.resync_events > 0 and crc_fail_rate < 1.0:
        print(f"  -> Резинки редки ({diag.resync_events}), CRC-fail rate низкий")
        print(f"     ({crc_fail_rate:.3f}%). Поток почти целый, небольшие точечные")
        print(f"     потери -- нормальная картина для реальной линии на высокой")
        print(f"     скорости, не системный баг парсера.")
    elif crc_fail_rate >= 1.0 and diag.header_candidates > diag.crc_ok_frames * 1.5:
        print(f"  -> Высокий CRC-fail rate ({crc_fail_rate:.2f}%) и много лишних")
        print(f"     header-кандидатов относительно valid frames. Это означает")
        print(f"     системную порчу/сдвиг байт ДО парсера -- то есть на линии,")
        print(f"     в capture-слое (недочитанные/задвоенные байты) или в самой")
        print(f"     прошивке. Нужно сверить с RAW throughput из do_capture:")
        print(f"     если raw throughput там был < 95% nominal -- виноват Python")
        print(f"     capture; если raw throughput был близок к 100% -- виноват")
        print(f"     источник данных (линия/прошивка).")
    else:
        print(f"  -> Смешанная картина, нужен дополнительный прогон с")
        print(f"     синтетическим тестовым кадром для однозначной локализации.")
    print(f"{'='*60}\n")

    return dict(
        raw_path=str(raw_path), n_frames=n_frames, counters=counters,
        accel=accel, gyro=gyro, temp=temp, tstamp=tstamp, invalid=invalid,
        valid_per=valid_per, invalid_per=inv_per, t=t,
        counter_gaps=counter_gaps, missing_frames=missing_frames,
        real_fps=real_fps, time_source=time_source, diag=diag,
    )


# =============================================================================
#  PLOT — Plotly (тёмная тема, WebGL Scattergl, открывается в браузере)
# =============================================================================
_BASE_LAYOUT = dict(
    template="plotly_dark",
    paper_bgcolor=BG,
    plot_bgcolor=BG2,
    font=dict(color=TEXT_C, family="JetBrains Mono, Consolas, monospace", size=10),
    hoverlabel=dict(bgcolor=BG2, font_size=11, font_family="Consolas"),
    margin=dict(l=40, r=20, t=60, b=40),
)

def _axis_style():
    return dict(
        showgrid=True, gridcolor=GRID_C, gridwidth=1,
        zeroline=True, zerolinecolor=GRID_C, zerolinewidth=1,
        linecolor=GRID_C, tickfont=dict(size=8),
    )


def _make_grid_fig(subtitle_list, title):
    fig = make_subplots(
        rows=6, cols=6,
        subplot_titles=subtitle_list,
        horizontal_spacing=0.035,
        vertical_spacing=0.07,
    )
    fig.update_layout(
        **_BASE_LAYOUT,
        title=dict(text=title, font=dict(size=16, color=TEXT_C), x=0.5, xanchor="center"),
        height=1500, width=2100,
    )
    for ann in fig.layout.annotations:
        ann.font = dict(size=8, color=C_GRAY)
    fig.update_xaxes(**_axis_style())
    fig.update_yaxes(**_axis_style())
    return fig


def _no_data_annotation(fig, row, col):
    fig.add_annotation(
        text="NO DATA", xref="x domain", yref="y domain",
        x=0.5, y=0.5, showarrow=False,
        font=dict(color=C_RED, size=10, family="Consolas"),
        row=row, col=col,
    )


def plot_accel(d: dict, plot_dir: Path):
    data, t  = d["accel"], d["t"]
    vp, nf   = d["valid_per"], d["n_frames"]
    labels   = ["Ax", "Ay", "Az"]
    colors   = [C_RED, C_BLUE, C_GREEN]
    dashes   = ["solid", "dash", "dot"]

    titles = [f"S{k:02d}  {vp[k]}/{nf}" for k in range(N_SENSORS)]
    fig    = _make_grid_fig(titles, "Accelerometer - все 36 датчиков [g]")

    for k in range(N_SENSORS):
        row, col = k // 6 + 1, k % 6 + 1
        if vp[k] == 0:
            _no_data_annotation(fig, row, col)
            continue
        for ch, (lbl, col_c, ds) in enumerate(zip(labels, colors, dashes)):
            fig.add_trace(go.Scattergl(
                x=t, y=data[:, k, ch],
                mode="lines", name=lbl,
                line=dict(color=col_c, width=0.9, dash=ds),
                showlegend=(k == 0),
                legendgroup=lbl,
                hovertemplate=f"<b>{lbl}</b>: %{{y:.5f}} g<br>t=%{{x:.3f}} s<extra>S{k:02d}</extra>",
            ), row=row, col=col)

    fig.update_layout(legend=dict(
        orientation="h", x=0.5, xanchor="center", y=1.02,
        bgcolor="rgba(0,0,0,0.4)", bordercolor=GRID_C, borderwidth=1,
    ))
    out = plot_dir / "accel.html"
    fig.write_html(str(out), include_plotlyjs="cdn")
    print(f"[MIIB] + {out}")
    return out


def plot_gyro(d: dict, plot_dir: Path):
    data, t  = d["gyro"], d["t"]
    vp, nf   = d["valid_per"], d["n_frames"]
    labels   = ["Gx", "Gy", "Gz"]
    colors   = [C_RED, C_BLUE, C_GREEN]
    dashes   = ["solid", "dash", "dot"]

    titles = [f"S{k:02d}  {vp[k]}/{nf}" for k in range(N_SENSORS)]
    fig    = _make_grid_fig(titles, "Gyroscope - все 36 датчиков [deg/s]")

    for k in range(N_SENSORS):
        row, col = k // 6 + 1, k % 6 + 1
        if vp[k] == 0:
            _no_data_annotation(fig, row, col)
            continue
        for ch, (lbl, col_c, ds) in enumerate(zip(labels, colors, dashes)):
            fig.add_trace(go.Scattergl(
                x=t, y=data[:, k, ch],
                mode="lines", name=lbl,
                line=dict(color=col_c, width=0.9, dash=ds),
                showlegend=(k == 0),
                legendgroup=lbl,
                hovertemplate=f"<b>{lbl}</b>: %{{y:.4f}} deg/s<br>t=%{{x:.3f}} s<extra>S{k:02d}</extra>",
            ), row=row, col=col)

    fig.update_layout(legend=dict(
        orientation="h", x=0.5, xanchor="center", y=1.02,
        bgcolor="rgba(0,0,0,0.4)", bordercolor=GRID_C, borderwidth=1,
    ))
    out = plot_dir / "gyro.html"
    fig.write_html(str(out), include_plotlyjs="cdn")
    print(f"[MIIB] + {out}")
    return out


def plot_temp(d: dict, plot_dir: Path):
    data, t  = d["temp"], d["t"]
    vp, nf   = d["valid_per"], d["n_frames"]

    titles = [f"S{k:02d}  {vp[k]}/{nf}" for k in range(N_SENSORS)]
    fig    = _make_grid_fig(titles, "Temperature - все 36 датчиков [C]")

    for k in range(N_SENSORS):
        row, col = k // 6 + 1, k % 6 + 1
        if vp[k] == 0:
            _no_data_annotation(fig, row, col)
            continue
        fig.add_trace(go.Scattergl(
            x=t, y=data[:, k],
            mode="lines", name=f"S{k:02d}",
            line=dict(color=C_ORG, width=1.0),
            showlegend=False,
            hovertemplate=f"%{{y:.2f}} C  t=%{{x:.3f}} s<extra>S{k:02d}</extra>",
        ), row=row, col=col)

    out = plot_dir / "temperature.html"
    fig.write_html(str(out), include_plotlyjs="cdn")
    print(f"[MIIB] + {out}")
    return out


def plot_timestamp(d: dict, plot_dir: Path):
    data, t  = d["tstamp"], d["t"]
    vp, nf   = d["valid_per"], d["n_frames"]

    titles = [f"S{k:02d}  {vp[k]}/{nf}" for k in range(N_SENSORS)]
    fig    = _make_grid_fig(titles, "Timestamp - все 36 датчиков [LSB]")

    for k in range(N_SENSORS):
        row, col = k // 6 + 1, k % 6 + 1
        if vp[k] == 0:
            _no_data_annotation(fig, row, col)
            continue
        fig.add_trace(go.Scattergl(
            x=t, y=data[:, k],
            mode="lines", name=f"S{k:02d}",
            line=dict(color=C_PURP, width=1.0),
            showlegend=False,
            hovertemplate=f"%{{y:.0f}} LSB  t=%{{x:.3f}} s<extra>S{k:02d}</extra>",
        ), row=row, col=col)

    out = plot_dir / "timestamp.html"
    fig.write_html(str(out), include_plotlyjs="cdn")
    print(f"[MIIB] + {out}")
    return out


def plot_summary(d: dict, plot_dir: Path):
    xlbls = [f"S{k:02d}" for k in range(N_SENSORS)]
    diag: ParseDiag = d["diag"]

    fig = make_subplots(
        rows=2, cols=2,
        subplot_titles=[
            "Valid frames / sensor",
            "Invalid frames / sensor",
            "Delta Counter (должен быть 1)",
            "Sensor health map",
        ],
        horizontal_spacing=0.08,
        vertical_spacing=0.14,
    )
    fig.update_layout(
        **_BASE_LAYOUT,
        title=dict(text="MIIB Summary Dashboard", font=dict(size=17, color=TEXT_C),
                   x=0.5, xanchor="center"),
        height=750, width=1500,
        showlegend=False,
    )
    for ann in fig.layout.annotations:
        ann.font = dict(size=11, color=TEXT_C)

    fig.add_trace(go.Bar(
        x=xlbls, y=d["valid_per"].tolist(),
        marker=dict(color=d["valid_per"].tolist(), colorscale="Greens",
                    line=dict(color=GRID_C, width=0.5)),
        hovertemplate="<b>%{x}</b><br>Valid: %{y}<extra></extra>",
    ), row=1, col=1)

    fig.add_trace(go.Bar(
        x=xlbls, y=d["invalid_per"].tolist(),
        marker=dict(color=d["invalid_per"].tolist(), colorscale="Reds",
                    line=dict(color=GRID_C, width=0.5)),
        hovertemplate="<b>%{x}</b><br>Invalid: %{y}<extra></extra>",
    ), row=1, col=2)

    if d["n_frames"] > 1:
        dcnt = (np.diff(d["counters"].astype(np.int32)) % 65536).tolist()
        fig.add_trace(go.Scattergl(
            y=dcnt, mode="lines",
            line=dict(color=C_BLUE, width=0.8),
            hovertemplate="Frame %{x}: Delta=%{y}<extra></extra>",
        ), row=2, col=1)
        fig.add_hline(y=1, line_dash="dash", line_color=C_GREEN,
                      annotation_text="Expected=1",
                      annotation_font=dict(color=C_GREEN), row=2, col=1)

    health = np.zeros((6, 6))
    for k in range(N_SENSORS):
        r, c = k // 6, k % 6
        pct  = d["valid_per"][k] / max(d["n_frames"], 1) * 100
        health[r, c] = pct

    fig.add_trace(go.Heatmap(
        z=health,
        zmin=0, zmax=100,
        colorscale=[
            [0.0,  "#d32f2f"],
            [0.5,  "#f57c00"],
            [0.85, "#fdd835"],
            [1.0,  "#388e3c"],
        ],
        text=[[f"S{r*6+c:02d}<br>{health[r,c]:.0f}%" for c in range(6)] for r in range(6)],
        texttemplate="%{text}",
        textfont=dict(size=11, color="white"),
        showscale=True,
        colorbar=dict(
            title=dict(text="Valid %", font=dict(color=TEXT_C)),
            tickfont=dict(color=TEXT_C),
        ),
        hovertemplate="<b>%{text}</b><extra></extra>",
    ), row=2, col=2)

    info = (f"File: {d['raw_path']}  |  "
            f"Frames: {d['n_frames']}  |  "
            f"Gaps: {d['counter_gaps']}  |  "
            f"Missing: {d['missing_frames']}  |  "
            f"Rate: {d['real_fps']:.1f} fps  |  "
            f"Duration: {d['t'][-1]:.2f} s  |  "
            f"{d['time_source']}  |  "
            f"CRC-fail: {diag.crc_fail_events}  |  "
            f"Resync: {diag.resync_events}")
    fig.add_annotation(
        text=info, xref="paper", yref="paper",
        x=0.5, y=-0.06, showarrow=False,
        font=dict(size=10, color=C_GRAY, family="Consolas"),
        align="center",
    )

    fig.update_xaxes(**_axis_style())
    fig.update_yaxes(**_axis_style())

    out = plot_dir / "summary.html"
    fig.write_html(str(out), include_plotlyjs="cdn")
    print(f"[MIIB] + {out}")
    return out


def do_plots(d: dict, plot_dir: Path):
    if not HAS_PLOTLY:
        print("[MIIB] Plots skipped - pip install plotly")
        return

    plot_dir.mkdir(parents=True, exist_ok=True)
    print(f"\n[MIIB] Building plots -> {plot_dir}")

    files = []
    files.append(plot_accel    (d, plot_dir))
    files.append(plot_gyro     (d, plot_dir))
    files.append(plot_temp     (d, plot_dir))
    files.append(plot_timestamp(d, plot_dir))
    files.append(plot_summary  (d, plot_dir))

    print(f"\n[MIIB] All plots saved to: {plot_dir}")
    webbrowser.open(str(files[-1].resolve()))


# =============================================================================
#  CLI
# =============================================================================
def build_parser():
    p = argparse.ArgumentParser(
        prog="miib_capture",
        description="MIIB v5: reliable capture -> parse -> interactive Plotly plots",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    sub = p.add_subparsers(dest="cmd", required=True)

    cap = sub.add_parser("capture", help="Захват из UART + парсинг + графики")
    cap.add_argument("--port",     required=True, help="COM3 или /dev/ttyUSB0")
    cap.add_argument("--baud",     type=int,   default=12_000_000)
    cap.add_argument("--duration", type=float, default=30.0)
    cap.add_argument("--outfile",  default="", help="Путь к .bin (авто если пусто)")
    cap.add_argument("--chunk",    type=int,   default=65536,
                      help="Максимальный размер одного readinto()")
    cap.add_argument("--read-timeout", type=float, default=0.02, dest="read_timeout",
                      help="Блокирующий timeout pyserial на readinto(), сек")
    cap.add_argument("--queue-max", type=int, default=4096, dest="queue_max",
                      help="Максимальный размер очереди чанков reader->writer")
    cap.add_argument("--no-parse", action="store_true", dest="no_parse")
    cap.add_argument("--no-plots", action="store_true", dest="no_plots")
    cap.add_argument("--plot-dir", default="", dest="plot_dir")

    prs = sub.add_parser("parse", help="Парсинг + графики из .bin")
    prs.add_argument("--infile",   required=True)
    prs.add_argument("--no-plots", action="store_true", dest="no_plots")
    prs.add_argument("--plot-dir", default="", dest="plot_dir")

    return p


def main_cli():
    args = build_parser().parse_args()
    ts   = time.strftime("%Y%m%d_%H%M%S")

    if args.cmd == "capture":
        raw_path, elapsed, _stats = do_capture(args)
        if args.no_parse:
            return
        d = do_parse(raw_path, capture_elapsed=elapsed)
        if not args.no_plots:
            pdir = Path(args.plot_dir) if args.plot_dir else Path(f"miib_plots_{ts}")
            do_plots(d, pdir)

    elif args.cmd == "parse":
        d = do_parse(Path(args.infile), capture_elapsed=None)
        if not args.no_plots:
            pdir = Path(args.plot_dir) if args.plot_dir \
                   else Path(args.infile).with_suffix("") / f"plots_{ts}"
            do_plots(d, pdir)


if __name__ == "__main__":
    # ==================================================================
    #  SPYDER / прямой запуск - настрой здесь и жми Run (F5)
    # ==================================================================
    SPYDER_MODE = True          # True = ручная конфигурация ниже

    if SPYDER_MODE or len(sys.argv) == 1:

        MODE = "capture"        # "capture" или "parse"

        # -- НАСТРОЙКИ ЗАХВАТА --------------------------------------
        PORT         = "COM11"
        BAUD         = 12_000_000
        DURATION     = 30.0
        OUTFILE      = ""
        CHUNK        = 65536
        READ_TIMEOUT = 0.02
        QUEUE_MAX    = 4096
        NO_PARSE     = False
        NO_PLOTS     = False

        # -- НАСТРОЙКИ ПАРСИНГА (если MODE = "parse") ---------------
        INFILE   = r"C:\Users\17082\OneDrive\Рабочий стол\Final version of MIIB\Final version of MIIB GIT\miib_raw_20260815_200255.bin"
        PLOT_DIR = ""

        ts = time.strftime("%Y%m%d_%H%M%S")

        if MODE == "capture":
            class _Args:
                port         = PORT
                baud         = BAUD
                duration     = DURATION
                outfile      = OUTFILE
                chunk        = CHUNK
                read_timeout = READ_TIMEOUT
                queue_max    = QUEUE_MAX
                no_parse     = NO_PARSE
                no_plots     = NO_PLOTS
                plot_dir     = PLOT_DIR

            args = _Args()
            raw_path, elapsed, _stats = do_capture(args)

            if not args.no_parse:
                d = do_parse(raw_path, capture_elapsed=elapsed)
                if not args.no_plots:
                    pdir = Path(args.plot_dir) if args.plot_dir \
                           else Path(f"miib_plots_{ts}")
                    do_plots(d, pdir)

        elif MODE == "parse":
            d = do_parse(Path(INFILE), capture_elapsed=None)
            if not NO_PLOTS:
                pdir = Path(PLOT_DIR) if PLOT_DIR \
                       else Path(INFILE).with_suffix("") / f"plots_{ts}"
                do_plots(d, pdir)

    else:
        main_cli()