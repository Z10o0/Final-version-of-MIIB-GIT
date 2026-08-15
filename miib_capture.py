#!/usr/bin/env python3
"""
miib.py  —  MIIB unified tool: capture → offline parse → interactive Plotly plots

Wire format (690 bytes):
  [0..1]    Header 0xAA 0x55
  [2..3]    frame_counter uint16 LE
  [4..687]  36 × 19-byte HIRES IMU block
  [688..689] CRC16-CCITT LE  (covers bytes [2..687])

Прибор реально шлёт кадры на НОМИНАЛЬНОЙ частоте 1600 fps (FRAMES_PER_BATCH=16 × BATCHES_PER_SEC=100).
Временная ось строится по РЕАЛЬНОМУ времени захвата (elapsed/n_frames), а не по фиктивному
делению на BATCHES_PER_SEC — это был баг, растягивавший ось X в ~16 раз.

Install:
  pip install pyserial numpy plotly

Usage:
  python miib.py capture --port COM3 --duration 30
  python miib.py capture --port COM3 --duration 60 --no-parse
  python miib.py parse   --infile miib_raw_20260814_200000.bin
"""

import argparse
import os
import sys
import time
import struct
import webbrowser
from pathlib import Path

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


# ═══════════════════════════════════════════════════════════════════════════════
#  Протокол
# ═══════════════════════════════════════════════════════════════════════════════
FRAME_LEN        = 690
N_SENSORS        = 36
IMU_BYTES        = 19
OFF_COUNTER      = 2
OFF_SAMPLES      = 4
OFF_CRC          = 688
PAYLOAD_LEN      = 686

FRAMES_PER_BATCH = 16
BATCHES_PER_SEC  = 100
NOMINAL_FPS      = FRAMES_PER_BATCH * BATCHES_PER_SEC     # 1600 fps — номинал прибора
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


# ═══════════════════════════════════════════════════════════════════════════════
#  CRC-16/CCITT-FALSE
# ═══════════════════════════════════════════════════════════════════════════════
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


# ═══════════════════════════════════════════════════════════════════════════════
#  CAPTURE
# ═══════════════════════════════════════════════════════════════════════════════
def do_capture(args) -> tuple[Path, float]:
    """Возвращает (путь_к_файлу, реальное_время_захвата_сек)."""
    if serial is None:
        sys.exit("ERROR: pip install pyserial")

    ts       = time.strftime("%Y%m%d_%H%M%S")
    out_path = Path(args.outfile) if args.outfile else Path(f"miib_raw_{ts}.bin")

    ser = serial.Serial()
    ser.port = args.port;  ser.baudrate = args.baud
    ser.bytesize = serial.EIGHTBITS;  ser.parity = serial.PARITY_NONE
    ser.stopbits = serial.STOPBITS_ONE;  ser.timeout = 0
    ser.rtscts = ser.dsrdtr = ser.xonxoff = False
    ser.open()

    elapsed = 0.0
    try:
        try:
            ser.set_buffer_size(rx_size=16 * 1024 * 1024, tx_size=64 * 1024)
        except Exception:
            pass
        ser.reset_input_buffer()

        total = last_total = 0
        t0 = t_last = time.perf_counter()

        print(f"[MIIB] Port    : {args.port} @ {args.baud} baud")
        print(f"[MIIB] Frame   : {FRAME_LEN} bytes  ({N_SENSORS} sensors × {IMU_BYTES} B)")
        print(f"[MIIB] Nominal : {NOMINAL_FPS} fps  →  {EXPECTED_BPS} B/s  ({EXPECTED_BPS/1024:.1f} KiB/s)")
        print(f"[MIIB] Duration: {args.duration:.1f} s  →  Output: {out_path}")

        with open(out_path, "wb", buffering=1024 * 1024) as f:
            while True:
                now = time.perf_counter()
                if now - t0 >= args.duration:
                    break
                waiting = ser.in_waiting
                if waiting > 0:
                    data = ser.read(min(waiting, args.chunk))
                    if data:
                        f.write(data);  total += len(data)
                else:
                    time.sleep(args.poll)
                if now - t_last >= 1.0:
                    el = now - t0
                    dt = now - t_last
                    print(f"[MIIB] t={el:6.2f}s | "
                          f"inst={(total - last_total)/dt/1024:8.1f} KiB/s | "
                          f"avg={total/el/1024:8.1f} KiB/s | "
                          f"total={total:10d} B")
                    t_last, last_total = now, total

            time.sleep(0.05)
            while ser.in_waiting:
                data = ser.read(min(ser.in_waiting, args.chunk))
                if not data: break
                f.write(data);  total += len(data)
            f.flush();  os.fsync(f.fileno())

        elapsed = time.perf_counter() - t0
        bps     = total / max(elapsed, 1e-9)
        print(f"[MIIB] Done   : {total} B in {elapsed:.3f} s = "
              f"{bps/1024:.1f} KiB/s ({bps/EXPECTED_BPS*100:.2f}% of nominal)")
        print(f"[MIIB] Saved  : {out_path}")
    finally:
        try: ser.close()
        except Exception: pass

    return out_path, elapsed


# ═══════════════════════════════════════════════════════════════════════════════
#  PARSE
# ═══════════════════════════════════════════════════════════════════════════════
def _s20(u: int) -> int:
    return u - (1 << 20) if u >= (1 << 19) else u

def do_parse(raw_path: Path, capture_elapsed: float | None = None) -> dict:
    """
    capture_elapsed — реальное время захвата в секундах (из do_capture()).
    Используется для построения честной временной оси t.
    Если None (например, при офлайн-парсинге старого .bin без метаданных времени),
    используется НОМИНАЛЬНАЯ частота прибора NOMINAL_FPS=1600 как fallback.
    """
    raw = Path(raw_path).read_bytes()
    buf = np.frombuffer(raw, dtype=np.uint8)

    is_hdr  = (buf[:-1] == 0xAA) & (buf[1:] == 0x55)
    hdr_pos = np.where(is_hdr)[0]
    hdr_pos = hdr_pos[hdr_pos + FRAME_LEN <= len(raw)]

    valid_starts, i = [], 0
    while i < len(hdr_pos):
        pos   = int(hdr_pos[i])
        frame = raw[pos: pos + FRAME_LEN]
        if crc16(frame[OFF_COUNTER: OFF_COUNTER + PAYLOAD_LEN]) == \
               struct.unpack_from("<H", frame, OFF_CRC)[0]:
            valid_starts.append(pos)
            j = int(np.searchsorted(hdr_pos, pos + FRAME_LEN))
            if j >= len(hdr_pos): break
            i = j
        else:
            i += 1

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
                invalid[fi, k] = True;  continue

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

    # ── ИСПРАВЛЕНО: временная ось строится по реальному времени захвата,
    #    а НЕ через фиктивное деление counter-delta на BATCHES_PER_SEC.
    if capture_elapsed and capture_elapsed > 0:
        dt_frames    = capture_elapsed / n_frames
        time_source  = f"measured ({n_frames} frames / {capture_elapsed:.3f} s capture)"
    else:
        dt_frames    = 1.0 / NOMINAL_FPS
        time_source  = f"nominal ({NOMINAL_FPS} fps fallback — no capture time available)"

    t          = np.arange(n_frames) * dt_frames
    real_fps   = 1.0 / dt_frames
    valid_per  = (~invalid).sum(axis=0)
    inv_per    = invalid.sum(axis=0)

    print(f"\n{'═'*48}")
    print(f"MIIB SUMMARY")
    print(f"{'═'*48}")
    print(f"  Valid frames   : {n_frames}")
    print(f"  Counter gaps   : {counter_gaps}   Missing: {missing_frames}")
    print(f"  Time source    : {time_source}")
    print(f"  Frame rate     : {real_fps:.2f} fps  (nominal: {NOMINAL_FPS} fps)")
    print(f"  Duration       : {t[-1]:.3f} s")
    print(f"{'─'*48}")
    print(f"  {'Sensor':<8} {'Valid':>8} {'Invalid':>8}  {'Status'}")
    for k in range(N_SENSORS):
        status = "✓ OK" if inv_per[k] == 0 else ("✗ DEAD" if valid_per[k] == 0 else "⚠ ERR")
        print(f"  S{k:02d}      {valid_per[k]:8d}   {inv_per[k]:8d}  {status}")
    print(f"{'═'*48}\n")

    return dict(
        raw_path=str(raw_path), n_frames=n_frames, counters=counters,
        accel=accel, gyro=gyro, temp=temp, tstamp=tstamp, invalid=invalid,
        valid_per=valid_per, invalid_per=inv_per, t=t,
        counter_gaps=counter_gaps, missing_frames=missing_frames,
        real_fps=real_fps, time_source=time_source,
    )


# ═══════════════════════════════════════════════════════════════════════════════
#  PLOT — Plotly  (тёмная тема, WebGL Scattergl, открывается в браузере)
# ═══════════════════════════════════════════════════════════════════════════════
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
    fig    = _make_grid_fig(titles, "🔵 Accelerometer — все 36 датчиков [g]")

    for k in range(N_SENSORS):
        row, col = k // 6 + 1, k % 6 + 1
        if vp[k] == 0:
            _no_data_annotation(fig, row, col);  continue
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
    print(f"[MIIB] ✓ {out}")
    return out


def plot_gyro(d: dict, plot_dir: Path):
    data, t  = d["gyro"], d["t"]
    vp, nf   = d["valid_per"], d["n_frames"]
    labels   = ["Gx", "Gy", "Gz"]
    colors   = [C_RED, C_BLUE, C_GREEN]
    dashes   = ["solid", "dash", "dot"]

    titles = [f"S{k:02d}  {vp[k]}/{nf}" for k in range(N_SENSORS)]
    fig    = _make_grid_fig(titles, "🟢 Gyroscope — все 36 датчиков [°/s]")

    for k in range(N_SENSORS):
        row, col = k // 6 + 1, k % 6 + 1
        if vp[k] == 0:
            _no_data_annotation(fig, row, col);  continue
        for ch, (lbl, col_c, ds) in enumerate(zip(labels, colors, dashes)):
            fig.add_trace(go.Scattergl(
                x=t, y=data[:, k, ch],
                mode="lines", name=lbl,
                line=dict(color=col_c, width=0.9, dash=ds),
                showlegend=(k == 0),
                legendgroup=lbl,
                hovertemplate=f"<b>{lbl}</b>: %{{y:.4f}} °/s<br>t=%{{x:.3f}} s<extra>S{k:02d}</extra>",
            ), row=row, col=col)

    fig.update_layout(legend=dict(
        orientation="h", x=0.5, xanchor="center", y=1.02,
        bgcolor="rgba(0,0,0,0.4)", bordercolor=GRID_C, borderwidth=1,
    ))
    out = plot_dir / "gyro.html"
    fig.write_html(str(out), include_plotlyjs="cdn")
    print(f"[MIIB] ✓ {out}")
    return out


def plot_temp(d: dict, plot_dir: Path):
    data, t  = d["temp"], d["t"]
    vp, nf   = d["valid_per"], d["n_frames"]

    titles = [f"S{k:02d}  {vp[k]}/{nf}" for k in range(N_SENSORS)]
    fig    = _make_grid_fig(titles, "🟠 Temperature — все 36 датчиков [°C]")

    for k in range(N_SENSORS):
        row, col = k // 6 + 1, k % 6 + 1
        if vp[k] == 0:
            _no_data_annotation(fig, row, col);  continue
        fig.add_trace(go.Scattergl(
            x=t, y=data[:, k],
            mode="lines", name=f"S{k:02d}",
            line=dict(color=C_ORG, width=1.0),
            showlegend=False,
            hovertemplate=f"%{{y:.2f}} °C  t=%{{x:.3f}} s<extra>S{k:02d}</extra>",
        ), row=row, col=col)

    out = plot_dir / "temperature.html"
    fig.write_html(str(out), include_plotlyjs="cdn")
    print(f"[MIIB] ✓ {out}")
    return out


def plot_timestamp(d: dict, plot_dir: Path):
    data, t  = d["tstamp"], d["t"]
    vp, nf   = d["valid_per"], d["n_frames"]

    titles = [f"S{k:02d}  {vp[k]}/{nf}" for k in range(N_SENSORS)]
    fig    = _make_grid_fig(titles, "🟣 Timestamp — все 36 датчиков [LSB]")

    for k in range(N_SENSORS):
        row, col = k // 6 + 1, k % 6 + 1
        if vp[k] == 0:
            _no_data_annotation(fig, row, col);  continue
        fig.add_trace(go.Scattergl(
            x=t, y=data[:, k],
            mode="lines", name=f"S{k:02d}",
            line=dict(color=C_PURP, width=1.0),
            showlegend=False,
            hovertemplate=f"%{{y:.0f}} LSB  t=%{{x:.3f}} s<extra>S{k:02d}</extra>",
        ), row=row, col=col)

    out = plot_dir / "timestamp.html"
    fig.write_html(str(out), include_plotlyjs="cdn")
    print(f"[MIIB] ✓ {out}")
    return out


def plot_summary(d: dict, plot_dir: Path):
    xlbls = [f"S{k:02d}" for k in range(N_SENSORS)]

    fig = make_subplots(
        rows=2, cols=2,
        subplot_titles=[
            "✅ Valid frames / sensor",
            "❌ Invalid frames / sensor",
            "Δ Counter (должен быть 1)",
            "Sensor health map",
        ],
        horizontal_spacing=0.08,
        vertical_spacing=0.14,
    )
    fig.update_layout(
        **_BASE_LAYOUT,
        title=dict(text="📊 MIIB Summary Dashboard", font=dict(size=17, color=TEXT_C),
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
            hovertemplate="Frame %{x}: Δ=%{y}<extra></extra>",
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
            f"{d['time_source']}")
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
    print(f"[MIIB] ✓ {out}")
    return out


def do_plots(d: dict, plot_dir: Path):
    if not HAS_PLOTLY:
        print("[MIIB] Plots skipped — pip install plotly");  return

    plot_dir.mkdir(parents=True, exist_ok=True)
    print(f"\n[MIIB] Building plots → {plot_dir}")

    files = []
    files.append(plot_accel    (d, plot_dir))
    files.append(plot_gyro     (d, plot_dir))
    files.append(plot_temp     (d, plot_dir))
    files.append(plot_timestamp(d, plot_dir))
    files.append(plot_summary  (d, plot_dir))

    print(f"\n[MIIB] All plots saved to: {plot_dir}")
    webbrowser.open(str(files[-1].resolve()))


# ═══════════════════════════════════════════════════════════════════════════════
#  CLI
# ═══════════════════════════════════════════════════════════════════════════════
def build_parser():
    p = argparse.ArgumentParser(
        prog="miib",
        description="MIIB: capture → parse → interactive Plotly plots",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    sub = p.add_subparsers(dest="cmd", required=True)

    cap = sub.add_parser("capture", help="Захват из UART + парсинг + графики")
    cap.add_argument("--port",     required=True, help="COM3 или /dev/ttyUSB0")
    cap.add_argument("--baud",     type=int,   default=12_000_000)
    cap.add_argument("--duration", type=float, default=30.0)
    cap.add_argument("--outfile",  default="", help="Путь к .bin (авто если пусто)")
    cap.add_argument("--chunk",    type=int,   default=65536)
    cap.add_argument("--poll",     type=float, default=0.0005)
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
        raw_path, elapsed = do_capture(args)
        if args.no_parse: return
        d = do_parse(raw_path, capture_elapsed=elapsed)
        if not args.no_plots:
            pdir = Path(args.plot_dir) if args.plot_dir else Path(f"miib_plots_{ts}")
            do_plots(d, pdir)

    elif args.cmd == "parse":
        # Офлайн-парсинг без метаданных времени → используется NOMINAL_FPS fallback
        d = do_parse(Path(args.infile), capture_elapsed=None)
        if not args.no_plots:
            pdir = Path(args.plot_dir) if args.plot_dir \
                   else Path(args.infile).with_suffix("") / f"plots_{ts}"
            do_plots(d, pdir)


if __name__ == "__main__":
    # ══════════════════════════════════════════════════════════════
    #  SPYDER / прямой запуск — настрой здесь и жми Run (F5)
    # ══════════════════════════════════════════════════════════════
    SPYDER_MODE = True          # True = ручная конфигурация ниже

    if SPYDER_MODE or len(sys.argv) == 1:

        MODE = "capture"        # "capture" или "parse"

        # ── НАСТРОЙКИ ЗАХВАТА ────────────────────────────────────
        PORT     = "COM11"
        BAUD     = 12_000_000
        DURATION = 10.0
        OUTFILE  = ""
        CHUNK    = 65536
        POLL     = 0.0005
        NO_PARSE = False
        NO_PLOTS = False

        # ── НАСТРОЙКИ ПАРСИНГА (если MODE = "parse") ────────────
        INFILE   = r"C:\Users\17082\OneDrive\Рабочий стол\Final version of MIIB\Final version of MIIB GIT\miib_raw_20260815_200255.bin"
        PLOT_DIR = ""

        ts = time.strftime("%Y%m%d_%H%M%S")

        if MODE == "capture":
            class _Args:
                port     = PORT
                baud     = BAUD
                duration = DURATION
                outfile  = OUTFILE
                chunk    = CHUNK
                poll     = POLL
                no_parse = NO_PARSE
                no_plots = NO_PLOTS
                plot_dir = PLOT_DIR

            args = _Args()
            raw_path, elapsed = do_capture(args)

            if not args.no_parse:
                d = do_parse(raw_path, capture_elapsed=elapsed)
                if not args.no_plots:
                    pdir = Path(args.plot_dir) if args.plot_dir \
                           else Path(f"miib_plots_{ts}")
                    do_plots(d, pdir)

        elif MODE == "parse":
            # ВАЖНО: при офлайн-парсинге старого .bin реального elapsed нет,
            # поэтому временная ось строится по NOMINAL_FPS=1600 fallback.
            d = do_parse(Path(INFILE), capture_elapsed=None)
            if not NO_PLOTS:
                pdir = Path(PLOT_DIR) if PLOT_DIR \
                       else Path(INFILE).with_suffix("") / f"plots_{ts}"
                do_plots(d, pdir)

    else:
        main_cli()