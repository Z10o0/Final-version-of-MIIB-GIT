# ============================================================================
# ВАРИАЦИЯ АЛЛАНА МАССИВА IMU
# CUDA GPU + HDF5 + RAM float32 + расчёт float64
#
# Ожидаемый HDF5 layout:
#   gyro:   [n_samples, n_sensors, 3]
#   accel:  [n_samples, n_sensors, 3]
#   tstamp: [n_samples, n_sensors]
#
# Единицы входных данных:
#   gyro  : град/с
#   accel : g
#
# Единицы результатов:
#   Гироскоп:
#       ADEV: град/ч
#       ARW : град/√ч
#       BI  : град/ч
#
#   Акселерометр:
#       ADEV: mg
#       VRW : м/с/√ч
#       BI  : мкg
#
# ВАЖНО:
#   - IMU №5 исключён из всех расчётов.
#   - X/Y загружаются совместно в RAM.
#   - Z загружается отдельным проходом.
#   - Не используются memmap и временные файлы на SSD.
#
# Установка:
#   pip install numpy h5py matplotlib cupy-cuda12x
# ============================================================================

import os
import gc
import time
import math

import h5py
import numpy as np
import cupy as cp
import matplotlib.pyplot as plt


# ============================================================================
# НАСТРОЙКИ
# ============================================================================

H5_FILE_PATH = 'miib_data_single_12h.h5'
OUTPUT_DIR = 'output'

# Нумерация человеческая: 1, 2, 3, ..., 36.
EXCLUDED_IMU_1BASED = [5]

# Сетка tau.
PTS_PER_DECADE = 45
MAX_TAU_SECONDS = 10_000.0
MIN_OVERLAPS = 100_000

# HDF5 / RAM.
HDF5_TIME_BLOCK = 4_000_000
RAM_AXIS_DTYPE = np.float32

# GPU.
GPU_SENSOR_BATCH = 6
GPU_TIME_CHUNK = 8_000_000
GPU_MEMORY_FRACTION = 0.90

MIN_TOTAL_VRAM_GB = 5.70
MIN_FREE_VRAM_GB = 4.40

# Вывод.
TAU_PROGRESS_STEP = 25
SAVE_NUMERIC_RESULTS = True
PLOT_DPI = 150

# Физические константы.
G_STANDARD = 9.80665
BI_FLICKER_FACTOR = 0.664282


# ============================================================================
# GPU
# ============================================================================

def get_gpu_info():
    """Проверка GPU и доступной VRAM."""

    try:
        device_id = cp.cuda.runtime.getDevice()
        props = cp.cuda.runtime.getDeviceProperties(device_id)
        free_bytes, total_bytes = cp.cuda.runtime.memGetInfo()
    except Exception as exc:
        raise RuntimeError(
            '\nCUDA GPU недоступен.\n'
            'Проверь драйвер NVIDIA и установленный CuPy.\n'
            'Для CUDA 12 используй:\n'
            'pip install cupy-cuda12x'
        ) from exc

    gpu_name = props['name']

    if isinstance(gpu_name, bytes):
        gpu_name = gpu_name.decode('utf-8', errors='replace')
    else:
        gpu_name = str(gpu_name)

    total_gb = total_bytes / (1024 ** 3)
    free_gb = free_bytes / (1024 ** 3)

    print('========================= CUDA GPU =========================')
    print(f'GPU:                         {gpu_name}')
    print(f'Compute Capability:          {props["major"]}.{props["minor"]}')
    print(f'Всего VRAM:                  {total_gb:.3f} GB')
    print(f'Свободно VRAM:               {free_gb:.3f} GB')
    print(f'IMU в GPU-пакете:            {GPU_SENSOR_BATCH}')
    print(f'Буфер GPU-разностей:         {GPU_TIME_CHUNK:,} кадров')
    print('Расчёт theta и ADEV:         float64')
    print('Хранение осей в RAM:         float32')
    print('Временный SSD-кэш:           нет')
    print('============================================================\n')

    if total_gb < MIN_TOTAL_VRAM_GB:
        raise RuntimeError(
            f'Недостаточный общий объём VRAM: {total_gb:.3f} GB.'
        )

    if free_gb < MIN_FREE_VRAM_GB:
        raise RuntimeError(
            f'Свободно VRAM: {free_gb:.3f} GB.\n'
            f'Для GPU_SENSOR_BATCH={GPU_SENSOR_BATCH} требуется '
            f'минимум {MIN_FREE_VRAM_GB:.2f} GB.\n\n'
            'Закрой программы, использующие GPU, либо снизь:\n'
            'GPU_SENSOR_BATCH = 5'
        )


def release_gpu_memory():
    """Освобождение пулов памяти CuPy."""

    cp.cuda.Stream.null.synchronize()
    cp.get_default_memory_pool().free_all_blocks()
    cp.get_default_pinned_memory_pool().free_all_blocks()
    gc.collect()


def check_gpu_memory(n_samples, batch_size):
    """Консервативная проверка VRAM."""

    free_bytes, _ = cp.cuda.runtime.memGetInfo()

    time_chunk = min(GPU_TIME_CHUNK, n_samples)

    theta_bytes = batch_size * (n_samples + 1) * 8
    d2_bytes = batch_size * time_chunk * 8
    upload_bytes = batch_size * min(HDF5_TIME_BLOCK, n_samples) * 8
    reserve_bytes = 300 * 1024 * 1024

    required_bytes = (
        theta_bytes
        + d2_bytes
        + upload_bytes
        + reserve_bytes
    )

    allowed_bytes = int(free_bytes * GPU_MEMORY_FRACTION)

    if required_bytes > allowed_bytes:
        raise MemoryError(
            '\nНедостаточно VRAM для текущего GPU-пакета.\n'
            f'Оценка потребности: {required_bytes / (1024 ** 3):.2f} GiB\n'
            f'Допустимо:           {allowed_bytes / (1024 ** 3):.2f} GiB\n\n'
            'Уменьши GPU_SENSOR_BATCH, например до 5.'
        )


# ============================================================================
# HDF5
# ============================================================================

def detect_3d_layout(dset):
    """
    Определяет оси [время, датчик, XYZ].

    Для вашего файла должен получиться layout [0, 1, 2].
    """

    shape = dset.shape

    if len(shape) != 3:
        raise ValueError(
            f'Ожидался 3D-датасет, фактическая форма: {shape}'
        )

    xyz_candidates = [
        axis
        for axis, dimension in enumerate(shape)
        if dimension == 3
    ]

    if len(xyz_candidates) != 1:
        raise ValueError(
            f'Не удалось определить XYZ-ось: {shape}'
        )

    xyz_axis = xyz_candidates[0]

    remaining_axes = [
        (axis, shape[axis])
        for axis in range(3)
        if axis != xyz_axis
    ]

    remaining_axes.sort(key=lambda x: x[1])

    sensor_axis = remaining_axes[0][0]
    time_axis = remaining_axes[1][0]

    return (
        time_axis,
        sensor_axis,
        xyz_axis,
        int(shape[time_axis]),
        int(shape[sensor_axis])
    )


def detect_tstamp_layout(dset, n_sensors):
    """Определяет оси timestamp-датасета."""

    if len(dset.shape) != 2:
        raise ValueError(
            f'tstamp должен быть двумерным: {dset.shape}'
        )

    if dset.shape[1] == n_sensors:
        return 0, 1

    if dset.shape[0] == n_sensors:
        return 1, 0

    raise ValueError(
        f'Не удалось определить layout tstamp: {dset.shape}'
    )


def estimate_sample_period(h5_path, n_sensors, n_samples):
    """
    Оценивает dt0 по timestamp.

    Предположение:
    - timestamp выражен в микросекундах;
    - переполнение 16-битного счётчика: 65536.
    """

    with h5py.File(h5_path, 'r') as h5_file:
        tstamp = h5_file['tstamp']

        time_axis, _ = detect_tstamp_layout(
            tstamp,
            n_sensors
        )

        n_check = min(50_000, n_samples)

        if time_axis == 0:
            timestamps = np.asarray(
                tstamp[:n_check, :],
                dtype=np.float64
            ).T
        else:
            timestamps = np.asarray(
                tstamp[:, :n_check],
                dtype=np.float64
            )

    nominal_ticks = np.zeros(n_sensors, dtype=np.float64)
    jitter_std = np.zeros(n_sensors, dtype=np.float64)

    for sensor_idx in range(n_sensors):
        sensor_timestamps = timestamps[sensor_idx]

        dt_ticks = np.mod(
            np.diff(sensor_timestamps),
            65536.0
        )

        dt_ticks = dt_ticks[dt_ticks > 0.0]

        if dt_ticks.size == 0:
            continue

        rounded_ticks = np.round(dt_ticks)

        values, counts = np.unique(
            rounded_ticks,
            return_counts=True
        )

        mode_tick = values[np.argmax(counts)]

        nominal_ticks[sensor_idx] = mode_tick

        stable_ticks = dt_ticks[
            dt_ticks < mode_tick * 1.5
        ]

        if stable_ticks.size > 0:
            jitter_std[sensor_idx] = np.std(stable_ticks)

    valid_ticks = nominal_ticks[
        nominal_ticks > 0.0
    ]

    if valid_ticks.size == 0:
        raise RuntimeError(
            'Не удалось определить частоту дискретизации.'
        )

    values, counts = np.unique(
        np.round(valid_ticks),
        return_counts=True
    )

    mean_ticks = values[np.argmax(counts)]

    fs = 1e6 / mean_ticks
    dt0 = 1.0 / fs

    return (
        dt0,
        fs,
        mean_ticks,
        nominal_ticks,
        jitter_std
    )


# ============================================================================
# СЕТКА TAU
# ============================================================================

def build_tau_grid(n_samples, dt0):
    """Создаёт логарифмическую сетку времени усреднения tau."""

    max_m_from_overlaps = (
        n_samples - MIN_OVERLAPS
    ) // 2

    if max_m_from_overlaps < 2:
        raise ValueError(
            'MIN_OVERLAPS слишком велик для длины записи.'
        )

    max_m_from_tau = int(
        math.floor(MAX_TAU_SECONDS / dt0)
    )

    max_m = min(
        max_m_from_overlaps,
        max_m_from_tau
    )

    max_m = max(2, max_m)

    n_decades = np.log10(max_m)

    n_tau_requested = max(
        3,
        int(round(n_decades * PTS_PER_DECADE))
    )

    m_steps = np.unique(
        np.round(
            np.logspace(
                0.0,
                np.log10(max_m),
                n_tau_requested
            )
        ).astype(np.int64)
    )

    tau_vals = m_steps.astype(np.float64) * dt0

    idx_tau_1s = int(
        np.argmin(np.abs(tau_vals - 1.0))
    )

    return m_steps, tau_vals, idx_tau_1s


# ============================================================================
# СОСТАВ МАССИВА
# ============================================================================

def build_active_sensor_mask(n_sensors):
    """Строит маску активных и исключённых IMU."""

    excluded_1based = np.asarray(
        EXCLUDED_IMU_1BASED,
        dtype=np.int64
    )

    excluded_zero_based = excluded_1based - 1

    if np.any(excluded_zero_based < 0):
        raise ValueError(
            'Номера IMU должны начинаться с 1.'
        )

    if np.any(excluded_zero_based >= n_sensors):
        raise ValueError(
            f'Допустимые номера IMU: от 1 до {n_sensors}.'
        )

    active_mask = np.ones(
        n_sensors,
        dtype=bool
    )

    active_mask[excluded_zero_based] = False

    active_indices = np.flatnonzero(active_mask)

    if active_indices.size == 0:
        raise RuntimeError(
            'Не осталось активных IMU.'
        )

    return (
        active_mask,
        active_indices,
        excluded_zero_based
    )


def get_excluded_imu_text():
    """Формирует текст исключённых IMU."""

    if len(EXCLUDED_IMU_1BASED) == 0:
        return 'нет'

    return ', '.join(
        str(number)
        for number in EXCLUDED_IMU_1BASED
    )


# ============================================================================
# HDF5 -> RAM
# ============================================================================

def print_ram_plan(n_samples, n_sensors, n_axes):
    """Печатает оценку требуемой RAM."""

    axis_bytes = (
        n_samples
        * n_sensors
        * np.dtype(RAM_AXIS_DTYPE).itemsize
    )

    axes_gib = (
        n_axes
        * axis_bytes
        / (1024 ** 3)
    )

    read_buffer_bytes = (
        min(HDF5_TIME_BLOCK, n_samples)
        * n_sensors
        * n_axes
        * np.dtype(RAM_AXIS_DTYPE).itemsize
    )

    read_buffer_gib = (
        read_buffer_bytes / (1024 ** 3)
    )

    print('====================== ПЛАН RAM ===========================')
    print(f'Одновременно загружаемых осей: {n_axes}')
    print(f'RAM для постоянных осей:       {axes_gib:.3f} GiB')
    print(f'RAM для HDF5-буфера:           {read_buffer_gib:.3f} GiB')
    print(f'Тип данных в RAM:              {RAM_AXIS_DTYPE.__name__}')
    print('Временные файлы SSD:           нет')
    print('============================================================\n')


def load_axes_to_ram(
    dset,
    axis_indices,
    n_samples,
    n_sensors,
    dataset_label
):
    """
    Загружает одну или две соседние XYZ-оси в RAM.

    Например:
    axis_indices=[0, 1] читает HDF5-срез:
        dset[start:stop, :, 0:2]
    """

    axis_indices = list(axis_indices)

    if len(axis_indices) not in (1, 2):
        raise ValueError(
            'Можно загрузить только одну или две оси.'
        )

    expected_axes = list(
        range(
            axis_indices[0],
            axis_indices[0] + len(axis_indices)
        )
    )

    if axis_indices != expected_axes:
        raise ValueError(
            'Оси должны быть последовательными.'
        )

    n_axes = len(axis_indices)

    print_ram_plan(
        n_samples=n_samples,
        n_sensors=n_sensors,
        n_axes=n_axes
    )

    try:
        axis_arrays = [
            np.empty(
                (n_samples, n_sensors),
                dtype=RAM_AXIS_DTYPE,
                order='C'
            )
            for _ in axis_indices
        ]
    except MemoryError as exc:
        raise MemoryError(
            '\nНе удалось выделить RAM для осей.\n'
            'Перезапусти kernel Spyder и освободи память.'
        ) from exc

    max_rows = min(HDF5_TIME_BLOCK, n_samples)

    try:
        read_buffer = np.empty(
            (max_rows, n_sensors, n_axes),
            dtype=RAM_AXIS_DTYPE,
            order='C'
        )
    except MemoryError as exc:
        axis_arrays.clear()
        gc.collect()

        raise MemoryError(
            '\nНе удалось выделить HDF5-буфер.\n'
            'Уменьши HDF5_TIME_BLOCK, например до 3_000_000.'
        ) from exc

    axis_first = axis_indices[0]
    axis_last = axis_indices[-1] + 1

    n_blocks = math.ceil(
        n_samples / HDF5_TIME_BLOCK
    )

    print('====================== HDF5 -> RAM ========================')
    print(f'Датасет:                     {dataset_label}')
    print(f'Загружаемые оси:             {axis_indices}')
    print(f'Размер HDF5-блока:           {HDF5_TIME_BLOCK:,}')
    print(f'Количество блоков:           {n_blocks}')
    print('============================================================\n')

    load_start = time.perf_counter()

    for block_idx, start in enumerate(
        range(0, n_samples, HDF5_TIME_BLOCK),
        start=1
    ):
        stop = min(
            start + HDF5_TIME_BLOCK,
            n_samples
        )

        rows = stop - start

        destination = read_buffer[:rows, :, :]

        dset.read_direct(
            destination,
            source_sel=np.s_[
                start:stop,
                :,
                axis_first:axis_last
            ]
        )

        for local_axis in range(n_axes):
            axis_arrays[local_axis][start:stop, :] = (
                destination[:, :, local_axis]
            )

        if (
            block_idx == 1
            or block_idx % 4 == 0
            or block_idx == n_blocks
        ):
            elapsed = time.perf_counter() - load_start
            progress = 100.0 * stop / n_samples

            print(
                f'  HDF5 -> RAM: {progress:6.2f}% | '
                f'{block_idx}/{n_blocks} блоков | '
                f'{elapsed:.1f} с'
            )

    elapsed = time.perf_counter() - load_start

    del read_buffer
    gc.collect()

    print(
        f'\n{dataset_label}, оси {axis_indices}: '
        f'загружены за {elapsed:.2f} с\n'
    )

    return axis_arrays


# ============================================================================
# ПОДГОТОВКА ДАННЫХ
# ============================================================================

def calculate_sensor_means(
    axis_data,
    active_sensor_indices
):
    """Вычисляет среднее активных IMU."""

    n_sensors = axis_data.shape[1]

    means = np.zeros(
        n_sensors,
        dtype=np.float64
    )

    valid_sensors = np.zeros(
        n_sensors,
        dtype=bool
    )

    for sensor_idx in active_sensor_indices:
        signal = axis_data[:, sensor_idx]

        finite_mask = np.isfinite(signal)

        if np.any(finite_mask):
            means[sensor_idx] = np.mean(
                signal[finite_mask],
                dtype=np.float64
            )

            valid_sensors[sensor_idx] = True

        del finite_mask

    gc.collect()

    return means, valid_sensors


# ============================================================================
# RAM -> GPU: THETA
# ============================================================================

def build_theta_from_ram_gpu(
    axis_data,
    sensor_indices,
    dt0,
    means,
    remove_mean
):
    """
    Строит интеграл theta на GPU.

    Вход:
        axis_data: [time, sensor], float32 в RAM.

    Выход:
        d_theta: [sensor, time + 1], float64 на GPU.
    """

    n_samples = axis_data.shape[0]

    sensor_indices = np.asarray(
        sensor_indices,
        dtype=np.int64
    )

    batch_size = len(sensor_indices)

    check_gpu_memory(
        n_samples=n_samples,
        batch_size=batch_size
    )

    d_theta = cp.empty(
        (batch_size, n_samples + 1),
        dtype=cp.float64
    )

    d_theta[:, 0] = 0.0

    d_offset = cp.zeros(
        batch_size,
        dtype=cp.float64
    )

    local_means = means[sensor_indices]

    n_blocks = math.ceil(
        n_samples / HDF5_TIME_BLOCK
    )

    build_start = time.perf_counter()

    for block_idx, start in enumerate(
        range(0, n_samples, HDF5_TIME_BLOCK),
        start=1
    ):
        stop = min(
            start + HDF5_TIME_BLOCK,
            n_samples
        )

        cpu_block = np.ascontiguousarray(
            axis_data[start:stop, :][:, sensor_indices].T,
            dtype=np.float64
        )

        for local_idx in range(batch_size):
            signal = cpu_block[local_idx]

            finite_mask = np.isfinite(signal)

            if not np.all(finite_mask):
                signal[~finite_mask] = local_means[local_idx]

            if remove_mean:
                signal -= local_means[local_idx]

            del finite_mask

        d_block = cp.asarray(
            cpu_block,
            dtype=cp.float64
        )

        theta_slice = d_theta[
            :,
            start + 1:stop + 1
        ]

        cp.cumsum(
            d_block,
            axis=1,
            dtype=cp.float64,
            out=theta_slice
        )

        theta_slice *= dt0
        theta_slice += d_offset[:, None]

        d_offset = theta_slice[:, -1].copy()

        del cpu_block
        del d_block

        if (
            block_idx == 1
            or block_idx % 4 == 0
            or block_idx == n_blocks
        ):
            cp.cuda.Stream.null.synchronize()

            elapsed = time.perf_counter() - build_start
            progress = 100.0 * stop / n_samples

            print(
                f'  RAM -> GPU theta: {progress:6.2f}% | '
                f'{block_idx}/{n_blocks} | '
                f'{elapsed:.1f} с'
            )

    cp.cuda.Stream.null.synchronize()

    elapsed = time.perf_counter() - build_start

    del d_offset

    return d_theta, elapsed


# ============================================================================
# ВИРТУАЛЬНЫЙ IMU
# ============================================================================

def get_virtual_mean(
    axis_data,
    active_sensor_indices
):
    """
    Вычисляет среднее виртуального IMU.

    В расчёт входят только активные IMU.
    """

    n_samples = axis_data.shape[0]

    total_sum = 0.0
    total_count = 0

    for start in range(
        0,
        n_samples,
        HDF5_TIME_BLOCK
    ):
        stop = min(
            start + HDF5_TIME_BLOCK,
            n_samples
        )

        block = axis_data[
            start:stop,
            :
        ][:, active_sensor_indices]

        finite_mask = np.isfinite(block)

        counts = np.sum(
            finite_mask,
            axis=1,
            dtype=np.int16
        )

        sums = np.sum(
            block,
            axis=1,
            dtype=np.float64,
            where=finite_mask
        )

        valid_rows = counts > 0

        total_sum += np.sum(
            sums[valid_rows],
            dtype=np.float64
        )

        total_count += int(
            np.sum(valid_rows)
        )

        del block
        del finite_mask
        del counts
        del sums
        del valid_rows

    gc.collect()

    if total_count == 0:
        return 0.0, False

    return total_sum / total_count, True


def build_virtual_theta_from_ram_gpu(
    axis_data,
    active_sensor_indices,
    dt0,
    remove_mean
):
    """
    Строит theta виртуального IMU.

    Виртуальный IMU = среднее всех активных IMU
    в каждый момент времени.
    """

    n_samples = axis_data.shape[0]

    check_gpu_memory(
        n_samples=n_samples,
        batch_size=1
    )

    virtual_mean = 0.0

    if remove_mean:
        print('  Расчёт среднего виртуального акселерометра...')

        virtual_mean, is_valid = get_virtual_mean(
            axis_data,
            active_sensor_indices
        )

        if not is_valid:
            raise RuntimeError(
                'Виртуальный IMU полностью невалиден.'
            )

    d_theta = cp.empty(
        (1, n_samples + 1),
        dtype=cp.float64
    )

    d_theta[:, 0] = 0.0

    d_offset = cp.zeros(
        1,
        dtype=cp.float64
    )

    n_blocks = math.ceil(
        n_samples / HDF5_TIME_BLOCK
    )

    build_start = time.perf_counter()

    for block_idx, start in enumerate(
        range(0, n_samples, HDF5_TIME_BLOCK),
        start=1
    ):
        stop = min(
            start + HDF5_TIME_BLOCK,
            n_samples
        )

        block = axis_data[
            start:stop,
            :
        ][:, active_sensor_indices]

        finite_mask = np.isfinite(block)

        counts = np.sum(
            finite_mask,
            axis=1,
            dtype=np.int16
        )

        sums = np.sum(
            block,
            axis=1,
            dtype=np.float64,
            where=finite_mask
        )

        virtual_signal = np.zeros(
            stop - start,
            dtype=np.float64
        )

        valid_rows = counts > 0

        virtual_signal[valid_rows] = (
            sums[valid_rows]
            / counts[valid_rows]
        )

        if remove_mean:
            virtual_signal -= virtual_mean

        d_block = cp.asarray(
            virtual_signal.reshape(1, -1),
            dtype=cp.float64
        )

        theta_slice = d_theta[
            :,
            start + 1:stop + 1
        ]

        cp.cumsum(
            d_block,
            axis=1,
            dtype=cp.float64,
            out=theta_slice
        )

        theta_slice *= dt0
        theta_slice += d_offset[:, None]

        d_offset = theta_slice[:, -1].copy()

        del block
        del finite_mask
        del counts
        del sums
        del valid_rows
        del virtual_signal
        del d_block

        if (
            block_idx == 1
            or block_idx % 4 == 0
            or block_idx == n_blocks
        ):
            cp.cuda.Stream.null.synchronize()

            elapsed = time.perf_counter() - build_start
            progress = 100.0 * stop / n_samples

            print(
                f'  Виртуальный theta: {progress:6.2f}% | '
                f'{block_idx}/{n_blocks} | '
                f'{elapsed:.1f} с'
            )

    cp.cuda.Stream.null.synchronize()

    elapsed = time.perf_counter() - build_start

    del d_offset

    return d_theta, elapsed


# ============================================================================
# GPU: OVERLAPPING ALLAN DEVIATION
# ============================================================================

def compute_oadev_from_theta_gpu(
    d_theta,
    n_samples,
    m_steps,
    tau_vals
):
    """
    Вычисляет перекрывающуюся Allan deviation.

    d2 = theta[i + 2m] - 2*theta[i + m] + theta[i]

    ADEV(tau) =
      sqrt(sum(d2^2) / (2 * tau^2 * (N - 2m)))
    """

    batch_size = d_theta.shape[0]
    n_tau = len(m_steps)

    time_chunk = min(
        GPU_TIME_CHUNK,
        n_samples
    )

    d2_buffer = cp.empty(
        (batch_size, time_chunk),
        dtype=cp.float64
    )

    d_sum_sq = cp.empty(
        batch_size,
        dtype=cp.float64
    )

    adev = np.full(
        (n_tau, batch_size),
        np.nan,
        dtype=np.float64
    )

    event_start = cp.cuda.Event()
    event_end = cp.cuda.Event()

    event_start.record()

    for tau_idx, m in enumerate(m_steps):
        m = int(m)

        diff_count = n_samples - 2 * m

        if diff_count <= 0:
            continue

        d_sum_sq.fill(0.0)

        for start in range(
            0,
            diff_count,
            time_chunk
        ):
            stop = min(
                start + time_chunk,
                diff_count
            )

            current_length = stop - start

            d2 = d2_buffer[:, :current_length]

            d2[...] = d_theta[
                :,
                start + 2 * m:stop + 2 * m
            ]

            d2 -= d_theta[
                :,
                start + m:stop + m
            ]

            d2 -= d_theta[
                :,
                start + m:stop + m
            ]

            d2 += d_theta[
                :,
                start:stop
            ]

            cp.multiply(
                d2,
                d2,
                out=d2
            )

            d_sum_sq += cp.sum(
                d2,
                axis=1,
                dtype=cp.float64
            )

        denominator = (
            2.0
            * tau_vals[tau_idx]
            * tau_vals[tau_idx]
            * float(diff_count)
        )

        d_adev = cp.sqrt(
            d_sum_sq / denominator
        )

        adev[tau_idx, :] = cp.asnumpy(
            d_adev
        )

        if (
            tau_idx == 0
            or (tau_idx + 1) % TAU_PROGRESS_STEP == 0
            or tau_idx == n_tau - 1
        ):
            cp.cuda.Stream.null.synchronize()

            print(
                f'    tau {tau_idx + 1:3d}/{n_tau}: '
                f'm={m:10d}, '
                f'tau={tau_vals[tau_idx]:10.4f} с, '
                f'перекрытий={diff_count:,}'
            )

    event_end.record()
    event_end.synchronize()

    gpu_seconds = (
        cp.cuda.get_elapsed_time(
            event_start,
            event_end
        ) / 1000.0
    )

    del d2_buffer
    del d_sum_sq

    return adev, gpu_seconds


# ============================================================================
# РАСЧЁТ ОДНОЙ ОСИ
# ============================================================================

def calculate_axis_fast(
    axis_data,
    dt0,
    m_steps,
    tau_vals,
    active_sensor_indices,
    active_sensor_mask,
    remove_mean,
    axis_label
):
    """Рассчитывает ADEV отдельных и виртуального IMU."""

    n_samples, n_sensors = axis_data.shape
    n_tau = len(tau_vals)
    n_active = len(active_sensor_indices)

    axis_start = time.perf_counter()

    adev_matrix = np.full(
        (n_tau, n_sensors),
        np.nan,
        dtype=np.float64
    )

    print('\n============================================================')
    print(f'ОСЬ: {axis_label}')
    print(f'Активных IMU: {n_active}')
    print('Режим: RAM float32 -> GPU float64')
    print('============================================================')

    if remove_mean:
        print('\nРасчёт средних активных IMU...')

        means, valid_sensors = calculate_sensor_means(
            axis_data,
            active_sensor_indices
        )
    else:
        means = np.zeros(
            n_sensors,
            dtype=np.float64
        )

        valid_sensors = active_sensor_mask.copy()

    n_batches = math.ceil(
        n_active / GPU_SENSOR_BATCH
    )

    print('\n================ ИНДИВИДУАЛЬНЫЕ IMU ================')

    for batch_number, batch_start in enumerate(
        range(0, n_active, GPU_SENSOR_BATCH),
        start=1
    ):
        batch_indices = active_sensor_indices[
            batch_start:batch_start + GPU_SENSOR_BATCH
        ]

        imu_numbers = ', '.join(
            str(int(index) + 1)
            for index in batch_indices
        )

        print(
            f'\nGPU-пакет {batch_number}/{n_batches}: '
            f'IMU [{imu_numbers}]'
        )

        batch_start_time = time.perf_counter()

        d_theta, theta_seconds = build_theta_from_ram_gpu(
            axis_data=axis_data,
            sensor_indices=batch_indices,
            dt0=dt0,
            means=means,
            remove_mean=remove_mean
        )

        print('  Расчёт перекрывающейся ADEV на GPU...')

        adev_batch, oadev_seconds = compute_oadev_from_theta_gpu(
            d_theta=d_theta,
            n_samples=n_samples,
            m_steps=m_steps,
            tau_vals=tau_vals
        )

        for local_idx, global_idx in enumerate(batch_indices):
            if valid_sensors[global_idx]:
                adev_matrix[:, global_idx] = (
                    adev_batch[:, local_idx]
                )

        del adev_batch
        del d_theta

        release_gpu_memory()

        elapsed = time.perf_counter() - batch_start_time

        print(
            f'  Пакет готов: {elapsed:.2f} с | '
            f'theta: {theta_seconds:.2f} с | '
            f'ADEV GPU: {oadev_seconds:.2f} с'
        )

    adev_matrix[:, ~active_sensor_mask] = np.nan

    print('\n================ ВИРТУАЛЬНЫЙ МАССИВ ================')

    virtual_start = time.perf_counter()

    d_theta_virtual, theta_seconds = build_virtual_theta_from_ram_gpu(
        axis_data=axis_data,
        active_sensor_indices=active_sensor_indices,
        dt0=dt0,
        remove_mean=remove_mean
    )

    print('  Расчёт ADEV виртуального массива на GPU...')

    adev_virtual_batch, oadev_seconds = compute_oadev_from_theta_gpu(
        d_theta=d_theta_virtual,
        n_samples=n_samples,
        m_steps=m_steps,
        tau_vals=tau_vals
    )

    adev_virtual = adev_virtual_batch[:, 0]

    del adev_virtual_batch
    del d_theta_virtual
    del means
    del valid_sensors

    release_gpu_memory()

    virtual_elapsed = time.perf_counter() - virtual_start
    full_elapsed = time.perf_counter() - axis_start

    print('\n============================================================')
    print(f'Ось завершена:               {axis_label}')
    print(f'theta виртуального массива:  {theta_seconds:.2f} с')
    print(f'ADEV виртуального массива:   {oadev_seconds:.2f} с')
    print(f'Виртуальный массив всего:    {virtual_elapsed:.2f} с')
    print(f'Полное время оси:            {full_elapsed / 60.0:.2f} мин')
    print('============================================================\n')

    return adev_matrix, adev_virtual


# ============================================================================
# ПАРАМЕТРЫ ГИРОСКОПА
# ============================================================================

def extract_gyro_parameters(
    adev_matrix,
    adev_virtual,
    idx_tau_1s,
    tau_vals
):
    """
    Входная ADEV гироскопа: град/с.

    ARW:
      град/√ч

    BI:
      град/ч
    """

    n_sensors = adev_matrix.shape[1]

    arw = np.full(
        n_sensors,
        np.nan,
        dtype=np.float64
    )

    bi = np.full(
        n_sensors,
        np.nan,
        dtype=np.float64
    )

    tau_bi = np.full(
        n_sensors,
        np.nan,
        dtype=np.float64
    )

    for sensor_idx in range(n_sensors):
        adev = adev_matrix[:, sensor_idx]

        if not np.any(np.isfinite(adev)):
            continue

        arw[sensor_idx] = (
            adev[idx_tau_1s] * 60.0
        )

        min_idx = int(
            np.nanargmin(adev)
        )

        bi[sensor_idx] = (
            adev[min_idx]
            / BI_FLICKER_FACTOR
            * 3600.0
        )

        tau_bi[sensor_idx] = tau_vals[min_idx]

    min_idx = int(
        np.nanargmin(adev_virtual)
    )

    arw_virtual = (
        adev_virtual[idx_tau_1s] * 60.0
    )

    bi_virtual = (
        adev_virtual[min_idx]
        / BI_FLICKER_FACTOR
        * 3600.0
    )

    tau_bi_virtual = tau_vals[min_idx]

    return (
        arw,
        bi,
        tau_bi,
        arw_virtual,
        bi_virtual,
        tau_bi_virtual
    )


# ============================================================================
# ПАРАМЕТРЫ АКСЕЛЕРОМЕТРА
# ============================================================================

def extract_accel_parameters(
    adev_matrix,
    adev_virtual,
    idx_tau_1s,
    tau_vals
):
    """
    Входная ADEV акселерометра: g.

    VRW:
      м/с/√ч

    BI:
      мкg
    """

    n_sensors = adev_matrix.shape[1]

    vrw = np.full(
        n_sensors,
        np.nan,
        dtype=np.float64
    )

    bi_micro_g = np.full(
        n_sensors,
        np.nan,
        dtype=np.float64
    )

    tau_bi = np.full(
        n_sensors,
        np.nan,
        dtype=np.float64
    )

    for sensor_idx in range(n_sensors):
        adev = adev_matrix[:, sensor_idx]

        if not np.any(np.isfinite(adev)):
            continue

        vrw[sensor_idx] = (
            adev[idx_tau_1s]
            * G_STANDARD
            * 60.0
        )

        min_idx = int(
            np.nanargmin(adev)
        )

        bi_micro_g[sensor_idx] = (
            adev[min_idx]
            / BI_FLICKER_FACTOR
            * 1e6
        )

        tau_bi[sensor_idx] = tau_vals[min_idx]

    min_idx = int(
        np.nanargmin(adev_virtual)
    )

    vrw_virtual = (
        adev_virtual[idx_tau_1s]
        * G_STANDARD
        * 60.0
    )

    bi_virtual_micro_g = (
        adev_virtual[min_idx]
        / BI_FLICKER_FACTOR
        * 1e6
    )

    tau_bi_virtual = tau_vals[min_idx]

    return (
        vrw,
        bi_micro_g,
        tau_bi,
        vrw_virtual,
        bi_virtual_micro_g,
        tau_bi_virtual
    )


# ============================================================================
# ГРАФИКИ
# ============================================================================

def positive_finite_values(*arrays):
    """Возвращает все положительные конечные значения."""

    values = []

    for array in arrays:
        array = np.asarray(array)

        valid_values = array[
            np.isfinite(array)
            & (array > 0.0)
        ]

        if valid_values.size > 0:
            values.append(valid_values)

    if len(values) == 0:
        return np.array([1e-12], dtype=np.float64)

    return np.concatenate(values)


def save_gyro_plot(
    output_path,
    tau_vals,
    adev_matrix,
    adev_virtual,
    axis_name,
    axis_short,
    arw,
    bi,
    arw_virtual,
    bi_virtual,
    tau_bi_virtual,
    active_sensor_indices,
    record_hours,
    fs
):
    """Строит график ADEV гироскопа."""

    n_active = len(active_sensor_indices)

    adev_sensors_dph = adev_matrix * 3600.0
    adev_virtual_dph = adev_virtual * 3600.0

    mean_adev_dph = np.nanmean(
        adev_sensors_dph[:, active_sensor_indices],
        axis=1
    )

    mean_arw = np.nanmean(
        arw[active_sensor_indices]
    )

    mean_bi = np.nanmean(
        bi[active_sensor_indices]
    )

    gain_arw = mean_arw / arw_virtual
    gain_bi = mean_bi / bi_virtual

    min_idx = int(
        np.nanargmin(adev_virtual_dph)
    )

    min_value = adev_virtual_dph[min_idx]

    fig, ax = plt.subplots(
        figsize=(14, 8.5)
    )

    for sensor_idx in active_sensor_indices:
        ax.loglog(
            tau_vals,
            adev_sensors_dph[:, sensor_idx],
            color=(0.83, 0.83, 0.83),
            linewidth=0.45
        )

    ax.loglog(
        tau_vals,
        mean_adev_dph,
        'k--',
        linewidth=2.0,
        label='Средняя ADEV исправных IMU'
    )

    ax.loglog(
        tau_vals,
        adev_virtual_dph,
        color=(0.85, 0.08, 0.08),
        linewidth=3.0,
        label=f'Виртуальный массив ({n_active} IMU)'
    )

    arw_mask = (
        (tau_vals >= max(tau_vals[0], 0.008))
        & (tau_vals <= min(tau_vals[-1], 15.0))
    )

    if np.any(arw_mask):
        tau_arw = tau_vals[arw_mask]

        arw_curve = (
            arw_virtual
            / 60.0
            / np.sqrt(tau_arw)
            * 3600.0
        )

        ax.loglog(
            tau_arw,
            arw_curve,
            'b-.',
            linewidth=1.5,
            label='Участок углового случайного блуждания (ARW)'
        )

    bi_mask = (
        (tau_vals >= tau_bi_virtual * 0.2)
        & (tau_vals <= tau_bi_virtual * 5.0)
    )

    if np.any(bi_mask):
        tau_bi = tau_vals[bi_mask]

        ax.loglog(
            tau_bi,
            np.full_like(tau_bi, min_value),
            'g--',
            linewidth=1.5,
            label='Участок нестабильности смещения (BI)'
        )

    ax.plot(
        tau_vals[min_idx],
        min_value,
        marker='*',
        markersize=14,
        markerfacecolor=(1.0, 0.85, 0.05),
        markeredgecolor='black'
    )

    passport = (
        f'ПАСПОРТ ГИРОСКОПА ({axis_short})\n'
        '────────────────────────────────────\n'
        f'Активных датчиков: {n_active}\n'
        f'Исключены датчики: {get_excluded_imu_text()}\n\n'
        'Угловое случайное блуждание (ARW)\n'
        r'$\mathrm{ARW} = \sigma(1\,\mathrm{с}) \cdot 60$' '\n'
        f'Одиночный IMU: {mean_arw:.5f} град/√ч\n'
        f'Массив IMU: {arw_virtual:.5f} град/√ч\n'
        f'Выигрыш: {gain_arw:.3f}×\n\n'
        'Нестабильность смещения (BI)\n'
        r'$\mathrm{BI} = \sigma_{\min} / 0.664282$' '\n'
        f'Одиночный IMU: {mean_bi:.4f} град/ч\n'
        f'Массив IMU: {bi_virtual:.4f} град/ч\n'
        f'Выигрыш: {gain_bi:.3f}×\n'
        f'Время BI: {tau_bi_virtual:.2f} с\n'
        '────────────────────────────────────\n'
        f'Частота: {fs:.3f} Гц\n'
        f'Длительность: {record_hours:.3f} ч'
    )

    ax.text(
        0.63,
        0.95,
        passport,
        transform=ax.transAxes,
        fontsize=8.2,
        va='top',
        ha='left',
        bbox=dict(
            facecolor='white',
            edgecolor=(0.4, 0.4, 0.4),
            alpha=0.96
        )
    )

    all_values = positive_finite_values(
        mean_adev_dph,
        adev_virtual_dph
    )

    ax.set_ylim(
        np.min(all_values) * 0.3,
        np.max(all_values) * 3.0
    )

    ax.set_xlim(
        tau_vals[0],
        tau_vals[-1]
    )

    ax.grid(
        True,
        which='both',
        alpha=0.35
    )

    ax.set_xlabel(
        'Время усреднения tau [с]',
        fontsize=12,
        fontweight='bold'
    )

    ax.set_ylabel(
        'Отклонение Аллана (ADEV) [град/ч]',
        fontsize=12,
        fontweight='bold'
    )

    ax.set_title(
        f'Вариация Аллана — {axis_name}\n'
        f'Расчёт GPU float64, виртуальный массив: {n_active} IMU',
        fontsize=13,
        fontweight='bold'
    )

    ax.legend(
        loc='lower left',
        fontsize=8.5
    )

    fig.tight_layout()
    fig.savefig(
        output_path,
        dpi=PLOT_DPI
    )

    plt.close(fig)


def save_accel_plot(
    output_path,
    tau_vals,
    adev_matrix,
    adev_virtual,
    axis_name,
    axis_short,
    vrw,
    bi_micro_g,
    vrw_virtual,
    bi_virtual_micro_g,
    tau_bi_virtual,
    active_sensor_indices,
    record_hours,
    fs
):
    """
    Строит график ADEV акселерометра.

    ADEV на графике: mg.
    BI в паспорте: мкg.
    """

    n_active = len(active_sensor_indices)

    adev_sensors_mg = adev_matrix * 1000.0
    adev_virtual_mg = adev_virtual * 1000.0

    mean_adev_mg = np.nanmean(
        adev_sensors_mg[:, active_sensor_indices],
        axis=1
    )

    mean_vrw = np.nanmean(
        vrw[active_sensor_indices]
    )

    mean_bi_micro_g = np.nanmean(
        bi_micro_g[active_sensor_indices]
    )

    gain_vrw = mean_vrw / vrw_virtual
    gain_bi = mean_bi_micro_g / bi_virtual_micro_g

    min_idx = int(
        np.nanargmin(adev_virtual_mg)
    )

    min_value_mg = adev_virtual_mg[min_idx]

    fig, ax = plt.subplots(
        figsize=(14, 8.5)
    )

    for sensor_idx in active_sensor_indices:
        ax.loglog(
            tau_vals,
            adev_sensors_mg[:, sensor_idx],
            color=(0.83, 0.83, 0.83),
            linewidth=0.45
        )

    ax.loglog(
        tau_vals,
        mean_adev_mg,
        'k--',
        linewidth=2.0,
        label='Средняя ADEV исправных IMU'
    )

    ax.loglog(
        tau_vals,
        adev_virtual_mg,
        color=(0.08, 0.42, 0.85),
        linewidth=3.0,
        label=f'Виртуальный массив ({n_active} IMU)'
    )

    vrw_mask = (
        (tau_vals >= max(tau_vals[0], 0.008))
        & (tau_vals <= min(tau_vals[-1], 15.0))
    )

    if np.any(vrw_mask):
        tau_vrw = tau_vals[vrw_mask]

        vrw_g_sqrt_s = (
            vrw_virtual
            / G_STANDARD
            / 60.0
        )

        vrw_curve_mg = (
            vrw_g_sqrt_s
            / np.sqrt(tau_vrw)
            * 1000.0
        )

        ax.loglog(
            tau_vrw,
            vrw_curve_mg,
            'r-.',
            linewidth=1.5,
            label='Участок скоростного случайного блуждания (VRW)'
        )

    bi_mask = (
        (tau_vals >= tau_bi_virtual * 0.2)
        & (tau_vals <= tau_bi_virtual * 5.0)
    )

    if np.any(bi_mask):
        tau_bi = tau_vals[bi_mask]

        ax.loglog(
            tau_bi,
            np.full_like(
                tau_bi,
                min_value_mg
            ),
            'g--',
            linewidth=1.5,
            label='Участок нестабильности смещения (BI)'
        )

    ax.plot(
        tau_vals[min_idx],
        min_value_mg,
        marker='*',
        markersize=14,
        markerfacecolor=(1.0, 0.85, 0.05),
        markeredgecolor='black'
    )

    passport = (
        f'ПАСПОРТ АКСЕЛЕРОМЕТРА ({axis_short})\n'
        '────────────────────────────────────\n'
        f'Активных датчиков: {n_active}\n'
        f'Исключены датчики: {get_excluded_imu_text()}\n\n'
        'Скоростное случайное блуждание (VRW)\n'
        r'$\mathrm{VRW} = \sigma(1\,\mathrm{с}) \cdot g_0 \cdot 60$' '\n'
        f'Одиночный IMU: {mean_vrw:.5f} м/с/√ч\n'
        f'Массив IMU: {vrw_virtual:.5f} м/с/√ч\n'
        f'Выигрыш: {gain_vrw:.3f}×\n\n'
        'Нестабильность смещения (BI)\n'
        r'$\mathrm{BI} = \sigma_{\min} / 0.664282$' '\n'
        r'$\mathrm{BI}_{\mu g} = \mathrm{BI}_{g} \cdot 10^6$' '\n'
        f'Одиночный IMU: {mean_bi_micro_g:.3f} мкg\n'
        f'Массив IMU: {bi_virtual_micro_g:.3f} мкg\n'
        f'Выигрыш: {gain_bi:.3f}×\n'
        f'Время BI: {tau_bi_virtual:.2f} с\n'
        '────────────────────────────────────\n'
        f'Частота: {fs:.3f} Гц\n'
        f'Длительность: {record_hours:.3f} ч'
    )

    ax.text(
        0.63,
        0.95,
        passport,
        transform=ax.transAxes,
        fontsize=8.1,
        va='top',
        ha='left',
        bbox=dict(
            facecolor='white',
            edgecolor=(0.4, 0.4, 0.4),
            alpha=0.96
        )
    )

    all_values = positive_finite_values(
        mean_adev_mg,
        adev_virtual_mg
    )

    ax.set_ylim(
        np.min(all_values) * 0.3,
        np.max(all_values) * 3.0
    )

    ax.set_xlim(
        tau_vals[0],
        tau_vals[-1]
    )

    ax.grid(
        True,
        which='both',
        alpha=0.35
    )

    ax.set_xlabel(
        'Время усреднения tau [с]',
        fontsize=12,
        fontweight='bold'
    )

    ax.set_ylabel(
        'Отклонение Аллана (ADEV) [mg]',
        fontsize=12,
        fontweight='bold'
    )

    ax.set_title(
        f'Вариация Аллана — {axis_name}\n'
        f'Расчёт GPU float64, виртуальный массив: {n_active} IMU',
        fontsize=13,
        fontweight='bold'
    )

    ax.legend(
        loc='lower left',
        fontsize=8.5
    )

    fig.tight_layout()
    fig.savefig(
        output_path,
        dpi=PLOT_DPI
    )

    plt.close(fig)


# ============================================================================
# ОБРАБОТКА XYZ-ДАТАСЕТА
# ============================================================================

def process_dataset_axes(
    dset,
    layout,
    dataset_name,
    axis_names,
    dt0,
    m_steps,
    tau_vals,
    active_sensor_mask,
    active_sensor_indices,
    remove_mean
):
    """
    Обрабатывает X, Y, Z.

    X/Y загружаются совместно, затем полностью освобождаются.
    Z загружается и обрабатывается отдельно.
    """

    (
        time_axis,
        sensor_axis,
        xyz_axis,
        n_samples,
        n_sensors
    ) = layout

    if not (
        time_axis == 0
        and sensor_axis == 1
        and xyz_axis == 2
    ):
        raise ValueError(
            'Ожидался layout [time, sensor, xyz].\n'
            f'Фактическая форма: {dset.shape}'
        )

    n_tau = len(tau_vals)

    allan_individual = np.full(
        (n_tau, n_sensors, 3),
        np.nan,
        dtype=np.float64
    )

    allan_virtual = np.full(
        (n_tau, 3),
        np.nan,
        dtype=np.float64
    )

    axis_groups = [
        [0, 1],
        [2]
    ]

    for axis_group in axis_groups:
        print('\n============================================================')
        print(
            f'{dataset_name}: загрузка осей {axis_group} в RAM'
        )
        print('============================================================')

        axis_arrays = load_axes_to_ram(
            dset=dset,
            axis_indices=axis_group,
            n_samples=n_samples,
            n_sensors=n_sensors,
            dataset_label=dataset_name
        )

        for local_axis, axis_idx in enumerate(axis_group):
            axis_data = axis_arrays[local_axis]

            adev_matrix, adev_virtual = calculate_axis_fast(
                axis_data=axis_data,
                dt0=dt0,
                m_steps=m_steps,
                tau_vals=tau_vals,
                active_sensor_indices=active_sensor_indices,
                active_sensor_mask=active_sensor_mask,
                remove_mean=remove_mean,
                axis_label=axis_names[axis_idx]
            )

            allan_individual[:, :, axis_idx] = adev_matrix
            allan_virtual[:, axis_idx] = adev_virtual

            del adev_matrix
            del adev_virtual

            # Убирает локальную ссылку на большую матрицу RAM.
            axis_data = None

            gc.collect()

        # КРИТИЧЕСКОЕ ИСПРАВЛЕНИЕ:
        # Нельзя делать:
        #
        # for i in range(len(axis_arrays)):
        #     del axis_arrays[i]
        #
        # После del первый элемент удаляется, список уменьшается,
        # и индекс следующего элемента становится некорректным.
        #
        # clear() освобождает все ссылки корректно.
        axis_arrays.clear()
        del axis_arrays

        gc.collect()

    return allan_individual, allan_virtual


# ============================================================================
# MAIN
# ============================================================================

def main():

    total_start = time.perf_counter()

    if not os.path.isfile(H5_FILE_PATH):
        raise FileNotFoundError(
            f'Файл HDF5 не найден:\n{H5_FILE_PATH}'
        )

    os.makedirs(
        OUTPUT_DIR,
        exist_ok=True
    )

    print('\n============================================================')
    print(' ВАРИАЦИЯ АЛЛАНА: CUDA GPU, КРУПНЫЕ HDF5-БЛОКИ, БЕЗ SSD')
    print('============================================================\n')

    get_gpu_info()

    with h5py.File(H5_FILE_PATH, 'r') as h5_file:
        for dataset_name in ('gyro', 'accel', 'tstamp'):
            if dataset_name not in h5_file:
                raise KeyError(
                    f'В HDF5 отсутствует /{dataset_name}'
                )

        gyro_shape = h5_file['gyro'].shape
        accel_shape = h5_file['accel'].shape
        tstamp_shape = h5_file['tstamp'].shape

        gyro_dtype = h5_file['gyro'].dtype
        accel_dtype = h5_file['accel'].dtype

        gyro_chunks = h5_file['gyro'].chunks
        accel_chunks = h5_file['accel'].chunks

        gyro_compression = h5_file['gyro'].compression
        accel_compression = h5_file['accel'].compression

        gyro_layout = detect_3d_layout(
            h5_file['gyro']
        )

        accel_layout = detect_3d_layout(
            h5_file['accel']
        )

    _, _, _, n_samples, n_sensors = gyro_layout

    if accel_layout[3] != n_samples:
        raise ValueError(
            'gyro и accel имеют разное число кадров.'
        )

    if accel_layout[4] != n_sensors:
        raise ValueError(
            'gyro и accel имеют разное число IMU.'
        )

    (
        active_sensor_mask,
        active_sensor_indices,
        excluded_indices
    ) = build_active_sensor_mask(
        n_sensors
    )

    n_active_sensors = len(
        active_sensor_indices
    )

    print('======================= HDF5 DATA =========================')
    print(f'Файл:                        {H5_FILE_PATH}')
    print(f'Размер gyro:                 {gyro_shape}')
    print(f'Размер accel:                {accel_shape}')
    print(f'Размер tstamp:               {tstamp_shape}')
    print(f'Тип gyro:                    {gyro_dtype}')
    print(f'Тип accel:                   {accel_dtype}')
    print(f'Chunks gyro:                 {gyro_chunks}')
    print(f'Chunks accel:                {accel_chunks}')
    print(f'Сжатие gyro:                 {gyro_compression}')
    print(f'Сжатие accel:                {accel_compression}')
    print(f'Количество кадров:           {n_samples:,}')
    print(f'Количество IMU:              {n_sensors}')
    print('============================================================\n')

    print('==================== СОСТАВ МАССИВА ======================')
    print(f'Всего IMU:                   {n_sensors}')
    print(f'Исключённые IMU:             {EXCLUDED_IMU_1BASED}')
    print(f'Индексы Python:              {excluded_indices.tolist()}')
    print(f'Используемые IMU:            {n_active_sensors}')
    print('IMU №5 участвует:            нет')
    print('===========================================================\n')

    print('================ АНАЛИЗ СИНХРОНИЗАЦИИ ====================')

    (
        dt0,
        fs,
        mean_ticks,
        nominal_ticks,
        jitter_std
    ) = estimate_sample_period(
        h5_path=H5_FILE_PATH,
        n_sensors=n_sensors,
        n_samples=n_samples
    )

    valid_ticks = nominal_ticks[
        nominal_ticks > 0.0
    ]

    record_hours = (
        n_samples
        * dt0
        / 3600.0
    )

    print(
        f'Номинальный шаг таймера:     '
        f'{mean_ticks:.0f} тиков '
        f'({dt0 * 1000.0:.6f} мс)'
    )

    print(
        f'Частота опроса:              {fs:.4f} Гц'
    )

    print(
        f'Разброс шагов IMU:           '
        f'{np.min(valid_ticks):.0f} / '
        f'{np.max(valid_ticks):.0f} тиков'
    )

    print(
        f'Средний джиттер:             '
        f'{np.mean(jitter_std):.6f} мкс'
    )

    print(
        f'Длительность записи:         '
        f'{record_hours:.3f} ч'
    )

    print('===========================================================\n')

    m_steps, tau_vals, idx_tau_1s = build_tau_grid(
        n_samples=n_samples,
        dt0=dt0
    )

    max_m = int(m_steps[-1])

    max_overlaps = (
        n_samples - 2 * max_m
    )

    print('======================== TAU GRID =========================')
    print(f'Точек tau:                   {len(tau_vals)}')
    print(f'tau min:                     {tau_vals[0]:.8f} с')
    print(f'tau max:                     {tau_vals[-1]:.3f} с')
    print(
        f'tau для ARW / VRW:           '
        f'{tau_vals[idx_tau_1s]:.6f} с'
    )
    print(f'MIN_OVERLAPS:                {MIN_OVERLAPS:,}')
    print(f'Перекрытий на tau max:       {max_overlaps:,}')
    print('===========================================================\n')

    gyro_names = [
        'Гироскоп X (Gx)',
        'Гироскоп Y (Gy)',
        'Гироскоп Z (Gz)'
    ]

    gyro_short_names = [
        'Gx',
        'Gy',
        'Gz'
    ]

    accel_names = [
        'Акселерометр X (Ax)',
        'Акселерометр Y (Ay)',
        'Акселерометр Z (Az)'
    ]

    accel_short_names = [
        'Ax',
        'Ay',
        'Az'
    ]

    # ========================================================================
    # ГИРОСКОПЫ
    # ========================================================================

    print('============================================================')
    print('                     ГИРОСКОПЫ')
    print('============================================================')

    gyro_start = time.perf_counter()

    with h5py.File(
        H5_FILE_PATH,
        'r',
        rdcc_nbytes=1024 * 1024 * 1024,
        rdcc_nslots=1_000_003
    ) as h5_file:

        allan_gyro, allan_gyro_virtual = process_dataset_axes(
            dset=h5_file['gyro'],
            layout=gyro_layout,
            dataset_name='ГИРОСКОПЫ',
            axis_names=gyro_names,
            dt0=dt0,
            m_steps=m_steps,
            tau_vals=tau_vals,
            active_sensor_mask=active_sensor_mask,
            active_sensor_indices=active_sensor_indices,
            remove_mean=False
        )

    gyro_elapsed = time.perf_counter() - gyro_start

    # ========================================================================
    # АКСЕЛЕРОМЕТРЫ
    # ========================================================================

    print('\n============================================================')
    print('                  АКСЕЛЕРОМЕТРЫ')
    print('============================================================')

    accel_start = time.perf_counter()

    with h5py.File(
        H5_FILE_PATH,
        'r',
        rdcc_nbytes=1024 * 1024 * 1024,
        rdcc_nslots=1_000_003
    ) as h5_file:

        allan_accel, allan_accel_virtual = process_dataset_axes(
            dset=h5_file['accel'],
            layout=accel_layout,
            dataset_name='АКСЕЛЕРОМЕТРЫ',
            axis_names=accel_names,
            dt0=dt0,
            m_steps=m_steps,
            tau_vals=tau_vals,
            active_sensor_mask=active_sensor_mask,
            active_sensor_indices=active_sensor_indices,
            remove_mean=True
        )

    accel_elapsed = time.perf_counter() - accel_start

    # ========================================================================
    # ВЫЧИСЛЕНИЕ ПАСПОРТНЫХ ПАРАМЕТРОВ
    # ========================================================================

    arw_gyro = np.full(
        (n_sensors, 3),
        np.nan,
        dtype=np.float64
    )

    bi_gyro = np.full(
        (n_sensors, 3),
        np.nan,
        dtype=np.float64
    )

    tau_bi_gyro = np.full(
        (n_sensors, 3),
        np.nan,
        dtype=np.float64
    )

    arw_gyro_virtual = np.full(
        3,
        np.nan,
        dtype=np.float64
    )

    bi_gyro_virtual = np.full(
        3,
        np.nan,
        dtype=np.float64
    )

    tau_bi_gyro_virtual = np.full(
        3,
        np.nan,
        dtype=np.float64
    )

    vrw_accel = np.full(
        (n_sensors, 3),
        np.nan,
        dtype=np.float64
    )

    bi_accel_micro_g = np.full(
        (n_sensors, 3),
        np.nan,
        dtype=np.float64
    )

    tau_bi_accel = np.full(
        (n_sensors, 3),
        np.nan,
        dtype=np.float64
    )

    vrw_accel_virtual = np.full(
        3,
        np.nan,
        dtype=np.float64
    )

    bi_accel_virtual_micro_g = np.full(
        3,
        np.nan,
        dtype=np.float64
    )

    tau_bi_accel_virtual = np.full(
        3,
        np.nan,
        dtype=np.float64
    )

    for axis_idx in range(3):
        (
            arw,
            bi,
            tau_bi,
            arw_virtual,
            bi_virtual,
            tau_bi_virtual
        ) = extract_gyro_parameters(
            adev_matrix=allan_gyro[:, :, axis_idx],
            adev_virtual=allan_gyro_virtual[:, axis_idx],
            idx_tau_1s=idx_tau_1s,
            tau_vals=tau_vals
        )

        arw_gyro[:, axis_idx] = arw
        bi_gyro[:, axis_idx] = bi
        tau_bi_gyro[:, axis_idx] = tau_bi

        arw_gyro_virtual[axis_idx] = arw_virtual
        bi_gyro_virtual[axis_idx] = bi_virtual
        tau_bi_gyro_virtual[axis_idx] = tau_bi_virtual

        (
            vrw,
            bi_micro_g,
            tau_bi,
            vrw_virtual,
            bi_virtual_micro_g,
            tau_bi_virtual
        ) = extract_accel_parameters(
            adev_matrix=allan_accel[:, :, axis_idx],
            adev_virtual=allan_accel_virtual[:, axis_idx],
            idx_tau_1s=idx_tau_1s,
            tau_vals=tau_vals
        )

        vrw_accel[:, axis_idx] = vrw
        bi_accel_micro_g[:, axis_idx] = bi_micro_g
        tau_bi_accel[:, axis_idx] = tau_bi

        vrw_accel_virtual[axis_idx] = vrw_virtual
        bi_accel_virtual_micro_g[axis_idx] = bi_virtual_micro_g
        tau_bi_accel_virtual[axis_idx] = tau_bi_virtual

    # ========================================================================
    # ВЫВОД ПАСПОРТА ГИРОСКОПОВ
    # ========================================================================

    print('\n============================================================')
    print('                 ПАСПОРТ ГИРОСКОПОВ')
    print('============================================================')

    for axis_idx in range(3):
        mean_arw = np.nanmean(
            arw_gyro[
                active_sensor_indices,
                axis_idx
            ]
        )

        mean_bi = np.nanmean(
            bi_gyro[
                active_sensor_indices,
                axis_idx
            ]
        )

        gain_arw = (
            mean_arw
            / arw_gyro_virtual[axis_idx]
        )

        gain_bi = (
            mean_bi
            / bi_gyro_virtual[axis_idx]
        )

        print(f'\n{gyro_names[axis_idx]}')
        print('------------------------------------------------------------')
        print(f'Активных IMU:                {n_active_sensors}')
        print(f'Исключены IMU:               {EXCLUDED_IMU_1BASED}')
        print('Угловое случайное блуждание (ARW)')
        print(
            f'  ARW одиночного IMU:        '
            f'{mean_arw:.6f} град/√ч'
        )
        print(
            f'  ARW массива IMU:           '
            f'{arw_gyro_virtual[axis_idx]:.6f} град/√ч'
        )
        print(
            f'  Выигрыш ARW:               {gain_arw:.3f} x'
        )
        print('Нестабильность смещения (BI)')
        print(
            f'  BI одиночного IMU:         '
            f'{mean_bi:.4f} град/ч'
        )
        print(
            f'  BI массива IMU:            '
            f'{bi_gyro_virtual[axis_idx]:.4f} град/ч'
        )
        print(
            f'  Выигрыш BI:                {gain_bi:.3f} x'
        )
        print(
            f'  Время BI:                  '
            f'{tau_bi_gyro_virtual[axis_idx]:.2f} с'
        )

    # ========================================================================
    # ВЫВОД ПАСПОРТА АКСЕЛЕРОМЕТРОВ
    # ========================================================================

    print('\n============================================================')
    print('               ПАСПОРТ АКСЕЛЕРОМЕТРОВ')
    print('============================================================')

    for axis_idx in range(3):
        mean_vrw = np.nanmean(
            vrw_accel[
                active_sensor_indices,
                axis_idx
            ]
        )

        mean_bi_micro_g = np.nanmean(
            bi_accel_micro_g[
                active_sensor_indices,
                axis_idx
            ]
        )

        gain_vrw = (
            mean_vrw
            / vrw_accel_virtual[axis_idx]
        )

        gain_bi = (
            mean_bi_micro_g
            / bi_accel_virtual_micro_g[axis_idx]
        )

        print(f'\n{accel_names[axis_idx]}')
        print('------------------------------------------------------------')
        print(f'Активных IMU:                {n_active_sensors}')
        print(f'Исключены IMU:               {EXCLUDED_IMU_1BASED}')
        print('Скоростное случайное блуждание (VRW)')
        print(
            f'  VRW одиночного IMU:        '
            f'{mean_vrw:.6f} м/с/√ч'
        )
        print(
            f'  VRW массива IMU:           '
            f'{vrw_accel_virtual[axis_idx]:.6f} м/с/√ч'
        )
        print(
            f'  Выигрыш VRW:               {gain_vrw:.3f} x'
        )
        print('Нестабильность смещения (BI)')
        print(
            f'  BI одиночного IMU:         '
            f'{mean_bi_micro_g:.3f} мкg'
        )
        print(
            f'  BI массива IMU:            '
            f'{bi_accel_virtual_micro_g[axis_idx]:.3f} мкg'
        )
        print(
            f'  Выигрыш BI:                {gain_bi:.3f} x'
        )
        print(
            f'  Время BI:                  '
            f'{tau_bi_accel_virtual[axis_idx]:.2f} с'
        )

    # ========================================================================
    # ГРАФИКИ
    # ========================================================================

    print('\nПостроение графиков гироскопов...')

    for axis_idx in range(3):
        save_gyro_plot(
            output_path=os.path.join(
                OUTPUT_DIR,
                f'allan_gyro_{gyro_short_names[axis_idx]}_gpu.png'
            ),
            tau_vals=tau_vals,
            adev_matrix=allan_gyro[:, :, axis_idx],
            adev_virtual=allan_gyro_virtual[:, axis_idx],
            axis_name=gyro_names[axis_idx],
            axis_short=gyro_short_names[axis_idx],
            arw=arw_gyro[:, axis_idx],
            bi=bi_gyro[:, axis_idx],
            arw_virtual=arw_gyro_virtual[axis_idx],
            bi_virtual=bi_gyro_virtual[axis_idx],
            tau_bi_virtual=tau_bi_gyro_virtual[axis_idx],
            active_sensor_indices=active_sensor_indices,
            record_hours=record_hours,
            fs=fs
        )

    print('Построение графиков акселерометров...')

    for axis_idx in range(3):
        save_accel_plot(
            output_path=os.path.join(
                OUTPUT_DIR,
                f'allan_accel_{accel_short_names[axis_idx]}_gpu.png'
            ),
            tau_vals=tau_vals,
            adev_matrix=allan_accel[:, :, axis_idx],
            adev_virtual=allan_accel_virtual[:, axis_idx],
            axis_name=accel_names[axis_idx],
            axis_short=accel_short_names[axis_idx],
            vrw=vrw_accel[:, axis_idx],
            bi_micro_g=bi_accel_micro_g[:, axis_idx],
            vrw_virtual=vrw_accel_virtual[axis_idx],
            bi_virtual_micro_g=bi_accel_virtual_micro_g[axis_idx],
            tau_bi_virtual=tau_bi_accel_virtual[axis_idx],
            active_sensor_indices=active_sensor_indices,
            record_hours=record_hours,
            fs=fs
        )

    # ========================================================================
    # СОХРАНЕНИЕ ЧИСЛОВЫХ РЕЗУЛЬТАТОВ
    # ========================================================================

    if SAVE_NUMERIC_RESULTS:
        result_path = os.path.join(
            OUTPUT_DIR,
            'allan_gpu_35imu_results.npz'
        )

        np.savez_compressed(
            result_path,

            dt0=dt0,
            fs=fs,

            n_samples=n_samples,
            n_sensors=n_sensors,
            n_active_sensors=n_active_sensors,

            excluded_imu_1based=np.asarray(
                EXCLUDED_IMU_1BASED,
                dtype=np.int64
            ),

            excluded_imu_zerobased=excluded_indices,
            active_sensor_indices=active_sensor_indices,

            m_steps=m_steps,
            tau_vals=tau_vals,

            allan_gyro=allan_gyro,
            allan_gyro_virtual=allan_gyro_virtual,

            allan_accel=allan_accel,
            allan_accel_virtual=allan_accel_virtual,

            arw_gyro=arw_gyro,
            bi_gyro_deg_per_hour=bi_gyro,
            tau_bi_gyro=tau_bi_gyro,

            arw_gyro_virtual=arw_gyro_virtual,
            bi_gyro_virtual_deg_per_hour=bi_gyro_virtual,
            tau_bi_gyro_virtual=tau_bi_gyro_virtual,

            vrw_accel=vrw_accel,
            bi_accel_micro_g=bi_accel_micro_g,
            tau_bi_accel=tau_bi_accel,

            vrw_accel_virtual=vrw_accel_virtual,
            bi_accel_virtual_micro_g=bi_accel_virtual_micro_g,
            tau_bi_accel_virtual=tau_bi_accel_virtual
        )

        print(
            f'\nЧисловые результаты сохранены:\n{result_path}'
        )

    # ========================================================================
    # ЗАВЕРШЕНИЕ
    # ========================================================================

    total_elapsed = time.perf_counter() - total_start

    release_gpu_memory()

    print('\n============================================================')
    print('                 РАСЧЁТ УСПЕШНО ЗАВЕРШЁН')
    print('============================================================')
    print(f'Исключённые IMU:             {EXCLUDED_IMU_1BASED}')
    print(f'Использовано IMU:            {n_active_sensors}')
    print(
        f'Время гироскопов:            '
        f'{gyro_elapsed / 60.0:.2f} мин'
    )
    print(
        f'Время акселерометров:        '
        f'{accel_elapsed / 60.0:.2f} мин'
    )
    print(
        f'Полное время:                '
        f'{total_elapsed / 60.0:.2f} мин'
    )
    print(f'Выходная папка:              {OUTPUT_DIR}')
    print('============================================================\n')


if __name__ == '__main__':
    main()