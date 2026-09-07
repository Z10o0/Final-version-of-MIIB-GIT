# ============================================================================
# ВАРИАЦИЯ АЛЛАНА МАССИВА IMU
# CUDA GPU + HDF5 + RAM float32 + вычисления GPU float64
# ============================================================================
#
# НАЗНАЧЕНИЕ
# ----------
# Этот скрипт выполняет расчёт вариации Аллана (Allan deviation, ADEV) для
# массива IMU по данным из HDF5-файла. Скрипт рассчитан на большой объём данных,
# поэтому использует следующую архитектуру:
#
#   1. Исходные оси читаются из HDF5 крупными блоками.
#   2. Оси временно хранятся в RAM в формате float32.
#   3. Интегрирование theta и вычисление ADEV выполняются на GPU в float64.
#   4. Для каждой оси строятся:
#      - ADEV всех отдельных IMU;
#      - ADEV равновесового виртуального массива;
#      - ADEV кластерно-взвешенного массива.
#
#
# ГЛАВНАЯ ДОБАВКА ЭТОЙ ВЕРСИИ
# ---------------------------
# В данной физической сборке часть датчиков установлена с разворотом платы
# на 180 градусов вокруг оси OZ датчика. Это означает, что локальные оси X и Y
# у этих датчиков направлены противоположно по отношению к остальным датчикам,
# а ось Z остаётся сонаправленной.
#
# Если такую геометрию не компенсировать программно, тогда при усреднении
# по всем датчикам:
#
#   mean(signal_all_imu)
#
# сигналы от одной половины массива по X и Y будут частично вычитаться из
# сигналов другой половины массива, а не складываться. В результате:
#
#   - полезный сигнал массива будет занижен;
#   - оценка выигрыша по ARW / VRW будет искусственно ухудшена;
#   - вместо близкого к теоретическому выигрыша примерно sqrt(36)=6
#     получится заметно меньший выигрыш;
#   - графики и паспортные оценки массива будут искажены.
#
#
# МАТЕМАТИКА ПОВОРОТА
# ------------------
# Для поворота системы координат на 180 градусов вокруг оси OZ используется
# матрица:
#
#     Rz(pi) = [ -1   0   0
#                 0  -1   0
#                 0   0   1 ]
#
# Следовательно, для всех датчиков, которые физически повернуты вокруг OZ,
# программная коррекция должна быть:
#
#     X -> -X
#     Y -> -Y
#     Z ->  Z
#
# Именно это и реализовано в данном файле.
#
#
# КАКИЕ ИМЕННО ДАТЧИКИ ПОВОРАЧИВАЮТСЯ
# -----------------------------------
# В этой версии математически разворачиваются датчики:
#
#     IMU 19 ... IMU 36
#
# То есть ровно половина массива.
#
# Нумерация в пользовательских настройках — ОТ 1.
# Внутри numpy/cupy после этого используется стандартная индексация ОТ 0.
#
#
# ГДЕ ИМЕННО ПРИМЕНЯЕТСЯ КОРРЕКЦИЯ
# --------------------------------
# Коррекция выполняется сразу после загрузки осей из HDF5 в RAM и ДО:
#
#   - вычисления средних;
#   - интегрирования theta;
#   - расчёта вариации Аллана;
#   - формирования равновесового виртуального массива;
#   - кластеризации IMU;
#   - формирования кластерно-взвешенного массива;
#   - построения итоговых графиков.
#
# Благодаря этому вся последующая математика выполняется уже в единой,
# согласованной системе осей.
#
#
# ЧТО СТРОИТСЯ
# ------------
# Для каждой из 3 осей гироскопа и 3 осей акселерометра строятся:
#
#   1. Все отдельные IMU — тонкие серые кривые.
#   2. Равновесовой массив — среднее по всем активным IMU.
#   3. Кластерно-взвешенный массив — среднее по лучшему кластеру,
#      где более качественные IMU получают больший вес.
#
# Дополнительно вычисляются:
#
#   - для гироскопов:
#       ARW  (Angle Random Walk)
#       BI   (Bias Instability)
#
#   - для акселерометров:
#       VRW  (Velocity Random Walk)
#       BI   (Bias Instability)
#
#
# ЛОГИКА КЛАСТЕРИЗАЦИИ
# --------------------
# После расчёта ADEV для отдельных датчиков из каждой оси извлекаются
# показатели качества:
#
#   - белый шум (ARW для гироскопов / VRW для акселерометров);
#   - нестабильность смещения BI.
#
# Затем:
#
#   - эти признаки логарифмируются;
#   - робастно нормализуются;
#   - выполняется K-Means на 2 кластера;
#   - выбирается лучший кластер по интегральной метрике качества;
#   - внутри лучшего кластера формируются веса IMU.
#
#
# ОЖИДАЕМАЯ СТРУКТУРА HDF5
# ------------------------
# Файл должен содержать датасеты:
#
#   gyro:   [n_samples, n_sensors, 3], float32, град/с
#   accel:  [n_samples, n_sensors, 3], float32, g
#   tstamp: [n_samples, n_sensors],    uint16
#
# Если layout отличается, скрипт пытается определить его автоматически,
# но расчёт осей ниже рассчитан на стандартное расположение:
#
#   [time, sensor, xyz]
#
#
# ЕДИНИЦЫ ИЗМЕРЕНИЯ
# -----------------
# Вход:
#   gyro  - град/с
#   accel - g
#
# Выход:
#   графики гироскопов      - ADEV в град/ч
#   графики акселерометров  - ADEV в mg
#   ARW                     - град/√ч
#   VRW                     - м/с/√ч
#   BI гироскопа            - град/ч
#   BI акселерометра        - мкg
#
#
# ВАЖНОЕ ПРИМЕЧАНИЕ ПО remove_mean
# --------------------------------
# Для гироскопов в этой версии remove_mean=False, а для акселерометров
# remove_mean=True, как и было в исходной логике.
#
# Это означает:
#
#   - гироскоп: интегрируется исходный сигнал;
#   - акселерометр: перед интегрированием удаляется среднее по каждому IMU
#     или по виртуальному массиву, чтобы анализировать шумовую составляющую.
#
# Но в обоих случаях разворот IMU 19...36 выполняется ДО этого шага.
#
#
# ВЫХОДНЫЕ ФАЙЛЫ
# --------------
# В папку OUTPUT_DIR сохраняются:
#
#   - PNG-графики ADEV по всем осям гироскопа;
#   - PNG-графики ADEV по всем осям акселерометра;
#   - NPZ-файл с числовыми результатами и служебными массивами.
#
# В NPZ дополнительно записываются:
#
#   - список исключённых IMU;
#   - список развёрнутых IMU;
#   - маски выбранных IMU;
#   - веса кластерного осреднения;
#   - полные массивы Allan deviation.
#
#
# КАК ПОЛЬЗОВАТЬСЯ
# ----------------
# 1. Укажи путь к HDF5-файлу в H5_FILE_PATH.
# 2. При необходимости укажи исключаемые IMU в EXCLUDED_IMU_1BASED.
# 3. Убедись, что список ROTATE_OZ_180_IMU_1BASED содержит именно те
#    датчики, которые физически повернуты вокруг OZ.
# 4. Запусти скрипт.
#
#
# РЕЗЮМЕ ПО КОРРЕКЦИИ ГЕОМЕТРИИ
# -----------------------------
# Эта версия специально исправляет проблему физической сборки, когда одна
# половина массива установлена с поворотом на 180° вокруг OZ. Без этой
# коррекции mean() по всем датчикам давал бы частичное взаимное вычитание
# по X и Y. С коррекцией:
#
#   - X и Y всех датчиков приводятся к общему направлению;
#   - Z остаётся без изменения;
#   - виртуальный массив усредняется корректно;
#   - выигрыш массива по ARW/VRW становится физически осмысленным.
#
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

# Нумерация IMU для пользователя начинается с 1.
EXCLUDED_IMU_1BASED = [5]

# IMU, физически повернутые на 180° вокруг OZ.
# Для них программно выполняется:
#   X -> -X
#   Y -> -Y
#   Z ->  Z
ROTATE_OZ_180_IMU_1BASED = list(range(19, 37))

# ---------------------------------------------------------------------------
# Сетка tau.
# ---------------------------------------------------------------------------

PTS_PER_DECADE = 45
MAX_TAU_SECONDS = 20_000.0
MIN_OVERLAPS = 100_000

# ---------------------------------------------------------------------------
# HDF5 и RAM.
# ---------------------------------------------------------------------------

HDF5_TIME_BLOCK = 4_000_000
RAM_AXIS_DTYPE = np.float32

# ---------------------------------------------------------------------------
# GPU.
# ---------------------------------------------------------------------------

GPU_SENSOR_BATCH = 6
GPU_TIME_CHUNK = 8_000_000
GPU_MEMORY_FRACTION = 0.90

MIN_TOTAL_VRAM_GB = 5.70
MIN_FREE_VRAM_GB = 4.40

# ---------------------------------------------------------------------------
# Кластеризация и веса.
# ---------------------------------------------------------------------------

QUALITY_WEIGHT_WHITE_NOISE = 0.30
QUALITY_WEIGHT_BIAS_INSTABILITY = 0.70
MIN_SELECTED_IMUS = 18
MAX_WEIGHT_RATIO = 2.0
KMEANS_MAX_ITERATIONS = 30

# ---------------------------------------------------------------------------
# Вывод.
# ---------------------------------------------------------------------------

TAU_PROGRESS_STEP = 25
SAVE_NUMERIC_RESULTS = True
PLOT_DPI = 150

# ---------------------------------------------------------------------------
# Константы.
# ---------------------------------------------------------------------------

G_STANDARD = 9.80665
BI_FLICKER_FACTOR = 0.664282


# ============================================================================
# ОБЩИЕ ФУНКЦИИ
# ============================================================================

def positive_finite_values(*arrays):
    values = []

    for array in arrays:
        array = np.asarray(array)
        current = array[np.isfinite(array) & (array > 0.0)]
        if current.size:
            values.append(current)

    if not values:
        return np.array([1e-12], dtype=np.float64)

    return np.concatenate(values)


def get_zero_based_indices_from_one_based(indices_1based, n_sensors, label):
    indices = np.asarray(indices_1based, dtype=np.int64) - 1

    if np.any(indices < 0) or np.any(indices >= n_sensors):
        raise ValueError(
            f'{label}: номера IMU должны быть в диапазоне 1...{n_sensors}'
        )

    return np.unique(indices)


def build_active_sensor_mask(n_sensors):
    excluded_indices = get_zero_based_indices_from_one_based(
        EXCLUDED_IMU_1BASED,
        n_sensors,
        'EXCLUDED_IMU_1BASED'
    )

    active_mask = np.ones(n_sensors, dtype=bool)
    active_mask[excluded_indices] = False

    active_indices = np.flatnonzero(active_mask)

    if active_indices.size == 0:
        raise RuntimeError('После исключения IMU не осталось активных датчиков.')

    return active_mask, active_indices, excluded_indices


def build_rotation_mask(n_sensors):
    rotate_indices = get_zero_based_indices_from_one_based(
        ROTATE_OZ_180_IMU_1BASED,
        n_sensors,
        'ROTATE_OZ_180_IMU_1BASED'
    )

    rotate_mask = np.zeros(n_sensors, dtype=bool)
    rotate_mask[rotate_indices] = True

    return rotate_mask, rotate_indices


# ============================================================================
# CUDA GPU
# ============================================================================

def get_gpu_info():
    try:
        device_id = cp.cuda.runtime.getDevice()
        props = cp.cuda.runtime.getDeviceProperties(device_id)
        free_bytes, total_bytes = cp.cuda.runtime.memGetInfo()
    except Exception as exc:
        raise RuntimeError(
            '\nCUDA GPU недоступен.\n'
            'Проверь NVIDIA driver и установленный CuPy.\n'
            'Для CUDA 12: pip install cupy-cuda12x'
        ) from exc

    gpu_name = props['name']
    if isinstance(gpu_name, bytes):
        gpu_name = gpu_name.decode('utf-8', errors='replace')

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
    print('============================================================\n')

    if total_gb < MIN_TOTAL_VRAM_GB:
        raise RuntimeError(f'Недостаточный общий объём VRAM: {total_gb:.3f} GB')

    if free_gb < MIN_FREE_VRAM_GB:
        raise RuntimeError(
            f'\nСвободно VRAM: {free_gb:.3f} GB\n'
            f'Для текущего расчёта необходимо минимум {MIN_FREE_VRAM_GB:.2f} GB.\n'
            'Закрой программы, использующие GPU, либо уменьши GPU_SENSOR_BATCH.'
        )


def release_gpu_memory():
    cp.cuda.Stream.null.synchronize()
    cp.get_default_memory_pool().free_all_blocks()
    cp.get_default_pinned_memory_pool().free_all_blocks()
    gc.collect()


def check_gpu_memory(n_samples, batch_size):
    free_bytes, _ = cp.cuda.runtime.memGetInfo()

    time_chunk = min(GPU_TIME_CHUNK, n_samples)

    theta_bytes = batch_size * (n_samples + 1) * 8
    d2_bytes = batch_size * time_chunk * 8
    upload_bytes = batch_size * min(HDF5_TIME_BLOCK, n_samples) * 8
    reserve_bytes = 300 * 1024 * 1024

    required_bytes = theta_bytes + d2_bytes + upload_bytes + reserve_bytes
    allowed_bytes = int(free_bytes * GPU_MEMORY_FRACTION)

    if required_bytes > allowed_bytes:
        raise MemoryError(
            '\nНедостаточно VRAM.\n'
            f'Требуется: {required_bytes / (1024 ** 3):.2f} GiB\n'
            f'Доступно:  {allowed_bytes / (1024 ** 3):.2f} GiB\n'
            'Уменьши GPU_SENSOR_BATCH.'
        )


# ============================================================================
# HDF5 И ВРЕМЯ
# ============================================================================

def detect_3d_layout(dset):
    shape = dset.shape

    if len(shape) != 3:
        raise ValueError(f'Ожидался 3D-датасет, получено: {shape}')

    xyz_candidates = [axis for axis, size in enumerate(shape) if size == 3]

    if len(xyz_candidates) != 1:
        raise ValueError(f'Не удалось определить XYZ-ось: {shape}')

    xyz_axis = xyz_candidates[0]

    remaining = [(axis, shape[axis]) for axis in range(3) if axis != xyz_axis]
    remaining.sort(key=lambda item: item[1])

    sensor_axis = remaining[0][0]
    time_axis = remaining[1][0]

    return (
        time_axis,
        sensor_axis,
        xyz_axis,
        int(shape[time_axis]),
        int(shape[sensor_axis])
    )


def detect_tstamp_layout(dset, n_sensors):
    if len(dset.shape) != 2:
        raise ValueError(f'tstamp должен быть 2D: {dset.shape}')

    if dset.shape[1] == n_sensors:
        return 0, 1

    if dset.shape[0] == n_sensors:
        return 1, 0

    raise ValueError(f'Не удалось определить layout tstamp: {dset.shape}')


def estimate_sample_period(h5_path, n_sensors, n_samples):
    with h5py.File(h5_path, 'r') as h5_file:
        tstamp = h5_file['tstamp']
        time_axis, _ = detect_tstamp_layout(tstamp, n_sensors)

        n_check = min(50_000, n_samples)

        if time_axis == 0:
            timestamps = np.asarray(tstamp[:n_check, :], dtype=np.float64).T
        else:
            timestamps = np.asarray(tstamp[:, :n_check], dtype=np.float64)

    nominal_ticks = np.zeros(n_sensors, dtype=np.float64)
    jitter_std = np.zeros(n_sensors, dtype=np.float64)

    for sensor_idx in range(n_sensors):
        sensor_timestamps = timestamps[sensor_idx]

        dt_ticks = np.mod(np.diff(sensor_timestamps), 65536.0)
        dt_ticks = dt_ticks[dt_ticks > 0.0]

        if dt_ticks.size == 0:
            continue

        rounded_ticks = np.round(dt_ticks)
        values, counts = np.unique(rounded_ticks, return_counts=True)
        mode_tick = values[np.argmax(counts)]

        nominal_ticks[sensor_idx] = mode_tick

        stable_ticks = dt_ticks[dt_ticks < mode_tick * 1.5]
        if stable_ticks.size:
            jitter_std[sensor_idx] = np.std(stable_ticks)

    valid_ticks = nominal_ticks[nominal_ticks > 0.0]

    if valid_ticks.size == 0:
        raise RuntimeError('Не удалось определить частоту дискретизации.')

    values, counts = np.unique(np.round(valid_ticks), return_counts=True)
    mean_ticks = values[np.argmax(counts)]

    fs = 1e6 / mean_ticks
    dt0 = 1.0 / fs

    return dt0, fs, mean_ticks, nominal_ticks, jitter_std


def build_tau_grid(n_samples, dt0):
    max_m_by_overlap = (n_samples - MIN_OVERLAPS) // 2

    if max_m_by_overlap < 2:
        raise ValueError('MIN_OVERLAPS слишком велик для записи.')

    max_m_by_tau = int(math.floor(MAX_TAU_SECONDS / dt0))
    max_m = max(2, min(max_m_by_overlap, max_m_by_tau))

    n_decades = np.log10(max_m)
    n_tau_requested = max(3, int(round(n_decades * PTS_PER_DECADE)))

    m_steps = np.unique(
        np.round(np.logspace(0.0, np.log10(max_m), n_tau_requested)).astype(np.int64)
    )

    tau_vals = m_steps.astype(np.float64) * dt0
    idx_tau_1s = int(np.argmin(np.abs(tau_vals - 1.0)))

    return m_steps, tau_vals, idx_tau_1s


# ============================================================================
# РАЗВОРОТ ОСЕЙ IMU 19...36 НА 180° ВОКРУГ OZ
# ============================================================================

def apply_oz_180_rotation_inplace(axis_arrays, axis_indices, rotate_mask):
    """
    Применяет поворот на 180° вокруг OZ к выбранным IMU.

    Для поворота Rz(pi):
      X -> -X
      Y -> -Y
      Z ->  Z

    Параметры
    ---------
    axis_arrays : list[np.ndarray]
        Список массивов формы [n_samples, n_sensors] для осей, перечисленных
        в axis_indices.

    axis_indices : list[int]
        Индексы осей:
          0 -> X
          1 -> Y
          2 -> Z

        Обычно сюда передаётся либо [0, 1], либо [2].

    rotate_mask : np.ndarray(bool)
        Булева маска формы [n_sensors], где True означает, что соответствующий
        датчик физически повернут и должен быть программно развернут.

    Логика
    ------
    Если датчик повернут вокруг OZ на 180°, то:
      - компонента X меняет знак,
      - компонента Y меняет знак,
      - компонента Z не меняется.

    ВАЖНО:
    Эта коррекция применяется до расчёта средних, theta, Allan deviation
    и любых усреднений по массиву.
    """

    for local_axis, axis_idx in enumerate(axis_indices):
        if axis_idx in (0, 1):
            axis_arrays[local_axis][:, rotate_mask] *= -1.0
        elif axis_idx == 2:
            pass
        else:
            raise ValueError(f'Недопустимый индекс оси: {axis_idx}')


# ============================================================================
# ЗАГРУЗКА ОСЕЙ В RAM
# ============================================================================

def print_ram_plan(n_samples, n_sensors, n_axes):
    axis_bytes = n_samples * n_sensors * np.dtype(RAM_AXIS_DTYPE).itemsize
    axes_gib = axis_bytes * n_axes / (1024 ** 3)

    buffer_bytes = (
        min(HDF5_TIME_BLOCK, n_samples)
        * n_sensors
        * n_axes
        * np.dtype(RAM_AXIS_DTYPE).itemsize
    )

    print('====================== ПЛАН RAM ===========================')
    print(f'Оси одновременно:           {n_axes}')
    print(f'RAM постоянных осей:         {axes_gib:.3f} GiB')
    print(f'RAM HDF5-буфера:             {buffer_bytes / (1024 ** 3):.3f} GiB')
    print(f'Тип RAM:                     {RAM_AXIS_DTYPE.__name__}')
    print('============================================================\n')


def load_axes_to_ram(
    dset,
    axis_indices,
    n_samples,
    n_sensors,
    dataset_label,
    rotate_mask
):
    axis_indices = list(axis_indices)

    if len(axis_indices) not in (1, 2):
        raise ValueError('Можно загружать только одну или две оси.')

    expected = list(range(axis_indices[0], axis_indices[0] + len(axis_indices)))
    if axis_indices != expected:
        raise ValueError('Оси должны быть последовательными.')

    n_axes = len(axis_indices)

    print_ram_plan(n_samples, n_sensors, n_axes)

    try:
        axis_arrays = [
            np.empty((n_samples, n_sensors), dtype=RAM_AXIS_DTYPE, order='C')
            for _ in axis_indices
        ]
    except MemoryError as exc:
        raise MemoryError('Не удалось выделить RAM для осей.') from exc

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
            'Не удалось выделить HDF5-буфер. Уменьши HDF5_TIME_BLOCK.'
        ) from exc

    axis_first = axis_indices[0]
    axis_last = axis_indices[-1] + 1

    n_blocks = math.ceil(n_samples / HDF5_TIME_BLOCK)

    print('====================== HDF5 -> RAM ========================')
    print(f'Датасет:                     {dataset_label}')
    print(f'Оси:                         {axis_indices}')
    print(f'HDF5-блок:                   {HDF5_TIME_BLOCK:,}')
    print(f'Количество блоков:           {n_blocks}')
    print('============================================================')

    start_time = time.perf_counter()

    for block_idx, start in enumerate(range(0, n_samples, HDF5_TIME_BLOCK), start=1):
        stop = min(start + HDF5_TIME_BLOCK, n_samples)
        rows = stop - start

        destination = read_buffer[:rows, :, :]

        dset.read_direct(
            destination,
            source_sel=np.s_[start:stop, :, axis_first:axis_last]
        )

        for local_axis in range(n_axes):
            axis_arrays[local_axis][start:stop, :] = destination[:, :, local_axis]

        if (
            block_idx == 1
            or block_idx % 4 == 0
            or block_idx == n_blocks
        ):
            elapsed = time.perf_counter() - start_time
            progress = 100.0 * stop / n_samples
            print(f'  HDF5 -> RAM: {progress:6.2f}% | {block_idx}/{n_blocks} | {elapsed:.1f} с')

    # -----------------------------------------------------------------------
    # КЛЮЧЕВОЙ ШАГ:
    # После загрузки осей в RAM выполняем математический разворот
    # для IMU 19...36 вокруг OZ:
    #
    #   X -> -X
    #   Y -> -Y
    #   Z ->  Z
    #
    # Благодаря этому далее все датчики находятся в единой системе осей.
    # -----------------------------------------------------------------------
    apply_oz_180_rotation_inplace(
        axis_arrays=axis_arrays,
        axis_indices=axis_indices,
        rotate_mask=rotate_mask
    )

    elapsed = time.perf_counter() - start_time

    del read_buffer
    gc.collect()

    print(f'{dataset_label}, оси {axis_indices}: {elapsed:.2f} с\n')

    return axis_arrays


# ============================================================================
# ИНДИВИДУАЛЬНЫЕ IMU: THETA И ADEV
# ============================================================================

def calculate_sensor_means(axis_data, active_sensor_indices):
    n_sensors = axis_data.shape[1]

    means = np.zeros(n_sensors, dtype=np.float64)
    valid_sensors = np.zeros(n_sensors, dtype=bool)

    for sensor_idx in active_sensor_indices:
        signal = axis_data[:, sensor_idx]
        valid = np.isfinite(signal)

        if np.any(valid):
            means[sensor_idx] = np.mean(signal[valid], dtype=np.float64)
            valid_sensors[sensor_idx] = True

        del valid

    gc.collect()
    return means, valid_sensors


def build_theta_from_ram_gpu(axis_data, sensor_indices, dt0, means, remove_mean):
    n_samples = axis_data.shape[0]

    sensor_indices = np.asarray(sensor_indices, dtype=np.int64)
    batch_size = len(sensor_indices)

    check_gpu_memory(n_samples, batch_size)

    d_theta = cp.empty((batch_size, n_samples + 1), dtype=cp.float64)
    d_theta[:, 0] = 0.0

    d_offset = cp.zeros(batch_size, dtype=cp.float64)
    local_means = means[sensor_indices]

    n_blocks = math.ceil(n_samples / HDF5_TIME_BLOCK)
    start_time = time.perf_counter()

    for block_idx, start in enumerate(range(0, n_samples, HDF5_TIME_BLOCK), start=1):
        stop = min(start + HDF5_TIME_BLOCK, n_samples)

        cpu_block = np.ascontiguousarray(
            axis_data[start:stop, :][:, sensor_indices].T,
            dtype=np.float64
        )

        for local_idx in range(batch_size):
            signal = cpu_block[local_idx]
            valid = np.isfinite(signal)

            if not np.all(valid):
                signal[~valid] = local_means[local_idx]

            if remove_mean:
                signal -= local_means[local_idx]

            del valid

        d_block = cp.asarray(cpu_block, dtype=cp.float64)
        theta_slice = d_theta[:, start + 1:stop + 1]

        cp.cumsum(d_block, axis=1, dtype=cp.float64, out=theta_slice)

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
            elapsed = time.perf_counter() - start_time
            progress = 100.0 * stop / n_samples
            print(f'  RAM -> GPU theta: {progress:6.2f}% | {block_idx}/{n_blocks} | {elapsed:.1f} с')

    cp.cuda.Stream.null.synchronize()
    elapsed = time.perf_counter() - start_time

    del d_offset
    return d_theta, elapsed


def compute_oadev_from_theta_gpu(d_theta, n_samples, m_steps, tau_vals):
    batch_size = d_theta.shape[0]
    n_tau = len(m_steps)

    time_chunk = min(GPU_TIME_CHUNK, n_samples)

    d2_buffer = cp.empty((batch_size, time_chunk), dtype=cp.float64)
    d_sum_sq = cp.empty(batch_size, dtype=cp.float64)

    adev = np.full((n_tau, batch_size), np.nan, dtype=np.float64)

    event_start = cp.cuda.Event()
    event_end = cp.cuda.Event()

    event_start.record()

    for tau_idx, m in enumerate(m_steps):
        m = int(m)
        diff_count = n_samples - 2 * m

        if diff_count <= 0:
            continue

        d_sum_sq.fill(0.0)

        for start in range(0, diff_count, time_chunk):
            stop = min(start + time_chunk, diff_count)
            length = stop - start

            d2 = d2_buffer[:, :length]

            d2[...] = d_theta[:, start + 2 * m:stop + 2 * m]
            d2 -= d_theta[:, start + m:stop + m]
            d2 -= d_theta[:, start + m:stop + m]
            d2 += d_theta[:, start:stop]

            cp.multiply(d2, d2, out=d2)
            d_sum_sq += cp.sum(d2, axis=1, dtype=cp.float64)

        denominator = 2.0 * tau_vals[tau_idx] * tau_vals[tau_idx] * float(diff_count)
        d_adev = cp.sqrt(d_sum_sq / denominator)
        adev[tau_idx, :] = cp.asnumpy(d_adev)

        if (
            tau_idx == 0
            or (tau_idx + 1) % TAU_PROGRESS_STEP == 0
            or tau_idx == n_tau - 1
        ):
            cp.cuda.Stream.null.synchronize()
            print(
                f'    tau {tau_idx + 1:3d}/{n_tau}: '
                f'm={m:10d}, tau={tau_vals[tau_idx]:10.4f} с, '
                f'перекрытий={diff_count:,}'
            )

    event_end.record()
    event_end.synchronize()

    gpu_seconds = cp.cuda.get_elapsed_time(event_start, event_end) / 1000.0

    del d2_buffer
    del d_sum_sq

    return adev, gpu_seconds


# ============================================================================
# ВИРТУАЛЬНЫЙ МАССИВ
# ============================================================================

def get_weighted_virtual_mean(axis_data, sensor_indices, weights):
    n_samples = axis_data.shape[0]

    sensor_indices = np.asarray(sensor_indices, dtype=np.int64)
    weights = np.asarray(weights, dtype=np.float64)

    total_sum = 0.0
    total_count = 0

    for start in range(0, n_samples, HDF5_TIME_BLOCK):
        stop = min(start + HDF5_TIME_BLOCK, n_samples)

        block = np.asarray(
            axis_data[start:stop, :][:, sensor_indices],
            dtype=np.float64
        )

        valid = np.isfinite(block)

        weighted_sum = np.sum(
            np.where(valid, block, 0.0) * weights[None, :],
            axis=1,
            dtype=np.float64
        )

        valid_weights = np.sum(
            valid * weights[None, :],
            axis=1,
            dtype=np.float64
        )

        valid_rows = valid_weights > 0.0

        virtual_signal = np.zeros(stop - start, dtype=np.float64)
        virtual_signal[valid_rows] = weighted_sum[valid_rows] / valid_weights[valid_rows]

        total_sum += np.sum(virtual_signal[valid_rows], dtype=np.float64)
        total_count += int(np.count_nonzero(valid_rows))

        del block
        del valid
        del weighted_sum
        del valid_weights
        del valid_rows
        del virtual_signal

    gc.collect()

    if total_count == 0:
        return 0.0, False

    return total_sum / total_count, True


def build_weighted_virtual_theta_from_ram_gpu(
    axis_data,
    sensor_indices,
    weights,
    dt0,
    remove_mean,
    label
):
    n_samples = axis_data.shape[0]

    sensor_indices = np.asarray(sensor_indices, dtype=np.int64)
    weights = np.asarray(weights, dtype=np.float64)

    if sensor_indices.size != weights.size:
        raise ValueError('Число IMU и весов не совпадает.')

    if sensor_indices.size == 0:
        raise ValueError('Нет IMU для виртуального массива.')

    if not np.all(np.isfinite(weights)):
        raise ValueError('Обнаружены невалидные веса.')

    if np.any(weights < 0.0):
        raise ValueError('Вес IMU не может быть отрицательным.')

    weight_sum = np.sum(weights, dtype=np.float64)
    if weight_sum <= 0.0:
        raise ValueError('Сумма весов должна быть больше нуля.')

    weights = weights / weight_sum

    check_gpu_memory(n_samples, batch_size=1)

    virtual_mean = 0.0

    if remove_mean:
        print(f'  Расчёт среднего: {label}')
        virtual_mean, is_valid = get_weighted_virtual_mean(axis_data, sensor_indices, weights)

        if not is_valid:
            raise RuntimeError('Виртуальный IMU не содержит валидных данных.')

    d_theta = cp.empty((1, n_samples + 1), dtype=cp.float64)
    d_theta[:, 0] = 0.0

    d_offset = cp.zeros(1, dtype=cp.float64)

    n_blocks = math.ceil(n_samples / HDF5_TIME_BLOCK)
    start_time = time.perf_counter()

    for block_idx, start in enumerate(range(0, n_samples, HDF5_TIME_BLOCK), start=1):
        stop = min(start + HDF5_TIME_BLOCK, n_samples)

        block = np.asarray(
            axis_data[start:stop, :][:, sensor_indices],
            dtype=np.float64
        )

        valid = np.isfinite(block)

        weighted_sum = np.sum(
            np.where(valid, block, 0.0) * weights[None, :],
            axis=1,
            dtype=np.float64
        )

        valid_weights = np.sum(
            valid * weights[None, :],
            axis=1,
            dtype=np.float64
        )

        virtual_signal = np.zeros(stop - start, dtype=np.float64)
        valid_rows = valid_weights > 0.0
        virtual_signal[valid_rows] = weighted_sum[valid_rows] / valid_weights[valid_rows]

        if remove_mean:
            virtual_signal -= virtual_mean

        d_block = cp.asarray(virtual_signal.reshape(1, -1), dtype=cp.float64)
        theta_slice = d_theta[:, start + 1:stop + 1]

        cp.cumsum(d_block, axis=1, dtype=cp.float64, out=theta_slice)
        theta_slice *= dt0
        theta_slice += d_offset[:, None]

        d_offset = theta_slice[:, -1].copy()

        del block
        del valid
        del weighted_sum
        del valid_weights
        del valid_rows
        del virtual_signal
        del d_block

        if (
            block_idx == 1
            or block_idx % 4 == 0
            or block_idx == n_blocks
        ):
            cp.cuda.Stream.null.synchronize()
            elapsed = time.perf_counter() - start_time
            progress = 100.0 * stop / n_samples
            print(f'  {label}: {progress:6.2f}% | {block_idx}/{n_blocks} | {elapsed:.1f} с')

    cp.cuda.Stream.null.synchronize()
    elapsed = time.perf_counter() - start_time

    del d_offset
    return d_theta, elapsed


# ============================================================================
# ПАРАМЕТРЫ ADEV
# ============================================================================

def extract_gyro_parameters(adev_matrix, adev_virtual, idx_tau_1s, tau_vals):
    n_sensors = adev_matrix.shape[1]

    arw = np.full(n_sensors, np.nan, dtype=np.float64)
    bi = np.full(n_sensors, np.nan, dtype=np.float64)
    tau_bi = np.full(n_sensors, np.nan, dtype=np.float64)

    for sensor_idx in range(n_sensors):
        adev = adev_matrix[:, sensor_idx]

        if not np.any(np.isfinite(adev)):
            continue

        arw[sensor_idx] = adev[idx_tau_1s] * 60.0

        min_idx = int(np.nanargmin(adev))
        bi[sensor_idx] = adev[min_idx] / BI_FLICKER_FACTOR * 3600.0
        tau_bi[sensor_idx] = tau_vals[min_idx]

    min_idx = int(np.nanargmin(adev_virtual))
    arw_virtual = adev_virtual[idx_tau_1s] * 60.0
    bi_virtual = adev_virtual[min_idx] / BI_FLICKER_FACTOR * 3600.0
    tau_bi_virtual = tau_vals[min_idx]

    return arw, bi, tau_bi, arw_virtual, bi_virtual, tau_bi_virtual


def extract_accel_parameters(adev_matrix, adev_virtual, idx_tau_1s, tau_vals):
    n_sensors = adev_matrix.shape[1]

    vrw = np.full(n_sensors, np.nan, dtype=np.float64)
    bi_micro_g = np.full(n_sensors, np.nan, dtype=np.float64)
    tau_bi = np.full(n_sensors, np.nan, dtype=np.float64)

    for sensor_idx in range(n_sensors):
        adev = adev_matrix[:, sensor_idx]

        if not np.any(np.isfinite(adev)):
            continue

        vrw[sensor_idx] = adev[idx_tau_1s] * G_STANDARD * 60.0

        min_idx = int(np.nanargmin(adev))
        bi_micro_g[sensor_idx] = adev[min_idx] / BI_FLICKER_FACTOR * 1e6
        tau_bi[sensor_idx] = tau_vals[min_idx]

    min_idx = int(np.nanargmin(adev_virtual))
    vrw_virtual = adev_virtual[idx_tau_1s] * G_STANDARD * 60.0
    bi_virtual_micro_g = adev_virtual[min_idx] / BI_FLICKER_FACTOR * 1e6
    tau_bi_virtual = tau_vals[min_idx]

    return vrw, bi_micro_g, tau_bi, vrw_virtual, bi_virtual_micro_g, tau_bi_virtual


# ============================================================================
# КЛАСТЕРИЗАЦИЯ И ВЕСА
# ============================================================================

def robust_standardize(values):
    values = np.asarray(values, dtype=np.float64)

    median = np.median(values)
    mad = np.median(np.abs(values - median))
    robust_sigma = 1.4826 * mad

    if not np.isfinite(robust_sigma) or robust_sigma < 1e-12:
        robust_sigma = np.std(values)

    if not np.isfinite(robust_sigma) or robust_sigma < 1e-12:
        robust_sigma = 1.0

    return (values - median) / robust_sigma


def kmeans_two_clusters(features):
    n_samples = features.shape[0]

    if n_samples < 2:
        return np.zeros(n_samples, dtype=np.int64)

    order = np.argsort(features[:, 0])

    center_0 = features[order[0]].copy()
    center_1 = features[order[-1]].copy()

    labels = np.zeros(n_samples, dtype=np.int64)

    for _ in range(KMEANS_MAX_ITERATIONS):
        dist_0 = np.sum((features - center_0) ** 2, axis=1)
        dist_1 = np.sum((features - center_1) ** 2, axis=1)

        new_labels = (dist_1 < dist_0).astype(np.int64)

        if np.any(new_labels == 0) and np.any(new_labels == 1):
            new_center_0 = np.mean(features[new_labels == 0], axis=0)
            new_center_1 = np.mean(features[new_labels == 1], axis=0)
        else:
            median_feature = np.median(features[:, 0])
            new_labels = (features[:, 0] > median_feature).astype(np.int64)

            if not np.any(new_labels == 0):
                new_labels[0] = 0
            if not np.any(new_labels == 1):
                new_labels[-1] = 1

            new_center_0 = np.mean(features[new_labels == 0], axis=0)
            new_center_1 = np.mean(features[new_labels == 1], axis=0)

        centers_changed = (
            not np.allclose(center_0, new_center_0)
            or not np.allclose(center_1, new_center_1)
        )
        labels_changed = not np.array_equal(labels, new_labels)

        labels = new_labels
        center_0 = new_center_0
        center_1 = new_center_1

        if not centers_changed and not labels_changed:
            break

    return labels


def select_cluster_and_weights(white_noise, bias_instability, active_sensor_indices):
    n_sensors = len(white_noise)

    active_sensor_indices = np.asarray(active_sensor_indices, dtype=np.int64)

    full_scores = np.full(n_sensors, np.nan, dtype=np.float64)
    full_weights = np.zeros(n_sensors, dtype=np.float64)
    full_labels = np.full(n_sensors, -1, dtype=np.int64)

    white_active = white_noise[active_sensor_indices]
    bias_active = bias_instability[active_sensor_indices]

    valid = (
        np.isfinite(white_active)
        & np.isfinite(bias_active)
        & (white_active > 0.0)
        & (bias_active > 0.0)
    )

    valid_indices = active_sensor_indices[valid]

    if valid_indices.size < MIN_SELECTED_IMUS:
        raise RuntimeError(
            'Слишком мало IMU с валидными ARW/VRW и BI для кластеризации.'
        )

    log_white = np.log(white_noise[valid_indices])
    log_bias = np.log(bias_instability[valid_indices])

    z_white = robust_standardize(log_white)
    z_bias = robust_standardize(log_bias)

    features = np.column_stack((z_white, z_bias))

    score = (
        QUALITY_WEIGHT_WHITE_NOISE * z_white
        + QUALITY_WEIGHT_BIAS_INSTABILITY * z_bias
    )

    full_scores[valid_indices] = score

    labels = kmeans_two_clusters(features)
    full_labels[valid_indices] = labels

    cluster_score_0 = np.mean(score[labels == 0])
    cluster_score_1 = np.mean(score[labels == 1])

    best_label = 0 if cluster_score_0 <= cluster_score_1 else 1
    selected_indices = valid_indices[labels == best_label]

    if selected_indices.size < MIN_SELECTED_IMUS:
        ordered = valid_indices[np.argsort(score)]
        selected_indices = ordered[:MIN_SELECTED_IMUS]

    selected_scores = full_scores[selected_indices]
    shifted_score = selected_scores - np.min(selected_scores)

    raw_weights = np.exp(-shifted_score)
    raw_weights /= np.min(raw_weights)
    raw_weights = np.clip(raw_weights, 1.0, MAX_WEIGHT_RATIO)

    selected_weights = raw_weights / np.sum(raw_weights, dtype=np.float64)
    full_weights[selected_indices] = selected_weights

    return selected_indices, selected_weights, full_scores, full_weights, full_labels


# ============================================================================
# РАСЧЁТ ОСИ
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
    n_samples, n_sensors = axis_data.shape
    n_tau = len(tau_vals)
    n_active = len(active_sensor_indices)

    axis_start = time.perf_counter()

    adev_matrix = np.full((n_tau, n_sensors), np.nan, dtype=np.float64)

    print('\n============================================================')
    print(f'ОСЬ: {axis_label}')
    print(f'Активных IMU: {n_active}')
    print('Режим: RAM float32 -> GPU float64')
    print('============================================================')

    if remove_mean:
        print('Расчёт средних отдельных IMU...')
        means, valid_sensors = calculate_sensor_means(axis_data, active_sensor_indices)
    else:
        means = np.zeros(n_sensors, dtype=np.float64)
        valid_sensors = active_sensor_mask.copy()

    n_batches = math.ceil(n_active / GPU_SENSOR_BATCH)

    print('\n================ ИНДИВИДУАЛЬНЫЕ IMU ================')

    for batch_number, batch_start in enumerate(
        range(0, n_active, GPU_SENSOR_BATCH),
        start=1
    ):
        batch_indices = active_sensor_indices[batch_start:batch_start + GPU_SENSOR_BATCH]

        imu_numbers = ', '.join(str(int(index) + 1) for index in batch_indices)
        print(f'\nGPU-пакет {batch_number}/{n_batches}: IMU [{imu_numbers}]')

        current_start = time.perf_counter()

        d_theta, theta_seconds = build_theta_from_ram_gpu(
            axis_data=axis_data,
            sensor_indices=batch_indices,
            dt0=dt0,
            means=means,
            remove_mean=remove_mean
        )

        print('  Расчёт ADEV на GPU...')

        adev_batch, adev_seconds = compute_oadev_from_theta_gpu(
            d_theta=d_theta,
            n_samples=n_samples,
            m_steps=m_steps,
            tau_vals=tau_vals
        )

        for local_idx, global_idx in enumerate(batch_indices):
            if valid_sensors[global_idx]:
                adev_matrix[:, global_idx] = adev_batch[:, local_idx]

        del adev_batch
        del d_theta

        release_gpu_memory()

        elapsed = time.perf_counter() - current_start
        print(
            f'  Пакет готов: {elapsed:.2f} с | '
            f'theta: {theta_seconds:.2f} с | '
            f'ADEV GPU: {adev_seconds:.2f} с'
        )

    adev_matrix[:, ~active_sensor_mask] = np.nan

    print('\n================ РАВНОВЕСОВОЙ МАССИВ ================')

    equal_weights = np.full(n_active, 1.0 / n_active, dtype=np.float64)

    virtual_start = time.perf_counter()

    d_theta_virtual, theta_seconds = build_weighted_virtual_theta_from_ram_gpu(
        axis_data=axis_data,
        sensor_indices=active_sensor_indices,
        weights=equal_weights,
        dt0=dt0,
        remove_mean=remove_mean,
        label='Равновесовой theta'
    )

    print('  Расчёт ADEV равновесового массива на GPU...')

    adev_virtual_batch, adev_seconds = compute_oadev_from_theta_gpu(
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
    axis_elapsed = time.perf_counter() - axis_start

    print(
        f'Равновесовой массив: {virtual_elapsed:.2f} с | '
        f'полное время оси: {axis_elapsed / 60.0:.2f} мин'
    )

    return adev_matrix, adev_virtual


def calculate_cluster_weighted_adev(
    axis_data,
    selected_indices,
    selected_weights,
    dt0,
    m_steps,
    tau_vals,
    remove_mean,
    axis_label
):
    print('\n================ КЛАСТЕРНО-ВЗВЕШЕННЫЙ МАССИВ ================')
    print(f'Ось:                         {axis_label}')
    print(f'Выбрано IMU:                 {len(selected_indices)}')

    start_time = time.perf_counter()

    d_theta, theta_seconds = build_weighted_virtual_theta_from_ram_gpu(
        axis_data=axis_data,
        sensor_indices=selected_indices,
        weights=selected_weights,
        dt0=dt0,
        remove_mean=remove_mean,
        label='Кластерно-взвешенный theta'
    )

    print('  Расчёт ADEV взвешенного массива на GPU...')

    adev_batch, adev_seconds = compute_oadev_from_theta_gpu(
        d_theta=d_theta,
        n_samples=axis_data.shape[0],
        m_steps=m_steps,
        tau_vals=tau_vals
    )

    adev_weighted = adev_batch[:, 0]

    del adev_batch
    del d_theta

    release_gpu_memory()

    elapsed = time.perf_counter() - start_time

    print(
        f'Кластерно-взвешенный массив готов: {elapsed:.2f} с | '
        f'theta: {theta_seconds:.2f} с | ADEV GPU: {adev_seconds:.2f} с'
    )

    return adev_weighted


# ============================================================================
# ГРАФИКИ
# ============================================================================

def save_gyro_plot(
    output_path,
    tau_vals,
    adev_matrix,
    adev_equal,
    adev_weighted,
    axis_name,
    axis_short,
    arw,
    bi,
    arw_equal,
    bi_equal,
    tau_bi_equal,
    arw_weighted,
    bi_weighted,
    tau_bi_weighted,
    n_active,
    n_selected,
    active_sensor_indices,
    record_hours,
    fs
):
    adev_sensors_dph = adev_matrix * 3600.0
    adev_equal_dph = adev_equal * 3600.0
    adev_weighted_dph = adev_weighted * 3600.0

    mean_adev_dph = np.nanmean(adev_sensors_dph[:, active_sensor_indices], axis=1)

    mean_arw = np.nanmean(arw[active_sensor_indices])
    mean_bi = np.nanmean(bi[active_sensor_indices])

    gain_arw_equal = mean_arw / arw_equal
    gain_bi_equal = mean_bi / bi_equal

    gain_arw_weighted = arw_equal / arw_weighted
    gain_bi_weighted = bi_equal / bi_weighted

    min_idx_equal = int(np.nanargmin(adev_equal_dph))
    min_idx_weighted = int(np.nanargmin(adev_weighted_dph))

    fig, ax = plt.subplots(figsize=(14, 8.5))

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
        linewidth=1.8,
        label='Средняя ADEV активных IMU'
    )

    ax.loglog(
        tau_vals,
        adev_equal_dph,
        color=(0.08, 0.42, 0.85),
        linewidth=2.8,
        label=f'Равновесовой массив ({n_active} IMU)'
    )

    ax.loglog(
        tau_vals,
        adev_weighted_dph,
        color=(0.02, 0.55, 0.18),
        linewidth=3.0,
        label=f'Кластерно-взвешенный массив ({n_selected} IMU)'
    )

    arw_mask = (
        (tau_vals >= max(tau_vals[0], 0.008))
        & (tau_vals <= min(tau_vals[-1], 15.0))
    )

    if np.any(arw_mask):
        tau_arw = tau_vals[arw_mask]
        arw_curve = arw_weighted / 60.0 / np.sqrt(tau_arw) * 3600.0

        ax.loglog(
            tau_arw,
            arw_curve,
            'r-.',
            linewidth=1.4,
            label='Участок ARW'
        )

    bi_mask = (tau_vals >= tau_bi_weighted * 0.2) & (tau_vals <= tau_bi_weighted * 5.0)

    if np.any(bi_mask):
        ax.loglog(
            tau_vals[bi_mask],
            np.full(np.count_nonzero(bi_mask), adev_weighted_dph[min_idx_weighted]),
            color=(0.95, 0.55, 0.0),
            linestyle='--',
            linewidth=1.5,
            label='Участок BI'
        )

    ax.plot(
        tau_vals[min_idx_equal],
        adev_equal_dph[min_idx_equal],
        marker='o',
        markersize=6,
        markerfacecolor=(0.08, 0.42, 0.85),
        markeredgecolor='black'
    )

    ax.plot(
        tau_vals[min_idx_weighted],
        adev_weighted_dph[min_idx_weighted],
        marker='*',
        markersize=14,
        markerfacecolor=(1.0, 0.85, 0.05),
        markeredgecolor='black'
    )

    passport = (
        f'ПАСПОРТ ГИРОСКОПА ({axis_short})\n'
        '────────────────────────────────────\n'
        f'Активных датчиков: {n_active}\n'
        f'Датчиков в лучшем кластере: {n_selected}\n\n'
        'ARW\n'
        f'Одиночный IMU: {mean_arw:.5f} град/√ч\n'
        f'Равновесовой массив: {arw_equal:.5f} град/√ч\n'
        f'Взвешенный массив: {arw_weighted:.5f} град/√ч\n'
        f'Выигрыш массива: {gain_arw_equal:.3f}×\n'
        f'Выигрыш взвешивания: {gain_arw_weighted:.3f}×\n\n'
        'BI\n'
        f'Одиночный IMU: {mean_bi:.4f} град/ч\n'
        f'Равновесовой массив: {bi_equal:.4f} град/ч\n'
        f'Взвешенный массив: {bi_weighted:.4f} град/ч\n'
        f'Выигрыш массива: {gain_bi_equal:.3f}×\n'
        f'Выигрыш взвешивания: {gain_bi_weighted:.3f}×\n'
        f'Время BI взвешенного массива: {tau_bi_weighted:.2f} с\n'
        '────────────────────────────────────\n'
        f'Частота: {fs:.3f} Гц\n'
        f'Длительность: {record_hours:.3f} ч'
    )

    ax.text(
        0.59,
        0.95,
        passport,
        transform=ax.transAxes,
        fontsize=7.7,
        va='top',
        ha='left',
        bbox=dict(facecolor='white', edgecolor=(0.4, 0.4, 0.4), alpha=0.96)
    )

    all_values = positive_finite_values(mean_adev_dph, adev_equal_dph, adev_weighted_dph)

    ax.set_ylim(np.min(all_values) * 0.25, np.max(all_values) * 4.0)
    ax.set_xlim(tau_vals[0], tau_vals[-1])

    ax.grid(True, which='both', alpha=0.35)

    ax.set_xlabel('Время усреднения tau [с]', fontsize=12, fontweight='bold')
    ax.set_ylabel('Отклонение Аллана (ADEV) [град/ч]', fontsize=12, fontweight='bold')

    ax.set_title(
        f'Вариация Аллана — {axis_name}\n'
        f'Равновесовое и кластерно-взвешенное осреднение',
        fontsize=13,
        fontweight='bold'
    )

    ax.legend(loc='lower left', fontsize=8.0)

    fig.tight_layout()
    fig.savefig(output_path, dpi=PLOT_DPI)
    plt.close(fig)


def save_accel_plot(
    output_path,
    tau_vals,
    adev_matrix,
    adev_equal,
    adev_weighted,
    axis_name,
    axis_short,
    vrw,
    bi_micro_g,
    vrw_equal,
    bi_equal_micro_g,
    tau_bi_equal,
    vrw_weighted,
    bi_weighted_micro_g,
    tau_bi_weighted,
    n_active,
    n_selected,
    active_sensor_indices,
    record_hours,
    fs
):
    adev_sensors_mg = adev_matrix * 1000.0
    adev_equal_mg = adev_equal * 1000.0
    adev_weighted_mg = adev_weighted * 1000.0

    mean_adev_mg = np.nanmean(adev_sensors_mg[:, active_sensor_indices], axis=1)

    mean_vrw = np.nanmean(vrw[active_sensor_indices])
    mean_bi = np.nanmean(bi_micro_g[active_sensor_indices])

    gain_vrw_equal = mean_vrw / vrw_equal
    gain_bi_equal = mean_bi / bi_equal_micro_g

    gain_vrw_weighted = vrw_equal / vrw_weighted
    gain_bi_weighted = bi_equal_micro_g / bi_weighted_micro_g

    min_idx_equal = int(np.nanargmin(adev_equal_mg))
    min_idx_weighted = int(np.nanargmin(adev_weighted_mg))

    fig, ax = plt.subplots(figsize=(14, 8.5))

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
        linewidth=1.8,
        label='Средняя ADEV активных IMU'
    )

    ax.loglog(
        tau_vals,
        adev_equal_mg,
        color=(0.08, 0.42, 0.85),
        linewidth=2.8,
        label=f'Равновесовой массив ({n_active} IMU)'
    )

    ax.loglog(
        tau_vals,
        adev_weighted_mg,
        color=(0.02, 0.55, 0.18),
        linewidth=3.0,
        label=f'Кластерно-взвешенный массив ({n_selected} IMU)'
    )

    vrw_mask = (
        (tau_vals >= max(tau_vals[0], 0.008))
        & (tau_vals <= min(tau_vals[-1], 15.0))
    )

    if np.any(vrw_mask):
        tau_vrw = tau_vals[vrw_mask]
        vrw_g_sqrt_s = vrw_weighted / G_STANDARD / 60.0
        vrw_curve_mg = vrw_g_sqrt_s / np.sqrt(tau_vrw) * 1000.0

        ax.loglog(
            tau_vrw,
            vrw_curve_mg,
            'r-.',
            linewidth=1.4,
            label='Участок VRW'
        )

    bi_mask = (tau_vals >= tau_bi_weighted * 0.2) & (tau_vals <= tau_bi_weighted * 5.0)

    if np.any(bi_mask):
        ax.loglog(
            tau_vals[bi_mask],
            np.full(np.count_nonzero(bi_mask), adev_weighted_mg[min_idx_weighted]),
            color=(0.95, 0.55, 0.0),
            linestyle='--',
            linewidth=1.5,
            label='Участок BI'
        )

    ax.plot(
        tau_vals[min_idx_equal],
        adev_equal_mg[min_idx_equal],
        marker='o',
        markersize=6,
        markerfacecolor=(0.08, 0.42, 0.85),
        markeredgecolor='black'
    )

    ax.plot(
        tau_vals[min_idx_weighted],
        adev_weighted_mg[min_idx_weighted],
        marker='*',
        markersize=14,
        markerfacecolor=(1.0, 0.85, 0.05),
        markeredgecolor='black'
    )

    passport = (
        f'ПАСПОРТ АКСЕЛЕРОМЕТРА ({axis_short})\n'
        '────────────────────────────────────\n'
        f'Активных датчиков: {n_active}\n'
        f'Датчиков в лучшем кластере: {n_selected}\n\n'
        'VRW\n'
        f'Одиночный IMU: {mean_vrw:.5f} м/с/√ч\n'
        f'Равновесовой массив: {vrw_equal:.5f} м/с/√ч\n'
        f'Взвешенный массив: {vrw_weighted:.5f} м/с/√ч\n'
        f'Выигрыш массива: {gain_vrw_equal:.3f}×\n'
        f'Выигрыш взвешивания: {gain_vrw_weighted:.3f}×\n\n'
        'BI\n'
        f'Одиночный IMU: {mean_bi:.3f} мкg\n'
        f'Равновесовой массив: {bi_equal_micro_g:.3f} мкg\n'
        f'Взвешенный массив: {bi_weighted_micro_g:.3f} мкg\n'
        f'Выигрыш массива: {gain_bi_equal:.3f}×\n'
        f'Выигрыш взвешивания: {gain_bi_weighted:.3f}×\n'
        f'Время BI взвешенного массива: {tau_bi_weighted:.2f} с\n'
        '────────────────────────────────────\n'
        f'Частота: {fs:.3f} Гц\n'
        f'Длительность: {record_hours:.3f} ч'
    )

    ax.text(
        0.57,
        0.95,
        passport,
        transform=ax.transAxes,
        fontsize=7.45,
        va='top',
        ha='left',
        bbox=dict(facecolor='white', edgecolor=(0.4, 0.4, 0.4), alpha=0.96)
    )

    all_values = positive_finite_values(mean_adev_mg, adev_equal_mg, adev_weighted_mg)

    ax.set_ylim(np.min(all_values) * 0.25, np.max(all_values) * 4.0)
    ax.set_xlim(tau_vals[0], tau_vals[-1])

    ax.grid(True, which='both', alpha=0.35)

    ax.set_xlabel('Время усреднения tau [с]', fontsize=12, fontweight='bold')
    ax.set_ylabel('Отклонение Аллана (ADEV) [mg]', fontsize=12, fontweight='bold')

    ax.set_title(
        f'Вариация Аллана — {axis_name}\n'
        f'Равновесовое и кластерно-взвешенное осреднение',
        fontsize=13,
        fontweight='bold'
    )

    ax.legend(loc='lower left', fontsize=8.0)

    fig.tight_layout()
    fig.savefig(output_path, dpi=PLOT_DPI)
    plt.close(fig)


# ============================================================================
# ОБРАБОТКА ВСЕХ ОСЕЙ ОДНОГО ДАТАСЕТА
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
    rotate_mask,
    remove_mean,
    sensor_type,
    idx_tau_1s
):
    (
        time_axis,
        sensor_axis,
        xyz_axis,
        n_samples,
        n_sensors
    ) = layout

    if not (time_axis == 0 and sensor_axis == 1 and xyz_axis == 2):
        raise ValueError(f'Ожидался layout [time, sensor, xyz], получено: {dset.shape}')

    n_tau = len(tau_vals)

    allan_individual = np.full((n_tau, n_sensors, 3), np.nan, dtype=np.float64)
    allan_equal = np.full((n_tau, 3), np.nan, dtype=np.float64)
    allan_weighted = np.full((n_tau, 3), np.nan, dtype=np.float64)

    selected_indices_all = np.full((n_sensors, 3), False, dtype=bool)
    weights_all = np.zeros((n_sensors, 3), dtype=np.float64)
    scores_all = np.full((n_sensors, 3), np.nan, dtype=np.float64)
    labels_all = np.full((n_sensors, 3), -1, dtype=np.int64)

    white_noise_all = np.full((n_sensors, 3), np.nan, dtype=np.float64)
    bias_all = np.full((n_sensors, 3), np.nan, dtype=np.float64)

    white_noise_equal = np.full(3, np.nan, dtype=np.float64)
    bias_equal = np.full(3, np.nan, dtype=np.float64)
    tau_bi_equal = np.full(3, np.nan, dtype=np.float64)

    white_noise_weighted = np.full(3, np.nan, dtype=np.float64)
    bias_weighted = np.full(3, np.nan, dtype=np.float64)
    tau_bi_weighted = np.full(3, np.nan, dtype=np.float64)

    axis_groups = [
        [0, 1],
        [2]
    ]

    for axis_group in axis_groups:
        print('\n============================================================')
        print(f'{dataset_name}: загрузка осей {axis_group}')
        print('============================================================')

        axis_arrays = load_axes_to_ram(
            dset=dset,
            axis_indices=axis_group,
            n_samples=n_samples,
            n_sensors=n_sensors,
            dataset_label=dataset_name,
            rotate_mask=rotate_mask
        )

        for local_axis, axis_idx in enumerate(axis_group):
            axis_data = axis_arrays[local_axis]

            adev_matrix, adev_equal = calculate_axis_fast(
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
            allan_equal[:, axis_idx] = adev_equal

            if sensor_type == 'gyro':
                (
                    white_noise,
                    bias,
                    _,
                    white_equal,
                    bias_eq,
                    tau_eq
                ) = extract_gyro_parameters(
                    adev_matrix=adev_matrix,
                    adev_virtual=adev_equal,
                    idx_tau_1s=idx_tau_1s,
                    tau_vals=tau_vals
                )
            else:
                (
                    white_noise,
                    bias,
                    _,
                    white_equal,
                    bias_eq,
                    tau_eq
                ) = extract_accel_parameters(
                    adev_matrix=adev_matrix,
                    adev_virtual=adev_equal,
                    idx_tau_1s=idx_tau_1s,
                    tau_vals=tau_vals
                )

            (
                selected_indices,
                selected_weights,
                scores,
                full_weights,
                labels
            ) = select_cluster_and_weights(
                white_noise=white_noise,
                bias_instability=bias,
                active_sensor_indices=active_sensor_indices
            )

            adev_weighted = calculate_cluster_weighted_adev(
                axis_data=axis_data,
                selected_indices=selected_indices,
                selected_weights=selected_weights,
                dt0=dt0,
                m_steps=m_steps,
                tau_vals=tau_vals,
                remove_mean=remove_mean,
                axis_label=axis_names[axis_idx]
            )

            if sensor_type == 'gyro':
                (
                    _,
                    _,
                    _,
                    white_weighted,
                    bias_w,
                    tau_w
                ) = extract_gyro_parameters(
                    adev_matrix=adev_matrix,
                    adev_virtual=adev_weighted,
                    idx_tau_1s=idx_tau_1s,
                    tau_vals=tau_vals
                )
            else:
                (
                    _,
                    _,
                    _,
                    white_weighted,
                    bias_w,
                    tau_w
                ) = extract_accel_parameters(
                    adev_matrix=adev_matrix,
                    adev_virtual=adev_weighted,
                    idx_tau_1s=idx_tau_1s,
                    tau_vals=tau_vals
                )

            allan_weighted[:, axis_idx] = adev_weighted

            selected_indices_all[selected_indices, axis_idx] = True
            weights_all[:, axis_idx] = full_weights
            scores_all[:, axis_idx] = scores
            labels_all[:, axis_idx] = labels

            white_noise_all[:, axis_idx] = white_noise
            bias_all[:, axis_idx] = bias

            white_noise_equal[axis_idx] = white_equal
            bias_equal[axis_idx] = bias_eq
            tau_bi_equal[axis_idx] = tau_eq

            white_noise_weighted[axis_idx] = white_weighted
            bias_weighted[axis_idx] = bias_w
            tau_bi_weighted[axis_idx] = tau_w

            del adev_matrix
            del adev_equal
            del adev_weighted
            del white_noise
            del bias
            del scores
            del full_weights
            del labels
            del selected_indices
            del selected_weights

            gc.collect()

        axis_arrays.clear()
        del axis_arrays
        gc.collect()

    return {
        'allan_individual': allan_individual,
        'allan_equal': allan_equal,
        'allan_weighted': allan_weighted,
        'selected_mask': selected_indices_all,
        'weights': weights_all,
        'scores': scores_all,
        'labels': labels_all,
        'white_noise': white_noise_all,
        'bias': bias_all,
        'white_noise_equal': white_noise_equal,
        'bias_equal': bias_equal,
        'tau_bi_equal': tau_bi_equal,
        'white_noise_weighted': white_noise_weighted,
        'bias_weighted': bias_weighted,
        'tau_bi_weighted': tau_bi_weighted
    }


# ============================================================================
# MAIN
# ============================================================================

def main():
    total_start = time.perf_counter()

    if not os.path.isfile(H5_FILE_PATH):
        raise FileNotFoundError(f'Не найден HDF5-файл:\n{H5_FILE_PATH}')

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    print('\n============================================================')
    print('    ВАРИАЦИЯ АЛЛАНА: GPU + КЛАСТЕРНО-ВЗВЕШЕННЫЙ МАССИВ')
    print('    С МАТЕМАТИЧЕСКИМ РАЗВОРОТОМ IMU 19...36 ВОКРУГ OZ')
    print('============================================================\n')

    get_gpu_info()

    with h5py.File(H5_FILE_PATH, 'r') as h5_file:
        for dataset_name in ('gyro', 'accel', 'tstamp'):
            if dataset_name not in h5_file:
                raise KeyError(f'В HDF5 отсутствует /{dataset_name}')

        gyro_shape = h5_file['gyro'].shape
        accel_shape = h5_file['accel'].shape
        tstamp_shape = h5_file['tstamp'].shape

        gyro_dtype = h5_file['gyro'].dtype
        accel_dtype = h5_file['accel'].dtype

        gyro_chunks = h5_file['gyro'].chunks
        accel_chunks = h5_file['accel'].chunks

        gyro_compression = h5_file['gyro'].compression
        accel_compression = h5_file['accel'].compression

        gyro_layout = detect_3d_layout(h5_file['gyro'])
        accel_layout = detect_3d_layout(h5_file['accel'])

    (
        _,
        _,
        _,
        n_samples,
        n_sensors
    ) = gyro_layout

    if accel_layout[3] != n_samples:
        raise ValueError('gyro и accel имеют разное число кадров.')

    if accel_layout[4] != n_sensors:
        raise ValueError('gyro и accel имеют разное число IMU.')

    active_sensor_mask, active_sensor_indices, excluded_indices = build_active_sensor_mask(n_sensors)
    rotate_mask, rotate_indices = build_rotation_mask(n_sensors)

    n_active = len(active_sensor_indices)

    print('======================= HDF5 DATA =========================')
    print(f'Файл:                        {H5_FILE_PATH}')
    print(f'gyro:                        {gyro_shape}')
    print(f'accel:                       {accel_shape}')
    print(f'tstamp:                      {tstamp_shape}')
    print(f'Тип gyro:                    {gyro_dtype}')
    print(f'Тип accel:                   {accel_dtype}')
    print(f'Chunks gyro:                 {gyro_chunks}')
    print(f'Chunks accel:                {accel_chunks}')
    print(f'Сжатие gyro:                 {gyro_compression}')
    print(f'Сжатие accel:                {accel_compression}')
    print(f'Количество кадров:           {n_samples:,}')
    print(f'Количество IMU:              {n_sensors}')
    print(f'Активных IMU:                {n_active}')
    print(f'Исключённые IMU:             {EXCLUDED_IMU_1BASED}')
    print(f'Поворачиваем вокруг OZ:      {ROTATE_OZ_180_IMU_1BASED}')
    print('============================================================\n')

    dt0, fs, mean_ticks, nominal_ticks, jitter_std = estimate_sample_period(
        h5_path=H5_FILE_PATH,
        n_sensors=n_sensors,
        n_samples=n_samples
    )

    valid_ticks = nominal_ticks[nominal_ticks > 0.0]
    record_hours = n_samples * dt0 / 3600.0

    print('================ АНАЛИЗ СИНХРОНИЗАЦИИ ====================')
    print(f'Номинальный шаг:             {mean_ticks:.0f} тиков ({dt0 * 1000.0:.6f} мс)')
    print(f'Частота:                     {fs:.4f} Гц')
    print(f'Шаги IMU:                    {np.min(valid_ticks):.0f} / {np.max(valid_ticks):.0f} тиков')
    print(f'Средний джиттер:             {np.mean(jitter_std):.6f} мкс')
    print(f'Длительность:                {record_hours:.3f} ч')
    print('============================================================\n')

    m_steps, tau_vals, idx_tau_1s = build_tau_grid(n_samples=n_samples, dt0=dt0)

    print('======================== TAU GRID =========================')
    print(f'Точек tau:                   {len(tau_vals)}')
    print(f'tau min:                     {tau_vals[0]:.8f} с')
    print(f'tau max:                     {tau_vals[-1]:.3f} с')
    print(f'tau около 1 с:               {tau_vals[idx_tau_1s]:.6f} с')
    print(f'Перекрытий на tau max:       {n_samples - 2 * int(m_steps[-1]):,}')
    print('============================================================\n')

    gyro_names = ['Гироскоп X (Gx)', 'Гироскоп Y (Gy)', 'Гироскоп Z (Gz)']
    gyro_short_names = ['Gx', 'Gy', 'Gz']

    accel_names = ['Акселерометр X (Ax)', 'Акселерометр Y (Ay)', 'Акселерометр Z (Az)']
    accel_short_names = ['Ax', 'Ay', 'Az']

    print('\n============================================================')
    print('                       ГИРОСКОПЫ')
    print('============================================================')

    gyro_start = time.perf_counter()

    with h5py.File(
        H5_FILE_PATH,
        'r',
        rdcc_nbytes=1024 * 1024 * 1024,
        rdcc_nslots=1_000_003
    ) as h5_file:

        gyro_result = process_dataset_axes(
            dset=h5_file['gyro'],
            layout=gyro_layout,
            dataset_name='ГИРОСКОПЫ',
            axis_names=gyro_names,
            dt0=dt0,
            m_steps=m_steps,
            tau_vals=tau_vals,
            active_sensor_mask=active_sensor_mask,
            active_sensor_indices=active_sensor_indices,
            rotate_mask=rotate_mask,
            remove_mean=False,
            sensor_type='gyro',
            idx_tau_1s=idx_tau_1s
        )

    gyro_elapsed = time.perf_counter() - gyro_start

    print('\n============================================================')
    print('                    АКСЕЛЕРОМЕТРЫ')
    print('============================================================')

    accel_start = time.perf_counter()

    with h5py.File(
        H5_FILE_PATH,
        'r',
        rdcc_nbytes=1024 * 1024 * 1024,
        rdcc_nslots=1_000_003
    ) as h5_file:

        accel_result = process_dataset_axes(
            dset=h5_file['accel'],
            layout=accel_layout,
            dataset_name='АКСЕЛЕРОМЕТРЫ',
            axis_names=accel_names,
            dt0=dt0,
            m_steps=m_steps,
            tau_vals=tau_vals,
            active_sensor_mask=active_sensor_mask,
            active_sensor_indices=active_sensor_indices,
            rotate_mask=rotate_mask,
            remove_mean=True,
            sensor_type='accel',
            idx_tau_1s=idx_tau_1s
        )

    accel_elapsed = time.perf_counter() - accel_start

    print('\nПостроение графиков гироскопов...')

    for axis_idx in range(3):
        n_selected = int(np.count_nonzero(gyro_result['selected_mask'][:, axis_idx]))

        save_gyro_plot(
            output_path=os.path.join(
                OUTPUT_DIR,
                f'allan_gyro_{gyro_short_names[axis_idx]}_clustered_gpu.png'
            ),
            tau_vals=tau_vals,
            adev_matrix=gyro_result['allan_individual'][:, :, axis_idx],
            adev_equal=gyro_result['allan_equal'][:, axis_idx],
            adev_weighted=gyro_result['allan_weighted'][:, axis_idx],
            axis_name=gyro_names[axis_idx],
            axis_short=gyro_short_names[axis_idx],
            arw=gyro_result['white_noise'][:, axis_idx],
            bi=gyro_result['bias'][:, axis_idx],
            arw_equal=gyro_result['white_noise_equal'][axis_idx],
            bi_equal=gyro_result['bias_equal'][axis_idx],
            tau_bi_equal=gyro_result['tau_bi_equal'][axis_idx],
            arw_weighted=gyro_result['white_noise_weighted'][axis_idx],
            bi_weighted=gyro_result['bias_weighted'][axis_idx],
            tau_bi_weighted=gyro_result['tau_bi_weighted'][axis_idx],
            n_active=n_active,
            n_selected=n_selected,
            active_sensor_indices=active_sensor_indices,
            record_hours=record_hours,
            fs=fs
        )

    print('Построение графиков акселерометров...')

    for axis_idx in range(3):
        n_selected = int(np.count_nonzero(accel_result['selected_mask'][:, axis_idx]))

        save_accel_plot(
            output_path=os.path.join(
                OUTPUT_DIR,
                f'allan_accel_{accel_short_names[axis_idx]}_clustered_gpu.png'
            ),
            tau_vals=tau_vals,
            adev_matrix=accel_result['allan_individual'][:, :, axis_idx],
            adev_equal=accel_result['allan_equal'][:, axis_idx],
            adev_weighted=accel_result['allan_weighted'][:, axis_idx],
            axis_name=accel_names[axis_idx],
            axis_short=accel_short_names[axis_idx],
            vrw=accel_result['white_noise'][:, axis_idx],
            bi_micro_g=accel_result['bias'][:, axis_idx],
            vrw_equal=accel_result['white_noise_equal'][axis_idx],
            bi_equal_micro_g=accel_result['bias_equal'][axis_idx],
            tau_bi_equal=accel_result['tau_bi_equal'][axis_idx],
            vrw_weighted=accel_result['white_noise_weighted'][axis_idx],
            bi_weighted_micro_g=accel_result['bias_weighted'][axis_idx],
            tau_bi_weighted=accel_result['tau_bi_weighted'][axis_idx],
            n_active=n_active,
            n_selected=n_selected,
            active_sensor_indices=active_sensor_indices,
            record_hours=record_hours,
            fs=fs
        )

    if SAVE_NUMERIC_RESULTS:
        result_path = os.path.join(
            OUTPUT_DIR,
            'allan_gpu_clustered_weighted_results_rotated_oz_180.npz'
        )

        np.savez_compressed(
            result_path,

            h5_file_path=H5_FILE_PATH,

            dt0=dt0,
            fs=fs,

            n_samples=n_samples,
            n_sensors=n_sensors,
            n_active=n_active,

            excluded_imu_1based=np.asarray(EXCLUDED_IMU_1BASED, dtype=np.int64),
            excluded_imu_zerobased=excluded_indices,

            rotated_imu_1based=np.asarray(ROTATE_OZ_180_IMU_1BASED, dtype=np.int64),
            rotated_imu_zerobased=rotate_indices,

            active_sensor_indices=active_sensor_indices,

            m_steps=m_steps,
            tau_vals=tau_vals,

            gyro_allan_individual=gyro_result['allan_individual'],
            gyro_allan_equal=gyro_result['allan_equal'],
            gyro_allan_weighted=gyro_result['allan_weighted'],
            gyro_selected_mask=gyro_result['selected_mask'],
            gyro_weights=gyro_result['weights'],
            gyro_scores=gyro_result['scores'],
            gyro_labels=gyro_result['labels'],
            gyro_arw=gyro_result['white_noise'],
            gyro_bi_deg_per_hour=gyro_result['bias'],
            gyro_arw_equal=gyro_result['white_noise_equal'],
            gyro_bi_equal_deg_per_hour=gyro_result['bias_equal'],
            gyro_tau_bi_equal=gyro_result['tau_bi_equal'],
            gyro_arw_weighted=gyro_result['white_noise_weighted'],
            gyro_bi_weighted_deg_per_hour=gyro_result['bias_weighted'],
            gyro_tau_bi_weighted=gyro_result['tau_bi_weighted'],

            accel_allan_individual=accel_result['allan_individual'],
            accel_allan_equal=accel_result['allan_equal'],
            accel_allan_weighted=accel_result['allan_weighted'],
            accel_selected_mask=accel_result['selected_mask'],
            accel_weights=accel_result['weights'],
            accel_scores=accel_result['scores'],
            accel_labels=accel_result['labels'],
            accel_vrw=accel_result['white_noise'],
            accel_bi_micro_g=accel_result['bias'],
            accel_vrw_equal=accel_result['white_noise_equal'],
            accel_bi_equal_micro_g=accel_result['bias_equal'],
            accel_tau_bi_equal=accel_result['tau_bi_equal'],
            accel_vrw_weighted=accel_result['white_noise_weighted'],
            accel_bi_weighted_micro_g=accel_result['bias_weighted'],
            accel_tau_bi_weighted=accel_result['tau_bi_weighted'],

            quality_weight_white_noise=QUALITY_WEIGHT_WHITE_NOISE,
            quality_weight_bias_instability=QUALITY_WEIGHT_BIAS_INSTABILITY,
            min_selected_imus=MIN_SELECTED_IMUS,
            max_weight_ratio=MAX_WEIGHT_RATIO
        )

        print(f'\nЧисловые результаты сохранены:\n{result_path}')

    print('\n============================================================')
    print('                 ИТОГИ КЛАСТЕРИЗАЦИИ')
    print('============================================================')

    print('\nГИРОСКОПЫ')
    for axis_idx in range(3):
        selected = np.flatnonzero(gyro_result['selected_mask'][:, axis_idx]) + 1

        print(f'\n{gyro_names[axis_idx]}')
        print(f'  IMU лучшего кластера:      {selected.tolist()}')
        print(f'  ARW равновесовой:          {gyro_result["white_noise_equal"][axis_idx]:.6f} град/√ч')
        print(f'  ARW взвешенной:            {gyro_result["white_noise_weighted"][axis_idx]:.6f} град/√ч')
        print(f'  BI равновесовой:           {gyro_result["bias_equal"][axis_idx]:.4f} град/ч')
        print(f'  BI взвешенной:             {gyro_result["bias_weighted"][axis_idx]:.4f} град/ч')

    print('\nАКСЕЛЕРОМЕТРЫ')
    for axis_idx in range(3):
        selected = np.flatnonzero(accel_result['selected_mask'][:, axis_idx]) + 1

        print(f'\n{accel_names[axis_idx]}')
        print(f'  IMU лучшего кластера:      {selected.tolist()}')
        print(f'  VRW равновесовой:          {accel_result["white_noise_equal"][axis_idx]:.6f} м/с/√ч')
        print(f'  VRW взвешенной:            {accel_result["white_noise_weighted"][axis_idx]:.6f} м/с/√ч')
        print(f'  BI равновесовой:           {accel_result["bias_equal"][axis_idx]:.3f} мкg')
        print(f'  BI взвешенной:             {accel_result["bias_weighted"][axis_idx]:.3f} мкg')

    release_gpu_memory()

    total_elapsed = time.perf_counter() - total_start

    print('\n============================================================')
    print('                 РАСЧЁТ УСПЕШНО ЗАВЕРШЁН')
    print('============================================================')
    print(f'Время гироскопов:            {gyro_elapsed / 60.0:.2f} мин')
    print(f'Время акселерометров:        {accel_elapsed / 60.0:.2f} мин')
    print(f'Полное время:                {total_elapsed / 60.0:.2f} мин')
    print(f'Выходная папка:              {OUTPUT_DIR}')
    print('============================================================\n')


if __name__ == '__main__':
    main()