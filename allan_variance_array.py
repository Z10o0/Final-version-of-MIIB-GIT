# =========================================================================
# КЛАССИЧЕСКИЙ РАСЧЕТ ВАРИАЦИИ АЛЛАНА (IEEE STD 952 / IEEE STD 1293)
# Двухпараметрическая академическая модель (ARW/VRW + Bias Instability)
# Полная оптимизация на NUMBA: параллелизм по TAU + SIMD AVX
# =========================================================================
import os
import time
import h5py
import numpy as np
import matplotlib.pyplot as plt
import numba as nb
from numba import prange

# ===================== 1. НАСТРОЙКИ ЭКСПЕРИМЕНТА =====================
H5_FILE_PATH   = 'miib_data_single_12h.h5'   # Имя HDF5-файла
PTS_PER_DECADE = 45                          # Академическая плотность сетки tau

if not os.path.exists(H5_FILE_PATH):
    raise FileNotFoundError(f'Файл {H5_FILE_PATH} не найден! Проверьте путь к файлу.')

# ===================== 2. ВЫСОКОПРОИЗВОДИТЕЛЬНЫЕ ЯДРА НА NUMBA =====================
@nb.njit(nogil=True)
def _preprocess_signal_1d(sig, dt0, remove_mean=False):
    """Линейная интерполяция NaN, центрирование и расчет интеграла theta."""
    n_samples = len(sig)
    x = np.empty(n_samples, dtype=np.float64)
    for i in range(n_samples):
        x[i] = sig[i]

    first_valid = -1
    for i in range(n_samples):
        if not np.isnan(x[i]):
            first_valid = i
            break

    if first_valid == -1:
        return np.zeros(0, dtype=np.float64), False

    for i in range(first_valid):
        x[i] = x[first_valid]

    last_valid = first_valid
    for i in range(first_valid + 1, n_samples):
        if not np.isnan(x[i]):
            if i > last_valid + 1:
                v0 = x[last_valid]
                v1 = x[i]
                dx = float(i - last_valid)
                for j in range(last_valid + 1, i):
                    x[j] = v0 + (v1 - v0) * float(j - last_valid) / dx
            last_valid = i

    for i in range(last_valid + 1, n_samples):
        x[i] = x[last_valid]

    if remove_mean:
        mean_val = 0.0
        for i in range(n_samples):
            mean_val += x[i]
        mean_val /= n_samples
        for i in range(n_samples):
            x[i] -= mean_val

    th = np.empty(n_samples + 1, dtype=np.float64)
    th[0] = 0.0
    acc = 0.0
    for i in range(n_samples):
        acc += x[i] * dt0
        th[i + 1] = acc

    return th, True


@nb.njit(parallel=True, fastmath=True, nogil=True)
def numba_calc_oavar_parallel_tau(th, m_steps, tau_vals, n_samples):
    """Параллельный расчет точек tau по всем ядрам процессора."""
    n_tau = len(tau_vals)
    adev = np.empty(n_tau, dtype=np.float64)

    for k in prange(n_tau):
        m = int(m_steps[k])
        tau = tau_vals[k]
        diff_count = n_samples - 2 * m
        if diff_count <= 0:
            adev[k] = np.nan
            continue

        sum_sq = 0.0
        for i in range(diff_count):
            d2 = th[i + 2 * m] - 2.0 * th[i + m] + th[i]
            sum_sq += d2 * d2

        avar = sum_sq / (2.0 * (tau * tau) * float(diff_count))
        adev[k] = np.sqrt(avar)

    return adev


def compute_oavar_matrix(data_matrix, dt0, m_steps, tau_vals, remove_mean=False):
    n_samples, n_sensors = data_matrix.shape
    n_tau = len(tau_vals)
    adev_out = np.full((n_tau, n_sensors), np.nan, dtype=np.float64)

    for s in range(n_sensors):
        col = np.ascontiguousarray(data_matrix[:, s])
        th, is_valid = _preprocess_signal_1d(col, dt0, remove_mean=remove_mean)
        if is_valid:
            adev_out[:, s] = numba_calc_oavar_parallel_tau(th, m_steps, tau_vals, n_samples)

    return adev_out


def compute_oavar_single(sig_1d, dt0, m_steps, tau_vals, remove_mean=False):
    n_samples = len(sig_1d)
    col = np.ascontiguousarray(sig_1d)
    th, is_valid = _preprocess_signal_1d(col, dt0, remove_mean=remove_mean)
    if is_valid:
        return numba_calc_oavar_parallel_tau(th, m_steps, tau_vals, n_samples)
    return np.full(len(tau_vals), np.nan, dtype=np.float64)


def get_dataset_axis(dset, axis_idx):
    shape = dset.shape
    if shape[0] == 3:
        return np.ascontiguousarray(dset[axis_idx, :, :].T.astype(np.float64))
    elif shape[2] == 3:
        return np.ascontiguousarray(dset[:, :, axis_idx].astype(np.float64))
    elif shape[1] == 3:
        return np.ascontiguousarray(dset[:, axis_idx, :].astype(np.float64))
    else:
        raise ValueError(f"Не удалось определить ориентацию датасета: {shape}")


def warmup_numba():
    print("Инициализация и JIT-компиляция Numba для всех ядер CPU...")
    dummy_data = np.ones(200, dtype=np.float64)
    dummy_m = np.array([1, 2, 4], dtype=np.int64)
    dummy_tau = dummy_m * 0.001
    th, _ = _preprocess_signal_1d(dummy_data, 0.001, False)
    _ = numba_calc_oavar_parallel_tau(th, dummy_m, dummy_tau, 200)
    print(f"JIT-компиляция завершена! Активных потоков: {nb.get_num_threads()}\n")


# ===================== 3. ОСНОВНОЙ ПАЙПЛАЙН ВЫЧИСЛЕНИЙ =====================
if __name__ == '__main__':
    warmup_numba()

    # ===================== КРОСС-АНАЛИЗ СИНХРОНИЗАЦИИ =====================
    print('================== 1. КРОСС-АНАЛИЗ СИНХРОНИЗАЦИИ МАССИВА ==================')
    with h5py.File(H5_FILE_PATH, 'r') as f:
        gyro_shape = f['gyro'].shape
        tstamp_dset = f['tstamp']
        tstamp_shape = tstamp_dset.shape

        n_samples = max(gyro_shape)
        other_dims = sorted(gyro_shape)
        n_sensors = other_dims[1]

        n_check = min(50000, n_samples)
        if tstamp_shape[0] == n_sensors:
            all_ts = tstamp_dset[:, :n_check].astype(np.float64)
        else:
            all_ts = tstamp_dset[:n_check, :].T.astype(np.float64)

    nominal_steps = np.zeros(n_sensors)
    jitter_std    = np.zeros(n_sensors)

    for s in range(n_sensors):
        ts = all_ts[s, :]
        d_ts = np.mod(np.diff(ts), 65536)
        d_ts = d_ts[d_ts > 0]
        if d_ts.size > 0:
            vals, counts = np.unique(np.round(d_ts), return_counts=True)
            mode_val = vals[np.argmax(counts)]
            nominal_steps[s] = mode_val
            sub = d_ts[d_ts < mode_val * 1.5]
            jitter_std[s] = np.std(sub) if sub.size > 0 else 0.0

    pos_steps = nominal_steps[nominal_steps > 0]
    vals, counts = np.unique(np.round(pos_steps), return_counts=True)
    mean_ticks = vals[np.argmax(counts)]

    Fs  = 1e6 / mean_ticks
    dt0 = 1.0 / Fs

    print(f'Номинальный шаг таймера:                 {mean_ticks:.0f} тиков ({dt0*1000:.4f} мс)')
    print(f'Частота опроса системы:                 Fs = {Fs:.2f} Гц')
    print(f'Разброс шагов между датчиками (min/max):{np.min(pos_steps):.0f} / {np.max(pos_steps):.0f} тиков')
    print(f'Средний аппаратный джиттер по {n_sensors} чипам: {np.mean(jitter_std):.4f} тика ({np.mean(jitter_std):.4f} мкс)')
    print(f'Длительность полной записи:             {(n_samples*dt0)/3600:.2f} ч '
          f'({(n_samples*dt0)/60:.1f} мин / {n_samples} кадров)')
    print('===========================================================================\n')

    # ===================== 4. СЕТКА TAU =====================
    max_cluster = int(np.floor(n_samples / 2))          # IEEE 952: отсечение правого хвоста
    n_decades   = np.log10(max_cluster)
    n_tau_points = int(round(n_decades * PTS_PER_DECADE))

    m_steps = np.unique(np.round(np.logspace(0, np.log10(max_cluster), n_tau_points)).astype(np.int64))
    tau_vals = m_steps * dt0
    n_tau = len(tau_vals)
    idx_tau1s = int(np.argmin(np.abs(tau_vals - 1.0)))

    print(f'Сетка tau: {n_tau} точек ({n_tau/n_decades:.1f} т/дек) в диапазоне от '
          f'{tau_vals[0]:.4f} с до {tau_vals[-1]:.1f} с ({tau_vals[-1]/3600:.2f} ч)\n')

    # ###########################################################################
    # ЧАСТЬ I: ГИРОСКОПЫ (КЛАССИЧЕСКИЙ OAVAR, IEEE STD 952)
    # ###########################################################################
    print('===========================================================================')
    print('                ЧАСТЬ I: ГИРОСКОПЫ (КЛАССИЧЕСКИЙ IEEE STD 952)             ')
    print('===========================================================================')

    t_total_gyro = time.time()
    SCALE_DPH = 3600.0   # deg/s -> deg/hr

    allan_dev_gyro_3d   = np.zeros((n_tau, n_sensors, 3))
    allan_dev_gyro_virt  = np.zeros((n_tau, 3))
    gyro_axis_names = ['Гироскоп X (Gx)', 'Гироскоп Y (Gy)', 'Гироскоп Z (Gz)']
    gyro_axis_short = ['Gx', 'Gy', 'Gz']

    N_arw_gyro   = np.full((n_sensors, 3), np.nan)
    B_bi_gyro    = np.full((n_sensors, 3), np.nan)
    tau_BI_gyro  = np.full((n_sensors, 3), np.nan)

    N_virt_gyro   = np.full(3, np.nan)
    B_virt_gyro   = np.full(3, np.nan)
    tau_BI_virt_g = np.full(3, np.nan)

    with h5py.File(H5_FILE_PATH, 'r') as f:
        gyro_dset = f['gyro']

        for a in range(3):
            print(f'Классический расчет оси {gyro_axis_short[a]}...')
            t_axis = time.time()

            gyro_axis = get_dataset_axis(gyro_dset, a)   # [n_samples x n_sensors]

            adev_local = compute_oavar_matrix(gyro_axis, dt0, m_steps, tau_vals, remove_mean=False)
            allan_dev_gyro_3d[:, :, a] = adev_local

            # Синтез виртуального массива (36-IMU Fusion)
            valid_sensors = ~np.isnan(gyro_axis[0, :])
            virtual_rate = np.mean(gyro_axis[:, valid_sensors], axis=1)
            allan_dev_gyro_virt[:, a] = compute_oavar_single(virtual_rate, dt0, m_steps, tau_vals, remove_mean=False)

            # Идентификация ключевых параметров IEEE 952
            for s in range(n_sensors):
                adev = adev_local[:, s]
                if np.all(np.isnan(adev)):
                    continue
                N_arw_gyro[s, a] = adev[idx_tau1s] * 60.0   # deg / sqrt(hr)
                min_idx = int(np.nanargmin(adev))
                min_val = adev[min_idx]
                B_bi_gyro[s, a]  = (min_val / 0.664282) * SCALE_DPH   # deg/hr
                tau_BI_gyro[s, a] = tau_vals[min_idx]

            adev_v = allan_dev_gyro_virt[:, a]
            N_virt_gyro[a] = adev_v[idx_tau1s] * 60.0
            min_v_idx = int(np.nanargmin(adev_v))
            min_v = adev_v[min_v_idx]
            B_virt_gyro[a]  = (min_v / 0.664282) * SCALE_DPH
            tau_BI_virt_g[a] = tau_vals[min_v_idx]

            print(f'  Ось {gyro_axis_short[a]} рассчитана за: {time.time()-t_axis:.2f} с')

    print(f'Все 3 оси гироскопов рассчитаны за: {time.time()-t_total_gyro:.2f} с!')

    # ===================== 5. ВЫВОД ПАСПОРТА ГИРОСКОПОВ =====================
    for a in range(3):
        gain_N = np.nanmean(N_arw_gyro[:, a]) / N_virt_gyro[a]
        gain_B = np.nanmean(B_bi_gyro[:, a])  / B_virt_gyro[a]

        print(f'\n================== ПАСПОРТ IEEE 952: {gyro_axis_names[a]} ==================')
        print('ПАРАМЕТР                              | ОДИНОЧНЫЙ ЧИП (СРЕДНЕЕ) | МАССИВ 36-IMU  | ВЫИГРЫШ')
        print('--------------------------------------+-------------------------+----------------+---------')
        print(f'Угловой случайный уход (ARW, N) [°/√ч]| {np.nanmean(N_arw_gyro[:, a]):17.4f}       | '
              f'{N_virt_gyro[a]:10.4f}     | {gain_N:6.2f} x (теор. 6.0x)')
        print(f'Нестабильность нуля (BI, B) [°/ч]     | {np.nanmean(B_bi_gyro[:, a]):17.2f}       | '
              f'{B_virt_gyro[a]:10.2f}     | {gain_B:6.2f} x (tau = {tau_BI_virt_g[a]:.0f} с)')
        print('--------------------------------------+-------------------------+----------------+---------')

    # ===================== 6. ПОСТРОЕНИЕ 3 ГРАФИКОВ ГИРОСКОПОВ =====================
    os.makedirs('output', exist_ok=True)

    for a in range(3):
        fig, ax = plt.subplots(figsize=(14, 8.5))

        adev_sensors_dph = allan_dev_gyro_3d[:, :, a] * SCALE_DPH
        mean_adev_dph = np.nanmean(adev_sensors_dph, axis=1)
        adev_v_dph = allan_dev_gyro_virt[:, a] * SCALE_DPH
        min_v_idx = int(np.nanargmin(adev_v_dph))
        min_v_dph = adev_v_dph[min_v_idx]

        gain_N = np.nanmean(N_arw_gyro[:, a]) / N_virt_gyro[a]
        gain_B = np.nanmean(B_bi_gyro[:, a])  / B_virt_gyro[a]

        for s in range(n_sensors):
            ax.loglog(tau_vals, adev_sensors_dph[:, s], color=(0.85, 0.85, 0.85), linewidth=0.5)
        ax.loglog(tau_vals, mean_adev_dph, 'k--', linewidth=2.0,
                   label=f'Средний одиночный чип (ARW={np.nanmean(N_arw_gyro[:, a]):.4f}, BI={np.nanmean(B_bi_gyro[:, a]):.2f})')
        ax.loglog(tau_vals, adev_v_dph, color=(0.85, 0.08, 0.08), linewidth=3.0,
                   label=f'Синтезированный массив 36-IMU (ARW={N_virt_gyro[a]:.4f}, BI={B_virt_gyro[a]:.2f})')

        mask_n = (tau_vals >= 0.008) & (tau_vals <= 15)
        tN = tau_vals[mask_n]
        ax.loglog(tN, ((N_virt_gyro[a] / 60.0) / np.sqrt(tN)) * SCALE_DPH, 'b-.', linewidth=1.8,
                   label='Белый шум ARW (наклон -1/2)')

        mask_b = (tau_vals >= tau_BI_virt_g[a]*0.2) & (tau_vals <= tau_BI_virt_g[a]*5)
        tB = tau_vals[mask_b]
        ax.loglog(tB, np.full_like(tB, min_v_dph), 'g--', linewidth=1.8,
                   label='Нестабильность нуля BI (наклон 0)')

        ax.plot(tau_vals[min_v_idx], min_v_dph, marker='*', markersize=13,
                markerfacecolor=(1, 0.85, 0.1), markeredgecolor='k', linewidth=1.3)

        tau1_y_virt = adev_v_dph[idx_tau1s]
        ax.text(1.0, tau1_y_virt * 0.45,
                f'ARW (tau=1 с):\nМассив: {N_virt_gyro[a]:.4f} град/√ч\nЧип: {np.nanmean(N_arw_gyro[:, a]):.4f} град/√ч\n'
                f'Выигрыш: {gain_N:.2f}x (теор. 6.0x)',
                fontsize=9, bbox=dict(facecolor='white', edgecolor=(0, 0.3, 0.8), alpha=0.92))

        ax.text(tau_vals[min_v_idx] * 1.3, min_v_dph * 0.85,
                f'Bias Instability (BI):\nМассив: {B_virt_gyro[a]:.2f} град/ч (tau={tau_BI_virt_g[a]:.0f} с)\n'
                f'Чип: {np.nanmean(B_bi_gyro[:, a]):.2f} град/ч\nВыигрыш: {gain_B:.2f}x',
                fontsize=9, bbox=dict(facecolor='white', edgecolor=(0, 0.6, 0.2), alpha=0.92))

        ax.grid(True, which='both', alpha=0.4)
        ax.set_xlim(tau_vals[0], tau_vals[-1])
        ax.set_ylim(min(min_v_dph*0.35, np.nanmin(mean_adev_dph)*0.35), np.nanmax(mean_adev_dph)*3.0)
        ax.set_xlabel('Время усреднения tau [секунды]', fontsize=12, fontweight='bold')
        ax.set_ylabel('Allan Deviation sigma(tau) [град/ч]', fontsize=12, fontweight='bold')
        ax.set_title(f'Вариация Аллана IEEE Std 952 — {gyro_axis_names[a]} (12-часовая запись, {n_sensors} IMU)',
                     fontsize=13, fontweight='bold')
        ax.legend(loc='lower left', fontsize=9.0)

        table_text = (
            f'ПАСПОРТ ПОГРЕШНОСТЕЙ IEEE 952 ({gyro_axis_short[a]})\n'
            '-----------------------------------------------------------------\n'
            f'1. Угловой случайный уход (ARW, N):\n'
            f'   Массив: {N_virt_gyro[a]:.4f} град/√ч | Чип: {np.nanmean(N_arw_gyro[:, a]):.4f} град/√ч\n'
            f'   Выигрыш синтеза: {gain_N:.2f}x (теор. 6.0x)\n'
            f'2. Нестабильность смещения нуля (BI, B):\n'
            f'   Массив: {B_virt_gyro[a]:.2f} град/ч | Чип: {np.nanmean(B_bi_gyro[:, a]):.2f} град/ч\n'
            f'   Выигрыш синтеза: {gain_B:.2f}x (при tau={tau_BI_virt_g[a]:.0f} с)\n'
            '-----------------------------------------------------------------\n'
            f'Эксперимент: {(n_samples*dt0)/3600:.2f} ч ({n_samples/1e6:.1f} млн отсчетов) @ Fs={Fs:.2f} Гц'
        )
        ax.text(0.61, 0.92, table_text, transform=ax.transAxes, fontsize=8.5,
                va='top', ha='left', bbox=dict(facecolor='white', edgecolor=(0.4, 0.4, 0.4), alpha=0.95))

        fig.tight_layout()
        fig.savefig(f'output/allan_gyro_{gyro_axis_short[a]}.png', dpi=150)
        plt.close(fig)

    # ###########################################################################
    # ЧАСТЬ II: АКСЕЛЕРОМЕТРЫ (КЛАССИЧЕСКИЙ OAVAR, IEEE STD 1293)
    # ###########################################################################
    print('\n===========================================================================')
    print('                ЧАСТЬ II: АКСЕЛЕРОМЕТРЫ (КЛАССИЧЕСКИЙ IEEE STD 1293)       ')
    print('===========================================================================')

    t_total_accel = time.time()
    SCALE_MG = 1000.0
    G_CONST  = 9.80665

    allan_dev_accel_3d  = np.zeros((n_tau, n_sensors, 3))
    allan_dev_accel_virt = np.zeros((n_tau, 3))
    accel_axis_names = ['Акселерометр X (Ax)', 'Акселерометр Y (Ay)', 'Акселерометр Z (Az)']
    accel_axis_short = ['Ax', 'Ay', 'Az']

    VRW_mps_sqrt_h = np.full((n_sensors, 3), np.nan)
    BI_micro_g     = np.full((n_sensors, 3), np.nan)
    tau_BI_accel   = np.full((n_sensors, 3), np.nan)

    VRW_virt_accel = np.full(3, np.nan)
    BI_virt_accel  = np.full(3, np.nan)
    tau_BI_virt_a  = np.full(3, np.nan)

    with h5py.File(H5_FILE_PATH, 'r') as f:
        accel_dset = f['accel']

        for a in range(3):
            print(f'Классический расчет оси {accel_axis_short[a]}...')
            t_axis = time.time()

            accel_axis = get_dataset_axis(accel_dset, a)   # [n_samples x n_sensors], [g]

            adev_local = compute_oavar_matrix(accel_axis, dt0, m_steps, tau_vals, remove_mean=True)
            allan_dev_accel_3d[:, :, a] = adev_local

            # Виртуальный массив акселерометров
            valid_sensors = ~np.isnan(accel_axis[0, :])
            virtual_acc = np.mean(accel_axis[:, valid_sensors], axis=1)
            allan_dev_accel_virt[:, a] = compute_oavar_single(virtual_acc, dt0, m_steps, tau_vals, remove_mean=True)

            # Идентификация ключевых параметров IEEE 1293
            for s in range(n_sensors):
                adev = adev_local[:, s]
                if np.all(np.isnan(adev)):
                    continue
                VRW_mps_sqrt_h[s, a] = adev[idx_tau1s] * G_CONST * 60.0
                min_idx = int(np.nanargmin(adev))
                min_val = adev[min_idx]
                BI_micro_g[s, a] = (min_val / 0.664282) * 1e6
                tau_BI_accel[s, a] = tau_vals[min_idx]

            adev_v = allan_dev_accel_virt[:, a]
            VRW_virt_accel[a] = adev_v[idx_tau1s] * G_CONST * 60.0
            min_v_idx = int(np.nanargmin(adev_v))
            min_v = adev_v[min_v_idx]
            BI_virt_accel[a] = (min_v / 0.664282) * 1e6
            tau_BI_virt_a[a] = tau_vals[min_v_idx]

            print(f'  Ось {accel_axis_short[a]} рассчитана за: {time.time()-t_axis:.2f} с')

    print(f'Все 3 оси акселерометров рассчитаны за: {time.time()-t_total_accel:.2f} с!')

    # ===================== 7. ВЫВОД ПАСПОРТА АКСЕЛЕРОМЕТРОВ =====================
    for a in range(3):
        gain_VRW = np.nanmean(VRW_mps_sqrt_h[:, a]) / VRW_virt_accel[a]
        gain_BIa = np.nanmean(BI_micro_g[:, a])     / BI_virt_accel[a]

        print(f'\n================== ПАСПОРТ IEEE 1293: {accel_axis_names[a]} ==================')
        print('ПАРАМЕТР                              | ОДИНОЧНЫЙ ЧИП (СРЕДНЕЕ) | МАССИВ 36-IMU  | ВЫИГРЫШ')
        print('--------------------------------------+-------------------------+----------------+---------')
        print(f'Случайный уход скорости (VRW) [м/с/√ч]| {np.nanmean(VRW_mps_sqrt_h[:, a]):17.4f}       | '
              f'{VRW_virt_accel[a]:10.4f}     | {gain_VRW:6.2f} x (теор. 6.0x)')
        print(f'Нестабильность нуля (BI, Ba) [мк-g]   | {np.nanmean(BI_micro_g[:, a]):17.2f}       | '
              f'{BI_virt_accel[a]:10.2f}     | {gain_BIa:6.2f} x (tau = {tau_BI_virt_a[a]:.0f} с)')
        print('--------------------------------------+-------------------------+----------------+---------')

    # ===================== 8. ПОСТРОЕНИЕ 3 ГРАФИКОВ АКСЕЛЕРОМЕТРОВ =====================
    for a in range(3):
        fig, ax = plt.subplots(figsize=(14, 8.5))

        adev_sensors_mg = allan_dev_accel_3d[:, :, a] * SCALE_MG
        mean_adev_mg = np.nanmean(adev_sensors_mg, axis=1)
        adev_v_mg = allan_dev_accel_virt[:, a] * SCALE_MG
        min_v_idx = int(np.nanargmin(adev_v_mg))
        min_v_mg = adev_v_mg[min_v_idx]

        gain_VRW = np.nanmean(VRW_mps_sqrt_h[:, a]) / VRW_virt_accel[a]
        gain_BIa = np.nanmean(BI_micro_g[:, a])     / BI_virt_accel[a]

        for s in range(n_sensors):
            ax.loglog(tau_vals, adev_sensors_mg[:, s], color=(0.85, 0.85, 0.85), linewidth=0.5)
        ax.loglog(tau_vals, mean_adev_mg, 'k--', linewidth=2.0,
                   label=f'Средний одиночный чип (VRW={np.nanmean(VRW_mps_sqrt_h[:, a]):.3f}, BI={np.nanmean(BI_micro_g[:, a]):.0f} мк-g)')
        ax.loglog(tau_vals, adev_v_mg, color=(0.08, 0.45, 0.85), linewidth=3.0,
                   label=f'Синтезированный массив 36-IMU (VRW={VRW_virt_accel[a]:.4f}, BI={BI_virt_accel[a]:.1f} мк-g)')

        mask_n = (tau_vals >= 0.008) & (tau_vals <= 15)
        tN = tau_vals[mask_n]
        vrw_virt_g_s = VRW_virt_accel[a] / G_CONST / 60.0
        ax.loglog(tN, (vrw_virt_g_s / np.sqrt(tN)) * SCALE_MG, 'r-.', linewidth=1.8,
                   label='Белый шум VRW (наклон -1/2)')

        mask_b = (tau_vals >= tau_BI_virt_a[a]*0.2) & (tau_vals <= tau_BI_virt_a[a]*5)
        tB = tau_vals[mask_b]
        ax.loglog(tB, np.full_like(tB, min_v_mg), 'g--', linewidth=1.8,
                   label='Нестабильность нуля BI (наклон 0)')

        ax.plot(tau_vals[min_v_idx], min_v_mg, marker='*', markersize=13,
                markerfacecolor=(1, 0.85, 0.1), markeredgecolor='k', linewidth=1.3)

        tau1_y_virt = adev_v_mg[idx_tau1s]
        ax.text(1.0, tau1_y_virt * 0.45,
                f'VRW (tau=1 с):\nМассив: {VRW_virt_accel[a]:.4f} м/с/√ч\nЧип: {np.nanmean(VRW_mps_sqrt_h[:, a]):.4f} м/с/√ч\n'
                f'Выигрыш: {gain_VRW:.2f}x (теор. 6.0x)',
                fontsize=9, bbox=dict(facecolor='white', edgecolor=(0.8, 0, 0), alpha=0.92))

        ax.text(tau_vals[min_v_idx] * 1.3, min_v_mg * 0.85,
                f'Bias Instability (Ba):\nМассив: {BI_virt_accel[a]:.1f} мк-g (tau={tau_BI_virt_a[a]:.0f} с)\n'
                f'Чип: {np.nanmean(BI_micro_g[:, a]):.1f} мк-g\nВыигрыш: {gain_BIa:.2f}x',
                fontsize=9, bbox=dict(facecolor='white', edgecolor=(0, 0.6, 0.2), alpha=0.92))

        ax.grid(True, which='both', alpha=0.4)
        ax.set_xlim(tau_vals[0], tau_vals[-1])
        ax.set_ylim(min(min_v_mg*0.35, np.nanmin(mean_adev_mg)*0.35), np.nanmax(mean_adev_mg)*3.0)
        ax.set_xlabel('Время усреднения tau [секунды]', fontsize=12, fontweight='bold')
        ax.set_ylabel('Allan Deviation sigma(tau) [мг]', fontsize=12, fontweight='bold')
        ax.set_title(f'Вариация Аллана IEEE Std 1293 — {accel_axis_names[a]} (12-часовая запись, {n_sensors} IMU)',
                     fontsize=13, fontweight='bold')
        ax.legend(loc='lower left', fontsize=9.0)

        table_text = (
            f'ПАСПОРТ ПОГРЕШНОСТЕЙ IEEE 1293 ({accel_axis_short[a]})\n'
            '-----------------------------------------------------------------\n'
            f'1. Случайный уход скорости (VRW, Kv):\n'
            f'   Массив: {VRW_virt_accel[a]:.4f} м/с/√ч | Чип: {np.nanmean(VRW_mps_sqrt_h[:, a]):.4f} м/с/√ч\n'
            f'   Выигрыш синтеза: {gain_VRW:.2f}x (теор. 6.0x)\n'
            f'2. Нестабильность смещения нуля (BI, Ba):\n'
            f'   Массив: {BI_virt_accel[a]:.1f} мк-g | Чип: {np.nanmean(BI_micro_g[:, a]):.1f} мк-g\n'
            f'   Выигрыш синтеза: {gain_BIa:.2f}x (при tau={tau_BI_virt_a[a]:.0f} с)\n'
            '-----------------------------------------------------------------\n'
            f'Эксперимент: {(n_samples*dt0)/3600:.2f} ч ({n_samples/1e6:.1f} млн отсчетов) @ Fs={Fs:.2f} Гц'
        )
        ax.text(0.61, 0.92, table_text, transform=ax.transAxes, fontsize=8.5,
                va='top', ha='left', bbox=dict(facecolor='white', edgecolor=(0.4, 0.4, 0.4), alpha=0.95))

        fig.tight_layout()
        fig.savefig(f'output/allan_accel_{accel_axis_short[a]}.png', dpi=150)
        plt.close(fig)

    print('\nВсе 6 графиков вариации Аллана успешно построены и сохранены в output/!')