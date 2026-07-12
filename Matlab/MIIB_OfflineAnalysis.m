% =========================================================================
% MIIB_OfflineAnalysis.m
% Полная офлайн метрологическая оценка сигналов 18×ICM-45686
%
% ВХОД: .mat файл, созданный MIIB_RealTime_Monitor.m
%       (структура 'capture' с полями gyro_raw, accel_raw, temp_raw, ...)
%       ИЛИ указать RAW_FILE = '' для генерации тестовых данных.
%
% МЕТРИКИ:
%   1.  Allan Variance / ADEV (угловое блуждание, смещение bias, скорость
%       нарастания bias, квантовый шум, марковский коррелят)
%   2.  Power Spectral Density (PSD) гироскопа и акселерометра
%   3.  Статическое смещение (bias) и его дрейф
%   4.  Шум: ARW (rad/√s), VRW (m/s/√s), σ_bias
%   5.  Автокорреляция сигнала
%   6.  Распределение: гистограмма + тест Колмогорова-Смирнова на нормальность
%   7.  Термокомпенсация: зависимость bias от температуры (линейная регрессия)
%   8.  Межосевое перекрёстное смешение (cross-axis coupling)
%   9.  Когерентность между датчиками одной шины
%   10. Сводная таблица метрик (exportable в CSV)
% =========================================================================

clc; clear; close all;
fprintf('=== MIIB Offline Analysis v2.0 ===\n\n');

% =========================================================================
%  ПАРАМЕТРЫ
% =========================================================================
RAW_FILE    = '';   % '' = использовать тестовые данные; иначе: 'MIIB_capture_20260712_200000.mat'
ODR         = 3200; % Гц (частота дискретизации)
N_SENSORS   = 18;
N_SAMPLES   = 10;   % Сэмплов ICM-45686 в одном пакете
GYRO_SCALE  = 1.0 / 16.4;   % LSB → deg/s
ACCEL_SCALE = 1.0 / 2048.0; % LSB → g
TEMP_SCALE  = 1.0 / 2.07;
TEMP_OFFSET = 25.0;

% Какие датчики анализировать (1..18), [] = все
SENSORS_TO_ANALYZE = 1:18;

% Папка для сохранения результатов и графиков
OUT_DIR = sprintf('MIIB_Analysis_%s', datestr(now, 'yyyymmdd_HHMMSS'));
mkdir(OUT_DIR);

% =========================================================================
%  ЗАГРУЗКА / ГЕНЕРАЦИЯ ДАННЫХ
% =========================================================================
if isempty(RAW_FILE)
    fprintf('[!] Файл не указан — генерирую синтетические данные для демонстрации...\n');
    % Синтетические данные: 60 секунд × 3200 Гц
    N_PKT    = 60 * (ODR / N_SAMPLES);
    N_TOTAL  = N_PKT * N_SAMPLES;

    % Создаём структуру capture
    rng(42);
    capture.N_SENSORS   = N_SENSORS;
    capture.N_SAMPLES   = N_SAMPLES;
    capture.ODR         = ODR;
    capture.GYRO_SCALE  = GYRO_SCALE;
    capture.ACCEL_SCALE = ACCEL_SCALE;
    % Гироскоп: белый шум + bias + медленный дрейф + температурная зависимость
    bias_deg_s = randn(N_SENSORS, 3) * 0.5;   % случайный bias 0..0.5 dps
    for s = 1:N_SENSORS
        for ax = 1:3
            bias_trend = linspace(0, randn*0.05, N_TOTAL);
            noise      = randn(1, N_TOTAL) * 0.012;  % ARW ~ 0.02 deg/√s
            rw         = cumsum(randn(1, N_TOTAL) * 0.0001);  % bias instability
            sig        = (bias_deg_s(s,ax) + bias_trend + noise + rw) / GYRO_SCALE;
            capture.gyro_raw(1:N_PKT, s, 1:N_SAMPLES, ax) = ...
                int16(reshape(sig, N_SAMPLES, N_PKT)');
        end
    end
    % Акселерометр: 1g по оси Z + шум
    for s = 1:N_SENSORS
        accel_bias = randn(1,3) * 0.002;
        for ax = 1:3
            g_ref = (ax == 3) * 1.0;
            noise = randn(1, N_TOTAL) * 0.0008;
            sig   = (g_ref + accel_bias(ax) + noise) / ACCEL_SCALE;
            capture.accel_raw(1:N_PKT, s, 1:N_SAMPLES, ax) = ...
                int16(reshape(sig, N_SAMPLES, N_PKT)');
        end
    end
    % Температура: линейный дрейф 20→35°C
    temp_trend = linspace(20, 35, N_TOTAL);
    for s = 1:N_SENSORS
        noise = randn(1, N_TOTAL) * 0.3;
        sig   = (temp_trend + noise - TEMP_OFFSET) * TEMP_SCALE;
        capture.temp_raw(1:N_PKT, s, 1:N_SAMPLES) = ...
            int8(reshape(sig, N_SAMPLES, N_PKT)');
    end
    fprintf('   Синтетические данные: %d пакетов, %.0f сек, %d датчиков.\n\n', ...
            N_PKT, N_PKT * N_SAMPLES / ODR, N_SENSORS);
else
    fprintf('Загружаю %s...\n', RAW_FILE);
    load(RAW_FILE, 'capture');
    ODR         = capture.ODR;
    N_SENSORS   = capture.N_SENSORS;
    N_SAMPLES   = capture.N_SAMPLES;
    GYRO_SCALE  = capture.GYRO_SCALE;
    ACCEL_SCALE = capture.ACCEL_SCALE;
    N_PKT = size(capture.gyro_raw, 1);
    fprintf('Загружено: %d пакетов, %.1f сек\n\n', N_PKT, N_PKT*N_SAMPLES/ODR);
end

% =========================================================================
%  РАЗВОРАЧИВАЕМ ДАННЫЕ В ВРЕМЕННЫЕ РЯДЫ
% =========================================================================
N_TOTAL  = N_PKT * N_SAMPLES;
dt       = 1.0 / ODR;
t_axis   = (0:N_TOTAL-1) * dt;  % секунды

% gyro_dps [N_TOTAL × N_SENSORS × 3]
gyro_dps  = zeros(N_TOTAL, N_SENSORS, 3);
accel_g   = zeros(N_TOTAL, N_SENSORS, 3);
temp_degC = zeros(N_TOTAL, N_SENSORS);

for s = 1:N_SENSORS
    for ax = 1:3
        raw_g = squeeze(capture.gyro_raw(:, s, :, ax));  % [N_PKT × N_SAMPLES]
        gyro_dps(:, s, ax) = reshape(raw_g', N_TOTAL, 1) * GYRO_SCALE;

        raw_a = squeeze(capture.accel_raw(:, s, :, ax));
        accel_g(:, s, ax) = reshape(raw_a', N_TOTAL, 1) * ACCEL_SCALE;
    end
    raw_t = squeeze(capture.temp_raw(:, s, :));
    temp_degC(:, s) = reshape(raw_t', N_TOTAL, 1) * TEMP_SCALE + TEMP_OFFSET;
end

fprintf('Данные развёрнуты: [%d × %d × 3] (время × датчики × оси)\n\n', N_TOTAL, N_SENSORS);

% =========================================================================
%  СТРУКТУРА ДЛЯ ХРАНЕНИЯ РЕЗУЛЬТАТОВ
% =========================================================================
results = struct();
AX_NAMES = {'X', 'Y', 'Z'};

% =========================================================================
%  1. ALLAN DEVIATION (ADEV)
% =========================================================================
fprintf('--- [1/8] Вычисление Allan Deviation...\n');

function [tau, adev] = compute_adev(data, dt)
% Вычисление Allan Deviation методом перекрывающихся кластеров.
% data: вектор [N×1] в grad/s
% dt:   период дискретизации
    N = length(data);
    % Интегрирование угла
    theta = cumsum(data) * dt;
    % Диапазон длин кластеров (логарифмическая шкала)
    m_max = floor(N / 4);
    m_vec = unique(round(logspace(0, log10(m_max), 200)));
    m_vec = m_vec(m_vec >= 1 & m_vec <= m_max);
    tau   = m_vec * dt;
    adev  = zeros(size(tau));
    for idx = 1:numel(m_vec)
        m  = m_vec(idx);
        n  = floor(N / m);
        clusters = reshape(theta(1:n*m), m, n);
        cluster_means = mean(clusters, 1);
        diffs = diff(cluster_means);
        adev(idx) = sqrt(sum(diffs.^2) / (2 * (n-1)));
    end
end

% Вычисляем ADEV для каждого датчика и каждой оси гироскопа
for s_idx = 1:numel(SENSORS_TO_ANALYZE)
    s = SENSORS_TO_ANALYZE(s_idx);
    for ax = 1:3
        data = gyro_dps(:, s, ax);
        [tau_v, adev_v] = compute_adev(data, dt);
        results.adev_tau{s, ax}  = tau_v;
        results.adev_val{s, ax}  = adev_v;

        % ARW: наклон -0.5 в log-log пространстве (τ ≈ 1с)
        % Ищем ближайший τ к 1 секунде
        [~, idx1] = min(abs(tau_v - 1.0));
        results.ARW_deg_sqrts(s, ax) = adev_v(idx1) * sqrt(tau_v(idx1));

        % Bias Instability: минимум ADEV
        results.BI_deg_s(s, ax) = min(adev_v);

        % Rate Walk: наклон +0.5 (BI на большом τ)
        [~, imin] = min(adev_v);
        if imin < numel(tau_v)
            [~, idx3] = min(abs(tau_v - tau_v(end)/2));
            results.RW_deg_s_sqrts(s, ax) = adev_v(idx3) / sqrt(tau_v(idx3));
        else
            results.RW_deg_s_sqrts(s, ax) = NaN;
        end
    end
end
fprintf('   ARW гироскопа (датч.1, ось X): %.4f deg/√s\n', results.ARW_deg_sqrts(1,1));
fprintf('   Bias Instability  (датч.1, X) : %.4f deg/s\n\n', results.BI_deg_s(1,1));

% =========================================================================
%  2. PSD (Power Spectral Density)
% =========================================================================
fprintf('--- [2/8] Вычисление PSD...\n');
% Используем Welch метод
NFFT_PSD = min(2^14, 2^nextpow2(N_TOTAL/4));

for s_idx = 1:numel(SENSORS_TO_ANALYZE)
    s = SENSORS_TO_ANALYZE(s_idx);
    for ax = 1:3
        [pxx, f_psd] = pwelch(gyro_dps(:,s,ax), ...
                               hann(NFFT_PSD), NFFT_PSD/2, NFFT_PSD, ODR);
        results.psd_gyro_f{s, ax} = f_psd;
        results.psd_gyro_p{s, ax} = 10*log10(pxx);

        [pxx_a, ~] = pwelch(accel_g(:,s,ax), ...
                             hann(NFFT_PSD), NFFT_PSD/2, NFFT_PSD, ODR);
        results.psd_accel_p{s, ax} = 10*log10(pxx_a);
    end
end
fprintf('   PSD вычислен для %d датчиков × 3 оси.\n\n', numel(SENSORS_TO_ANALYZE));

% =========================================================================
%  3. СТАТИЧЕСКОЕ СМЕЩЕНИЕ (BIAS) И ДРЕЙФ
% =========================================================================
fprintf('--- [3/8] Оценка bias и дрейфа...\n');
for s_idx = 1:numel(SENSORS_TO_ANALYZE)
    s = SENSORS_TO_ANALYZE(s_idx);
    for ax = 1:3
        data = gyro_dps(:, s, ax);
        results.bias_mean(s, ax)  = mean(data);
        results.bias_std(s, ax)   = std(data);
        results.bias_min(s, ax)   = min(data);
        results.bias_max(s, ax)   = max(data);
        results.bias_peak(s, ax)  = max(abs(data));

        % Дрейф: линейная регрессия bias во времени
        p = polyfit(t_axis(:), data(:), 1);
        results.bias_drift_deg_s2(s, ax) = p(1);  % deg/s²

        % Скользящее среднее (1 сек) для оценки нестабильности bias
        win = round(ODR);
        if N_TOTAL > win
            bias_1s = movmean(data, win);
            results.bias_instab(s, ax) = std(bias_1s);
        else
            results.bias_instab(s, ax) = NaN;
        end
    end

    % Акселерометр
    for ax = 1:3
        a_data = accel_g(:, s, ax);
        results.accel_bias_mean(s, ax) = mean(a_data) - (ax == 3)*1.0;  % относительно 1g
        results.accel_bias_std(s, ax)  = std(a_data);
        results.VRW_m_s_sqrth(s, ax)   = std(a_data) / sqrt(ODR) * sqrt(3600) * 9.81;
    end
end
fprintf('   Bias mean гироскопа (датч.1, X): %.4f deg/s\n', results.bias_mean(1,1));
fprintf('   Drift     (датч.1, X): %.2e deg/s²\n\n', results.bias_drift_deg_s2(1,1));

% =========================================================================
%  4. АВТОКОРРЕЛЯЦИЯ
% =========================================================================
fprintf('--- [4/8] Автокорреляция...\n');
LAG_S = 1.0;  % максимальный лаг в секундах
MAX_LAG = round(LAG_S * ODR);

for s_idx = 1:numel(SENSORS_TO_ANALYZE)
    s = SENSORS_TO_ANALYZE(s_idx);
    for ax = 1:3
        data = gyro_dps(:, s, ax) - mean(gyro_dps(:, s, ax));
        [acor, lags] = xcorr(data(1:min(end,10*ODR)), MAX_LAG, 'normalized');
        results.autocorr{s, ax}      = acor;
        results.autocorr_lags{s, ax} = lags / ODR;  % в секундах

        % Корреляционное время (где автокорреляция падает до 1/e)
        half   = round(numel(acor)/2);
        acor_p = acor(half:end);
        idx_e  = find(acor_p <= 1/exp(1), 1, 'first');
        if ~isempty(idx_e)
            results.corr_time_s(s, ax) = idx_e / ODR;
        else
            results.corr_time_s(s, ax) = NaN;
        end
    end
end
fprintf('   Время корреляции гироскопа (датч.1, X): %.3f с\n\n', results.corr_time_s(1,1));

% =========================================================================
%  5. АНАЛИЗ НОРМАЛЬНОСТИ РАСПРЕДЕЛЕНИЯ
% =========================================================================
fprintf('--- [5/8] Тест нормальности (KS-test)...\n');
for s_idx = 1:numel(SENSORS_TO_ANALYZE)
    s = SENSORS_TO_ANALYZE(s_idx);
    for ax = 1:3
        data = gyro_dps(:, s, ax);
        [h, p_val, ksstat] = kstest((data - mean(data)) / std(data));
        results.ks_h(s, ax)      = h;       % 1 = не нормальное
        results.ks_p(s, ax)      = p_val;   % p-value
        results.ks_stat(s, ax)   = ksstat;
        % Эксцесс и асимметрия
        results.skewness_g(s, ax) = skewness(data);
        results.kurtosis_g(s, ax) = kurtosis(data);
    end
end
fprintf('   Гироскоп датч.1 X: p=%.4f, эксцесс=%.2f\n\n', ...
    results.ks_p(1,1), results.kurtosis_g(1,1));

% =========================================================================
%  6. ТЕМПЕРАТУРНАЯ КОМПЕНСАЦИЯ
% =========================================================================
fprintf('--- [6/8] Термокомпенсация...\n');
for s_idx = 1:numel(SENSORS_TO_ANALYZE)
    s = SENSORS_TO_ANALYZE(s_idx);
    T = temp_degC(:, s);
    for ax = 1:3
        data = gyro_dps(:, s, ax);
        p = polyfit(T, data, 1);
        results.thermo_slope(s, ax)    = p(1);  % deg/s/°C
        results.thermo_intercept(s,ax) = p(2);
        % Коэффициент корреляции Пирсона
        cc = corrcoef(T, data);
        results.thermo_R2(s, ax) = cc(1,2)^2;
    end
end
fprintf('   Термо-коэф. гироскопа датч.1 X: %.5f deg/s/°C\n\n', ...
    results.thermo_slope(1,1));

% =========================================================================
%  7. МЕЖОСЕВОЕ ПЕРЕКРЁСТНОЕ СМЕШЕНИЕ (Cross-Axis Coupling)
% =========================================================================
fprintf('--- [7/8] Cross-axis coupling...\n');
for s_idx = 1:numel(SENSORS_TO_ANALYZE)
    s = SENSORS_TO_ANALYZE(s_idx);
    for ax1 = 1:3
        for ax2 = 1:3
            if ax1 ~= ax2
                cc = corrcoef(gyro_dps(:,s,ax1), gyro_dps(:,s,ax2));
                results.cross_corr(s, ax1, ax2) = abs(cc(1,2));
            else
                results.cross_corr(s, ax1, ax2) = 1.0;
            end
        end
    end
end
fprintf('   Cross-corr датч.1 X-Y: %.4f\n\n', results.cross_corr(1,1,2));

% =========================================================================
%  8. КОГЕРЕНТНОСТЬ МЕЖДУ ДАТЧИКАМИ ОДНОЙ ШИНЫ
% =========================================================================
fprintf('--- [8/8] Когерентность между датчиками...\n');
% Сравниваем датчик 1 с датчиками 2..6 (SPI4)
ref_sensor = 1;
NFFT_COH   = min(2^12, 2^nextpow2(N_TOTAL/8));
for s = 2:6
    for ax = 1:3
        [coh, f_coh] = mscohere(gyro_dps(:,ref_sensor,ax), ...
                                gyro_dps(:,s,ax), ...
                                hann(NFFT_COH), NFFT_COH/2, NFFT_COH, ODR);
        results.coherence{s, ax} = coh;
        results.coherence_f{s, ax} = f_coh;
        % Средняя когерентность в диапазоне 0.1..50 Гц
        mask = f_coh >= 0.1 & f_coh <= 50;
        results.mean_coherence(s, ax) = mean(coh(mask));
    end
end
fprintf('   Когерентность датч.1↔датч.2 (X, 0.1-50Гц): %.3f\n\n', ...
    results.mean_coherence(2,1));

% =========================================================================
%  СВОДНАЯ ТАБЛИЦА МЕТРИК
% =========================================================================
fprintf('Формирование сводной таблицы...\n');
sensor_ids = SENSORS_TO_ANALYZE;
sensor_names = arrayfun(@(x) sprintf('Sensor_%02d', x), sensor_ids, 'UniformOutput', false);

% Создаём таблицу (только ось X для краткости — в CSV все оси)
table_data = table( ...
    sensor_ids(:), ...
    results.bias_mean(sensor_ids, 1), ...
    results.bias_std(sensor_ids, 1), ...
    results.ARW_deg_sqrts(sensor_ids, 1), ...
    results.BI_deg_s(sensor_ids, 1), ...
    results.bias_drift_deg_s2(sensor_ids, 1), ...
    results.thermo_slope(sensor_ids, 1), ...
    results.ks_p(sensor_ids, 1), ...
    'VariableNames', {'SensorID', 'Bias_mean_dps', 'Bias_std_dps', ...
                      'ARW_deg_sqrts', 'BiasInstab_dps', ...
                      'Drift_dps2', 'ThermoCoef_dps_C', 'KS_pvalue'});

disp(table_data);

% Сохранение в CSV
csv_file = fullfile(OUT_DIR, 'metrics_summary.csv');
writetable(table_data, csv_file);
fprintf('Сводная таблица сохранена: %s\n\n', csv_file);

% =========================================================================
%  ПОСТРОЕНИЕ ГРАФИКОВ
% =========================================================================
fprintf('Построение графиков...\n');

COLORS_18 = turbo(18);
AXES_COLORS = {[0.2 0.6 1.0], [0.2 1.0 0.5], [1.0 0.5 0.2]};

% ─── ГРАФИК 1: Allan Deviation для всех датчиков (ось X гироскопа) ─────
fig1 = figure('Name', 'Allan Deviation — Гироскоп X', ...
              'Color', [0.08 0.08 0.12], ...
              'Position', [50 50 1400 700]);
ax_ad = subplot(1,1,1);
set(ax_ad, 'Color', [0.08 0.08 0.12], 'XColor', [0.8 0.8 0.8], ...
    'YColor', [0.8 0.8 0.8], 'GridColor', [0.3 0.3 0.3], 'XScale','log', 'YScale','log');
grid(ax_ad, 'on'); hold(ax_ad, 'on');

for s_idx = 1:numel(SENSORS_TO_ANALYZE)
    s = SENSORS_TO_ANALYZE(s_idx);
    loglog(ax_ad, results.adev_tau{s,1}, results.adev_val{s,1}, ...
        'Color', COLORS_18(s,:), 'LineWidth', 1.5, ...
        'DisplayName', sprintf('S%02d', s));
end
% Наклонные линии-ориентиры
tau_ref = [0.01 1000];
loglog(ax_ad, tau_ref, 0.05*tau_ref.^(-0.5), 'w--', 'LineWidth', 0.8, ...
    'DisplayName', 'ARW slope (-0.5)');
loglog(ax_ad, tau_ref, 0.002*ones(size(tau_ref)), 'y--', 'LineWidth', 0.8, ...
    'DisplayName', 'BI (flat)');
loglog(ax_ad, tau_ref, 0.0001*tau_ref.^0.5, 'm--', 'LineWidth', 0.8, ...
    'DisplayName', 'RRW slope (+0.5)');
xlabel(ax_ad, 'Время усреднения τ [с]', 'Color', [0.9 0.9 0.9]);
ylabel(ax_ad, 'ADEV [deg/s]', 'Color', [0.9 0.9 0.9]);
title(ax_ad, 'Allan Deviation — Гироскоп ось X (18 датчиков)', ...
    'Color', [0.9 0.9 0.9], 'FontSize', 13, 'FontWeight', 'bold');
lg = legend(ax_ad, 'show', 'Location', 'southwest', 'NumColumns', 3);
lg.TextColor = [0.8 0.8 0.8]; lg.Color = [0.1 0.1 0.15];
saveas(fig1, fullfile(OUT_DIR, 'fig1_AllanDeviation.png'));

% ─── ГРАФИК 2: PSD Гироскопа — все датчики, ось X ───────────────────────
fig2 = figure('Name', 'PSD Гироскоп', ...
              'Color', [0.08 0.08 0.12], 'Position', [100 50 1400 700]);
ax_psd = subplot(1,1,1);
set(ax_psd, 'Color', [0.08 0.08 0.12], 'XColor', [0.8 0.8 0.8], ...
    'YColor', [0.8 0.8 0.8], 'GridColor', [0.3 0.3 0.3], 'XScale','log');
grid(ax_psd, 'on'); hold(ax_psd, 'on');

for s_idx = 1:numel(SENSORS_TO_ANALYZE)
    s = SENSORS_TO_ANALYZE(s_idx);
    plot(ax_psd, results.psd_gyro_f{s,1}, results.psd_gyro_p{s,1}, ...
        'Color', [COLORS_18(s,:) 0.7], 'LineWidth', 0.9, ...
        'DisplayName', sprintf('S%02d', s));
end
xlabel(ax_psd, 'Частота [Гц]', 'Color', [0.9 0.9 0.9]);
ylabel(ax_psd, 'PSD [(deg/s)²/Гц] [dB]', 'Color', [0.9 0.9 0.9]);
title(ax_psd, 'PSD Гироскопа — ось X (18 датчиков)', ...
    'Color', [0.9 0.9 0.9], 'FontSize', 13, 'FontWeight', 'bold');
lg2 = legend(ax_psd, 'show', 'Location', 'southwest', 'NumColumns', 3);
lg2.TextColor = [0.8 0.8 0.8]; lg2.Color = [0.1 0.1 0.15];
saveas(fig2, fullfile(OUT_DIR, 'fig2_PSD_Gyro.png'));

% ─── ГРАФИК 3: Временной ряд bias — первые 10 сек ────────────────────────
fig3 = figure('Name', 'Bias — Временной ряд', ...
              'Color', [0.08 0.08 0.12], 'Position', [150 50 1600 900]);
tl3 = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl3, 'Гироскоп: временной ряд bias (18 датчиков, 10 сек)', ...
    'Color', [0.9 0.9 0.9], 'FontSize', 13, 'FontWeight', 'bold');

t_short = t_axis <= 10.0;
for ax = 1:3
    axh = nexttile(tl3);
    set(axh, 'Color', [0.08 0.08 0.12], 'XColor', [0.8 0.8 0.8], ...
        'YColor', [0.8 0.8 0.8], 'GridColor', [0.3 0.3 0.3]);
    grid(axh, 'on'); hold(axh, 'on');
    for s_idx = 1:numel(SENSORS_TO_ANALYZE)
        s = SENSORS_TO_ANALYZE(s_idx);
        plot(axh, t_axis(t_short), gyro_dps(t_short, s, ax), ...
            'Color', [COLORS_18(s,:) 0.6], 'LineWidth', 0.8);
    end
    ylabel(axh, sprintf('Ось %s [deg/s]', AX_NAMES{ax}), 'Color', [0.8 0.8 0.8]);
    xlabel(axh, 'Время [с]', 'Color', [0.8 0.8 0.8]);
end
saveas(fig3, fullfile(OUT_DIR, 'fig3_Gyro_TimeSeries.png'));

% ─── ГРАФИК 4: Гистограммы распределения шума (датч. 1-6, ось X) ─────────
fig4 = figure('Name', 'Гистограммы шума', ...
              'Color', [0.08 0.08 0.12], 'Position', [200 50 1600 700]);
tl4 = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl4, 'Распределение шума гироскопа — Ось X (датчики 1-6)', ...
    'Color', [0.9 0.9 0.9], 'FontSize', 13);
for i = 1:6
    s = i;
    axh = nexttile(tl4);
    set(axh, 'Color', [0.08 0.08 0.12], 'XColor', [0.8 0.8 0.8], ...
        'YColor', [0.8 0.8 0.8]);
    data = gyro_dps(:, s, 1);
    histogram(axh, data, 80, ...
        'FaceColor', COLORS_18(s,:), 'EdgeColor', 'none', 'FaceAlpha', 0.75);
    hold(axh, 'on');
    % Наложение нормального распределения
    mu_v = mean(data); sg_v = std(data);
    x_g  = linspace(mu_v-4*sg_v, mu_v+4*sg_v, 200);
    n_count = numel(data);
    bin_w   = (max(data)-min(data))/80;
    plot(axh, x_g, n_count*bin_w * normpdf(x_g, mu_v, sg_v), ...
        'w-', 'LineWidth', 2);
    title(axh, sprintf('Датч.%d  μ=%.3f  σ=%.4f  Kurt=%.2f', ...
        s, mu_v, sg_v, kurtosis(data)), ...
        'Color', COLORS_18(s,:), 'FontSize', 8);
    xlabel(axh, '[deg/s]', 'Color', [0.7 0.7 0.7], 'FontSize', 8);
    grid(axh, 'on');
end
saveas(fig4, fullfile(OUT_DIR, 'fig4_Noise_Histograms.png'));

% ─── ГРАФИК 5: Термокомпенсация — bias vs temperature ────────────────────
fig5 = figure('Name', 'Температурная компенсация', ...
              'Color', [0.08 0.08 0.12], 'Position', [250 50 1600 700]);
tl5 = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl5, 'Bias гироскопа vs Температура — Ось X (датчики 1-6)', ...
    'Color', [0.9 0.9 0.9], 'FontSize', 13);
for i = 1:6
    s = i;
    axh = nexttile(tl5);
    set(axh, 'Color', [0.08 0.08 0.12], 'XColor', [0.8 0.8 0.8], ...
        'YColor', [0.8 0.8 0.8]);
    T_data   = temp_degC(:, s);
    g_data   = gyro_dps(:, s, 1);
    % Прореживаем для скорости отрисовки
    step = max(1, floor(numel(T_data)/2000));
    scatter(axh, T_data(1:step:end), g_data(1:step:end), 2, ...
        COLORS_18(s,:), 'filled', 'MarkerFaceAlpha', 0.3);
    hold(axh, 'on');
    T_fit = linspace(min(T_data), max(T_data), 100);
    g_fit = results.thermo_slope(s,1)*T_fit + results.thermo_intercept(s,1);
    plot(axh, T_fit, g_fit, 'w-', 'LineWidth', 2);
    title(axh, sprintf('Датч.%d  k=%.5f °/s/°C  R²=%.3f', ...
        s, results.thermo_slope(s,1), results.thermo_R2(s,1)), ...
        'Color', COLORS_18(s,:), 'FontSize', 8);
    xlabel(axh, 'Температура [°C]', 'Color', [0.7 0.7 0.7], 'FontSize', 8);
    ylabel(axh, '[deg/s]', 'Color', [0.7 0.7 0.7], 'FontSize', 8);
    grid(axh, 'on');
end
saveas(fig5, fullfile(OUT_DIR, 'fig5_ThermalCompensation.png'));

% ─── ГРАФИК 6: Сводная метрика ARW / BI — heatmap по всем датчикам ────────
fig6 = figure('Name', 'Метрики — Heatmap', ...
              'Color', [0.08 0.08 0.12], 'Position', [300 50 1400 600]);
tl6 = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl6, 'Сводная карта метрик: ARW и Bias Instability', ...
    'Color', [0.9 0.9 0.9], 'FontSize', 13);

% ARW
axh = nexttile(tl6);
ARW_mat = results.ARW_deg_sqrts(SENSORS_TO_ANALYZE, :);
imagesc(axh, ARW_mat);
colormap(axh, turbo);
colorbar(axh);
set(axh, 'XTick', 1:3, 'XTickLabel', {'X','Y','Z'}, ...
    'YTick', 1:numel(SENSORS_TO_ANALYZE), ...
    'YTickLabel', arrayfun(@(x) sprintf('S%02d',x), SENSORS_TO_ANALYZE, 'UniformOutput', false), ...
    'XColor', [0.8 0.8 0.8], 'YColor', [0.8 0.8 0.8], 'Color', [0.08 0.08 0.12], ...
    'FontSize', 8);
title(axh, 'ARW [deg/√s]', 'Color', [0.9 0.9 0.9], 'FontSize', 11);
xlabel(axh, 'Ось', 'Color', [0.8 0.8 0.8]);

% Bias Instability
axh2 = nexttile(tl6);
BI_mat = results.BI_deg_s(SENSORS_TO_ANALYZE, :);
imagesc(axh2, BI_mat);
colormap(axh2, parula);
colorbar(axh2);
set(axh2, 'XTick', 1:3, 'XTickLabel', {'X','Y','Z'}, ...
    'YTick', 1:numel(SENSORS_TO_ANALYZE), ...
    'YTickLabel', arrayfun(@(x) sprintf('S%02d',x), SENSORS_TO_ANALYZE, 'UniformOutput', false), ...
    'XColor', [0.8 0.8 0.8], 'YColor', [0.8 0.8 0.8], 'Color', [0.08 0.08 0.12], ...
    'FontSize', 8);
title(axh2, 'Bias Instability [deg/s]', 'Color', [0.9 0.9 0.9], 'FontSize', 11);
xlabel(axh2, 'Ось', 'Color', [0.8 0.8 0.8]);
saveas(fig6, fullfile(OUT_DIR, 'fig6_Metrics_Heatmap.png'));

% ─── ГРАФИК 7: Когерентность датч.1↔датч.2..6 (SPI4, ось X) ─────────────
fig7 = figure('Name', 'Когерентность', ...
              'Color', [0.08 0.08 0.12], 'Position', [350 50 1400 700]);
axh = axes(fig7);
set(axh, 'Color', [0.08 0.08 0.12], 'XColor', [0.8 0.8 0.8], ...
    'YColor', [0.8 0.8 0.8], 'GridColor', [0.3 0.3 0.3], 'XScale','log');
grid(axh, 'on'); hold(axh, 'on');
for s = 2:6
    if isfield(results, 'coherence') && ~isempty(results.coherence{s,1})
        plot(axh, results.coherence_f{s,1}, results.coherence{s,1}, ...
            'Color', COLORS_18(s,:), 'LineWidth', 1.5, ...
            'DisplayName', sprintf('S01↔S%02d (%.3f)', s, results.mean_coherence(s,1)));
    end
end
xlabel(axh, 'Частота [Гц]', 'Color', [0.9 0.9 0.9]);
ylabel(axh, 'Когерентность', 'Color', [0.9 0.9 0.9]);
ylim(axh, [0 1]);
title(axh, 'Когерентность Датч.1↔Датч.2-6 — Гироскоп X (SPI4)', ...
    'Color', [0.9 0.9 0.9], 'FontSize', 13, 'FontWeight', 'bold');
lg7 = legend(axh, 'show', 'Location', 'southwest');
lg7.TextColor = [0.8 0.8 0.8]; lg7.Color = [0.1 0.1 0.15];
saveas(fig7, fullfile(OUT_DIR, 'fig7_Coherence.png'));

% ─── ГРАФИК 8: Cross-axis matrix (датч. 1, гироскоп) ─────────────────────
fig8 = figure('Name', 'Cross-Axis Coupling', ...
              'Color', [0.08 0.08 0.12], 'Position', [400 50 500 500]);
axh = axes(fig8);
ca_mat = squeeze(results.cross_corr(1,:,:));
imagesc(axh, ca_mat);
colormap(axh, hot);
cb = colorbar(axh); cb.Color = [0.8 0.8 0.8];
clim(axh, [0 1]);
set(axh, 'XTick', 1:3, 'XTickLabel', {'X','Y','Z'}, ...
    'YTick', 1:3, 'YTickLabel', {'X','Y','Z'}, ...
    'XColor', [0.8 0.8 0.8], 'YColor', [0.8 0.8 0.8], 'Color', [0.08 0.08 0.12], ...
    'FontSize', 11);
title(axh, 'Cross-Axis Coupling (датч.1)', 'Color', [0.9 0.9 0.9], 'FontSize', 12);
for r = 1:3
    for c = 1:3
        text(axh, c, r, sprintf('%.3f', ca_mat(r,c)), ...
            'HorizontalAlignment', 'center', 'Color', 'w', 'FontWeight', 'bold', 'FontSize', 11);
    end
end
saveas(fig8, fullfile(OUT_DIR, 'fig8_CrossAxis.png'));

% ─── ФИНАЛЬНЫЙ ОТЧЁТ ─────────────────────────────────────────────────────
fprintf('\n========== ИТОГОВЫЙ ОТЧЁТ ==========\n');
fprintf('Длительность записи   : %.1f сек\n', N_TOTAL/ODR);
fprintf('Частота дискретизации : %d Гц\n', ODR);
fprintf('Число датчиков        : %d\n\n', N_SENSORS);
fprintf('Метрики (датч.1, ось X):\n');
fprintf('  ARW гироскопа       : %.4f  deg/√s\n', results.ARW_deg_sqrts(1,1));
fprintf('  Bias Instability    : %.5f  deg/s\n',  results.BI_deg_s(1,1));
fprintf('  Bias Mean           : %+.4f  deg/s\n', results.bias_mean(1,1));
fprintf('  Bias Std            : %.5f  deg/s\n',  results.bias_std(1,1));
fprintf('  Drift               : %.3e deg/s²\n', results.bias_drift_deg_s2(1,1));
fprintf('  Thermo coefficient  : %.5f  deg/s/°C\n', results.thermo_slope(1,1));
fprintf('  Corr. time          : %.3f  с\n',  results.corr_time_s(1,1));
fprintf('  KS test p-value     : %.4f %s\n', results.ks_p(1,1), ...
    ternary(results.ks_h(1,1)==0, '(нормальное)', '(НЕ нормальное)'));
fprintf('\n  VRW акселерометра X : %.4f  m/s/√h\n', results.VRW_m_s_sqrth(1,1));
fprintf('\n');
fprintf('Графики сохранены в: %s/\n', OUT_DIR);
fprintf('CSV метрик:           %s\n', csv_file);
fprintf('=====================================\n');

% Сохраняем полный workspace результатов
save(fullfile(OUT_DIR, 'full_results.mat'), 'results', 'capture', '-v7.3');

% =========================================================================
%  ЛОКАЛЬНЫЕ ФУНКЦИИ
% =========================================================================

function s = ternary(cond, a, b)
    if cond; s = a; else; s = b; end
end

