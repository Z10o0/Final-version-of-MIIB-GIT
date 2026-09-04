%% =========================================================================
%% КЛАССИЧЕСКИЙ РАСЧЕТ ВАРИАЦИИ АЛЛАНА (IEEE STD 952 / IEEE STD 1293)
%% =========================================================================
clear; clc; close all;

%% ===================== 1. НАСТРОЙКИ ЭКСПЕРИМЕНТА =====================
h5FilePath   = 'miib_data_single_12h.h5'; % Имя HDF5-файла
ptsPerDecade = 45;                        % Академическая плотность сетки tau

if ~exist(h5FilePath, 'file')
    error('Файл %s не найден! Проверьте путь к файлу.', h5FilePath);
end

%% ===================== 2. ИНИЦИАЛИЗАЦИЯ ПАРАЛЛЕЛЬНОГО ПУЛА =====================
pool = gcp('nocreate');
if isempty(pool)
    fprintf('Запуск параллельного пула MATLAB...\n');
    pool = parpool('local');
else
    fprintf('Используется активный пул: %d воркеров (ядер).\n', pool.NumWorkers);
end

%% ===================== 3. КРОСС-АНАЛИЗ СИНХРОНИЗАЦИИ ТАЙМШТАМПОВ =====================
fprintf('\n================== 1. КРОСС-АНАЛИЗ СИНХРОНИЗАЦИИ МАССИВА ==================\n');

infoGyro = h5info(h5FilePath, '/gyro');
nSamples = infoGyro.Dataspace.Size(3); % Все 69 120 000 кадров
nSensors = infoGyro.Dataspace.Size(2); % 36 датчиков

% Читаем таймштампы всех 36 датчиков
nCheck = min(50000, nSamples);
allTs  = h5read(h5FilePath, '/tstamp', [1, 1], [nSensors, nCheck]);

nominalSteps = zeros(nSensors, 1);
jitterStd    = zeros(nSensors, 1);

for s = 1:nSensors
    ts = double(squeeze(allTs(s, :)));
    dTs_s = mod(diff(ts), 65536);
    dTs_s = dTs_s(dTs_s > 0);
    if ~isempty(dTs_s)
        nominalSteps(s) = mode(dTs_s);
        jitterStd(s)    = std(dTs_s(dTs_s < nominalSteps(s)*1.5));
    end
end

meanTicks = mode(nominalSteps(nominalSteps > 0));
Fs  = 1e6 / meanTicks; % ~1642...1647 Гц
dt0 = 1.0 / Fs;

fprintf('Номинальный шаг таймера:                 %d тиков (%.4f мс)\n', meanTicks, dt0*1000);
fprintf('Частота опроса системы:                 Fs = %.2f Гц\n', Fs);
fprintf('Разброс шагов между датчиками (min/max):%d / %d тиков\n', min(nominalSteps), max(nominalSteps));
fprintf('Средний аппаратный джиттер по 36 чипам: %.4f тика (%.4f мкс)\n', mean(jitterStd), mean(jitterStd));
fprintf('Длительность полной записи:             %.2f ч (%.1f мин / %d кадров)\n', ...
    (nSamples*dt0)/3600, (nSamples*dt0)/60, nSamples);
fprintf('===========================================================================\n\n');

%% ===================== 4. КЛАССИЧЕСКАЯ НЕПРЕРЫВНАЯ СЕТКА TAU =====================
maxCluster = floor(nSamples / 9); % IEEE 952: отсечение недостоверного правого хвоста
nDecades   = log10(maxCluster);
nTauPoints = round(nDecades * ptsPerDecade);

mSteps  = unique(round(logspace(0, log10(maxCluster), nTauPoints)));
tauVals = mSteps * dt0;
nTau    = numel(tauVals);
[~, idxTau1s] = min(abs(tauVals - 1.0));

fprintf('Сетка tau: %d точек (%.1f т/дек) в диапазоне от %.4f с до %.1f с (%.2f ч)\n\n', ...
    nTau, nTau / nDecades, tauVals(1), tauVals(end), tauVals(end)/3600);

%% #########################################################################
%% ЧАСТЬ I: ГИРОСКОПЫ (КЛАССИЧЕСКИЙ OAVAR, IEEE STD 952)
%% #########################################################################
fprintf('===========================================================================\n');
fprintf('                ЧАСТЬ I: ГИРОСКОПЫ (КЛАССИЧЕСКИЙ IEEE STD 952)             \n');
fprintf('===========================================================================\n');
tTotalGyro = tic;

SCALE_DPH = 3600.0; % deg/s -> deg/hr

allanDevGyro3D   = zeros(nTau, nSensors, 3);
allanDevGyroVirt = zeros(nTau, 3);
gyroAxisNames    = {'Гироскоп X (Gx)', 'Гироскоп Y (Gy)', 'Гироскоп Z (Gz)'};
gyroAxisShort    = {'Gx', 'Gy', 'Gz'};

N_arw_gyro  = NaN(nSensors, 3);
B_bi_gyro   = NaN(nSensors, 3);
K_rrw_gyro  = NaN(nSensors, 3);
tau_BI_gyro = NaN(nSensors, 3);

N_virt_gyro   = NaN(3, 1);
B_virt_gyro   = NaN(3, 1);
K_virt_gyro   = NaN(3, 1);
tau_BI_virt_g = NaN(3, 1);

for a = 1:3
    fprintf('Классический расчет оси %s...\n', gyroAxisShort{a});
    tAxis = tic;
    rawAxis = h5read(h5FilePath, '/gyro', [a, 1, 1], [1, nSensors, nSamples]);
    gyroAxis = squeeze(rawAxis)'; % [nSamples x 36], single
    
    adevLocal = zeros(nTau, nSensors);
    parfor s = 1:nSensors
        rate = double(gyroAxis(:, s));
        if ~all(isnan(rate))
            adevLocal(:, s) = classical_oavar_core(rate, dt0, mSteps, tauVals);
        else
            adevLocal(:, s) = NaN(nTau, 1);
        end
    end
    allanDevGyro3D(:, :, a) = adevLocal;
    
    % Виртуальный массив (36-IMU Fusion)
    validSensors = ~isnan(gyroAxis(1, :));
    virtualRate  = double(mean(gyroAxis(:, validSensors), 2));
    allanDevGyroVirt(:, a) = classical_oavar_core(virtualRate, dt0, mSteps, tauVals);
    
    % Идентификация параметров погрешностей
    for s = 1:nSensors
        adev = adevLocal(:, s);
        if all(isnan(adev)), continue; end
        N_arw_gyro(s, a) = adev(idxTau1s) * 60.0; % deg / sqrt(hr)
        [minVal, minIdx] = min(adev);
        B_bi_gyro(s, a)   = (minVal / 0.664282) * SCALE_DPH; % deg/hr
        tau_BI_gyro(s, a) = tauVals(minIdx);
        
        idxK = find(tauVals >= 150 & tauVals <= tauVals(end));
        if numel(idxK) >= 3
            pK = polyfit(log10(tauVals(idxK)), log10(adev(idxK)), 1);
            K_rrw_gyro(s, a) = 10^(pK(2)) * sqrt(3) * 60.0 * SCALE_DPH;
        else
            K_rrw_gyro(s, a) = adev(end) * sqrt(3 / tauVals(end)) * 60.0 * SCALE_DPH;
        end
    end
    
    adevV = allanDevGyroVirt(:, a);
    N_virt_gyro(a) = adevV(idxTau1s) * 60.0;
    [minV, minVIdx] = min(adevV);
    B_virt_gyro(a)   = (minV / 0.664282) * SCALE_DPH;
    tau_BI_virt_g(a) = tauVals(minVIdx);
    
    idxKv = find(tauVals >= max(200, tau_BI_virt_g(a)*5) & tauVals <= tauVals(end));
    if numel(idxKv) >= 3
        pKv = polyfit(log10(tauVals(idxKv)), log10(adevV(idxKv)), 1);
        K_virt_gyro(a) = 10^(pKv(2)) * sqrt(3) * 60.0 * SCALE_DPH;
    else
        K_virt_gyro(a) = adevV(end) * sqrt(3 / tauVals(end)) * 60.0 * SCALE_DPH;
    end
    
    clear gyroAxis rawAxis;
    fprintf('  Ось %s рассчитана за: %.2f с\n', gyroAxisShort{a}, toc(tAxis));
end
fprintf('Все 3 оси гироскопов рассчитаны за: %.2f с!\n', toc(tTotalGyro));

%% ===================== 5. ВЫВОД ПАСПОРТА ГИРОСКОПОВ =====================
for a = 1:3
    gain_N = mean(N_arw_gyro(:, a), 'omitnan') / N_virt_gyro(a);
    gain_B = mean(B_bi_gyro(:, a), 'omitnan') / B_virt_gyro(a);
    gain_K = mean(K_rrw_gyro(:, a), 'omitnan') / K_virt_gyro(a);
    
    fprintf('\n================== ПАСПОРТ IEEE 952: %s ==================\n', gyroAxisNames{a});
    fprintf('ПАРАМЕТР                              | ОДИНОЧНЫЙ ЧИП (СРЕДНЕЕ) | МАССИВ 36-IMU  | ВЫИГРЫШ\n');
    fprintf('--------------------------------------+-------------------------+----------------+---------\n');
    fprintf('Угловой случайный уход (ARW, N) [°/√ч]| %17.4f       | %10.4f     | %6.2f x (теор. 6.0x)\n', ...
        mean(N_arw_gyro(:, a), 'omitnan'), N_virt_gyro(a), gain_N);
    fprintf('Нестабильность нуля (BI, B) [°/ч]     | %17.2f       | %10.2f     | %6.2f x (tau = %.0f с)\n', ...
        mean(B_bi_gyro(:, a), 'omitnan'), B_virt_gyro(a), gain_B, tau_BI_virt_g(a));
    fprintf('Случайный уход скорости (RRW, K)      | %17.2f       | %10.2f     | %6.2f x [°/ч^1.5]\n', ...
        mean(K_rrw_gyro(:, a), 'omitnan'), K_virt_gyro(a), gain_K);
    fprintf('--------------------------------------+-------------------------+----------------+---------\n');
end

%% ===================== 6. ПОСТРОЕНИЕ 3 ГРАФИКОВ ГИРОСКОПОВ =====================
for a = 1:3
    figG = figure('Name', sprintf('MIIB: Вариация Аллана IEEE 952 — %s', gyroAxisNames{a}), ...
        'Color', 'w', 'Position', [50 + (a-1)*40, 40 + (a-1)*30, 1400, 850]);
    
    adevSensors_dph = squeeze(allanDevGyro3D(:, :, a)) * SCALE_DPH;
    meanAdev_dph    = mean(adevSensors_dph, 2, 'omitnan');
    adevV_dph       = allanDevGyroVirt(:, a) * SCALE_DPH;
    [minV_dph, minVIdx] = min(adevV_dph);
    
    gain_N = mean(N_arw_gyro(:, a), 'omitnan') / N_virt_gyro(a);
    gain_B = mean(B_bi_gyro(:, a), 'omitnan') / B_virt_gyro(a);
    gain_K = mean(K_rrw_gyro(:, a), 'omitnan') / K_virt_gyro(a);

    hIndiv = loglog(tauVals, adevSensors_dph, 'Color', [0.85 0.85 0.85], 'LineWidth', 0.5); hold on;
    hMean  = loglog(tauVals, meanAdev_dph, 'k--', 'LineWidth', 2.0);
    hVirt  = loglog(tauVals, adevV_dph, 'Color', [0.85 0.08 0.08], 'LineWidth', 3.0);
    
    tN = tauVals(tauVals >= 0.008 & tauVals <= 15);
    hN_line = loglog(tN, ((N_virt_gyro(a) / 60.0) ./ sqrt(tN)) * SCALE_DPH, 'b-.', 'LineWidth', 1.8);
    
    tB = tauVals(tauVals >= tau_BI_virt_g(a)*0.2 & tauVals <= tau_BI_virt_g(a)*5);
    hB_line = loglog(tB, repmat(minV_dph, size(tB)), 'g--', 'LineWidth', 1.8);
    
    tK = tauVals(tauVals >= max(200, tau_BI_virt_g(a)*5) & tauVals <= tauVals(end));
    k_dps = K_virt_gyro(a) / (60.0 * SCALE_DPH);
    hK_line = loglog(tK, (k_dps * sqrt(tK / 3)) * SCALE_DPH, 'c-.', 'LineWidth', 1.8);
    
    plot(tauVals(minVIdx), minV_dph, 'kp', 'MarkerSize', 13, 'MarkerFaceColor', [1 0.85 0.1], 'LineWidth', 1.3);

    tau1_y_virt = adevV_dph(idxTau1s);
    text(1.0, tau1_y_virt * 0.45, ...
        sprintf('{\\bf\\color[rgb]{0,0.2,0.8}ARW (\\tau=1 с):}\nМассив: {\\bf%.4f} град/\\surdч\nЧип: %.4f град/\\surdч\n{\\bf\\color[rgb]{0.8,0,0}Выигрыш: %.2fx (теор. 6.0x)}', ...
        N_virt_gyro(a), mean(N_arw_gyro(:, a), 'omitnan'), gain_N), ...
        'FontSize', 9.5, 'BackgroundColor', [1 1 1 0.92], 'EdgeColor', [0 0.3 0.8]);
    
    % ИСПРАВЛЕННАЯ СТРОКА (B_bi_gyro вместо B_bi):
    text(tauVals(minVIdx) * 1.3, minV_dph * 0.85, ...
        sprintf('{\\bf\\color[rgb]{0,0.6,0.2}Bias Instability (BI):}\nМассив: {\\bf%.2f} град/ч (\\tau=%.0f с)\nЧип: %.2f град/ч\n{\\bf\\color[rgb]{0.8,0,0}Выигрыш: %.2fx}', ...
        B_virt_gyro(a), tau_BI_virt_g(a), mean(B_bi_gyro(:, a), 'omitnan'), gain_B), ...
        'FontSize', 9.5, 'BackgroundColor', [1 1 1 0.92], 'EdgeColor', [0 0.6 0.2]);

    grid on;
    xlim([tauVals(1), tauVals(end)]);
    ylim([min(minV_dph*0.35, min(meanAdev_dph)*0.35), max(meanAdev_dph)*3.0]);
    xlabel('Время усреднения \tau [секунды]', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('Allan Deviation \sigma(\tau) [град/ч]', 'FontSize', 12, 'FontWeight', 'bold');
    title(sprintf('Вариация Аллана IEEE Std 952 — %s (12-часовая запись, 36 IMU)', gyroAxisNames{a}), ...
        'FontSize', 13, 'FontWeight', 'bold');
    
    legend([hIndiv(1), hMean, hVirt, hN_line, hB_line, hK_line], { ...
        '36 одиночных датчиков (разброс чипов)', ...
        sprintf('Средний одиночный чип (ARW = %.4f, BI = %.2f)', mean(N_arw_gyro(:, a), 'omitnan'), mean(B_bi_gyro(:, a), 'omitnan')), ...
        sprintf('Синтезированный массив 36-IMU (ARW = %.4f, BI = %.2f)', N_virt_gyro(a), B_virt_gyro(a)), ...
        'Белый шум ARW (наклон -1/2)', ...
        'Нестабильность нуля BI (наклон 0)', ...
        'Случайный уход скорости RRW (наклон +1/2)' ...
        }, 'Location', 'southwest', 'FontSize', 9.0);

    dim = [0.61, 0.56, 0.27, 0.36];
    tableText = { ...
        sprintf('\\bf\\fontsize{10}ПАСПОРТ ПОГРЕШНОСТЕЙ IEEE 952 (%s)', gyroAxisShort{a}), ...
        '-----------------------------------------------------------------', ...
        sprintf('\\bf1. Угловой случайный уход (ARW, N):\\rm'), ...
        sprintf('   • Массив: \\bf%.4f\\rm град/\\surdч | Чип: %.4f град/\\surdч', N_virt_gyro(a), mean(N_arw_gyro(:, a), 'omitnan')), ...
        sprintf('   • Выигрыш синтеза: \\bf\\color[rgb]{0.8,0,0}%.2f x\\rm (теор. \\bf6.0 x\\rm)', gain_N), ...
        sprintf('\\bf2. Нестабильность смещения нуля (BI, B):\\rm'), ...
        sprintf('   • Массив: \\bf%.2f\\rm град/ч | Чип: %.2f град/ч', B_virt_gyro(a), mean(B_bi_gyro(:, a), 'omitnan')), ...
        sprintf('   • Выигрыш синтеза: \\bf\\color[rgb]{0.8,0,0}%.2f x\\rm (при \\tau = %.0f с)', gain_B, tau_BI_virt_g(a)), ...
        sprintf('\\bf3. Случайный уход скорости (RRW, K) [град/ч^{1.5}]:\\rm'), ...
        sprintf('   • Массив: \\bf%.2f\\rm | Чип: %.2f  [\\bf\\color[rgb]{0.8,0,0}%.2f x\\rm]', ...
            K_virt_gyro(a), mean(K_rrw_gyro(:, a), 'omitnan'), gain_K), ...
        '-----------------------------------------------------------------', ...
        sprintf('\\it\\fontsize{8}Эксперимент: %.2f ч (%.1f млн отсчетов) @ Fs=%.2f Гц\\rm', (nSamples*dt0)/3600, nSamples/1e6, Fs) ...
    };
    annotation('textbox', dim, 'String', tableText, 'FitBoxToText', 'on', ...
        'BackgroundColor', [1 1 1 0.95], 'EdgeColor', [0.4 0.4 0.4], 'LineWidth', 1.0, 'FontSize', 9.0);
end

%% #########################################################################
%% ЧАСТЬ II: АКСЕЛЕРОМЕТРЫ (КЛАССИЧЕСКИЙ OAVAR, IEEE STD 1293)
%% #########################################################################
fprintf('\n===========================================================================\n');
fprintf('                ЧАСТЬ II: АКСЕЛЕРОМЕТРЫ (КЛАССИЧЕСКИЙ IEEE STD 1293)       \n');
fprintf('===========================================================================\n');
tTotalAccel = tic;

SCALE_MG = 1000.0;
G_CONST  = 9.80665;

allanDevAccel3D   = zeros(nTau, nSensors, 3);
allanDevAccelVirt = zeros(nTau, 3);
accelAxisNames    = {'Акселерометр X (Ax)', 'Акселерометр Y (Ay)', 'Акселерометр Z (Az)'};
accelAxisShort    = {'Ax', 'Ay', 'Az'};

VRW_mps_sqrt_h = NaN(nSensors, 3);
BI_micro_g     = NaN(nSensors, 3);
Ka_accel_walk  = NaN(nSensors, 3);
tau_BI_accel   = NaN(nSensors, 3);

VRW_virt_accel = NaN(3, 1);
BI_virt_accel  = NaN(3, 1);
Ka_virt_accel  = NaN(3, 1);
tau_BI_virt_a  = NaN(3, 1);

for a = 1:3
    fprintf('Классический расчет оси %s...\n', accelAxisShort{a});
    tAxis = tic;
    rawAxis = h5read(h5FilePath, '/accel', [a, 1, 1], [1, nSensors, nSamples]);
    accelAxis = squeeze(rawAxis)'; % [nSamples x 36], [g]
    
    adevLocal = zeros(nTau, nSensors);
    parfor s = 1:nSensors
        acc = double(accelAxis(:, s));
        if ~all(isnan(acc))
            acc_d = acc - mean(acc, 'omitnan');
            adevLocal(:, s) = classical_oavar_core(acc_d, dt0, mSteps, tauVals);
        else
            adevLocal(:, s) = NaN(nTau, 1);
        end
    end
    allanDevAccel3D(:, :, a) = adevLocal;
    
    % Виртуальный массив акселерометров
    validSensors = ~isnan(accelAxis(1, :));
    virtualAcc   = double(mean(accelAxis(:, validSensors), 2));
    virtualAcc_d = virtualAcc - mean(virtualAcc, 'omitnan');
    allanDevAccelVirt(:, a) = classical_oavar_core(virtualAcc_d, dt0, mSteps, tauVals);
    
    % Идентификация параметров погрешностей
    for s = 1:nSensors
        adev = adevLocal(:, s);
        if all(isnan(adev)), continue; end
        VRW_mps_sqrt_h(s, a) = adev(idxTau1s) * G_CONST * 60.0; % [m/s / sqrt(hr)]
        [minVal, minIdx] = min(adev);
        BI_micro_g(s, a)   = (minVal / 0.664282) * 1e6; % микро-g
        tau_BI_accel(s, a) = tauVals(minIdx);
        
        idxK = find(tauVals >= 150 & tauVals <= tauVals(end));
        if numel(idxK) >= 3
            pK = polyfit(log10(tauVals(idxK)), log10(adev(idxK)), 1);
            Ka_accel_walk(s, a) = 10^(pK(2)) * sqrt(3) * 60.0 * SCALE_MG;
        else
            Ka_accel_walk(s, a) = adev(end) * sqrt(3 / tauVals(end)) * 60.0 * SCALE_MG;
        end
    end
    
    adevV = allanDevAccelVirt(:, a);
    VRW_virt_accel(a) = adevV(idxTau1s) * G_CONST * 60.0;
    [minV, minVIdx]   = min(adevV);
    BI_virt_accel(a)  = (minV / 0.664282) * 1e6; % микро-g
    tau_BI_virt_a(a)  = tauVals(minVIdx);
    
    idxKa_v = find(tauVals >= max(200, tau_BI_virt_a(a)*5) & tauVals <= tauVals(end));
    if numel(idxKa_v) >= 3
        pKa_v = polyfit(log10(tauVals(idxKa_v)), log10(adevV(idxKa_v)), 1);
        Ka_virt_accel(a) = 10^(pKa_v(2)) * sqrt(3) * 60.0 * SCALE_MG;
    else
        Ka_virt_accel(a) = adevV(end) * sqrt(3 / tauVals(end)) * 60.0 * SCALE_MG;
    end
    
    clear accelAxis rawAxis;
    fprintf('  Ось %s рассчитана за: %.2f с\n', accelAxisShort{a}, toc(tAxis));
end
fprintf('Все 3 оси акселерометров рассчитаны за: %.2f с!\n', toc(tTotalAccel));

%% ===================== 7. ВЫВОД ПАСПОРТА АКСЕЛЕРОМЕТРОВ =====================
for a = 1:3
    gain_VRW = mean(VRW_mps_sqrt_h(:, a), 'omitnan') / VRW_virt_accel(a);
    gain_BIa = mean(BI_micro_g(:, a), 'omitnan') / BI_virt_accel(a);
    gain_Ka  = mean(Ka_accel_walk(:, a), 'omitnan') / Ka_virt_accel(a);
    
    fprintf('\n================== ПАСПОРТ IEEE 1293: %s ==================\n', accelAxisNames{a});
    fprintf('ПАРАМЕТР                              | ОДИНОЧНЫЙ ЧИП (СРЕДНЕЕ) | МАССИВ 36-IMU  | ВЫИГРЫШ\n');
    fprintf('--------------------------------------+-------------------------+----------------+---------\n');
    fprintf('Случайный уход скорости (VRW) [м/с/√ч]| %17.4f       | %10.4f     | %6.2f x (теор. 6.0x)\n', ...
        mean(VRW_mps_sqrt_h(:, a), 'omitnan'), VRW_virt_accel(a), gain_VRW);
    fprintf('Нестабильность нуля (BI, Ba) [мк-g]   | %17.2f       | %10.2f     | %6.2f x (tau = %.0f с)\n', ...
        mean(BI_micro_g(:, a), 'omitnan'), BI_virt_accel(a), gain_BIa, tau_BI_virt_a(a));
    fprintf('Случайный уход ускорения (Ka) [мг/√ч] | %17.2f       | %10.2f     | %6.2f x\n', ...
        mean(Ka_accel_walk(:, a), 'omitnan'), Ka_virt_accel(a), gain_Ka);
    fprintf('--------------------------------------+-------------------------+----------------+---------\n');
end

%% ===================== 8. ПОСТРОЕНИЕ 3 ГРАФИКОВ АКСЕЛЕРОМЕТРОВ =====================
for a = 1:3
    figA = figure('Name', sprintf('MIIB: Вариация Аллана IEEE 1293 — %s', accelAxisNames{a}), ...
        'Color', 'w', 'Position', [70 + (a-1)*40, 60 + (a-1)*30, 1400, 850]);
    
    adevSensors_mg = squeeze(allanDevAccel3D(:, :, a)) * SCALE_MG;
    meanAdev_mg    = mean(adevSensors_mg, 2, 'omitnan');
    adevV_mg       = allanDevAccelVirt(:, a) * SCALE_MG;
    [minV_mg, minVIdx] = min(adevV_mg);
    
    gain_VRW = mean(VRW_mps_sqrt_h(:, a), 'omitnan') / VRW_virt_accel(a);
    gain_BIa = mean(BI_micro_g(:, a), 'omitnan') / BI_virt_accel(a);
    gain_Ka  = mean(Ka_accel_walk(:, a), 'omitnan') / Ka_virt_accel(a);

    hIndiv = loglog(tauVals, adevSensors_mg, 'Color', [0.85 0.85 0.85], 'LineWidth', 0.5); hold on;
    hMean  = loglog(tauVals, meanAdev_mg, 'k--', 'LineWidth', 2.0);
    hVirt  = loglog(tauVals, adevV_mg, 'Color', [0.08 0.45 0.85], 'LineWidth', 3.0);
    
    tN = tauVals(tauVals >= 0.008 & tauVals <= 15);
    vrw_virt_g_s = (VRW_virt_accel(a) / G_CONST / 60.0);
    hVRW_line = loglog(tN, (vrw_virt_g_s ./ sqrt(tN)) * SCALE_MG, 'r-.', 'LineWidth', 1.8);
    
    tB = tauVals(tauVals >= tau_BI_virt_a(a)*0.2 & tauVals <= tau_BI_virt_a(a)*5);
    hB_line = loglog(tB, repmat(minV_mg, size(tB)), 'g--', 'LineWidth', 1.8);
    
    tK = tauVals(tauVals >= max(200, tau_BI_virt_a(a)*5) & tauVals <= tauVals(end));
    ka_g_s = (Ka_virt_accel(a) / SCALE_MG) / 60.0;
    hKa_line = loglog(tK, (ka_g_s * sqrt(tK / 3)) * SCALE_MG, 'm-.', 'LineWidth', 1.8);
    
    plot(tauVals(minVIdx), minV_mg, 'kp', 'MarkerSize', 13, 'MarkerFaceColor', [1 0.85 0.1], 'LineWidth', 1.3);

    tau1_y_virt = adevV_mg(idxTau1s);
    text(1.0, tau1_y_virt * 0.45, ...
        sprintf('{\\bf\\color[rgb]{0.8,0,0}VRW (\\tau=1 с):}\nМассив: {\\bf%.4f} м/с/\\surdч\nЧип: %.4f м/с/\\surdч\n{\\bf\\color[rgb]{0,0.4,0.8}Выигрыш: %.2fx (теор. 6.0x)}', ...
        VRW_virt_accel(a), mean(VRW_mps_sqrt_h(:, a), 'omitnan'), gain_VRW), ...
        'FontSize', 9.5, 'BackgroundColor', [1 1 1 0.92], 'EdgeColor', [0.8 0 0]);
    
    text(tauVals(minVIdx) * 1.3, minV_mg * 0.85, ...
        sprintf('{\\bf\\color[rgb]{0,0.6,0.2}Bias Instability (Ba):}\nМассив: {\\bf%.1f} мк-g (\\tau=%.0f с)\nЧип: %.1f мк-g\n{\\bf\\color[rgb]{0,0.4,0.8}Выигрыш: %.2fx}', ...
        BI_virt_accel(a), tau_BI_virt_a(a), mean(BI_micro_g(:, a), 'omitnan'), gain_BIa), ...
        'FontSize', 9.5, 'BackgroundColor', [1 1 1 0.92], 'EdgeColor', [0 0.6 0.2]);

    grid on;
    xlim([tauVals(1), tauVals(end)]);
    ylim([min(minV_mg*0.35, min(meanAdev_mg)*0.35), max(meanAdev_mg)*3.0]);
    xlabel('Время усреднения \tau [секунды]', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('Allan Deviation \sigma(\tau) [мг] (mg)', 'FontSize', 12, 'FontWeight', 'bold');
    title(sprintf('Вариация Аллана IEEE Std 1293 — %s (12-часовая запись, 36 IMU)', accelAxisNames{a}), ...
        'FontSize', 13, 'FontWeight', 'bold');
    
    legend([hIndiv(1), hMean, hVirt, hVRW_line, hB_line, hKa_line], { ...
        '36 одиночных акселерометров (разброс чипов)', ...
        sprintf('Средний одиночный чип (VRW = %.3f, BI = %.0f мк-g)', mean(VRW_mps_sqrt_h(:, a), 'omitnan'), mean(BI_micro_g(:, a), 'omitnan')), ...
        sprintf('Синтезированный массив 36-IMU (VRW = %.4f, BI = %.1f мк-g)', VRW_virt_accel(a), BI_virt_accel(a)), ...
        'Белый шум VRW (наклон -1/2)', ...
        'Нестабильность нуля BI (наклон 0)', ...
        'Случайный уход ускорения (наклон +1/2)' ...
        }, 'Location', 'southwest', 'FontSize', 9.0);

    dim = [0.61, 0.56, 0.27, 0.36];
    tableText = { ...
        sprintf('\\bf\\fontsize{10}ПАСПОРТ ПОГРЕШНОСТЕЙ IEEE 1293 (%s)', accelAxisShort{a}), ...
        '-----------------------------------------------------------------', ...
        sprintf('\\bf1. Случайный уход скорости (VRW, Kv):\\rm'), ...
        sprintf('   • Массив: \\bf%.4f\\rm м/с/\\surdч | Чип: %.4f м/с/\\surdч', VRW_virt_accel(a), mean(VRW_mps_sqrt_h(:, a), 'omitnan')), ...
        sprintf('   • Выигрыш синтеза: \\bf\\color[rgb]{0,0.4,0.8}%.2f x\\rm (теор. \\bf6.0 x\\rm)', gain_VRW), ...
        sprintf('\\bf2. Нестабильность смещения нуля (BI, Ba):\\rm'), ...
        sprintf('   • Массив: \\bf%.1f\\rm мк-g | Чип: %.1f мк-g', BI_virt_accel(a), mean(BI_micro_g(:, a), 'omitnan')), ...
        sprintf('   • Выигрыш синтеза: \\bf\\color[rgb]{0.8,0,0}%.2f x\\rm (при \\tau = %.0f с)', gain_BIa, tau_BI_virt_a(a)), ...
        sprintf('\\bf3. Случайный уход ускорения (Ka) [мг/\\surdч]:\\rm'), ...
        sprintf('   • Массив: \\bf%.2f\\rm | Чип: %.2f  [\\bf\\color[rgb]{0.8,0,0}%.2f x\\rm]', ...
            Ka_virt_accel(a), mean(Ka_accel_walk(:, a), 'omitnan'), gain_Ka), ...
        '-----------------------------------------------------------------', ...
        sprintf('\\it\\fontsize{8}Эксперимент: %.2f ч (%.1f млн отсчетов) @ Fs=%.2f Гц\\rm', (nSamples*dt0)/3600, nSamples/1e6, Fs) ...
    };
    annotation('textbox', dim, 'String', tableText, 'FitBoxToText', 'on', ...
        'BackgroundColor', [1 1 1 0.95], 'EdgeColor', [0.4 0.4 0.4], 'LineWidth', 1.0, 'FontSize', 9.0);
end

fprintf('\nВсе 6 окон классической вариации Аллана успешно построены!\n');

%% ===================== 9. ЛОКАЛЬНАЯ КЛАССИЧЕСКАЯ ФУНКЦИЯ OAVAR =====================
function adev = classical_oavar_core(sig, dt0, mSteps, tauVals)
% CLASSICAL_OAVAR_CORE Непрерывный классический расчет OAVAR без приближений
    nSamples = numel(sig);
    nTau = numel(tauVals);
    adev = zeros(nTau, 1);
    
    nanIdx = isnan(sig);
    if any(nanIdx)
        validIdx = find(~nanIdx);
        sig(nanIdx) = interp1(validIdx, sig(validIdx), find(nanIdx), 'linear', 'extrap');
    end
    
    th = [0; cumsum(sig * dt0)];
    
    for k = 1:nTau
        m = mSteps(k);
        tau = tauVals(k);
        diff2 = th(1 + 2*m : end) - 2 * th(1 + m : end - m) + th(1 : end - 2*m);
        avar  = sum(diff2.^2) / (2 * (tau^2) * (nSamples - 2*m));
        adev(k) = sqrt(avar);
    end
end