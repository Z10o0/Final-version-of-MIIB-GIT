%% ===================== НАСТРОЙКИ РАСЧЕТА =====================
clear; clc; close all;

h5FilePath   = 'miib_data_single_12h.h5'; % Имя HDF5-файла
ptsPerDecade = 45;                     % Академическая плотность (45 точек на декаду)

if ~exist(h5FilePath, 'file')
    error('Файл %s не найден! Проверьте путь к файлу.', h5FilePath);
end

%% ===================== ИНИЦИАЛИЗАЦИЯ ПАРАЛЛЕЛЬНОГО ПУЛА =====================
pool = gcp('nocreate');
if isempty(pool)
    fprintf('Запуск параллельного пула MATLAB...\n');
    pool = parpool('local');
else
    fprintf('Используется активный пул: %d воркеров.\n', pool.NumWorkers);
end

%% ===================== ЧТЕНИЕ ИНФОРМАЦИИ И МЕТАДАННЫХ =====================
info = h5info(h5FilePath, '/gyro');
nSamples = info.Dataspace.Size(3); % Все 69 120 000 кадров
nSensors = info.Dataspace.Size(2); % 36 датчиков

% Оценка Fs по аппаратному таймштампу
tstampBlock = h5read(h5FilePath, '/tstamp', [1, 1], [1, min(50000, nSamples)]);
dTs = mod(diff(double(squeeze(tstampBlock))), 65536);
dTs = dTs(dTs > 0);
meanTicks = mode(dTs);
Fs = 1e6 / meanTicks; % ~1647.45 Гц
dt0 = 1.0 / Fs;

fprintf('Длина выборки: %d кадров (%.2f минут / %.2f часов)\n', ...
    nSamples, (nSamples * dt0) / 60, (nSamples * dt0) / 3600);
fprintf('Частота опроса: Fs = %.2f Гц (dt = %.4f мс)\n\n', Fs, dt0 * 1000);

%% ===================== СЕТКА КЛАСТЕРОВ TAU (IEEE STD 952) =====================
maxCluster = floor(nSamples / 9); % IEEE Std 952: отсекаем недостоверный хвост
nDecades   = log10(maxCluster);
nTauPoints = round(nDecades * ptsPerDecade);

mSteps  = unique(round(logspace(0, log10(maxCluster), nTauPoints)));
tauVals = mSteps * dt0;
nTau    = numel(tauVals);

fprintf('Сетка tau: %d точек (%.1f точек/декаду) от %.4f с до %.1f с (%.2f ч)\n\n', ...
    nTau, nTau / nDecades, tauVals(1), tauVals(end), tauVals(end)/3600);

%% ===================== ПАРАЛЛЕЛЬНЫЙ РАСЧЕТ БЕЗ ПЕРЕГРУЗКИ ОЗУ =====================
% Результат: 3D массив всего 300 x 36 x 3 (несколько килобайт)
allanDev3D = zeros(nTau, nSensors, 3);
allanDevVirt = zeros(nTau, 3);

axisNames = {'Гироскоп X (Gx)', 'Гироскоп Y (Gy)', 'Гироскоп Z (Gz)'};
axisShort = {'Gx', 'Gy', 'Gz'};

for a = 1:3
    fprintf('---------------------------------------------------\n');
    fprintf('ОБРАБОТКА ОСИ %d/3: %s\n', a, axisNames{a});
    fprintf('---------------------------------------------------\n');
    
    tAxis = tic;
    fprintf('  Чтение оси %s из HDF5 (36 датчиков, single)...\n', axisShort{a});
    % Читаем одну ось [1 x 36 x nSamples] -> преобразуем в single [nSamples x 36]
    % Занимает в ОЗУ всего ~4.9 ГБ в формате single
    rawAxis = h5read(h5FilePath, '/gyro', [a, 1, 1], [1, nSensors, nSamples]);
    gyroAxis = squeeze(rawAxis)'; % [nSamples x 36], single
    
    fprintf('  Старт параллельного расчета OAVAR (parfor по 36 датчикам)...\n');
    adevAxisLocal = zeros(nTau, nSensors);
    
    parfor s = 1:nSensors
        rate = double(gyroAxis(:, s));
        nanIdx = isnan(rate);
        
        if all(nanIdx)
            adevAxisLocal(:, s) = NaN(nTau, 1);
            continue;
        end
        
        % Устранение редких пропусков
        if any(nanIdx)
            validIdx = find(~nanIdx);
            rate(nanIdx) = interp1(validIdx, rate(validIdx), find(nanIdx), 'linear', 'extrap');
        end
        
        % Интегрирование фазы для одного датчика (~550 МБ в локальной памяти ядра)
        th = [0; cumsum(rate * dt0)];
        
        adevSensor = zeros(nTau, 1);
        for k = 1:nTau
            m = mSteps(k);
            tau = tauVals(k);
            diff2 = th(1 + 2*m : end) - 2 * th(1 + m : end - m) + th(1 : end - 2*m);
            avar = sum(diff2.^2) / (2 * (tau^2) * (nSamples - 2*m));
            adevSensor(k) = sqrt(avar);
        end
        adevAxisLocal(:, s) = adevSensor;
    end
    allanDev3D(:, :, a) = adevAxisLocal;
    
    %% Расчет виртуального синтезированного массива (Array Fusion)
    fprintf('  Расчет синтезированного виртуального массива (36-IMU Fusion)...\n');
    validSensors = ~isnan(gyroAxis(1, :));
    virtualRate = double(mean(gyroAxis(:, validSensors), 2));
    thVirt = [0; cumsum(virtualRate * dt0)];
    
    virtLocal = zeros(nTau, 1);
    parfor k = 1:nTau
        m = mSteps(k);
        tau = tauVals(k);
        diff2 = thVirt(1 + 2*m : end) - 2 * thVirt(1 + m : end - m) + thVirt(1 : end - 2*m);
        virtLocal(k) = sqrt(sum(diff2.^2) / (2 * (tau^2) * (nSamples - 2*m)));
    end
    allanDevVirt(:, a) = virtLocal;
    
    % Очищаем ось из памяти перед чтением следующей
    clear gyroAxis rawAxis;
    fprintf('  Ось %s успешно рассчитана за %.2f с!\n\n', axisShort{a}, toc(tAxis));
end

%% ===================== ОЦЕНКА ПАРАМЕТРОВ ПОГРЕШНОСТЕЙ (IEEE STD 952) =====================
[~, idxTau1s] = min(abs(tauVals - 1.0));

ARW_single = zeros(3, 1);
BI_single  = zeros(3, 1);
ARW_array  = zeros(3, 1);
BI_array   = zeros(3, 1);
tau_BI_arr = zeros(3, 1);

fprintf('================== АКАДЕМИЧЕСКАЯ ОЦЕНКА МАССИВА 36-IMU ==================\n');

for a = 1:3
    adevSensors = squeeze(allanDev3D(:, :, a));
    
    % Поодиночные датчики
    arw_all = adevSensors(idxTau1s, :) * 60.0;             % deg/sqrt(hr)
    bi_all  = (min(adevSensors, [], 1) / 0.664) * 3600.0; % deg/hr
    
    ARW_single(a) = mean(arw_all, 'omitnan');
    BI_single(a)  = mean(bi_all, 'omitnan');
    
    % Синтезированный массив
    adevV = allanDevVirt(:, a);
    [minV, minVIdx] = min(adevV);
    ARW_array(a)  = adevV(idxTau1s) * 60.0;
    BI_array(a)   = (minV / 0.664) * 3600.0;
    tau_BI_arr(a) = tauVals(minVIdx);
    
    fprintf('ОСЬ %s:\n', axisShort{a});
    fprintf('  1 Чип:   ARW = %.4f deg/sqrt(hr) | Bias Instability = %.2f deg/hr\n', ...
        ARW_single(a), BI_single(a));
    fprintf('  Массив:  ARW = %.4f deg/sqrt(hr) | Bias Instability = %.2f deg/hr (tau = %.1f с)\n', ...
        ARW_array(a), BI_array(a), tau_BI_arr(a));
    fprintf('  Выигрыш: ARW улучшен в %.2f x | BI улучшена в %.2f x\n\n', ...
        ARW_single(a) / ARW_array(a), BI_single(a) / BI_array(a));
end
fprintf('=========================================================================\n\n');

%% ===================== ПОСТРОЕНИЕ 3 ГРАФИКОВ (X, Y, Z) =====================
for a = 1:3
    fig = figure('Name', sprintf('MIIB: Вариация Аллана — %s', axisNames{a}), ...
        'Color', 'w', 'Position', [80 + (a-1)*60, 60 + (a-1)*50, 1300, 800]);
    
    adevSensors = squeeze(allanDev3D(:, :, a));
    meanAdev = mean(adevSensors, 2, 'omitnan');
    adevV = allanDevVirt(:, a);
    [minV, minVIdx] = min(adevV);
    
    % 1. Все 36 одиночных датчиков
    hIndiv = loglog(tauVals, adevSensors, 'Color', [0.82 0.82 0.82], 'LineWidth', 0.6); hold on;
    
    % 2. Средняя кривая чипа
    hMean = loglog(tauVals, meanAdev, 'k--', 'LineWidth', 2.0);
    
    % 3. Синтезированный виртуальный массив 36-IMU
    hVirt = loglog(tauVals, adevV, 'Color', [0.85 0.1 0.1], 'LineWidth', 2.8);
    
    % 4. Асимптота белого шума (ARW: наклон -0.5)
    tauFit = tauVals(tauVals <= 8);
    hARW = loglog(tauFit, (adevV(idxTau1s) ./ sqrt(tauFit)), 'b-.', 'LineWidth', 1.6);
    
    % 5. Точка минимума Bias Instability массива
    hBI_pt = plot(tauVals(minVIdx), minV, 'kp', 'MarkerSize', 11, ...
        'MarkerFaceColor', [1 0.8 0.1], 'LineWidth', 1.2);
    
    grid on;
    xlim([tauVals(1), tauVals(end)]);
    xlabel('Время усреднения \tau [секунды]', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('Allan Deviation \sigma(\tau) [deg/s]', 'FontSize', 12, 'FontWeight', 'bold');
    title(sprintf('Вариация Аллана IEEE Std 952 — %s (12-часовая запись, 36 IMU)', axisNames{a}), ...
        'FontSize', 13, 'FontWeight', 'bold');
    
    legend([hIndiv(1), hMean, hVirt, hARW, hBI_pt], { ...
        sprintf('36 одиночных датчиков (разброс чипов)'), ...
        sprintf('Средний одиночный датчик (ARW = %.4f °/\\surdч, BI = %.2f °/ч)', ARW_single(a), BI_single(a)), ...
        sprintf('Синтезированный массив 36-IMU (ARW = %.4f °/\\surdч, BI = %.2f °/ч)', ARW_array(a), BI_array(a)), ...
        'Асимптота белого шума ARW (наклон -1/2)', ...
        sprintf('Минимум нестабильности нуля: %.2f °/ч (\\tau = %.1f с)', BI_array(a), tau_BI_arr(a)) ...
        }, 'Location', 'southwest', 'FontSize', 10);
    
    % Текстовая плашка параметров
    dim = [0.62, 0.68, 0.26, 0.22];
    str = { ...
        sprintf('\\bfПараметры оси %s:', axisShort{a}), ...
        sprintf('\\rm• Выигрыш ARW: \\bf%.2f x\\rm (теор. \\surd36 = 6.0x)', ARW_single(a) / ARW_array(a)), ...
        sprintf('• Выигрыш BI:  \\bf%.2f x\\rm', BI_single(a) / BI_array(a)), ...
        sprintf('• Точек \\tau: %d (%.1f т/дек)', nTau, nTau/nDecades), ...
        sprintf('• Запись: %.2f часов (69.1 млн отсчетов)', (nSamples*dt0)/3600) ...
    };
    annotation('textbox', dim, 'String', str, 'FitBoxToText', 'on', ...
        'BackgroundColor', [1 1 1 0.9], 'EdgeColor', [0.7 0.7 0.7], 'FontSize', 10);
end

fprintf('Все 3 окна графиков (X, Y, Z) успешно построены!\n');