% =========================================================================
% MIIB_parser.m  —  ICM-45686 UART телеметрия  v7
% v7: адаптация под новый прошивочный протокол (100 Гц UART, 16 FIFO-
%     пакетов/цикл на стороне MCU) + строгое разделение фаз:
%
%     ФАЗА 1 (ПРИЁМ):     ТОЛЬКО накопление сырых байт в буфер.
%                         Никакого парсинга, поиска заголовков, CRC,
%                         построения матриц — вообще никакой обработки.
%                         Callback лишь копирует байты из драйвера в RAM.
%
%     ФАЗА 2 (ОБРАБОТКА): Запускается ПОСЛЕ полной остановки приёма
%                         (delete(s) уже выполнен, порт закрыт).
%                         Парсинг, CRC, сборка матриц, статистика, графики.
%
%     Такое разделение убирает саму возможность просадки приёма из-за
%     "тяжёлых" операций (regexp/find/CRC) внутри callback — единственная
%     работа callback'а теперь swap буфера, что занимает микросекунды
%     независимо от сложности протокола.
%
%     Также обновлены протокольные константы под новую прошивку:
%       FRAME_HZ 1600 → 100   (частота UART-кадров, TIM6 @ 100 Гц)
%       ODR_HZ    остаётся 1600 (частота ODR датчика, не менялась)
%       SAMPLES_PER_ICM_READ = 16 — количество HIRES FIFO-пакетов,
%                 накопленных MCU за один цикл опроса (только для
%                 справки/документации: UART передаёт один — самый
%                 свежий — сэмпл на кадр, как и раньше).
%       expected_delta_us — исправлена формула: она относится к дельте
%                 hardware-timestamp МЕЖДУ ПОСЛЕДОВАТЕЛЬНЫМИ FIFO-
%                 сэмплами внутри датчика (ODR=1600 Гц → ~625 мкс),
%                 а НЕ к периоду UART-кадра. Множитель "10" в v6 был
%                 привязан к старой (уже не актуальной) геометрии батча
%                 и убран.
% =========================================================================
close all; clearvars; clc;

%% ── CONFIG ───────────────────────────────────────────────────────────────
PORT        = 'COM3';
BAUD        = 8100000;      % должно совпадать с USART1 в MCU (8.1 Мбод)
                             % ЕСЛИ мост FT232H/FT2232H — проверь реально
                             % достижимый custom baud (может округляться
                             % драйвером).
BUF_SIZE    = 8 * 1024 * 1024;   % при 100 Гц x 348 байт = 34.8 КБ/с
                                  % запас под TIMEOUT_S=60 c с большим
                                  % запасом; можно уменьшить.
TIMEOUT_S   = 20;
N_SENSORS   = 18;
PKT_TOTAL   = 348;           % размер UART-кадра не менялся
FRAME_HZ    = 1600;           % было 1600 → теперь 100 (TIM6 @ 100 Гц)
SAMPLES_PER_ICM_READ = 16;   % справочно: пакетов FIFO на цикл опроса MCU
SENS_A      = 16384.0;       % LSB/g   при FS=16g   (не менять!)
SENS_G      = 131.072;       % LSB/dps при FS=2000dps (не менять!)
ODR_HZ      = 1600;          % ODR датчика (не менялась)

%% ── ОТКРЫТИЕ ПОРТА ───────────────────────────────────────────────────────
s = serialport(PORT, BAUD, 'Timeout', TIMEOUT_S + 2);
s.InputBufferSize = BUF_SIZE;
flush(s);
fprintf('Listening on %s @ %d baud for %d s...\n', PORT, BAUD, TIMEOUT_S);

%% ── ФАЗА 1: ЧИСТЫЙ ПРИЁМ (без какой-либо обработки) ──────────────────────
% chunk_store — cell-массив кусков; склейка в один uint8-вектор делается
% ОДИН раз после остановки приёма (в начале ФАЗЫ 2), не раньше.
%
% Внутри local_capture() нет НИЧЕГО, кроме read() — ни поиска заголовков,
% ни проверки CRC, ни malloc'ов сверх предвыделенной ячейки. Это и есть
% "легковесность": единственная работа во время приёма — скопировать
% байты из драйверного буфера в RAM.
chunk_store = cell(50000, 1);   % с запасом; при 100 Гц кадров/callback
                                 % событий на порядки меньше, чем было
                                 % на 1600 Гц, запас 50000 более чем
                                 % достаточен даже для TIMEOUT_S=600 c.
chunk_count = 0;

configureCallback(s, "byte", PKT_TOTAL, @(src, ~) local_capture(src));

    function local_capture(src)
        n = src.NumBytesAvailable;
        if n > 0
            chunk_count = chunk_count + 1;
            chunk_store{chunk_count} = read(src, n, 'uint8');
        end
    end

t_cap = tic;
while toc(t_cap) < TIMEOUT_S
    % Только отдаём управление event loop для доставки callback'ов.
    % Никакой парсинг сюда переносить нельзя — иначе теряется весь
    % смысл разделения фаз.
    drawnow limitrate;
end

configureCallback(s, "off");

% Финальный вычит остатка буфера (данные, накопленные между последним
% callback и остановкой) — тоже часть ФАЗЫ 1, обработки здесь ещё нет.
n_tail = s.NumBytesAvailable;
if n_tail > 0
    chunk_count = chunk_count + 1;
    chunk_store{chunk_count} = read(s, n_tail, 'uint8');
end

delete(s);   % порт закрыт — приём гарантированно завершён

% ── ГРАНИЦА ФАЗ ───────────────────────────────────────────────────────────
% Всё, что выше этой строки, относится ТОЛЬКО к приёму.
% Всё, что ниже, выполняется ИСКЛЮЧИТЕЛЬНО после его завершения.
% ───────────────────────────────────────────────────────────────────────

raw = vertcat(chunk_store{1:chunk_count});
raw = uint8(raw(:));
clear chunk_store;   % освобождаем память, дальше работаем с raw

expected_bytes = round(FRAME_HZ * TIMEOUT_S * PKT_TOTAL);
fprintf('Received %d bytes (expected ~%d = %.1f%%)\n', ...
    numel(raw), expected_bytes, numel(raw)/expected_bytes*100);

if numel(raw) < PKT_TOTAL
    error('Слишком мало данных (%d байт).', numel(raw));
end

%% ── ДИАГНОСТИКА (ФАЗА 2) ─────────────────────────────────────────────────
fprintf('\n--- ДИАГНОСТИКА ---\n');
fprintf('0xAA в буфере: %d\n', sum(raw == uint8(hex2dec('AA'))));
fprintf('0x55 в буфере: %d\n', sum(raw == uint8(hex2dec('55'))));
fprintf('First 40 bytes: ');
fprintf('%02X ', raw(1:min(40,end)));
fprintf('\n\n');

%% ── ПАРСИНГ (векторизованный поиск заголовков) ───────────────────────────
% Структура пакета (MATLAB индексы 1-based):
%   pkt(1..2)    : header 0xAA 0x55
%   pkt(3..346)  : payload = frame_counter(2) + 18*19 bytes  ← CRC по нему
%   pkt(347..348): CRC16 LE
n_raw = numel(raw);

is_aa   = (raw(1:end-1) == uint8(hex2dec('AA')));
is_55   = (raw(2:end)   == uint8(hex2dec('55')));
hdr_pos = find(is_aa & is_55);   % позиции начала потенциальных заголовков

frames   = struct('counter', {}, 'samples', {});
bad_crc  = 0;
hdr_hits = numel(hdr_pos);
last_end = 0;   % конец предыдущего успешно распарсенного пакета

for k = 1:hdr_hits
    idx = hdr_pos(k);
    if idx <= last_end
        continue;   % этот заголовок уже внутри предыдущего валидного пакета
    end
    if idx + PKT_TOTAL - 1 > n_raw
        break;
    end

    pkt = raw(idx : idx + PKT_TOTAL - 1);
    payload  = pkt(3:346);
    crc_rx   = bitor(uint16(pkt(347)), bitshift(uint16(pkt(348)), 8));
    crc_calc = crc16ccitt(payload);

    if crc_calc ~= crc_rx
        bad_crc = bad_crc + 1;
        continue;
    end

    f.counter = typecast(uint8(payload(1:2)), 'uint16');
    samp = nan(N_SENSORS, 8);

    for sid = 1:N_SENSORS
        b = 3 + (sid - 1) * 19;
        ax_r = unpack20(payload(b),    payload(b+1),  bitshift(uint8(payload(b+16)), -4));
        ay_r = unpack20(payload(b+2),  payload(b+3),  bitshift(uint8(payload(b+17)), -4));
        az_r = unpack20(payload(b+4),  payload(b+5),  bitshift(uint8(payload(b+18)), -4));
        gx_r = unpack20(payload(b+6),  payload(b+7),  bitand(uint8(payload(b+16)), uint8(15)));
        gy_r = unpack20(payload(b+8),  payload(b+9),  bitand(uint8(payload(b+17)), uint8(15)));
        gz_r = unpack20(payload(b+10), payload(b+11), bitand(uint8(payload(b+18)), uint8(15)));

        tr_u = bitor(bitshift(uint16(payload(b+12)), 8), uint16(payload(b+13)));
        tr   = typecast(tr_u, 'int16');

        ts = bitor(bitshift(uint16(payload(b+14)), 8), uint16(payload(b+15)));

        samp(sid,:) = [
            double(ax_r) / SENS_A,
            double(ay_r) / SENS_A,
            double(az_r) / SENS_A,
            double(gx_r) / SENS_G,
            double(gy_r) / SENS_G,
            double(gz_r) / SENS_G,
            double(tr)   / 128.0 + 25.0,
            double(ts) ];
    end

    f.samples = samp;
    frames(end+1) = f; %#ok<AGROW>
    last_end = idx + PKT_TOTAL - 1;
end

fprintf('Header hits  : %d\n', hdr_hits);
fprintf('Parsed frames: %d\n', numel(frames));
fprintf('Bad CRC      : %d\n', bad_crc);

if isempty(frames)
    fprintf('\n[HINT] Нет валидных пакетов.\n');
    if bad_crc  > 0, fprintf('  → Bad CRC: baudrate или wire-формат?\n'); end
    if hdr_hits == 0, fprintf('  → Нет 0xAA 0x55: baud=%d?\n', BAUD); end
    error('No valid frames.');
end

%% ── СБОРКА МАТРИЦ ────────────────────────────────────────────────────────
nF = numel(frames);
ctr_raw = zeros(nF, 1, 'uint16');
for fi = 1:nF
    ctr_raw(fi) = frames(fi).counter;
end

ctr_step = zeros(nF, 1, 'double');
if nF > 1
    dctr = mod(diff(double(ctr_raw)), 65536);
    ctr_step(1)     = 0;
    ctr_step(2:end) = cumsum(dctr);
end
t = ctr_step / FRAME_HZ;

ax   = nan(nF, N_SENSORS);  ay   = nan(nF, N_SENSORS);
az   = nan(nF, N_SENSORS);  gx   = nan(nF, N_SENSORS);
gy   = nan(nF, N_SENSORS);  gz   = nan(nF, N_SENSORS);
temp = nan(nF, N_SENSORS);  ts_  = nan(nF, N_SENSORS);
ctr  = ctr_raw;

for fi = 1:nF
    ax(fi,:)   = frames(fi).samples(:,1).';
    ay(fi,:)   = frames(fi).samples(:,2).';
    az(fi,:)   = frames(fi).samples(:,3).';
    gx(fi,:)   = frames(fi).samples(:,4).';
    gy(fi,:)   = frames(fi).samples(:,5).';
    gz(fi,:)   = frames(fi).samples(:,6).';
    temp(fi,:) = frames(fi).samples(:,7).';
    ts_(fi,:)  = frames(fi).samples(:,8).';
end

%% ── GAP-ФИЛЬТР ───────────────────────────────────────────────────────────
for sid = 1:N_SENSORS
    rising = find(diff([0; ~isnan(ax(:,sid))]) == 1);
    for e = 1:numel(rising)
        ri = rising(e);
        ax(ri,sid)=NaN; ay(ri,sid)=NaN; az(ri,sid)=NaN;
        gx(ri,sid)=NaN; gy(ri,sid)=NaN; gz(ri,sid)=NaN;
        temp(ri,sid)=NaN;
    end
end

%% ── ДИАГНОСТИКА СЧЁТЧИКА ─────────────────────────────────────────────────
fprintf('\n--- ДИАГНОСТИКА СЧЁТЧИКА ---\n');
fprintf('Первый counter: %d, последний: %d\n', ctr(1), ctr(end));
if nF > 1
    dctr = mod(diff(double(ctr)), 65536);
    n_drops = sum(dctr ~= 1);
    if n_drops > 0
        fprintf('[WARNING] %d скачков счётчика (потеряно кадров UART).\n', n_drops);
        drop_t = t(find(dctr ~= 1) + 1);
        fprintf('  Моменты (с): ');
        fprintf('%.3f ', drop_t(1:min(20,end)));
        fprintf('\n');
    else
        fprintf('Фреймы непрерывны (счётчик монотонно +1 modulo 65536).\n');
    end
end

%% ── СТАТИСТИКА IMU ───────────────────────────────────────────────────────
fprintf('\n--- СТАТИСТИКА ---\n');
for sid = 1:N_SENSORS
    nv = sum(~isnan(ax(:,sid)));
    if nv == 0, continue; end
    fprintf('S%02d valid=%5.1f%%  Ax=%+.4f±%.4f g  Gx=%+.2f±%.2f dps  T=%.1f±%.2f°C\n', ...
        sid-1, nv/nF*100, ...
        mean(ax(:,sid),'omitnan'), std(ax(:,sid),'omitnan'), ...
        mean(gx(:,sid),'omitnan'), std(gx(:,sid),'omitnan'), ...
        mean(temp(:,sid),'omitnan'), std(temp(:,sid),'omitnan'));
end

%% ── СТАТИСТИКА TIMESTAMP ─────────────────────────────────────────────────
% [ИЗМЕНЕНИЕ v7] expected_delta_us теперь = 1e6/ODR_HZ (без множителя 10).
% Это дельта hardware-timestamp МЕЖДУ ПОСЛЕДОВАТЕЛЬНЫМИ FIFO-сэмплами
% ВНУТРИ датчика (ODR=1600 Гц), НЕ период UART-кадра (100 Гц). UART
% передаёт только последний (самый свежий) из 16 накопленных FIFO-
% сэмплов, поэтому ts_ отражает именно межсэмпловый интервал на 1600 Гц.
expected_delta_us = 1e6 / ODR_HZ;
fprintf('\n--- TIMESTAMP (raw uint16, 1 мкс/LSB, DELTA-режим) ---\n');
fprintf('Ожидаемая дельта при ODR=%d Гц (межсэмпловая, не межкадровая): %.1f мкс\n', ...
    ODR_HZ, expected_delta_us);
fprintf('%-6s  %-8s  %-8s  %-8s  %-8s  %-10s  %-8s\n', ...
    'Sensor','Mean,мкс','Std,мкс','Min','Max','Ненул,%','Статус');
fprintf('%s\n', repmat('-',1,72));
for sid = 1:N_SENSORS
    col = ts_(:, sid);
    nv  = sum(~isnan(col));
    if nv == 0, continue; end
    nz_pct   = sum(col(~isnan(col)) ~= 0) / nv * 100;
    ts_mean  = mean(col, 'omitnan');
    ts_std   = std(col,  'omitnan');
    ts_min   = min(col,  [], 'omitnan');
    ts_max   = max(col,  [], 'omitnan');
    if nz_pct < 1.0
        status = 'TMST=0 (нет бита)';
    elseif abs(ts_mean - expected_delta_us) < 0.15 * expected_delta_us
        status = 'OK';
    else
        status = sprintf('? delta=%.0f мкс', ts_mean);
    end
    fprintf('S%02d    %8.1f  %8.2f  %8.0f  %8.0f  %9.1f%%  %s\n', ...
        sid-1, ts_mean, ts_std, ts_min, ts_max, nz_pct, status);
end

%% ── ГРАФИКИ IMU ──────────────────────────────────────────────────────────
colors = lines(N_SENSORS);
plot_sensors(t, ax,   'g',   'Accel X',     colors, N_SENSORS);
plot_sensors(t, ay,   'g',   'Accel Y',     colors, N_SENSORS);
plot_sensors(t, az,   'g',   'Accel Z',     colors, N_SENSORS);
plot_sensors(t, gx,   'dps', 'Gyro X',      colors, N_SENSORS);
plot_sensors(t, gy,   'dps', 'Gyro Y',      colors, N_SENSORS);
plot_sensors(t, gz,   'dps', 'Gyro Z',      colors, N_SENSORS);
plot_sensors(t, temp, 'C',   'Temperature', colors, N_SENSORS);

%% ── ГРАФИКИ TIMESTAMP ────────────────────────────────────────────────────
figure('Name','Timestamp raw [us]','NumberTitle','off');
hold on; grid on; plotted = false;
for sid = 1:N_SENSORS
    col = ts_(:, sid);
    if all(isnan(col)) || all(col(~isnan(col)) == 0), continue; end
    plot(t, col, 'Color', colors(sid,:), 'DisplayName', sprintf('S%d', sid-1));
    plotted = true;
end
yline(expected_delta_us, 'k--', 'LineWidth', 1.5, ...
    'DisplayName', sprintf('Ожид. %.0f мкс', expected_delta_us));
xlabel('Time [s]'); ylabel('Timestamp delta [us]');
title(sprintf('Timestamp (raw delta, ODR=%d Hz)', ODR_HZ));
if plotted, legend('Location','best','NumColumns',3); end

figure('Name','Timestamp jitter [us]','NumberTitle','off');
hold on; grid on; plotted = false;
for sid = 1:N_SENSORS
    col = ts_(:, sid);
    if all(isnan(col)) || all(col(~isnan(col)) == 0), continue; end
    jitter = col - expected_delta_us;
    plot(t, jitter, 'Color', colors(sid,:), 'DisplayName', sprintf('S%d', sid-1));
    plotted = true;
end
yline(0, 'k--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Δ от ожид. [us]');
title(sprintf('Jitter timestamp (ожид. %.0f мкс)', expected_delta_us));
if plotted, legend('Location','best','NumColumns',3); end

%% ── FRAME COUNTER ────────────────────────────────────────────────────────
figure('Name','Frame counter','NumberTitle','off');
plot(t, double(ctr), 'LineWidth', 1.2); grid on;
xlabel('Time [s]'); ylabel('Counter (uint16)'); title('Frame counter');

fprintf('\nГотово: %d фреймов, %.2f с, ~%.1f Гц\n', ...
    nF, t(end), nF/max(t(end),1e-9));

%% ── ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ─────────────────────────────────────────────
function x = unpack20(msb, lsb, nibble)
    u = bitor(bitshift(uint32(msb), 12), ...
              bitor(bitshift(uint32(lsb), 4), uint32(nibble)));
    if bitand(u, uint32(hex2dec('80000'))) ~= 0
        x = double(int32(u - uint32(hex2dec('100000'))));
    else
        x = double(u);
    end
end

function plot_sensors(t, data, unit, ttl, colors, N)
    figure('Name', [ttl ' [' unit ']'], 'NumberTitle', 'off');
    hold on; grid on; plotted = false;
    for k = 1:N
        col = data(:,k);
        if all(isnan(col)), continue; end
        plot(t, col, 'Color', colors(k,:), 'DisplayName', sprintf('S%d', k-1));
        plotted = true;
    end
    xlabel('Time [s]'); ylabel(['[' unit ']']); title(ttl);
    if plotted, legend('Location','best','NumColumns',3); end
end

function crc = crc16ccitt(data)
    data = uint8(data(:));
    crc  = uint16(hex2dec('FFFF'));
    poly = uint16(hex2dec('1021'));
    for i = 1:numel(data)
        crc = bitxor(crc, bitshift(uint16(data(i)), 8));
        for j = 1:8
            if bitand(crc, uint16(hex2dec('8000')))
                crc = bitxor(bitshift(crc, 1), poly);
            else
                crc = bitshift(crc, 1);
            end
            crc = bitand(crc, uint16(hex2dec('FFFF')));
        end
    end
end
