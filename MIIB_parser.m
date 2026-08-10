% =========================================================================
% MIIB_parser.m  —  ICM-45686 UART телеметрия  v4
% v4: добавлен анализ и графики timestamp по каждому датчику
% =========================================================================
close all; clearvars; clc;

%% ── CONFIG ───────────────────────────────────────────────────────────────
PORT        = 'COM3';
BAUD        = 8000000;
BUF_SIZE    = 32 * 1024 * 1024;
TIMEOUT_S   = 20;
N_SENSORS   = 18;
PKT_TOTAL   = 518;
FRAME_HZ    = 1600;
SENS_A      = 16384.0;
SENS_G      = 131.072;
ODR_HZ      = 1600;   % ODR датчика → ожидаемая дельта timestamp

%% ── ОТКРЫТИЕ ПОРТА ───────────────────────────────────────────────────────
s = serialport(PORT, BAUD, 'Timeout', TIMEOUT_S + 2);
s.InputBufferSize = BUF_SIZE;
flush(s);
fprintf('Listening on %s @ %d baud for %d s...\n', PORT, BAUD, TIMEOUT_S);

%% ── ЗАХВАТ ───────────────────────────────────────────────────────────────
raw   = uint8([]);
t_cap = tic;
while toc(t_cap) < TIMEOUT_S
    n = s.NumBytesAvailable;
    if n >= PKT_TOTAL
        chunk = read(s, n, 'uint8');
        raw   = [raw; chunk(:)]; %#ok<AGROW>
    end
    pause(0.005);
end
delete(s);

expected_bytes = round(FRAME_HZ * TIMEOUT_S * PKT_TOTAL);
fprintf('Received %d bytes (expected ~%d = %.1f%%)\n', ...
    numel(raw), expected_bytes, numel(raw)/expected_bytes*100);
if numel(raw) < PKT_TOTAL
    error('Слишком мало данных (%d байт).', numel(raw));
end

%% ── ДИАГНОСТИКА ──────────────────────────────────────────────────────────
fprintf('\n--- ДИАГНОСТИКА ---\n');
fprintf('0xAA в буфере: %d\n', sum(raw == uint8(hex2dec('AA'))));
fprintf('0x55 в буфере: %d\n', sum(raw == uint8(hex2dec('55'))));
fprintf('0x0D в буфере: %d\n', sum(raw == uint8(hex2dec('0D'))));
fprintf('0x0A в буфере: %d\n', sum(raw == uint8(hex2dec('0A'))));
fprintf('First 40 bytes: ');
fprintf('%02X ', raw(1:min(40,end)));
fprintf('\n\n');

%% ── ПАРСИНГ ──────────────────────────────────────────────────────────────
frames     = struct('counter',{},'mask',{},'samples',{});
idx        = 1;
n_raw      = numel(raw);
bad_crc    = 0;
bad_footer = 0;
hdr_hits   = 0;

while idx <= n_raw - PKT_TOTAL + 1
    rel = find(raw(idx:end-1) == uint8(hex2dec('AA')) & ...
               raw(idx+1:end) == uint8(hex2dec('55')), 1);
    if isempty(rel), break; end
    idx = idx + rel - 1;
    if idx + PKT_TOTAL - 1 > n_raw, break; end

    hdr_hits = hdr_hits + 1;
    pkt = raw(idx : idx + PKT_TOTAL - 1);

    if pkt(517) ~= uint8(hex2dec('0D')) || pkt(518) ~= uint8(hex2dec('0A'))
        bad_footer = bad_footer + 1;
        idx = idx + 1;
        continue;
    end

    payload  = pkt(3:514);
    crc_calc = crc16ccitt(payload);
    crc_rx   = bitor(uint16(pkt(515)), bitshift(uint16(pkt(516)), 8));
    if crc_calc ~= crc_rx
        bad_crc = bad_crc + 1;
        idx = idx + 1;
        continue;
    end

    f.counter = typecast(uint8(payload(1:4)), 'uint32');
    f.mask    = typecast(uint8(payload(5:8)), 'uint32');
    samp = nan(N_SENSORS, 8);

    for sid = 1:N_SENSORS
        if ~bitget(f.mask, sid), continue; end
        b    = 9 + (sid-1)*28;
        ax_r = typecast(uint8(payload(b     : b+3 )), 'int32');
        ay_r = typecast(uint8(payload(b+4   : b+7 )), 'int32');
        az_r = typecast(uint8(payload(b+8   : b+11)), 'int32');
        gx_r = typecast(uint8(payload(b+12  : b+15)), 'int32');
        gy_r = typecast(uint8(payload(b+16  : b+19)), 'int32');
        gz_r = typecast(uint8(payload(b+20  : b+23)), 'int32');
        tr   = typecast(uint8(payload(b+24  : b+25)), 'int16');
        ts   = typecast(uint8(payload(b+26  : b+27)), 'uint16');
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
    idx = idx + PKT_TOTAL;
end

fprintf('Header hits  : %d\n', hdr_hits);
fprintf('Parsed frames: %d\n', numel(frames));
fprintf('Bad CRC      : %d\n', bad_crc);
fprintf('Bad footer   : %d\n', bad_footer);

if isempty(frames)
    fprintf('\n[HINT] Нет валидных пакетов.\n');
    if bad_crc    > 0, fprintf('  → Bad CRC.\n'); end
    if bad_footer > 0, fprintf('  → Bad footer.\n'); end
    if hdr_hits  == 0, fprintf('  → Нет 0xAA 0x55: baud=%d?\n', BAUD); end
    error('No valid frames.');
end

%% ── СБОРКА МАТРИЦ ────────────────────────────────────────────────────────
nF   = numel(frames);
t    = (0:nF-1).' / FRAME_HZ;

ax   = nan(nF, N_SENSORS);  ay   = nan(nF, N_SENSORS);
az   = nan(nF, N_SENSORS);  gx   = nan(nF, N_SENSORS);
gy   = nan(nF, N_SENSORS);  gz   = nan(nF, N_SENSORS);
temp = nan(nF, N_SENSORS);  ts_  = nan(nF, N_SENSORS);
mask = zeros(nF, 1, 'uint32');
ctr  = zeros(nF, 1, 'uint32');

for fi = 1:nF
    ax(fi,:)   = frames(fi).samples(:,1).';
    ay(fi,:)   = frames(fi).samples(:,2).';
    az(fi,:)   = frames(fi).samples(:,3).';
    gx(fi,:)   = frames(fi).samples(:,4).';
    gy(fi,:)   = frames(fi).samples(:,5).';
    gz(fi,:)   = frames(fi).samples(:,6).';
    temp(fi,:) = frames(fi).samples(:,7).';
    ts_(fi,:)  = frames(fi).samples(:,8).';
    mask(fi)   = frames(fi).mask;
    ctr(fi)    = frames(fi).counter;
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

%% ── ДИАГНОСТИКА МАСКИ ────────────────────────────────────────────────────
fprintf('\nSensor mask (frame 1): 0x%08X\n', frames(1).mask);
fprintf('Valid sensors  : %s\n', mat2str(find(bitget(frames(1).mask, 1:N_SENSORS))-1));

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
% ts_ содержит сырое uint16 значение (дельта, 1 мкс/LSB при TMST_RESOL=0).
% Ожидаемая дельта при ODR 6400 Гц и FDR=0 (нет децимации):
%   10 семплов в батче, берём последний → дельта = 10 * (1e6/6400) ≈ 1562 мкс.
% Если TMST_DELTA_EN=0 или FIFO_HDR_TMST_BIT не выставлен — все значения 0.

expected_delta_us = 10 * 1e6 / ODR_HZ;   % мкс между UART-пакетами

fprintf('\n--- TIMESTAMP (raw uint16, 1 мкс/LSB, DELTA-режим) ---\n');
fprintf('Ожидаемая дельта при ODR=%d Гц, 10 сэмплов/батч: %.1f мкс\n', ...
    ODR_HZ, expected_delta_us);
fprintf('%-6s  %-8s  %-8s  %-8s  %-8s  %-10s  %-8s\n', ...
    'Sensor','Mean,мкс','Std,мкс','Min','Max','Ненул,%','Статус');
fprintf('%s\n', repmat('-',1,72));

ts_status = cell(N_SENSORS, 1);
for sid = 1:N_SENSORS
    col = ts_(:, sid);
    nv  = sum(~isnan(col));
    if nv == 0
        ts_status{sid} = 'нет данных';
        continue;
    end

    nz_pct   = sum(col(~isnan(col)) ~= 0) / nv * 100;
    ts_mean  = mean(col, 'omitnan');
    ts_std   = std(col,  'omitnan');
    ts_min   = min(col,  [], 'omitnan');
    ts_max   = max(col,  [], 'omitnan');

    % Определяем статус
    if nz_pct < 1.0
        status = '⚠ TMST=0 (нет бита)';
    elseif abs(ts_mean - expected_delta_us) < 0.15 * expected_delta_us
        status = '✓ OK';
    else
        status = sprintf('? delta=%.0f мкс', ts_mean);
    end
    ts_status{sid} = status;

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
plot_sensors(t, temp, '°C',  'Temperature', colors, N_SENSORS);

%% ── ГРАФИКИ TIMESTAMP ────────────────────────────────────────────────────
% График 1: сырой timestamp (дельта) по каждому датчику
figure('Name','Timestamp raw [µs]','NumberTitle','off');
hold on; grid on;
plotted = false;
for sid = 1:N_SENSORS
    col = ts_(:, sid);
    if all(isnan(col)) || all(col(~isnan(col)) == 0), continue; end
    plot(t, col, 'Color', colors(sid,:), 'DisplayName', sprintf('S%d', sid-1));
    plotted = true;
end
yline(expected_delta_us, 'k--', 'LineWidth', 1.5, ...
    'DisplayName', sprintf('Ожид. %.0f мкс', expected_delta_us));
xlabel('Time [s]'); ylabel('Timestamp delta [µs]');
title(sprintf('Timestamp (raw delta, ODR=%d Hz)', ODR_HZ));
if plotted, legend('Location','best','NumColumns',3); end

% График 2: накопленное абсолютное время по каждому датчику (cumsum дельт)
% Позволяет увидеть расхождение часов между датчиками
figure('Name','Timestamp cumsum [ms]','NumberTitle','off');
hold on; grid on;
plotted = false;
for sid = 1:N_SENSORS
    col = ts_(:, sid);
    if all(isnan(col)) || all(col(~isnan(col)) == 0), continue; end
    % Заполняем NaN предыдущим значением для непрерывного cumsum
    col_filled = col;
    for fi = 2:nF
        if isnan(col_filled(fi)), col_filled(fi) = col_filled(fi-1); end
    end
    if isnan(col_filled(1)), col_filled(1) = 0; end
    t_abs = cumsum(col_filled) / 1000;   % мкс → мс
    plot(t, t_abs, 'Color', colors(sid,:), 'DisplayName', sprintf('S%d', sid-1));
    plotted = true;
end
xlabel('Time [s]'); ylabel('Cumulative timestamp [ms]');
title('Накопленное время по датчикам (расхождение = джиттер/пропуски)');
if plotted, legend('Location','best','NumColumns',3); end

% График 3: отклонение дельты от ожидаемого значения [мкс] — джиттер
figure('Name','Timestamp jitter [µs]','NumberTitle','off');
hold on; grid on;
plotted = false;
for sid = 1:N_SENSORS
    col = ts_(:, sid);
    if all(isnan(col)) || all(col(~isnan(col)) == 0), continue; end
    jitter = col - expected_delta_us;
    plot(t, jitter, 'Color', colors(sid,:), 'DisplayName', sprintf('S%d', sid-1));
    plotted = true;
end
yline(0, 'k--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Δ от ожид. [µs]');
title(sprintf('Jitter timestamp (0 = идеально, ожид. %.0f мкс)', expected_delta_us));
if plotted, legend('Location','best','NumColumns',3); end

% График 4: heatmap — медианное значение ts по каждому датчику
figure('Name','Timestamp median heatmap','NumberTitle','off');
ts_med = zeros(1, N_SENSORS);
for sid = 1:N_SENSORS
    col = ts_(:, sid);
    if all(isnan(col)), ts_med(sid) = NaN; continue; end
    ts_med(sid) = median(col, 'omitnan');
end
bar(0:N_SENSORS-1, ts_med, 'FaceColor', [0.2 0.6 0.9]);
hold on;
yline(expected_delta_us, 'r--', 'LineWidth', 1.5, ...
    'DisplayName', sprintf('Ожид. %.0f мкс', expected_delta_us));
xlabel('Sensor ID'); ylabel('Median timestamp delta [µs]');
title('Медианная дельта timestamp по датчикам');
xticks(0:N_SENSORS-1); grid on;
legend('show');

%% ── РАСТРОВАЯ КАРТА МАСКИ + СЧЁТЧИК ──────────────────────────────────────
figure('Name','Sensor mask','NumberTitle','off');
bits = zeros(N_SENSORS, nF);
for fi = 1:nF
    for sid = 0:N_SENSORS-1
        bits(sid+1,fi) = bitget(mask(fi), sid+1);
    end
end
imagesc(t.', 0:N_SENSORS-1, bits);
colormap([0.85 0.2 0.2; 0.2 0.8 0.2]);
colorbar('Ticks',[0 1],'TickLabels',{'missing','valid'});
xlabel('Time [s]'); ylabel('Sensor ID');
title('Sensor mask  (green = valid, red = missing)');
yticks(0:N_SENSORS-1); grid on;

figure('Name','Frame counter','NumberTitle','off');
plot(t, double(ctr), 'LineWidth', 1.2); grid on;
xlabel('Time [s]'); ylabel('Counter'); title('Frame counter');

dctr    = diff(double(ctr));
n_drops = sum(dctr ~= 1);
if n_drops > 0
    fprintf('\n[WARNING] %d скачков счётчика.\n', n_drops);
    drop_t = t(find(dctr ~= 1) + 1);
    fprintf('  Моменты (с): ');
    fprintf('%.3f ', drop_t(1:min(20,end)));
    fprintf('\n');
else
    fprintf('\nФреймы непрерывны (счётчик монотонно +1).\n');
end
fprintf('\nГотово: %d фреймов, %.2f с, ~%.1f Гц\n', ...
    nF, t(end), nF/max(t(end),1e-9));

%% ── ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ─────────────────────────────────────────────
function plot_sensors(t, data, unit, ttl, colors, N)
    figure('Name', [ttl ' [' unit ']'], 'NumberTitle', 'off');
    hold on; grid on;
    plotted = false;
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