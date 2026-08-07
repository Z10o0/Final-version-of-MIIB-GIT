% =========================================================================
% MIIB_parser.m  —  ICM-45686 UART телеметрия, парсер с надёжным захватом
%
% Структура пакета (518 байт, 1-based MATLAB):
%   [1..2]     0xAA 0x55           заголовок
%   [3..514]   ICM_SyncFrame_t     payload (512 байт)
%                [3..6]   frame_counter  uint32 LE
%                [7..10]  sensor_mask    uint32 LE
%                [11..514] samples[18]  ICM_Sample_t × 18 (28 байт каждый)
%   [515..516] CRC16 CCITT LE      по байтам [3..514]
%   [517]      0x0D  (CR)
%   [518]      0x0A  (LF)
%
% ICM_Sample_t (28 байт):
%   accel_x/y/z  int32 LE  (raw >> 12) / 32768 * 16  → [g]
%   gyro_x/y/z   int32 LE  (raw >> 12) / 32768 * 2000 → [dps]
%   temp_raw     int16 LE  val/2 + 25                 → [°C]
%   timestamp    uint16 LE                             → [µs]
% =========================================================================

close all; clearvars; clc;

%% ── CONFIG ───────────────────────────────────────────────────────────────
PORT        = 'COM3';
BAUD        = 5000000;
BUF_SIZE    = 8 * 1024 * 1024;   % 8 МБ — обязательно для 5 Мбит/с
TIMEOUT_S   = 10;

N_SENSORS   = 18;
PKT_TOTAL   = 518;
SCALE_A     = 16.0   / 32768.0;
SCALE_G     = 2000.0 / 32768.0;
FRAME_HZ    = 640;

%% ── ОТКРЫТИЕ ПОРТА ───────────────────────────────────────────────────────
s = serialport(PORT, BAUD, 'Timeout', TIMEOUT_S + 2);
s.InputBufferSize = BUF_SIZE;     % ← КЛЮЧЕВОЕ: 8 МБ вместо дефолтных 64 КБ
flush(s);
fprintf('Listening on %s @ %d baud for %d s...\n', PORT, BAUD, TIMEOUT_S);

%% ── ЗАХВАТ: пауза + одно чтение (нет polling — нет потерь) ───────────────
% Захват небольшого куска для анализа
pause(0.5);
n = s.NumBytesAvailable;
probe = read(s, n, 'uint8');
fprintf('Probe %d bytes: ', numel(probe));
fprintf('%02X ', probe(1:min(60,end)));
fprintf('\n');




pause(TIMEOUT_S);
n_avail = s.NumBytesAvailable;
if n_avail > 0
    raw = read(s, n_avail, 'uint8');
    raw = raw(:);
else
    raw = uint8([]);
end
delete(s);
fprintf('Received %d bytes, parsing...\n', numel(raw));

if numel(raw) < PKT_TOTAL
    error('Слишком мало данных (%d байт). Проверьте соединение.', numel(raw));
end

%% ── ДИАГНОСТИКА ──────────────────────────────────────────────────────────
fprintf('\n--- ДИАГНОСТИКА ---\n');
fprintf('0xAA в буфере: %d\n',  sum(raw == uint8(hex2dec('AA'))));
fprintf('0x55 в буфере: %d\n',  sum(raw == uint8(hex2dec('55'))));
fprintf('0x0D в буфере: %d\n',  sum(raw == uint8(hex2dec('0D'))));
fprintf('0x0A в буфере: %d\n',  sum(raw == uint8(hex2dec('0A'))));
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

    % Поиск заголовка AA 55
    rel = find(raw(idx:end-1) == uint8(hex2dec('AA')) & ...
               raw(idx+1:end) == uint8(hex2dec('55')), 1, 'first');
    if isempty(rel), break; end
    idx = idx + rel - 1;

    if idx + PKT_TOTAL - 1 > n_raw, break; end

    hdr_hits = hdr_hits + 1;
    pkt = raw(idx : idx + PKT_TOTAL - 1);

    % Footer
    if pkt(517) ~= uint8(hex2dec('0D')) || pkt(518) ~= uint8(hex2dec('0A'))
        bad_footer = bad_footer + 1;
        idx = idx + 1;
        continue;
    end

    % CRC16 CCITT по payload pkt(3..514)
    payload  = pkt(3:514);
    crc_calc = crc16ccitt(payload);
    crc_rx   = bitor(uint16(pkt(515)), bitshift(uint16(pkt(516)), 8));

    if crc_calc ~= crc_rx
        bad_crc = bad_crc + 1;
        idx = idx + 1;
        continue;
    end

    % Распаковка
    f.counter = typecast(uint8(payload(1:4)),  'uint32');
    f.mask    = typecast(uint8(payload(5:8)),  'uint32');

    samp = zeros(N_SENSORS, 8);
    for sid = 1:N_SENSORS
        b  = 9 + (sid-1)*28;
        ax = typecast(uint8(payload(b     : b+3 )), 'int32');
        ay = typecast(uint8(payload(b+4   : b+7 )), 'int32');
        az = typecast(uint8(payload(b+8   : b+11)), 'int32');
        gx = typecast(uint8(payload(b+12  : b+15)), 'int32');
        gy = typecast(uint8(payload(b+16  : b+19)), 'int32');
        gz = typecast(uint8(payload(b+20  : b+23)), 'int32');
        tr = typecast(uint8(payload(b+24  : b+25)), 'int16');
        ts = typecast(uint8(payload(b+26  : b+27)), 'uint16');

        samp(sid,:) = [
            double(ax)/4096 * SCALE_A, ...
            double(ay)/4096 * SCALE_A, ...
            double(az)/4096 * SCALE_A, ...
            double(gx)/4096 * SCALE_G, ...
            double(gy)/4096 * SCALE_G, ...
            double(gz)/4096 * SCALE_G, ...
            double(tr)/2 + 25, ...
            double(ts) ];
    end
    f.samples = samp;
    frames(end+1) = f; %#ok<AGROW>
    idx = idx + PKT_TOTAL;    % жёсткий прыжок к следующему пакету
end

fprintf('Header hits  : %d\n', hdr_hits);
fprintf('Parsed frames: %d\n', numel(frames));
fprintf('Bad CRC      : %d\n', bad_crc);
fprintf('Bad footer   : %d\n', bad_footer);

if isempty(frames)
    fprintf('\n[HINT] Нет валидных пакетов.\n');
    if bad_crc > 0
        fprintf('  → Bad CRC > 0: InputBufferSize переполнился — поток разорван.\n');
    end
    if bad_footer > 0
        fprintf('  → Bad footer: 0D/0A не на байтах 517/518.\n');
    end
    if hdr_hits == 0
        fprintf('  → Нет 0xAA 0x55: проверьте baud rate = %d и соединение.\n', BAUD);
    end
    error('No valid frames.');
end

%% ── СБОРКА МАТРИЦ ────────────────────────────────────────────────────────
nF   = numel(frames);
t    = (0:nF-1).' / FRAME_HZ;

ax   = zeros(nF, N_SENSORS);  ay   = zeros(nF, N_SENSORS);
az   = zeros(nF, N_SENSORS);  gx   = zeros(nF, N_SENSORS);
gy   = zeros(nF, N_SENSORS);  gz   = zeros(nF, N_SENSORS);
temp = zeros(nF, N_SENSORS);  ts_  = zeros(nF, N_SENSORS);
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

%% ── ГРАФИКИ ──────────────────────────────────────────────────────────────
colors = lines(N_SENSORS);

plot_sensors(t, ax,   'g',   'Accel X',     colors, N_SENSORS);
plot_sensors(t, ay,   'g',   'Accel Y',     colors, N_SENSORS);
plot_sensors(t, az,   'g',   'Accel Z',     colors, N_SENSORS);
plot_sensors(t, gx,   'dps', 'Gyro X',      colors, N_SENSORS);
plot_sensors(t, gy,   'dps', 'Gyro Y',      colors, N_SENSORS);
plot_sensors(t, gz,   'dps', 'Gyro Z',      colors, N_SENSORS);
plot_sensors(t, temp, '°C',  'Temperature', colors, N_SENSORS);

% Растровая карта маски датчиков
figure('Name','Sensor mask','NumberTitle','off');
bits = zeros(N_SENSORS, nF);
for fi = 1:nF
    for sid = 0:N_SENSORS-1
        bits(sid+1, fi) = bitget(mask(fi), sid+1);
    end
end
imagesc(t.', 0:N_SENSORS-1, bits);
colormap([0.85 0.2 0.2; 0.2 0.8 0.2]);
colorbar('Ticks',[0 1],'TickLabels',{'missing','valid'});
xlabel('Time [s]'); ylabel('Sensor ID');
title('Sensor mask (green = valid, red = missing)');
yticks(0:N_SENSORS-1); grid on;

% Счётчик кадров
figure('Name','Frame counter','NumberTitle','off');
plot(t, double(ctr), 'LineWidth', 1.2); grid on;
xlabel('Time [s]'); ylabel('Counter'); title('Frame counter');

% Проверка непрерывности счётчика
dctr = diff(double(ctr));
n_drops = sum(dctr ~= 1);
if n_drops > 0
    fprintf('\n[WARNING] %d скачков счётчика (dropped frames на STM32).\n', n_drops);
else
    fprintf('\nФреймы непрерывны (счётчик монотонно +1).\n');
end
fprintf('\nГотово: %d фреймов, %.2f с, ~%.1f Гц\n', nF, t(end), nF/max(t(end),1e-9));

%% ── ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ─────────────────────────────────────────────
function plot_sensors(t, data, unit, ttl, colors, N)
    figure('Name', [ttl ' [' unit ']'], 'NumberTitle', 'off');
    hold on; grid on;
    for k = 1:N
        if any(data(:,k) ~= 0)
            plot(t, data(:,k), 'Color', colors(k,:), ...
                 'DisplayName', sprintf('S%d', k-1));
        end
    end
    xlabel('Time [s]'); ylabel(['[' unit ']']); title(ttl);
    legend('Location','best','NumColumns',3);
end

function crc = crc16ccitt(data)
    % CRC16-CCITT: poly=0x1021, init=0xFFFF
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