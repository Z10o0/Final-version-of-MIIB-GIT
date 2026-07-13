% =========================================================================
% MIIB_RealTime_Monitor.m
% Монитор реального времени для системы MIIB: 18×ICM-45686
% Приём пакетов по UART (5 500 000 бод), верификация, отображение.
%
% ИСПОЛЬЗОВАНИЕ:
%   1. Подключить STM32H723 по USB-UART (5.5 МБод).
%   2. Указать COM-порт в переменной UART_PORT ниже.
%   3. Запустить скрипт. Закрыть окно графика для остановки.
%
% ПАРАМЕТРЫ ПАКЕТА (должны совпадать с uart_telemetry.h):
%   SYNC     : AA 55 BB 44 (4 байта)
%   CNT      : uint32, монотонный счётчик (4 байта)
%   TS       : uint32, TIM7 тики (4 байта)
%   LEN      : uint16, размер payload = 2340 (2 байта)
%   DROPPED  : uint16, пропущенные пакеты (2 байта)
%   PAYLOAD  : 18 датчиков × 10 сэмплов × 14 байт = 2520 байт
%              [sensor_id(1) ax(2) ay(2) az(2) gx(2) gy(2) gz(2) temp(1)]
%   CRC32    : uint32 (4 байта)
%   ИТОГО    : 2534 байт — ВНИМАНИЕ: обновить если изменили конфиг!
%
% МАСШТАБИРУЮЩИЕ КОЭФФИЦИЕНТЫ (ICM-45686 FS=2000dps, FS=16g):
%   gyro_dps  = raw / 16.4
%   accel_g   = raw / 2048.0
%   temp_degC = raw / 2.07 + 25.0
% =========================================================================

clc; clear; close all;
fprintf('=== MIIB Real-Time Monitor v2.0 ===\n');

% -------------------------------------------------------------------------
%  ПАРАМЕТРЫ — ИЗМЕНЯТЬ ЗДЕСЬ
% -------------------------------------------------------------------------
UART_PORT      = 'COM3';          % Порт UART (Windows: 'COM3', Linux: '/dev/ttyUSB0')
BAUD_RATE      = 5500000;         % Бодрейт
DISPLAY_SENSORS = [1 2 3 4 5 6]; % Какие датчики показывать (1..18)
DISPLAY_AXIS   = 'gyro';         % 'gyro' или 'accel'
PLOT_HISTORY_S = 2.0;            % Сколько секунд истории держать на экране
MAX_PACKETS    = 100000;          % Лимит пакетов перед авто-стопом (inf = бесконечно)
SAVE_TO_FILE   = true;            % true = автосохранение в .mat файл
SAVE_FILENAME  = sprintf('MIIB_capture_%s.mat', datestr(now,'yyyymmdd_HHMMSS'));

% -------------------------------------------------------------------------
%  КОНСТАНТЫ ПРОТОКОЛА
% -------------------------------------------------------------------------
SYNC_MAGIC     = uint8([0xAA, 0x55, 0xBB, 0x44]);
HEADER_SIZE    = 16;              % байт (4+4+4+2+2)
N_SENSORS      = 18;
N_SAMPLES      = 10;
SAMPLE_SIZE    = 14;              % байт
PAYLOAD_SIZE   = N_SENSORS * N_SAMPLES * SAMPLE_SIZE; % = 2520
CRC_SIZE       = 4;
PACKET_SIZE    = HEADER_SIZE + PAYLOAD_SIZE + CRC_SIZE; % = 2540

% Масштабирование (FS=2000dps, FS=16g)
GYRO_SCALE     = 1.0 / 16.4;     % LSB → dps
ACCEL_SCALE    = 1.0 / 2048.0;   % LSB → g
TEMP_OFFSET    = 25.0;
TEMP_SCALE     = 1.0 / 2.07;

% -------------------------------------------------------------------------
%  ОТКРЫТИЕ UART
% -------------------------------------------------------------------------
fprintf('Открываю порт %s @ %d бод...\n', UART_PORT, BAUD_RATE);
s = serialport(UART_PORT, BAUD_RATE, ...
    'DataBits', 8, 'Parity', 'none', 'StopBits', 1, ...
    'FlowControl', 'none', 'Timeout', 5.0);
configureTerminator(s, 'CR/LF');
flush(s);
fprintf('Порт открыт. Ожидание данных...\n\n');

% -------------------------------------------------------------------------
%  БУФЕРЫ ДЛЯ ХРАНЕНИЯ ИСТОРИИ
% -------------------------------------------------------------------------
ODR            = 3200;            % Гц
SAMPLES_PER_PKT = N_SAMPLES;
PKT_RATE       = ODR / N_SAMPLES; % 320 пакетов/сек
HIST_PACKETS   = ceil(PLOT_HISTORY_S * PKT_RATE);
HIST_SAMPLES   = HIST_PACKETS * SAMPLES_PER_PKT;

% Буфер: [время × датчик × ось]
hist_time      = nan(HIST_SAMPLES, 1);
hist_gyro      = nan(HIST_SAMPLES, N_SENSORS, 3);  % [N, sensor, xyz]
hist_accel     = nan(HIST_SAMPLES, N_SENSORS, 3);
hist_temp      = nan(HIST_SAMPLES, N_SENSORS);
buf_write_idx  = 1;  % Указатель записи (циклический буфер)

% Статистика
stats.total_packets  = 0;
stats.crc_errors     = 0;
stats.sync_errors    = 0;
stats.total_dropped  = 0;
stats.prev_cnt       = uint32(0);

% Буфер для сохранения (динамический)
if SAVE_TO_FILE
    save_buf.packets  = uint32(zeros(MAX_PACKETS, 1));
    save_buf.ts       = uint32(zeros(MAX_PACKETS, 1));
    save_buf.dropped  = uint16(zeros(MAX_PACKETS, 1));
    save_buf.gyro_raw = int16(zeros(MAX_PACKETS, N_SENSORS, N_SAMPLES, 3));
    save_buf.accel_raw= int16(zeros(MAX_PACKETS, N_SENSORS, N_SAMPLES, 3));
    save_buf.temp_raw = int8(zeros(MAX_PACKETS, N_SENSORS, N_SAMPLES));
    save_buf.write_idx = 1;
end

% -------------------------------------------------------------------------
%  СОЗДАНИЕ ИНТЕРФЕЙСА ГРАФИКОВ
% -------------------------------------------------------------------------
N_DISP      = numel(DISPLAY_SENSORS);
fig         = figure('Name', 'MIIB Real-Time Monitor', ...
                     'NumberTitle', 'off', ...
                     'Color', [0.1 0.1 0.15], ...
                     'Position', [50 50 1800 950]);

% Сетка осей: N_DISP строк × 4 колонки (X, Y, Z, Temp)
tile = tiledlayout(N_DISP, 4, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tile, 'MIIB — 18×ICM-45686 Real-Time Monitor', ...
      'Color', [0.9 0.9 0.9], 'FontSize', 14, 'FontWeight', 'bold');

ax       = gobjects(N_DISP, 4);
ln       = gobjects(N_DISP, 4);
AXIS_LABELS = {'X', 'Y', 'Z', 'Temp'};
COLORS = lines(N_DISP);

for row = 1:N_DISP
    sid = DISPLAY_SENSORS(row);
    for col = 1:4
        ax(row, col) = nexttile(tile);
        ax(row, col).Color           = [0.08 0.08 0.12];
        ax(row, col).XColor          = [0.6 0.6 0.6];
        ax(row, col).YColor          = [0.6 0.6 0.6];
        ax(row, col).GridColor       = [0.3 0.3 0.3];
        ax(row, col).GridAlpha       = 0.4;
        ax(row, col).MinorGridAlpha  = 0.2;
        grid(ax(row, col), 'on');
        hold(ax(row, col), 'on');
        ln(row, col) = plot(ax(row, col), nan, nan, ...
            'Color', COLORS(row,:), 'LineWidth', 1.2);
        if col < 4
            if strcmpi(DISPLAY_AXIS, 'gyro')
                ylabel(ax(row,col), '[dps]', 'Color', [0.7 0.7 0.7], 'FontSize', 8);
            else
                ylabel(ax(row,col), '[g]', 'Color', [0.7 0.7 0.7], 'FontSize', 8);
            end
            ylim(ax(row,col), [-2100 2100]);
        else
            ylabel(ax(row,col), '[°C]', 'Color', [0.7 0.7 0.7], 'FontSize', 8);
            ylim(ax(row,col), [15 45]);
        end
        xlabel(ax(row,col), 'Время [с]', 'Color', [0.7 0.7 0.7], 'FontSize', 8);
        title(ax(row,col), sprintf('Датч.%d — %s', sid, AXIS_LABELS{col}), ...
              'Color', COLORS(row,:), 'FontSize', 9, 'FontWeight', 'bold');
    end
end

% Текстовая панель статистики
ax_stat = axes(fig, 'Position', [0.0 0.0 1.0 0.025]);
ax_stat.Visible = 'off';
stat_text = text(ax_stat, 0.01, 0.5, 'Инициализация...', ...
    'Color', [0.8 0.8 0.3], 'FontSize', 9, 'FontName', 'Courier New', ...
    'VerticalAlignment', 'middle');

drawnow;

% -------------------------------------------------------------------------
%  ГЛАВНЫЙ ЦИКЛ ПРИЁМА
% -------------------------------------------------------------------------
raw_buf    = uint8([]);  % Накопительный буфер из UART
t_start    = tic;
last_draw  = tic;
DRAW_INTERVAL_S = 0.05;  % Обновление графиков каждые 50 мс

fprintf('Сбор данных запущен. Закройте окно для остановки.\n');

while ishandle(fig) && stats.total_packets < MAX_PACKETS

    % --- Приём байт из UART ---
    n_avail = s.NumBytesAvailable;
    if n_avail > 0
        raw_buf = [raw_buf; read(s, n_avail, 'uint8')]; %#ok<AGROW>
    else
        pause(0.001);
        continue;
    end

    % --- Обработка всех полных пакетов в буфере ---
    while numel(raw_buf) >= PACKET_SIZE

        % Поиск синхрослова
        sync_pos = find_sync(raw_buf, SYNC_MAGIC);

        if isempty(sync_pos)
            % Нет синхрослова — сбрасываем всё кроме последних 3 байт
            stats.sync_errors = stats.sync_errors + max(0, numel(raw_buf) - 3);
            raw_buf = raw_buf(max(1, end-2):end);
            break;
        end

        if sync_pos > 1
            % Мусор перед синхрословом
            stats.sync_errors = stats.sync_errors + (sync_pos - 1);
            raw_buf = raw_buf(sync_pos:end);
        end

        if numel(raw_buf) < PACKET_SIZE
            break;  % Пакет ещё не полный — ждём ещё байт
        end

        pkt = raw_buf(1:PACKET_SIZE);

        % --- Разбор заголовка ---
        pkt_cnt  = typecast(pkt(5:8),   'uint32');
        pkt_ts   = typecast(pkt(9:12),  'uint32');
        pkt_len  = typecast(pkt(13:14), 'uint16');
        pkt_drop = typecast(pkt(15:16), 'uint16');

        % --- Проверка CRC32 ---
        crc_recv = typecast(pkt(HEADER_SIZE+PAYLOAD_SIZE+1 : end), 'uint32');
        crc_calc = compute_crc32(pkt(1:HEADER_SIZE+PAYLOAD_SIZE));

        if crc_recv ~= crc_calc
            stats.crc_errors = stats.crc_errors + 1;
            % Сдвигаемся на 1 байт для поиска следующего пакета
            raw_buf = raw_buf(2:end);
            continue;
        end

        % --- Пакет корректен ---
        stats.total_packets = stats.total_packets + 1;
        stats.total_dropped = stats.total_dropped + double(pkt_drop);

        % --- Разбор payload ---
        payload = pkt(HEADER_SIZE+1 : HEADER_SIZE+PAYLOAD_SIZE);
        t_pkt   = double(pkt_ts) / 1e6;  % тики → секунды (TIM7 1 МГц)

        for s_idx = 1:N_SENSORS
            for k = 1:N_SAMPLES
                off = ((s_idx-1)*N_SAMPLES + (k-1)) * SAMPLE_SIZE + 1;
                % Байт 1: sensor_id (игнорируем — уже знаем порядок)
                ax_raw = typecast(payload(off+1:off+2),   'int16');
                ay_raw = typecast(payload(off+3:off+4),   'int16');
                az_raw = typecast(payload(off+5:off+6),   'int16');
                gx_raw = typecast(payload(off+7:off+8),   'int16');
                gy_raw = typecast(payload(off+9:off+10),  'int16');
                gz_raw = typecast(payload(off+11:off+12), 'int16');
                t_raw  = typecast(payload(off+13),        'int8');

                % Вычисление времени для этого сэмпла:
                % линейная интерполяция внутри пакета
                t_sample = t_pkt - (N_SAMPLES - k) / ODR;

                % Запись в циклический буфер
                idx = buf_write_idx;
                hist_time(idx)          = t_sample;
                hist_gyro(idx, s_idx,1) = double(gx_raw) * GYRO_SCALE;
                hist_gyro(idx, s_idx,2) = double(gy_raw) * GYRO_SCALE;
                hist_gyro(idx, s_idx,3) = double(gz_raw) * GYRO_SCALE;
                hist_accel(idx,s_idx,1) = double(ax_raw) * ACCEL_SCALE;
                hist_accel(idx,s_idx,2) = double(ay_raw) * ACCEL_SCALE;
                hist_accel(idx,s_idx,3) = double(az_raw) * ACCEL_SCALE;
                hist_temp(idx, s_idx)   = double(t_raw) * TEMP_SCALE + TEMP_OFFSET;

                buf_write_idx = mod(buf_write_idx, HIST_SAMPLES) + 1;
            end
        end

        % --- Сохранение в буфер ---
        if SAVE_TO_FILE && save_buf.write_idx <= MAX_PACKETS
            wi = save_buf.write_idx;
            save_buf.packets(wi) = pkt_cnt;
            save_buf.ts(wi)      = pkt_ts;
            save_buf.dropped(wi) = pkt_drop;
            for s_idx = 1:N_SENSORS
                for k = 1:N_SAMPLES
                    off = ((s_idx-1)*N_SAMPLES + (k-1)) * SAMPLE_SIZE + 1;
                    save_buf.gyro_raw(wi, s_idx, k, 1) = typecast(payload(off+7:off+8),  'int16');
                    save_buf.gyro_raw(wi, s_idx, k, 2) = typecast(payload(off+9:off+10), 'int16');
                    save_buf.gyro_raw(wi, s_idx, k, 3) = typecast(payload(off+11:off+12),'int16');
                    save_buf.accel_raw(wi,s_idx, k, 1) = typecast(payload(off+1:off+2),  'int16');
                    save_buf.accel_raw(wi,s_idx, k, 2) = typecast(payload(off+3:off+4),  'int16');
                    save_buf.accel_raw(wi,s_idx, k, 3) = typecast(payload(off+5:off+6),  'int16');
                    save_buf.temp_raw(wi,s_idx, k)     = typecast(payload(off+13),        'int8');
                end
            end
            save_buf.write_idx = save_buf.write_idx + 1;
        end

        raw_buf = raw_buf(PACKET_SIZE+1:end);

    end  % while пакеты

    % --- Обновление графиков с ограниченной частотой ---
    if toc(last_draw) >= DRAW_INTERVAL_S && ishandle(fig)
        last_draw = tic;

        % Разматываем циклический буфер в хронологический порядок
        ord = [(buf_write_idx:HIST_SAMPLES), (1:buf_write_idx-1)];
        t_hist = hist_time(ord);

        for row = 1:N_DISP
            sid = DISPLAY_SENSORS(row);
            for col = 1:4
                if col < 4
                    if strcmpi(DISPLAY_AXIS, 'gyro')
                        ydata = hist_gyro(ord, sid, col);
                    else
                        ydata = hist_accel(ord, sid, col);
                    end
                    ylim(ax(row,col), [-2100 2100]);
                else
                    ydata = hist_temp(ord, sid);
                    ylim(ax(row,col), [15 45]);
                end
                set(ln(row,col), 'XData', t_hist, 'YData', ydata);
                xlim(ax(row,col), [t_hist(1) t_hist(end)+0.01]);
            end
        end

        % Обновление строки статистики
        elapsed = toc(t_start);
        pkt_rate_real = stats.total_packets / elapsed;
        drop_pct = 100 * stats.total_dropped / max(1, stats.total_packets + stats.total_dropped);
        stat_str = sprintf(['  Пакеты: %7d  |  Ошибок CRC: %4d  |  '...
                            'Пропущено пакетов (FW): %6d (%.1f%%)  |  '...
                            'Ошибок синхр.: %6d  |  Скорость: %.1f пкт/с  |  '...
                            'Время: %.1f с'], ...
            stats.total_packets, stats.crc_errors, ...
            stats.total_dropped, drop_pct, ...
            stats.sync_errors, pkt_rate_real, elapsed);
        set(stat_text, 'String', stat_str);

        drawnow limitrate;
    end

end  % while главный цикл

% -------------------------------------------------------------------------
%  ЗАВЕРШЕНИЕ: закрытие порта и сохранение данных
% -------------------------------------------------------------------------
fprintf('\n--- Сбор данных завершён ---\n');
fprintf('Принято пакетов : %d\n', stats.total_packets);
fprintf('Ошибок CRC      : %d\n', stats.crc_errors);
fprintf('Пропущено (FW)  : %d\n', stats.total_dropped);
fprintf('Ошибок синхр.   : %d\n', stats.sync_errors);

delete(s);
fprintf('UART закрыт.\n');

if SAVE_TO_FILE && stats.total_packets > 0
    n = save_buf.write_idx - 1;
    capture.packets   = save_buf.packets(1:n);
    capture.ts        = save_buf.ts(1:n);
    capture.dropped   = save_buf.dropped(1:n);
    capture.gyro_raw  = save_buf.gyro_raw(1:n,:,:,:);
    capture.accel_raw = save_buf.accel_raw(1:n,:,:,:);
    capture.temp_raw  = save_buf.temp_raw(1:n,:,:);
    capture.ODR       = ODR;
    capture.N_SENSORS = N_SENSORS;
    capture.N_SAMPLES = N_SAMPLES;
    capture.GYRO_SCALE  = GYRO_SCALE;
    capture.ACCEL_SCALE = ACCEL_SCALE;
    capture.date      = datestr(now);
    save(SAVE_FILENAME, 'capture', '-v7.3');
    fprintf('Данные сохранены: %s (%d пакетов)\n', SAVE_FILENAME, n);
end

% =========================================================================
%  ЛОКАЛЬНЫЕ ФУНКЦИИ
% =========================================================================

function pos = find_sync(buf, sync_magic)
% Поиск позиции синхрослова в буфере.
    pos = [];
    for i = 1:(numel(buf)-3)
        if all(buf(i:i+3) == sync_magic)
            pos = i;
            return;
        end
    end
end

function crc = compute_crc32(data)
% CRC32/MPEG-2: poly=0x04C11DB7, init=0xFFFFFFFF, no reflect, no XOR out.
% Совпадает с аппаратным CRC STM32H7 (CRC-32 default).
    poly  = uint32(0x04C11DB7);
    crc   = uint32(0xFFFFFFFF);
    for i = 1:numel(data)
        crc = bitxor(crc, bitshift(uint32(data(i)), 24));
        % for ~1:8 % -- исправлено ниже
            if bitand(crc, uint32(0x80000000)) ~= 0
                crc = bitxor(bitshift(crc, 1, 'uint32'), poly, 'uint32');
            else
                crc = bitshift(crc, 1);
            end
        end
    end
    % Правильный цикл (MATLAB не поддерживает ~1:8)
    % Пересчитываем корректно:
    crc = uint32(0xFFFFFFFF);
    for i = 1:numel(data)
        crc = bitxor(crc, bitshift(uint32(data(i)), 24, 'uint32'), 'uint32');
        for k = 1:8
            if bitand(crc, uint32(0x80000000)) ~= 0
                crc = bitxor(bitshift(crc, 1, 'uint32'), poly, 'uint32');
            else
                crc = bitshift(crc, 1, 'uint32');
            end
        end
    end

