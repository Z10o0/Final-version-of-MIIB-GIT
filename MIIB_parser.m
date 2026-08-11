function result = MIIB_run(varargin)
% MIIB_run.m — ОДИН файл: приём -> парсинг -> графики.
%
% Работает в 3 последовательных этапа внутри одной функции:
%   1) ПРИЁМ: чистое чтение сырых байт с UART в память (без парсинга,
%      без CRC, без графиков -- максимальная скорость чтения порта).
%   2) ПАРСИНГ: после остановки приёма разбирается весь накопленный
%      буфер целиком (поиск заголовков, проверка CRC16, распаковка
%      20-битных IMU-блоков).
%   3) ГРАФИКИ: строятся итоговые графики и статистика по всем 18 датчикам.
%
% Wire-формат (строго по uart_telemetry.c / uart_telemetry.h, 348 байт):
%   Offset   Size   Content
%   0..1     2      Header: 0xAA 0x55
%   2..3     2      frame_counter, uint16 LE
%   4..345   342    S00..S17: 18 x 19-byte HIRES IMU block
%   346..347 2      CRC16-CCITT LE по bytes [2..345] (344 байта)
%
% Использование:
%   result = MIIB_run();                                  % настройки по умолчанию
%   result = MIIB_run('port','COM3','duration',20);        % 20 секунд приёма
%   result = MIIB_run('port','COM3','duration',10,'plot_sensor',5);

    %% ===================== НАСТРОЙКИ =====================
    p = inputParser;
    addParameter(p, 'port',        "COM3");
    addParameter(p, 'baudrate',    8000000);
    addParameter(p, 'duration',    15);        % секунд приёма
    addParameter(p, 'accel_fs_g',  32);
    addParameter(p, 'gyro_fs_dps', 4000);
    addParameter(p, 'temp_scale',  1/128);
    addParameter(p, 'temp_offset', 25);
    addParameter(p, 'plot_sensor', 1);          % какой датчик (1..18) рисовать подробно
    addParameter(p, 'save_raw',    false);       % сохранить сырой .bin рядом на всякий случай
    parse(p, varargin{:});
    cfg = p.Results;

    cfg.header0     = uint8(hex2dec('AA'));
    cfg.header1     = uint8(hex2dec('55'));
    cfg.frame_len   = 348;
    cfg.n_sensors   = 18;
    cfg.imu_bytes   = 19;
    cfg.off_counter = 3;   % 1-indexed
    cfg.off_samples = 5;
    cfg.off_crc     = 347;
    cfg.payload_len = 344;
    cfg.accel_lsb   = cfg.accel_fs_g  / (2^19);
    cfg.gyro_lsb    = cfg.gyro_fs_dps / (2^19);

    %% ===================== ЭТАП 1: ПРИЁМ =====================
    % [ВАЖНО] configureTerminator НЕ вызывается -- это бинарный протокол,
    % терминатор строк здесь не нужен и мог мешать корректному чтению.
    s = serialport(cfg.port, cfg.baudrate, "Timeout", 1);
    s.InputBufferSize = 16 * 1024 * 1024;  % 16 MB, запас на весь захват
    flush(s);

    fprintf('[MIIB] Подключено к %s @ %d baud\n', cfg.port, cfg.baudrate);
    fprintf('[MIIB] Приём данных %.1f секунд...\n', cfg.duration);

    % Предвыделяем буфер под приём (с запасом), реальный размер обрежем в конце.
    est_bytes = ceil(cfg.duration * cfg.baudrate / 8 * 1.5);
    raw = zeros(1, est_bytes, 'uint8');
    raw_len = 0;

    t_start = tic;
    t_last_log = tic;

    while toc(t_start) < cfg.duration
        n_avail = s.NumBytesAvailable;
        if n_avail > 0
            chunk = read(s, n_avail, "uint8");
            n = numel(chunk);
            if raw_len + n > numel(raw)
                raw = [raw, zeros(1, numel(raw), 'uint8')]; %#ok<AGROW> % рост буфера при необходимости
            end
            raw(raw_len+1 : raw_len+n) = chunk;
            raw_len = raw_len + n;
        else
            pause(0.0005);
        end

        if toc(t_last_log) >= 1
            elapsed = toc(t_start);
            fprintf('\r[MIIB] Приём: t=%5.1f/%5.1fs bytes=%10d rate=%8.0f KB/s', ...
                elapsed, cfg.duration, raw_len, (raw_len/1024)/max(elapsed,eps));
            t_last_log = tic;
        end
    end

    raw = raw(1:raw_len);
    clear s;
    fprintf('\n[MIIB] Приём завершён. Всего байт: %d\n', raw_len);

    if cfg.save_raw
        raw_filename = sprintf('miib_raw_%s.bin', datestr(now,'yyyymmdd_HHMMSS'));
        fid = fopen(raw_filename, 'w');
        fwrite(fid, raw, 'uint8');
        fclose(fid);
        fprintf('[MIIB] Сырые данные сохранены: %s\n', raw_filename);
    end

    %% ===================== ЭТАП 2: ПАРСИНГ =====================
    fprintf('[MIIB] Парсинг накопленного буфера...\n');

    crc_table = build_crc16_table();

    % Векторизованный поиск всех потенциальных заголовков 0xAA 0x55.
    is_hdr = (raw(1:end-1) == cfg.header0) & (raw(2:end) == cfg.header1);
    hdr_positions = find(is_hdr);
    hdr_positions = hdr_positions(hdr_positions + cfg.frame_len - 1 <= numel(raw));

    fprintf('[MIIB] Найдено потенциальных заголовков: %d\n', numel(hdr_positions));

    n_cand = numel(hdr_positions);
    frame_starts = zeros(1, n_cand, 'uint32');
    n_frames = 0;

    i = 1;
    while i <= n_cand
        pos = hdr_positions(i);
        frame = raw(pos : pos + cfg.frame_len - 1);
        payload = frame(cfg.off_counter : cfg.off_counter + cfg.payload_len - 1);
        crc_calc = crc16_ccitt_table(payload, crc_table);
        crc_recv = double(frame(cfg.off_crc)) + double(frame(cfg.off_crc+1)) * 256;

        if crc_calc == crc_recv
            n_frames = n_frames + 1;
            frame_starts(n_frames) = pos;
            next_pos = pos + cfg.frame_len;
            i = find(hdr_positions >= next_pos, 1, 'first');
            if isempty(i)
                break;
            end
        else
            i = i + 1;
        end
    end
    frame_starts = frame_starts(1:n_frames);

    fprintf('[MIIB] Валидных кадров (CRC OK): %d\n', n_frames);
    fprintf('[MIIB] Отброшено (CRC fail / вложенные): %d\n', n_cand - n_frames);

    if n_frames == 0
        error('MIIB_run:nodata', 'Не найдено ни одного валидного кадра. Проверь port/baudrate.');
    end

    counters = zeros(n_frames, 1);
    accel    = nan(n_frames, cfg.n_sensors, 3);
    gyro     = nan(n_frames, cfg.n_sensors, 3);
    temp     = nan(n_frames, cfg.n_sensors);
    tstamp   = nan(n_frames, cfg.n_sensors);
    invalid  = false(n_frames, cfg.n_sensors);

    for fidx = 1:n_frames
        pos = frame_starts(fidx);
        frame = raw(pos : pos + cfg.frame_len - 1);
        counters(fidx) = double(frame(cfg.off_counter)) + double(frame(cfg.off_counter+1)) * 256;

        for k = 1:cfg.n_sensors
            off = cfg.off_samples + (k-1)*cfg.imu_bytes;
            blk = frame(off : off + cfg.imu_bytes - 1);

            if all(blk == 0)
                invalid(fidx, k) = true;
                continue;
            end

            ax20 = bitor(bitor(bitshift(uint32(blk(1)),12), bitshift(uint32(blk(2)),4)), bitshift(uint32(bitand(blk(17),0xF0)),-4));
            ay20 = bitor(bitor(bitshift(uint32(blk(3)),12), bitshift(uint32(blk(4)),4)), bitshift(uint32(bitand(blk(18),0xF0)),-4));
            az20 = bitor(bitor(bitshift(uint32(blk(5)),12), bitshift(uint32(blk(6)),4)), bitshift(uint32(bitand(blk(19),0xF0)),-4));
            gx20 = bitor(bitor(bitshift(uint32(blk(7)),12), bitshift(uint32(blk(8)),4)), uint32(bitand(blk(17),0x0F)));
            gy20 = bitor(bitor(bitshift(uint32(blk(9)),12), bitshift(uint32(blk(10)),4)), uint32(bitand(blk(18),0x0F)));
            gz20 = bitor(bitor(bitshift(uint32(blk(11)),12), bitshift(uint32(blk(12)),4)), uint32(bitand(blk(19),0x0F)));

            accel(fidx,k,1) = to_signed20(ax20) * cfg.accel_lsb;
            accel(fidx,k,2) = to_signed20(ay20) * cfg.accel_lsb;
            accel(fidx,k,3) = to_signed20(az20) * cfg.accel_lsb;
            gyro(fidx,k,1)  = to_signed20(gx20) * cfg.gyro_lsb;
            gyro(fidx,k,2)  = to_signed20(gy20) * cfg.gyro_lsb;
            gyro(fidx,k,3)  = to_signed20(gz20) * cfg.gyro_lsb;

            temp_raw = int16(bitshift(uint16(blk(13)),8) + uint16(blk(14))); % Big Endian
            temp(fidx,k) = double(temp_raw) * cfg.temp_scale + cfg.temp_offset;
            tstamp(fidx,k) = double(bitshift(uint16(blk(15)),8) + uint16(blk(16))); % Big Endian
        end

        if mod(fidx, 5000) == 0
            fprintf('\r[MIIB] Распаковано %d / %d кадров', fidx, n_frames);
        end
    end
    fprintf('\r[MIIB] Распаковано %d / %d кадров\n', n_frames, n_frames);

    counter_gaps = 0;
    if n_frames > 1
        d = mod(diff(counters), 65536);
        counter_gaps = sum(d(d ~= 1));
    end

    fprintf('\n===== MIIB: итоговая статистика =====\n');
    fprintf('Всего валидных кадров: %d\n', n_frames);
    fprintf('Counter gaps:          %d\n', counter_gaps);
    for k = 1:cfg.n_sensors
        n_invalid = sum(invalid(:,k));
        fprintf('  sensor%02d: valid=%6d invalid=%6d\n', k-1, n_frames-n_invalid, n_invalid);
    end

    %% ===================== ЭТАП 3: ГРАФИКИ =====================
    k = cfg.plot_sensor;
    figure('Name', sprintf('MIIB — sensor #%d', k-1), 'NumberTitle', 'off', ...
           'Position', [80 80 1300 750]);

    subplot(2,2,1);
    plot(squeeze(accel(:,k,:)));
    title(sprintf('Accel sensor #%d [g] (X/Y/Z)', k-1)); grid on;
    legend({'X','Y','Z'}, 'Location','best');

    subplot(2,2,2);
    plot(squeeze(gyro(:,k,:)));
    title(sprintf('Gyro sensor #%d [dps] (X/Y/Z)', k-1)); grid on;
    legend({'X','Y','Z'}, 'Location','best');

    subplot(2,2,3);
    bar(sum(~invalid,1));
    title('Валидных кадров на датчик'); grid on;
    xlabel('Sensor ID (0..17)');

    subplot(2,2,4);
    bar(sum(invalid,1));
    title('Invalid/fault маркеров на датчик'); grid on;
    xlabel('Sensor ID (0..17)');

    %% ===================== ВОЗВРАТ РЕЗУЛЬТАТА =====================
    result.counters     = counters;
    result.accel        = accel;   % [n_frames x 18 x 3], g
    result.gyro         = gyro;    % [n_frames x 18 x 3], dps
    result.temp         = temp;    % [n_frames x 18], °C
    result.timestamp    = tstamp;  % [n_frames x 18]
    result.invalid      = invalid; % [n_frames x 18] logical
    result.n_frames     = n_frames;
    result.counter_gaps = counter_gaps;
    result.cfg          = cfg;

    fprintf('\n[MIIB] Готово. Данные доступны в возвращённой структуре result.\n');
end

%% ============================================================
function v = to_signed20(u20)
    u20 = double(u20);
    if u20 >= 2^19
        v = u20 - 2^20;
    else
        v = u20;
    end
end

function table = build_crc16_table()
    table = zeros(1, 256, 'uint16');
    poly = uint16(hex2dec('1021'));
    for i = 0:255
        crc = bitshift(uint16(i), 8);
        for bit = 1:8
            if bitand(crc, uint16(hex2dec('8000'))) ~= 0
                crc = bitxor(bitshift(crc,1), poly);
            else
                crc = bitshift(crc,1);
            end
        end
        table(i+1) = crc;
    end
end

function crc = crc16_ccitt_table(data, table)
    crc = uint16(hex2dec('FFFF'));
    d = uint16(data);
    for b = 1:numel(d)
        idx = bitxor(bitshift(crc, -8), d(b)) + 1;
        crc = bitxor(bitshift(crc, 8), table(idx));
    end
    crc = double(crc);
end
