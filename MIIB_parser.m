function result = MIIB_run(varargin)
% MIIB_run.m — приём RAW -> парсинг -> графики.
%
% Поток от STM32:
%   100 батчей/с;
%   16 UART-кадров/батч;
%   348 байт/кадр;
%   5568 байт/батч;
%   556800 байт/с.
%
% Приём сделан burst-режимом: раз в 10 мс MATLAB забирает всё,
% что накопилось в драйверном буфере, не привязываясь к границе пакета.
% Это необходимо, потому что Windows/MATLAB не являются RTOS.

    %% ===================== НАСТРОЙКИ =====================
    p = inputParser;

    addParameter(p, 'port',         "COM3");
    addParameter(p, 'baudrate',     8000000);
    addParameter(p, 'duration',     10);
    addParameter(p, 'accel_fs_g',   32);
    addParameter(p, 'gyro_fs_dps',  4000);
    addParameter(p, 'temp_scale',   1/128);
    addParameter(p, 'temp_offset',  25);
    addParameter(p, 'plot_sensor',  1);
    addParameter(p, 'save_raw',     false);

    parse(p, varargin{:});
    cfg = p.Results;

    cfg.header0     = uint8(hex2dec('AA'));
    cfg.header1     = uint8(hex2dec('55'));
    cfg.frame_len   = 348;
    cfg.n_sensors   = 18;
    cfg.imu_bytes   = 19;
    cfg.off_counter = 3;       % MATLAB 1-indexed
    cfg.off_samples = 5;
    cfg.off_crc     = 347;
    cfg.payload_len = 344;

    cfg.frames_per_batch = 16;
    cfg.batches_per_sec  = 100;
    cfg.batch_bytes      = cfg.frames_per_batch * cfg.frame_len;  % 5568
    cfg.expected_bps     = cfg.batch_bytes * cfg.batches_per_sec; % 556800

    cfg.accel_lsb = cfg.accel_fs_g  / (2^19);
    cfg.gyro_lsb  = cfg.gyro_fs_dps / (2^19);

    %% ===================== ЭТАП 1: RAW ПРИЁМ =====================
    % Не используем configureTerminator: поток бинарный.
    %
    % Windows не гарантирует вызов цикла раз в 10.000 мс.
    % Поэтому читаем не ровно 5568 байт, а всё накопившееся,
    % ограничивая одну операцию 8 батчами.
    RX_PERIOD_S       = 0.010;
    RX_MAX_BATCHES_RD = 8;
    RX_READ_LIMIT     = RX_MAX_BATCHES_RD * cfg.batch_bytes; % 44544 bytes
    RX_BUFFER_MARGIN  = 1.20;

    s = serialport(cfg.port, cfg.baudrate, "Timeout", 0.25);

    % Для 60 с нужно ~33.4 MB; используем запас.
    s.InputBufferSize = max(16 * 1024 * 1024, ...
        ceil(cfg.duration * cfg.expected_bps * RX_BUFFER_MARGIN));

    flush(s);

    % Буфер именно по ожидаемому полезному потоку.
    raw_capacity = ceil(cfg.duration * cfg.expected_bps * RX_BUFFER_MARGIN) ...
                 + 4 * RX_READ_LIMIT;
    raw = zeros(raw_capacity, 1, 'uint8');
    raw_len = 0;

    rx_peak_available = 0;
    rx_peak_read      = 0;
    rx_iterations     = 0;
    rx_over_capacity  = 0;

    fprintf('[MIIB] COM: %s @ %d baud\n', cfg.port, cfg.baudrate);
    fprintf('[MIIB] Expected: %d bytes/s, %d bytes/batch\n', ...
        cfg.expected_bps, cfg.batch_bytes);
    fprintf('[MIIB] RAW capture: %.3f s\n', cfg.duration);

    t_start = tic;
    t_next  = 0;

    while true
        elapsed = toc(t_start);
        if elapsed >= cfg.duration
            break;
        end

        if elapsed < t_next
            pause(max(0, t_next - elapsed));
        end
        t_next = t_next + RX_PERIOD_S;

        n_avail = s.NumBytesAvailable;
        rx_peak_available = max(rx_peak_available, n_avail);
        rx_iterations = rx_iterations + 1;

        % Если MATLAB не успел, драйвер мог накопить >RX_READ_LIMIT.
        % В этом случае быстро дочитываем буфер несколькими крупными read.
        while n_avail > 0
            n_read = min(n_avail, RX_READ_LIMIT);
            chunk = read(s, n_read, "uint8");
            n = numel(chunk);

            if n == 0
                break;
            end

            rx_peak_read = max(rx_peak_read, n);

            if raw_len + n > numel(raw)
                % Редкая защита: рост большого блока, а не частое AGROW.
                grow_by = max(numel(raw), 16 * RX_READ_LIMIT);
                raw = [raw; zeros(grow_by, 1, 'uint8')]; %#ok<AGROW>
                rx_over_capacity = rx_over_capacity + 1;
            end

            raw(raw_len + 1 : raw_len + n) = chunk;
            raw_len = raw_len + n;

            n_avail = s.NumBytesAvailable;
            rx_peak_available = max(rx_peak_available, n_avail);
        end
    end

    % Дать USB/VCP доставить последний уже отправленный UART-батч,
    % затем забрать весь остаток из драйверного буфера.
    pause(0.030);

    while s.NumBytesAvailable > 0
        n_read = min(s.NumBytesAvailable, RX_READ_LIMIT);
        chunk = read(s, n_read, "uint8");
        n = numel(chunk);

        if raw_len + n > numel(raw)
            raw = [raw; zeros(max(numel(raw), 16 * RX_READ_LIMIT), 1, 'uint8')]; %#ok<AGROW>
            rx_over_capacity = rx_over_capacity + 1;
        end

        raw(raw_len + 1 : raw_len + n) = chunk;
        raw_len = raw_len + n;
    end

    elapsed_rx = toc(t_start);

    flush(s);
    clear s;

    raw = raw(1:raw_len).';  % далее оставляем row-vector, как в твоём коде

    rx_stats.duration_s         = elapsed_rx;
    rx_stats.bytes              = raw_len;
    rx_stats.bytes_per_sec      = raw_len / elapsed_rx;
    rx_stats.kib_per_sec        = raw_len / elapsed_rx / 1024;
    rx_stats.expected_bps       = cfg.expected_bps;
    rx_stats.capture_ratio      = raw_len / max(elapsed_rx * cfg.expected_bps, 1);
    rx_stats.period_s           = RX_PERIOD_S;
    rx_stats.iterations         = rx_iterations;
    rx_stats.max_available      = rx_peak_available;
    rx_stats.max_single_read    = rx_peak_read;
    rx_stats.buffer_extensions  = rx_over_capacity;

    fprintf('[MIIB] RX done: %d bytes in %.3f s = %.1f KiB/s (%.2f%% expected)\n', ...
        raw_len, elapsed_rx, rx_stats.kib_per_sec, 100 * rx_stats.capture_ratio);
    fprintf('[MIIB] Driver backlog peak: %d bytes; read peak: %d bytes\n', ...
        rx_peak_available, rx_peak_read);

    if cfg.save_raw
        raw_filename = sprintf('miib_raw_%s.bin', datestr(now, 'yyyymmdd_HHMMSS'));
        fid = fopen(raw_filename, 'w');
        assert(fid >= 0, 'MIIB_run:file', 'Cannot open raw output file.');
        fwrite(fid, raw, 'uint8');
        fclose(fid);
        fprintf('[MIIB] RAW saved: %s\n', raw_filename);
    end

    %% ===================== ЭТАП 2: ПАРСИНГ =====================
    fprintf('[MIIB] Parsing raw buffer...\n');

    crc_table = build_crc16_table();

    is_hdr = (raw(1:end-1) == cfg.header0) & ...
             (raw(2:end)   == cfg.header1);

    hdr_positions = find(is_hdr);
    hdr_positions = hdr_positions( ...
        hdr_positions + cfg.frame_len - 1 <= numel(raw));

    fprintf('[MIIB] Candidate headers: %d\n', numel(hdr_positions));

    n_cand = numel(hdr_positions);
    frame_starts = zeros(1, n_cand, 'uint32');
    n_frames = 0;
    i = 1;

    while i <= n_cand
        pos = hdr_positions(i);
        frame = raw(pos : pos + cfg.frame_len - 1);

        payload = frame(cfg.off_counter : ...
                        cfg.off_counter + cfg.payload_len - 1);

        crc_calc = crc16_ccitt_table(payload, crc_table);
        crc_recv = double(frame(cfg.off_crc)) + ...
                   256 * double(frame(cfg.off_crc + 1));

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

    fprintf('[MIIB] Valid CRC frames: %d\n', n_frames);
    fprintf('[MIIB] Rejected candidates: %d\n', n_cand - n_frames);

    if n_frames == 0
        error('MIIB_run:nodata', ...
            'No valid frame found. Check COM port, baud rate and wire format.');
    end

    %% ===================== РАСПАКОВКА =====================
    counters = zeros(n_frames, 1);
    accel    = nan(n_frames, cfg.n_sensors, 3);
    gyro     = nan(n_frames, cfg.n_sensors, 3);
    temp     = nan(n_frames, cfg.n_sensors);
    tstamp   = nan(n_frames, cfg.n_sensors);
    invalid  = false(n_frames, cfg.n_sensors);

    for fidx = 1:n_frames
        pos = double(frame_starts(fidx));
        frame = raw(pos : pos + cfg.frame_len - 1);

        counters(fidx) = double(frame(cfg.off_counter)) + ...
                          256 * double(frame(cfg.off_counter + 1));

        for k = 1:cfg.n_sensors
            off = cfg.off_samples + (k - 1) * cfg.imu_bytes;
            blk = frame(off : off + cfg.imu_bytes - 1);

            if all(blk == 0)
                invalid(fidx, k) = true;
                continue;
            end

            ax20 = bitor( ...
                bitor(bitshift(uint32(blk(1)), 12), ...
                      bitshift(uint32(blk(2)), 4)), ...
                bitshift(uint32(bitand(blk(17), 0xF0)), -4));

            ay20 = bitor( ...
                bitor(bitshift(uint32(blk(3)), 12), ...
                      bitshift(uint32(blk(4)), 4)), ...
                bitshift(uint32(bitand(blk(18), 0xF0)), -4));

            az20 = bitor( ...
                bitor(bitshift(uint32(blk(5)), 12), ...
                      bitshift(uint32(blk(6)), 4)), ...
                bitshift(uint32(bitand(blk(19), 0xF0)), -4));

            gx20 = bitor( ...
                bitor(bitshift(uint32(blk(7)), 12), ...
                      bitshift(uint32(blk(8)), 4)), ...
                uint32(bitand(blk(17), 0x0F)));

            gy20 = bitor( ...
                bitor(bitshift(uint32(blk(9)), 12), ...
                      bitshift(uint32(blk(10)), 4)), ...
                uint32(bitand(blk(18), 0x0F)));

            gz20 = bitor( ...
                bitor(bitshift(uint32(blk(11)), 12), ...
                      bitshift(uint32(blk(12)), 4)), ...
                uint32(bitand(blk(19), 0x0F)));

            accel(fidx, k, 1) = to_signed20(ax20) * cfg.accel_lsb;
            accel(fidx, k, 2) = to_signed20(ay20) * cfg.accel_lsb;
            accel(fidx, k, 3) = to_signed20(az20) * cfg.accel_lsb;

            gyro(fidx, k, 1) = to_signed20(gx20) * cfg.gyro_lsb;
            gyro(fidx, k, 2) = to_signed20(gy20) * cfg.gyro_lsb;
            gyro(fidx, k, 3) = to_signed20(gz20) * cfg.gyro_lsb;

            temp_raw = int16( ...
                bitshift(uint16(blk(13)), 8) + uint16(blk(14)));

            temp(fidx, k) = double(temp_raw) * cfg.temp_scale + ...
                             cfg.temp_offset;

            tstamp(fidx, k) = double( ...
                bitshift(uint16(blk(15)), 8) + uint16(blk(16)));
        end
    end

    %% ===================== СТАТИСТИКА =====================
    if n_frames > 1
        d = mod(diff(counters), 65536);
        counter_gaps = sum(d ~= 1);
        missing_frames = sum(max(d - 1, 0));
    else
        counter_gaps = 0;
        missing_frames = 0;
    end

    fprintf('\n===== MIIB summary =====\n');
    fprintf('RAW:                  %d bytes\n', raw_len);
    fprintf('Valid CRC frames:     %d\n', n_frames);
    fprintf('Counter gap events:   %d\n', counter_gaps);
    fprintf('Missing frame count:  %d\n', missing_frames);

    for k = 1:cfg.n_sensors
        n_invalid = sum(invalid(:, k));
        fprintf('sensor%02d: valid=%d invalid=%d\n', ...
            k - 1, n_frames - n_invalid, n_invalid);
    end

    %% ===================== ГРАФИКИ =====================
    k = cfg.plot_sensor;

    figure( ...
        'Name', sprintf('MIIB — sensor #%d', k - 1), ...
        'NumberTitle', 'off', ...
        'Position', [80 80 1300 750]);

    subplot(2, 2, 1);
    plot(squeeze(accel(:, k, :)));
    title(sprintf('Accel sensor #%d [g]', k - 1));
    grid on;
    legend({'X', 'Y', 'Z'}, 'Location', 'best');

    subplot(2, 2, 2);
    plot(squeeze(gyro(:, k, :)));
    title(sprintf('Gyro sensor #%d [dps]', k - 1));
    grid on;
    legend({'X', 'Y', 'Z'}, 'Location', 'best');

    subplot(2, 2, 3);
    bar(sum(~invalid, 1));
    title('Valid frames per sensor');
    xlabel('Sensor ID (0..17)');
    grid on;

    subplot(2, 2, 4);
    bar(sum(invalid, 1));
    title('Invalid / fault markers per sensor');
    xlabel('Sensor ID (0..17)');
    grid on;

    %% ===================== РЕЗУЛЬТАТ =====================
    result.counters       = counters;
    result.accel          = accel;
    result.gyro           = gyro;
    result.temp           = temp;
    result.timestamp      = tstamp;
    result.invalid        = invalid;
    result.n_frames       = n_frames;
    result.counter_gaps   = counter_gaps;
    result.missing_frames = missing_frames;
    result.rx_stats       = rx_stats;
    result.cfg            = cfg;

    fprintf('\n[MIIB] Done.\n');
end

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
                crc = bitxor(bitshift(crc, 1), poly);
            else
                crc = bitshift(crc, 1);
            end
        end

        table(i + 1) = crc;
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