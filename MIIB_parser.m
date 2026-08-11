function MIIB_receiver()
% MIIB_receiver.m — MATLAB-приёмник телеметрии STM32H723 + 18x ICM-45686.
%
% Wire-формат строго по uart_telemetry.c / uart_telemetry.h (348 байт):
%
%   Offset   Size   Content
%   ------   ----   -----------------------------------------------
%   0..1     2      Header: 0xAA 0x55
%   2..3     2      frame_counter, uint16 LE
%   4..345   342    S00..S17: 18 x 19-byte HIRES IMU block
%   346..347 2      CRC16-CCITT LE, посчитан по bytes [2..345] (344 байта)
%
%   Один 19-byte IMU block (HIRES, 20-бит на ось, two's complement):
%     [0]  Ax[19:12]     [10] Gz[19:12]
%     [1]  Ax[11:4]      [11] Gz[11:4]
%     [2]  Ay[19:12]     [12] temp_raw[15:8]  (Big Endian, int16)
%     [3]  Ay[11:4]      [13] temp_raw[7:0]
%     [4]  Az[19:12]     [14] timestamp[15:8] (Big Endian, uint16)
%     [5]  Az[11:4]      [15] timestamp[7:0]
%     [6]  Gx[19:12]     [16] Ax[3:0]<<4 | Gx[3:0]
%     [7]  Gx[11:4]      [17] Ay[3:0]<<4 | Gy[3:0]
%     [8]  Gy[19:12]     [18] Az[3:0]<<4 | Gz[3:0]
%     [9]  Gy[11:4]
%
%   Слот, полностью состоящий из нулей == invalid marker (датчик fault
%   ИЛИ этот sample_idx отсутствовал в батче этого датчика) -> NaN.
%
%   frame_counter и CRC16: Little Endian. temp_raw/timestamp внутри
%   IMU-блока: Big Endian (см. UART_PackIMU19()).
%
%   Кадры идут на частоте до 1600 Гц (16 кадров на каждый цикл TIM6 @ 100 Гц,
%   см. UART_BuildAndSendSyncFrame() v2 / ring-buffer depth 32).

    clear; clc; close all;

    %% ===================== НАСТРОЙКИ =====================
    cfg.port           = "COM3";     % <-- поменяй под свой COM-порт
    cfg.baudrate       = 6000000;    % <-- 8.1 Mbaud, см. расчёт времени в прошивке
    cfg.header0        = uint8(hex2dec('AA'));
    cfg.header1        = uint8(hex2dec('55'));
    cfg.frame_len      = 348;        % UART_PKT_TOTAL_BYTES
    cfg.n_sensors      = 18;         % UART_SENSOR_COUNT
    cfg.imu_bytes      = 19;         % UART_IMU_WIRE_BYTES
    cfg.off_header     = 1;          % (MATLAB 1-indexed) байт 1..2
    cfg.off_counter    = 3;          % байт 3..4
    cfg.off_samples    = 5;          % байт 5..346 (18 x 19)
    cfg.off_crc        = 347;        % байт 347..348
    cfg.payload_len    = 344;        % UART_PAYLOAD_BYTES (counter + 18*19)

    % Масштабы HIRES 20-бит raw -> физические единицы.
    % ICM-45686 HIRES accel/gyro raw имеет тот же LSB, что и обычный 16-бит
    % режим, но с 4 дополнительными младшими битами точности. Поставь свои
    % реальные FS_SEL (пример ниже: Accel FS=±16g, Gyro FS=±2000dps,
    % полная шкала 20 бит -> LSB = FS / 2^19).
    cfg.accel_fs_g     = 16;         % <-- подставь свой ACCEL_FS_SEL
    cfg.gyro_fs_dps    = 2000;       % <-- подставь свой GYRO_FS_SEL
    cfg.accel_lsb      = cfg.accel_fs_g  / (2^19);   % [g / LSB]
    cfg.gyro_lsb       = cfg.gyro_fs_dps / (2^19);   % [dps / LSB]
    cfg.temp_scale     = 1/128;      % ICM-45686 temp: raw/128 + 25 (типично)
    cfg.temp_offset    = 25;

    cfg.plot_update_hz = 20;
    cfg.log_to_file    = true;
    cfg.log_filename   = sprintf('miib_log_%s.bin', datestr(now,'yyyymmdd_HHMMSS'));
    cfg.ring_len       = 16000;      % глубина ring-буфера графиков (сэмплов)

    %% ===================== ПОДКЛЮЧЕНИЕ =====================
    s = serialport(cfg.port, cfg.baudrate, "Timeout", 1);
    flush(s);
    fprintf('[MIIB] Подключено к %s @ %d baud\n', cfg.port, cfg.baudrate);

    if cfg.log_to_file
        log_fid = fopen(cfg.log_filename, 'w');
    else
        log_fid = -1;
    end

    %% ===================== БУФЕРЫ =====================
    data = struct();
    for k = 1:cfg.n_sensors
        data.sensor(k).accel      = nan(cfg.ring_len, 3);
        data.sensor(k).gyro       = nan(cfg.ring_len, 3);
        data.sensor(k).temp       = nan(cfg.ring_len, 1);
        data.sensor(k).timestamp  = nan(cfg.ring_len, 1);
        data.sensor(k).head       = 1;
        data.sensor(k).total      = 0;
        data.sensor(k).invalid    = 0;
    end

    stats.total_bytes    = 0;
    stats.total_frames   = 0;
    stats.crc_errors     = 0;
    stats.resync_events  = 0;
    stats.last_counter   = -1;
    stats.counter_gaps   = 0;
    stats.t_start        = tic;

    %% ===================== ГРАФИКИ (live) =====================
    fig = figure('Name', 'MIIB IMU Array Receiver (18x ICM-45686)', ...
                 'NumberTitle', 'off', 'Position', [80 80 1300 750]);

    ax1 = subplot(2,2,1);
    h1 = plot(ax1, nan(2,3)); title(ax1, 'Accel sensor #1 [g] (X/Y/Z)'); grid(ax1,'on');
    legend(ax1, {'X','Y','Z'}, 'Location','best');

    ax2 = subplot(2,2,2);
    h2  = plot(ax2, nan(2,3)); title(ax2, 'Gyro sensor #1 [dps] (X/Y/Z)'); grid(ax2,'on');
    legend(ax2, {'X','Y','Z'}, 'Location','best');

    ax3 = subplot(2,2,3);
    h3  = bar(ax3, zeros(1, cfg.n_sensors)); title(ax3, 'Кадров принято на датчик'); grid(ax3,'on');
    xlabel(ax3, 'Sensor ID (0..17)');

    ax4 = subplot(2,2,4);
    h4  = bar(ax4, zeros(1, cfg.n_sensors)); title(ax4, 'Invalid/fault маркеров на датчик'); grid(ax4,'on');
    xlabel(ax4, 'Sensor ID (0..17)');

    last_plot_time = tic;
    byte_buf = uint8.empty(1,0);

    cleanupObj = onCleanup(@() cleanup_port(s, log_fid, cfg)); %#ok<NASGU>

    %% ===================== ОСНОВНОЙ ЦИКЛ =====================
    while ishandle(fig)
        n_avail = s.NumBytesAvailable;
        if n_avail > 0
            chunk = read(s, n_avail, "uint8");
            byte_buf = [byte_buf, chunk]; %#ok<AGROW>
            stats.total_bytes = stats.total_bytes + n_avail;
        end

        [frames, byte_buf, n_resync] = extract_frames(byte_buf, cfg);
        stats.resync_events = stats.resync_events + n_resync;

        for i = 1:numel(frames)
            frm = frames{i};
            [ok, parsed] = parse_frame(frm, cfg);
            if ~ok
                stats.crc_errors = stats.crc_errors + 1;
                continue;
            end

            if stats.last_counter >= 0
                expected = mod(stats.last_counter + 1, 65536);
                if parsed.counter ~= expected
                    stats.counter_gaps = stats.counter_gaps + ...
                        mod(parsed.counter - expected, 65536);
                end
            end
            stats.last_counter = parsed.counter;
            stats.total_frames = stats.total_frames + 1;

            data = ingest_frame(data, parsed, cfg);

            if log_fid > 0
                fwrite(log_fid, frm, 'uint8');
            end
        end

        if toc(last_plot_time) > 1/cfg.plot_update_hz
            update_plots(h1, h2, h3, h4, data, cfg, stats);
            last_plot_time = tic;
        end

        drawnow limitrate;
    end

    print_final_stats(stats, data, cfg);
end

%% ============================================================
function [frames, buf, n_resync] = extract_frames(buf, cfg)
    frames = {};
    n_resync = 0;
    while true
        idx = find_header(buf, cfg);
        if isempty(idx)
            if numel(buf) > 1
                buf = buf(end);   % оставляем "хвост" на случай разрыва header на границе чтения
            end
            return;
        end
        if idx > 1
            n_resync = n_resync + 1;
        end
        buf = buf(idx:end);
        if numel(buf) < cfg.frame_len
            return; % ждём остальные байты кадра
        end
        frames{end+1} = buf(1:cfg.frame_len); %#ok<AGROW>
        buf = buf(cfg.frame_len+1:end);
    end
end

function idx = find_header(buf, cfg)
    idx = [];
    if numel(buf) < 2
        return;
    end
    pos = strfind(double(buf), double([cfg.header0, cfg.header1]));
    if ~isempty(pos)
        idx = pos(1);
    end
end

function [ok, p] = parse_frame(frame, cfg)
    ok = false;
    p = struct();

    % CRC16-CCITT по payload [3..346] (1-индексация MATLAB) == bytes [2..345] прошивки
    payload = frame(cfg.off_counter : cfg.off_counter + cfg.payload_len - 1);
    crc_calc = crc16_ccitt(payload);
    crc_recv = double(frame(cfg.off_crc)) + double(frame(cfg.off_crc+1)) * 256;
    if crc_calc ~= crc_recv
        return;
    end

    p.counter = double(frame(cfg.off_counter)) + double(frame(cfg.off_counter+1)) * 256;

    p.accel     = nan(cfg.n_sensors, 3);
    p.gyro      = nan(cfg.n_sensors, 3);
    p.temp      = nan(cfg.n_sensors, 1);
    p.timestamp = nan(cfg.n_sensors, 1);
    p.invalid   = false(cfg.n_sensors, 1);

    for k = 1:cfg.n_sensors
        off = cfg.off_samples + (k-1)*cfg.imu_bytes;
        blk = frame(off : off + cfg.imu_bytes - 1);

        if all(blk == 0)
            p.invalid(k) = true;
            continue;
        end

        ax20 = bitor(bitor(bitshift(uint32(blk(1)),12), bitshift(uint32(blk(2)),4)), bitshift(uint32(bitand(blk(17),0xF0)),-4));
        ay20 = bitor(bitor(bitshift(uint32(blk(3)),12), bitshift(uint32(blk(4)),4)), bitshift(uint32(bitand(blk(18),0xF0)),-4));
        az20 = bitor(bitor(bitshift(uint32(blk(5)),12), bitshift(uint32(blk(6)),4)), bitshift(uint32(bitand(blk(19),0xF0)),-4));
        gx20 = bitor(bitor(bitshift(uint32(blk(7)),12), bitshift(uint32(blk(8)),4)), uint32(bitand(blk(17),0x0F)));
        gy20 = bitor(bitor(bitshift(uint32(blk(9)),12), bitshift(uint32(blk(10)),4)), uint32(bitand(blk(18),0x0F)));
        gz20 = bitor(bitor(bitshift(uint32(blk(11)),12), bitshift(uint32(blk(12)),4)), uint32(bitand(blk(19),0x0F)));

        p.accel(k,1) = to_signed20(ax20) * cfg.accel_lsb;
        p.accel(k,2) = to_signed20(ay20) * cfg.accel_lsb;
        p.accel(k,3) = to_signed20(az20) * cfg.accel_lsb;
        p.gyro(k,1)  = to_signed20(gx20) * cfg.gyro_lsb;
        p.gyro(k,2)  = to_signed20(gy20) * cfg.gyro_lsb;
        p.gyro(k,3)  = to_signed20(gz20) * cfg.gyro_lsb;

        temp_raw = int16(bitshift(uint16(blk(13)),8) + uint16(blk(14)));  % Big Endian
        p.temp(k) = double(temp_raw) * cfg.temp_scale + cfg.temp_offset;

        p.timestamp(k) = double(bitshift(uint16(blk(15)),8) + uint16(blk(16))); % Big Endian
    end

    ok = true;
end

function v = to_signed20(u20)
    % Two's complement 20-bit -> signed double.
    u20 = double(u20);
    if u20 >= 2^19
        v = u20 - 2^20;
    else
        v = u20;
    end
end

function crc = crc16_ccitt(data)
    % Идентично CRC16_CCITT() в uart_telemetry.c: poly 0x1021, init 0xFFFF,
    % XOR только старшего байта на входе (без табличной оптимизации).
    crc = uint16(hex2dec('FFFF'));
    poly = uint16(hex2dec('1021'));
    for b = 1:numel(data)
        crc = bitxor(crc, bitshift(uint16(data(b)), 8));
        for bit = 1:8
            if bitand(crc, uint16(hex2dec('8000'))) ~= 0
                crc = bitxor(bitshift(crc,1), poly);
            else
                crc = bitshift(crc,1);
            end
        end
    end
    crc = double(crc);
end

function data = ingest_frame(data, p, cfg)
    for k = 1:cfg.n_sensors
        sd = data.sensor(k);
        h = sd.head;
        ring_len = size(sd.accel,1);

        if p.invalid(k)
            sd.invalid = sd.invalid + 1;
            sd.accel(h,:) = nan(1,3);
            sd.gyro(h,:)  = nan(1,3);
            sd.temp(h)    = nan;
            sd.timestamp(h) = nan;
        else
            sd.accel(h,:) = p.accel(k,:);
            sd.gyro(h,:)  = p.gyro(k,:);
            sd.temp(h)    = p.temp(k);
            sd.timestamp(h) = p.timestamp(k);
        end

        sd.total = sd.total + 1;
        sd.head  = mod(h, ring_len) + 1;
        data.sensor(k) = sd;
    end
end

function update_plots(h1, h2, h3, h4, data, cfg, stats)
    s1 = data.sensor(1);
    valid = ~isnan(s1.accel(:,1));
    if any(valid)
        idxs = find(valid);
        set(h1(1), 'XData', 1:numel(idxs), 'YData', s1.accel(idxs,1));
        set(h1(2), 'XData', 1:numel(idxs), 'YData', s1.accel(idxs,2));
        set(h1(3), 'XData', 1:numel(idxs), 'YData', s1.accel(idxs,3));
        set(h2(1), 'XData', 1:numel(idxs), 'YData', s1.gyro(idxs,1));
        set(h2(2), 'XData', 1:numel(idxs), 'YData', s1.gyro(idxs,2));
        set(h2(3), 'XData', 1:numel(idxs), 'YData', s1.gyro(idxs,3));
    end

    totals   = arrayfun(@(k) data.sensor(k).total,   1:cfg.n_sensors);
    invalids = arrayfun(@(k) data.sensor(k).invalid, 1:cfg.n_sensors);
    set(h3, 'YData', totals);
    set(h4, 'YData', invalids);

    elapsed = toc(stats.t_start);
    fprintf(['\r[MIIB] t=%6.1fs frames=%8d bytes=%10d crc_err=%4d ' ...
             'resync=%4d ctr_gaps=%5d rate=%7.0f fps'], ...
        elapsed, stats.total_frames, stats.total_bytes, stats.crc_errors, ...
        stats.resync_events, stats.counter_gaps, stats.total_frames/max(elapsed,eps));
end

function print_final_stats(stats, data, cfg)
    fprintf('\n\n===== MIIB Receiver: итоговая статистика =====\n');
    fprintf('Всего кадров:      %d\n', stats.total_frames);
    fprintf('Всего байт:        %d\n', stats.total_bytes);
    fprintf('Ошибок CRC:        %d\n', stats.crc_errors);
    fprintf('Resync событий:    %d\n', stats.resync_events);
    fprintf('Counter gaps:      %d\n', stats.counter_gaps);
    for k = 1:cfg.n_sensors
        sd = data.sensor(k);
        fprintf('  sensor%02d: total=%6d invalid=%4d\n', k-1, sd.total, sd.invalid);
    end
end

function cleanup_port(s, log_fid, cfg)
    if log_fid > 0
        fclose(log_fid);
        fprintf('\n[MIIB] Лог сохранён: %s\n', cfg.log_filename);
    end
    clear s;
    fprintf('[MIIB] Порт закрыт.\n');
end
