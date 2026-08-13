function result = MIIB_run_py_allplots(varargin)
% MIIB_run_py_allplots.m
% Conservative MATLAB version:
% - launches Python capture
% - saves RAW near this .m file
% - parses MIIB frames offline
% - plots all 18 sensors for accel/gyro/temp/timestamp
% - avoids invisible legend axes and fragile tiledlayout tricks
%
% Example:
%   result = MIIB_run_py_allplots();
%   result = MIIB_run_py_allplots('port',"COM3",'duration',10,'export_plots',true);

    %% ===================== SETTINGS =====================
    p = inputParser;
    addParameter(p, 'port',           "COM3");
    addParameter(p, 'baudrate',       12000000);
    addParameter(p, 'duration',       10);
    addParameter(p, 'python_exe',     "C:\ProgramData\spyder-6\envs\spyder-runtime\python.exe");
    addParameter(p, 'python_script',  "");
    addParameter(p, 'accel_fs_g',     32);
    addParameter(p, 'gyro_fs_dps',    4000);
    addParameter(p, 'temp_scale',     1/128);
    addParameter(p, 'temp_offset',    25);
    addParameter(p, 'keep_raw',       true);
    addParameter(p, 'export_plots',   false);
    addParameter(p, 'show_figures',   true);
    addParameter(p, 'save_mat',       false);
    parse(p, varargin{:});
    cfg = p.Results;

    cfg.header0     = uint8(hex2dec('AA'));
    cfg.header1     = uint8(hex2dec('55'));
    cfg.frame_len   = 348;
    cfg.n_sensors   = 18;
    cfg.imu_bytes   = 19;
    cfg.off_counter = 3;
    cfg.off_samples = 5;
    cfg.off_crc     = 347;
    cfg.payload_len = 344;
    cfg.frames_per_batch = 16;
    cfg.batches_per_sec  = 100;
    cfg.batch_bytes      = cfg.frames_per_batch * cfg.frame_len;
    cfg.expected_bps     = cfg.batch_bytes * cfg.batches_per_sec;
    cfg.accel_lsb   = cfg.accel_fs_g  / (2^19);
    cfg.gyro_lsb    = cfg.gyro_fs_dps / (2^19);

    %% ===================== PATHS =====================
    this_file = mfilename('fullpath');
    if isempty(this_file)
        script_dir = string(pwd);
    else
        script_dir = string(fileparts(this_file));
    end

    if strlength(string(cfg.python_script)) == 0
        cfg.python_script = fullfile(script_dir, 'miib_capture.py');
    end

    if ~isfile(cfg.python_exe)
        error('MIIB_run_py_allplots:python', 'Python executable not found: %s', cfg.python_exe);
    end
    if ~isfile(cfg.python_script)
        error('MIIB_run_py_allplots:python_script', 'Python capture script not found: %s', cfg.python_script);
    end

    ts = datestr(now,'yyyymmdd_HHMMSS');
    raw_filename = sprintf('miib_raw_py_%s.bin', ts);
    raw_path = fullfile(script_dir, raw_filename);
    plot_dir = fullfile(script_dir, sprintf('miib_plots_%s', ts));
    if cfg.export_plots && ~exist(plot_dir, 'dir')
        mkdir(plot_dir);
    end

    %% ===================== RUN PYTHON CAPTURE =====================
    fprintf('[MIIB] Python capture start...\n');
    fprintf('[MIIB] Python exe   : %s\n', cfg.python_exe);
    fprintf('[MIIB] Python script: %s\n', cfg.python_script);
    fprintf('[MIIB] Output file  : %s\n', raw_path);

    cmd = sprintf('"%s" "%s" --port %s --baud %d --duration %.6f --outfile "%s"', ...
        char(cfg.python_exe), char(cfg.python_script), char(cfg.port), ...
        cfg.baudrate, cfg.duration, char(raw_path));

    [status, cmdout] = system(cmd);
    fprintf('%s\n', cmdout);

    if status ~= 0
        error('MIIB_run_py_allplots:python_failed', 'Python capture failed with status %d', status);
    end
    if ~isfile(raw_path)
        error('MIIB_run_py_allplots:no_raw', 'Raw file not created: %s', raw_path);
    end

    d = dir(raw_path);
    fprintf('[MIIB] RAW file size: %d bytes\n', double(d.bytes));

    %% ===================== OFFLINE READ =====================
    fid = fopen(raw_path, 'rb');
    if fid < 0
        error('MIIB_run_py_allplots:file_open', 'Cannot open raw file: %s', raw_path);
    end
    raw = fread(fid, inf, 'uint8=>uint8').';
    fclose(fid);
    fprintf('[MIIB] Offline read complete: %d bytes\n', numel(raw));

    %% ===================== PARSING =====================
    fprintf('[MIIB] Parsing raw buffer...\n');
    crc_table = build_crc16_table();

    is_hdr = (raw(1:end-1) == cfg.header0) & (raw(2:end) == cfg.header1);
    hdr_positions = find(is_hdr);
    hdr_positions = hdr_positions(hdr_positions + cfg.frame_len - 1 <= numel(raw));
    fprintf('[MIIB] Candidate headers: %d\n', numel(hdr_positions));

    n_cand = numel(hdr_positions);
    frame_starts = zeros(1, n_cand, 'uint32');
    n_frames = 0;
    i = 1;

    while i <= n_cand
        pos = hdr_positions(i);
        frame = raw(pos : pos + cfg.frame_len - 1);
        payload = frame(cfg.off_counter : cfg.off_counter + cfg.payload_len - 1);
        crc_calc = crc16_ccitt_table(payload, crc_table);
        crc_recv = double(frame(cfg.off_crc)) + double(frame(cfg.off_crc + 1)) * 256;

        if crc_calc == crc_recv
            n_frames = n_frames + 1;
            frame_starts(n_frames) = pos;
            next_pos = pos + cfg.frame_len;
            j = find(hdr_positions >= next_pos, 1, 'first');
            if isempty(j)
                break;
            end
            i = j;
        else
            i = i + 1;
        end
    end

    frame_starts = frame_starts(1:n_frames);
    fprintf('[MIIB] Valid CRC frames: %d\n', n_frames);
    fprintf('[MIIB] Rejected candidates: %d\n', n_cand - n_frames);

    if n_frames == 0
        error('MIIB_run_py_allplots:nodata', 'No valid frame found in raw file.');
    end

    %% ===================== UNPACK =====================
    counters = zeros(n_frames, 1);
    accel    = nan(n_frames, cfg.n_sensors, 3);
    gyro     = nan(n_frames, cfg.n_sensors, 3);
    temp     = nan(n_frames, cfg.n_sensors);
    tstamp   = nan(n_frames, cfg.n_sensors);
    invalid  = false(n_frames, cfg.n_sensors);

    for fidx = 1:n_frames
        pos = frame_starts(fidx);
        frame = raw(pos : pos + cfg.frame_len - 1);
        counters(fidx) = double(frame(cfg.off_counter)) + double(frame(cfg.off_counter + 1)) * 256;

        for k = 1:cfg.n_sensors
            off = cfg.off_samples + (k - 1) * cfg.imu_bytes;
            blk = frame(off : off + cfg.imu_bytes - 1);

            if all(blk == 0)
                invalid(fidx, k) = true;
                continue;
            end

            ax20 = bitor(bitor(bitshift(uint32(blk(1)),12),  bitshift(uint32(blk(2)),4)),  bitshift(uint32(bitand(blk(17),240)),-4));
            ay20 = bitor(bitor(bitshift(uint32(blk(3)),12),  bitshift(uint32(blk(4)),4)),  bitshift(uint32(bitand(blk(18),240)),-4));
            az20 = bitor(bitor(bitshift(uint32(blk(5)),12),  bitshift(uint32(blk(6)),4)),  bitshift(uint32(bitand(blk(19),240)),-4));
            gx20 = bitor(bitor(bitshift(uint32(blk(7)),12),  bitshift(uint32(blk(8)),4)),  uint32(bitand(blk(17),15)));
            gy20 = bitor(bitor(bitshift(uint32(blk(9)),12),  bitshift(uint32(blk(10)),4)), uint32(bitand(blk(18),15)));
            gz20 = bitor(bitor(bitshift(uint32(blk(11)),12), bitshift(uint32(blk(12)),4)), uint32(bitand(blk(19),15)));

            accel(fidx,k,1) = to_signed20(ax20) * cfg.accel_lsb;
            accel(fidx,k,2) = to_signed20(ay20) * cfg.accel_lsb;
            accel(fidx,k,3) = to_signed20(az20) * cfg.accel_lsb;
            gyro(fidx,k,1)  = to_signed20(gx20) * cfg.gyro_lsb;
            gyro(fidx,k,2)  = to_signed20(gy20) * cfg.gyro_lsb;
            gyro(fidx,k,3)  = to_signed20(gz20) * cfg.gyro_lsb;

            temp_raw = int16(bitshift(uint16(blk(13)),8) + uint16(blk(14)));
            temp(fidx,k) = double(temp_raw) * cfg.temp_scale + cfg.temp_offset;
            tstamp(fidx,k) = double(bitshift(uint16(blk(15)),8) + uint16(blk(16)));
        end
    end

    %% ===================== TIME AXIS =====================
    if n_frames > 1
        dcnt = mod(diff(counters), 65536);
        valid_steps = dcnt(dcnt > 0);
        if isempty(valid_steps)
            dt_frames = 1;
        else
            dt_frames = median(valid_steps);
        end
    else
        dt_frames = 1;
    end
    t = (0:n_frames-1)' * double(dt_frames) / 100.0;

    %% ===================== STATS =====================
    if n_frames > 1
        dcnt = mod(diff(counters), 65536);
        counter_gaps = sum(dcnt ~= 1);
        missing_frames = sum(max(double(dcnt) - 1, 0));
    else
        counter_gaps = 0;
        missing_frames = 0;
    end

    valid_per_sensor = sum(~invalid, 1);
    invalid_per_sensor = sum(invalid, 1);

    fprintf('\n===== MIIB summary =====\n');
    fprintf('RAW file:              %s\n', raw_path);
    fprintf('RAW bytes:             %d\n', numel(raw));
    fprintf('Valid CRC frames:      %d\n', n_frames);
    fprintf('Counter gap events:    %d\n', counter_gaps);
    fprintf('Missing frame count:   %d\n', missing_frames);
    for k = 1:cfg.n_sensors
        fprintf('sensor%02d: valid=%d invalid=%d\n', k-1, valid_per_sensor(k), invalid_per_sensor(k));
    end

    %% ===================== FIGURE VISIBILITY =====================
    if cfg.show_figures
        fig_vis = 'on';
    else
        fig_vis = 'off';
    end

    figs = gobjects(0);
    exported = strings(0,1);

    %% ===================== ACCEL PLOT =====================
    fig1 = figure('Name','MIIB Accel All Sensors', 'NumberTitle','off', ...
                  'Color','w', 'Position',[40 40 1600 920], 'Visible', fig_vis);
    figs(end+1) = fig1;
    tl1 = tiledlayout(fig1, 6, 3, 'TileSpacing','compact', 'Padding','compact');
    title(tl1, sprintf('Acceleration for all sensors | %d valid frames | %s', n_frames, raw_filename), 'Interpreter','none');
    xlabel(tl1, 'Time [s]');
    ylabel(tl1, 'Acceleration [g]');

    for k = 1:cfg.n_sensors
        ax = nexttile(tl1, k);
        hA = plot(ax, t, squeeze(accel(:,k,1)), 'r-', 'LineWidth', 0.9); hold(ax,'on');
        hB = plot(ax, t, squeeze(accel(:,k,2)), 'b-', 'LineWidth', 0.9);
        hC = plot(ax, t, squeeze(accel(:,k,3)), 'g-', 'LineWidth', 0.9);
        if k == 1
            legend(ax, [hA hB hC], {'Ax','Ay','Az'}, 'Location','best');
        end
        grid(ax,'on');
        box(ax,'on');
        title(ax, sprintf('Sensor %02d | valid %d / %d', k-1, valid_per_sensor(k), n_frames));
        if numel(t) > 1 && t(end) > t(1)
            xlim(ax, [t(1) t(end)]);
        end
    end

    %% ===================== GYRO PLOT =====================
    fig2 = figure('Name','MIIB Gyro All Sensors', 'NumberTitle','off', ...
                  'Color','w', 'Position',[60 60 1600 920], 'Visible', fig_vis);
    figs(end+1) = fig2;
    tl2 = tiledlayout(fig2, 6, 3, 'TileSpacing','compact', 'Padding','compact');
    title(tl2, sprintf('Angular rate for all sensors | %d valid frames | %s', n_frames, raw_filename), 'Interpreter','none');
    xlabel(tl2, 'Time [s]');
    ylabel(tl2, 'Angular rate [dps]');

    for k = 1:cfg.n_sensors
        ax = nexttile(tl2, k);
        hA = plot(ax, t, squeeze(gyro(:,k,1)), 'r-', 'LineWidth', 0.9); hold(ax,'on');
        hB = plot(ax, t, squeeze(gyro(:,k,2)), 'b-', 'LineWidth', 0.9);
        hC = plot(ax, t, squeeze(gyro(:,k,3)), 'g-', 'LineWidth', 0.9);
        if k == 1
            legend(ax, [hA hB hC], {'Gx','Gy','Gz'}, 'Location','best');
        end
        grid(ax,'on');
        box(ax,'on');
        title(ax, sprintf('Sensor %02d | valid %d / %d', k-1, valid_per_sensor(k), n_frames));
        if numel(t) > 1 && t(end) > t(1)
            xlim(ax, [t(1) t(end)]);
        end
    end

    %% ===================== TEMP PLOT =====================
    fig3 = figure('Name','MIIB Temperature All Sensors', 'NumberTitle','off', ...
                  'Color','w', 'Position',[80 80 1600 920], 'Visible', fig_vis);
    figs(end+1) = fig3;
    tl3 = tiledlayout(fig3, 6, 3, 'TileSpacing','compact', 'Padding','compact');
    title(tl3, sprintf('Temperature for all sensors | %d valid frames | %s', n_frames, raw_filename), 'Interpreter','none');
    xlabel(tl3, 'Time [s]');
    ylabel(tl3, 'Temperature [degC]');

    for k = 1:cfg.n_sensors
        ax = nexttile(tl3, k);
        plot(ax, t, temp(:,k), 'k-', 'LineWidth', 1.0);
        grid(ax,'on');
        box(ax,'on');
        title(ax, sprintf('Sensor %02d | valid %d / %d', k-1, valid_per_sensor(k), n_frames));
        if numel(t) > 1 && t(end) > t(1)
            xlim(ax, [t(1) t(end)]);
        end
    end

    %% ===================== TIMESTAMP PLOT =====================
    fig4 = figure('Name','MIIB Timestamp All Sensors', 'NumberTitle','off', ...
                  'Color','w', 'Position',[100 100 1600 920], 'Visible', fig_vis);
    figs(end+1) = fig4;
    tl4 = tiledlayout(fig4, 6, 3, 'TileSpacing','compact', 'Padding','compact');
    title(tl4, sprintf('Timestamp field for all sensors | %d valid frames | %s', n_frames, raw_filename), 'Interpreter','none');
    xlabel(tl4, 'Time [s]');
    ylabel(tl4, 'Timestamp [raw]');

    for k = 1:cfg.n_sensors
        ax = nexttile(tl4, k);
        plot(ax, t, tstamp(:,k), '-', 'Color', [0.45 0.2 0.75], 'LineWidth', 1.0);
        grid(ax,'on');
        box(ax,'on');
        title(ax, sprintf('Sensor %02d | valid %d / %d', k-1, valid_per_sensor(k), n_frames));
        if numel(t) > 1 && t(end) > t(1)
            xlim(ax, [t(1) t(end)]);
        end
    end

    %% ===================== SUMMARY PLOT =====================
    fig5 = figure('Name','MIIB Summary Overview', 'NumberTitle','off', ...
                  'Color','w', 'Position',[120 120 1500 850], 'Visible', fig_vis);
    figs(end+1) = fig5;
    tl5 = tiledlayout(fig5, 2, 2, 'TileSpacing','compact', 'Padding','compact');
    title(tl5, sprintf('MIIB offline overview | %s', raw_filename), 'Interpreter','none');

    ax1 = nexttile(tl5,1);
    bar(ax1, 0:cfg.n_sensors-1, valid_per_sensor, 'FaceColor',[0.1 0.55 0.1]);
    grid(ax1,'on'); box(ax1,'on');
    xlabel(ax1,'Sensor ID'); ylabel(ax1,'Valid frames'); title(ax1,'Valid frames per sensor');

    ax2 = nexttile(tl5,2);
    bar(ax2, 0:cfg.n_sensors-1, invalid_per_sensor, 'FaceColor',[0.8 0.15 0.15]);
    grid(ax2,'on'); box(ax2,'on');
    xlabel(ax2,'Sensor ID'); ylabel(ax2,'Invalid frames'); title(ax2,'Invalid / zero blocks per sensor');

    ax3 = nexttile(tl5,3);
    if numel(counters) > 1
        plot(ax3, mod(diff(counters),65536), 'b-', 'LineWidth', 1.0); hold(ax3,'on');
        yline(ax3, 1, '--k', 'Expected step=1', 'LineWidth', 1.0);
    else
        plot(ax3, counters, 'b-', 'LineWidth', 1.0);
    end
    grid(ax3,'on'); box(ax3,'on');
    xlabel(ax3,'Frame index'); ylabel(ax3,'Delta counter'); title(ax3,'Frame counter delta');

    ax4 = nexttile(tl5,4);
    text(ax4, 0.01, 0.95, sprintf(['File: %s\n' ...
        'Raw bytes: %d\n' ...
        'Valid CRC frames: %d\n' ...
        'Counter gap events: %d\n' ...
        'Missing frames: %d\n' ...
        'Expected throughput: %d B/s\n' ...
        'Port: %s @ %d baud'], ...
        raw_filename, numel(raw), n_frames, counter_gaps, missing_frames, ...
        cfg.expected_bps, cfg.port, cfg.baudrate), ...
        'Units','normalized', 'VerticalAlignment','top', 'FontName','Consolas', 'FontSize',11);
    axis(ax4,'off');
    title(ax4,'Capture summary');

    %% ===================== FORCE DRAW =====================
    drawnow;

    %% ===================== EXPORT =====================
    if cfg.export_plots
        names = { 'accel_all_sensors.png', ...
                  'gyro_all_sensors.png', ...
                  'temperature_all_sensors.png', ...
                  'timestamp_all_sensors.png', ...
                  'summary_overview.png' };
        for iFig = 1:numel(figs)
            out_png = fullfile(plot_dir, names{iFig});
            exportgraphics(figs(iFig), out_png, 'Resolution', 180);
            exported(end+1,1) = string(out_png);
        end
        fprintf('[MIIB] Plot export directory: %s\n', plot_dir);
    end

    %% ===================== SAVE MAT =====================
    mat_path = "";
    if cfg.save_mat
        mat_path = fullfile(script_dir, sprintf('miib_result_%s.mat', ts));
        save(mat_path, 'counters','accel','gyro','temp','tstamp','invalid','raw_path','cfg', '-v7.3');
        fprintf('[MIIB] MAT saved: %s\n', mat_path);
    end

    %% ===================== RESULT =====================
    result.raw_file           = string(raw_path);
    result.raw_bytes          = numel(raw);
    result.counters           = counters;
    result.accel              = accel;
    result.gyro               = gyro;
    result.temp               = temp;
    result.timestamp          = tstamp;
    result.invalid            = invalid;
    result.valid_per_sensor   = valid_per_sensor;
    result.invalid_per_sensor = invalid_per_sensor;
    result.n_frames           = n_frames;
    result.counter_gaps       = counter_gaps;
    result.missing_frames     = missing_frames;
    result.time_s             = t;
    result.cfg                = cfg;
    result.python_stdout      = string(cmdout);
    result.plot_dir           = string(plot_dir);
    result.exported_plots     = exported;
    result.mat_file           = string(mat_path);
    result.figures            = figs;

    if ~cfg.keep_raw
        delete(raw_path);
        fprintf('[MIIB] RAW file deleted after parse.\n');
    end

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
