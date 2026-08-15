function result = MIIB_run_py_allplots(varargin)
% Robust sequential subplot version.
% - skips sensors with zero valid data
% - uses explicit axes handles
% - can force software OpenGL
% - avoids plotting all-NaN / all-empty channels

    p = inputParser;
    addParameter(p, 'port', 'COM3');
    addParameter(p, 'baudrate', 12000000);
    addParameter(p, 'duration', 30);
    addParameter(p, 'python_exe', 'C:\ProgramData\spyder-6\envs\spyder-runtime\python.exe');
    addParameter(p, 'python_script', '');
    addParameter(p, 'accel_fs_g', 32);
    addParameter(p, 'gyro_fs_dps', 4000);
    addParameter(p, 'temp_scale', 1/128);
    addParameter(p, 'temp_offset', 25);
    addParameter(p, 'keep_raw', true);
    addParameter(p, 'export_plots', false);
    addParameter(p, 'show_figures', true);
    addParameter(p, 'save_mat', true);
    addParameter(p, 'force_software_opengl', true);
    parse(p, varargin{:});
    cfg = p.Results;

    if cfg.force_software_opengl
        try
            opengl software;
            fprintf('[MIIB] OpenGL set to software mode.\n');
        catch ME
            fprintf('[MIIB] Could not switch to software OpenGL: %s\n', ME.message);
        end
    end

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
    cfg.accel_lsb        = cfg.accel_fs_g  / (2^19);
    cfg.gyro_lsb         = cfg.gyro_fs_dps / (2^19);

    this_file = mfilename('fullpath');
    if isempty(this_file)
        script_dir = pwd;
    else
        script_dir = fileparts(this_file);
    end

    if isempty(cfg.python_script)
        cfg.python_script = fullfile(script_dir, 'miib_capture.py');
    end

    if ~exist(cfg.python_exe, 'file')
        error('Python executable not found: %s', cfg.python_exe);
    end
    if ~exist(cfg.python_script, 'file')
        error('Python capture script not found: %s', cfg.python_script);
    end

    ts = datestr(now,'yyyymmdd_HHMMSS');
    raw_filename = ['miib_raw_py_' ts '.bin'];
    raw_path = fullfile(script_dir, raw_filename);
    plot_dir = fullfile(script_dir, ['miib_plots_' ts]);
    if cfg.export_plots && ~exist(plot_dir, 'dir')
        mkdir(plot_dir);
    end

    fprintf('[MIIB] Python capture start...\n');
    cmd = sprintf('"%s" "%s" --port %s --baud %d --duration %.6f --outfile "%s"', ...
        cfg.python_exe, cfg.python_script, cfg.port, cfg.baudrate, cfg.duration, raw_path);
    [status, cmdout] = system(cmd);
    fprintf('%s\n', cmdout);
    if status ~= 0
        error('Python capture failed with status %d', status);
    end
    if ~exist(raw_path, 'file')
        error('Raw file not created: %s', raw_path);
    end

    fid = fopen(raw_path, 'rb');
    if fid < 0
        error('Cannot open raw file: %s', raw_path);
    end
    raw = fread(fid, inf, 'uint8=>uint8').';
    fclose(fid);

    crc_table = build_crc16_table();
    is_hdr = (raw(1:end-1) == cfg.header0) & (raw(2:end) == cfg.header1);
    hdr_positions = find(is_hdr);
    hdr_positions = hdr_positions(hdr_positions + cfg.frame_len - 1 <= numel(raw));

    n_cand = numel(hdr_positions);
    frame_starts = zeros(1, n_cand, 'uint32');
    n_frames = 0;
    i = 1;
    while i <= n_cand
        pos = hdr_positions(i);
        frame = raw(pos:pos + cfg.frame_len - 1);
        payload = frame(cfg.off_counter : cfg.off_counter + cfg.payload_len - 1);
        crc_calc = crc16_ccitt_table(payload, crc_table);
        crc_recv = double(frame(cfg.off_crc)) + 256 * double(frame(cfg.off_crc + 1));
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
    if n_frames == 0
        error('No valid frame found in raw file.');
    end

    counters = zeros(n_frames, 1);
    accel    = nan(n_frames, cfg.n_sensors, 3);
    gyro     = nan(n_frames, cfg.n_sensors, 3);
    temp     = nan(n_frames, cfg.n_sensors);
    tstamp   = nan(n_frames, cfg.n_sensors);
    invalid  = false(n_frames, cfg.n_sensors);

    for fidx = 1:n_frames
        pos = frame_starts(fidx);
        frame = raw(pos:pos + cfg.frame_len - 1);
        counters(fidx) = double(frame(cfg.off_counter)) + 256 * double(frame(cfg.off_counter + 1));
        for k = 1:cfg.n_sensors
            off = cfg.off_samples + (k - 1) * cfg.imu_bytes;
            blk = frame(off:off + cfg.imu_bytes - 1);
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
    fprintf('Valid CRC frames: %d\n', n_frames);
    for k = 1:cfg.n_sensors
        fprintf('sensor%02d: valid=%d invalid=%d\n', k-1, valid_per_sensor(k), invalid_per_sensor(k));
    end

    if cfg.show_figures
        fig_vis = 'on';
    else
        fig_vis = 'off';
    end

    figs = [];
    exported = {};

    fig1 = figure('Name','MIIB Accel All Sensors','NumberTitle','off','Color','w','Visible',fig_vis);
    set(fig1, 'Position', [30 30 1600 900]);
    clf(fig1);
    for k = 1:cfg.n_sensors
        ax = subplot(6,3,k,'Parent',fig1);
        cla(ax);
        if valid_per_sensor(k) == 0
            axis(ax,'off');
            text(ax, 0.5, 0.5, sprintf('Sensor %02d\nNO DATA', k-1), 'Units','normalized', 'HorizontalAlignment','center', 'VerticalAlignment','middle', 'FontWeight','bold', 'Color',[0.8 0 0]);
            drawnow;
            continue;
        end
        y1 = squeeze(accel(:,k,1));
        y2 = squeeze(accel(:,k,2));
        y3 = squeeze(accel(:,k,3));
        p1 = plot(ax, t, y1, 'r-', 'LineWidth', 0.8); hold(ax,'on');
        p2 = plot(ax, t, y2, 'b-', 'LineWidth', 0.8);
        p3 = plot(ax, t, y3, 'g-', 'LineWidth', 0.8); hold(ax,'off');
        grid(ax,'on'); box(ax,'on');
        title(ax, sprintf('Sensor %02d | valid %d / %d', k-1, valid_per_sensor(k), n_frames));
        if numel(t) > 1 && t(end) > t(1)
            xlim(ax,[t(1) t(end)]);
        end
        if k == 1
            legend(ax,[p1 p2 p3],{'Ax','Ay','Az'},'Location','best');
        end
        drawnow;
    end
    figs(end+1) = fig1;

    fig2 = figure('Name','MIIB Gyro All Sensors','NumberTitle','off','Color','w','Visible',fig_vis);
    set(fig2, 'Position', [50 50 1600 900]);
    clf(fig2);
    for k = 1:cfg.n_sensors
        ax = subplot(6,3,k,'Parent',fig2);
        cla(ax);
        if valid_per_sensor(k) == 0
            axis(ax,'off');
            text(ax, 0.5, 0.5, sprintf('Sensor %02d\nNO DATA', k-1), 'Units','normalized', 'HorizontalAlignment','center', 'VerticalAlignment','middle', 'FontWeight','bold', 'Color',[0.8 0 0]);
            drawnow;
            continue;
        end
        y1 = squeeze(gyro(:,k,1));
        y2 = squeeze(gyro(:,k,2));
        y3 = squeeze(gyro(:,k,3));
        p1 = plot(ax, t, y1, 'r-', 'LineWidth', 0.8); hold(ax,'on');
        p2 = plot(ax, t, y2, 'b-', 'LineWidth', 0.8);
        p3 = plot(ax, t, y3, 'g-', 'LineWidth', 0.8); hold(ax,'off');
        grid(ax,'on'); box(ax,'on');
        title(ax, sprintf('Sensor %02d | valid %d / %d', k-1, valid_per_sensor(k), n_frames));
        if numel(t) > 1 && t(end) > t(1)
            xlim(ax,[t(1) t(end)]);
        end
        if k == 1
            legend(ax,[p1 p2 p3],{'Gx','Gy','Gz'},'Location','best');
        end
        drawnow;
    end
    figs(end+1) = fig2;

    fig3 = figure('Name','MIIB Temperature All Sensors','NumberTitle','off','Color','w','Visible',fig_vis);
    set(fig3, 'Position', [70 70 1600 900]);
    clf(fig3);
    for k = 1:cfg.n_sensors
        ax = subplot(6,3,k,'Parent',fig3);
        cla(ax);
        if valid_per_sensor(k) == 0
            axis(ax,'off');
            text(ax, 0.5, 0.5, sprintf('Sensor %02d\nNO DATA', k-1), 'Units','normalized', 'HorizontalAlignment','center', 'VerticalAlignment','middle', 'FontWeight','bold', 'Color',[0.8 0 0]);
            drawnow;
            continue;
        end
        y = temp(:,k);
        plot(ax, t, y, 'k-', 'LineWidth', 0.9);
        grid(ax,'on'); box(ax,'on');
        title(ax, sprintf('Sensor %02d | valid %d / %d', k-1, valid_per_sensor(k), n_frames));
        if numel(t) > 1 && t(end) > t(1)
            xlim(ax,[t(1) t(end)]);
        end
        drawnow;
    end
    figs(end+1) = fig3;

    fig4 = figure('Name','MIIB Timestamp All Sensors','NumberTitle','off','Color','w','Visible',fig_vis);
    set(fig4, 'Position', [90 90 1600 900]);
    clf(fig4);
    for k = 1:cfg.n_sensors
        ax = subplot(6,3,k,'Parent',fig4);
        cla(ax);
        if valid_per_sensor(k) == 0
            axis(ax,'off');
            text(ax, 0.5, 0.5, sprintf('Sensor %02d\nNO DATA', k-1), 'Units','normalized', 'HorizontalAlignment','center', 'VerticalAlignment','middle', 'FontWeight','bold', 'Color',[0.8 0 0]);
            drawnow;
            continue;
        end
        y = tstamp(:,k);
        plot(ax, t, y, 'm-', 'LineWidth', 0.9);
        grid(ax,'on'); box(ax,'on');
        title(ax, sprintf('Sensor %02d | valid %d / %d', k-1, valid_per_sensor(k), n_frames));
        if numel(t) > 1 && t(end) > t(1)
            xlim(ax,[t(1) t(end)]);
        end
        drawnow;
    end
    figs(end+1) = fig4;

    fig5 = figure('Name','MIIB Summary Overview','NumberTitle','off','Color','w','Visible',fig_vis);
    set(fig5, 'Position', [110 110 1400 850]);
    clf(fig5);
    ax1 = subplot(2,2,1,'Parent',fig5);
    bar(ax1, 0:cfg.n_sensors-1, valid_per_sensor, 'FaceColor',[0.1 0.55 0.1]);
    grid(ax1,'on'); box(ax1,'on');
    xlabel(ax1,'Sensor ID'); ylabel(ax1,'Valid frames'); title(ax1,'Valid frames per sensor');
    ax2 = subplot(2,2,2,'Parent',fig5);
    bar(ax2, 0:cfg.n_sensors-1, invalid_per_sensor, 'FaceColor',[0.8 0.15 0.15]);
    grid(ax2,'on'); box(ax2,'on');
    xlabel(ax2,'Sensor ID'); ylabel(ax2,'Invalid frames'); title(ax2,'Invalid / zero blocks per sensor');
    ax3 = subplot(2,2,3,'Parent',fig5);
    if numel(counters) > 1
        plot(ax3, mod(diff(counters),65536), 'b-', 'LineWidth', 1.0); hold(ax3,'on');
        yline(ax3, 1, '--k', 'Expected step=1', 'LineWidth', 1.0); hold(ax3,'off');
    else
        plot(ax3, counters, 'b-', 'LineWidth', 1.0);
    end
    grid(ax3,'on'); box(ax3,'on');
    xlabel(ax3,'Frame index'); ylabel(ax3,'Delta counter'); title(ax3,'Frame counter delta');
    ax4 = subplot(2,2,4,'Parent',fig5);
    axis(ax4,'off');
    text(ax4, 0.01, 0.95, sprintf(['File: %s\nValid CRC frames: %d\nCounter gap events: %d\nMissing frames: %d'], raw_filename, n_frames, counter_gaps, missing_frames), 'Units','normalized', 'VerticalAlignment','top', 'FontName','Consolas', 'FontSize',11);
    title(ax4,'Capture summary');
    figs(end+1) = fig5;
    drawnow;

    if cfg.export_plots
        names = {'accel_all_sensors.png','gyro_all_sensors.png','temperature_all_sensors.png','timestamp_all_sensors.png','summary_overview.png'};
        for iFig = 1:length(figs)
            out_png = fullfile(plot_dir, names{iFig});
            saveas(figs(iFig), out_png);
            exported{end+1,1} = out_png;
        end
    end

    mat_path = '';
    if cfg.save_mat
        mat_path = fullfile(script_dir, ['miib_result_' ts '.mat']);
        save(mat_path, 'counters','accel','gyro','temp','tstamp','invalid','raw_path','cfg', '-v7.3');
    end

    result.raw_file = raw_path;
    result.raw_bytes = numel(raw);
    result.counters = counters;
    result.accel = accel;
    result.gyro = gyro;
    result.temp = temp;
    result.timestamp = tstamp;
    result.invalid = invalid;
    result.valid_per_sensor = valid_per_sensor;
    result.invalid_per_sensor = invalid_per_sensor;
    result.n_frames = n_frames;
    result.counter_gaps = counter_gaps;
    result.missing_frames = missing_frames;
    result.time_s = t;
    result.cfg = cfg;
    result.python_stdout = cmdout;
    result.plot_dir = plot_dir;
    result.exported_plots = exported;
    result.mat_file = mat_path;
    result.figures = figs;

    if ~cfg.keep_raw
        delete(raw_path);
    end
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
