% MIIB_parser_fixed.m — ICM-45686 UART telemetry parser
% Packet: AA 55 | 512-byte payload | CRC16 LE | 0D 0A  (518 bytes)
%
% ICM_Sample_t (28 bytes):
%   accel_x/y/z  int32 LE → g   = (val >> 12) / 32768 * 16
%   gyro_x/y/z   int32 LE → dps = (val >> 12) / 32768 * 2000
%   temp_raw     int16 LE → °C  = val / 2 + 25
%   timestamp    uint16 LE → µs

close all
clearvars
clc

%% CONFIG
PORT      = 'COM3';
BAUD      = 5000000;
N_SENSORS = 18;
PKT_TOTAL = 518;
PAYLOAD_N = 512;
SCALE_A   = 16.0   / 32768.0;
SCALE_G   = 2000.0 / 32768.0;
TIMEOUT_S = 10;
FRAME_HZ  = 640;

%% OPEN PORT
s = serialport(PORT, BAUD, 'Timeout', 2);
flush(s);
fprintf('Listening on %s for %d s...\n', PORT, TIMEOUT_S);

%% CAPTURE
raw = uint8([]);
t0  = tic;
while toc(t0) < TIMEOUT_S
    av = s.NumBytesAvailable;
    if av > 0
        chunk = read(s, av, 'uint8');
        raw   = [raw; uint8(chunk(:))]; %#ok<AGROW>
    else
        pause(0.001);
    end
end
delete(s);
fprintf('Received %d bytes, parsing...\n', numel(raw));

%% PARSE
frames       = struct('counter',{},'mask',{},'samples',{});
idx          = 1;
n_raw        = numel(raw);
bad_crc      = 0;
bad_footer   = 0;
header_hits  = 0;

while idx <= n_raw - 1
    hdr_rel = find(raw(idx:end-1) == hex2dec('AA') & raw(idx+1:end) == hex2dec('55'), 1, 'first');
    if isempty(hdr_rel)
        break;
    end

    idx = idx + hdr_rel - 1;
    header_hits = header_hits + 1;

    if idx + PKT_TOTAL - 1 > n_raw
        break;
    end

    pkt = raw(idx : idx + PKT_TOTAL - 1);

    if pkt(517) ~= hex2dec('0D') || pkt(518) ~= hex2dec('0A')
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

    f = struct();
    f.counter = typecast(uint8(payload(1:4)), 'uint32');
    f.mask    = typecast(uint8(payload(5:8)), 'uint32');

    samp = zeros(N_SENSORS, 8); % [ax ay az gx gy gz temp ts]
    for sid = 1:N_SENSORS
        b = 9 + (sid-1)*28;

        ax = typecast(uint8(payload(b    : b+3 )), 'int32');
        ay = typecast(uint8(payload(b+4  : b+7 )), 'int32');
        az = typecast(uint8(payload(b+8  : b+11 )), 'int32');
        gx = typecast(uint8(payload(b+12 : b+15 )), 'int32');
        gy = typecast(uint8(payload(b+16 : b+19 )), 'int32');
        gz = typecast(uint8(payload(b+20 : b+23 )), 'int32');
        tr = typecast(uint8(payload(b+24 : b+25 )), 'int16');
        ts = typecast(uint8(payload(b+26 : b+27 )), 'uint16');

        samp(sid,:) = [double(ax)/4096*SCALE_A, ...
                       double(ay)/4096*SCALE_A, ...
                       double(az)/4096*SCALE_A, ...
                       double(gx)/4096*SCALE_G, ...
                       double(gy)/4096*SCALE_G, ...
                       double(gz)/4096*SCALE_G, ...
                       double(tr)/2 + 25, ...
                       double(ts)];
    end

    f.samples = samp;
    frames(end+1) = f; %#ok<AGROW>
    idx = idx + PKT_TOTAL;
end

fprintf('Header hits: %d\n', header_hits);
fprintf('Parsed %d frames, %d bad CRC, %d bad footer\n', numel(frames), bad_crc, bad_footer);
if isempty(frames)
    error('No valid frames. Check baud rate, RS-485 DE/RE timing, packet format, or CRC settings.');
end

%% ASSEMBLE
nF   = numel(frames);
t    = (0:nF-1) / FRAME_HZ;

ax   = zeros(nF,N_SENSORS); ay   = zeros(nF,N_SENSORS);
az   = zeros(nF,N_SENSORS); gx   = zeros(nF,N_SENSORS);
gy   = zeros(nF,N_SENSORS); gz   = zeros(nF,N_SENSORS);
temp = zeros(nF,N_SENSORS); ts0  = zeros(nF,N_SENSORS);
mask = zeros(nF,1,'uint32'); ctr = zeros(nF,1,'uint32');

for fi = 1:nF
    ax(fi,:)   = frames(fi).samples(:,1)';
    ay(fi,:)   = frames(fi).samples(:,2)';
    az(fi,:)   = frames(fi).samples(:,3)';
    gx(fi,:)   = frames(fi).samples(:,4)';
    gy(fi,:)   = frames(fi).samples(:,5)';
    gz(fi,:)   = frames(fi).samples(:,6)';
    temp(fi,:) = frames(fi).samples(:,7)';
    ts0(fi,:)  = frames(fi).samples(:,8)';
    mask(fi)   = frames(fi).mask;
    ctr(fi)    = frames(fi).counter;
end

%% PLOTS
colors = lines(N_SENSORS);

figure('Name','Accel X [g]','NumberTitle','off'); hold on;
for k = 1:N_SENSORS
    if any(ax(:,k) ~= 0)
        plot(t, ax(:,k), 'Color', colors(k,:), 'DisplayName', sprintf('S%d', k-1));
    end
end
xlabel('Time [s]'); ylabel('[g]'); title('Accel X'); legend('Location','best'); grid on;

figure('Name','Gyro Z [dps]','NumberTitle','off'); hold on;
for k = 1:N_SENSORS
    if any(gz(:,k) ~= 0)
        plot(t, gz(:,k), 'Color', colors(k,:), 'DisplayName', sprintf('S%d', k-1));
    end
end
xlabel('Time [s]'); ylabel('[dps]'); title('Gyro Z'); legend('Location','best'); grid on;

figure('Name','Temperature [°C]','NumberTitle','off'); hold on;
for k = 1:N_SENSORS
    if any(temp(:,k) ~= 0)
        plot(t, temp(:,k), 'Color', colors(k,:), 'DisplayName', sprintf('S%d', k-1));
    end
end
xlabel('Time [s]'); ylabel('[°C]'); title('Temperature'); legend('Location','best'); grid on;

figure('Name','Sensor mask','NumberTitle','off');
bits = zeros(N_SENSORS, nF);
for fi = 1:nF
    for sid = 0:N_SENSORS-1
        bits(sid+1, fi) = bitget(mask(fi), sid+1);
    end
end
imagesc(t, 0:N_SENSORS-1, bits);
colormap([0.85 0.2 0.2; 0.2 0.8 0.2]);
xlabel('Time [s]'); ylabel('Sensor ID');
title('Sensor mask (green = valid, red = missing)');
yticks(0:N_SENSORS-1); grid on;

figure('Name','Frame counter','NumberTitle','off');
plot(t, double(ctr), 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('Counter'); title('Frame counter'); grid on;

%% CRC16 CCITT helper (poly=0x1021, init=0xFFFF)
function crc = crc16ccitt(data)
    data = uint8(data(:));
    crc = uint16(hex2dec('FFFF'));
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