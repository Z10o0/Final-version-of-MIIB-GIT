%% ===================== НАСТРОЙКИ =====================
port      = "COM3";
baud      = 12000000;
duration  = 30;                 % сек
workDir   = pwd;
pyScript  = fullfile(workDir, "miib_capture_only.py");

pe = pyenv;                     % берём python.exe из окружения MATLAB
pyExe = pe.Executable;

ts = datestr(now, 'yyyymmdd_HHMMSS');
outfile = fullfile(workDir, "miib_raw_" + ts + ".bin");

%% ===================== ВЫЗОВ PYTHON =====================
cmd = sprintf('"%s" "%s" --port %s --baud %d --duration %g --outfile "%s"', ...
    pyExe, pyScript, port, baud, duration, outfile);
disp("Запуск: " + cmd);
[status, cmdout] = system(cmd);
disp(cmdout);
if status ~= 0
    error("Python capture завершился с ошибкой (status=%d)", status);
end

metaFile = strrep(outfile, ".bin", ".json");
meta = jsondecode(fileread(metaFile));
elapsedSec = meta.elapsed_sec;
fprintf("Захвачено %d байт за %.3f с\n", meta.total_bytes, elapsedSec);

%% ===================== ПРОТОКОЛ (константы) =====================
FRAME_LEN   = 690; N_SENSORS = 36; IMU_BYTES = 19;
PAYLOAD_LEN = 686;                       % байты [2..687] под CRC
ACCEL_LSB = 32.0/2^19;  GYRO_LSB = 4000.0/2^19;
TEMP_SCALE = 1/128; TEMP_OFFSET = 25;

%% ===================== ЧТЕНИЕ RAW ФАЙЛА =====================
fid = fopen(outfile,'rb'); raw = fread(fid,'*uint8'); fclose(fid);
L = numel(raw);

isHdr = (raw(1:end-1)==170) & (raw(2:end)==85);   % 0xAA 0x55
hdrPos = find(isHdr);                              % 1-based позиции header
hdrPos = hdrPos(hdrPos + FRAME_LEN - 1 <= L);

%% ===================== ВЕКТОРИЗОВАННЫЙ CRC16-CCITT-FALSE =====================
table = uint32(zeros(256,1));
poly = uint32(4129);                               % 0x1021
for i = 0:255
    crc = bitshift(uint32(i),8);
    for b = 1:8
        if bitand(crc,32768) ~= 0
            crc = bitxor(bitshift(crc,1), poly);
        else
            crc = bitshift(crc,1);
        end
        crc = bitand(crc,65535);
    end
    table(i+1) = crc;
end

nCand = numel(hdrPos);
offsets = uint32(0:PAYLOAD_LEN-1);
idxMat = uint32(hdrPos) + 2 + offsets;             % nCand x 686, global idx под CRC
byteMat = raw(idxMat);

crc = uint32(65535)*ones(nCand,1);
for col = 1:PAYLOAD_LEN
    bCol = uint32(byteMat(:,col));
    idxT = bitand(bitxor(bitshift(crc,-8), bCol), 255) + 1;
    crc = bitand(bitxor(bitshift(crc,8), table(idxT)), 65535);
end
crcExpected = uint32(raw(hdrPos+688)) + bitshift(uint32(raw(hdrPos+689)),8);
crcOk = (crc == crcExpected);

%% ===================== РЕЗИНК: убираем перекрывающиеся кандидаты =====================
candPos = hdrPos(crcOk);
validStarts = zeros(numel(candPos),1);
nValid = 0; prevEnd = -1;
for i = 1:numel(candPos)
    p = candPos(i);
    if p > prevEnd
        nValid = nValid + 1;
        validStarts(nValid) = p;
        prevEnd = p + FRAME_LEN - 1;
    end
end
validStarts = validStarts(1:nValid);
nF = nValid;
fprintf("Valid CRC frames: %d\n", nF);
assert(nF > 0, "Нет ни одного валидного кадра");

%% ===================== РАСПАКОВКА ДАННЫХ (векторизовано) =====================
sensorOffsets = uint32((0:N_SENSORS-1)*IMU_BYTES);
idxBase = uint32(validStarts) + 4 + sensorOffsets;   % nF x 36

B = cell(1,19);
for j = 0:18
    B{j+1} = double(raw(idxBase + uint32(j)));       % nF x 36
end

hi16 = floor(B{17}/16); lo16 = mod(B{17},16);
hi17 = floor(B{18}/16); lo17 = mod(B{18},16);
hi18 = floor(B{19}/16); lo18 = mod(B{19},16);

ax20 = B{1}*4096 + B{2}*16 + hi16;
ay20 = B{3}*4096 + B{4}*16 + hi17;
az20 = B{5}*4096 + B{6}*16 + hi18;
gx20 = B{7}*4096 + B{8}*16 + lo16;
gy20 = B{9}*4096 + B{10}*16 + lo17;
gz20 = B{11}*4096 + B{12}*16 + lo18;

s20 = @(x) x - (x >= 2^19)*2^20;

accelX = s20(ax20)*ACCEL_LSB; accelY = s20(ay20)*ACCEL_LSB; accelZ = s20(az20)*ACCEL_LSB;
gyroX  = s20(gx20)*GYRO_LSB;  gyroY  = s20(gy20)*GYRO_LSB;  gyroZ  = s20(gz20)*GYRO_LSB;

tRaw = B{13}*256 + B{14};
tRaw(tRaw >= 32768) = tRaw(tRaw >= 32768) - 65536;
temp = tRaw*TEMP_SCALE + TEMP_OFFSET;

tstamp = B{15}*256 + B{16};

invalid = false(nF, N_SENSORS);
sumBytes = zeros(nF, N_SENSORS);
for j = 1:19
    sumBytes = sumBytes + B{j};
end
invalid = (sumBytes == 0);

accelX(invalid)=NaN; accelY(invalid)=NaN; accelZ(invalid)=NaN;
gyroX(invalid)=NaN; gyroY(invalid)=NaN; gyroZ(invalid)=NaN;
temp(invalid)=NaN; tstamp(invalid)=NaN;

%% ===================== ВРЕМЕННАЯ ОСЬ =====================
dt = elapsedSec / nF;
t = (0:nF-1)' * dt;
validPer = sum(~invalid,1);
fprintf("Frame rate: %.1f fps, duration %.2f s\n", 1/dt, t(end));

%% ===================== ГРАФИКИ (36 датчиков, сетка 6x6) =====================
plotGrid3(t, accelX, accelY, accelZ, validPer, nF, "Accelerometer, все 36 датчиков [g]");
plotGrid1(t, gyroX, gyroY, gyroZ, validPer, nF, "Gyroscope, все 36 датчиков [deg/s]"); % см. ниже — тоже 3 линии
plotGrid1s(t, temp, validPer, nF, "Temperature, все 36 датчиков [C]");
plotGrid1s(t, tstamp, validPer, nF, "Timestamp, все 36 датчиков [LSB]");

figure('Name','MIIB Summary','Color','w');
subplot(1,2,1); bar(validPer); title('Valid frames / sensor'); xlabel('Sensor'); grid on;
subplot(1,2,2); bar(sum(invalid,1)); title('Invalid frames / sensor'); xlabel('Sensor'); grid on;

%% ===================== ЛОКАЛЬНЫЕ ФУНКЦИИ =====================
function plotGrid3(t, X, Y, Z, validPer, nF, ttl)
    figure('Name', ttl, 'Color','w','Position',[50 50 1600 1200]);
    tl = tiledlayout(6,6,'TileSpacing','compact','Padding','compact');
    title(tl, ttl);
    for k = 1:36
        nexttile;
        if validPer(k) == 0
            text(0.5,0.5,'NO DATA','HorizontalAlignment','center'); axis off;
        else
            plot(t, X(:,k), 'r', t, Y(:,k), 'b', t, Z(:,k), 'g', 'LineWidth', 0.7);
        end
        title(sprintf('S%02d  %d/%d', k-1, validPer(k), nF), 'FontSize', 7);
        set(gca,'FontSize',6);
    end
end

function plotGrid1(t, X, Y, Z, validPer, nF, ttl)
    plotGrid3(t, X, Y, Z, validPer, nF, ttl);
end

function plotGrid1s(t, D, validPer, nF, ttl)
    figure('Name', ttl, 'Color','w','Position',[50 50 1600 1200]);
    tl = tiledlayout(6,6,'TileSpacing','compact','Padding','compact');
    title(tl, ttl);
    for k = 1:36
        nexttile;
        if validPer(k) == 0
            text(0.5,0.5,'NO DATA','HorizontalAlignment','center'); axis off;
        else
            plot(t, D(:,k), 'Color', [1 0.65 0], 'LineWidth', 0.7);
        end
        title(sprintf('S%02d  %d/%d', k-1, validPer(k), nF), 'FontSize', 7);
        set(gca,'FontSize',6);
    end
end