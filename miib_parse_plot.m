%% ===================== НАСТРОЙКИ СКРИПТА =====================
clear; clc; close all;

binFilePath      = 'miib_raw_20260831_142519.bin';
outputH5Path     = 'miib_data_calib_temp_2h.h5';
chunkMb          = 500;                             % Размер порции чтения в ОЗУ (МБ)

%% ===================== ПОДГОТОВКА ФАЙЛА =====================
if exist(outputH5Path, 'file')
    fprintf('Удаление старого файла: %s\n', outputH5Path);
    delete(outputH5Path);
end

%% ===================== КОНСТАНТЫ ПРОТОКОЛА =====================
FRAME_LEN   = 690;
N_SENSORS   = 36;
IMU_BYTES   = 19;
PAYLOAD_LEN = 686;

ACCEL_LSB   = single(32.0 / 2^19);   % [g]
GYRO_LSB    = single(4000.0 / 2^19); % [deg/s]
TEMP_SCALE  = single(1.0 / 128.0);
TEMP_OFFSET = single(25.0);

%% ===================== ТАБЛИЦА CRC16-CCITT =====================
crcTable = uint32(zeros(256, 1));
poly = uint32(4129); % 0x1021
for i = 0:255
    crcVal = bitshift(uint32(i), 8);
    for b = 1:8
        if bitand(crcVal, 32768) ~= 0
            crcVal = bitxor(bitshift(crcVal, 1), poly);
        else
            crcVal = bitshift(crcVal, 1);
        end
        crcVal = bitand(crcVal, 65535);
    end
    crcTable(i + 1) = crcVal;
end

%% ===================== ИНИЦИАЛИЗАЦИЯ HDF5 СО СЖАТИЕМ =====================
% ChunkSize = 30000 кадров (~3 МБ на чанк) обеспечивает оптимальное окно компрессии
H5_CHUNK_FRAMES = 30000;
DEFLATE_LEVEL   = 4;     % Уровень сжатия (1..9): 4 дает оптимум степени и скорости

% 1. Акселерометры (3 x 36 x Inf)
h5create(outputH5Path, '/accel', [3, N_SENSORS, Inf], ...
    'ChunkSize', [3, N_SENSORS, H5_CHUNK_FRAMES], ...
    'Datatype', 'single', ...
    'Deflate', DEFLATE_LEVEL, ...
    'Shuffle', true);

% 2. Гироскопы (3 x 36 x Inf)
h5create(outputH5Path, '/gyro', [3, N_SENSORS, Inf], ...
    'ChunkSize', [3, N_SENSORS, H5_CHUNK_FRAMES], ...
    'Datatype', 'single', ...
    'Deflate', DEFLATE_LEVEL, ...
    'Shuffle', true);

% 3. Температура (36 x Inf)
h5create(outputH5Path, '/temp', [N_SENSORS, Inf], ...
    'ChunkSize', [N_SENSORS, H5_CHUNK_FRAMES], ...
    'Datatype', 'single', ...
    'Deflate', DEFLATE_LEVEL, ...
    'Shuffle', true);

% 4. Таймштампы (36 x Inf)
h5create(outputH5Path, '/tstamp', [N_SENSORS, Inf], ...
    'ChunkSize', [N_SENSORS, H5_CHUNK_FRAMES], ...
    'Datatype', 'uint16', ...
    'Deflate', DEFLATE_LEVEL, ...
    'Shuffle', true);

%% ===================== СЧЕТЧИКИ И ПОТОКОВОЕ ЧТЕНИЕ =====================
totalCandidates     = 0;
totalCrcErrors      = 0;
totalOverlapDropped = 0;
totalFramesWritten  = 0;

fid = fopen(binFilePath, 'rb');
if fid == -1
    error('Не удалось открыть файл: %s', binFilePath);
end

fseek(fid, 0, 'eof');
totalBytes = ftell(fid);
fseek(fid, 0, 'bof');

chunkBytes = chunkMb * 1024 * 1024;
carryOverBuf = uint8([]);
partIdx = 1;

fprintf('Старт анализа и конвертации файла: %s (%.2f ГБ)\n', binFilePath, totalBytes / 1e9);
fprintf('Параметры сжатия: Deflate Level = %d, Shuffle Filter = ВКЛ\n\n', DEFLATE_LEVEL);
tStart = tic;

try
    while ~feof(fid) || ~isempty(carryOverBuf)
        newBytes = fread(fid, chunkBytes, '*uint8');
        if isempty(newBytes) && isempty(carryOverBuf)
            break;
        end

        raw = [carryOverBuf; newBytes];
        L = numel(raw);

        if L < FRAME_LEN
            carryOverBuf = raw;
            if feof(fid), break; end
            continue;
        end

        % Поиск заголовков 0xAA 0x55 (170, 85)
        isHdr = (raw(1:end-1) == 170) & (raw(2:end) == 85);
        hdrPos = uint32(find(isHdr));
        hdrPos = hdrPos(:);
        hdrPos = hdrPos(hdrPos + uint32(FRAME_LEN - 1) <= uint32(L));

        if isempty(hdrPos)
            carryOverBuf = raw(max(1, end - FRAME_LEN + 1):end);
            if feof(fid), break; end
            continue;
        end

        nCand = numel(hdrPos);
        totalCandidates = totalCandidates + nCand;

        % Расчет CRC16-CCITT
        offsets = uint32(0:PAYLOAD_LEN-1);
        idxMat = hdrPos + uint32(2) + offsets;
        byteMat = raw(idxMat);

        crc = repmat(uint32(65535), nCand, 1);
        for col = 1:PAYLOAD_LEN
            bCol = uint32(byteMat(:, col));
            idxT = bitand(bitxor(bitshift(crc, -8), bCol), uint32(255)) + uint32(1);
            crc = bitand(bitxor(bitshift(crc, 8), crcTable(idxT)), uint32(65535));
        end
        
        crcExpected = uint32(raw(hdrPos + 688)) + bitshift(uint32(raw(hdrPos + 689)), 8);
        crcOk = (crc == crcExpected);
        
        nCrcErr = sum(~crcOk);
        totalCrcErrors = totalCrcErrors + nCrcErr;

        candPos = hdrPos(crcOk);
        if isempty(candPos)
            carryOverBuf = raw(max(1, end - FRAME_LEN + 1):end);
            if feof(fid), break; end
            continue;
        end

        % Дедупликация перекрывающихся кадров
        validStarts = zeros(numel(candPos), 1, 'uint32');
        nValid = 0;
        prevEnd = uint32(0);
        for i = 1:numel(candPos)
            p = candPos(i);
            if p > prevEnd
                nValid = nValid + 1;
                validStarts(nValid) = p;
                prevEnd = p + uint32(FRAME_LEN - 1);
            else
                totalOverlapDropped = totalOverlapDropped + 1;
            end
        end
        validStarts = validStarts(1:nValid);
        nF = nValid;

        lastConsumed = validStarts(end) + uint32(FRAME_LEN - 1);
        if lastConsumed < L
            carryOverBuf = raw(lastConsumed + 1:end);
        else
            carryOverBuf = uint8([]);
        end

        if nF == 0
            if feof(fid), break; end
            continue;
        end

        %% ===================== РАСПАКОВКА ДАННЫХ =====================
        sensorOffsets = uint32(0:N_SENSORS-1) * uint32(IMU_BYTES);
        idxBase = validStarts + uint32(4) + sensorOffsets;

        B = cell(1, 19);
        for j = 0:18
            B{j+1} = single(raw(idxBase + uint32(j)));
        end

        hi16 = floor(B{17} / 16); lo16 = mod(B{17}, 16);
        hi17 = floor(B{18} / 16); lo17 = mod(B{18}, 16);
        hi18 = floor(B{19} / 16); lo18 = mod(B{19}, 16);

        ax20 = B{1}*4096 + B{2}*16 + hi16;
        ay20 = B{3}*4096 + B{4}*16 + hi17;
        az20 = B{5}*4096 + B{6}*16 + hi18;
        gx20 = B{7}*4096 + B{8}*16 + lo16;
        gy20 = B{9}*4096 + B{10}*16 + lo17;
        gz20 = B{11}*4096 + B{12}*16 + lo18;

        s20 = @(x) x - single(x >= 524288) .* single(1048576);

        accelX = s20(ax20) .* ACCEL_LSB;
        accelY = s20(ay20) .* ACCEL_LSB;
        accelZ = s20(az20) .* ACCEL_LSB;
        gyroX  = s20(gx20)  .* GYRO_LSB;
        gyroY  = s20(gy20)  .* GYRO_LSB;
        gyroZ  = s20(gz20)  .* GYRO_LSB;

        tRaw = B{13}*256 + B{14};
        tRaw(tRaw >= 32768) = tRaw(tRaw >= 32768) - 65536;
        temp = tRaw .* TEMP_SCALE + TEMP_OFFSET;
        tstamp = uint16(B{15}*256 + B{16});

        % Замена нулевых пакетов на NaN
        sumBytes = zeros(nF, N_SENSORS, 'single');
        for j = 1:19, sumBytes = sumBytes + B{j}; end
        invalid = (sumBytes == 0);

        accelX(invalid) = NaN; accelY(invalid) = NaN; accelZ(invalid) = NaN;
        gyroX(invalid)  = NaN; gyroY(invalid)  = NaN; gyroZ(invalid)  = NaN;
        temp(invalid)   = NaN;

        chunkAccel = zeros(3, N_SENSORS, nF, 'single');
        chunkGyro  = zeros(3, N_SENSORS, nF, 'single');
        chunkAccel(1, :, :) = accelX';
        chunkAccel(2, :, :) = accelY';
        chunkAccel(3, :, :) = accelZ';
        chunkGyro(1, :, :)  = gyroX';
        chunkGyro(2, :, :)  = gyroY';
        chunkGyro(3, :, :)  = gyroZ';

        %% ===================== ЗАПИСЬ СО СЖАТИЕМ =====================
        startIdx = totalFramesWritten + 1;
        h5write(outputH5Path, '/accel',  chunkAccel, [1, 1, startIdx], [3, N_SENSORS, nF]);
        h5write(outputH5Path, '/gyro',   chunkGyro,  [1, 1, startIdx], [3, N_SENSORS, nF]);
        h5write(outputH5Path, '/temp',   temp',      [1, startIdx],    [N_SENSORS, nF]);
        h5write(outputH5Path, '/tstamp', tstamp',    [1, startIdx],    [N_SENSORS, nF]);

        totalFramesWritten = totalFramesWritten + nF;
        currBytes = ftell(fid);
        progress = (double(currBytes) / double(totalBytes)) * 100;
        fprintf('  [Блок %04d] Записано: %d | Ошибок CRC в блоке: %d | Прогресс: %.1f%%\n', ...
            partIdx, nF, nCrcErr, progress);
        partIdx = partIdx + 1;
    end
catch ME
    fclose(fid);
    rethrow(ME);
end

fclose(fid);
elapsedTotal = toc(tStart);

%% ===================== ИТОГОВЫЙ ОТЧЕТ И ОЦЕНКА СЖАТИЯ =====================
fileInfoH5 = dir(outputH5Path);
h5FinalBytes = fileInfoH5.bytes;

fprintf('\n================== ИТОГОВЫЙ ОТЧЕТ ==================\n');
fprintf('Размер исходного .bin файла:        %.2f ГБ\n', totalBytes / 1e9);
fprintf('Размер сжатого .h5 файла:           %.2f ГБ (Сжатие: %.1f%% от несжатого)\n', ...
    h5FinalBytes / 1e9, (h5FinalBytes / (totalFramesWritten * 1080)) * 100);
fprintf('Успешно записано кадров:            %d\n', totalFramesWritten);
fprintf('Ошибок CRC16 отброшено:             %d (%.3f%%)\n', ...
    totalCrcErrors, (totalCrcErrors / max(1, totalCandidates)) * 100);
fprintf('Время работы:                       %.1f с (%.1f МБ/с)\n', ...
    elapsedTotal, (double(totalBytes) / 1e6) / elapsedTotal);
fprintf('====================================================\n\n');

%% ===================== АНАЛИЗ ТАЙМШТАМПОВ =====================
fprintf('Анализ непрерывности таймштампов первого датчика (S00)...\n');
try
    tsSample = h5read(outputH5Path, '/tstamp', [1, 1], [1, totalFramesWritten]);
    tsSample = double(squeeze(tsSample));
    
    dTs = mod(diff(tsSample), 65536);
    dTsValid = dTs(dTs > 0);
    if ~isempty(dTsValid)
        nominalStep = mode(dTsValid);
        jumps = find(dTs > nominalStep * 1.5);
        
        fprintf('Номинальный шаг таймштампа:         %d тиков\n', nominalStep);
        fprintf('Обнаружено скрытых разрывов потока: %d мест(а)\n', numel(jumps));
        if ~isempty(jumps)
            totalLostTicks = sum(dTs(jumps) - nominalStep);
            fprintf('Оценочное число потерянных кадров:  ~%d кадров\n', round(totalLostTicks / nominalStep));
        end
    end
catch
    fprintf('Не удалось выполнить анализ шага таймштампов.\n');
end