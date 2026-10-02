function R=miib_analyze(inputPath)
% MIIB_ANALYZE  Single-file MIIB analyser.
% R=miib_analyze;  R=miib_analyze('capture.bin');
% R=miib_analyze('decoded_miib_data.mat');  miib_analyze('selftest');
% Includes: BIN/HEX read, CRC, 36-IMU decode, Rz180 for 19..36,
% full-record mean removal, windows 1/10/100/1000 s, drift passport,
% black line = equal mean of 36, red line = dynamic CLUSTER mean (bad sensors dropped).
% MATLAB R2020b+, base MATLAB only. Data layout [sample,sensor,axis].
cfg.fs=400; cfg.sensorODR=3200; cfg.avgFactor=8;
cfg.windows=[1 10 100 1000]; cfg.rawPlotSeconds=Inf;
cfg.maxPlotPoints=6000; cfg.savePNG=true; cfg.saveFIG=true;
cfg.gyroScale=4000/524288; cfg.accelScale=32*9.80665/524288;
cfg.clusterWindow=64;          % raw samples per clustering decision
cfg.clusterGate=4.0;           % clustering starts only beyond this robust z-score
cfg.clusterKmax=5; cfg.clusterIterations=10;
cfg.clusterSilhouetteMin=0.55; % weaker structure = one homogeneous group
cfg.secondaryWeight=0;         % weight of non-main clusters with >=2 sensors
cfg.minKeep=18;                % smaller main group -> equal mean of all, flagged
selfTest(cfg);
if nargin>=1 && (ischar(inputPath) || isstring(inputPath)) && strcmpi(char(inputPath),'selftest')
    R=struct('selftestPassed',true); fprintf('MIIB SELFTEST PASSED.\n'); return
end
if nargin<1 || isempty(inputPath)
    [file,folder]=uigetfile({'*.bin;*.txt;*.mat','MIIB BIN / HEX TXT / decoded MAT'; '*.*','All files'}, ...
        'Выберите захват или decoded_miib_data.mat');
    if isequal(file,0), R=[]; return; end
    inputPath=fullfile(folder,file);
end
inputPath=char(inputPath);
if ~isfile(inputPath), error('MIIB:File','Файл не найден: %s',inputPath); end
[folder,stem,ext]=fileparts(inputPath);
if isempty(folder), folder=pwd; inputPath=fullfile(folder,inputPath); end
fprintf('Файл: %s\n',inputPath);
if strcmpi(ext,'.mat')
    [gyro,accel,counter,stats,cfg]=loadDecoded(inputPath,cfg); out=folder; kind='decoded_mat';
else
    out=fullfile(folder,[stem '_stability']); if ~isfolder(out), mkdir(out); end
    [data,kind]=readCapture(inputPath); fprintf('Тип: %s; байт: %d\n',kind,numel(data));
    [starts,counter,stats]=findFrames(data);
    fprintf('Валидных кадров: %d; CRC errors: %d\n',stats.valid_frames,stats.crc_errors);
    if isempty(starts), error('MIIB:Frames','Валидные 690-байтовые кадры AA 55 не найдены.'); end
    [gyro,accel]=decodeFrames(data,starts,cfg); clear data
    saveDecoded(fullfile(out,'decoded_miib_data.mat'),gyro,accel,counter,starts,stats,cfg,inputPath);
    clear starts
end
n=size(gyro,1); duration=n/cfg.fs; nW=numel(cfg.windows);
fprintf('Среднее каждого датчика и оси за ВСЮ запись...\n');
gref=fullMean(gyro); aref=fullMean(accel); saveReferences(out,gref,aref,n,cfg.fs);

nRaw=min(n,max(1,round(cfg.rawPlotSeconds*cfg.fs)));
fprintf('Динамическая кластеризация, исходные кадры: гироскопы...\n');
DG=dynamicCluster(gyro,gref,3600,cfg,nRaw);
fprintf('Динамическая кластеризация, исходные кадры: акселерометры...\n');
DA=dynamicCluster(accel,aref,1,cfg,nRaw);
saveDynamicRaw(out,DG,DA,cfg);
selectionParts=cell(2+nW,1); keepParts=selectionParts;
selectionParts{1}=selectionTable(DG,'gyro',0); keepParts{1}=keepStats(DG,'gyro',0);
selectionParts{2}=selectionTable(DA,'accel',0); keepParts{2}=keepStats(DA,'accel',0);

parts=cell(nW,1); centeredParts=parts; blocks=parts;
sensorPassParts=parts; arrayPassParts=parts; dynPassParts=parts; skipped=zeros(1,0);
for wi=1:nW
    w=cfg.windows(wi); k=round(w*cfg.fs); nb=floor(n/k);
    if nb<1
        skipped(end+1)=w; %#ok<AGROW>
        sensorPassParts{wi}=driftPassport(zeros(0,36,3),gref,1:36,w,k,n,cfg.fs,nan(36,3));
        arrayPassParts{wi}=driftPassport(zeros(0,1,3),mean(gref,1),0,w,k,n,cfg.fs,nan(1,3));
        dynPassParts{wi}=driftPassport(zeros(0,1,3),zeros(1,3),-1,w,k,n,cfg.fs,nan(1,3));
        fprintf('Окно %g с: нет полных блоков.\n',w); continue
    end
    fprintf('Окно %g с: %d блоков, %d отсчётов в блоке.\n',w,nb,k);
    [gb,gsd]=blockMeansStd(gyro,k); [ab,asd]=blockMeansStd(accel,k);
    writetable([metrics(gb,'gyro','deg/s',w,k,true); metrics(ab,'accel','m/s^2',w,k,true)], ...
        fullfile(out,sprintf('sensor_stability_%gs.csv',w)));
    parts{wi}=[metrics(mean(gb,2),'gyro','deg/s',w,k,false); metrics(mean(ab,2),'accel','m/s^2',w,k,false)];
    gc=centerOnFullMean(gb,gref).*3600;
    writetable(metrics(gc,'gyro','deg/h',w,k,true),fullfile(out,sprintf('gyro_centered_stability_%gs.csv',w)));
    centeredParts{wi}=metrics(mean(gc,2),'gyro','deg/h',w,k,false);
    [noiseSensor,noiseArray]=residualNoise(gyro,gb,k);
    sensorPassParts{wi}=driftPassport(gc,gref,1:36,w,k,n,cfg.fs,noiseSensor.*3600);
    arrayPassParts{wi}=driftPassport(mean(gc,2),mean(gref,1),0,w,k,n,cfg.fs,noiseArray.*3600);
    % New clustering for every block (block mean + within-block std); not an average of the raw red line.
    Gd=dynamicClusterBlocks(gb,gsd,gref,3600,cfg); Ad=dynamicClusterBlocks(ab,asd,aref,1,cfg);
    dynPassParts{wi}=driftPassport(reshape(Gd.centered,nb,1,3),zeros(1,3),-1,w,k,n,cfg.fs,nan(1,3));
    selectionParts{2+wi}=[selectionTable(Gd,'gyro',w); selectionTable(Ad,'accel',w)];
    keepParts{2+wi}=[keepStats(Gd,'gyro',w); keepStats(Ad,'accel',w)];
    saveBlocks(fullfile(out,sprintf('block_means_%gs.mat',w)),gb,ab,gc,gref,Gd,Ad,w,k,cfg);
    blocks{wi}=struct('gyro',gb,'accel',ab,'window',w,'dynGyro',Gd,'dynAccel',Ad);
end
keepW=~cellfun(@isempty,parts);
if any(keepW)
    summary=vertcat(parts{keepW}); centeredSummary=vertcat(centeredParts{keepW});
    writetable(summary,fullfile(out,'array_stability_summary.csv'));
    writetable(centeredSummary,fullfile(out,'gyro_centered_array_summary.csv'));
else, summary=table(); centeredSummary=table(); end
sensorPassport=vertcat(sensorPassParts{:}); arrayPassport=vertcat(arrayPassParts{:});
dynamicPassport=vertcat(dynPassParts{:});
selectionLog=vertcat(selectionParts{~cellfun(@isempty,selectionParts)});
keepTable=vertcat(keepParts{~cellfun(@isempty,keepParts)});
writetable(sensorPassport,fullfile(out,'gyro_drift_passport_sensors.csv'));
writetable(arrayPassport,fullfile(out,'gyro_drift_passport_array.csv'));
writetable(dynamicPassport,fullfile(out,'gyro_drift_passport_dynamic.csv'));
writetable(selectionLog,fullfile(out,'dynamic_cluster_selection.csv'));
writetable(keepTable,fullfile(out,'dynamic_cluster_keep_stats.csv'));
save(fullfile(out,'gyro_drift_passport.mat'),'sensorPassport','arrayPassport','dynamicPassport', ...
    'selectionLog','keepTable','gref','stats','cfg','-v7.3');
writeReport(out,inputPath,kind,stats,duration,cfg,arrayPassport,dynamicPassport);
writeHTML(out,inputPath,stats,duration,cfg,arrayPassport,dynamicPassport,sensorPassport,keepTable);
cols={'axis','window_sec','n_blocks','sigma_block_dph','peak_to_peak_dph','max_abs_centered_dph','end_start_dph','status'};
fprintf('\nПаспорт: РАВНОЕ среднее 36, град/ч\n'); disp(arrayPassport(:,cols));
fprintf('Паспорт: ДИНАМИЧЕСКОЕ кластерное среднее, град/ч\n'); disp(dynamicPassport(:,cols));

% Numerical outputs are complete. Plot jobs below store only decimated points.
t=(0:nRaw-1)'/cfg.fs; jobs=cell(2+2*sum(keepW),1);
jobs{1}=makeJob(gyro,nRaw,t,gref,3600,true,DG,'Гироскопы: исходный сигнал','deg/h','raw_gyro',cfg);
jobs{2}=makeJob(accel,nRaw,t,aref,1,false,DA,'Акселерометры: исходный сигнал','m/s^2','raw_accel',cfg);
j=2;
for wi=1:nW
    if isempty(blocks{wi}), continue; end
    b=blocks{wi}; w=b.window; nb=size(b.gyro,1); bt=((0:nb-1)'+0.5)*w;
    j=j+1; jobs{j}=makeJob(b.gyro,nb,bt,gref,3600,true,b.dynGyro, ...
        sprintf('Гироскопы: блоковые средние, %g с',w),'deg/h',sprintf('gyro_block_means_%gs',w),cfg);
    j=j+1; jobs{j}=makeJob(b.accel,nb,bt,aref,1,false,b.dynAccel, ...
        sprintf('Акселерометры: блоковые средние, %g с',w),'m/s^2',sprintf('accel_block_means_%gs',w),cfg);
end
figures=gobjects(0,1); errors=cell(0,1);
for j=1:numel(jobs)
    try
        f=signalFigure(jobs{j},out); figures(end+1,1)=f; autoSave(f,cfg); %#ok<AGROW>
    catch err
        errors{end+1,1}=[jobs{j}.name ': ' err.message]; %#ok<AGROW>
        warning('MIIB:Graphics','%s. Численные результаты уже сохранены.',errors{end});
    end
end
try, figures(end+1,1)=passportFigure(sensorPassport,arrayPassport,dynamicPassport);
catch err, errors{end+1,1}=err.message; warning('MIIB:Graphics','Таблица паспорта: %s',err.message); end
try, f=passportPlots(arrayPassport,dynamicPassport,out); figures(end+1,1)=f; autoSave(f,cfg);
catch err, errors{end+1,1}=err.message; warning('MIIB:Graphics','График паспорта: %s',err.message); end
R=struct('outputDir',out,'stats',stats,'durationSec',duration,'summary',summary, ...
    'centeredGyroSummary',centeredSummary,'sensorPassport',sensorPassport, ...
    'arrayPassport',arrayPassport,'dynamicPassport',dynamicPassport,'selectionLog',selectionLog, ...
    'gyroMeanAllDps',gref,'accelMeanAllMps2',aref,'figures',figures, ...
    'graphicsErrors',{errors},'skippedWindows',skipped,'config',cfg);
fprintf('\nГотово: %d кадров, %.3f с, %d окон.\nРезультаты: %s\n',n,duration,numel(figures),out);
if stats.counter_gaps || stats.counter_duplicates || stats.counter_restarts
    warning('MIIB:Counter',['Пропуски=%d, дубликаты=%d, рестарты=%d. ' ...
        'Время: индекс принятого кадра / Fs; потерянные интервалы не восстановлены.'], ...
        stats.counter_gaps,stats.counter_duplicates,stats.counter_restarts);
end
end

function J=makeJob(v,nRows,t,ref,scale,center,D,heading,units,name,cfg)
ix=unique(round(linspace(1,nRows,min(nRows,cfg.maxPlotPoints))));
J=struct('values',double(v(ix,:,:)).*scale,'time',reshape(t(ix),[],1),'ref',double(ref).*scale, ...
    'center',center,'dynC',D.centered(ix,:),'dynA',D.absolute(ix,:),'nRows',nRows, ...
    'title',heading,'units',units,'name',name);
end

function [data,kind]=readCapture(path)
fid=fopen(path,'rb'); if fid<0, error('MIIB:Read','Не удалось открыть файл.'); end
cleanup=onCleanup(@() fclose(fid)); %#ok<NASGU>
raw=fread(fid,Inf,'*uint8').'; [~,~,ext]=fileparts(path);
hc=uint8('0123456789abcdefABCDEF'); ws=uint8([9 10 11 12 13 32]);
sample=raw(1:min(numel(raw),65536));
hex=~isempty(sample) && all(ismember(sample,[hc ws])) && any(ismember(sample,hc));
if strcmpi(ext,'.txt') || hex
    s=raw(~ismember(raw,ws));
    if ~all(ismember(s,hc)), error('MIIB:Hex','Недопустимые HEX-символы.'); end
    if mod(numel(s),2), error('MIIB:Hex','Нечётное число HEX-символов.'); end
    lut=zeros(1,256,'uint8'); lut(double(uint8('0123456789'))+1)=uint8(0:9);
    lut(double(uint8('abcdef'))+1)=uint8(10:15); lut(double(uint8('ABCDEF'))+1)=uint8(10:15);
    d=lut(double(s)+1); data=bitor(bitshift(d(1:2:end),4),d(2:2:end)); kind='hex_text';
else, data=raw; kind='binary'; end
end

function S=emptyStats(n)
S=struct('input_bytes',n,'valid_frames',0,'crc_errors',0,'discarded_bytes',0, ...
    'counter_gaps',0,'counter_duplicates',0,'counter_restarts',0);
end

function [starts,counters,S]=findFrames(data)
data=reshape(data,1,[]); L=numel(data); S=emptyStats(L);
if L<690, starts=[]; counters=zeros(0,1,'uint16'); S.discarded_bytes=L; return; end
scanSize=16*1024*1024; chunks=cell(ceil((L-1)/scanSize),1); np=0;
for first=1:scanSize:L-1
    last=min(first+scanSize-1,L-1);
    h=find(data(first:last)==170 & data(first+1:last+1)==85);
    np=np+1; chunks{np}=reshape(h+first-1,1,[]);
end
headers=[chunks{:}]; headers=headers(headers+689<=L); valid=false(1,numel(headers));
for first=1:4096:numel(headers)
    last=min(first+4095,numel(headers)); h=headers(first:last);
    b=reshape(data(bsxfun(@plus,(2:687).',h)),686,numel(h));
    received=bitor(uint16(data(h+688)),bitshift(uint16(data(h+689)),8));
    valid(first:last)=crcBytes(b)==reshape(received,1,[]);
end
starts=zeros(1,numel(headers)); count=0; position=1;
for j=1:numel(headers)
    h=headers(j); if h<position, continue; end
    if valid(j), count=count+1; starts(count)=h; position=h+690;
    else, S.crc_errors=S.crc_errors+1; position=h+1; end
end
starts=starts(1:count);
counters=reshape(bitor(uint16(data(starts+2)),bitshift(uint16(data(starts+3)),8)),[],1);
S.valid_frames=count; S.discarded_bytes=L-count*690;
if count>1
    d=mod(diff(double(counters)),65536); forward=d>0 & d<32768;
    S.counter_duplicates=sum(d==0); S.counter_gaps=sum(d(forward)-1); S.counter_restarts=sum(d>=32768);
end
end

function crc=crcBytes(bytes)
persistent lut
if isempty(lut)
    lut=zeros(1,256,'uint16');
    for j=0:255
        c=bitshift(uint16(j),8);
        for k=1:8
            top=bitand(c,uint16(32768))~=0; c=bitshift(c,1);
            if top, c=bitxor(c,uint16(4129)); end
        end
        lut(j+1)=c;
    end
end
crc=repmat(uint16(65535),1,size(bytes,2));
for j=1:size(bytes,1)
    ix=double(bitxor(bitshift(crc,-8),reshape(uint16(bytes(j,:)),1,[])))+1;
    crc=bitxor(bitshift(crc,8),reshape(lut(ix),1,[]));
end
end

function [g,a]=decodeFrames(data,starts,cfg)
n=numel(starts); g=zeros(n,36,3); a=zeros(n,36,3);
for first=1:8192:n
    last=min(first+8191,n); h=reshape(starts(first:last),1,[]);
    b=reshape(data(bsxfun(@plus,(4:687).',h)),684,numel(h));
    p=double(permute(reshape(b,19,36,numel(h)),[3 2 1]));
    for ax=1:3
        av=p(:,:,2*ax-1)*4096+p(:,:,2*ax)*16+floor(p(:,:,16+ax)/16);
        gv=p(:,:,2*ax+5)*4096+p(:,:,2*ax+6)*16+mod(p(:,:,16+ax),16);
        av(av>=524288)=av(av>=524288)-1048576; gv(gv>=524288)=gv(gv>=524288)-1048576;
        a(first:last,:,ax)=av.*cfg.accelScale; g(first:last,:,ax)=gv.*cfg.gyroScale;
    end
end
g(:,19:36,1:2)=-g(:,19:36,1:2); a(:,19:36,1:2)=-a(:,19:36,1:2);
end

function ref=fullMean(v)
n=size(v,1); if n<1, error('MIIB:Mean','Пустая запись.'); end
ref=zeros(36,3);
for first=1:100000:n
    last=min(first+99999,n); ref=ref+reshape(sum(double(v(first:last,:,:)),1),36,3);
end
ref=ref/n;
end

function c=centerOnFullMean(v,ref)
c=bsxfun(@minus,double(v),reshape(double(ref),1,size(v,2),3));
end

function b=blockMeans(v,k)
nb=floor(size(v,1)/k); ns=size(v,2); b=zeros(nb,ns,3); step=max(1,floor(200000/k));
for first=1:step:nb
    last=min(first+step-1,nb); chunk=double(v((first-1)*k+1:last*k,:,:));
    b(first:last,:,:)=reshape(mean(reshape(chunk,k,last-first+1,ns,3),1),last-first+1,ns,3);
end
end

function [b,sd]=blockMeansStd(v,k)
nb=floor(size(v,1)/k); ns=size(v,2); b=zeros(nb,ns,3); sd=zeros(nb,ns,3); step=max(1,floor(200000/k));
for first=1:step:nb
    last=min(first+step-1,nb); nbc=last-first+1;
    chunk=reshape(double(v((first-1)*k+1:last*k,:,:)),k,nbc,ns,3);
    b(first:last,:,:)=reshape(mean(chunk,1),nbc,ns,3);
    sd(first:last,:,:)=reshape(std(chunk,0,1),nbc,ns,3);
end
end

function [noiseSensor,noiseArray]=residualNoise(v,b,k)
% Pooled std of raw samples after subtracting each complete block's own mean.
nb=size(b,1); ns=size(v,2); nc=nb*k; df=nc-nb;
if df<=0, noiseSensor=nan(ns,3); noiseArray=nan(1,3); return; end
ssSensor=zeros(ns,3); ssArray=zeros(1,3); arrayBlocks=mean(b,2);
for first=1:50000:nc
    last=min(first+49999,nc); idx=floor(((first:last)-1)/k)+1;
    sumArray=zeros(last-first+1,1,3);
    for s0=1:6:ns
        sel=s0:min(s0+5,ns); chunk=double(v(first:last,sel,:));
        sumArray=sumArray+sum(chunk,2);
        resid=chunk-double(b(idx,sel,:));
        ssSensor(sel,:)=ssSensor(sel,:)+reshape(sum(resid.^2,1),numel(sel),3);
    end
    resid=sumArray./ns-double(arrayBlocks(idx,1,:));
    ssArray=ssArray+reshape(sum(resid.^2,1),1,3);
end
noiseSensor=sqrt(ssSensor/df); noiseArray=sqrt(ssArray/df);
end

function col=pack(v)
col=reshape(v.',[],1);
end

function T=metrics(v,q,units,w,k,withSensor)
nb=size(v,1); ns=size(v,2); nr=ns*3;
if nb<1, error('MIIB:Metric','Нет блоков.'); end
mu=reshape(mean(v,1),ns,3);
if nb>=2, sd=reshape(std(v,0,1),ns,3); else, sd=nan(ns,3); end
lo=reshape(min(v,[],1),ns,3); hi=reshape(max(v,[],1),ns,3);
drift=reshape(v(end,:,:)-v(1,:,:),ns,3); ptp=hi-lo;
if nb<2, drift=nan(ns,3); ptp=nan(ns,3); end
T=table(repmat({q},nr,1),repmat({'X';'Y';'Z'},ns,1),repmat(w,nr,1), ...
    repmat(nb,nr,1),repmat(k,nr,1),pack(mu),pack(sd),pack(lo),pack(hi),pack(ptp),pack(drift), ...
    repmat({units},nr,1),'VariableNames',{'quantity','axis','window_sec','n_blocks', ...
    'samples_per_block','mean','std_block_means','min_block_mean','max_block_mean', ...
    'peak_to_peak','drift_end_start','units'});
if withSensor, sensor_id=reshape(repmat(1:ns,3,1),[],1); T=[table(sensor_id),T]; end
end

function T=driftPassport(b,ref,sensorIDs,w,k,n,fs,noiseStd)
% sensor_id: 0 = equal 36 mean, -1 = dynamic cluster mean, 1..36 = sensors.
nb=size(b,1); ns=numel(sensorIDs); nr=ns*3;
mu=nan(ns,3); sd=mu; lo=mu; hi=mu; ptp=mu; endStart=mu; maxFirst=mu;
rmsFull=mu; maxAbs=mu; slope=mu; slopeChange=mu; fitResidual=mu;
finiteAll=nb>=1 && all(isfinite(b(:)));
if finiteAll
    mu=reshape(mean(b,1),ns,3); lo=reshape(min(b,[],1),ns,3); hi=reshape(max(b,[],1),ns,3);
    rmsFull=reshape(sqrt(mean(b.^2,1)),ns,3); maxAbs=reshape(max(abs(b),[],1),ns,3);
end
if finiteAll && nb>=2
    sd=reshape(std(b,0,1),ns,3); ptp=hi-lo; endStart=reshape(b(end,:,:)-b(1,:,:),ns,3);
    maxFirst=reshape(max(abs(bsxfun(@minus,b,b(1,:,:))),[],1),ns,3);
    th=(((0:nb-1)'+0.5)*k/fs)/3600; tc=th-mean(th);
    V=reshape(b,nb,ns*3); slopeRow=(tc.'*V)/sum(tc.^2);
    slope=reshape(slopeRow,ns,3); slopeChange=slope*(th(end)-th(1));
    if nb>=3
        resid=bsxfun(@minus,V,mean(V,1)+tc*slopeRow);
        fitResidual=reshape(sqrt(sum(resid.^2,1)/(nb-2)),ns,3);
    end
end
if nb==0, status='no_complete_blocks';
elseif ~finiteAll, status='insufficient_dynamic_points';
elseif nb==1, status='insufficient_one_block';
elseif nb<10, status='2_to_9_blocks';
else, status='10plus_blocks'; end
sensor_id=reshape(repmat(sensorIDs(:).',3,1),[],1); axis=repmat({'X';'Y';'Z'},ns,1);
T=table(sensor_id,axis,repmat(w,nr,1),repmat(nb,nr,1),repmat(k,nr,1), ...
    repmat(nb*k/fs,nr,1),repmat((n-nb*k)/fs,nr,1),pack(reshape(ref,ns,3).*3600), ...
    pack(mu),pack(sd),pack(3*sd),pack(rmsFull),pack(maxAbs),pack(lo),pack(hi),pack(ptp), ...
    pack(endStart),pack(maxFirst),pack(slope),pack(slopeChange),pack(fitResidual),pack(noiseStd), ...
    repmat({status},nr,1),'VariableNames',{'sensor_id','axis','window_sec','n_blocks','samples_per_block', ...
    'complete_duration_sec','dropped_tail_sec','full_record_mean_dph','mean_centered_dph', ...
    'sigma_block_dph','sigma3_block_dph','rms_centered_dph','max_abs_centered_dph', ...
    'min_block_dph','max_block_dph','peak_to_peak_dph','end_start_dph','max_from_first_dph', ...
    'linear_slope_dph_per_hour','linear_change_dph','linear_residual_std_dph', ...
    'residual_noise_std_dph','status'});
end

function D=newDynamic(n,scale,ref,nWin)
D=struct('centered',nan(n,3),'absolute',nan(n,3),'nKept',zeros(n,3,'uint8'), ...
    'flags',zeros(n,3,'uint8'),'nClusters',zeros(n,3,'uint8'),'inclusionCount',zeros(36,3), ...
    'scale',scale,'full_record_reference',ref,'windowStart',zeros(nWin,1),'windowEnd',zeros(nWin,1), ...
    'weights',zeros(nWin,36,3,'single'));
end

function D=dynamicCluster(v,ref,scale,cfg,nRows)
% Raw data: every cfg.clusterWindow samples, features = [window mean, window std].
% The same window supplies the weights and the averaged samples (offline analysis).
W=cfg.clusterWindow; nWin=max(1,floor(nRows/W));
i1=(0:nWin-1)*W+1; i2=i1+W-1; i2(end)=nRows;
D=newDynamic(nRows,scale,ref,nWin); D.windowStart=i1(:); D.windowEnd=i2(:);
for a=1:3
    for wi=1:nWin
        r=i1(wi):i2(wi); x=bsxfun(@minus,double(v(r,:,a)),ref(:,a).');
        [wt,flag,K]=clusterWeights(mean(x,1).',std(x,0,1).',cfg);
        D=storeWindow(D,r,a,x,wt,flag,K,ref(:,a),scale); D.weights(wi,:,a)=single(wt.');
    end
end
end

function D=dynamicClusterBlocks(b,sd,ref,scale,cfg)
% Block windows: features = [block mean, std of raw samples inside that block].
nb=size(b,1); D=newDynamic(nb,scale,ref,nb); D.windowStart=(1:nb).'; D.windowEnd=(1:nb).';
for a=1:3
    for k=1:nb
        x=reshape(double(b(k,:,a)),1,36)-ref(:,a).';
        [wt,flag,K]=clusterWeights(x.',reshape(double(sd(k,:,a)),[],1),cfg);
        D=storeWindow(D,k,a,x,wt,flag,K,ref(:,a),scale); D.weights(k,:,a)=single(wt.');
    end
end
end

function D=storeWindow(D,r,a,x,wt,flag,K,refa,scale)
L=numel(r);
if any(isnan(wt))
    D.centered(r,a)=NaN; D.absolute(r,a)=NaN; D.nKept(r,a)=0;
else
    x0=x; x0(~isfinite(x0))=0; y=x0*wt;
    D.centered(r,a)=y.*scale; D.absolute(r,a)=(y+refa(:).'*wt).*scale;
    D.nKept(r,a)=uint8(nnz(wt>0));
    D.inclusionCount(:,a)=D.inclusionCount(:,a)+double(wt>0)*L;
end
D.flags(r,a)=flag; D.nClusters(r,a)=uint8(K);
end

function [w,flag,K]=clusterWeights(m,s,cfg)
% m: centered mean of each sensor; s: its std. Returns normalised sensor weights.
m=m(:); s=s(:); w=zeros(numel(m),1); K=0; flag=uint8(0);
idx=find(isfinite(m) & isfinite(s));
if numel(idx)<cfg.minKeep, w(:)=NaN; flag=uint8(2); return; end
w(idx)=1/numel(idx); K=1;
f1=robustZ(m(idx)); f2=robustZ(log(max(s(idx),realmin)));
if ~any(abs(f1)>cfg.clusterGate | abs(f2)>cfg.clusterGate), return; end
P=[f1 f2]; n=size(P,1); best=-Inf; bestLab=ones(n,1);
for k=2:min(cfg.clusterKmax,n-1)
    lab=kmeansFarthest(P,k,cfg.clusterIterations); sc=meanSilhouette(P,lab);
    if sc>best, best=sc; bestLab=lab; end
end
if best<cfg.clusterSilhouetteMin, return; end
cnt=accumarray(bestLab,1); main=find(cnt==max(cnt),1); K=numel(cnt);
ws=zeros(n,1);
for c=1:K
    if c==main, ws(bestLab==c)=1;
    elseif cnt(c)>=2, ws(bestLab==c)=cfg.secondaryWeight; end
end
if cnt(main)<cfg.minKeep, flag=uint8(1); return; end
w(:)=0; w(idx)=ws/sum(ws);
end

function z=robustZ(x)
med=median(x); sc=max(1.482602218505602*median(abs(x-med)),64*eps(max(1,max(abs(x)))));
z=(x-med)/sc;
end

function lab=kmeansFarthest(P,k,iters)
% Deterministic farthest-point initialisation + Lloyd iterations.
n=size(P,1); [~,first]=min(sum(P.^2,2)); C=P(first,:);
for c=2:k
    d=min(sqDist(P,C),[],2); [~,nxt]=max(d); C(end+1,:)=P(nxt,:); %#ok<AGROW>
end
lab=zeros(n,1);
for it=1:iters
    [~,newLab]=min(sqDist(P,C),[],2);
    if isequal(newLab,lab), break; end
    lab=newLab;
    for c=1:k
        mk=lab==c; if any(mk), C(c,:)=mean(P(mk,:),1); end
    end
end
[~,~,lab]=unique(lab); lab=lab(:);
end

function D=sqDist(P,C)
D=bsxfun(@minus,P(:,1),C(:,1).').^2+bsxfun(@minus,P(:,2),C(:,2).').^2;
end

function sc=meanSilhouette(P,lab)
n=size(P,1); D=sqrt(sqDist(P,P)); k=max(lab); s=zeros(n,1);
for i=1:n
    same=lab==lab(i); same(i)=false;
    if ~any(same), continue; end
    a=mean(D(i,same)); b=Inf;
    for c=1:k
        if c==lab(i), continue; end
        mk=lab==c; if any(mk), b=min(b,mean(D(i,mk))); end
    end
    den=max(a,b); if den>0 && isfinite(b), s(i)=(b-a)/den; end
end
sc=mean(s);
end

function T=selectionTable(D,q,w)
sensor_id=reshape(repmat(1:36,3,1),[],1); axis=repmat({'X';'Y';'Z'},36,1);
np=size(D.centered,1); acc=reshape(D.inclusionCount.',[],1);
T=table(repmat({q},108,1),sensor_id,axis,repmat(w,108,1),repmat(np,108,1),acc,acc/max(np,1), ...
    'VariableNames',{'quantity','sensor_id','axis','window_sec','n_points','accepted_points','accepted_fraction'});
end

function T=keepStats(D,q,w)
np=size(D.centered,1);
T=table(repmat({q},3,1),{'X';'Y';'Z'},repmat(w,3,1),repmat(np,3,1), ...
    mean(double(D.nKept),1).',min(double(D.nKept),[],1).',mean(double(D.nClusters),1).', ...
    sum(D.flags==1,1).',sum(D.flags==2,1).', ...
    'VariableNames',{'quantity','axis','window_sec','n_points','mean_sensors_kept','min_sensors_kept', ...
    'mean_clusters','fallback_equal_mean_points','insufficient_points'});
end

function saveDecoded(path,g,a,counter,starts,stats,cfg,source_file)
gyro_dps=single(g); accel_mps2=single(a); %#ok<NASGU>
output_fs_hz=cfg.fs; sensor_odr_hz=cfg.sensorODR; averaging_factor=cfg.avgFactor; %#ok<NASGU>
frame_byte_offset=uint64(starts(:)-1); %#ok<NASGU>
time_note='Accepted-frame index / Fs; Rz180 already applied; NOT mean-centered'; %#ok<NASGU>
save(path,'gyro_dps','accel_mps2','counter','stats','output_fs_hz','sensor_odr_hz', ...
    'averaging_factor','frame_byte_offset','source_file','time_note','-v7.3');
end

function [g,a,counter,stats,cfg]=loadDecoded(path,cfg)
S=load(path); required={'gyro_dps','accel_mps2','output_fs_hz'};
for j=1:numel(required)
    if ~isfield(S,required{j}), error('MIIB:Mat','Нет поля %s.',required{j}); end
end
if ~isnumeric(S.gyro_dps) || ~isnumeric(S.accel_mps2) || isempty(S.gyro_dps) ...
        || ~isreal(S.gyro_dps) || ~isreal(S.accel_mps2) ...
        || ~isequal(size(S.gyro_dps),size(S.accel_mps2)) ...
        || ndims(S.gyro_dps)~=3 || size(S.gyro_dps,2)~=36 || size(S.gyro_dps,3)~=3
    error('MIIB:Mat','Ожидаются gyro_dps и accel_mps2 размера N x 36 x 3.');
end
if ~isscalar(S.output_fs_hz) || ~isfinite(S.output_fs_hz) || S.output_fs_hz<=0
    error('MIIB:Mat','Некорректная output_fs_hz.');
end
g=S.gyro_dps; a=S.accel_mps2; cfg.fs=double(S.output_fs_hz);
if isfield(S,'sensor_odr_hz'), cfg.sensorODR=double(S.sensor_odr_hz); end
if isfield(S,'averaging_factor'), cfg.avgFactor=double(S.averaging_factor); end
stats=emptyStats(0); fields=fieldnames(stats);
if isfield(S,'stats') && isstruct(S.stats) && isscalar(S.stats)
    for j=1:numel(fields)
        if isfield(S.stats,fields{j}), stats.(fields{j})=S.stats.(fields{j}); end
    end
end
stats.valid_frames=size(g,1);
if isfield(S,'counter') && numel(S.counter)==size(g,1), counter=reshape(uint16(S.counter),[],1);
else, counter=zeros(0,1,'uint16'); end
% Rz180 already applied in decoded MAT. Never rotate the data twice.
end

function saveReferences(out,gref,aref,n,fs)
gyro_full_record_mean_dps=gref; accel_full_record_mean_mps2=aref; %#ok<NASGU>
n_samples=n; output_fs_hz=fs; %#ok<NASGU>
save(fullfile(out,'full_record_centering_reference.mat'), ...
    'gyro_full_record_mean_dps','accel_full_record_mean_mps2','n_samples','output_fs_hz');
sensor_id=reshape(repmat(1:36,3,1),[],1); axis=repmat({'X';'Y';'Z'},36,1);
mean_dps=pack(gref); mean_dph=mean_dps.*3600;
writetable(table(sensor_id,axis,mean_dps,mean_dph),fullfile(out,'gyro_full_record_means.csv'));
end

function saveDynamicRaw(out,DG,DA,cfg)
gyro=DG; accel=DA; settings=cfg; %#ok<NASGU>
note='weights: window x sensor x axis; flags 1 = main cluster < minKeep (equal mean), 2 = insufficient finite sensors'; %#ok<NASGU>
save(fullfile(out,'dynamic_cluster_raw.mat'),'gyro','accel','settings','note','-v7.3');
end

function saveBlocks(path,g,a,c,ref,Gd,Ad,w,k,cfg)
gyro_dps=single(g); accel_mps2=single(a); gyro_centered_dph=single(c); %#ok<NASGU>
gyro_full_record_mean_dps=ref; dynamic_gyro=Gd; dynamic_accel=Ad; %#ok<NASGU>
window_sec=w; samples_per_block=k; settings=cfg; %#ok<NASGU>
save(path,'gyro_dps','accel_mps2','gyro_centered_dph','gyro_full_record_mean_dps', ...
    'dynamic_gyro','dynamic_accel','window_sec','samples_per_block','settings','-v7.3');
end

function notes=passportNotes()
notes={ ...
    'Паспорт наблюдаемых изменений угловой скорости внутри одной записи; это не сертификат абсолютной точности, не ГОСТ-паспорт и не Allan bias instability.', ...
    'Изменение угловой скорости можно трактовать как дрейф только при неподвижном приборе и неизменных внешних условиях. Температура и движение здесь отдельно не оценены.', ...
    'Из каждой оси каждого датчика вычтено одно среднее за ВСЮ запись, включая неполный хвост. Та же константа применена ко всем окнам и без перецентрирования окон.', ...
    'sensor_id=0: равное арифметическое среднее 36 датчиков. sensor_id=-1: динамическое кластерное среднее. Единицы: град/ч.', ...
    'Динамическая кластеризация: каждые 64 кадра для каждой оси строятся признаки каждого датчика: среднее (смещение) и СКО внутри окна; признаки нормируются робастно (медиана/MAD).', ...
    'Кластеризация запускается, только если какой-то датчик по смещению или шуму дальше 4 робастных сигм. Иначе все датчики считаются одной группой, чтобы не дробить однородный массив.', ...
    'При запуске выполняется k-means (K=2..5, детерминированная инициализация); K выбирается по среднему силуэту, структура принимается при силуэте >= 0.55. Метод локтя на однородном ядре из 36 датчиков нестабилен.', ...
    'Самый большой кластер получает вес 1, одиночные датчики исключаются, вторичные кластеры с >=2 датчиками получают secondaryWeight (по умолчанию 0). Веса нормируются на сумму 1.', ...
    'Если главный кластер меньше 18 датчиков, используется равное среднее всех конечных датчиков (fallback_equal_mean_points). Меньше 18 конечных датчиков даёт NaN.', ...
    'Для окон 1/10/100/1000 с кластеризация выполняется заново для каждого блока: признаки — среднее блока и СКО исходных отсчётов внутри блока. Это не блоковое среднее красной линии с исходного графика.', ...
    'Окно кластеризации и усредняемые отсчёты совпадают (офлайн-анализ): веса не причинны. Динамический выход не центрируется повторно, смещение из-за смены состава видно в паспорте.', ...
    'sigma_block: выборочное СКО блоковых средних, делитель M-1. 3 СКО — описательная величина, не гарантированная граница и не доверительный интервал.', ...
    'peak_to_peak=max-min; end_start=последний минус первый блок; linear_change=изменение МНК-линии между центрами первого и последнего блока. Тренд не вычитается.', ...
    'residual_noise_std: СКО исходных отсчётов после вычитания собственного блокового среднего, делитель N-M. Включает изменения внутри блока, не только белый шум.', ...
    'Один полный блок не означает нулевой дрейф: СКО, размах, конец-начало и тренд равны NaN. Для динамической линии NaN в любой точке даёт NaN временных метрик.', ...
    'Пропуски, дубликаты и рестарты счётчика не восстанавливаются. Время строится по индексу принятого кадра/Fs, блоки могут пересекать разрывы.', ...
    'Неполный хвост не входит в блоковые метрики, но входит в среднее за запись. Уменьшение разброса после отбора не доказывает улучшение абсолютной точности.', ...
    'Исходный decoded MAT хранится в single и не центрирован; центрированные паспортные расчёты выполняются в double.'};
end

function writeReport(out,path,kind,S,duration,cfg,A,D)
fid=fopen(fullfile(out,'report.txt'),'w','n','UTF-8');
if fid<0, error('MIIB:Report','Не удалось создать report.txt.'); end
cleanup=onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'MIIB gyro drift passport\nInput: %s\nType: %s\n',path,kind);
fields=fieldnames(S);
for j=1:numel(fields), fprintf(fid,'%s: %g\n',fields{j},S.(fields{j})); end
fprintf(fid,'Duration: %.6f s\nFs: %g Hz\nWindows [s]:',duration,cfg.fs); fprintf(fid,' %g',cfg.windows);
fprintf(fid,'\nDynamic clustering: window %d, gate %.4g, Kmax %d, silhouette min %.3g, secondary weight %.3g, minKeep %d\n\n', ...
    cfg.clusterWindow,cfg.clusterGate,cfg.clusterKmax,cfg.clusterSilhouetteMin,cfg.secondaryWeight,cfg.minKeep);
notes=passportNotes(); for j=1:numel(notes), fprintf(fid,'%s\n',notes{j}); end
for part=1:2
    if part==1, T=A; fprintf(fid,'\nEQUAL 36-SENSOR ARRAY\n'); else, T=D; fprintf(fid,'\nDYNAMIC CLUSTER MEAN\n'); end
    fprintf(fid,'axis,window_s,n_blocks,sigma_dph,ptp_dph,max_abs_dph,end_start_dph,trend_change_dph,status\n');
    for j=1:height(T)
        fprintf(fid,'%s,%g,%d,%.9g,%.9g,%.9g,%.9g,%.9g,%s\n',T.axis{j},T.window_sec(j),T.n_blocks(j), ...
            T.sigma_block_dph(j),T.peak_to_peak_dph(j),T.max_abs_centered_dph(j),T.end_start_dph(j), ...
            T.linear_change_dph(j),T.status{j});
    end
end
end

function writeHTML(out,path,S,duration,cfg,A,D,P,K)
fid=fopen(fullfile(out,'gyro_drift_passport.html'),'w','n','UTF-8');
if fid<0, error('MIIB:HTML','Не удалось создать HTML-паспорт.'); end
cleanup=onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,['<!doctype html><html lang="ru"><head><meta charset="utf-8"><title>Паспорт дрейфа MIIB</title>' ...
    '<style>body{font-family:Arial,sans-serif;margin:28px;color:#182536}h1,h2{color:#164d85}' ...
    'table{border-collapse:collapse;font-size:12px;white-space:nowrap}th,td{border:1px solid #d4dce6;padding:6px}' ...
    'th{background:#e9f0f8}tr:nth-child(even){background:#f5f8fc}.scroll{overflow-x:auto;margin:14px 0}' ...
    'li{margin-bottom:7px}summary{cursor:pointer;font-weight:bold;padding:10px}</style></head><body>']);
fprintf(fid,'<h1>Паспорт дрейфа гироскопов в запуске</h1><p>Файл: %s</p>',htmlEscape(path));
fprintf(fid,'<p>Кадров: %d; длительность: %.6f с; Fs: %g Гц. Отклонения: град/ч.</p>',S.valid_frames,duration,cfg.fs);
fprintf(fid,'<p>CRC: %d; пропуски: %d; дубликаты: %d; рестарты: %d.</p>', ...
    S.crc_errors,S.counter_gaps,S.counter_duplicates,S.counter_restarts);
fprintf(fid,'<h2>Равное среднее 36 датчиков</h2>'); htmlTable(fid,A);
fprintf(fid,'<h2>Динамическое кластерное среднее</h2>'); htmlTable(fid,D);
fprintf(fid,'<h2>Сколько датчиков принято и сколько кластеров</h2>'); htmlTable(fid,K);
fprintf(fid,'<h2>Методика и ограничения</h2><ul>'); notes=passportNotes();
for j=1:numel(notes), fprintf(fid,'<li>%s</li>',htmlEscape(notes{j})); end
fprintf(fid,'</ul><h2>Отдельные датчики</h2>');
for s=1:36
    fprintf(fid,'<details><summary>Датчик %02d: X / Y / Z, все окна</summary>',s);
    htmlTable(fid,P(P.sensor_id==s,:)); fprintf(fid,'</details>');
end
fprintf(fid,'</body></html>');
end

function htmlTable(fid,T)
fprintf(fid,'<div class="scroll"><table><thead><tr>'); names=T.Properties.VariableNames;
for j=1:numel(names), fprintf(fid,'<th>%s</th>',htmlEscape(names{j})); end
fprintf(fid,'</tr></thead><tbody>'); data=table2cell(T);
for i=1:size(data,1)
    fprintf(fid,'<tr>');
    for j=1:size(data,2)
        value=data{i,j};
        if isnumeric(value)
            if isnan(value), tt='нет оценки'; else, tt=sprintf('%.8g',value); end
        else, tt=char(value); end
        fprintf(fid,'<td>%s</td>',htmlEscape(tt));
    end
    fprintf(fid,'</tr>');
end
fprintf(fid,'</tbody></table></div>');
end

function s=htmlEscape(s)
s=char(s); s=strrep(s,'&','&amp;'); s=strrep(s,'<','&lt;'); s=strrep(s,'>','&gt;'); s=strrep(s,'"','&quot;');
end

function f=signalFigure(J,out)
vd=J.values; td=J.time; ref=J.ref; names='XYZ';
f=figure('Name',J.title,'NumberTitle','off','Color','w','Visible','on', ...
    'Units','normalized','Position',[0.05 0.06 0.9 0.85],'MenuBar','figure','ToolBar','figure');
axs=gobjects(3,1); hnd=gobjects(38,3);
if numel(td)==1, marker='o'; else, marker='none'; end
for a=1:3
    axs(a)=subplot(3,1,a,'Parent',f);
    set(axs(a),'Units','normalized','Position',[0.085 0.665-(a-1)*0.275 0.875 0.22]); hold(axs(a),'on');
    for s=1:36
        if s<=18, c=[0.62 0.76 0.94]; else, c=[0.95 0.73 0.57]; end
        hnd(s,a)=plot(axs(a),td,vd(:,s,a),'Color',c,'LineWidth',0.65,'Marker',marker, ...
            'DisplayName',sprintf('Датчик %02d | %c',s,names(a)));
    end
    hnd(37,a)=plot(axs(a),td,mean(vd(:,:,a),2),'k-','LineWidth',1.8,'Marker',marker, ...
        'DisplayName',sprintf('Равное среднее 36 | %c',names(a)));
    hnd(38,a)=plot(axs(a),td,J.dynA(:,a),'Color',[0.85 0.1 0.16],'LineWidth',2,'Marker',marker, ...
        'DisplayName',sprintf('Динамическая кластеризация | %c',names(a)));
    formatAxes(axs(a)); ylabel(axs(a),sprintf('%c, %s',names(a),J.units),'Interpreter','none');
    if a<3, set(axs(a),'XTickLabel',[]); end
end
xlabel(axs(3),'Время по валидным кадрам, с'); linkaxes(axs,'x');
choices=[{'Все 36','Датчики 1–18','Датчики 19–36','Только средние (чёрная + красная)'}, ...
    arrayfun(@(s) sprintf('Датчик %02d',s),1:36,'UniformOutput',false)];
sensorBox=uicontrol(f,'Style','popupmenu','String',choices,'Value',1,'Units','normalized', ...
    'Position',[0.085 0.935 0.19 0.04],'Callback',@refreshFigure);
modeBox=uicontrol(f,'Style','popupmenu', ...
    'String',{'Абсолютные значения','Минус среднее каждого датчика за ВСЮ запись'}, ...
    'Value',1+double(J.center),'Units','normalized','Position',[0.285 0.935 0.35 0.04],'Callback',@refreshFigure);
uicontrol(f,'Style','pushbutton','String','Сохранить текущий вид','Units','normalized', ...
    'Position',[0.65 0.935 0.18 0.04],'Callback',@saveView);
uicontrol(f,'Style','pushbutton','String','Сброс масштаба','Units','normalized', ...
    'Position',[0.84 0.935 0.12 0.04],'Callback',@resetView);
uicontrol(f,'Style','text','String',sprintf('%s | %s | чёрная: равное среднее 36; красная: динамическая кластеризация | показано %d из %d', ...
    J.title,J.units,numel(td),J.nRows),'Units','normalized','Position',[0.085 0.89 0.875 0.035], ...
    'BackgroundColor','w','HorizontalAlignment','left');
S=struct('axes',axs,'lines',hnd,'values',vd,'ref',ref,'dynC',J.dynC,'dynA',J.dynA, ...
    'sensorBox',sensorBox,'modeBox',modeBox);
setappdata(f,'MIIBState',S); setappdata(f,'MIIBSaveBase',fullfile(out,J.name));
refreshFigure(sensorBox,[]); dcm=datacursormode(f); set(dcm,'UpdateFcn',@tipText); drawnow;
end

function refreshFigure(source,~)
f=ancestor(source,'figure'); if isempty(f) || ~isgraphics(f), return; end
S=getappdata(f,'MIIBState'); if isempty(S), return; end
choice=get(S.sensorBox,'Value'); centered=get(S.modeBox,'Value')==2;
switch choice
    case 1, selected=1:36;
    case 2, selected=1:18;
    case 3, selected=19:36;
    case 4, selected=[];
    otherwise, selected=choice-4;
end
names='XYZ';
for a=1:3
    for s=1:36
        y=S.values(:,s,a); if centered, y=y-S.ref(s,a); end
        vis='off'; if any(selected==s), vis='on'; end
        if numel(selected)==1, c=[0.45 0.45 0.45]; lw=1.3;
        elseif s<=18, c=[0.62 0.76 0.94]; lw=0.65;
        else, c=[0.95 0.73 0.57]; lw=0.65; end
        set(S.lines(s,a),'YData',y,'Visible',vis,'Color',c,'LineWidth',lw);
    end
    y=mean(S.values(:,:,a),2); if centered, y=y-mean(S.ref(:,a)); end
    set(S.lines(37,a),'YData',y);
    if centered, set(S.lines(38,a),'YData',S.dynC(:,a)); else, set(S.lines(38,a),'YData',S.dynA(:,a)); end
    lh=gobjects(0,1); labels=cell(0,1);
    if numel(selected)==1
        lh(end+1,1)=S.lines(selected,a); labels{end+1,1}=sprintf('Датчик %02d',selected);
    elseif ~isempty(selected)
        if any(selected<=18), ss=selected(find(selected<=18,1)); lh(end+1,1)=S.lines(ss,a); labels{end+1,1}='Датчики 1–18'; end
        if any(selected>=19), ss=selected(find(selected>=19,1)); lh(end+1,1)=S.lines(ss,a); labels{end+1,1}='Датчики 19–36'; end
    end
    lh(end+1,1)=S.lines(37,a); labels{end+1,1}='Равное среднее 36';
    lh(end+1,1)=S.lines(38,a); labels{end+1,1}='Динамическая кластеризация';
    legend(S.axes(a),lh,labels,'Location','northeast','Interpreter','none','FontSize',9);
    if centered, prefix='Минус среднее за всю запись | '; else, prefix='Абсолютные значения | '; end
    title(S.axes(a),[prefix names(a)],'Interpreter','none'); set(S.axes(a),'YLimMode','auto');
end
drawnow limitrate;
end

function f=passportFigure(P,A,D)
f=figure('Name','Паспорт дрейфа в запуске | град/ч','NumberTitle','off','Color','w','Visible','on', ...
    'Units','normalized','Position',[0.03 0.1 0.94 0.77]);
choices=[{'Массив: равное среднее 36','Массив: динамическая кластеризация'}, ...
    arrayfun(@(s) sprintf('Датчик %02d',s),1:36,'UniformOutput',false)];
sensorBox=uicontrol(f,'Style','popupmenu','String',choices,'Value',1,'Units','normalized', ...
    'Position',[0.02 0.92 0.26 0.05],'Callback',@refreshPassport);
w=unique(A.window_sec).'; wc=[{'Все окна'},arrayfun(@(x) sprintf('%g с',x),w,'UniformOutput',false)];
windowBox=uicontrol(f,'Style','popupmenu','String',wc,'Value',1,'Units','normalized', ...
    'Position',[0.3 0.92 0.18 0.05],'Callback',@refreshPassport);
uicontrol(f,'Style','text','BackgroundColor','w','HorizontalAlignment','left','Units','normalized', ...
    'Position',[0.02 0.85 0.95 0.04],'String','NaN = нет оценки. 3 СКО — описательная величина, не гарантия точности.');
cols={'sensor_id','axis','window_sec','n_blocks','sigma_block_dph','sigma3_block_dph','peak_to_peak_dph', ...
    'max_abs_centered_dph','end_start_dph','linear_change_dph','residual_noise_std_dph','dropped_tail_sec','status'};
labels={'ID (0 равное, -1 кластер.)','Ось','Окно, с','Блоков','СКО, град/ч','3 СКО, град/ч','Размах, град/ч', ...
    'Макс. |откл.|, град/ч','Конец–начало, град/ч','Тренд, град/ч','Остаточное СКО, град/ч','Хвост, с','Статус'};
grid=uitable(f,'Units','normalized','Position',[0.01 0.02 0.98 0.82],'ColumnName',labels, ...
    'ColumnWidth',repmat({125},1,numel(labels)),'RowName',[]);
S=struct('data',[A;D;P],'sensorBox',sensorBox,'windowBox',windowBox,'windows',w,'grid',grid,'columns',{cols});
setappdata(f,'MIIBPassport',S); refreshPassport(sensorBox,[]); drawnow;
end

function refreshPassport(source,~)
f=ancestor(source,'figure'); S=getappdata(f,'MIIBPassport'); v=get(S.sensorBox,'Value');
if v==1, id=0; elseif v==2, id=-1; else, id=v-2; end
sel=S.data.sensor_id==id; wi=get(S.windowBox,'Value');
if wi>1, sel=sel & S.data.window_sec==S.windows(wi-1); end
set(S.grid,'Data',table2cell(S.data(sel,S.columns)));
end

function f=passportPlots(A,D,out)
f=figure('Name','Дрейф массива по окнам осреднения','NumberTitle','off','Color','w','Visible','on', ...
    'Units','normalized','Position',[0.05 0.07 0.9 0.82]);
fields={'sigma_block_dph','peak_to_peak_dph','max_abs_centered_dph','end_start_dph','linear_change_dph','residual_noise_std_dph'};
heads={'СКО блоковых средних','Размах блоковых средних','Максимум модуля отклонения', ...
    'Последний блок минус первый','Изменение по линейному тренду','Остаточное СКО внутри блоков'};
colors=lines(3); names='XYZ';
for j=1:6
    ax=subplot(2,3,j,'Parent',f); hold(ax,'on');
    for a=1:3
        PA=sortrows(A(strcmp(A.axis,names(a)),:),'window_sec');
        PD=sortrows(D(strcmp(D.axis,names(a)),:),'window_sec');
        plot(ax,PA.window_sec,PA.(fields{j}),'-o','Color',colors(a,:),'LineWidth',1.4, ...
            'DisplayName',[names(a) ' равное']);
        plot(ax,PD.window_sec,PD.(fields{j}),'--s','Color',colors(a,:),'LineWidth',1.4, ...
            'DisplayName',[names(a) ' кластер.']);
    end
    set(ax,'XScale','log'); formatAxes(ax); title(ax,heads{j});
    xlabel(ax,'Окно, с'); ylabel(ax,'град/ч'); legend(ax,'show','Location','best','FontSize',8);
end
setappdata(f,'MIIBSaveBase',fullfile(out,'gyro_drift_passport_array'));
dcm=datacursormode(f); set(dcm,'UpdateFcn',@tipText); drawnow;
end

function formatAxes(ax)
set(ax,'FontName','Arial','FontSize',10,'LineWidth',0.8,'Box','on','Color','w', ...
    'GridColor',[0.5 0.55 0.6],'GridAlpha',0.2); grid(ax,'on');
end

function tt=tipText(~,event)
p=event.Position; tt={get(event.Target,'DisplayName'),sprintf('X: %.9g',p(1)),sprintf('Y: %.12g',p(2))};
end

function resetView(source,~)
f=ancestor(source,'figure'); ax=findall(f,'Type','axes');
for j=1:numel(ax), set(ax(j),'XLimMode','auto','YLimMode','auto'); end
end

function saveView(source,~)
f=ancestor(source,'figure'); base=[getappdata(f,'MIIBSaveBase') '_view'];
try, writeFigure(f,base,true,true); fprintf('Сохранён вид: %s (.fig / .png)\n',base);
catch err, warning('MIIB:SaveFigure','Ошибка сохранения: %s',err.message); end
end

function autoSave(f,cfg)
if ~(cfg.saveFIG || cfg.savePNG), return; end
try, writeFigure(f,getappdata(f,'MIIBSaveBase'),cfg.saveFIG,cfg.savePNG);
catch err, warning('MIIB:SaveFigure','Ошибка сохранения: %s. Окно остаётся открытым.',err.message); end
end

function writeFigure(f,base,doFIG,doPNG)
if doFIG, savefig(f,[base '.fig']); end
if doPNG, set(f,'PaperPositionMode','auto'); print(f,[base '.png'],'-dpng','-r150','-noui'); end
end

function selfTest(cfg)
assert(crcBytes(uint8('123456789').')==uint16(hex2dec('29B1')),'MIIB selftest: CRC failed.');
z=repmat(reshape(1:11,[],1,1),1,36,3); ref=fullMean(z);
assert(all(ref(:)==6),'MIIB selftest: full-record mean failed.');
b=blockMeans(z,4); c=centerOnFullMean(b,ref);
assert(all(reshape(c(1,:,:),[],1)==-3.5) && all(reshape(c(2,:,:),[],1)==0.5),'MIIB selftest: recentering detected.');
assert(all(reshape(mean(c,1),[],1)==-1.5),'MIIB selftest: tail reference failed.');
T=metrics(c,'gyro','deg/h',4,4,true);
assert(height(T)==108 && T.sensor_id(4)==2 && strcmp(T.axis{4},'X'),'MIIB selftest: table order failed.');
[ns,na]=residualNoise(z,b,4);
assert(max(abs(ns(:)-sqrt(10/6)))<1e-12 && max(abs(na(:)-sqrt(10/6)))<1e-12,'MIIB selftest: residual noise failed.');
P=driftPassport(c,ref,1:36,4,4,11,1,ns);
assert(height(P)==108 && all(P.end_start_dph==4) && max(abs(P.linear_change_dph-4))<1e-10 ...
    && all(P.dropped_tail_sec==3),'MIIB selftest: drift passport failed.');
One=driftPassport(c(1,:,:),ref,1:36,4,4,11,1,ns);
assert(all(isnan(One.sigma_block_dph)) && all(isnan(One.end_start_dph)) && all(isnan(One.linear_change_dph)), ...
    'MIIB selftest: one block must not imply zero drift.');
Zero=driftPassport(zeros(0,36,3),ref,1:36,1000,1000,11,1,nan(36,3));
assert(all(Zero.n_blocks==0) && all(isnan(Zero.max_abs_centered_dph)),'MIIB selftest: empty window failed.');
Bad=driftPassport(reshape([0;NaN],2,1,1).*ones(1,1,3),zeros(1,3),-1,1,1,2,1,nan(1,3));
assert(all(isnan(Bad.sigma_block_dph)) && strcmp(Bad.status{1},'insufficient_dynamic_points'),'MIIB selftest: NaN dynamic point failed.');
[bm,bs]=blockMeansStd(z,4);
assert(isequal(bm,blockMeans(z,4)) && all(abs(bs(:)-sqrt(5/3))<1e-12),'MIIB selftest: block std failed.');
q=cfg; q.clusterWindow=4;
mm=linspace(-.005,.005,36).'; ss=1+linspace(-.05,.05,36).';
[w,fl,K]=clusterWeights(mm,ss,q);
assert(all(abs(w-1/36)<1e-15) && fl==0 && K==1,'MIIB selftest: homogeneous group was split.');
m2=mm; m2(36)=50; [w,fl,K]=clusterWeights(m2,ss,q);
assert(w(36)==0 && abs(sum(w)-1)<1e-12 && all(abs(w(1:35)-1/35)<1e-12) && K==2 && fl==0,'MIIB selftest: bias outlier not removed.');
[w3,~,~]=clusterWeights(m2+200,ss,q); assert(max(abs(w3-w))<1e-12,'MIIB selftest: common offset changed clustering.');
m2=mm; m2(29:36)=m2(29:36)+30; [w,~,~]=clusterWeights(m2,ss,q);
assert(all(w(29:36)==0) && abs(sum(w)-1)<1e-12 && sum(w>0)==28,'MIIB selftest: secondary bias group retained.');
q2=q; q2.secondaryWeight=0.5; [w,~,~]=clusterWeights(m2,ss,q2);
assert(abs(w(1)/w(36)-2)<1e-12,'MIIB selftest: secondary weight failed.');
s2=ss; s2(4)=100; [w,~,~]=clusterWeights(mm,s2,q);
assert(w(4)==0 && sum(w>0)==35,'MIIB selftest: noisy sensor not removed.');
m2=mm; m2(10)=NaN; [w,~,~]=clusterWeights(m2,ss,q);
assert(w(10)==0 && sum(w>0)==35,'MIIB selftest: non-finite sensor retained.');
m2=nan(36,1); m2(1:17)=1; [w,fl,~]=clusterWeights(m2,ss,q);
assert(all(isnan(w)) && fl==2,'MIIB selftest: insufficient sensors must give NaN.');
rf=repmat((1:36).',1,3); v=repmat(reshape(rf,1,36,3),8,1,1);
v(1:4,36,:)=v(1:4,36,:)+100; v(5:8,1,:)=v(5:8,1,:)-100;
Dd=dynamicCluster(v,rf,1,q,8);
assert(all(Dd.centered(:)==0) && all(Dd.nKept(:)==35),'MIIB selftest: dynamic centered output failed.');
assert(all(all(abs(Dd.absolute(1:4,:)-18)<1e-12)) && all(all(abs(Dd.absolute(5:8,:)-19)<1e-12)),'MIIB selftest: dynamic absolute output failed.');
assert(Dd.weights(1,36,1)==0 && Dd.weights(2,1,1)==0 && Dd.weights(2,36,1)>0,'MIIB selftest: window weights failed.');
bb=repmat(reshape(rf,1,36,3),2,1,1); bb(1,36,:)=bb(1,36,:)+100; bb(2,1,:)=bb(2,1,:)-100;
Db=dynamicClusterBlocks(bb,ones(2,36,3),rf,1,q);
assert(all(Db.centered(:)==0) && all(abs(Db.absolute(:,1)-[18;19])<1e-12),'MIIB selftest: block clustering failed.');
frame=zeros(690,1,'uint8'); frame(1:2)=uint8([170;85]); frame(3)=uint8(7);
gc=repmat([-524288 -1 524287],36,1); ac=repmat([0 1 -524288],36,1); p=zeros(19,36,'uint8');
for a=1:3
    g=uint32(mod(gc(:,a).',1048576)); av=uint32(mod(ac(:,a).',1048576));
    p(2*a-1,:)=uint8(bitshift(av,-12)); p(2*a,:)=uint8(bitand(bitshift(av,-4),uint32(255)));
    p(2*a+5,:)=uint8(bitshift(g,-12)); p(2*a+6,:)=uint8(bitand(bitshift(g,-4),uint32(255)));
    p(16+a,:)=uint8(bitor(bitshift(bitand(av,uint32(15)),4),bitand(g,uint32(15))));
end
frame(5:688)=p(:); crc=crcBytes(frame(3:688));
frame(689)=uint8(bitand(crc,uint16(255))); frame(690)=uint8(bitshift(crc,-8));
[h,counter,S]=findFrames(frame.'); [g,a]=decodeFrames(frame.',h,cfg);
eg=gc.*cfg.gyroScale; ea=ac.*cfg.accelScale;
eg(19:36,1:2)=-eg(19:36,1:2); ea(19:36,1:2)=-ea(19:36,1:2);
assert(S.valid_frames==1 && counter==7 && isequal(reshape(g,36,3),eg) && isequal(reshape(a,36,3),ea), ...
    'MIIB selftest: single-frame decoding failed.');
end
