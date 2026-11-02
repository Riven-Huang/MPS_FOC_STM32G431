% 三环开环 Bode 图：电流环、速度环、位置环各一个独立窗口。
% 普通脚本，直接运行；需要 Control System Toolbox。
% 参数集中在前面。模型用于线性分析，不是实机扫频结果。
% 本文“截止频率”指开环 0 dB 交越频率，不是闭环 -3 dB 带宽。

%% 1. 电机参数（电机轴机械量，沿用原三环脚本）
resistanceOhm = 0.725;         % 等效相电阻，ohm
inductanceH = 4.10e-4;         % 等效相电感，H
torqueConstant = 0.60;         % 转矩常数，N*m/A
inertiaKgM2 = 3.0e-4;          % 折算到电机轴的等效惯量，kg*m^2
viscousFriction = 0;           % 黏性阻尼，N*m/(rad/s)

%% 2. 控制参数（通常只需要修改这一节）
currentRateHz = 10000;         % 电流环运行频率
currentDelayS = 1/currentRateHz; % 总延迟：一个控制周期，100 us；不再叠加旧预算
currentDesignHz = 500;        % 电流 PI 整定频率

speedDesignHz = 30;           % 速度 PI 整定参数，不等于实际 0 dB 交越频率
speedDamping = 1.0;           % 速度 PI 的二阶设计阻尼比
speedRateHz = 2000;           % 速度环运行频率
speedWindowSamples = 5;      % 测速窗口长度，以电流环周期计
speedSampleAgeS = 0.1e-3;    % 速度反馈样本龄假设，需按实际调整
speedFilterHz = 300;          % 速度反馈一阶低通截止频率
speedDelayS = speedSampleAgeS + speedWindowSamples/(2*currentRateHz) ...
    + 1/(2*speedRateHz);      % 沿用原脚本测速延迟预算；也可直接填写实测值

positionDesignHz = 5;        % 位置比例增益的整定频率
positionKi = 0;              % 位置积分增益，默认不使用积分
positionKd = 0.3;            % 速度反馈阻尼系数（沿用原脚本结构）
positionRateHz = 200;        % 位置环运行频率
positionSampleAgeS = 0.1e-3; % 位置反馈样本龄假设
positionDelayS = 1/(2*positionRateHz) + positionSampleAgeS;

% 默认按上述整定频率计算增益；需要直接调 PI 时在这里修改。
currentKp = 2*pi*currentDesignHz*inductanceH;
currentKi = 2*pi*currentDesignHz*resistanceOhm;
speedKp = 2*speedDamping*2*pi*speedDesignHz*inertiaKgM2/torqueConstant;
speedKi = (2*pi*speedDesignHz)^2*inertiaKgM2/torqueConstant;
positionKp = 2*pi*positionDesignHz;
% 这里速度增益使用机械 rad/s 域；固件若使用电角速度，需除以极对数。

%% 3. 绘图参数
% 留空时自动覆盖交越点及增益裕度标记；填写 [最小Hz 最大Hz] 可手动缩放。
currentRangeHz = [];
speedRangeHz = [];
positionRangeHz = [];
saveFigures = false;          % true：额外保存 PNG 和可编辑 FIG
scriptFolder = fileparts(mfilename('fullpath'));
outputFolder = fullfile(scriptFolder, 'three_loops_bode');

%% 4. 串级模型：从内到外闭合回路
s = tf('s');
currentPlant = tf(1, 1, 'InputDelay', currentDelayS) ...
    /(inductanceH*s + resistanceOhm);
currentController = currentKp + currentKi/s;
currentOpenLoop = currentController*currentPlant;
currentClosedLoop = feedback(currentOpenLoop, 1);

mechanicalPlant = torqueConstant/(inertiaKgM2*s + viscousFriction);
speedFeedback = tf(1, 1, 'InputDelay', speedDelayS)/(1 + s/(2*pi*speedFilterHz));
speedController = speedKp + speedKi/s;
speedForward = speedController*currentClosedLoop*mechanicalPlant;
speedOpenLoop = speedForward*speedFeedback;
speedClosedLoop = feedback(speedForward, speedFeedback);

positionFeedback = tf(1, 1, 'InputDelay', positionDelayS);
positionPlant = speedClosedLoop/s;
% 位置 D 项位于速度反馈支路；这里求的是用于稳定性分析的回路传递函数。
positionOpenLoop = ((positionKp + positionKi/s)*positionFeedback ...
    + positionKd*s*speedFeedback)*positionPlant;

%% 5. 三张 MATLAB 原生裕度图，分别弹窗
loopModels = {currentOpenLoop, speedOpenLoop, positionOpenLoop};
loopNames = ["电流环", "速度环", "位置环"];
fileNames = ["current_loop", "speed_loop", "position_loop"];
frequencyRanges = {currentRangeHz, speedRangeHz, positionRangeHz};
phaseMarginDeg = nan(3, 1);
crossoverHz = nan(3, 1);
gainMarginDb = nan(3, 1);
figureHandles = gobjects(3, 1);

for loopIndex = 1:3
    loopModel = loopModels{loopIndex};
    [gainMargin, phaseMarginDeg(loopIndex), gainCrossOmega, crossOmega] = margin(loopModel);
    crossoverHz(loopIndex) = crossOmega/(2*pi);
    gainMarginDb(loopIndex) = 20*log10(gainMargin);

    % 重复运行只替换本脚本的三张图，不关闭其他工作的窗口。
    plotTag = "ThreeLoopsBode_" + fileNames(loopIndex);
    delete(findall(groot, 'Type', 'figure', 'Tag', plotTag));
    figureHandles(loopIndex) = figure(Name=loopNames(loopIndex), ...
        NumberTitle='off', WindowStyle='normal', Tag=plotTag);
    margin(loopModel);
    bodeChart = findobj(figureHandles(loopIndex), 'Type', 'bode');
    % 横轴、原生标题和数据提示统一使用 Hz；模型运算仍采用 rad/s。
    bodeChart.FrequencyUnit = 'Hz';
    title(bodeChart, sprintf('%s | PM = %.1f deg | fc = %.2f Hz | GM = %.2f dB', ...
        loopNames(loopIndex), phaseMarginDeg(loopIndex), ...
        crossoverHz(loopIndex), gainMarginDb(loopIndex)));
    grid(bodeChart, 'on');

    rangeHz = frequencyRanges{loopIndex};
    if isempty(rangeHz) && isfinite(crossOmega) && crossOmega > 0
        upperOmega = 3*crossOmega;
        if isfinite(gainCrossOmega) && gainCrossOmega > 0
            upperOmega = max(upperOmega, 1.2*gainCrossOmega);
        end
        rangeHz = [crossOmega/10, upperOmega]/(2*pi);
    end
    if ~isempty(rangeHz)
        assert(numel(rangeHz) == 2 && all(isfinite(rangeHz)) ...
            && rangeHz(1) > 0 && rangeHz(2) > rangeHz(1), ...
            '频率范围必须是递增的两个正数，单位 Hz。');
        displayOmega = logspace(log10(2*pi*rangeHz(1)), log10(2*pi*rangeHz(2)), 1500);
        [mag, phase] = bode(loopModel, displayOmega);
        magLimits = [min(-10, floor(min(20*log10(mag(:)))/10)*10), ...
            max(10, ceil(max(20*log10(mag(:)))/10)*10)];
        phaseLimits = [min(-180, floor(min(phase(:))/45)*45), ...
            max(-135, ceil(max(phase(:))/45)*45)];
        setoptions(bodeChart, 'XLim', rangeHz, 'YLim', {magLimits; phaseLimits});
    end
    drawnow;
    % R2025a 原生 margin 在刷新坐标范围时会把标记横坐标写回 rad/s。
    % 仅修正原生标记的单位，不修改曲线或裕度；重绘后同样保持 Hz。
    if strcmp(version('-release'), '2025a')
        fixMarkers = @(~,~) alignMarginMarkersHz(figureHandles(loopIndex), ...
            crossOmega/(2*pi), phaseMarginDeg(loopIndex), ...
            gainCrossOmega/(2*pi), gainMarginDb(loopIndex));
        plotAxes = findall(figureHandles(loopIndex), 'Type', 'axes');
        markerListener = cell(numel(plotAxes), 1);
        for axesIndex = 1:numel(plotAxes)
            markerListener{axesIndex} = addlistener(plotAxes(axesIndex), 'MarkedClean', fixMarkers);
        end
        setappdata(figureHandles(loopIndex), 'HzMarginListener', markerListener);
        fixMarkers([], []);
        drawnow;
    end
    if saveFigures
        if ~isfolder(outputFolder)
            mkdir(outputFolder);
        end
        outputBase = fullfile(outputFolder, fileNames(loopIndex));
        exportgraphics(figureHandles(loopIndex), outputBase + ".png", Resolution=200);
        savefig(figureHandles(loopIndex), outputBase + ".fig");
    end
end

%% 6. 命令行结果（仅三环裕度与交越频率，不做其他扫描）
resultTable = table(loopNames(:), crossoverHz, phaseMarginDeg, gainMarginDb, ...
    VariableNames={'Loop', 'CrossoverHz', 'PhaseMarginDeg', 'GainMarginDb'});
disp(resultTable);

function alignMarginMarkersHz(fig, phaseHz, phaseMargin, gainHz, gainMargin)
% Work around R2025a BodeStabilityMarginView.updateDataByLimits unit handling.
% Keep the native marker objects and their MATLAB data tips.
    tags = {'BodePhaseMarginMarker', 'BodeGainMarginScatter'};
    lineTags = {'BodePhaseMarginXLine', 'BodeGainMarginXLine'};
    frequencies = [phaseHz, gainHz];
    levels = [phaseMargin-180, -gainMargin];
    referenceLevels = [-180, 0];
    for index = 1:2
        markers = findall(fig, 'Tag', tags{index});
        for marker = reshape(markers, 1, [])
            if isstruct(marker.UserData) && isfield(marker.UserData, 'ValueOutsideLimits')
                x = NaN;
                y = NaN;
            else
                x = frequencies(index);
                y = levels(index);
            end
            if ~isequaln(marker.XData, x) || ~isequaln(marker.YData, y)
                set(marker, 'XData', x, 'YData', y);
            end
        end
        lines = findall(fig, 'Tag', lineTags{index});
        for lineHandle = reshape(lines, 1, [])
            x = [frequencies(index), frequencies(index), NaN];
            y = [referenceLevels(index), levels(index), NaN];
            if ~isequaln(lineHandle.XData, x) || ~isequaln(lineHandle.YData, y)
                set(lineHandle, 'XData', x, 'YData', y);
            end
        end
    end
end
