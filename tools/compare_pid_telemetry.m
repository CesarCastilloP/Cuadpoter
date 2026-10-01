function results = compare_pid_telemetry(initialCsv, currentCsv, outputDirectory)
%% Compare two MCU_Cuadcopter PID telemetry captures
% This function compares an initial flight capture against a current one.
% It quantifies raw IMU vibration, sample-to-sample controller variation,
% motor command steps, and PID-output spectra without requiring additional
% MATLAB toolboxes.
%
% The analysis uses samples with throttle_setpoint > 0.20. Time histories
% are shown for context but are not treated as synchronized experiments,
% because pilot commands and flight trajectories can differ between files.
%
% Usage:
%   results = compare_pid_telemetry;
%   results = compare_pid_telemetry("initial.csv", "current.csv");
%   results = compare_pid_telemetry( ...
%       "initial.csv", "current.csv", "PID_Comparison_Output");

clc;
close all;

%% Analysis configuration

activeThrottleThreshold = 0.20; % Analyze controller activity above this throttle.
initialRateCutoffHz = 30.0;      % Initial gyro-rate low-pass cutoff [Hz].
currentRateCutoffHz = 20.0;      % Current gyro-rate low-pass cutoff [Hz].
initialDerivativeCutoffHz = 30.0; % Initial PID derivative cutoff [Hz].
currentDerivativeCutoffHz = 15.0; % Current PID derivative cutoff [Hz].
initialAngleKp = 3.5;            % Initial outer angle-loop gain [(deg/s)/deg].
currentAngleKp = 3.0;            % Current outer angle-loop gain [(deg/s)/deg].

initialColor = [0.36, 0.48, 0.68];
currentColor = [0.10, 0.62, 0.55];
highlightColor = [0.93, 0.61, 0.20];

%% Select input files

if nargin < 1 || strlength(string(initialCsv)) == 0
    initialCsv = selectCsvFile('Select the INITIAL telemetry CSV');
end

if nargin < 2 || strlength(string(currentCsv)) == 0
    currentCsv = selectCsvFile('Select the CURRENT telemetry CSV');
end

if ~isfile(initialCsv)
    error('Initial telemetry file not found: %s', string(initialCsv));
end

if ~isfile(currentCsv)
    error('Current telemetry file not found: %s', string(currentCsv));
end

if nargin < 3 || strlength(string(outputDirectory)) == 0
    outputDirectory = fullfile(fileparts(char(currentCsv)), ...
                               'PID_Comparison_Output');
end

if ~isfolder(outputDirectory)
    mkdir(outputDirectory);
end

%% Load and validate captures

initial = loadCapture(initialCsv, activeThrottleThreshold);
current = loadCapture(currentCsv, activeThrottleThreshold);

fprintf('\nMCU_Cuadcopter PID telemetry comparison\n');
fprintf('Initial file: %s\n', string(initialCsv));
fprintf('Current file: %s\n', string(currentCsv));
fprintf('Output directory: %s\n\n', string(outputDirectory));

captureSummary = table( ...
    ["Initial"; "Current"], ...
    [initial.sampleCount; current.sampleCount], ...
    [initial.durationS; current.durationS], ...
    [initial.telemetryRateHz; current.telemetryRateHz], ...
    [initial.lostFrames; current.lostFrames], ...
    [initial.activeSampleCount; current.activeSampleCount], ...
    [initial.meanActiveThrottle; current.meanActiveThrottle], ...
    'VariableNames', { ...
        'Capture', 'Samples', 'DurationS', 'TelemetryRateHz', ...
        'LostFrames', 'ActiveSamples', 'MeanActiveThrottle'});

disp('Capture summary:');
disp(captureSummary);

%% Calculate comparison metrics

metricNames = [ ...
    "Raw gyro X standard deviation"
    "Raw gyro Y standard deviation"
    "Roll measured-rate sample variation"
    "Pitch measured-rate sample variation"
    "Roll derivative-term sample variation"
    "Pitch derivative-term sample variation"
    "Roll PID-output sample variation"
    "Pitch PID-output sample variation"
];

metricUnits = [ ...
    "deg/s RMS"
    "deg/s RMS"
    "deg/s delta RMS"
    "deg/s delta RMS"
    "normalized delta RMS"
    "normalized delta RMS"
    "normalized delta RMS"
    "normalized delta RMS"
];

initialMetric = [ ...
    initial.rawGyroStdDegS(1)
    initial.rawGyroStdDegS(2)
    sampleDifferenceRms(initial.active.roll_rate_measured)
    sampleDifferenceRms(initial.active.pitch_rate_measured)
    sampleDifferenceRms(initial.active.roll_derivative)
    sampleDifferenceRms(initial.active.pitch_derivative)
    sampleDifferenceRms(initial.active.roll_output)
    sampleDifferenceRms(initial.active.pitch_output)
];

currentMetric = [ ...
    current.rawGyroStdDegS(1)
    current.rawGyroStdDegS(2)
    sampleDifferenceRms(current.active.roll_rate_measured)
    sampleDifferenceRms(current.active.pitch_rate_measured)
    sampleDifferenceRms(current.active.roll_derivative)
    sampleDifferenceRms(current.active.pitch_derivative)
    sampleDifferenceRms(current.active.roll_output)
    sampleDifferenceRms(current.active.pitch_output)
];

metricReductionPercent = 100.0 * ...
    (initialMetric - currentMetric) ./ initialMetric;

metricSummary = table( ...
    metricNames, metricUnits, initialMetric, currentMetric, ...
    metricReductionPercent, ...
    'VariableNames', { ...
        'Metric', 'Unit', 'Initial', 'Current', 'ReductionPercent'});

motorNames = ["Front left PF0"; "Front right PF2"; ...
              "Rear right PG0"; "Rear left PK4"];
motorFields = {'motor_front_left', 'motor_front_right', ...
               'motor_rear_right', 'motor_rear_left'};

initialMotorStepP95Us = zeros(4, 1);
currentMotorStepP95Us = zeros(4, 1);

for index = 1:4
    initialSteps = abs(diff(initial.active.(motorFields{index})));
    currentSteps = abs(diff(current.active.(motorFields{index})));
    initialMotorStepP95Us(index) = percentileLinear(initialSteps, 95.0);
    currentMotorStepP95Us(index) = percentileLinear(currentSteps, 95.0);
end

motorReductionPercent = 100.0 * ...
    (initialMotorStepP95Us - currentMotorStepP95Us) ./ ...
    initialMotorStepP95Us;

motorSummary = table( ...
    motorNames, initialMotorStepP95Us, currentMotorStepP95Us, ...
    motorReductionPercent, ...
    'VariableNames', { ...
        'Motor', 'InitialStepP95Us', 'CurrentStepP95Us', ...
        'ReductionPercent'});

disp('Controller variation summary:');
disp(metricSummary);
disp('Motor pulse-step summary:');
disp(motorSummary);

%% Estimate PID-output spectra

[initialRollFrequencyHz, initialRollPsd] = manualWelch( ...
    initial.longestActive.roll_output, initial.telemetryRateHz);
[currentRollFrequencyHz, currentRollPsd] = manualWelch( ...
    current.longestActive.roll_output, current.telemetryRateHz);

[initialPitchFrequencyHz, initialPitchPsd] = manualWelch( ...
    initial.longestActive.pitch_output, initial.telemetryRateHz);
[currentPitchFrequencyHz, currentPitchPsd] = manualWelch( ...
    current.longestActive.pitch_output, current.telemetryRateHz);

rollReference = max([initialRollPsd; currentRollPsd]);
pitchReference = max([initialPitchPsd; currentPitchPsd]);

initialRollPsdDb = 10.0 * log10(max(initialRollPsd / rollReference, eps));
currentRollPsdDb = 10.0 * log10(max(currentRollPsd / rollReference, eps));
initialPitchPsdDb = 10.0 * log10(max(initialPitchPsd / pitchReference, eps));
currentPitchPsdDb = 10.0 * log10(max(currentPitchPsd / pitchReference, eps));

%% Figure 1: measured improvement

figureSummary = figure( ...
    'Name', 'PID comparison - measured improvement', ...
    'Color', 'white', 'Position', [80, 60, 1450, 820]);
summaryLayout = tiledlayout(figureSummary, 2, 2, ...
    'TileSpacing', 'compact', 'Padding', 'compact');

nexttile(summaryLayout, 1);
normalizedMetric = 100.0 * [ ...
    ones(size(initialMetric)), currentMetric ./ initialMetric];
metricBars = barh(normalizedMetric, 'grouped');
metricBars(1).FaceColor = initialColor;
metricBars(2).FaceColor = currentColor;
set(gca, 'YTick', 1:numel(metricNames), ...
         'YTickLabel', shortMetricLabels(), 'YDir', 'reverse');
xlim([0, 110]);
xlabel('Variación relativa [%], inicial = 100%');
title('Ruido y variación rápida');
legend({'Inicial', 'Actual'}, 'Location', 'southoutside', ...
       'Orientation', 'horizontal');
grid on;

nexttile(summaryLayout, 2);
motorBars = bar([initialMotorStepP95Us, currentMotorStepP95Us], 'grouped');
motorBars(1).FaceColor = initialColor;
motorBars(2).FaceColor = currentColor;
set(gca, 'XTick', 1:4, 'XTickLabel', {'FL PF0', 'FR PF2', ...
    'RR PG0', 'RL PK4'});
ylabel('|\Delta pulso| percentil 95 [\mu s]');
title('Cambios rápidos enviados a los ESC');
legend({'Inicial', 'Actual'}, 'Location', 'southoutside', ...
       'Orientation', 'horizontal');
grid on;

nexttile(summaryLayout, 3);
plotPidSpectrum( ...
    initialRollFrequencyHz, initialRollPsdDb, ...
    currentRollFrequencyHz, currentRollPsdDb, ...
    initialColor, currentColor, 'Roll');

nexttile(summaryLayout, 4);
plotPidSpectrum( ...
    initialPitchFrequencyHz, initialPitchPsdDb, ...
    currentPitchFrequencyHz, currentPitchPsdDb, ...
    initialColor, currentColor, 'Pitch');

title(summaryLayout, ...
    'MCU Cuadcopter: comparación inicial y actual del controlador');

%% Figure 2: mathematical effect of modified parameters

figureParameters = figure( ...
    'Name', 'PID comparison - parameter effects', ...
    'Color', 'white', 'Position', [120, 100, 1300, 560]);
parameterLayout = tiledlayout(figureParameters, 1, 2, ...
    'TileSpacing', 'compact', 'Padding', 'compact');

frequencyHz = logspace(0, log10(50.0), 400).';
initialRateMagnitudeDb = firstOrderMagnitudeDb( ...
    frequencyHz, initialRateCutoffHz);
currentRateMagnitudeDb = firstOrderMagnitudeDb( ...
    frequencyHz, currentRateCutoffHz);
currentDerivativeMagnitudeDb = firstOrderMagnitudeDb( ...
    frequencyHz, currentDerivativeCutoffHz);

nexttile(parameterLayout, 1);
semilogx(frequencyHz, initialRateMagnitudeDb, ...
    'LineWidth', 2.0, 'Color', initialColor);
hold on;
semilogx(frequencyHz, currentRateMagnitudeDb, ...
    'LineWidth', 2.0, 'Color', currentColor);
semilogx(frequencyHz, currentDerivativeMagnitudeDb, ...
    'LineWidth', 2.0, 'Color', highlightColor);
xline(initialRateCutoffHz, ':', '30 Hz');
xline(currentRateCutoffHz, ':', '20 Hz');
xline(currentDerivativeCutoffHz, ':', '15 Hz');
hold off;
xlim([1, 50]);
ylim([-12, 0.5]);
xlabel('Frecuencia [Hz]');
ylabel('Ganancia del filtro [dB]');
title('Atenuación teórica de los filtros');
legend({'Inicial 30 Hz', 'Rate actual 20 Hz', ...
        'Derivada actual 15 Hz'}, 'Location', 'southwest');
grid on;

angleErrorDeg = linspace(-12.0, 12.0, 241).';

nexttile(parameterLayout, 2);
plot(angleErrorDeg, initialAngleKp * angleErrorDeg, ...
    'LineWidth', 2.0, 'Color', initialColor);
hold on;
plot(angleErrorDeg, currentAngleKp * angleErrorDeg, ...
    'LineWidth', 2.0, 'Color', currentColor);
plot(5.0, initialAngleKp * 5.0, 'o', ...
    'MarkerFaceColor', initialColor, 'MarkerEdgeColor', initialColor);
plot(5.0, currentAngleKp * 5.0, 'o', ...
    'MarkerFaceColor', currentColor, 'MarkerEdgeColor', currentColor);
hold off;
xlabel('Error angular [deg]');
ylabel('Velocidad angular solicitada [deg/s]');
title('Efecto de angle\_kp');
legend({ ...
    sprintf('Inicial Kp = %.1f', initialAngleKp), ...
    sprintf('Actual Kp = %.1f', currentAngleKp)}, ...
    'Location', 'northwest');
grid on;

title(parameterLayout, 'Efecto matemático de los parámetros modificados');

%% Figure 3: non-synchronized time-history reference

figureTime = figure( ...
    'Name', 'PID comparison - time histories', ...
    'Color', 'white', 'Position', [150, 80, 1450, 850]);
timeLayout = tiledlayout(figureTime, 2, 2, ...
    'TileSpacing', 'compact', 'Padding', 'compact');

nexttile(timeLayout, 1);
plotAttitudeHistory(initial, initialColor, 'Captura inicial');

nexttile(timeLayout, 2);
plotAttitudeHistory(current, currentColor, 'Captura actual');

nexttile(timeLayout, 3);
plotMotorHistory(initial, 'Motores: captura inicial');

nexttile(timeLayout, 4);
plotMotorHistory(current, 'Motores: captura actual');

title(timeLayout, ...
    'Referencia temporal: las maniobras del piloto no están sincronizadas');

%% Save figures and tables

saveFigure(figureSummary, fullfile(outputDirectory, ...
    '01_resumen_ruido_y_espectro.png'));
saveFigure(figureParameters, fullfile(outputDirectory, ...
    '02_efecto_de_parametros.png'));
saveFigure(figureTime, fullfile(outputDirectory, ...
    '03_historial_temporal.png'));

writetable(captureSummary, fullfile(outputDirectory, ...
    'resumen_capturas.csv'));
writetable(metricSummary, fullfile(outputDirectory, ...
    'resumen_controlador.csv'));
writetable(motorSummary, fullfile(outputDirectory, ...
    'resumen_motores.csv'));

%% Return results to the MATLAB workspace

results = struct();
results.initialFile = string(initialCsv);
results.currentFile = string(currentCsv);
results.outputDirectory = string(outputDirectory);
results.captureSummary = captureSummary;
results.metricSummary = metricSummary;
results.motorSummary = motorSummary;
results.initial = initial;
results.current = current;

fprintf('Generated files:\n');
fprintf('  01_resumen_ruido_y_espectro.png\n');
fprintf('  02_efecto_de_parametros.png\n');
fprintf('  03_historial_temporal.png\n');
fprintf('  resumen_capturas.csv\n');
fprintf('  resumen_controlador.csv\n');
fprintf('  resumen_motores.csv\n\n');

end

function selectedPath = selectCsvFile(dialogTitle)
% Open a file dialog and return the selected CSV path.

[selectedFile, selectedDirectory] = uigetfile( ...
    {'*.csv', 'Telemetry CSV files (*.csv)'}, dialogTitle);

if isequal(selectedFile, 0)
    error('CSV file selection was cancelled.');
end

selectedPath = fullfile(selectedDirectory, selectedFile);

end

function capture = loadCapture(csvFile, activeThrottleThreshold)
% Load one telemetry file and calculate timing and active-flight subsets.

data = readtable(csvFile, 'VariableNamingRule', 'preserve');

requiredVariables = { ...
    'sequence', 'timestamp_us', 'throttle_setpoint', ...
    'imu_gyro_x', 'imu_gyro_y', 'imu_gyro_z', ...
    'roll_rate_measured', 'pitch_rate_measured', ...
    'roll_angle', 'pitch_angle', ...
    'roll_angle_setpoint', 'pitch_angle_setpoint', ...
    'roll_derivative', 'pitch_derivative', ...
    'roll_output', 'pitch_output', ...
    'motor_front_left', 'motor_front_right', ...
    'motor_rear_right', 'motor_rear_left'};

missingVariables = setdiff(requiredVariables, data.Properties.VariableNames);
if ~isempty(missingVariables)
    error('Missing required CSV variables: %s', ...
          strjoin(missingVariables, ', '));
end

timeS = (double(data.timestamp_us) - double(data.timestamp_us(1))) * 1.0e-6;
sampleIntervalS = diff(timeS);
telemetryRateHz = 1.0 / median(sampleIntervalS);
sequenceDelta = diff(double(data.sequence));
lostFrames = sum(max(sequenceDelta - 1.0, 0.0));

activeMask = double(data.throttle_setpoint) > activeThrottleThreshold;
if nnz(activeMask) < 512
    error('Capture %s does not contain enough active-throttle samples.', ...
          string(csvFile));
end

[runStart, runEnd] = longestTrueRun(activeMask);
longestActiveMask = false(height(data), 1);
longestActiveMask(runStart:runEnd) = true;

capture = struct();
capture.file = string(csvFile);
capture.data = data;
capture.timeS = timeS;
capture.sampleCount = height(data);
capture.durationS = timeS(end);
capture.telemetryRateHz = telemetryRateHz;
capture.lostFrames = lostFrames;
capture.activeMask = activeMask;
capture.activeSampleCount = nnz(activeMask);
capture.meanActiveThrottle = mean(double(data.throttle_setpoint(activeMask)));
capture.active = data(activeMask, :);
capture.activeTimeS = timeS(activeMask);
capture.longestActive = data(longestActiveMask, :);
capture.longestActiveTimeS = timeS(longestActiveMask);
capture.rawGyroStdDegS = std(rad2deg([ ...
    double(data.imu_gyro_x(activeMask)), ...
    double(data.imu_gyro_y(activeMask)), ...
    double(data.imu_gyro_z(activeMask))]), 0, 1);

end

function [runStart, runEnd] = longestTrueRun(mask)
% Return one-based indices for the longest contiguous true interval.

edge = diff([false; logical(mask(:)); false]);
starts = find(edge == 1);
stops = find(edge == -1) - 1;

if isempty(starts)
    error('No true interval was found.');
end

[~, longestIndex] = max(stops - starts + 1);
runStart = starts(longestIndex);
runEnd = stops(longestIndex);

end

function value = sampleDifferenceRms(signal)
% Calculate RMS magnitude of adjacent telemetry-sample differences.

signal = double(signal(:));
delta = diff(signal);
value = sqrt(mean(delta .^ 2));

end

function value = percentileLinear(signal, percentile)
% Calculate a percentile without requiring the Statistics Toolbox.

sortedSignal = sort(double(signal(:)));
if isempty(sortedSignal)
    value = NaN;
    return;
end

position = 1.0 + (numel(sortedSignal) - 1.0) * percentile / 100.0;
lowerIndex = floor(position);
upperIndex = ceil(position);
fraction = position - lowerIndex;

value = sortedSignal(lowerIndex) * (1.0 - fraction) + ...
        sortedSignal(upperIndex) * fraction;

end

function [frequencyHz, psd] = manualWelch(signal, sampleRateHz)
% Estimate a one-sided Welch PSD without the Signal Processing Toolbox.

signal = double(signal(:));
maximumSegmentLength = 512;
segmentLength = min(maximumSegmentLength, ...
    2 ^ floor(log2(numel(signal))));

if segmentLength < 128
    error('At least 128 samples are required for spectral analysis.');
end

overlap = floor(segmentLength / 2);
step = segmentLength - overlap;
windowIndex = (0:(segmentLength - 1)).';
window = 0.5 - 0.5 * cos( ...
    2.0 * pi * windowIndex / (segmentLength - 1.0));
windowPower = sum(window .^ 2);

segmentStarts = 1:step:(numel(signal) - segmentLength + 1);
accumulatedPsd = zeros(segmentLength / 2 + 1, 1);

for segmentStart = segmentStarts
    segment = signal(segmentStart:(segmentStart + segmentLength - 1));
    segment = segment - mean(segment);
    spectrum = fft(segment .* window);
    segmentPsd = abs(spectrum(1:(segmentLength / 2 + 1))) .^ 2 / ...
        (sampleRateHz * windowPower);
    segmentPsd(2:(end - 1)) = 2.0 * segmentPsd(2:(end - 1));
    accumulatedPsd = accumulatedPsd + segmentPsd;
end

psd = accumulatedPsd / numel(segmentStarts);
frequencyHz = (0:(segmentLength / 2)).' * ...
    sampleRateHz / segmentLength;

validMask = frequencyHz >= 1.0 & ...
            frequencyHz <= min(45.0, 0.45 * sampleRateHz);
frequencyHz = frequencyHz(validMask);
psd = psd(validMask);

end

function magnitudeDb = firstOrderMagnitudeDb(frequencyHz, cutoffHz)
% Return the magnitude response of a first-order low-pass filter.

magnitude = 1.0 ./ sqrt(1.0 + (frequencyHz / cutoffHz) .^ 2);
magnitudeDb = 20.0 * log10(magnitude);

end

function plotPidSpectrum(initialFrequencyHz, initialPsdDb, ...
                         currentFrequencyHz, currentPsdDb, ...
                         initialColor, currentColor, axisName)
% Plot comparable normalized PID-output spectra.

plot(initialFrequencyHz, initialPsdDb, ...
    'LineWidth', 1.5, 'Color', initialColor);
hold on;
plot(currentFrequencyHz, currentPsdDb, ...
    'LineWidth', 1.5, 'Color', currentColor);
xline(15.0, ':', '15 Hz');
xline(30.0, ':', '30 Hz');
hold off;
xlim([1, 45]);
ylim([-40, 3]);
xlabel('Frecuencia observada [Hz]');
ylabel('PSD relativa [dB]');
title(sprintf('Espectro de salida PID: %s', axisName));
legend({'Inicial', 'Actual'}, 'Location', 'southwest');
grid on;

end

function plotAttitudeHistory(capture, plotColor, plotTitle)
% Plot measured and requested roll/pitch during active throttle.

timeS = capture.activeTimeS - capture.activeTimeS(1);
plot(timeS, capture.active.roll_angle, ...
    'Color', plotColor, 'LineWidth', 1.0);
hold on;
plot(timeS, capture.active.roll_angle_setpoint, '--', ...
    'Color', plotColor, 'LineWidth', 1.0);
plot(timeS, capture.active.pitch_angle, ...
    'Color', 0.55 * plotColor, 'LineWidth', 1.0);
plot(timeS, capture.active.pitch_angle_setpoint, '--', ...
    'Color', 0.55 * plotColor, 'LineWidth', 1.0);
hold off;
xlabel('Tiempo activo [s]');
ylabel('Ángulo [deg]');
title(plotTitle);
legend({'Roll', 'Roll SP', 'Pitch', 'Pitch SP'}, ...
       'Location', 'best');
grid on;

end

function plotMotorHistory(capture, plotTitle)
% Plot all four ESC pulse widths during active throttle.

timeS = capture.activeTimeS - capture.activeTimeS(1);
plot(timeS, capture.active.motor_front_left, 'LineWidth', 0.8);
hold on;
plot(timeS, capture.active.motor_front_right, 'LineWidth', 0.8);
plot(timeS, capture.active.motor_rear_right, 'LineWidth', 0.8);
plot(timeS, capture.active.motor_rear_left, 'LineWidth', 0.8);
hold off;
xlabel('Tiempo activo [s]');
ylabel('Pulso ESC [\mu s]');
title(plotTitle);
legend({'FL', 'FR', 'RR', 'RL'}, 'Location', 'best');
grid on;

end

function labels = shortMetricLabels()
% Return concise labels used by the normalized bar chart.

labels = { ...
    'Gyro X crudo'
    'Gyro Y crudo'
    'Rate roll'
    'Rate pitch'
    'D roll'
    'D pitch'
    'PID roll'
    'PID pitch'
};

end

function saveFigure(figureHandle, outputFile)
% Save one figure at presentation resolution with a compatibility fallback.

try
    exportgraphics(figureHandle, outputFile, 'Resolution', 200);
catch
    print(figureHandle, outputFile, '-dpng', '-r200');
end

end
