function results = sweep_pid_filters(csvPath, outputDirectory)
%% Evaluate gyro-rate and PID-derivative filter cutoffs from one capture
% This function replays the raw roll/pitch gyroscope channels contained in a
% telemetry CSV through the same first-order filters used by the firmware.
% It then estimates PID and mixer activity for a grid of cutoff frequencies.
%
% The result is an OFFLINE tuning aid. Telemetry is recorded at about 100 Hz,
% while the real controller runs from fresh IMU samples at about 416 Hz.
% Therefore, frequencies above the telemetry Nyquist limit and the skipped
% IMU samples cannot be reconstructed. Confirm the recommended candidates in
% short, propeller-free tests and then in restrained flight tests.
%
% Usage:
%   results = sweep_pid_filters;
%   results = sweep_pid_filters("C:\path\telemetry.csv");
%   results = sweep_pid_filters("C:\path\telemetry.csv", "C:\output");

clc;
close all;

%% User-editable analysis configuration

rateCutoffsHz = [8 10 12 15 18 20 25 30 35];
derivativeCutoffsHz = [5 8 10 12 15 18 20 25 30];

currentRateCutoffHz = 20.0;
currentDerivativeCutoffHz = 15.0;
controllerRateHz = 416.0;
controlBandwidthHz = 10.0;

activeThrottleThreshold = 0.20;
minimumControlThrottle = 0.05;
integratorEnableThrottle = 0.35;

% Current inner-loop gains copied from flight_control.c.
rollPid = struct('kp', 0.0025, 'ki', 0.0008, 'kd', 0.000010, ...
                 'integratorLimit', 0.08, 'outputLimit', 0.28);
pitchPid = struct('kp', 0.0030, 'ki', 0.0015, 'kd', 0.000012, ...
                  'integratorLimit', 0.14, 'outputLimit', 0.32);

% Candidates must preserve enough response at the requested control band.
minimumRateMagnitude = 0.80;
minimumDerivativeMagnitude = 0.70;

% Score weights. Lower score is better among candidates that pass the
% response constraints. The score balances smoothness and phase delay.
scoreWeights = struct( ...
    'pidStep', 0.30, ...
    'motorStep', 0.25, ...
    'derivativeStep', 0.20, ...
    'rateStep', 0.15, ...
    'delay', 0.10);

%% Select and load telemetry

if nargin < 1 || strlength(string(csvPath)) == 0
    csvPath = newestTelemetryCsv();
end
if ~isfile(csvPath)
    error('Telemetry CSV not found: %s', string(csvPath));
end

if nargin < 2 || strlength(string(outputDirectory)) == 0
    outputDirectory = fullfile(fileparts(char(csvPath)), ...
                               'Filter_Sweep_Output');
end
if ~isfolder(outputDirectory)
    mkdir(outputDirectory);
end

telemetry = readtable(csvPath, 'VariableNamingRule', 'preserve');
requiredVariables = [ ...
    "timestamp_us", "imu_gyro_x", "imu_gyro_y", ...
    "roll_rate_measured", "pitch_rate_measured", ...
    "roll_rate_setpoint", "pitch_rate_setpoint", ...
    "roll_derivative", "pitch_derivative", ...
    "roll_output", "pitch_output", "yaw_output", ...
    "throttle_setpoint", "motor_front_left", ...
    "motor_front_right", "motor_rear_right", "motor_rear_left"];
validateVariables(telemetry, requiredVariables);

[data, capture] = prepareCapture(telemetry, activeThrottleThreshold);

fprintf('\nMCU_Cuadcopter filter sweep\n');
fprintf('CSV: %s\n', string(csvPath));
fprintf('Samples: %d | Duration: %.2f s | Telemetry: %.2f Hz\n', ...
    capture.sampleCount, capture.durationS, capture.telemetryRateHz);
fprintf('Active samples (throttle > %.2f): %d\n', ...
    activeThrottleThreshold, capture.activeSampleCount);
fprintf(['Offline replay is limited to %.2f Hz telemetry; firmware response ' ...
         'constraints use %.1f Hz.\n\n'], ...
    capture.telemetryRateHz, controllerRateHz);

%% Sweep every rate-filter and derivative-filter combination

rowCount = numel(rateCutoffsHz) * numel(derivativeCutoffsHz);
rateColumn = zeros(rowCount, 1);
derivativeColumn = zeros(rowCount, 1);
rateMagnitudeColumn = zeros(rowCount, 1);
derivativeMagnitudeColumn = zeros(rowCount, 1);
rateDelayColumn = zeros(rowCount, 1);
derivativeDelayColumn = zeros(rowCount, 1);
rateStepColumn = zeros(rowCount, 1);
derivativeStepColumn = zeros(rowCount, 1);
pidStepColumn = zeros(rowCount, 1);
motorStepColumn = zeros(rowCount, 1);
eligibleColumn = false(rowCount, 1);

simulations = cell(rowCount, 1);
row = 0;

for rateIndex = 1:numel(rateCutoffsHz)
    rateCutoff = rateCutoffsHz(rateIndex);
    filteredRollRate = firstOrderLowPass(data.rollGyroRawDegS, ...
                                         data.timeS, rateCutoff);
    filteredPitchRate = firstOrderLowPass(data.pitchGyroRawDegS, ...
                                          data.timeS, rateCutoff);

    [rateMagnitude, rateDelayMs] = discreteFilterResponse( ...
        rateCutoff, controlBandwidthHz, controllerRateHz);

    for derivativeIndex = 1:numel(derivativeCutoffsHz)
        derivativeCutoff = derivativeCutoffsHz(derivativeIndex);
        row = row + 1;

        roll = replayPid(filteredRollRate, data.rollRateSetpoint, ...
            data.timeS, data.throttle, derivativeCutoff, rollPid, ...
            minimumControlThrottle, integratorEnableThrottle);
        pitch = replayPid(filteredPitchRate, data.pitchRateSetpoint, ...
            data.timeS, data.throttle, derivativeCutoff, pitchPid, ...
            minimumControlThrottle, integratorEnableThrottle);
        motors = replayMixer(data.throttle, roll.output, pitch.output, ...
                             data.yawOutput);

        [derivativeMagnitude, derivativeDelayMs] = discreteFilterResponse( ...
            derivativeCutoff, controlBandwidthHz, controllerRateHz);

        rateColumn(row) = rateCutoff;
        derivativeColumn(row) = derivativeCutoff;
        rateMagnitudeColumn(row) = rateMagnitude;
        derivativeMagnitudeColumn(row) = derivativeMagnitude;
        rateDelayColumn(row) = rateDelayMs;
        derivativeDelayColumn(row) = derivativeDelayMs;
        rateStepColumn(row) = meanAxisStepRms( ...
            filteredRollRate, filteredPitchRate, data.pairMask);
        derivativeStepColumn(row) = meanAxisStepRms( ...
            roll.derivative, pitch.derivative, data.pairMask);
        pidStepColumn(row) = meanAxisStepRms( ...
            roll.output, pitch.output, data.pairMask);
        motorStepColumn(row) = motorStepP95(motors, data.pairMask);
        eligibleColumn(row) = ...
            (rateMagnitude >= minimumRateMagnitude) && ...
            (derivativeMagnitude >= minimumDerivativeMagnitude);

        simulations{row} = struct( ...
            'rollRate', filteredRollRate, ...
            'pitchRate', filteredPitchRate, ...
            'roll', roll, 'pitch', pitch, 'motors', motors);
    end
end

metricMatrix = [pidStepColumn, motorStepColumn, derivativeStepColumn, ...
                rateStepColumn, rateDelayColumn + derivativeDelayColumn];
normalizedMetrics = normalizeColumns(metricMatrix);
score = normalizedMetrics * [scoreWeights.pidStep; ...
    scoreWeights.motorStep; scoreWeights.derivativeStep; ...
    scoreWeights.rateStep; scoreWeights.delay];

% A failed response constraint must never outrank an eligible combination.
score(~eligibleColumn) = score(~eligibleColumn) + 1.0;

sweepTable = table( ...
    rateColumn, derivativeColumn, rateMagnitudeColumn, ...
    derivativeMagnitudeColumn, rateDelayColumn, derivativeDelayColumn, ...
    rateStepColumn, derivativeStepColumn, pidStepColumn, motorStepColumn, ...
    eligibleColumn, score, ...
    'VariableNames', { ...
    'RateCutoffHz', 'DerivativeCutoffHz', ...
    'RateMagnitudeAtControlBand', 'DerivativeMagnitudeAtControlBand', ...
    'RateDelayMsAtControlBand', 'DerivativeDelayMsAtControlBand', ...
    'RateStepRmsDegS', 'DerivativeStepRms', 'PidStepRms', ...
    'MotorStepP95Us', 'Eligible', 'Score'});

ranking = sortrows(sweepTable, {'Eligible', 'Score'}, {'descend', 'ascend'});
recommended = ranking(find(ranking.Eligible, 1, 'first'), :);
if isempty(recommended)
    error('No candidate passed the configured response constraints.');
end

recommendedRow = find( ...
    rateColumn == recommended.RateCutoffHz & ...
    derivativeColumn == recommended.DerivativeCutoffHz, 1, 'first');
currentRow = find( ...
    rateColumn == currentRateCutoffHz & ...
    derivativeColumn == currentDerivativeCutoffHz, 1, 'first');

%% Validate how closely the 100 Hz replay reproduces current logged signals

validation = buildValidationTable( ...
    data, simulations{currentRow}, data.activeMask);

fprintf('Best offline compromise: rate %.1f Hz, derivative %.1f Hz\n', ...
    recommended.RateCutoffHz, recommended.DerivativeCutoffHz);
fprintf('Current firmware:        rate %.1f Hz, derivative %.1f Hz\n', ...
    currentRateCutoffHz, currentDerivativeCutoffHz);
fprintf('Top eligible candidates:\n');
disp(ranking(1:min(10, height(ranking)), ...
    {'RateCutoffHz', 'DerivativeCutoffHz', 'RateMagnitudeAtControlBand', ...
     'DerivativeMagnitudeAtControlBand', 'PidStepRms', ...
     'MotorStepP95Us', 'Score'}));
fprintf('Replay-versus-logged validation (100 Hz approximation):\n');
disp(validation);

%% Create engineering plots

scoreMatrix = reshape(score, ...
    numel(derivativeCutoffsHz), numel(rateCutoffsHz));

figureOne = figure('Color', 'w', 'Name', 'Filter cutoff ranking', ...
                   'Position', [100 80 1250 760]);
tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile([2 1]);
imagesc(rateCutoffsHz, derivativeCutoffsHz, scoreMatrix);
set(gca, 'YDir', 'normal');
colormap(turbo);
colorbar;
hold on;
plot(currentRateCutoffHz, currentDerivativeCutoffHz, 'wo', ...
    'MarkerSize', 11, 'LineWidth', 2.2, 'DisplayName', 'Actual');
plot(recommended.RateCutoffHz, recommended.DerivativeCutoffHz, 'wp', ...
    'MarkerFaceColor', [1.0 0.75 0.1], 'MarkerSize', 15, ...
    'LineWidth', 1.5, 'DisplayName', 'Recomendado offline');
xlabel('Corte del filtro de tasa [Hz]');
ylabel('Corte del filtro derivativo [Hz]');
title({'Puntuación combinada', 'Menor es mejor; +1 si no conserva respuesta'});
legend('Location', 'southoutside');
grid on;

nexttile;
eligibleRows = sweepTable.Eligible;
scatter(sweepTable.RateDelayMsAtControlBand(eligibleRows) + ...
        sweepTable.DerivativeDelayMsAtControlBand(eligibleRows), ...
        sweepTable.MotorStepP95Us(eligibleRows), 45, ...
        sweepTable.Score(eligibleRows), 'filled');
hold on;
scatter(sweepTable.RateDelayMsAtControlBand(currentRow) + ...
        sweepTable.DerivativeDelayMsAtControlBand(currentRow), ...
        sweepTable.MotorStepP95Us(currentRow), 110, 'k', 'o', ...
        'LineWidth', 2);
xlabel(sprintf('Retardo combinado aproximado a %.0f Hz [ms]', ...
               controlBandwidthHz));
ylabel('Paso P95 estimado de motores [\mus]');
title('Compromiso entre retardo y suavidad');
grid on;
colorbar;

nexttile;
topCount = min(8, sum(ranking.Eligible));
top = ranking(1:topCount, :);
labels = compose('R%.0f / D%.0f', top.RateCutoffHz, ...
                 top.DerivativeCutoffHz);
barh(categorical(labels, flip(labels)), flip(top.Score), ...
    'FaceColor', [0.16 0.55 0.72]);
xlabel('Puntuación');
title('Mejores candidatos válidos');
grid on;

exportgraphics(figureOne, fullfile(outputDirectory, ...
    '01_ranking_filtros.png'), 'Resolution', 170);

figureTwo = plotReplayComparison(data, simulations{currentRow}, ...
    simulations{recommendedRow}, currentRateCutoffHz, ...
    currentDerivativeCutoffHz, recommended.RateCutoffHz, ...
    recommended.DerivativeCutoffHz);
exportgraphics(figureTwo, fullfile(outputDirectory, ...
    '02_reproduccion_temporal.png'), 'Resolution', 170);

figureThree = plotFilterResponses(rateCutoffsHz, derivativeCutoffsHz, ...
    controllerRateHz, controlBandwidthHz, currentRateCutoffHz, ...
    currentDerivativeCutoffHz, recommended.RateCutoffHz, ...
    recommended.DerivativeCutoffHz);
exportgraphics(figureThree, fullfile(outputDirectory, ...
    '03_respuesta_filtros.png'), 'Resolution', 170);

%% Save numerical results

writetable(ranking, fullfile(outputDirectory, 'ranking_filtros.csv'));
writetable(validation, fullfile(outputDirectory, ...
    'validacion_reproduccion_100Hz.csv'));

notesPath = fullfile(outputDirectory, 'LEEME_resultados.txt');
writeNotes(notesPath, csvPath, capture, recommended, ...
    currentRateCutoffHz, currentDerivativeCutoffHz, controllerRateHz, ...
    controlBandwidthHz, minimumRateMagnitude, ...
    minimumDerivativeMagnitude);

results = struct();
results.inputCsv = string(csvPath);
results.outputDirectory = string(outputDirectory);
results.capture = capture;
results.ranking = ranking;
results.recommended = recommended;
results.current = sweepTable(currentRow, :);
results.validation = validation;
results.limitation = ...
    "The replay uses the telemetry rate, not every 416 Hz IMU sample. " + ...
    "Use its ranking to choose flight-test candidates, not as proof of stability.";

fprintf('Results saved in: %s\n', string(outputDirectory));
fprintf(['Recommended next action: test the first 2 or 3 eligible rows ' ...
         'with identical maneuvers and compare new CSV captures.\n']);
end

function csvPath = newestTelemetryCsv()
downloads = fullfile(getenv('USERPROFILE'), 'Downloads');
files = dir(fullfile(downloads, 'telemetry_*.csv'));
if isempty(files)
    [fileName, folderName] = uigetfile('*.csv', ...
        'Select one MCU_Cuadcopter telemetry CSV');
    if isequal(fileName, 0)
        error('No telemetry CSV was selected.');
    end
    csvPath = fullfile(folderName, fileName);
    return;
end
[~, newestIndex] = max([files.datenum]);
csvPath = fullfile(files(newestIndex).folder, files(newestIndex).name);
fprintf('Automatically selected newest telemetry CSV: %s\n', csvPath);
end

function validateVariables(tableData, requiredVariables)
available = string(tableData.Properties.VariableNames);
missing = requiredVariables(~ismember(requiredVariables, available));
if ~isempty(missing)
    error('CSV is missing required variables: %s', strjoin(missing, ', '));
end
end

function [data, capture] = prepareCapture(T, activeThrottleThreshold)
timeS = double(T.timestamp_us) * 1.0e-6;
timeS = timeS - timeS(1);

requiredFinite = [timeS, double(T.imu_gyro_x), double(T.imu_gyro_y), ...
    double(T.roll_rate_setpoint), double(T.pitch_rate_setpoint), ...
    double(T.throttle_setpoint), double(T.yaw_output)];
valid = all(isfinite(requiredFinite), 2);
T = T(valid, :);
timeS = timeS(valid);

if numel(timeS) < 100
    error('The capture contains too few valid samples for a filter sweep.');
end

positiveDt = diff(timeS);
positiveDt = positiveDt(isfinite(positiveDt) & positiveDt > 0);
medianDt = median(positiveDt);
if isempty(positiveDt) || medianDt <= 0
    error('Telemetry timestamps are not valid or increasing.');
end

activeMask = double(T.throttle_setpoint) > activeThrottleThreshold;
pairMask = activeMask(2:end) & activeMask(1:end-1) & ...
           (diff(timeS) > 0) & (diff(timeS) < 2.5 * medianDt);
if sum(activeMask) < 50
    error('Fewer than 50 active-throttle samples are available.');
end

data = struct();
data.timeS = timeS;
data.rollGyroRawDegS = rad2deg(double(T.imu_gyro_x));
data.pitchGyroRawDegS = -rad2deg(double(T.imu_gyro_y));
data.rollRateSetpoint = double(T.roll_rate_setpoint);
data.pitchRateSetpoint = double(T.pitch_rate_setpoint);
data.rollRateLogged = double(T.roll_rate_measured);
data.pitchRateLogged = double(T.pitch_rate_measured);
data.rollDerivativeLogged = double(T.roll_derivative);
data.pitchDerivativeLogged = double(T.pitch_derivative);
data.rollOutputLogged = double(T.roll_output);
data.pitchOutputLogged = double(T.pitch_output);
data.yawOutput = double(T.yaw_output);
data.throttle = double(T.throttle_setpoint);
data.motorsLogged = [double(T.motor_front_left), ...
    double(T.motor_front_right), double(T.motor_rear_right), ...
    double(T.motor_rear_left)];
data.activeMask = activeMask;
data.pairMask = pairMask;
data.medianDtS = medianDt;

capture = struct();
capture.sampleCount = numel(timeS);
capture.durationS = timeS(end) - timeS(1);
capture.telemetryRateHz = 1.0 / medianDt;
capture.activeSampleCount = sum(activeMask);
capture.meanActiveThrottle = mean(data.throttle(activeMask));
end

function filtered = firstOrderLowPass(signal, timeS, cutoffHz)
filtered = zeros(size(signal));
filtered(1) = signal(1);
tau = 1.0 / (2.0 * pi * cutoffHz);
fallbackDt = median(diff(timeS));

for index = 2:numel(signal)
    dt = timeS(index) - timeS(index - 1);
    if ~isfinite(dt) || dt <= 0 || dt > 0.10
        dt = fallbackDt;
    end
    alpha = dt / (tau + dt);
    filtered(index) = filtered(index - 1) + ...
        alpha * (signal(index) - filtered(index - 1));
end
end

function axis = replayPid(measurement, setpoint, timeS, throttle, ...
                          derivativeCutoffHz, config, ...
                          minimumControlThrottle, integratorEnableThrottle)
sampleCount = numel(measurement);
axis.proportional = zeros(sampleCount, 1);
axis.integral = zeros(sampleCount, 1);
axis.derivative = zeros(sampleCount, 1);
axis.output = zeros(sampleCount, 1);

integrator = 0.0;
filteredDerivative = 0.0;
previousMeasurement = measurement(1);
initialized = false;
tau = 1.0 / (2.0 * pi * derivativeCutoffHz);
fallbackDt = median(diff(timeS));

for index = 1:sampleCount
    if index == 1
        dt = fallbackDt;
    else
        dt = timeS(index) - timeS(index - 1);
    end
    if ~isfinite(dt) || dt <= 0 || dt > 0.10
        dt = fallbackDt;
    end

    if throttle(index) <= minimumControlThrottle
        integrator = 0.0;
        filteredDerivative = 0.0;
        previousMeasurement = measurement(index);
        initialized = false;
        continue;
    end

    errorDegS = setpoint(index) - measurement(index);
    proportional = config.kp * errorDegS;

    if ~initialized
        previousMeasurement = measurement(index);
        filteredDerivative = 0.0;
        initialized = true;
    end

    rawDerivative = (measurement(index) - previousMeasurement) / dt;
    alpha = dt / (tau + dt);
    filteredDerivative = filteredDerivative + ...
        alpha * (rawDerivative - filteredDerivative);
    previousMeasurement = measurement(index);

    if throttle(index) >= integratorEnableThrottle
        candidateIntegrator = clampValue( ...
            integrator + config.ki * errorDegS * dt, ...
            -config.integratorLimit, config.integratorLimit);
    else
        integrator = 0.0;
        candidateIntegrator = 0.0;
    end

    derivative = -config.kd * filteredDerivative;
    unsaturated = proportional + candidateIntegrator + derivative;
    if ((unsaturated > config.outputLimit) && (errorDegS > 0)) || ...
       ((unsaturated < -config.outputLimit) && (errorDegS < 0))
        candidateIntegrator = integrator;
        unsaturated = proportional + candidateIntegrator + derivative;
    end

    integrator = candidateIntegrator;
    axis.proportional(index) = proportional;
    axis.integral(index) = integrator;
    axis.derivative(index) = derivative;
    axis.output(index) = clampValue(unsaturated, ...
                                   -config.outputLimit, config.outputLimit);
end
end

function motorsUs = replayMixer(throttle, roll, pitch, yaw)
correction = [roll + pitch + yaw, ...
             -roll + pitch - yaw, ...
             -roll - pitch + yaw, ...
              roll - pitch - yaw];
maximumCorrection = max(abs(correction), [], 2);
headroom = min(throttle, 1.0 - throttle);
scale = ones(size(throttle));
limited = maximumCorrection > headroom & maximumCorrection > 0;
scale(limited) = headroom(limited) ./ maximumCorrection(limited);
normalized = throttle + correction .* scale;
normalized = min(max(normalized, 0.0), 1.0);
motorsUs = 1000.0 + 1000.0 * normalized;
end

function [magnitude, delayMs] = discreteFilterResponse(cutoffHz, ...
                                                       frequencyHz, ...
                                                       sampleRateHz)
dt = 1.0 / sampleRateHz;
tau = 1.0 / (2.0 * pi * cutoffHz);
alpha = dt / (tau + dt);
omega = 2.0 * pi * frequencyHz / sampleRateHz;
response = alpha / (1.0 - (1.0 - alpha) * exp(-1i * omega));
magnitude = abs(response);
delayMs = -angle(response) / (2.0 * pi * frequencyHz) * 1000.0;
end

function value = meanAxisStepRms(axisOne, axisTwo, pairMask)
steps = [diff(axisOne), diff(axisTwo)];
selected = steps(pairMask, :);
value = sqrt(mean(selected(:).^2, 'omitnan'));
end

function value = motorStepP95(motorsUs, pairMask)
steps = abs(diff(motorsUs));
selected = steps(pairMask, :);
value = percentileNoToolbox(selected(:), 95.0);
end

function normalized = normalizeColumns(matrix)
normalized = zeros(size(matrix));
for column = 1:size(matrix, 2)
    minimum = min(matrix(:, column));
    maximum = max(matrix(:, column));
    if maximum > minimum
        normalized(:, column) = ...
            (matrix(:, column) - minimum) / (maximum - minimum);
    end
end
end

function validation = buildValidationTable(data, simulation, activeMask)
names = ["Roll rate"; "Pitch rate"; "Roll derivative"; ...
         "Pitch derivative"; "Roll PID output"; "Pitch PID output"];
logged = {data.rollRateLogged, data.pitchRateLogged, ...
          data.rollDerivativeLogged, data.pitchDerivativeLogged, ...
          data.rollOutputLogged, data.pitchOutputLogged};
replayed = {simulation.rollRate, simulation.pitchRate, ...
            simulation.roll.derivative, simulation.pitch.derivative, ...
            simulation.roll.output, simulation.pitch.output};

rmse = zeros(numel(names), 1);
correlation = zeros(numel(names), 1);
for index = 1:numel(names)
    first = logged{index}(activeMask);
    second = replayed{index}(activeMask);
    rmse(index) = sqrt(mean((first - second).^2, 'omitnan'));
    matrix = corrcoef(first, second, 'Rows', 'complete');
    if numel(matrix) >= 4
        correlation(index) = matrix(1, 2);
    else
        correlation(index) = NaN;
    end
end
validation = table(names, rmse, correlation, ...
    'VariableNames', {'Signal', 'ReplayRmse', 'Correlation'});
end

function figureHandle = plotReplayComparison(data, current, recommended, ...
    currentRate, currentDerivative, recommendedRate, recommendedDerivative)
activeIndices = find(data.activeMask);
centerIndex = activeIndices(round(numel(activeIndices) / 2));
halfWindowSamples = max(20, round(4.0 / data.medianDtS));
indices = max(1, centerIndex - halfWindowSamples): ...
          min(numel(data.timeS), centerIndex + halfWindowSamples);
time = data.timeS(indices) - data.timeS(indices(1));

figureHandle = figure('Color', 'w', 'Name', 'Offline replay', ...
                      'Position', [120 80 1280 820]);
tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(time, data.rollGyroRawDegS(indices), 'Color', [0.72 0.72 0.72]);
hold on;
plot(time, current.rollRate(indices), 'LineWidth', 1.2);
plot(time, recommended.rollRate(indices), 'LineWidth', 1.2);
ylabel('Roll [deg/s]');
title('Giroscopio y filtro de tasa');
legend('Crudo', sprintf('Actual %.0f Hz', currentRate), ...
       sprintf('Candidato %.0f Hz', recommendedRate));
grid on;

nexttile;
plot(time, data.pitchGyroRawDegS(indices), 'Color', [0.72 0.72 0.72]);
hold on;
plot(time, current.pitchRate(indices), 'LineWidth', 1.2);
plot(time, recommended.pitchRate(indices), 'LineWidth', 1.2);
ylabel('Pitch [deg/s]');
title('Giroscopio y filtro de tasa');
grid on;

nexttile;
plot(time, current.roll.derivative(indices), 'LineWidth', 1.1);
hold on;
plot(time, recommended.roll.derivative(indices), 'LineWidth', 1.1);
ylabel('D roll [norm]');
title(sprintf('Filtro D actual %.0f Hz frente a candidato %.0f Hz', ...
              currentDerivative, recommendedDerivative));
grid on;

nexttile;
plot(time, current.pitch.derivative(indices), 'LineWidth', 1.1);
hold on;
plot(time, recommended.pitch.derivative(indices), 'LineWidth', 1.1);
ylabel('D pitch [norm]');
title('Término derivativo reproducido');
grid on;

nexttile;
plot(time, current.roll.output(indices), 'LineWidth', 1.1);
hold on;
plot(time, recommended.roll.output(indices), 'LineWidth', 1.1);
ylabel('Salida roll [norm]');
xlabel('Tiempo local [s]');
title('Salida estimada del PID');
grid on;

nexttile;
currentMeanMotor = mean(current.motors(indices, :), 2);
recommendedMeanMotor = mean(recommended.motors(indices, :), 2);
currentDifferential = max(abs(current.motors(indices, :) - ...
    currentMeanMotor), [], 2);
recommendedDifferential = max(abs(recommended.motors(indices, :) - ...
    recommendedMeanMotor), [], 2);
plot(time, currentDifferential, 'LineWidth', 1.1);
hold on;
plot(time, recommendedDifferential, 'LineWidth', 1.1);
ylabel('Diferencial máximo [\mus]');
xlabel('Tiempo local [s]');
title('Actividad estimada de motores');
legend('Actual', 'Candidato');
grid on;
end

function figureHandle = plotFilterResponses(rateCutoffs, derivativeCutoffs, ...
    controllerRateHz, controlBandwidthHz, currentRate, currentDerivative, ...
    recommendedRate, recommendedDerivative)
frequencies = logspace(log10(0.5), ...
    log10(min(80.0, 0.45 * controllerRateHz)), 500);

figureHandle = figure('Color', 'w', 'Name', 'Filter responses', ...
                      'Position', [140 100 1180 720]);
tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plotResponseFamily(rateCutoffs, frequencies, controllerRateHz, ...
                   currentRate, recommendedRate);
xline(controlBandwidthHz, '--k', 'Banda de control');
ylabel('Magnitud');
title('Filtro de tasa');

nexttile;
plotDelayFamily(rateCutoffs, frequencies, controllerRateHz, ...
                currentRate, recommendedRate);
xline(controlBandwidthHz, '--k');
ylabel('Retardo equivalente [ms]');
title('Retardo del filtro de tasa');

nexttile;
plotResponseFamily(derivativeCutoffs, frequencies, controllerRateHz, ...
                   currentDerivative, recommendedDerivative);
xline(controlBandwidthHz, '--k');
xlabel('Frecuencia [Hz]');
ylabel('Magnitud');
title('Filtro del término D');

nexttile;
plotDelayFamily(derivativeCutoffs, frequencies, controllerRateHz, ...
                currentDerivative, recommendedDerivative);
xline(controlBandwidthHz, '--k');
xlabel('Frecuencia [Hz]');
ylabel('Retardo equivalente [ms]');
title('Retardo del filtro D');
end

function plotResponseFamily(cutoffs, frequencies, sampleRate, ...
                            currentCutoff, recommendedCutoff)
hold on;
for cutoff = cutoffs
    magnitude = responseVector(cutoff, frequencies, sampleRate, false);
    style = '-';
    width = 0.75;
    color = [0.65 0.70 0.75];
    if cutoff == currentCutoff
        color = [0.10 0.45 0.78];
        width = 2.2;
    end
    if cutoff == recommendedCutoff
        color = [0.95 0.50 0.08];
        width = 2.2;
    end
    semilogx(frequencies, magnitude, style, ...
        'Color', color, 'LineWidth', width);
end
ylim([0 1.05]);
grid on;
end

function plotDelayFamily(cutoffs, frequencies, sampleRate, ...
                         currentCutoff, recommendedCutoff)
hold on;
for cutoff = cutoffs
    delayMs = responseVector(cutoff, frequencies, sampleRate, true);
    width = 0.75;
    color = [0.65 0.70 0.75];
    if cutoff == currentCutoff
        color = [0.10 0.45 0.78];
        width = 2.2;
    end
    if cutoff == recommendedCutoff
        color = [0.95 0.50 0.08];
        width = 2.2;
    end
    semilogx(frequencies, delayMs, 'Color', color, 'LineWidth', width);
end
grid on;
end

function values = responseVector(cutoff, frequencies, sampleRate, returnDelay)
dt = 1.0 / sampleRate;
tau = 1.0 / (2.0 * pi * cutoff);
alpha = dt / (tau + dt);
omega = 2.0 * pi * frequencies / sampleRate;
response = alpha ./ (1.0 - (1.0 - alpha) .* exp(-1i * omega));
if returnDelay
    values = -angle(response) ./ (2.0 * pi * frequencies) * 1000.0;
else
    values = abs(response);
end
end

function writeNotes(path, csvPath, capture, recommended, currentRate, ...
                    currentDerivative, controllerRate, controlBandwidth, ...
                    minimumRateMagnitude, minimumDerivativeMagnitude)
file = fopen(path, 'w');
if file < 0
    warning('Could not create notes file: %s', path);
    return;
end
cleanup = onCleanup(@() fclose(file));

fprintf(file, 'MCU_Cuadcopter - offline filter sweep\n\n');
fprintf(file, 'Input CSV: %s\n', string(csvPath));
fprintf(file, 'Telemetry rate: %.3f Hz\n', capture.telemetryRateHz);
fprintf(file, 'Assumed firmware control rate: %.1f Hz\n', controllerRate);
fprintf(file, 'Current filter values: rate %.1f Hz, derivative %.1f Hz\n', ...
    currentRate, currentDerivative);
fprintf(file, 'Recommended offline candidate: rate %.1f Hz, derivative %.1f Hz\n', ...
    recommended.RateCutoffHz, recommended.DerivativeCutoffHz);
fprintf(file, 'Evaluation frequency: %.1f Hz\n', controlBandwidth);
fprintf(file, 'Minimum accepted magnitude: rate %.2f, derivative %.2f\n\n', ...
    minimumRateMagnitude, minimumDerivativeMagnitude);
fprintf(file, ['Interpretation:\n' ...
    '- Lower step metrics indicate smoother estimated PID/motor commands.\n' ...
    '- Lower cutoff values suppress more vibration but add phase delay.\n' ...
    '- Eligible candidates preserve the configured response magnitude.\n' ...
    '- Score ranks compromises; it does not prove closed-loop stability.\n\n']);
fprintf(file, ['Critical limitation:\n' ...
    'The CSV records about 100 Hz, but the real controller processes fresh ' ...
    'IMU samples at about 416 Hz. Missing samples and content above 50 Hz ' ...
    'cannot be reconstructed. Test the best candidates on hardware using ' ...
    'the same maneuver and compare new captures.\n']);
end

function value = clampValue(value, minimum, maximum)
value = min(max(value, minimum), maximum);
end

function value = percentileNoToolbox(values, percentile)
values = sort(values(isfinite(values)));
if isempty(values)
    value = NaN;
    return;
end
position = 1.0 + (numel(values) - 1.0) * percentile / 100.0;
lower = floor(position);
upper = ceil(position);
if lower == upper
    value = values(lower);
else
    fraction = position - lower;
    value = values(lower) * (1.0 - fraction) + values(upper) * fraction;
end
end
