function results = analyze_horizontal_displacement(csvFile)
%% MCU_Cuadcopter horizontal displacement analysis
% This script analyzes the six-axis IMU fields recorded by
% telemetry_dashboard.py. It reproduces the table-slide experiment used to
% evaluate whether horizontal velocity can be estimated from the current IMU.
%
% Three acceleration models are compared:
%   1. Local baseline subtraction only.
%   2. Gravity compensation using the firmware roll/pitch estimate.
%   3. Short-term attitude propagation using the raw gyroscope.
%
% The table experiment provides a known zero-velocity condition before and
% after every movement. The endpoint correction used below is valid for that
% experiment, but it must not be interpreted as an in-flight measurement.
% A vehicle moving at constant velocity cannot be distinguished from a
% stationary vehicle with a six-axis IMU alone.
%
% Usage:
%   results = analyze_horizontal_displacement;
%   results = analyze_horizontal_displacement("telemetry_capture.csv");

clc;
close all;

%% User configuration

gravityMps2 = 9.80665;

% Windows identified from the recorded MCU timestamp. Edit these values when
% processing a different experiment.
eventNames = {
    'Forward +X'
    'Backward -X'
    'Right -Y'
    'Left +Y'
};

eventWindowsS = [
    10.15, 12.95
    22.15, 24.95
    33.00, 35.05
    42.15, 44.95
];

% Expected displacement vectors from the physical 0.50 m marks.
expectedDisplacementM = [
     0.50,  0.00
    -0.50,  0.00
     0.00, -0.50
     0.00,  0.50
];

% A local baseline is calculated immediately before every movement.
baselineDurationS = 1.50;
baselineGapS = 0.15;

% Known stationary intervals used only to characterize sensor noise.
restNames = {'Rest 0'; 'Rest 1'; 'Rest 2'; 'Rest 3'; 'Rest 4'; 'Rest 5'};
restWindowsS = [
     0.0,  5.8
     7.2,  9.8
    14.0, 19.0
    26.0, 29.0
    37.0, 41.0
    47.0, 52.5
];

%% Select and load the CSV file

if nargin < 1 || strlength(string(csvFile)) == 0
    [selectedFile, selectedPath] = uigetfile( ...
        {'*.csv', 'Telemetry CSV files (*.csv)'}, ...
        'Select an MCU_Cuadcopter telemetry file');
    if isequal(selectedFile, 0)
        error('No telemetry file was selected.');
    end
    csvFile = fullfile(selectedPath, selectedFile);
end

if ~isfile(csvFile)
    error('Telemetry file not found: %s', string(csvFile));
end

data = readtable(csvFile);

requiredVariables = {
    'sequence', 'timestamp_us', ...
    'accel_x', 'accel_y', 'accel_z', ...
    'imu_gyro_x', 'imu_gyro_y', 'imu_gyro_z', ...
    'roll_angle', 'pitch_angle'
};

missingVariables = setdiff(requiredVariables, data.Properties.VariableNames);
if ~isempty(missingVariables)
    error('Missing required CSV variables: %s', strjoin(missingVariables, ', '));
end

timeS = (double(data.timestamp_us) - double(data.timestamp_us(1))) * 1.0e-6;
sampleDtS = diff(timeS);
sampleRateHz = 1.0 / median(sampleDtS);
sequenceDelta = diff(double(data.sequence));
lostFrames = sum(max(sequenceDelta - 1.0, 0.0));

fprintf('\nMCU_Cuadcopter IMU displacement analysis\n');
fprintf('File: %s\n', csvFile);
fprintf('Samples: %d\n', height(data));
fprintf('Duration: %.3f s\n', timeS(end));
fprintf('Median sample rate: %.3f Hz\n', sampleRateHz);
fprintf('Sequence losses: %.0f\n\n', lostFrames);

%% Stationary sensor statistics

restCount = size(restWindowsS, 1);
accelMean = zeros(restCount, 3);
accelStd = zeros(restCount, 3);
gyroMean = zeros(restCount, 3);
gyroStd = zeros(restCount, 3);
attitudeMeanDeg = zeros(restCount, 2);

for index = 1:restCount
    mask = timeS >= restWindowsS(index, 1) & ...
           timeS <= restWindowsS(index, 2);
    requireSamples(mask, restNames{index});

    acceleration = [data.accel_x(mask), data.accel_y(mask), data.accel_z(mask)];
    gyroscope = [data.imu_gyro_x(mask), data.imu_gyro_y(mask), ...
                 data.imu_gyro_z(mask)];

    accelMean(index, :) = mean(acceleration, 1);
    accelStd(index, :) = std(acceleration, 0, 1);
    gyroMean(index, :) = mean(gyroscope, 1);
    gyroStd(index, :) = std(gyroscope, 0, 1);
    attitudeMeanDeg(index, :) = [mean(data.roll_angle(mask)), ...
                                 mean(data.pitch_angle(mask))];
end

restStatistics = table( ...
    restNames, ...
    accelMean(:, 1), accelMean(:, 2), accelMean(:, 3), ...
    accelStd(:, 1), accelStd(:, 2), accelStd(:, 3), ...
    gyroStd(:, 1), gyroStd(:, 2), gyroStd(:, 3), ...
    attitudeMeanDeg(:, 1), attitudeMeanDeg(:, 2), ...
    'VariableNames', { ...
        'Interval', ...
        'AccelMeanX', 'AccelMeanY', 'AccelMeanZ', ...
        'AccelStdX', 'AccelStdY', 'AccelStdZ', ...
        'GyroStdX', 'GyroStdY', 'GyroStdZ', ...
        'RollMeanDeg', 'PitchMeanDeg'});

disp('Stationary statistics:');
disp(restStatistics);

%% Integrate each known table movement

eventCount = size(eventWindowsS, 1);
baselineDisplacementM = zeros(eventCount, 2);
attitudeDisplacementM = zeros(eventCount, 2);
gyroDisplacementM = zeros(eventCount, 2);
baselineResidualVelocityMps = zeros(eventCount, 2);
attitudeResidualVelocityMps = zeros(eventCount, 2);
gyroResidualVelocityMps = zeros(eventCount, 2);
eventResult = repmat(struct(), eventCount, 1);

for index = 1:eventCount
    eventStartS = eventWindowsS(index, 1);
    eventEndS = eventWindowsS(index, 2);

    eventMask = timeS >= eventStartS & timeS <= eventEndS;
    baselineMask = ...
        timeS >= (eventStartS - baselineDurationS) & ...
        timeS < (eventStartS - baselineGapS);

    requireSamples(eventMask, eventNames{index});
    requireSamples(baselineMask, [eventNames{index}, ' baseline']);

    eventTimeS = timeS(eventMask);
    eventTimeS = eventTimeS - eventTimeS(1);

    measuredAcceleration = [data.accel_x(eventMask), data.accel_y(eventMask)];
    baselineAcceleration = [mean(data.accel_x(baselineMask)), ...
                            mean(data.accel_y(baselineMask))];
    accelerationFromBaseline = measuredAcceleration - baselineAcceleration;

    % Method 1: subtract only the local stationary accelerometer baseline.
    [baselineVelocity, baselinePosition] = integrateAcceleration( ...
        eventTimeS, accelerationFromBaseline);
    [baselineVelocityZupt, baselinePositionZupt] = ...
        applyEndpointZeroVelocity(eventTimeS, baselineVelocity);

    % Method 2: remove the gravity change predicted by the firmware attitude.
    rollRad = data.roll_angle(eventMask) * pi / 180.0;
    pitchRad = data.pitch_angle(eventMask) * pi / 180.0;
    initialRollRad = mean(data.roll_angle(baselineMask)) * pi / 180.0;
    initialPitchRad = mean(data.pitch_angle(baselineMask)) * pi / 180.0;

    accelerationFromAttitude = accelerationFromBaseline;
    accelerationFromAttitude(:, 1) = accelerationFromAttitude(:, 1) - ...
        gravityMps2 * (sin(pitchRad) - sin(initialPitchRad));
    accelerationFromAttitude(:, 2) = accelerationFromAttitude(:, 2) - ...
        gravityMps2 * (sin(rollRad) - sin(initialRollRad));

    [attitudeVelocity, attitudePosition] = integrateAcceleration( ...
        eventTimeS, accelerationFromAttitude);
    [attitudeVelocityZupt, attitudePositionZupt] = ...
        applyEndpointZeroVelocity(eventTimeS, attitudeVelocity);

    % Method 3: propagate short-term roll, pitch, and yaw from the gyroscope.
    gyroscope = [data.imu_gyro_x(eventMask), ...
                 data.imu_gyro_y(eventMask), ...
                 data.imu_gyro_z(eventMask)];
    baselineGyroscope = [mean(data.imu_gyro_x(baselineMask)), ...
                         mean(data.imu_gyro_y(baselineMask)), ...
                         mean(data.imu_gyro_z(baselineMask))];
    gyroscope = gyroscope - baselineGyroscope;

    deltaRollRad = cumtrapz(eventTimeS, gyroscope(:, 1));
    deltaPitchRad = cumtrapz(eventTimeS, -gyroscope(:, 2));
    deltaYawRad = cumtrapz(eventTimeS, gyroscope(:, 3));

    accelerationFromGyro = accelerationFromBaseline;
    accelerationFromGyro(:, 1) = accelerationFromGyro(:, 1) - ...
        gravityMps2 * sin(deltaPitchRad);
    accelerationFromGyro(:, 2) = accelerationFromGyro(:, 2) - ...
        gravityMps2 * sin(deltaRollRad);

    % Rotate the body horizontal acceleration into an event-local frame.
    cosYaw = cos(deltaYawRad);
    sinYaw = sin(deltaYawRad);
    navigationAcceleration = [ ...
        cosYaw .* accelerationFromGyro(:, 1) - ...
        sinYaw .* accelerationFromGyro(:, 2), ...
        sinYaw .* accelerationFromGyro(:, 1) + ...
        cosYaw .* accelerationFromGyro(:, 2)];

    [gyroVelocity, gyroPosition] = integrateAcceleration( ...
        eventTimeS, navigationAcceleration);
    [gyroVelocityZupt, gyroPositionZupt] = ...
        applyEndpointZeroVelocity(eventTimeS, gyroVelocity);

    baselineDisplacementM(index, :) = baselinePositionZupt(end, :);
    attitudeDisplacementM(index, :) = attitudePositionZupt(end, :);
    gyroDisplacementM(index, :) = gyroPositionZupt(end, :);
    baselineResidualVelocityMps(index, :) = baselineVelocity(end, :);
    attitudeResidualVelocityMps(index, :) = attitudeVelocity(end, :);
    gyroResidualVelocityMps(index, :) = gyroVelocity(end, :);

    eventResult(index).timeS = eventTimeS;
    eventResult(index).expectedPositionM = expectedDisplacementM(index, :);
    eventResult(index).baselinePositionM = baselinePositionZupt;
    eventResult(index).attitudePositionM = attitudePositionZupt;
    eventResult(index).gyroPositionM = gyroPositionZupt;
    eventResult(index).baselineVelocityMps = baselineVelocityZupt;
    eventResult(index).attitudeVelocityMps = attitudeVelocityZupt;
    eventResult(index).gyroVelocityMps = gyroVelocityZupt;
end

expectedDistanceM = sqrt(sum(expectedDisplacementM.^2, 2));
baselineDistanceM = sqrt(sum(baselineDisplacementM.^2, 2));
attitudeDistanceM = sqrt(sum(attitudeDisplacementM.^2, 2));
gyroDistanceM = sqrt(sum(gyroDisplacementM.^2, 2));

eventSummary = table( ...
    eventNames, ...
    expectedDisplacementM(:, 1), expectedDisplacementM(:, 2), ...
    baselineDisplacementM(:, 1), baselineDisplacementM(:, 2), ...
    baselineDistanceM, ...
    100.0 * (baselineDistanceM - expectedDistanceM) ./ expectedDistanceM, ...
    attitudeDistanceM, gyroDistanceM, ...
    sqrt(sum(baselineResidualVelocityMps.^2, 2)), ...
    'VariableNames', { ...
        'Movement', 'ExpectedX', 'ExpectedY', ...
        'EstimatedX', 'EstimatedY', 'EstimatedDistance', ...
        'DistanceErrorPercent', 'AttitudeMethodDistance', ...
        'GyroMethodDistance', 'RawEndpointSpeedError'});

disp('Table-movement displacement estimates:');
disp(eventSummary);

%% Demonstrate unconstrained full-record drift

initialStationaryMask = timeS >= 0.0 & timeS <= 5.0;
initialAccelX = mean(data.accel_x(initialStationaryMask));
initialAccelY = mean(data.accel_y(initialStationaryMask));
initialRollRad = mean(data.roll_angle(initialStationaryMask)) * pi / 180.0;
initialPitchRad = mean(data.pitch_angle(initialStationaryMask)) * pi / 180.0;

fullBaselineAcceleration = [data.accel_x - initialAccelX, ...
                            data.accel_y - initialAccelY];

fullAttitudeAcceleration = fullBaselineAcceleration;
fullRollRad = data.roll_angle * pi / 180.0;
fullPitchRad = data.pitch_angle * pi / 180.0;
fullAttitudeAcceleration(:, 1) = fullAttitudeAcceleration(:, 1) - ...
    gravityMps2 * (sin(fullPitchRad) - sin(initialPitchRad));
fullAttitudeAcceleration(:, 2) = fullAttitudeAcceleration(:, 2) - ...
    gravityMps2 * (sin(fullRollRad) - sin(initialRollRad));

[fullBaselineVelocity, fullBaselinePosition] = integrateAcceleration( ...
    timeS, fullBaselineAcceleration);
[fullAttitudeVelocity, fullAttitudePosition] = integrateAcceleration( ...
    timeS, fullAttitudeAcceleration);

fprintf('\nUnconstrained integration over the complete %.2f s recording:\n', ...
    timeS(end));
fprintf('Baseline-only final speed: %.3f m/s, position error: %.3f m\n', ...
    norm(fullBaselineVelocity(end, :)), norm(fullBaselinePosition(end, :)));
fprintf('Firmware-attitude final speed: %.3f m/s, position error: %.3f m\n', ...
    norm(fullAttitudeVelocity(end, :)), norm(fullAttitudePosition(end, :)));
fprintf(['These full-record values demonstrate IMU drift. Endpoint ZUPT ', ...
         'must not be applied in flight unless an external measurement ', ...
         'confirms zero horizontal velocity.\n\n']);

fprintf('Vertical acceleration extrema: %.3f to %.3f m/s^2\n', ...
    min(data.accel_z), max(data.accel_z));
fprintf('Maximum gyroscope magnitude: %.3f rad/s\n', max(sqrt( ...
    data.imu_gyro_x.^2 + data.imu_gyro_y.^2 + data.imu_gyro_z.^2)));

%% Overview plots

figure('Color', 'w', 'Name', 'MCU Cuadcopter IMU overview');

subplot(4, 1, 1);
plot(timeS, data.accel_x, 'LineWidth', 0.9);
hold on;
plot(timeS, data.accel_y, 'LineWidth', 0.9);
plot(timeS, data.accel_z, 'LineWidth', 0.9);
addEventBoundaries(eventWindowsS);
grid on;
ylabel('m/s^2');
title('Physical IMU acceleration');
legend('X', 'Y', 'Z', 'Location', 'eastoutside');

subplot(4, 1, 2);
plot(timeS, data.imu_gyro_x, 'LineWidth', 0.9);
hold on;
plot(timeS, data.imu_gyro_y, 'LineWidth', 0.9);
plot(timeS, data.imu_gyro_z, 'LineWidth', 0.9);
addEventBoundaries(eventWindowsS);
grid on;
ylabel('rad/s');
title('Physical IMU angular rate');
legend('X', 'Y', 'Z', 'Location', 'eastoutside');

subplot(4, 1, 3);
plot(timeS, data.roll_angle, 'LineWidth', 1.0);
hold on;
plot(timeS, data.pitch_angle, 'LineWidth', 1.0);
addEventBoundaries(eventWindowsS);
grid on;
ylabel('deg');
title('Firmware attitude estimate');
legend('Roll', 'Pitch', 'Location', 'eastoutside');

subplot(4, 1, 4);
plot(timeS, fullBaselinePosition(:, 1), 'LineWidth', 1.0);
hold on;
plot(timeS, fullBaselinePosition(:, 2), 'LineWidth', 1.0);
plot(timeS, fullAttitudePosition(:, 1), '--', 'LineWidth', 1.0);
plot(timeS, fullAttitudePosition(:, 2), '--', 'LineWidth', 1.0);
addEventBoundaries(eventWindowsS);
grid on;
xlabel('MCU time (s)');
ylabel('m');
title('Unconstrained double-integration drift');
legend('Baseline X', 'Baseline Y', 'Attitude X', 'Attitude Y', ...
       'Location', 'eastoutside');

%% Per-event displacement plots

figure('Color', 'w', 'Name', 'Table movement reconstruction');
for index = 1:eventCount
    subplot(2, 2, index);
    plot(eventResult(index).baselinePositionM(:, 1), ...
         eventResult(index).baselinePositionM(:, 2), ...
         'LineWidth', 1.4);
    hold on;
    plot(eventResult(index).attitudePositionM(:, 1), ...
         eventResult(index).attitudePositionM(:, 2), '--', ...
         'LineWidth', 1.2);
    plot(eventResult(index).gyroPositionM(:, 1), ...
         eventResult(index).gyroPositionM(:, 2), ':', ...
         'LineWidth', 1.5);
    plot(expectedDisplacementM(index, 1), ...
         expectedDisplacementM(index, 2), 'kx', ...
         'MarkerSize', 10, 'LineWidth', 2.0);
    plot(0.0, 0.0, 'ko', 'MarkerFaceColor', 'k');
    axis equal;
    grid on;
    xlabel('X (m)');
    ylabel('Y (m)');
    title(eventNames{index});
    legend('Local baseline', 'Firmware attitude', 'Gyro propagated', ...
           'Expected endpoint', 'Start', 'Location', 'best');
end

figure('Color', 'w', 'Name', 'Displacement magnitude comparison');
bar([expectedDistanceM, baselineDistanceM, attitudeDistanceM, gyroDistanceM]);
grid on;
ylabel('Displacement magnitude (m)');
title('Known 0.50 m movements versus IMU estimates');
set(gca, 'XTick', 1:eventCount, 'XTickLabel', eventNames);
legend('Expected', 'Local baseline + endpoint ZUPT', ...
       'Firmware attitude + endpoint ZUPT', ...
       'Gyro propagated + endpoint ZUPT', ...
       'Location', 'best');

results = struct();
results.file = string(csvFile);
results.sampleRateHz = sampleRateHz;
results.lostFrames = lostFrames;
results.stationaryStatistics = restStatistics;
results.eventSummary = eventSummary;
results.fullRecord.baselineFinalVelocityMps = fullBaselineVelocity(end, :);
results.fullRecord.baselineFinalPositionM = fullBaselinePosition(end, :);
results.fullRecord.attitudeFinalVelocityMps = fullAttitudeVelocity(end, :);
results.fullRecord.attitudeFinalPositionM = fullAttitudePosition(end, :);

end

%% Local functions

function requireSamples(mask, intervalName)
    if nnz(mask) < 2
        error('Interval "%s" does not contain enough samples.', intervalName);
    end
end

function [velocity, position] = integrateAcceleration(timeS, acceleration)
    velocity = cumtrapz(timeS, acceleration);
    position = cumtrapz(timeS, velocity);
end

function [correctedVelocity, correctedPosition] = ...
    applyEndpointZeroVelocity(timeS, rawVelocity)
    durationS = timeS(end) - timeS(1);
    if durationS <= 0.0
        error('The integration interval must have a positive duration.');
    end

    normalizedTime = (timeS - timeS(1)) / durationS;

    % Enforce v(T) = 0 with the smallest linear endpoint correction:
    %   v_corrected(t) = v_raw(t) - (t / T) * v_raw(T)
    correction = bsxfun(@times, normalizedTime, rawVelocity(end, :));
    correctedVelocity = rawVelocity - correction;
    correctedPosition = cumtrapz(timeS, correctedVelocity);
end

function addEventBoundaries(eventWindowsS)
    limits = ylim;
    for index = 1:size(eventWindowsS, 1)
        line([eventWindowsS(index, 1), eventWindowsS(index, 1)], limits, ...
             'Color', [0.5, 0.5, 0.5], 'LineStyle', ':', ...
             'HandleVisibility', 'off');
        line([eventWindowsS(index, 2), eventWindowsS(index, 2)], limits, ...
             'Color', [0.5, 0.5, 0.5], 'LineStyle', ':', ...
             'HandleVisibility', 'off');
    end
    ylim(limits);
end
