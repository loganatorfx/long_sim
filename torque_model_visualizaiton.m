% visualize_torque_model.m
clear all
close all
clc

% Load model from CSV and build interpolant
T = readtable('Binned_Torque_Map.csv');

gears = unique(T.gear_bin);
throttles = unique(T.throttle_bin);
rpms = unique(T.rpm_bin);

[GEAR, THROTTLE, RPM] = ndgrid(gears, throttles, rpms);
TORQUE = nan(size(GEAR));

for i = 1:height(T)
    g_idx = find(gears == T.gear_bin(i));
    t_idx = find(throttles == T.throttle_bin(i));
    r_idx = find(rpms == T.rpm_bin(i));
    TORQUE(g_idx, t_idx, r_idx) = T.torque(i);
end

TORQUE = fillmissing(TORQUE, 'nearest');
F = griddedInterpolant(GEAR, THROTTLE, RPM, TORQUE, 'linear', 'nearest');

% Grid for plotting
[throttle_grid, rpm_grid] = meshgrid(0:5:100, 1000:250:7000);

% Plot for each gear
figure;
for g = gears'
    torque_grid = zeros(size(throttle_grid));
    for i = 1:numel(torque_grid)
        t = throttle_grid(i);
        r = rpm_grid(i);
        t = min(max(t, throttles(1)), throttles(end));
        r = min(max(r, rpms(1)), rpms(end));
        torque_grid(i) = F(g, t, r);
    end

    torque_grid = reshape(torque_grid, size(throttle_grid));

    % Apply 2D smoothing
    torque_grid = conv2(torque_grid, ones(3)/3, 'same');

    subplot(2,2,g)
    surf(rpm_grid, throttle_grid, torque_grid);
    xlabel('RPM'); ylabel('Throttle (%)'); zlabel('Torque [Nm]');
    title(sprintf('Smoothed Torque Surface - Gear %d', g));
    shading interp
end

% Print peak torque per gear (based on raw input data)
fprintf('\nPeak torque per gear (raw data):\n');
for i = 1:length(gears)
    g = gears(i);
    max_torque = max(T.torque(T.gear_bin == g));
    fprintf('  Gear %d: %.1f Nm\n', g, max_torque);
end
