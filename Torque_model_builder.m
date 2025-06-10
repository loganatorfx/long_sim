% build_torque_interpolant.m
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

% Apply smoothing along throttle and RPM dimensions
for g = 1:length(gears)
    for t = 1:length(throttles)
        TORQUE(g,t,:) = smooth(squeeze(TORQUE(g,t,:)), 3);
    end
    for r = 1:length(rpms)
        TORQUE(g,:,r) = smooth(squeeze(TORQUE(g,:,r)), 3);
    end
end

F = griddedInterpolant(GEAR, THROTTLE, RPM, TORQUE, 'linear', 'nearest');

fprintf("did we get here?")

save('torque_interpolant_smoothed.mat', 'F');
