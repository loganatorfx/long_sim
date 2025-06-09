clear; clc; close all;

%% Define raceline (synthetic for now)
raceline_distance = linspace(0, 4000, 1000);           % meters
raceline_speed = 30 + 10 * sin(2 * pi * raceline_distance / 2000);  % m/s

%% Parameters
dx = 2;  % meters per sim step
track_length = max(raceline_distance);
N = ceil(track_length / dx);

% Vehicle parameters
mass = 800; wheel_radius = 0.35;
final_drive = 3.0;
gear_ratios = [2.917, 1.875, 1.381, 1.115, 0.960, 0.889];
Cd = 0.858; A = 1.0; rho = 1.225; Cr = 0.015; eta = 0.9;

% Torque map
rpm_map = [1000 2000 3000 4000 5000 6000 7000];
torque_map = [400 500 600 700 700 650 600];

upshift_rpm = 6000; downshift_rpm = 2500;

%Braking:
max_brake_force = 8000;  % adjust this as needed

%% Preallocate
s = zeros(1, N);
v = zeros(1, N);
a = zeros(1, N);
rpm = zeros(1, N);
gear = ones(1, N);
T_engine = zeros(1, N);
v_target = zeros(1, N);

%% Initial condition
s(1) = raceline_distance(1);
v(1) = interp1(raceline_distance, raceline_speed, s(1), 'linear', 'extrap');

%% Simulation loop
for i = 1:N-1
    if s(i) >= track_length, break; end

    current_gear = gear(i);
    gear_ratio = gear_ratios(current_gear);
    
    % RPM
    rpm(i) = (v(i) * 60 / (2 * pi * wheel_radius)) * gear_ratio * final_drive;
    rpm(i) = min(max(rpm(i), 1000), 7000);
    
    % Shift logic        fprintf("Braking!")
    if rpm(i) > upshift_rpm && current_gear < length(gear_ratios)
        current_gear = current_gear + 1;
    elseif rpm(i) < downshift_rpm && current_gear > 1
        current_gear = current_gear - 1;
    end
    gear(i+1) = current_gear;
    gear_ratio = gear_ratios(current_gear);  % update

    % Target speed from raceline
    v_target(i) = interp1(raceline_distance, raceline_speed, s(i), 'linear', 'extrap');
    v_err = v_target(i) - v(i);

    % Simple control
    if v_err > 0.5
        throttle = 1.0; brake = 0;
    elseif v_err < -0.5
        throttle = 0.0; brake = 1.0;
    else
        throttle = 0.3; brake = 0.0;
    end

    % Torque and forces
    T_interp = interp1(rpm_map, torque_map, rpm(i), 'linear', 'extrap');
    T_engine(i) = throttle * T_interp;
    T_wheel = T_engine(i) * gear_ratio * final_drive * eta;
    F_trac = T_wheel / wheel_radius;
    F_drag = 0.5 * rho * Cd * A * v(i)^2;
    F_roll = Cr * mass * 9.81;
    F_brake = brake * max_brake_force;

    F_net = F_trac - F_drag - F_roll;

    % Integration
    a(i) = F_net / mass;
    v(i+1) = max(v(i) + a(i) * dx / max(v(i), 0.1), 0);  % avoid div-by-zero
    s(i+1) = s(i) + dx;
end

%% Plot
figure;
plot(s, v, 'b');
xlabel('Distance [m]');
ylabel('Speed [m/s]');
title('Vehicle Speed vs Distance');
grid on;

figure;
% plot(s, v, 'b', 'LineWidth', 1.5); hold on;

% Add raceline target speeds directly
plot(raceline_distance, raceline_speed, 'r--', 'LineWidth', 1.5);

xlabel('Distance [m]');
ylabel('Speed [m/s]');
title('Vehicle Speed vs Distance');
legend('Actual Speed', 'Target Speed (Raceline)');
grid on;
