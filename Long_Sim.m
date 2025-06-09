% Longitudinal Vehicle Simulation (Step 2: Static Torque Map)
clear all
close all
clc
fprintf('Starting new simulation run...\n');

% Import Raceline
raceline = readtable('raceline_cleaned.csv');
raceline_distance = raceline.distance_m;
raceline_speed = raceline.speed_mps;

% Spatial stepping parameters
dx = 1.0;  % meters per simulation step
track_length = max(raceline_distance);
N = ceil(track_length / dx) + 1;

% Vehicle parameters
mass = 800;                      % kg
wheel_radius = 0.35;            % meters
final_drive = 3.0;
gear_ratios = [2.917, 1.875, 1.381, 1.115, 0.960, 0.889];
Cd = 0.858;                     % drag coefficient
A = 1.0;                        % frontal area [m^2]
rho = 1.225;                    % air density [kg/m^3]
Cr = 0.015;                     % rolling resistance coefficient
efficiency = 0.9;               % drivetrain efficiency

% Gear-specific shift RPM thresholds
downshift_rpm_thresh = [0,     2000, 2815, 3375, 3785, 3980]; % gear 1 to 6 (lower bounds)
upshift_rpm_thresh   = [3900,  4500, 4800, 5000, 4800, 600];   % gear 1 to 6 (upper bounds)

% Static engine torque map (at full throttle)
rpm_map = [1000 2000 3000 4000 5000 6000 7000];
torque_map = [400 500 600 700 700 650 600];
idle_torque = 20;

% Shifting thresholds
upshift_rpm = 6000;
downshift_rpm = 2500;
max_gear = length(gear_ratios);

% Preallocate states
time = zeros(1, N);
v = zeros(1, N);
s = zeros(1, N);
a = zeros(1, N);
rpm = zeros(1, N);
gear = ones(1, N);  % start in 1st gear
T_engine = zeros(1, N);
v_target = zeros(1, N);

% Initial conditions
s(1) = 0;
v(1) = interp1(raceline_distance, raceline_speed, s(1), 'linear', 'extrap');
if isnan(v(1)) || v(1) <= 0
    v(1) = 10;
end

for i = 1:N-1
    if s(i) >= track_length
        fprintf('Reached end of track at i=%d\n', i);
        break;
    end

    current_gear = gear(i);
    gear_ratio = gear_ratios(current_gear);

    if v(i) > 0.5
        rpm(i) = (v(i) * 60 / (2 * pi * wheel_radius)) * gear_ratio * final_drive;
        rpm(i) = max(min(rpm(i), 7000), 1000);
    else
        rpm(i) = 1000;
    end

    v_target(i) = interp1(raceline_distance, raceline_speed, s(i), 'linear', 'extrap');
    if isnan(v_target(i)) || v_target(i) <= 0
        v_target(i) = 20;
    end

    v_err = v_target(i) - v(i);
    throttle = min(max(0.5 + 0.03 * v_err, 0), 1);
    brake = max(-0.1 * v_err, 0);

    T_interp = interp1(rpm_map, torque_map, rpm(i), 'linear', 'extrap');
    T_engine(i) = throttle * T_interp * (throttle > 0) + idle_torque * (throttle == 0);

    T_wheel = T_engine(i) * gear_ratio * final_drive * efficiency;
    F_trac = T_wheel / wheel_radius;
    F_drag = 0.5 * rho * Cd * A * v(i)^2;
    F_roll = Cr * mass * 9.81;
    F_brake = brake * 5000;

    F_net = F_trac - F_drag - F_roll - F_brake;
    a(i) = F_net / mass;

    v_current = max(v(i), 1.0);
    dt = dx / v_current;
    dt = max(min(dt, 2.0), 0.01);

    v(i+1) = max(v(i) + a(i) * dt, 0.5);
    s(i+1) = s(i) + dx;
    time(i+1) = time(i) + dt;

    next_rpm = (v(i+1) * 60 / (2 * pi * wheel_radius)) * gear_ratio * final_drive;

    if current_gear < max_gear && next_rpm > upshift_rpm_thresh(current_gear)
        gear(i+1) = current_gear + 1;
    elseif current_gear > 1 && next_rpm < downshift_rpm_thresh(current_gear)
        gear(i+1) = current_gear - 1;
    else
        gear(i+1) = current_gear;
    end


    if mod(i, 500) == 0 || v(i+1) < 1.0
        fprintf('i=%d: s=%.1fm, v=%.1fm/s, gear=%d, rpm=%.0f, dt=%.3fs\n', ...
            i, s(i), v(i), current_gear, rpm(i), dt);
    end

    if time(i+1) > 300
        fprintf('Simulation timeout at %.1f seconds\n', time(i+1));
        break;
    end
end

dt_check = diff(time);
non_increasing = find(dt_check <= 0);
fprintf('Non-increasing time steps: %d\n', length(non_increasing));


sim_length = i;
fprintf('Lap completed in %.2f seconds over %.1f meters\n', time(sim_length), s(sim_length));

% Truncate vectors
time = time(1:sim_length);
v = v(1:sim_length);
s = s(1:sim_length);
a = a(1:sim_length);
rpm = rpm(1:sim_length);
T_engine = T_engine(1:sim_length);
gear = gear(1:sim_length);
v_target = interp1(raceline_distance, raceline_speed, s, 'linear', 'extrap');

% Create uniform time vector for plotting (e.g., every 0.1 s)
time_uniform = 0:0.1:time(end);

% Resample key variables using interpolation
v_plot        = interp1(time, v, time_uniform, 'linear', 'extrap');
v_target_plot = interp1(time, v_target, time_uniform, 'linear', 'extrap');
rpm_plot      = interp1(time, rpm, time_uniform, 'linear', 'extrap');
T_engine_plot = interp1(time, T_engine, time_uniform, 'linear', 'extrap');
gear_plot     = interp1(time, gear, time_uniform, 'previous', 'extrap');  % piecewise constant


% Replace time_clean with time_uniform and interpolated data
figure(1); clf;
subplot(4,1,1);
plot(time_uniform, v_plot, 'b-', time_uniform, v_target_plot, 'r--');
ylabel('Speed [m/s]'); legend('Simulated', 'Target'); grid on;

subplot(4,1,2);
plot(time_uniform, rpm_plot, 'g-'); ylabel('RPM'); grid on;

subplot(4,1,3);
plot(time_uniform, T_engine_plot, 'm-'); ylabel('Torque [Nm]'); grid on;

subplot(4,1,4);
stairs(time_uniform, gear_plot, 'k-'); ylabel('Gear'); xlabel('Time [s]'); grid on;

