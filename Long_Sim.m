% Longitudinal Vehicle Simulation (Step 2: Static Torque Map)
clear all
close all
clc
fprintf('Starting new simulation run...\n');


% Import Raceline
raceline = readtable('laguna_seca_cleaned.csv');
raceline_distance = raceline.s_m;
raceline_speed = raceline.vx_mps;
raceline_acceleration = raceline.ax_mps2;
raceline_kappa_radpm = raceline .kappa_radpm;

%% 

% Spatial stepping parameters
dx = 1.0;  % meters per simulation step
track_length = max(raceline_distance);
N = ceil(track_length / dx) + 1;

% Vehicle parameters
mass = 800;                      % kg
wheel_radius = 0.35;            % meters
gear_ratios = [2.917, 1.875, 1.381, 1.115, 0.960, 0.889];
Cd = 0.858;                     % drag coefficient
A = 1.0;                        % frontal area [m^2]
rho = 1.225;                    % air density [kg/m^3]
Cr = 0.015;                     % rolling resistance coefficient
efficiency = 0.9;               % drivetrain efficiency
final_drive = 3.0;              % final drive ratio

% Gear-specific shift RPM thresholds (updated to match gearset)
downshift_rpm_thresh = [0,    2700, 3800, 4000, 4300, 4700]; % gear 1 to 6 (lower bounds)
upshift_rpm_thresh   = [5000, 6200, 6200, 6200, 6200, 6950]; % gear 1 to 6 (upper bounds)
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
a_cmd = zeros(1, N);
a_lat = zeros(1, N);  % Preallocate lateral acceleration


% Initial conditions
s(1) = 0;
v(1) = interp1(raceline_distance, raceline_speed, s(1), 'linear', 'extrap');
dt = 0.01;

for i = 1:N-1

    %Check For End Of Sim
    if s(i) >= track_length
        fprintf('Reached end of track at i=%d\n', i);
        break;
    end

    %Update Gear and Gear Ratio
    current_gear = gear(i);
    gear_ratio = gear_ratios(current_gear);

    %Update RPM
    rpm(i) = compute_rpm(v(i), gear_ratio, final_drive, wheel_radius);

    %Velocity Tracking Control 
    [v_target(i), a_cmd(i), throttle, brake] = velocity_control( ...
    s(i), v(i), raceline_distance, raceline_speed, raceline_acceleration, dt);

    % Polynomial model-based torque prediction (data-driven)
    T_engine(i) = torque_model(rpm(i), throttle, current_gear);

    %Compute Forces
    [F_net, ~] = compute_forces(T_engine(i), gear_ratio, final_drive, efficiency, ...
                                wheel_radius, v(i), rho, Cd, A, Cr, mass, brake);
    %Compute Acceleration
    a(i) = F_net / mass;

    %Find Timestep(sim is dist based)
    v_current = max(v(i), 1.0);
    dt = dx / v_current;
    dt = dt;

    %Update Velocit and Displacement and tim e
    v(i+1) = max(v(i) + a(i) * dt, 0.5);
    s(i+1) = s(i) + dx;
    time(i+1) = time(i) + dt;

    next_rpm = (v(i+1) * 60 / (2 * pi * wheel_radius)) * gear_ratio * final_drive;

    % Compute lateral acceleration
    kappa_i = interp1(raceline_distance, raceline_kappa_radpm, s(i), 'linear', 'extrap');
    a_lat(i) = v_target(i)^2 * kappa_i;
    
    % Gear update with lateral acceleration-based gear hold
    lat_thresh = 5.0;  % [m/s²], tune as needed
    if abs(a_lat(i)) > lat_thresh
        gear(i+1) = gear(i);  % hold current gear in high lateral load
    else
        gear(i+1) = update_gear(current_gear, next_rpm, throttle, ...
                                upshift_rpm_thresh, downshift_rpm_thresh, max_gear);
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
a_lat = a_lat(1:sim_length);

% Create uniform time vector for plotting (e.g., every 0.1 s)
time_uniform = 0:0.1:time(end);

% Resample key variables using interpolation
v_plot        = interp1(time, v, time_uniform, 'linear', 'extrap');
v_target_plot = interp1(time, v_target, time_uniform, 'linear', 'extrap');
rpm_plot      = interp1(time, rpm, time_uniform, 'linear', 'extrap');
T_engine_plot = interp1(time, T_engine, time_uniform, 'linear', 'extrap');
gear_plot     = interp1(time, gear, time_uniform, 'previous', 'extrap');  % piecewise constant
a_lat_plot = interp1(time, a_lat, time_uniform, 'linear', 'extrap');

% Lateral acceleration plot in its own figure
figure(2); clf;
plot(time_uniform, a_lat_plot, 'b-');
hold on;
yline(5, 'r--', 'Threshold +5');
yline(-5, 'r--', 'Threshold -5');
xlabel('Time [s]');
ylabel('Lateral Acceleration [m/s^2]');
title('Lateral Acceleration Over Time');
grid on;

% Main performance subplots in figure 1
figure(1); clf;
subplot(4,1,1);
plot(time_uniform, v_plot, 'b-', time_uniform, v_target_plot, 'r--');
ylabel('Speed [m/s]'); legend('Simulated', 'Target'); grid on;

subplot(4,1,2);
plot(time_uniform, rpm_plot, 'g-'); ylabel('RPM'); grid on;

subplot(4,1,3);
plot(time_uniform, T_engine_plot, 'm-');
ylabel('Torque [Nm]');
ylim([0 1500]);
grid on;

subplot(4,1,4);
stairs(time_uniform, gear_plot, 'k-'); ylabel('Gear'); xlabel('Time [s]'); grid on;

%%Function Models%%

%Torque Calc
function T = torque_model(rpm, throttle, gear)
    persistent F loaded
    if isempty(loaded)
        data = load('torque_interpolant_smoothed.mat');
        F = data.F;
        loaded = true;
    end

    % Clamp inputs to valid range
    rpm = min(max(rpm, 1000), 7000);
    throttle = min(max(throttle * 100, 0), 100); % convert to percent
    gear = round(min(max(gear, 1), 6));

    T = F(gear, throttle, rpm);

    if throttle == 0
        T = 20;
    end

    % Clamp torque to non-negative values
    T = max(T, 0);

    %Two WWheels Making Torque
    T = 2* T;
end

%RPM Calc
function rpm_val = compute_rpm(v, gear_ratio, final_drive, wheel_radius)
    if v > 0.5
        rpm_val = (v * 60 / (2 * pi * wheel_radius)) * gear_ratio * final_drive;
        rpm_val = max(min(rpm_val, 7000), 1000);
    else
        rpm_val = 1000;
    end
end

%Gear Shift Logic
function next_gear = update_gear(current_gear, next_rpm, throttle, ...
                                  upshift_rpm_thresh, downshift_rpm_thresh, max_gear)
    skip_upshift = throttle < 0.3;
    if current_gear < max_gear && next_rpm > upshift_rpm_thresh(current_gear) && ~skip_upshift
        next_gear = current_gear + 1;
    elseif current_gear > 1 && next_rpm < downshift_rpm_thresh(current_gear)
        next_gear = current_gear - 1;
    else
        next_gear = current_gear;
    end
end

%Force and Acceleration Based Control 

%Force Calcs
function [F_net, F_trac] = compute_forces(T_engine, gear_ratio, final_drive, efficiency, ...
                                          wheel_radius, v, rho, Cd, A, Cr, mass, brake)

    T_wheel = T_engine * gear_ratio * final_drive * efficiency;
    F_trac = T_wheel / wheel_radius;
    F_drag = 0.5 * rho * Cd * A * v^2;
    F_roll = Cr * mass * 9.81;
    F_brake = brake * 5000;

    F_net = F_trac - F_drag - F_roll - F_brake;
end

% Acceleration-Based Control
function [v_target_i, a_cmd, throttle, brake] = velocity_control(s_i, v_i, raceline_distance, raceline_speed, raceline_acceleration, dt)
    v_target_i = interp1(raceline_distance, raceline_speed, s_i, 'linear', 'extrap');
    a_path_i   = interp1(raceline_distance, raceline_acceleration, s_i, 'linear', 'extrap');
    
    if isnan(v_target_i) || v_target_i <= 0
        v_target_i = 20;
    end
    if isnan(a_path_i)
        a_path_i = 0;
    end

    % Feedforward + Asymmetric Feedback
    kp_throttle = 0.3;
    kp_brake = 0.8;
    k_ff = 0.5;  % feedforward scaling

    v_err = v_target_i - v_i;

    if v_err >= 0
        a_fb = kp_throttle * v_err;
    else
        a_fb = kp_brake * v_err;
    end

    a_cmd = k_ff * a_path_i + a_fb;

    % Clamp command acceleration
    a_cmd = max(min(a_cmd, 4.0), -5.0);

    % Acceleration to throttle/brake
    if a_cmd >= 0
        throttle = min(max(0.5 + a_cmd, 0), 1);
        brake = 0;
    else
        throttle = 0;
        brake = min(max(-a_cmd, 0), 1);
    end
end
