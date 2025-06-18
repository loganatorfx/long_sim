% Longitudinal Vehicle Simulation with Predictive Gear Shift
clear all
close all
clc
fprintf('Starting new simulation run...\n');

% Load Raceline
raceline = readtable('laguna_seca_cleaned.csv');
raceline_distance = raceline.s_m;
raceline_speed = raceline.vx_mps;
raceline_acceleration = raceline.ax_mps2;
raceline_kappa_radpm = raceline.kappa_radpm;

%% Parameters
% Vehicle
mass = 800; wheel_radius = 0.35;
gear_ratios = [2.917, 1.875, 1.381, 1.115, 0.960, 0.889];
Cd = 0.858; A = 1.0; rho = 1.225; Cr = 0.015; efficiency = 0.9;
final_drive = 3.0;

% Shift Parameters
lookahead_dist = 20;
shift_delay = 0.5;
gear_shift_timer = 0;
lat_thresh = 5.0;

% Simulation stepping
dx = 1.0;
track_length = max(raceline_distance);
N = ceil(track_length / dx) + 1;

% Preallocate
v = zeros(1,N); s = zeros(1,N); time = zeros(1,N);
a = zeros(1,N); rpm = zeros(1,N); gear = ones(1,N);
T_engine = zeros(1,N); v_target = zeros(1,N);
a_lat = zeros(1,N);

% Initial
s(1) = 0;
v(1) = interp1(raceline_distance, raceline_speed, s(1), 'linear', 'extrap');
dt = 0.01;
T_engine_prev = 0;
torque_rate_limit = 600;

%% Main Loop
for i = 1:N-1
    if s(i) >= track_length
        fprintf('Reached end of track at i=%d\n', i);
        break;
    end

    current_gear = gear(i);
    gear_ratio = gear_ratios(current_gear);
    rpm(i) = compute_rpm(v(i), gear_ratio, final_drive, wheel_radius);

    [v_target(i), a_cmd, throttle, brake] = velocity_control( ...
        s(i), v(i), raceline_distance, raceline_speed, raceline_acceleration, dt);

    T_engine(i) = torque_model(rpm(i), throttle, current_gear);
    T_engine(i) = min(T_engine_prev + torque_rate_limit * dt, T_engine(i));
    T_engine_prev = T_engine(i);

    [F_net, ~] = compute_forces(T_engine(i), gear_ratio, final_drive, efficiency, ...
        wheel_radius, v(i), rho, Cd, A, Cr, mass, brake);
    a(i) = F_net / mass;

    v_current = max(v(i), 1.0);
    dt = dx / v_current;

    v(i+1) = max(v(i) + a(i)*dt, 0.5);
    s(i+1) = s(i) + dx;
    time(i+1) = time(i) + dt;

    % Predictive Gear Shift
    s_lookahead = s(i) + lookahead_dist;
    v_lookahead = interp1(raceline_distance, raceline_speed, s_lookahead, 'linear', 'extrap');

    [gear(i+1), gear_shift_timer] = predictive_gear_shift( ...
        s_lookahead, current_gear, throttle, v_lookahead, ...
        raceline_distance, raceline_kappa_radpm, ...
        gear_ratios, final_drive, wheel_radius, length(gear_ratios), ...
        @torque_model, lat_thresh, gear_shift_timer, shift_delay);

    % if mod(i, 500) == 0
    %     fprintf('i=%d: s=%.1fm, v=%.1fm/s, Gear: %d \u2192 %d, RPM=%.0f\n', ...
    %         i, s(i), v(i), current_gear, gear(i+1), rpm(i));
    % end

    if time(i+1) > 300
        fprintf('Simulation timeout at %.1f seconds\n', time(i+1));
        break;
    end
end

%% Supporting Functions

function rpm_val = compute_rpm(v, gear_ratio, final_drive, wheel_radius)
    if v > 0.5
        rpm_val = (v * 60 / (2 * pi * wheel_radius)) * gear_ratio * final_drive;
        rpm_val = max(min(rpm_val, 7000), 1000);
    else
        rpm_val = 1000;
    end
end

function T = torque_model(rpm, throttle, gear)
    persistent F loaded
    if isempty(loaded)
        data = load('torque_interpolant_smoothed.mat');
        F = data.F;
        loaded = true;
    end
    rpm = min(max(rpm, 1000), 7000);
    throttle = min(max(throttle * 100, 0), 100);
    gear = round(min(max(gear, 1), 6));
    T = F(gear, throttle, rpm);
    if throttle == 0
        T = 20;
    end
    T = max(T, 0);
    T = 2 * T;  % two wheels
end

function [F_net, F_trac] = compute_forces(T_engine, gear_ratio, final_drive, efficiency, ...
    wheel_radius, v, rho, Cd, A, Cr, mass, brake)
    T_wheel = T_engine * gear_ratio * final_drive * efficiency;
    F_trac = T_wheel / wheel_radius;
    F_drag = 0.5 * rho * Cd * A * v^2;
    F_roll = Cr * mass * 9.81;
    F_brake = brake * 5000;
    F_net = F_trac - F_drag - F_roll - F_brake;
end

function [v_target_i, a_cmd, throttle, brake] = velocity_control(s_i, v_i, raceline_distance, raceline_speed, raceline_acceleration, dt)
    v_target_i = interp1(raceline_distance, raceline_speed, s_i, 'linear', 'extrap');
    a_path_i   = interp1(raceline_distance, raceline_acceleration, s_i, 'linear', 'extrap');
    if isnan(v_target_i) || v_target_i <= 0
        v_target_i = 20;
    end
    if isnan(a_path_i)
        a_path_i = 0;
    end
    kp_throttle = 0.3; kp_brake = 1.0;
    k_ff_throttle = 0.1; k_ff_brake = 0.2;
    v_err = v_target_i - v_i;
    a_fb = (v_err >= 0) * kp_throttle * v_err + (v_err < 0) * kp_brake * v_err;
    a_ff = (a_path_i >= 0) * k_ff_throttle * a_path_i + (a_path_i < 0) * k_ff_brake * a_path_i;
    a_cmd = a_fb + a_ff;
    throttle = min(max(a_cmd, 0), 1) * (a_cmd >= 0);
    brake = min(max(-a_cmd, 0), 1) * (a_cmd < 0);
end

function [next_gear, gear_shift_timer] = predictive_gear_shift( ...
    s_i, current_gear, throttle, v_lookahead, ...
    raceline_distance, raceline_kappa_radpm, ...
    gear_ratios, final_drive, wheel_radius, max_gear, ...
    torque_model, lat_thresh, gear_shift_timer, shift_delay)

    kappa_lookahead = interp1(raceline_distance, raceline_kappa_radpm, s_i, 'linear', 'extrap');
    a_lat_lookahead = v_lookahead^2 * kappa_lookahead;

    predicted_torques = zeros(1, max_gear);
    for g = 1:max_gear
        rpm_pred = compute_rpm(v_lookahead, gear_ratios(g), final_drive, wheel_radius);
        predicted_torques(g) = torque_model(rpm_pred, throttle, g);
    end
    [~, best_gear] = max(predicted_torques);

    if abs(a_lat_lookahead) > lat_thresh
        next_gear = current_gear;
    elseif best_gear ~= current_gear && gear_shift_timer <= 0
        next_gear = best_gear;
        gear_shift_timer = shift_delay;
    else
        next_gear = current_gear;
    end

    gear_shift_timer = max(gear_shift_timer - 1e-3, 0);  % optional, or pass dt
end
