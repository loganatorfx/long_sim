% Load raceline
clear all 
close all
clc

data = readmatrix('raceline_cleaned.csv');  % columns: distance (m), speed (m/s)
distance_raceline = data(:, 1);
v_target = data(:, 2);

% Parameters
m = 1500;              % vehicle mass in kg
dt = 1.0;             % time step in seconds
max_force = 4000;      % max driving/braking force in N
threshold = 0.5;       % bang-bang control threshold (m/s)

% Initialize state
x = 0;                 % position (m)
v = 0;                 % velocity (m/s)
t = 0;                 % time (s)

% Storage
x_log = [];
v_log = [];
t_log = [];
v_target_log = [];

% Simulation loop
i = 1;
while i <= length(distance_raceline)

    if t > 200  % safety timeout
    warning('Simulation exceeded 200 seconds. Stopping.');
    break;
    end

    % Get target velocity from raceline
    v_des = v_target(i);
    
    % Control (bang-bang)
    error = v_des - v;
    if error > threshold
        F = max_force;
    elseif error < -threshold
        F = -max_force;
    else
        F = 0;
    end
    
    % Dynamics
    a = F / m;
    v = v + a * dt;
    x = x + v * dt;
    t = t + dt;
    
    % Log data
    x_log(end+1) = x;
    v_log(end+1) = v;
    t_log(end+1) = t;
    v_target_log(end+1) = v_des;
    
    % Advance raceline index based on position
    while i < length(distance_raceline) && x > distance_raceline(i)
        i = i + 1;
    end
end

% Plot results
figure;
subplot(2,1,1);
plot(t_log, v_log, 'b', t_log, v_target_log, 'r--');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('Actual', 'Target');
title('Velocity Tracking');

subplot(2,1,2);
plot(t_log, x_log);
xlabel('Time (s)');
ylabel('Position (m)');
title('Distance over Time');
