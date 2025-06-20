load('torque_lookup_data.mat')  % Loads T, X, Y

Tq = reshape(T, [length(X), length(Y)]);  % Ensure it's 76×8

figure
surf(Y, X, Tq)  % X: engine speed, Y: throttle/load
xlabel('Throttle Position (breakpoints_y)')
ylabel('Engine Speed (breakpoints_x)')
zlabel('Torque (Nm)')
title('Engine Torque Map')
grid on
shading interp  % Optional for smoothing
