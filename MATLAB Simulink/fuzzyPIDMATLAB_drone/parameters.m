clc, clear, close all;

% Constants
g = 9.81; 
m = 1.5;
k = 2.980e-6; % Thrust factor of rotor (depends on density
              % geometry, etc)
           % to the centre of gravity           
l = 0.225; % Linear distance from the centre of the rotor
b = 1.140e-7; % Drag constant
Im = 3.357e-5; % Inertia moment of the rotor

% Inertia 
I_xx = 4.856e-3; % kg*m^2
I_yy = 4.856e-3; % kg*m^2
I_zz = 8.801e-3; % kg*m^2

% Initial configuration
roll_i = -0.1;   % rad
pitch_i = -0.8;  % rad
yaw_i = -4;      % rad
altitude_i = 0;  % meters

% Desired configuration
roll_d = -0.4;   % rad
pitch_d = 0;     % rad
yaw_d = -pi;     % rad
altitude_d = 6;  % meters

sim("quadrotor_model") % Initialize Simulink

% 1. Load hệ thống fuzzy từ file .fis
fis = readfis('fuzzyAttitudez.fis');

% 2. Tạo biến vào Workspace
assignin('base', 'fis', fis);  % để dùng trong Simulink

% 3. Các thông số min/max PID để scale lại
kp_min = 0;  kp_max = 60;
ki_min = 0;  ki_max = 1.0;
kd_min = 0;  kd_max = 20;

assignin('base', 'kp_min', kp_min);
assignin('base', 'kp_max', kp_max);
assignin('base', 'ki_min', ki_min);
assignin('base', 'ki_max', ki_max);
assignin('base', 'kd_min', kd_min);
assignin('base', 'kd_max', kd_max);



% Plot the movements
figure('Name', 'Positions', 'NumberTitle','off')
plot3(x, y, z, 'LineWidth', 2)
xlabel("Position x")
ylabel("Position y")
zlabel("Position z")
title("Positions")
grid on

% Roll angle
figure('Name', 'Roll angle', 'NumberTitle','off')
plot(tout, roll, 'b', 'LineWidth', 2); hold on
plot(tout, ones(size(tout))*roll_d, '--r', 'LineWidth', 2);
xlabel('Time (s)', 'interpreter', 'latex')
ylabel('Roll angle (rad)', 'interpreter', 'latex')
l = legend('$\phi$ Current roll angle', '$\phi_d$ Desired roll angle');
set(l, 'interpreter', 'latex')

% Pitch angle
figure('Name', 'Pitch angle', 'NumberTitle','off')
plot(tout, pitch, 'b', 'LineWidth', 2); hold on
plot(tout, ones(size(tout))*pitch_d, '--r', 'LineWidth', 2);
xlabel('Time (s)', 'interpreter', 'latex')
ylabel('Pitch angle (rad)', 'interpreter', 'latex')
l = legend('$\theta$ Current pitch angle', '$\theta_d$ Desired pitch angle');
set(l, 'interpreter', 'latex')

% Yaw angle
figure('Name', 'Yaw angle', 'NumberTitle','off')
plot(tout, yaw, 'b', 'LineWidth', 2); hold on
plot(tout, ones(size(tout))*yaw_d, '--r', 'LineWidth', 2);
xlabel('Time (s)', 'interpreter', 'latex')
ylabel('Yaw angle (rad)', 'interpreter', 'latex')
l = legend('$\psi$ Current yaw angle', '$\psi_d$ Desired yaw angle');
set(l, 'interpreter', 'latex')
