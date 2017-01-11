close all;
clear all;

%% Pitch loop
% Get transfer function values
a_theta1 = 0.6984;
a_theta2 = 27.1682;
a_theta3 = -35.7476;

% Max control surface deflection
delta_e_max = 35*pi/180;
theta_max = 10*pi/180;

% Natural frequ|ency and damping ratio
zeta_pitch = 0.95; % DESIGN PARAMETER
wn_pitch = sqrt(abs(a_theta3)*delta_e_max*sqrt(1-zeta_pitch^2)/theta_max);

% Calculate control gains
pitch_kp = (wn_pitch^2 - a_theta2) / a_theta3;
pitch_kd = (2 * zeta_pitch * wn_pitch - a_theta1) / a_theta3;
pitch_ki = 0;
K_theta_DC = pitch_kp * a_theta3 / (a_theta2 + pitch_kp * a_theta3)

sim pitch_loop;

figure(1);
plot(pitch);
grid on;