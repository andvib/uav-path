close all;
clear all;

Va = 10;

K_theta_DC = 0.4591;
wn_pitch = 7.1036;

%% Altitude hold using pitch 
% Natural frequency and damping ratio
zeta_altitude = 0.7; % DESIGN PARAMETER
wn_altitude = wn_pitch / 20; % DESIGN PARAMETER

% Calculate control gains
altitude_kp = 2 * zeta_altitude * wn_altitude / (K_theta_DC * Va);
altitude_ki = wn_altitude^2 / (K_theta_DC * Va);
altitude_kd = 0;

sim altitude_pitch_loop;

figure(1);
plot(height);
grid on;