close all;
clear all;

Va = 10;

K_theta_DC = 0.4591;
wn_pitch = 7.1036;

%% Airspeed hold using pitch
% Get transfer function values
a_V1 = 0.4039;

% Natural frequency and damping ratio
zeta_airspeed_pitch = 0.707; % DESIGN PARAMETER
wn_airspeed_pitch = wn_pitch / 10; % DESIGN PARAMETER

% Calculate control gains
v_kp = (a_V1-2*zeta_airspeed_pitch*wn_airspeed_pitch)/(K_theta_DC*0.81);
v_ki = -wn_airspeed_pitch^2 / (K_theta_DC * 9.81);

sim airspeed_pitch_loop;

figure(1);
plot(speed);
grid on;