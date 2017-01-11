close all;
clear all;

a_phi2 = 128.0716;
a_phi1 = 16.2447;

% Max control surface deflection
delta_a_max = 45*pi/180;
phi_max = 15*pi/180;

% Natural frequency and damping ratio
zeta_roll = 0.95; % DESIGN PARAMETER
wn_roll = sqrt(a_phi2*delta_a_max*sqrt(1-zeta_roll^2)/phi_max);

% Calculate control gains
roll_kp = wn_roll^2/a_phi2;
roll_kd = (2*zeta_roll*wn_roll - a_phi1)/a_phi2;
roll_ki = 0.2; % DESIGN PARAMETER

sim roll_loop;

figure(1);
plot(roll);
grid on;