close all;
clear all;

%% Sideslip hold
% Get transfer function values
a_beta2 = -2.9051;
a_beta1 = 0.4785;

% Max control surface deflection
delta_r_max = 20*pi/180;
vr_max = 3; % Roll command when delta_r_max is achieved

% Natural frequency and damping ratio
zeta_beta = 0.75; % DESIGN PARAMETER

% Calculate control gains
beta_kp = -delta_r_max / vr_max;
wn_beta = (a_beta2 * beta_kp + a_beta1) / 2 / zeta_beta;
beta_ki = wn_beta^2 / a_beta2;
beta_kd = 0;

sim sideslip_tune;

figure(1);
plot(beta);
grid on;