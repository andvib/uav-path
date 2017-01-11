close all;
clear all;

Va = 10;

K_theta_DC = 0.4591;
wn_pitch = 7.1036;

%% Airspeed hold using throttle
% Get transfer function values
a_V1 = 0.4039;
a_V2 = 6.4361;

% Natural frequency and damping ratio
zeta_airspeed_throttle = 0.6; % DESIGN PARAMETER
wn_airspeed_throttle = 1; % DESIGN PARAMETER

% Calculate control gains
v_kp = (2 * zeta_airspeed_throttle * wn_airspeed_throttle - a_V1) / a_V2; % Feil i Gryte-kode?
v_ki = wn_airspeed_throttle^2 / a_V2;
%P.airspeed_throttle_integrator_gain = a_V1 / (a_V2 * P.airspeed_throttle_ki);

sim airspeed_throttle_loop;

figure(1);
plot(speed);
grid on;