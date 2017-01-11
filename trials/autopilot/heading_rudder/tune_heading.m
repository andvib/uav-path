close all;
clear all;

% Transfer function values
a_phi1 = 9.8641;
a_phi2 = -34.2801;

% Natural frequency and damping ratio
zeta_heading_rudder = 0.6;
wn_heading_rudder = 3.5;

% Calculate control gains
heading_rudder_kp = wn_heading_rudder^2 / a_phi2;
heading_rudder_kd = (2*zeta_heading_rudder^2*wn_heading_rudder - a_phi1) / a_phi2;

sim heading_tune;

figure(1);
plot(course);
grid on;

sys = tf([a_phi2*heading_rudder_kp], [1 (a_phi1 + a_phi2*heading_rudder_kd) a_phi2*heading_rudder_kp]);
figure(2);
bode(sys,'k');
grid on;
hold on;
margin(sys);