close all;
clear all;

 %% Course hold
% Natural frequency and damping ratio
zeta_course = 5; % DESIGN PARAMETER
wn_course = 14.2267/30; % DESIGN PARAMETER

% Calculate control gains
course_kp = 2*zeta_course * wn_course * 10 / 9.81;
course_ki = wn_course^2 * 10 / 9.81;
course_kd = 0;

sim course_loop;

figure(1);
plot(course);
grid on;