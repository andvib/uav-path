%Run X8 simulations
close all; clear all; clc;

%%Choose what set of parameters to load: 'aerosonde', 'zagi', 'X8_param', or 'symbolic_param'
% param = 'aerosonde';
param = 'X8_param'
init;
bus_defintions;
%P.k_T_P = 0;
%% Choose control input source: trim(1), joystick(2) or autopilot(3)
input_source0 = 1;
input_step_time = 2;
input_source = 3;

%% environment
wind.speed = 6;%m/s @ 6m
wind.dir = 90;%degrees clockwise from N
wind.start_time = 2000;

%% Calculate trim conditions
% see p91 inStevens & Lewis
gamma = 0.0*pi/180;
phi = 0;%-10*pi/180;
R = Inf;%500;
h_ini = 150;
plot_bode = false;
%[A,B,C,D,Alat,Blat,Alon,Blon,X_trim,U_trim,Y_trim,DX,zpk_tf] = trimX8(plot_bode,P.Va,gamma,R,h_ini,phi);
U_trim = [0.0079    0.0000         0    0.1240];
%euler x0 = [-0.0000    0.0000 -100.0000    0.0000    0.2303    0.0000   17.5248    0.0000    4.1085   -0.0000    0.0000   -0.0000]';
x0 = [-0.0000    0.0000 -100.0000   0.9997    0.0000    0.0232   -0.0000   18.0168   0.0001    0.8366    0.0000    0.0000    0.0000    0.0039   -0.0000    0.0039  -0.0000    0.1240]';
X_trim = x0;



%% Run simulation
% sim X8