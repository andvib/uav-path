%Run X8 simulations
% close all; clear all; clc;

bus_defintions;
param = 'x8_param';
init;

%% Choose control input source: trim(1), joystick(2) or autopilot(3)
input_source0 = 1;
input_step_time = 0;
input_source = 3;

end_time = 150;

% %%
%Initial position reference for NED to LLA transformation
lat0 = 37.6205;%37.6637;
%KSFO: 37°37.13' / W122°22.53'
lon0 = -122.3815;%-122.4778;
h_ref = 100;

%% Initial trim conditions
X_trim = [0.0000    0.0000 -150.0000    0.9997    0.0000    0.0232   -0.0000 18.0168    0.0001    0.8366    0.0000    0.0000    0.0000 0.0039 -0.0000    0.0039   -0.0000    0.1240]';
U_trim = [0.0079    0.0000         0    0.1240]';

%% Tune controller
P.tau = 1; %Time constant for the band-limited differentiator
P.u_trim = U_trim;
[T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r]= compute_tf_model(X_trim,U_trim,P)
P=computeGains(T_phi_delta_a,T_v_delta_r,T_theta_delta_e,T_Va_theta,T_Va_delta_t,P);

%% Run simulation
% sim X8_demo