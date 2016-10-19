%%%%%%%%%%%%%%%%%%%%%%%%%
% AEROSONDE UAV - MODEL %
%%%%%%%%%%%%%%%%%%%%%%%%%

%% Physical parameters
m = 13.5;       % [kg]
J_x = 0.8244;   % [kg-m^2]
J_y = 1.135;    % [kg-m^2]
J_z = 1.759;    % [kg-m^2]
J_xz = 0.1204;  % [kg-m^2]
S = 0.55;       % [m^2]
b = 2.8956;     % [m]
c = 0.18994;    % [m]
S_prop = 0.2027;% [m^2]
ro = 1.2682;    % [kg/m^3]
k_motor = 80;
k_tp = 0;
k_omega = 0;
e = 0.9;

%% Longitudinal coeffisients
C_L0 = 0.28;
C_D0 = 0.03;
C_m0 = -0.02338;
C_L_alpha = 3.45;
C_D_alpha = 0.30;
C_m_alpha = -0.38;
C_Lq = 0;
C_Dq = 0;
C_mq = -3.6;
C_L_delta_e = -0.36;
C_D_delta_e = 0;
C_m_delta_e = -0.5;
C_prop = 1.0;
M = 50;
alpha_0 = 0.4712;
epsilon = 0.1592;
C_Dp = 0.0437;
C_n_delta_r = -0.032;

%% Lateral Coeffisients
C_Y0 = 0;
C_l0 = 0;
C_n0 = 0;
C_Y_beta = -0.98;
C_l_beta = -0.12;
C_n_beta = 0.25;
C_Yp = 0;
C_lp = -0.26;
C_np = 0.022;
C_Yr = 0;
C_Y_delta_a = 0;
C_l_delta_a = 0.08;
C_n_delta_a = 0.06;
C_Y_delta_r = -0.17;
C_l_delta_r = 0.105;