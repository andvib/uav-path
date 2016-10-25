%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% AEROSONDE UAV - PARAMETERS %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Physical parameters
P.m = 13.5;       % [kg]
P.J_x = 0.8244;   % [kg-m^2]
P.J_y = 1.135;    % [kg-m^2]
P.J_z = 1.759;    % [kg-m^2]
P.J_xz = 0.1204;  % [kg-m^2]
P.S = 0.55;       % [m^2]
P.b = 2.8956;     % [m]
P.c = 0.18994;    % [m]
P.S_prop = 0.2027;% [m^2]
P.rho = 1.2682;   % [kg/m^3]
P.k_motor = 80;
P.k_tp = 0;
P.k_omega = 0;
P.e = 0.9;

%% Longitudinal coeffisients
P.C_L0 = 0.28;
P.C_D0 = 0.03;
P.C_m0 = -0.02338;
P.C_L_alpha = 3.45;
P.C_D_alpha = 0.30;
P.C_m_alpha = -0.38;
P.C_Lq = 0;
P.C_Dq = 0;
P.C_mq = -3.6;
P.C_L_delta_e = -0.36;
P.C_D_delta_e = 0;
P.C_m_delta_e = -0.5;
P.C_prop = 1.0;
P.M = 50;
P.alpha_0 = 0.4712;
P.epsilon = 0.1592;
P.C_Dp = 0.0437;
P.C_n_delta_r = -0.032;

%% Lateral Coeffisients
P.C_Y0 = 0;
P.C_l0 = 0;
P.C_n0 = 0;
P.C_Y_beta = -0.98;
P.C_l_beta = -0.12;
P.C_n_beta = 0.25;
P.C_Yp = 0;
P.C_lp = -0.26;
P.C_np = 0.022;
P.C_Yr = 0;
P.C_lr = 0.14;
P.C_nr = -0.35;
P.C_Y_delta_a = 0;
P.C_l_delta_a = 0.08;
P.C_n_delta_a = 0.06;
P.C_Y_delta_r = -0.17;
P.C_l_delta_r = 0.105;