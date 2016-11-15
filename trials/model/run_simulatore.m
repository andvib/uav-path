close all;
clear all;

%% Initialize
param = 'aerosonde';
init;
bus_defintions;

%% TRIM
% U0 = [-0.064; 0.1; 0; 0.1]; %Avoid negative throttle by guessing thr=1
% IU0 = [];
% 
Va = 35;
gamma = 0*pi / 180;
R = Inf;
h_ini = 100;
% 
% DX0 = [0; 0; -Va*sin(gamma); 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
% IDX = [3; 4; 5; 6; 7; 8; 9; 10; 11; 12; 13; 14; 15; 16; 17; 18];
% 
% X0 = [0; 0; -h_ini; 0; 0; 0; 0; Va; 0; 0; 0; 0; 0;...
%                         (U0(1)-U0(2))/2; 0; (U0(1)+U0(2))/2; 0; U0(4)];
% IX0 = [];
% 
% Y0 = [Va; gamma; 0; X0(1:13); 0; 0; 0];
% IY0 = [1,3];
% 
% options(14) = 10000;
% [X_trim, U_trim, Y_trim, DX_trim] = trim('simulatore', X0, U0, Y0, ...
%                                          IX0, IU0, IY0, DX0, IDX, options)

%u0              = [-0.064 0.1 0 0.1]'; %Avoid negative throttle by guessing thr=1

u0 = [0; 0; 0; 1];
fixed_inputs    = [];

theta0=0;
psi0=0;
quat0 = euler2q(0,theta0,psi0);
%                   N E    D           q1       q2       q3         q4          u   v  w   p   q   r  servoRpos       servoRrate servoLpos        servoLrate   motor
%                   1 2    3            4       5         6         7           8   9  10  11  12  13  14               15        16                17          18
x0              =  [0 0 -h_ini         quat0(1) quat0(2) quat0(3)   quat0(4)    Va  0  0   0   0   0  (u0(1)-u0(2))/2      0     (u0(1)+u0(2))/2       0       u0(4)]';
dx0             =  [0 0 -Va*sin(gamma) 0        0           0       0           0   0  0   0   0   0    0                  0       0                   0       0]';
if R~=Inf,dx0(6)= Va/R; end %psidot = Va/R
fixed_states    =  [];%14:17 18];
fixed_deriv     =  [       3           4     5   6  7  8  9  10   11  12 13 14:17 18];

%                  Va alpha beta
y0              = [Va gamma 0   x0(1:13)' 0,0,0]';
fixed_outputs   = [1,3];%, 4:19];

% Calculate trim conditions
options(14) = 10000;
[X_trim,U_trim,Y_trim,DX,options]=trim('simulatore',x0,u0,y0, ...
                               fixed_states,fixed_inputs,fixed_outputs, ...
                               dx0,fixed_deriv,options);
X_trim(4:7) = quatnormalize(X_trim(4:7)')';
norm(DX(7:13)-dx0(7:13)) % Should be very low

[T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r]= compute_tf_model(X_trim,U_trim,P);
P=computeGainsAerosonde(T_phi_delta_a,T_v_delta_r,T_theta_delta_e,T_Va_theta,T_Va_delta_t,P);

%% RUN SIMULATORE
set_param('straight_level', 'StopTime', int2str(100));
x0 = X_trim;
P
sim straight_level
P
figure(1);
hold on;
grid on;
title('Position');
plot(states.Position.p_N);
plot(states.Position.p_E);
plot(states.Position.p_D);
legend('N', 'E', 'D');

figure(2);
hold on;
grid on;
title('Attitude');
plot(states.Attitude.phi);
plot(states.Attitude.theta);
plot(states.Attitude.psi);
legend('\phi','\theta','\psi');

figure(3);
hold on;
grid on;
title('Velocity');
plot(states.Velocity.u);
plot(states.Velocity.v);
plot(states.Velocity.w);
plot(airdata.Va);
legend('u','v','w','Va');