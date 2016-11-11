close all;
clear all;

%% Initialize
param = 'aerosonde';
init;
bus_defintions;

% X_trim = [0.0000; -0.0000; -100.0000; -0.2574; 0.0055; -0.0015; -0.9663;...
%           28.2017; 0.0002; 0.3236; -0.0000; -0.0000; -0.0000; 0.0095;...
%           -0.0000; 0.0095; -0.0000; 0.5301];
%       
% U_trim = [0.0190; 0.0000; 0; 0.5301];


X_trim = [0; 0; -100.0000; 0.9993; -0.0011; 0.0332; 0.0186; 35.7388;...
            -0.0122; 2.3784; 0; 0; 0; -0.0300; 0; -0.0294; 0; 0.0978];
U_trim = [-0.0594; 0.0005; -0.0010; 0.0978];

P.tau = 1;
[T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r]= compute_tf_model(X_trim,U_trim,P);
P=computeGainsAerosonde(T_phi_delta_a,T_v_delta_r,T_theta_delta_e,T_Va_theta,T_Va_delta_t,P);

%% RUN SIMULATORE
% Create track
heading_time = [0 35 50 75 100];
heading_d = [1 1 2 1 1];
Va = 25;
X_trim(8) = Va;

% Set inital conditions
X_trim(4:7) = euler2q(0,0,0);

set_param('autopilot_fly', 'StopTime', int2str(100));
x0 = X_trim;
sim autopilot_fly

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
plot([0:100], ones(101)*Va);
legend('u','v','w','Va','Va_d');

figure(4);
hold on;
grid on;
plot(heading_time, heading_d);
plot(states.Attitude.psi);
legend('Desired', 'Actual');
title('Heading - actual vs. desired');

figure(5);
hold on;
grid on;
plot(states.Position.p_E.Data, states.Position.p_N.Data);
title('Position - NE');

% figure(5);
% plot(delta);
% legend('e','a','r','t');
% figure(6);
% plot(delta1.signals.values);
% legend('e','a','r','t');

figure(7);
plot(control);
title('CONTROL INPUTS');
legend('e','a','r','t');

% attitude = [states.Attitude.phi.data, states.Attitude.theta.data, states.Attitude.psi.data]';
% pos = [states.Position.p_N, states.Position.p_E, states.Position.p_D]';
% 
% c_n = [2, length(attitude(1,:))];
% 
% for i = length(c_n(1,:))
%     c_n(:,i) = camera_pos(attitude(:,i), pos(:,i), 20);
% end