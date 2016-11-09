close all;
clear all;

%% Initialize
param = 'aerosonde';
init;
bus_defintions;

X_trim = [0.0000; -0.0000; -100.0000; -0.2574; 0.0055; -0.0015; -0.9663;...
          28.2017; 0.0002; 0.3236; -0.0000; -0.0000; -0.0000; 0.0095;...
          -0.0000; 0.0095; -0.0000; 0.5301];
      
U_trim = [0.0190; 0.0000; 0; 0.5301];

[T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r]= compute_tf_model(X_trim,U_trim,P);
P=computeGains(T_phi_delta_a,T_v_delta_r,T_theta_delta_e,T_Va_theta,T_Va_delta_t,P);

%% RUN SIMULATORE
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
legend('u','v','w','Va');

figure(5);
plot(delta);
legend('e','a','r','t');
figure(6);
plot(control);