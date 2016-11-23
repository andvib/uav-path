close all;
clear all;

%% Initialize
param = 'aerosonde';
init;
bus_defintions;

X_trim = [0; 0; -100.0; 0.9993; -0.0011; 0.0332; 0.0186; 35.7388;...
          -0.0122; 2.3784; 0; 0; 0; -0.0300; 0; -0.0294; 0; 0.0978];
U_trim = [-0.0594; 0.0005; -0.0010; 0.0978];


%% RUN SIMULATORE
% Create track
heading_time = [0 100];
heading_d = [0 0];
Vad = 35;

% Set inital conditions
X_trim(4:7) = euler2q(0,0,0);
X_trim(8) = Vad;
set_param('autopilot_fly', 'StopTime', int2str(100));
x0 = X_trim;

[phi0,theta0,psi0] = q2euler(X_trim(4:7)/norm(X_trim(4:7)));

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
plot(states.Velocity.u);
plot(states.Velocity.v);
plot(states.Velocity.w);
plot(airdata.Va);
plot(va_desired);
title('Velocity');
legend('u','v','w','Va','Va_d');

figure(4);
hold on;
grid on;
plot(heading_desired);
plot(states.Attitude.psi);
legend('Desired', 'Actual');
title('Heading - actual vs. desired');

figure(5);
hold on;
grid on;
plot(states.Position.p_E.Data, states.Position.p_N.Data);
title('Position - NE');

figure(6);
plot(control);
title('CONTROL INPUTS');
legend('e','a','r','t');
grid on;