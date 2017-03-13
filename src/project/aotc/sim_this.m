close all;
clear all;

param = 'aerosonde';
init;
bus_defintions;

%X_trim = [0.0000    0.0000 -150.0000    0.9997    0.0000    0.0232   -0.0000 18.0168    0.0001    0.8366    0.0000    0.0000    0.0000 0.0039 -0.0000    0.0039   -0.0000    0.1240]';
%U_trim = [0.0079    0.0000         0    0.1240]';
%Vad = 18;

X_trim =  [-0.0000  -0.0000 -150.0000 0.9994 -0.0009 0.0332 0.0131 35.7334 -0.0122 2.3795...
            0.0000 0.0000 0.0000 -0.0300 -0.0000 -0.0295 0.0000 0.0978]';
U_trim = [-0.0594 0.0005 -0.0010 0.0978]';
Vad = 35;

sim_time = 100;
set_param('andre_boller', 'StopTime', int2str(sim_time));
sim andre_boller;

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

% figure(3);
% hold on;
% grid on;
% plot(states.Velocity.u);
% plot(states.Velocity.v);
% plot(states.Velocity.w);
% plot(airdata.Va);
% plot(Vad);
% title('Velocity');
% legend('u','v','w','Va','Va_d');

figure(4);
plot(control);
title('CONTROL INPUTS');
legend('e','a','r','t');
grid on;

% figure(5);
% plot(desired_heading);
% hold on;
% plot(states.Attitude.psi);
% title('HEADING');
% grid on;

% figure(6);
% plot(auto);
% title('AUTOPILOT OUTPUTS');
% legend('e','a','r','t');
% grid on;

% figure(7);
% plot(states.Position.p_E.Data, states.Position.p_N.Data);
% hold on;
% grid on;
% 
% waypoints = [[0,0,0,0]; [2500,0,0,0]; [3000,3000,1,1]; [4000,5000,1,1]];
% [N,~] = size(waypoints);
% 
% for i = 1:N-1
%     plot([waypoints(i,2),waypoints(i+1,2)],[waypoints(i,1),waypoints(i+1,1)],'k');
% end