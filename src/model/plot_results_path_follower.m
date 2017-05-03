close all;
clear all;

run results/output.m;

%% LATERAL
% figure(1);
% grid on;
% hold on;
% plot(STATES(:,1),STATES(:,2));
% title("VELOCITY v");
% 
% figure(2);
% grid on;
% hold on;
% plot(STATES(:,1),STATES(:,3:4));
% title("ANGLE RATES");
% legend("p", "r");
% 
% figure(3);
% grid on;
% hold on;
% plot(STATES(:,1),STATES(:,5:6));
% title("ATTITUDE");
% legend("\phi", "\psi");
% 
% figure(4);
% grid on;
% hold on;
% plot(STATES(:,1),STATES(:,7:8));
% title("CONTROLS");
% legend("aileron", "rudder");
% 
% figure(5);
% grid on;
% hold on;
% plot(CONTROLS(:,1),CONTROLS(:,2:3));
% title("CONTROL RATES");
% legend("aileron", "rudder");

%% LONGITUDINAL
% figure(1);
% grid on;
% hold on;
% plot(STATES(:,1),STATES(:,2:3));
% title("VELOCITIES");
% legend("u", "w");
% 
% figure(2);
% grid on;
% hold on;
% plot(STATES(:,1),STATES(:,4));
% title("ANGLE RATE q");
% 
% figure(3);
% grid on;
% hold on;
% plot(STATES(:,1),STATES(:,5));
% title("ATTITUDE \theta");
% 
% figure(4);
% grid on;
% hold on;
% plot(STATES(:,1),STATES(:,6));
% title("HEIGHT");
% 
% figure(5);
% grid on;
% hold on;
% plot(STATES(:,1),STATES(:,7:8));
% title("CONTROLS");
% legend("ELEVATOR", "THROTTLE");


%% ENTIRE MODEL
figure(1);
grid on;
hold on;
plot(STATES(:,3), STATES(:,2));
title("POSITION");

figure(2);
grid on;
hold on;
plot(STATES(:,1),STATES(:,4));
title("HEIGHT");

figure(3);
grid on;
hold on;
plot(STATES(:,1),STATES(:,5:7));
title("VELOCITIES");
legend("u", "v", "w");

figure(4);
grid on;
hold on;
plot(STATES(:,1), STATES(:,8:10));
title("ATTITUDE");
legend("\phi", "\theta", "\psi");

figure(5);
grid on;
hold on;
plot(STATES(:,1), STATES(:,11:13));
title("ANGLE RATES");
legend("p", "q", "r");

figure(6);
grid on;
hold on;
plot(STATES(:,1), STATES(:,14:17));
title("CONTROL SURFACES");
legend("\delta_e", "\delta_a", "\delta_r", "\delta_t");

%figure(7);
%grid on;
%hold on;
%plot(CONTROLS(:,1),CONTROLS(:,2:5));
%title("CONTROL");
%legend("d\delta_e", "d\delta_a", "d\delta_r", "d\delta_t");