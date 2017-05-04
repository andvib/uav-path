close all;
clear all;

run results/results.m;


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

%figure(5);
%grid on;
%hold on;
%plot(STATES(:,1), STATES(:,11:13));
%title("ANGLE RATES");
%legend("p", "q", "r");

figure(6);
grid on;
hold on;
plot(STATES(:,1), STATES(:,14:17));
title("CONTROL SURFACES");
legend("\delta_e", "\delta_a", "\delta_r", "\delta_t");

%figure(7);
%grid on;
%hold on;
%plot(STATES(:,1), CONTROLS(:,1:4));
%title("CONTROL");
%legend("d\delta_e", "d\delta_a", "d\delta_r", "d\delta_t");
%ylim([-10 10]);