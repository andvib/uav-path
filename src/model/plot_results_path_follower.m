close all;
clear all;

run results/output.m;

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
plot(STATES(:,1), STATES(:,11:14));
title("CONTROL SURFACES");
legend("\delta_a", "\delta_e", "\delta_r", "\delta_t");

figure(6);
grid on;
hold on;
plot(CONTROLS(:,1),CONTROLS(:,2:5));
title("CONTROL");
legend("d\delta_e", "d\delta_a", "d\delta_r", "d\delta_t");