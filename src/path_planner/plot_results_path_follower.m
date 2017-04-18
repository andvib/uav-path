close all;
clear all;

run results/output.m;

figure(1);
grid on;
hold on;
plot(STATES(:,2), STATES(:,3));
title("POSITION");

figure(2);
grid on;
hold on;
plot(CONTROLS(:,2));
plot(CONTROLS(:,3));
title("CONTROLS");
legend("DX", "DY");