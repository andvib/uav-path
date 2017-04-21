close all;
clear all;

run results/results.m;

figure(1);
grid on;
hold on;
plot(STATES(:,1), STATES(:,2));
xlabel("X");
ylabel("Y");
title("POSITION");

figure(2);
grid on;
hold on;
plot(CONTROLS(:,1));
plot(CONTROLS(:,2));
title("CONTROLS");
legend("DX", "DY");