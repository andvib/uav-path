close all;
clear all;

run results/output.m;

figure(1);
grid on;
hold on;
plot(STATES(:,2), STATES(:,3));
xlabel("X");
ylabel("Y");
title("POSITION");

figure(2);
grid on;
hold on;
plot(CONTROLS(:,1), CONTROLS(:,2));
plot(CONTROLS(:,1), CONTROLS(:,3));
title("CONTROLS");
legend("DX", "DY");