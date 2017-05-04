close all;
clear all;

run results/results.m;


%% Calculate camera position

for i = (1:length(STATES(:,1)))
    [x_temp, y_temp] = camera_pos([STATES(i,8), STATES(i,9), STATES(i,10)],...
                                  [STATES(i,2), STATES(i,3), STATES(i,4)],...
                                  0.331612558);
    c_n_1(:,i) = x_temp;
    c_n_2(:,i) = y_temp;
end
    


%% ENTIRE MODEL
figure(1);
grid on;
hold on;
plot(STATES(:,3), STATES(:,2));
plot(c_n_1(2,:), c_n_1(1,:), 'k');
plot(c_n_2(2,:), c_n_2(1,:), 'k');
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