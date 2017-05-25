close all;
clear all;

iterations = (10:10:140);
%iterations = 130;
%% PLOT POSITION %%

figure(1);
grid on;
hold on;
for i = iterations
    a = sprintf('results_horizon/horizon_lin_%d.m', i);
    run (a)
    
    plot(STATES(:,3), STATES(:,2));
end
xlim([-300 500]);
ylim([0 800]);
ylabel('North [m]');
xlabel('East [m]');
%matlab2tikz('fig/uav_position.tex');
saveas(gcf, 'fig/uav_position', 'epsc');

%% PLOT CAMERA CENTRE POINT%%

figure(2);
grid on;
hold on;
for i = iterations
    a = sprintf('results_horizon/horizon_lin_%d.m', i);
    run (a)
    
    for i = (1:length(STATES(:,1)))
        [x_temp, y_temp] = camera_pos([STATES(i,8), STATES(i,9), STATES(i,10)],...
                                  [STATES(i,2), STATES(i,3), STATES(i,4)], 0.0);%0.331612);
        c_n_1(:,i) = x_temp;
        c_n_2(:,i) = y_temp;
    end

    plot(c_n_1(2,:),c_n_1(1,:));
    %plot(c_n_2(2,:),c_n_2(1,:));
    %plot(STATES(:,3), STATES(:,2));
end

plot(PATH(:,2), PATH(:,1),'k');
xlim([-300 500]);
ylim([0 800]);
ylabel('North [m]');
xlabel('East [m]');
%matlab2tikz('fig/camera_position.tex');
saveas(gcf, 'fig/camera_position', 'epsc');


%% PLOT HEIGHT %%

figure(3);
grid on;
hold on;
for i = iterations
    a = sprintf('results_horizon/horizon_lin_%d.m', i);
    run (a)
    
    plot(STATES(:,1), STATES(:,4));
end
xlim([0 40]);
ylim([100 160]);
ylabel('Height [m]');
xlabel('Time [s]');
%matlab2tikz('fig/height.tex');
saveas(gcf, 'fig/height', 'epsc');

%% PLOT ATTITUDE %%

figure(4);
grid on;
hold on;
k = 1;
for i = iterations
    a = sprintf('results_horizon/horizon_lin_%d.m', i);
    run (a)
    
    phi(k,:) = STATES(:,8)';
    theta(k,:) = STATES(:,9)';
    psi(k,:) = STATES(:,10)';
    k = k+1;
end
subplot(3,1,1);
grid on;
hold on;
plot(STATES(:,1), phi);
ylabel('\phi [rad]');
ylim([-1 1]);

subplot(3,1,2);
grid on;
hold on;
plot(STATES(:,1), theta);
ylabel('\theta [rad]');
ylim([-1 1]);

subplot(3,1,3);
grid on;
hold on;
plot(STATES(:,1), psi);
ylim([-1 1]);
ylabel('\psi [rad]');
xlabel('Time [s]');
%matlab2tikz('fig/attitude.tex');
saveas(gcf, 'fig/attitude','epsc');

%% PLOT CONTROLS %%

figure(5);
grid on;
hold on;
title('CONTROl');
k = 1;
for i = iterations
    a = sprintf('results_horizon/horizon_lin_%d.m', i);
    run (a)
    
    elevator(k,:) = STATES(:,14)';
    aileron(k,:) = STATES(:,15)';
    rudder(k,:) = STATES(:,16)';
    throttle(k,:) = STATES(:,17)';
    k = k+1;
end
subplot(4,1,1);
plot(STATES(:,1), elevator);
subplot(4,1,2);
plot(STATES(:,1), aileron);
subplot(4,1,3);
plot(STATES(:,1), rudder);
subplot(4,1,4);
plot(STATES(:,1), throttle);

%% PLOT DURATION %%

figure(6);
grid on;
hold on;
k = 1;
for i = 10:10:140
    a = sprintf('results_horizon/horizon_lin_%d.m', i);
    run (a)
    
    dur(k,1) = i;
    dur(k,2) = DURATION;
    k = k+1;
end
plot(dur(:,1), dur(:,2))
%matlab2tikz('fig/duration.tex');
saveas(gcf, 'fig/duration', 'epsc');
