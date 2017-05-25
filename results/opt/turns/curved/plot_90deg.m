close all;
clear all;

radii = [50, 100, 150, 200];
%radii = 50;


%% PLOT POSITION %%

for i = radii
    a = sprintf('cur_90deg_%dm.m', i);
    run (a);
    
    figure(1);
    grid on;
    hold on;
    plot(STATES(:,3),STATES(:,2));
    plot(PATH(:,2), PATH(:,1),'k');
    xlim([-300 500]);
    ylim([200 1000]);
    ylabel('North [m]');
    xlabel('East [m]');
    respath = sprintf('fig_90deg/uav_position_%dm', i);
    %cleanfigure;
    %matlab2tikz('fig/uav_position.tex');
    saveas(gcf, respath, 'epsc');
    close all;
end



%% PLOT CAMERA %%

for i = radii
    a = sprintf('cur_90deg_%dm.m', i);
    run (a);
    
    for j = (1:length(STATES(:,1)))
        [x_temp, y_temp] = camera_pos([STATES(j,8), STATES(j,9), STATES(j,10)],...
                                  [STATES(j,2), STATES(j,3), STATES(j,4)], 0.33);
        c_n_1(:,j) = x_temp;
        c_n_2(:,j) = y_temp;
    end
    
    figure(1);
    grid on;
    hold on;
    plot(c_n_1(2,:),c_n_1(1,:));
    %plot(c_n_2(2,:),c_n_2(1,:));
    plot(PATH(:,2), PATH(:,1),'k');
    xlim([-300 500]);
    ylim([200 1000]);
    ylabel('North [m]');
    xlabel('East [m]');
    respath = sprintf('fig_90deg/camera_position_%dm', i);
    %cleanfigure;
    %matlab2tikz('fig/uav_position.tex');
    saveas(gcf, respath, 'epsc');
    close all;
end



%% PLOT HEIGHT %%

figure(2);
grid on;
hold on;
for i = radii
    a = sprintf('cur_90deg_%dm.m', i);
    run (a)
    
    plot(STATES(:,1), STATES(:,4));
end
xlim([0 40]);
ylim([100 160]);
ylabel('Height [m]');
xlabel('Time [s]');
%cleanfigure;
%matlab2tikz('fig/height.tex');
saveas(gcf, 'fig_90deg/height', 'epsc');



%% PLOT ATTITUDE %%

figure(3);
grid on;
hold on;
k = 1;
for i = radii
    a = sprintf('cur_90deg_%dm.m', i);
    run (a)
    
    phi(k,:) = STATES(:,8)';
    theta(k,:) = STATES(:,9)';
    psi(k,:) = STATES(:,10)';
    k = k+1;
end
plot(STATES(:,1), phi);
ylabel('\phi [rad]');
xlabel('Time [s]');
legend('50','100','150','200','Location','NW');

% subplot(3,1,1);
% grid on;
% hold on;
% plot(STATES(:,1), phi);
% ylabel('\phi [rad]');
% ylim([-1 1]);
% 
% subplot(3,1,2);
% grid on;
% hold on;
% plot(STATES(:,1), theta);
% ylabel('\theta [rad]');
% ylim([-1 1]);
% 
% subplot(3,1,3);
% grid on;
% hold on;
% plot(STATES(:,1), psi);
% ylim([-1 1]);
% ylabel('\psi [rad]');
% xlabel('Time [s]');
%cleanfigure;
%matlab2tikz('fig/attitude.tex');
saveas(gcf, 'fig_90deg/attitude','epsc');


%% PLOT CONTROLS %%

figure(4);
grid on;
hold on;
title('CONTROl');
k = 1;
for i = radii
    a = sprintf('cur_90deg_%dm.m', i);
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