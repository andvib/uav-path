close all;
clear all;

radii = [50, 100, 150, 200, 250, 300];

x_limits = [[-300 200];
            [-150 300];
            [-100 400];
            [-100 500];
            [-100 600];
            [-100 700]];

y_limits = [[100 600];
            [150 600];
            [250 650];
            [300 900];
            [400 1100];
            [500 1300]];
        
%% PLOT POSITION %%
k = 1;
for i = radii
    a = sprintf('cur_180deg_%dm.m', i);
    run (a);
    
    figure(1);
    grid on;
    hold on;
    plot(STATES(:,3),STATES(:,2));
    plot(PATH(:,2), PATH(:,1),'k');
    xlim(x_limits(k,:));
    ylim(y_limits(k,:));
    ylabel('North [m]');
    xlabel('East [m]');
    respath = sprintf('fig_180deg/uav_position_%dm', i);
    %cleanfigure;
    %matlab2tikz('fig/uav_position.tex');
    saveas(gcf, respath, 'epsc');
    close all;
    k = k+1;
end



%% PLOT CAMERA %%
k=1;
for i = radii
    a = sprintf('cur_180deg_%dm.m', i);
    run (a);
    
    for j = (1:length(STATES(:,1)))
        [x_temp, y_temp] = camera_pos([STATES(j,8), STATES(j,9), STATES(j,10)],...
                                  [STATES(j,2), STATES(j,3), STATES(j,4)], 0.0);
        c_n_1(:,j) = x_temp;
        c_n_2(:,j) = y_temp;
    end
    
    figure(1);
    grid on;
    hold on;
    plot(c_n_1(2,:),c_n_1(1,:));
    %plot(c_n_2(2,:),c_n_2(1,:));
    plot(PATH(:,2), PATH(:,1),'k');
    xlim(x_limits(k,:));
    ylim(y_limits(k,:));
    ylabel('North [m]');
    xlabel('East [m]');
    respath = sprintf('fig_180deg/camera_position_%dm', i);
    %cleanfigure;
    %matlab2tikz('fig/uav_position.tex');
    saveas(gcf, respath, 'epsc');
    close all;
    k = k + 1;
end



%% PLOT HEIGHT %%

figure(2);
grid on;
hold on;
for i = radii
    a = sprintf('cur_180deg_%dm.m', i);
    run (a)
    
    plot(STATES(:,1), STATES(:,4));
end
xlim([0 40]);
ylim([100 160]);
ylabel('Height [m]');
xlabel('Time [s]');
%cleanfigure;
%matlab2tikz('fig/height.tex');
saveas(gcf, 'fig_180deg/height', 'epsc');



%% PLOT ATTITUDE %%

figure(3);
grid on;
hold on;
for i = radii
    a = sprintf('cur_180deg_%dm.m', i);
    run (a)
    
    %subplot(3,1,1);
    grid on;
    hold on;
    plot(STATES(:,1), STATES(:,8));
    ylabel('\phi [rad]');
    xlabel('Time [s]');
    ylim([-1 1]);

    %subplot(3,1,2);
    %grid on;
    %hold on;
    %plot(STATES(:,1), STATES(:,9));
    %ylabel('\theta [rad]');
    %ylim([-1 1]);

    %subplot(3,1,3);
    %grid on;
    %hold on;
    %plot(STATES(:,1), STATES(:,10));
    %ylabel('\psi [rad]');
    %ylim([-1 1]);
end
%cleanfigure;
%matlab2tikz('fig/attitude.tex');
legend('50', '100', '150', '200', '250', '300');
saveas(gcf, 'fig_180deg/attitude','epsc');


%% PLOT CONTROLS %%

figure(4);
grid on;
hold on;
title('CONTROl');
for i = radii
    a = sprintf('cur_180deg_%dm.m', i);
    run (a)
    
    subplot(4,1,1);
    plot(STATES(:,1), STATES(:,14));
    subplot(4,1,2);
    plot(STATES(:,1), STATES(:,15));
    subplot(4,1,3);
    plot(STATES(:,1), STATES(:,16));
    subplot(4,1,4);
    plot(STATES(:,1), STATES(:,17));
end
