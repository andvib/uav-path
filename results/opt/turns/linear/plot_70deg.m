close all;
clear all;

weights = [1, 3, 5];

%% PLOT POSITION %%

for i = weights
    a = sprintf('lin_70deg%d.m',i);
    run (a);
    
    figure(1);
    grid on;
    hold on;
    plot(STATES(:,3),STATES(:,2),'LineWidth',2.0);
    plot(PATH(:,2), PATH(:,1),'k','LineWidth',2.0);
    xlim([-300 500]);
    ylim([200 1000]);
    ylabel('North [m]');
    xlabel('East [m]');
    respath = sprintf('fig_70deg/uav_position_%d', i);
    set(gca, 'fontsize', 14);
    saveas(gcf, respath, 'epsc');
    close all;
end


%% PLOT CAMERA POSITION %%

for i = weights
    a = sprintf('lin_70deg%d.m', i);
    run (a);
    
    figure(2);
    grid on;
    hold on;
    for j = (1:length(STATES(:,1)))
        [x_temp, y_temp] = camera_pos([STATES(j,8), STATES(j,9), STATES(j,10)],...
                            [STATES(j,2), STATES(j,3), STATES(j,4)], 0.0);%0.331612);
        c_n_1(:,j) = x_temp;
        c_n_2(:,j) = y_temp;
    end
    plot(c_n_1(2,:),c_n_1(1,:),'color',[1 .4 0],'LineWidth',2.0);
    plot(PATH(:,2), PATH(:,1),'k','LineWidth',2.0);
    xlim([-300 500]);
    ylim([200 1000]);
    ylabel('North [m]');
    xlabel('East [m]');
    respath = sprintf('fig_70deg/camera_position_%d', i);
    set(gca, 'fontsize', 14);
    saveas(gcf, respath, 'epsc');
    close all;
end



%% PLOT HEIGHT %%
for i = weights
    a = sprintf('lin_70deg%d.m',i);
    run (a);
    
    figure(3);
    grid on;
    hold on;
    plot(STATES(:,1),STATES(:,4),'LineWidth',2.0);
    %xlim([-300 500]);
    %ylim([200 1000]);
    ylabel('Altitude [m]');
    xlabel('Time [s]');
    respath = sprintf('fig_70deg/height_%d', i);
    set(gca, 'fontsize', 14);
    saveas(gcf, respath, 'epsc');
    close all;
end

%% PLOT ATTITUDE %%

for i = weights
    a = sprintf('lin_70deg%d.m',i);
    run (a);
    
    figure(1);
    grid on;
    hold on;
    plot(STATES(:,1),STATES(:,8),'LineWidth',2.0);
    plot(STATES(:,1),STATES(:,9),'LineWidth',2.0);
    plot(STATES(:,1),STATES(:,10),'LineWidth',2.0);
    %xlim([-300 500]);
    %ylim([200 1000]);
    ylabel('Angle [rad]');
    xlabel('Time [s]');
    respath = sprintf('fig_70deg/attitude_%d', i);
    leg=legend('\phi', '\theta', '\psi', 'Location', 'NW');
    set(gca, 'fontsize', 14);
    set(leg, 'fontsize', 16);
    saveas(gcf, respath, 'epsc');
    close all;
end

%% PLOT CONTROLS %%

figure(5);
grid on;
hold on;

subplot(4,1,1);
plot(STATES(:,1), STATES(:,14));
subplot(4,1,2);
plot(STATES(:,1), STATES(:,15));
subplot(4,1,3);
plot(STATES(:,1), STATES(:,16));
subplot(4,1,4);
plot(STATES(:,1), STATES(:,17));
set(gca, 'fontsize', 14);
saveas(gcf, 'fig_70deg/control','epsc');