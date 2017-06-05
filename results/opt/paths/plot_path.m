close all;
clear all;

angles = [45, 70];

%% PLOT POSITION %%

for i = angles
    a = sprintf('results/path_cur_%ddeg_2turns_150m.m', i);
    run (a);
    
    figure(1);
    grid on;
    hold on;
    plot(STATES(:,3),STATES(:,2),'LineWidth',2.0);
    plot(PATH(:,2), PATH(:,1),'k','LineWidth',2.0);
    ylabel('North [m]');
    xlabel('East [m]');
    xlim([-100 400]);
    ylim([300 800]);
    set(gca, 'fontsize', 14);
    respath = sprintf('fig_cur/uav_position_%ddeg_150m', i);
    saveas(gcf, respath, 'epsc');
    close all;
    
end

%% PLOT CAMERA CENTRE POINT%%

for i = angles
    a = sprintf('results/path_cur_%ddeg_2turns_150m.m', i);
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
    
    plot(c_n_1(2,:), c_n_1(1,:),'color',[1 .4 0],'LineWidth',2.0);
    plot(PATH(:,2), PATH(:,1),'k','LineWidth',2.0);
    
    ylabel('North [m]');
    xlabel('East [m]');
    
    xlim([-100 400]);
    ylim([300 800]);
    set(gca, 'fontsize', 14);
    respath = sprintf('fig_cur/camera_position_%ddeg_150m', i);
    saveas(gcf, respath, 'epsc');
    close all;
end

%% PLOT HEIGHT %%

figure(3);
grid on;
hold on; 
plot(STATES(:,1), STATES(:,4),'LineWidth',2.0);
xlim([0 40]);
ylim([100 160]);
ylabel('Height [m]');
xlabel('Time [s]');
%matlab2tikz('fig/height.tex');
%saveas(gcf, 'fig_45deg/height', 'epsc');

%% PLOT ATTITUDE %%

figure(4);
grid on;
hold on;
for i = angles
    a = sprintf('results/path_cur_%ddeg_2turns_150m.m', i);
    run (a)
    
    %subplot(3,1,1);
    grid on;
    hold on;
    plot(STATES(:,1), STATES(:,10),'LineWidth',2.0);
    ylabel('\psi [rad]');
    xlabel('Time [s]');
end
leg=legend('45 degree', '70 degree');
set(gca, 'fontsize', 14);
set(leg, 'fontsize', 16);
saveas(gcf, 'fig_cur/heading', 'epsc');

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
%saveas(gcf, 'fig_45deg/control','epsc');


%% CALCULATE AND PLOT COURSE ANGLE %%
run results/path_cur_45deg_2turns_150m.m
chi_45(1) = 0;
for i = 2:length(STATES(:,1))
   chi_45(i) = atan2(STATES(i,3)-STATES(i-1,3), STATES(i,2)-STATES(i-1,2));
end

run results/path_cur_70deg_2turns_150m.m
chi_70(1) = 0;
for i = 2:length(STATES(:,1))
   chi_70(i) = atan2(STATES(i,3)-STATES(i-1,3), STATES(i,2)-STATES(i-1,2));
end

figure(6);
grid on;
hold on;
plot(STATES(:,1), chi_45,'LineWidth',2.0);
plot(STATES(:,1), chi_70,'LineWidth',2.0);
leg=legend('45 degree', '70 degree');
ylabel('\chi [rad]');
xlabel('Time [s]');
set(gca, 'fontsize', 14);
set(leg, 'fontsize', 16);
saveas(gcf, 'fig_cur/course', 'epsc');



