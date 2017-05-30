close all;
clear all;

weights = [1, 3, 5];

%% PLOT POSITION %%

for i = weights
    a = sprintf('lin_70deg1.m');
    run (a);
    
    figure(1);
    grid on;
    hold on;
    plot(STATES(:,3),STATES(:,2));
    plot(PATH(:,2), PATH(:,1),'k');
    for j = (1:length(STATES(:,1)))
        [x_temp, y_temp] = camera_pos([STATES(j,8), STATES(j,9), STATES(j,10)],...
                            [STATES(j,2), STATES(j,3), STATES(j,4)], 0.0);%0.331612);
        c_n_1(:,j) = x_temp;
        c_n_2(:,j) = y_temp;
    end
    plot(c_n_1(2,:),c_n_1(1,:));
    xlim([-300 500]);
    ylim([200 1000]);
    ylabel('North [m]');
    xlabel('East [m]');
    respath = sprintf('fig_70deg/uav_position_%d', i);
    %cleanfigure;
    %matlab2tikz('fig/uav_position.tex');
    set(gca, 'fontsize', 14);
    saveas(gcf, respath, 'png');
    close all;
end


%% PLOT POSITION %%

for i = weights
    a = sprintf('lin_70deg%d.m', i);
    run (a);
    
    figure(1);
    grid on;
    hold on;
    for j = (1:length(STATES(:,1)))
        [x_temp, y_temp] = camera_pos([STATES(j,8), STATES(j,9), STATES(j,10)],...
                            [STATES(j,2), STATES(j,3), STATES(j,4)], 0.0);%0.331612);
        c_n_1(:,j) = x_temp;
        c_n_2(:,j) = y_temp;
    end
    plot(c_n_1(2,:),c_n_1(1,:));
    plot(PATH(:,2), PATH(:,1),'k');
    xlim([-300 500]);
    ylim([200 1000]);
    ylabel('North [m]');
    xlabel('East [m]');
    respath = sprintf('fig_70deg/camera_position_%d', i);
    %cleanfigure;
    %matlab2tikz('fig/uav_position.tex');
    set(gca, 'fontsize', 14);
    saveas(gcf, respath, 'epsc');
    close all;
end


%% PLOT CAMERA CENTRE POINT%%

figure(2);
grid on;
hold on;
for i = (1:length(STATES(:,1)))
    [x_temp, y_temp] = camera_pos([STATES(i,8), STATES(i,9), STATES(i,10)],...
                            [STATES(i,2), STATES(i,3), STATES(i,4)], 0.0);%0.331612);
    c_n_1(:,i) = x_temp;
    c_n_2(:,i) = y_temp;
end

plot(c_n_1(2,:),c_n_1(1,:));
%plot(c_n_2(2,:),c_n_2(1,:));
%plot(STATES(:,3), STATES(:,2));

plot(PATH(:,2), PATH(:,1),'k');
xlim([-300 500]);
ylim([0 800]);
ylabel('North [m]');
xlabel('East [m]');
%matlab2tikz('fig/camera_position.tex');
set(gca, 'fontsize', 14);
saveas(gcf, 'fig_70deg/camera_position', 'epsc');


%% PLOT HEIGHT %%

figure(3);
grid on;
hold on; 
plot(STATES(:,1), STATES(:,4));
xlim([0 40]);
ylim([100 160]);
ylabel('Height [m]');
xlabel('Time [s]');
%matlab2tikz('fig/height.tex');
set(gca, 'fontsize', 14);
saveas(gcf, 'fig_70deg/height', 'epsc');

%% PLOT ATTITUDE %%

figure(4);
grid on;
hold on;

subplot(3,1,1);
grid on;
hold on;
plot(STATES(:,1), STATES(:,8));
ylabel('\phi [rad]');
ylim([-1 1]);

subplot(3,1,2);
grid on;
hold on;
plot(STATES(:,1), STATES(:,9));
ylabel('\theta [rad]');
ylim([-1 1]);

subplot(3,1,3);
grid on;
hold on;
plot(STATES(:,1), STATES(:,10));
ylim([-1 1]);
ylabel('\psi [rad]');
xlabel('Time [s]');
%matlab2tikz('fig/attitude.tex');
set(gca, 'fontsize', 14);
saveas(gcf, 'fig_70deg/attitude','epsc');

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