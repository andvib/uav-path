close all;
clear all;

run 'lin_45deg.m'

%% PLOT POSITION %%

figure(1);
grid on;
hold on;
plot(STATES(:,3), STATES(:,2),'LineWidth',2.0);
xlim([-300 500]);
ylim([0 800]);
ylabel('North [m]');
xlabel('East [m]');
%matlab2tikz('fig/uav_position.tex');
saveas(gcf, 'fig_45deg/uav_position', 'epsc');

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

plot(c_n_1(2,:),c_n_1(1,:),'LineWidth',2.0);
%plot(c_n_2(2,:),c_n_2(1,:));
%plot(STATES(:,3), STATES(:,2));

plot(PATH(:,2), PATH(:,1),'k','LineWidth',2.0);
xlim([-300 500]);
ylim([0 800]);
ylabel('North [m]');
xlabel('East [m]');
%matlab2tikz('fig/camera_position.tex');
saveas(gcf, 'fig_45deg/camera_position', 'epsc');


%% PLOT HEIGHT %%

figure(3);
grid on;
hold on; 
plot(STATES(:,1), STATES(:,4),'LineWidth',2.0);
xlim([0 40]);
ylim([100 160]);
ylabel('Height [m]');
xlabel('Time [s]');
saveas(gcf, 'fig_45deg/height', 'epsc');

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
saveas(gcf, 'fig_45deg/attitude','epsc');

%% PLOT CONTROLS %%

figure(5);
grid on;
hold on;

subplot(4,1,1);
plot(STATES(:,1), STATES(:,14),'LineWidth',2.0);
subplot(4,1,2);
plot(STATES(:,1), STATES(:,15),'LineWidth',2.0);
subplot(4,1,3);
plot(STATES(:,1), STATES(:,16),'LineWidth',2.0);
subplot(4,1,4);
plot(STATES(:,1), STATES(:,17),'LineWidth',2.0);
saveas(gcf, 'fig_45deg/control','epsc');


%% PLOT COMPARISON OF ATTITUDE %%

figure(6);
grid on;
hold on;
plot(STATES(:,1), STATES(:,8),'LineWidth',2.0);

run lin_45deg_05.m;
plot(STATES(:,1), STATES(:,8),'LineWidth',2.0);
ylabel('Angle [rad]');
xlabel('Time [s]');
leg=legend('\pm 0.08m', '\pm 1m');
set(gca, 'fontsize', 14);
set(leg, 'fontsize', 16);
saveas(gcf, 'fig_45deg/roll_comparison', 'epsc');


figure(7);
grid on;
hold on;
plot(PATH(:,2), PATH(:,1),'k','LineWidth',2.0);
%plot(STATES(:,3), STATES(:,2));
for i = (1:length(STATES(:,1)))
    [x_temp, y_temp] = camera_pos([STATES(i,8), STATES(i,9), STATES(i,10)],...
                            [STATES(i,2), STATES(i,3), STATES(i,4)], 0.0);%0.331612);
    c_n_1(:,i) = x_temp;
    c_n_2(:,i) = y_temp;
end
plot(c_n_1(2,:),c_n_1(1,:),'LineWidth',2.0);

ylabel('North [m]');
xlabel('East [m]');
xlim([-200 400]);
ylim([100 700]);
set(gca, 'fontsize', 14);
saveas(gcf, 'fig_45deg/bad_cam', 'epsc');