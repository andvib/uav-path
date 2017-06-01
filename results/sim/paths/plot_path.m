close all;
clear all;


%% Read Data %%
run 'results_paths/path_lin_easy.m';
time_lin = STATES(:,1);
path_lin = [PATH(:,1) PATH(:,2)];
pos_lin = [STATES(:,2) STATES(:,3)];
alt_lin = STATES(:,4);
phi_lin = STATES(:,8);
theta_lin = STATES(:,9);
psi_lin = STATES(:,10);

run 'results_paths/path_cur_easy.m';
time_cur = STATES(1:577,1);
path_cur = [PATH(:,1) PATH(:,2)];
pos_cur = [STATES(1:577,2) STATES(1:577,3)];
alt_cur = STATES(1:577,4);
phi_cur = STATES(1:577,8);
theta_cur = STATES(1:577,9);
psi_cur = STATES(1:577,10);



%% PLOT PATHS %%

figure(1);
grid on;
hold on;
plot(path_lin(:,2), path_lin(:,1));
plot(path_cur(:,2), path_cur(:,1),'k');
ylabel('North [m]');
xlabel('East [m]');
ylim([0 1200]);
xlim([-300 900]);
saveas(gcf, 'fig/path', 'epsc');


%% PLOT LINEAR %%

figure(2);
grid on;
hold on;
plot(path_lin(:,2), path_lin(:,1),'k');
plot(pos_lin(:,2), pos_lin(:,1));
ylabel('North [m]');
xlabel('East [m]');
ylim([0 1200]);
xlim([-300 900]);
saveas(gcf, 'fig/uav_pos_lin', 'epsc');

figure(3);
grid on;
hold on;
plot(path_lin(:,2), path_lin(:,1),'k');
for j = (1:length(pos_lin(:,1)))
    [x_temp, y_temp] = camera_pos([phi_lin(j), theta_lin(j), psi_lin(j)],...
                        [pos_lin(j,1), pos_lin(j,2), alt_lin(j)], 0.0);%0.331612);
    c_n_1(:,j) = x_temp;
    c_n_2(:,j) = y_temp;
end
plot(c_n_1(2,:), c_n_1(1,:),'color',[1 .4 0]);
ylabel('North [m]');
xlabel('East [m]');
ylim([0 1200]);
xlim([-300 900]);
saveas(gcf, 'fig/camera_pos_lin', 'epsc');


%% PLOT CURVED %%

figure(4);
grid on;
hold on;
plot(path_cur(:,2), path_cur(:,1),'k');
plot(pos_cur(:,2), pos_cur(:,1));
ylabel('North [m]');
xlabel('East [m]');
ylim([0 1200]);
xlim([-300 900]);
saveas(gcf, 'fig/uav_pos_cur', 'epsc');

figure(5);
grid on;
hold on;
plot(path_cur(:,2), path_cur(:,1),'k');
for j = (1:length(pos_cur(:,1)))
    [x_temp, y_temp] = camera_pos([phi_cur(j), theta_cur(j), psi_cur(j)],...
                        [pos_cur(j,1), pos_cur(j,2), alt_cur(j)], 0.0);%0.331612);
    c_n_1(:,j) = x_temp;
    c_n_2(:,j) = y_temp;
end
plot(c_n_1(2,:), c_n_1(1,:),'color',[1 .4 0]);
ylabel('North [m]');
xlabel('East [m]');
ylim([0 1200]);
xlim([-300 900]);
saveas(gcf, 'fig/camera_pos_cur', 'epsc');