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
set(gca, 'fontsize', 14);
saveas(gcf, 'fig/path', 'epsc');


%% PLOT LINEAR %%

% UAV Position
figure(2);
grid on;
hold on;
plot(path_lin(:,2), path_lin(:,1),'k');
plot(pos_lin(:,2), pos_lin(:,1));
ylabel('North [m]');
xlabel('East [m]');
ylim([0 1200]);
xlim([-300 900]);
set(gca, 'fontsize', 14);
saveas(gcf, 'fig/uav_pos_lin', 'epsc');

% Camera Position
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
set(gca, 'fontsize', 14);
saveas(gcf, 'fig/camera_pos_lin', 'epsc');

% Attitude
figure(4);
grid on;
hold on;
plot(time_lin, phi_lin);
plot(time_lin, theta_lin);
plot(time_lin, psi_lin);
xlim([0 110]);
ylim([-1 2.1]);
ylabel('Angle [rad]');
xlabel('Time [s]');
leg=legend('\phi', '\theta', '\psi', 'Location', 'NW');
set(gca, 'fontsize', 14);
set(leg, 'FontSize', 15);
saveas(gcf, 'fig/attitude_lin', 'epsc');

%% PLOT CURVED %%

% UAV Position
figure(5);
grid on;
hold on;
plot(path_cur(:,2), path_cur(:,1),'k');
plot(pos_cur(:,2), pos_cur(:,1));
ylabel('North [m]');
xlabel('East [m]');
ylim([0 1200]);
xlim([-300 900]);
set(gca, 'fontsize', 14);
saveas(gcf, 'fig/uav_pos_cur', 'epsc');

% Camera Position
figure(6);
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
set(gca, 'fontsize', 14);
saveas(gcf, 'fig/camera_pos_cur', 'epsc');

% Attitude
figure(7);
grid on;
hold on;
plot(time_cur, phi_cur);
plot(time_cur, theta_cur);
plot(time_cur, psi_cur);
xlim([0 110]);
ylim([-1 2.1]);
ylabel('Angle [rad]');
xlabel('Time [s]');
leg=legend('\phi', '\theta', '\psi', 'Location', 'NW');
set(gca, 'fontsize', 14);
set(leg, 'FontSize', 15);
saveas(gcf, 'fig/attitude_cur', 'epsc');

%% CALCULATE LIN ERROR %%

for i = 1:length(pos_lin(:,1))
    [x_temp, y_temp] = camera_pos([phi_lin(i), theta_lin(i), psi_lin(i)],...
                        [pos_lin(i,1), pos_lin(i,2), alt_lin(i)], 0.0);
    
    min_error = 100000;
    
    for j = 1:length(path_lin(:,1))
        dist = sqrt((path_lin(j,1) - x_temp(1))^2 + (path_lin(j,2) - x_temp(2))^2);
        if dist < min_error
            min_error = dist;
        end
    end
    lin_error(i) = min_error;
end

figure(11)
plot(time_lin, lin_error)

mean_lin_error = mean(lin_error)
max_lin_error = max(lin_error)
min_lin_error = min(lin_error)
std_lin_error = std(lin_error)


%% CALCULATE CUR ERROR %%

for i = 1:length(pos_cur(:,1))
    [x_temp, y_temp] = camera_pos([phi_cur(i), theta_cur(i), psi_cur(i)],...
                        [pos_cur(i,1), pos_cur(i,2), alt_cur(i)], 0.0);
    
    min_error = 100000;
    
    for j = 1:length(path_cur(:,1))
        dist = sqrt((path_cur(j,1) - x_temp(1))^2 + (path_cur(j,2) - x_temp(2))^2);
        if dist < min_error
            min_error = dist;
        end
    end
    cur_error(i) = min_error;
end

figure(12)
plot(time_cur, cur_error)

mean_cur_error = mean(cur_error)
max_cur_error = max(cur_error)
min_cur_error = min(cur_error)
std_cur_error = std(cur_error)
