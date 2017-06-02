close all;
clear all;

path_start_lat = 0.656755;
path_start_lon = -2.135836;
run path_cur_easy.m;

%% LOAD POS RUN %%
load('pos_cur.mat');
[pos_lat, pos_lon, pos_hgt] = ned2geodetic(EstimatedState.x,...
                                           EstimatedState.y,...
                                           EstimatedState.z,...
                                           EstimatedState.lat,...
                                           EstimatedState.lon,...
                                           EstimatedState.height,...
                                           referenceEllipsoid('wgs84'),...
                                           'radians');
                                           
[pos_x, pos_y, pos_z] = geodetic2ned(pos_lat, pos_lon, pos_hgt,...
                                     path_start_lat, path_start_lon,...
                                     EstimatedState.height(1),...
                                     referenceEllipsoid('wgs84'),...
                                     'radians');
                                     
pos_phi = EstimatedState.phi;
pos_theta = EstimatedState.theta;
pos_psi = EstimatedState.psi;


%% LOAD PATH RUN %%
load('path_cur.mat');
[path_lat, path_lon, path_hgt] = ned2geodetic(EstimatedState.x,...
                                           EstimatedState.y,...
                                           EstimatedState.z,...
                                           EstimatedState.lat,...
                                           EstimatedState.lon,...
                                           EstimatedState.height,...
                                           referenceEllipsoid('wgs84'),...
                                           'radians');
                                           
[path_x, path_y, path_z] = geodetic2ned(path_lat, path_lon, path_hgt,...
                                     path_start_lat, path_start_lon,...
                                     EstimatedState.height(1),...
                                     referenceEllipsoid('wgs84'),...
                                     'radians');
                                     
path_phi = EstimatedState.phi;
path_theta = EstimatedState.theta;
path_psi = EstimatedState.psi;


%% PLOT PATH RUN %%
figure(1);
hold on;
grid on;
plot(PATH(:,2), PATH(:,1),'k');
plot(path_y, path_x);
for i = (1:length(path_phi))
    [x_temp, y_temp] = camera_pos([path_phi(i), path_theta(i), path_psi(i)],...
                                  [path_x(i), path_y(i), path_hgt(i)], deg2rad(0));
    c_n_1(:,i) = x_temp;
    c_n_2(:,i) = y_temp;
end
    
plot(c_n_1(2,:),c_n_1(1,:),'color',[1 .4 0]);
plot(c_n_2(2,:),c_n_2(1,:),'color',[1 .4 0]);
xlim([-300 900]);
ylim([0 1200]);
xlabel('East [m]');
ylabel('North [m]');
set(gca, 'fontsize', 14);
saveas(gcf, 'fig_cur/path_run', 'epsc');

%% PLOT POS RUN %%
figure(2);
hold on;
grid on;
plot(PATH(:,2), PATH(:,1),'k');
plot(pos_y, pos_x);
for i = (1:length(pos_phi))
    [x_temp, y_temp] = camera_pos([pos_phi(i), pos_theta(i), pos_psi(i)],...
                                  [pos_x(i), pos_y(i), pos_hgt(i)], deg2rad(0));
    c_n_1(:,i) = x_temp;
    c_n_2(:,i) = y_temp;
end
    
plot(c_n_1(2,:),c_n_1(1,:),'color',[1 .4 0]);
plot(c_n_2(2,:),c_n_2(1,:),'color',[1 .4 0]);

plot(pos_y(1350), pos_x(1350),'*');

xlim([-300 900]);
ylim([0 1200]);
xlabel('East [m]');
ylabel('North [m]');
set(gca, 'fontsize', 14);
saveas(gcf, 'fig_cur/pos_run', 'epsc');


%% PLOT TRACKING PERF %%

figure(3);
hold on;
grid on;
plot(STATES(:,3), STATES(:,2));
plot(pos_y, pos_x);
legend('Optimized path','Flown path', 'Location', 'SE');
xlim([-300 900]);
ylim([0 1200]);
set(gca, 'fontsize', 14);
saveas(gcf, 'fig_cur/tracking', 'epsc');


%% CALCULATE path ERROR %%
path_started = 0;
for i = 1:length(path_x(1:1350))
    if((path_x(i) >= 0) && (~path_started))
        path_started = 1
    end
    
    if(path_started)
        [x_temp, y_temp] = camera_pos([path_phi(i), path_theta(i), path_psi(i)],...
                        [path_x(i), path_y(i), path_hgt(i)], 0.0);
    
        min_error = 100000;
    
        for j = 1:length(PATH(:,1))
            dist = sqrt((PATH(j,1) - x_temp(1))^2 + (PATH(j,2) - x_temp(2))^2);
            if dist < min_error
                min_error = dist;
            end
        end
        if(min_error > 46)
            i
        end
        path_error(i) = min_error;
    end
end

mean_path_error = mean(path_error)
max_path_error = max(path_error)
min_path_error = min(path_error)
std_path_error = std(path_error)

%% CALCULATE CUR ERROR %%
path_started = 0;
for i = 1:length(pos_x(1:1350))
    if((pos_x(i) >= 0) && (~path_started))
        path_started = 1
        i
    end
    
    if(path_started)
        [x_temp, y_temp] = camera_pos([pos_phi(i), pos_theta(i), pos_psi(i)],...
                        [pos_x(i), pos_y(i), pos_hgt(i)], 0.0);
    
        min_error = 100000;
    
        for j = 1:length(PATH(:,1))
            dist = sqrt((PATH(j,1) - x_temp(1))^2 + (PATH(j,2) - x_temp(2))^2);
            if dist < min_error
                min_error = dist;
            end
        end
        if(min_error > 46)
            i
        end
        cur_error(i) = min_error;
    end
end

mean_cur_error = mean(cur_error)
max_cur_error = max(cur_error)
min_cur_error = min(cur_error)
std_cur_error = std(cur_error)