close all;
clear all;

path_start_lat = 0.656755;
path_start_lon = -2.135836;
run path_lin_easy.m;

%% LOAD POS RUN %%
load('pos_lin.mat');
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
load('path_lin.mat');
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
                                  [path_x(i), path_y(i), path_hgt(i)], deg2rad(19));
    c_n_1(:,i) = x_temp;
    c_n_2(:,i) = y_temp;
end
    
plot(c_n_1(2,:),c_n_1(1,:),'color',[1 .4 0]);
plot(c_n_2(2,:),c_n_2(1,:),'color',[1 .4 0]);
xlim([-300 900]);
ylim([0 1200]);
xlabel('East [m]');
ylabel('North [m]');

%% PLOT POS RUN %%
figure(2);
hold on;
grid on;
plot(PATH(:,2), PATH(:,1),'k');
plot(pos_y, pos_x);
for i = (1:length(pos_phi))
    [x_temp, y_temp] = camera_pos([pos_phi(i), pos_theta(i), pos_psi(i)],...
                                  [pos_x(i), pos_y(i), pos_hgt(i)], deg2rad(19));
    c_n_1(:,i) = x_temp;
    c_n_2(:,i) = y_temp;
end
    
plot(c_n_1(2,:),c_n_1(1,:),'color',[1 .4 0]);
plot(c_n_2(2,:),c_n_2(1,:),'color',[1 .4 0]);
xlim([-300 900]);
ylim([0 1200]);
xlabel('East [m]');
ylabel('North [m]');