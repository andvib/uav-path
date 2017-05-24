close all;
clear all;

load('data.mat');
run 'results_horizon/horizon_lin_110.m'

%% Plot position
figure(1);
hold on;
grid on;

lat    = EstimatedState.lat;
lon    = EstimatedState.lon;
height = EstimatedState.height;

x = EstimatedState.x;
y = EstimatedState.y;
z = EstimatedState.z;

[lat_calc, lon_calc, h_calc] = ned2geodetic(x, y, z, lat, lon, height,...
                                             referenceEllipsoid('wgs84'));

[pos_x, pos_y, pos_z] = geodetic2ned(lat_calc, lon_calc, h_calc,...
                                     lat, lon, height,...
                                     referenceEllipsoid('wgs84'));

plot(pos_y, pos_x);
%plot(lon_calc, lat_calc);
%plot(lon_calc);

%plot(x);
%plot(y);
%plot(z);
%legend('lat','lon','x','y','z');

% figure(2);
% grid on;
% hold on;
% x = EstimatedState.x(477:890);
% y = EstimatedState.y(477:890);
% height = EstimatedState.height(477:890);
% phi = EstimatedState.phi(477:890);
% theta = EstimatedState.theta(477:890);
% psi = EstimatedState.psi(477:890);
% plot(y, x);
%    
% for i = (1:length(x))
%     x(i)
%     [x_temp, y_temp] = camera_pos([phi(i), theta(i), psi(i)],...
%                                   [x(i), y(i), height(i)], 0.331612);
%     c_n_1(:,i) = x_temp;
%     c_n_2(:,i) = y_temp;
% end
%     
% plot(c_n_1(2,:),c_n_1(1,:));
% plot(c_n_2(2,:),c_n_2(1,:));
% %plot(PATH(:,2),PATH(:,1),'k');
% 
% 
% figure(3);
% subplot(3,1,1);
% plot(phi);
% subplot(3,1,2);
% plot(theta);
% subplot(3,1,3);
% plot(psi);