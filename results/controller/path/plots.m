close all;
clear all;

res = load('path_ratc_thousand.mat');
%res = load('path_aotc_thousand.mat');

start_idx = 5424;
end_idx = 34452;

p_N = res.states.Position.p_N.Data(start_idx:end_idx);
p_E = res.states.Position.p_E.Data(start_idx:end_idx);
p_D = res.states.Position.p_D.Data(start_idx:end_idx);

phi = res.states.Attitude.phi.Data(start_idx:end_idx);
theta = res.states.Attitude.theta.Data(start_idx:end_idx);
psi = res.states.Attitude.psi.Data(start_idx:end_idx);


%% FLOWN ROUTE
figure(1);
hold on;
grid on;
f = plot(p_E, p_N, 'LineWidth', 1);

waypoints = [[0,0,0,0]; [1000,0,0,0]; [4000,200,1,1]; [6500,1000,1,1];[10000,1300,1,1]];
[N,~] = size(waypoints);

for i = 1:N-1
    o = plot([waypoints(i,2),waypoints(i+1,2)],[waypoints(i,1),waypoints(i+1,1)],'--k');
end

%legend('Flown Path', 'Observation path', 'Location', 'NW');
ylim([0 10000]);
xlim([-200 1600]);

xlabel('East');
ylabel('North');

%% CAMERA POS
c_1 = zeros(2,(end_idx-start_idx));
c_2 = zeros(2,(end_idx-start_idx));

for i = (1:(end_idx-start_idx))
    [x_temp, y_temp] = camera_pos([phi(i), theta(i), psi(i)],...
                                  [p_N(i), p_E(i), p_D(i)], 0.331612558);
    c_1(:,i) = x_temp(1:2);
    c_2(:,i) = y_temp(1:2);
end

plot(c_1(2,:),c_1(1,:));
b = plot(c_2(2,:),c_2(1,:),'k');
h = fill([c_1(2,:) fliplr(c_2(2,:))],[c_1(1,:) fliplr(c_2(1,:))],'k');
set(h, 'facealpha', 0.1);
%legend('Observation path', 'Location', 'NW');
legend([o b f], 'Observation path', 'Camera Footprint', 'Flown Path', 'Location', 'NW');


%% TURN COMPARISON
figure(2);
subplot(1,2,1);
hold on;
grid on;
plot(p_E, p_N);

for i = 1:N-1
    o = plot([waypoints(i,2),waypoints(i+1,2)],[waypoints(i,1),waypoints(i+1,1)],'--k');
end

plot(c_1(2,:),c_1(1,:));
b = plot(c_2(2,:),c_2(1,:),'k');
h = fill([c_1(2,:) fliplr(c_2(2,:))],[c_1(1,:) fliplr(c_2(1,:))],'k');
set(h, 'facealpha', 0.1);

ylim([900 5000]);
xlim([-50 400]);

subplot(1,2,2);
hold on;
grid on;
plot(p_E, p_N);

for i = 1:N-1
    o = plot([waypoints(i,2),waypoints(i+1,2)],[waypoints(i,1),waypoints(i+1,1)],'--k');
end

plot(c_1(2,:),c_1(1,:));
b = plot(c_2(2,:),c_2(1,:),'k');
h = fill([c_1(2,:) fliplr(c_2(2,:))],[c_1(1,:) fliplr(c_2(1,:))],'k');
set(h, 'facealpha', 0.1);

ylim([6000 10000]);
xlim([800 1400]);
lgnd = legend([o b f], 'Observation path', 'Camera Footprint', 'Flown Path', 'Location', 'NW');
set(lgnd.BoxFace, 'ColorType','truecoloralpha', 'ColorData',uint8(255*[1;1;1;.3]));