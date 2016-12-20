close all;
clear all;

res = load('path_aotc_hundred.mat');

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


%% COVERAGE STUFF
xq_1 = zeros(1,2000);
yq_1 = 0:0.5:999.5;

xq_2 = 0:0.5:199.5;
coefficients = polyfit([0 200], [1000 4000], 1);
a = coefficients(1);
b = coefficients(2);
yq_2 = a*xq_2 + b;

xq_3 = 200:0.5:999.5;
coefficients = polyfit([200 1000], [4000 6500], 1);
a = coefficients(1);
b = coefficients(2);
yq_3 = a*xq_3 + b;

xq_4 = 1000:0.5:1299.5;
coefficients = polyfit([1000 1300], [6500 10000], 1);
a = coefficients(1);
b = coefficients(2);
yq_4 = a*xq_4 + b;

xq = [xq_1 xq_2 xq_3 xq_4];
yq = [yq_1 yq_2 yq_3 yq_4];

figure(2);
plot(xq, yq);


xv = [c_1(2,:) fliplr(c_2(2,:))];
yv = [c_1(1,:) fliplr(c_2(1,:))];
%xq = [0 0];
%yq = [1000 2000];

[in, on] = inpolygon(xq, yq, xv, yv);
no_in = 0;
for i = 1:length(in)
    if in(i)
        no_in = no_in +  1;
    end
end

percentage = no_in / 4600