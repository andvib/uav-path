close all;
clear all;

start_idx = 5424;
end_idx = 34452;

%% LOAD FIRST DATASET
ratc_res = load('path_ratc_hundred.mat');
start_idx = 5277;
end_idx = 37549;
%length = (end_idx - start_idx);
%t = res.states.Position.p_N.Time(start_idx:end_idx)-start_idx*0.01;

p_N1 = ratc_res.states.Position.p_N.Data(start_idx:end_idx);
p_E1 = ratc_res.states.Position.p_E.Data(start_idx:end_idx);
p_D1 = ratc_res.states.Position.p_D.Data(start_idx:end_idx);

phi1 = ratc_res.states.Attitude.phi.Data(start_idx:end_idx);
theta1 = ratc_res.states.Attitude.theta.Data(start_idx:end_idx);
psi1 = ratc_res.states.Attitude.psi.Data(start_idx:end_idx);


%% LOAD SECOND DATASET
aotc_res = load('path_aotc_hundred.mat');
p_N2 = aotc_res.states.Position.p_N.Data(start_idx:end_idx);
p_E2 = aotc_res.states.Position.p_E.Data(start_idx:end_idx);
p_D2 = aotc_res.states.Position.p_D.Data(start_idx:end_idx);

phi2 = aotc_res.states.Attitude.phi.Data(start_idx:end_idx);
theta2 = aotc_res.states.Attitude.theta.Data(start_idx:end_idx);
psi2 = aotc_res.states.Attitude.psi.Data(start_idx:end_idx);

%% CALCULATE CAMERA 1
c_1_1 = zeros(2,(end_idx-start_idx));
c_1_2 = zeros(2,(end_idx-start_idx));

for i = (1:(end_idx-start_idx))
    [x_temp, y_temp] = camera_pos([phi1(i), theta1(i), psi1(i)],...
                                  [p_N1(i), p_E1(i), p_D1(i)], 0.331612558);
    c_1_1(:,i) = x_temp(1:2);
    c_1_2(:,i) = y_temp(1:2);
end

%% CALCULATE CAMERA 2
c_2_1 = zeros(2,(end_idx-start_idx-725));
c_2_2 = zeros(2,(end_idx-start_idx-725));

for i = (1:(end_idx-start_idx-725))
    [x_temp, y_temp] = camera_pos([phi2(i), theta2(i), psi2(i)],...
                                  [p_N2(i), p_E2(i), p_D2(i)], 0.331612558);
    c_2_1(:,i) = x_temp(1:2);
    c_2_2(:,i) = y_temp(1:2);
end


%% PLOT
figure(1);
hold on;
grid on;

waypoints = [[0,0,0,0]; [1000,0,0,0]; [4000,200,1,1]; [6500,1000,1,1];[10000,1300,1,1]];
[N,~] = size(waypoints);

for i = 1:N-1
    o = plot([waypoints(i,2),waypoints(i+1,2)],[waypoints(i,1),waypoints(i+1,1)],'--k');
end

% Footprint 1
h = fill([c_1_1(2,:) fliplr(c_1_2(2,:))],[c_1_1(1,:) fliplr(c_1_2(1,:))],'b');
set(h, 'facealpha', 0.3);

% Footprint 2
q = fill([c_2_1(2,:) fliplr(c_2_2(2,:))],[c_2_1(1,:) fliplr(c_2_2(1,:))],'r');
set(q, 'facealpha', 0.3);

ylim([0 10000]);
xlim([-200 1600]);

xlabel('East');
ylabel('North');
legend([h q o], 'Rudder Controller', 'Aileron Controller', 'Observation Path', 'Location', 'NW');