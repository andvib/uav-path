close all;
clear all;

%% LOAD FIRST DATASET
res = load('first_run.mat');
start_idx = 5277;
end_idx = 37549;
length = (end_idx - start_idx);
t = res.states.Position.p_N.Time(start_idx:end_idx)-start_idx*0.01;

p_N1 = res.states.Position.p_N.Data(start_idx:end_idx);
p_E1 = res.states.Position.p_E.Data(start_idx:end_idx);
p_D1 = res.states.Position.p_D.Data(start_idx:end_idx);

phi1 = res.states.Attitude.phi.Data(start_idx:end_idx);
theta1 = res.states.Attitude.theta.Data(start_idx:end_idx);
psi1 = res.states.Attitude.psi.Data(start_idx:end_idx);

%% LOAD SECOND DATASET
res = load('second_run.mat');
p_N2 = res.states2.Position.p_N.Data(start_idx:end_idx);
p_E2 = res.states2.Position.p_E.Data(start_idx:end_idx);
p_D2 = res.states2.Position.p_D.Data(start_idx:end_idx);

phi2 = res.states2.Attitude.phi.Data(start_idx:end_idx);
theta2 = res.states2.Attitude.theta.Data(start_idx:end_idx);
psi2 = res.states2.Attitude.psi.Data(start_idx:end_idx);


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

% Plot dubins Path
waypoints = [[0,0,0,0]; [1800,10,0,0];[1800,3500,0,pi/2];[5000,6000,0,pi]];
R = 600;

[N,~] = size(waypoints);

for i = (2: N)
    [L, c_s, lambda_s, c_e, lambda_e, z1, q1, z2, z3, q3] = ...
           dubinsPath(waypoints(i-1,1:3)',waypoints(i-1,4),waypoints(i,1:3)',waypoints(i,4),R);
    
    [x1, y1] = createCircle(c_s(2),c_s(1),R);
    [x2, y2] = createCircle(c_e(2),c_e(1),R);

    %plot(x1,y1,'--k');
    %plot(x2,y2,'--k');
    %plot(waypoints(i,2),waypoints(i,1),'*b');
    %plot(z1(2),z1(1),'ob');
    %plot(z2(2),z2(1),'or');
    %plot(z3(2),z3(1),'*b');
    w = plot([z1(2),z2(2)],[z1(1),z2(1)],'--k');
    
    if lambda_s == 1
        P0 = [c_s(2);c_s(1)];
        P2 = [waypoints(i-1,2);waypoints(i-1,1)];
        P1 = [z1(2);z1(1)];
    else
        P0 = [c_s(2);c_s(1)];
        P1 = [waypoints(i-1,2);waypoints(i-1,1)];
        P2 = [z1(2);z1(1)];
    end

    v1 = P1-P0;
    v2 = P2-P0;
    v3 = [0 -1;1 0]*v1; % (as you suggested)
    a = linspace(0,mod(atan2(det([v1,v2]),dot(v1,v2)),2*pi));
    % Note the absence of the 'abs' function in 'atan2'
    v = v1*cos(a)+v3*sin(a);
    plot(v(1,:)+P0(1),v(2,:)+P0(2),'--k')
    
    if lambda_e == -1
        P0 = [c_e(2);c_e(1)];
        P2 = [waypoints(i,2);waypoints(i,1)];
        P1 = [z2(2);z2(1)];
    else
        P0 = [c_e(2);c_e(1)];
        P1 = [waypoints(i,2);waypoints(i,1)];
        P2 = [z2(2);z2(1)];
    end

    v1 = P1-P0;
    v2 = P2-P0;
    v3 = [0 -1;1 0]*v1;
    a = linspace(0,mod(atan2(det([v1,v2]),dot(v1,v2)),2*pi));
    % Note the absence of the 'abs' function in 'atan2'
    v = v1*cos(a)+v3*sin(a);
    plot(v(1,:)+P0(1),v(2,:)+P0(2),'--k')
end
plot(waypoints(1,2),waypoints(1,1),'*k');
plot(waypoints(i,2),waypoints(i,1),'*k');

% Footprint 1
h = fill([c_1_1(2,:) fliplr(c_1_2(2,:))],[c_1_1(1,:) fliplr(c_1_2(1,:))],'b');
set(h, 'facealpha', 0.3);

% Footprint 2
q = fill([c_2_1(2,:) fliplr(c_2_2(2,:))],[c_2_1(1,:) fliplr(c_2_2(1,:))],'r');
set(q, 'facealpha', 0.3);

ylim([-50 5900]);
xlim([-100 6100]);

lgnd = legend([h q], 'Original Path', 'Altered Path', 'Location', 'NW');

%% TURN COMPARISON
figure(2);
% Figure 1
subplot(1,3,1);
hold on;
grid on;
waypoints = [[0,0,0,0]; [1800,10,0,0];[1800,3500,0,pi/2];[5000,6000,0,pi]];
R = 600;

[N,~] = size(waypoints);

for i = (2: N)
    [L, c_s, lambda_s, c_e, lambda_e, z1, q1, z2, z3, q3] = ...
           dubinsPath(waypoints(i-1,1:3)',waypoints(i-1,4),waypoints(i,1:3)',waypoints(i,4),R);
    
    [x1, y1] = createCircle(c_s(2),c_s(1),R);
    [x2, y2] = createCircle(c_e(2),c_e(1),R);

    %plot(x1,y1,'--k');
    %plot(x2,y2,'--k');
    %plot(waypoints(i,2),waypoints(i,1),'*b');
    %plot(z1(2),z1(1),'ob');
    %plot(z2(2),z2(1),'or');
    %plot(z3(2),z3(1),'*b');
    w = plot([z1(2),z2(2)],[z1(1),z2(1)],'--k');
    
    if lambda_s == 1
        P0 = [c_s(2);c_s(1)];
        P2 = [waypoints(i-1,2);waypoints(i-1,1)];
        P1 = [z1(2);z1(1)];
    else
        P0 = [c_s(2);c_s(1)];
        P1 = [waypoints(i-1,2);waypoints(i-1,1)];
        P2 = [z1(2);z1(1)];
    end

    v1 = P1-P0;
    v2 = P2-P0;
    v3 = [0 -1;1 0]*v1; % (as you suggested)
    a = linspace(0,mod(atan2(det([v1,v2]),dot(v1,v2)),2*pi));
    % Note the absence of the 'abs' function in 'atan2'
    v = v1*cos(a)+v3*sin(a);
    plot(v(1,:)+P0(1),v(2,:)+P0(2),'--k')
    
    if lambda_e == -1
        P0 = [c_e(2);c_e(1)];
        P2 = [waypoints(i,2);waypoints(i,1)];
        P1 = [z2(2);z2(1)];
    else
        P0 = [c_e(2);c_e(1)];
        P1 = [waypoints(i,2);waypoints(i,1)];
        P2 = [z2(2);z2(1)];
    end

    v1 = P1-P0;
    v2 = P2-P0;
    v3 = [0 -1;1 0]*v1;
    a = linspace(0,mod(atan2(det([v1,v2]),dot(v1,v2)),2*pi));
    % Note the absence of the 'abs' function in 'atan2'
    v = v1*cos(a)+v3*sin(a);
    plot(v(1,:)+P0(1),v(2,:)+P0(2),'--k')
end
plot(waypoints(1,2),waypoints(1,1),'*k');
plot(waypoints(i,2),waypoints(i,1),'*k');

% Footprint 1
h = fill([c_1_1(2,:) fliplr(c_1_2(2,:))],[c_1_1(1,:) fliplr(c_1_2(1,:))],'b');
set(h, 'facealpha', 0.3);
% Footprint 2
q = fill([c_2_1(2,:) fliplr(c_2_2(2,:))],[c_2_1(1,:) fliplr(c_2_2(1,:))],'r');
set(q, 'facealpha', 0.3);
lgnd = legend([h q], 'Original Path', 'Altered Path', 'Location', 'NW');

ylim([1800 2800]);
xlim([-50 1400]);


%Figure 2
subplot(1,3,2);
hold on;
grid on;

waypoints = [[0,0,0,0]; [1800,10,0,0];[1800,3500,0,pi/2];[5000,6000,0,pi]];
R = 600;

[N,~] = size(waypoints);

for i = (2: N)
    [L, c_s, lambda_s, c_e, lambda_e, z1, q1, z2, z3, q3] = ...
           dubinsPath(waypoints(i-1,1:3)',waypoints(i-1,4),waypoints(i,1:3)',waypoints(i,4),R);
    
    [x1, y1] = createCircle(c_s(2),c_s(1),R);
    [x2, y2] = createCircle(c_e(2),c_e(1),R);

    %plot(x1,y1,'--k');
    %plot(x2,y2,'--k');
    %plot(waypoints(i,2),waypoints(i,1),'*b');
    %plot(z1(2),z1(1),'ob');
    %plot(z2(2),z2(1),'or');
    %plot(z3(2),z3(1),'*b');
    plot([z1(2),z2(2)],[z1(1),z2(1)],'--k');
    
    if lambda_s == 1
        P0 = [c_s(2);c_s(1)];
        P2 = [waypoints(i-1,2);waypoints(i-1,1)];
        P1 = [z1(2);z1(1)];
    else
        P0 = [c_s(2);c_s(1)];
        P1 = [waypoints(i-1,2);waypoints(i-1,1)];
        P2 = [z1(2);z1(1)];
    end

    v1 = P1-P0;
    v2 = P2-P0;
    v3 = [0 -1;1 0]*v1; % (as you suggested)
    a = linspace(0,mod(atan2(det([v1,v2]),dot(v1,v2)),2*pi));
    % Note the absence of the 'abs' function in 'atan2'
    v = v1*cos(a)+v3*sin(a);
    plot(v(1,:)+P0(1),v(2,:)+P0(2),'--k')
    
    if lambda_e == -1
        P0 = [c_e(2);c_e(1)];
        P2 = [waypoints(i,2);waypoints(i,1)];
        P1 = [z2(2);z2(1)];
    else
        P0 = [c_e(2);c_e(1)];
        P1 = [waypoints(i,2);waypoints(i,1)];
        P2 = [z2(2);z2(1)];
    end

    v1 = P1-P0;
    v2 = P2-P0;
    v3 = [0 -1;1 0]*v1;
    a = linspace(0,mod(atan2(det([v1,v2]),dot(v1,v2)),2*pi));
    % Note the absence of the 'abs' function in 'atan2'
    v = v1*cos(a)+v3*sin(a);
    plot(v(1,:)+P0(1),v(2,:)+P0(2),'--k')
end
plot(waypoints(1,2),waypoints(1,1),'*k');
plot(waypoints(i,2),waypoints(i,1),'*k');

% Footprint 1
h = fill([c_1_1(2,:) fliplr(c_1_2(2,:))],[c_1_1(1,:) fliplr(c_1_2(1,:))],'b');
set(h, 'facealpha', 0.3);
% Footprint 2
q = fill([c_2_1(2,:) fliplr(c_2_2(2,:))],[c_2_1(1,:) fliplr(c_2_2(1,:))],'r');
set(q, 'facealpha', 0.3);

ylim([1700 2800]);
xlim([3200 4300]);

%Figure 3
subplot(1,3,3);
hold on;
grid on;

waypoints = [[0,0,0,0]; [1800,10,0,0];[1800,3500,0,pi/2];[5000,6000,0,pi]];
R = 600;

[N,~] = size(waypoints);

for i = (2: N)
    [L, c_s, lambda_s, c_e, lambda_e, z1, q1, z2, z3, q3] = ...
           dubinsPath(waypoints(i-1,1:3)',waypoints(i-1,4),waypoints(i,1:3)',waypoints(i,4),R);
    
    [x1, y1] = createCircle(c_s(2),c_s(1),R);
    [x2, y2] = createCircle(c_e(2),c_e(1),R);

    %plot(x1,y1,'--k');
    %plot(x2,y2,'--k');
    %plot(waypoints(i,2),waypoints(i,1),'*b');
    %plot(z1(2),z1(1),'ob');
    %plot(z2(2),z2(1),'or');
    %plot(z3(2),z3(1),'*b');
    plot([z1(2),z2(2)],[z1(1),z2(1)],'--k');
    
    if lambda_s == 1
        P0 = [c_s(2);c_s(1)];
        P2 = [waypoints(i-1,2);waypoints(i-1,1)];
        P1 = [z1(2);z1(1)];
    else
        P0 = [c_s(2);c_s(1)];
        P1 = [waypoints(i-1,2);waypoints(i-1,1)];
        P2 = [z1(2);z1(1)];
    end

    v1 = P1-P0;
    v2 = P2-P0;
    v3 = [0 -1;1 0]*v1; % (as you suggested)
    a = linspace(0,mod(atan2(det([v1,v2]),dot(v1,v2)),2*pi));
    % Note the absence of the 'abs' function in 'atan2'
    v = v1*cos(a)+v3*sin(a);
    plot(v(1,:)+P0(1),v(2,:)+P0(2),'--k')
    
    if lambda_e == -1
        P0 = [c_e(2);c_e(1)];
        P2 = [waypoints(i,2);waypoints(i,1)];
        P1 = [z2(2);z2(1)];
    else
        P0 = [c_e(2);c_e(1)];
        P1 = [waypoints(i,2);waypoints(i,1)];
        P2 = [z2(2);z2(1)];
    end

    v1 = P1-P0;
    v2 = P2-P0;
    v3 = [0 -1;1 0]*v1;
    a = linspace(0,mod(atan2(det([v1,v2]),dot(v1,v2)),2*pi));
    % Note the absence of the 'abs' function in 'atan2'
    v = v1*cos(a)+v3*sin(a);
    plot(v(1,:)+P0(1),v(2,:)+P0(2),'--k')
end
plot(waypoints(1,2),waypoints(1,1),'*k');
plot(waypoints(i,2),waypoints(i,1),'*k');

% Footprint 1
h = fill([c_1_1(2,:) fliplr(c_1_2(2,:))],[c_1_1(1,:) fliplr(c_1_2(1,:))],'b');
set(h, 'facealpha', 0.3);
% Footprint 2
q = fill([c_2_1(2,:) fliplr(c_2_2(2,:))],[c_2_1(1,:) fliplr(c_2_2(1,:))],'r');
set(q, 'facealpha', 0.3);

ylim([4900 5700]);
xlim([4800 6100]);