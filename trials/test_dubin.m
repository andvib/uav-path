close all;
clear all;

waypoints = [[2,2,0,0]; [6,8,0,0]];
R = 2;

[N,~] = size(waypoints)

figure(1);
hold on;
grid on;
axis equal

pos_x = [2 4 6 8];
pos_y = [2 4 4 6];
time = [0 10 20 30];
t = 0:0.1:30;
x_p = spline(time, pos_x, t);
y_p = spline(time, pos_y, t);
plot(x_p, y_p, 'k--');


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
    plot([z1(2),z2(2)],[z1(1),z2(1)],'k');
    
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
    plot(v(1,:)+P0(1),v(2,:)+P0(2),'k')
    
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
    v3 = [0 -1;1 0]*v1; % (as you suggested)
    a = linspace(0,mod(atan2(det([v1,v2]),dot(v1,v2)),2*pi));
    % Note the absence of the 'abs' function in 'atan2'
    v = v1*cos(a)+v3*sin(a);
    plot(v(1,:)+P0(1),v(2,:)+P0(2),'k')
end
legend('UAV path', 'Ground path','Location','NW');
plot(waypoints(1,2),waypoints(1,1),'*k');
plot(waypoints(i,2),waypoints(i,1),'*k');

% c_start = [0;30;0];
% c_end = [2;1;0];
% 
% [L, c_s, lambda_s, c_e, lambda_e, z1, q1, z2, z3, q3] = dubinsPath(c_start,0,c_end,0,1);
% 
% [x1, y1] = createCircle(c_s(2),c_s(1),1);
% [x2, y2] = createCircle(c_e(2),c_e(1),1);
% 
% hold on;
% grid on;
% plot(x1,y1,'--k');
% plot(x2,y2,'--k');
% plot(c_start(2),c_start(1),'*b');
% plot(z1(2),z1(1),'ob');
% plot(z2(2),z2(1),'ob');
% plot(z3(2),z3(1),'*b');
% plot([z1(2),z2(2)],[z1(1),z2(1)]);
% 
% c_start = [2;1;0];
% c_end = [10;10;0];
% 
% [L, c_s, lambda_s, c_e, lambda_e, z1, q1, z2, z3, q3] = dubinsPath(c_start,0,c_end,pi/2,1);
% 
% [x1, y1] = createCircle(c_s(2),c_s(1),1);
% [x2, y2] = createCircle(c_e(2),c_e(1),1);
% 
% plot(x1,y1,'--k');
% plot(x2,y2,'--k');
% plot(c_start(2),c_start(1),'*b');
% plot(z1(2),z1(1),'ob');
% plot(z2(2),z2(1),'ob');
% plot(z3(2),z3(1),'*b');
% plot([z1(2),z2(2)],[z1(1),z2(1)]);
% 
% c_start = [10;10;0];
% c_end = [10;15;0];
% 
% [L, c_s, lambda_s, c_e, lambda_e, z1, q1, z2, z3, q3] = dubinsPath(c_start,pi/2,c_end,0,1);
% 
% [x1, y1] = createCircle(c_s(2),c_s(1),1);
% [x2, y2] = createCircle(c_e(2),c_e(1),1);
% 
% plot(x1,y1,'--k');
% plot(x2,y2,'--k');
% plot(c_start(2),c_start(1),'*b');
% plot(z1(2),z1(1),'ob');
% plot(z2(2),z2(1),'ob');
% plot(z3(2),z3(1),'*b');
% plot([z1(2),z2(2)],[z1(1),z2(1)]);
% 
% c_start = [10;15;0];
% c_end = [15;15;0];
% 
% [L, c_s, lambda_s, c_e, lambda_e, z1, q1, z2, z3, q3] = dubinsPath(c_start,0,c_end,pi,1);
% 
% [x1, y1] = createCircle(c_s(2),c_s(1),1);
% [x2, y2] = createCircle(c_e(2),c_e(1),1);
% 
% plot(x1,y1,'--k');
% plot(x2,y2,'--k');
% plot(c_start(2),c_start(1),'*b');
% plot(z1(2),z1(1),'ob');
% plot(z2(2),z2(1),'ob');
% plot(z3(2),z3(1),'*b');
% plot([z1(2),z2(2)],[z1(1),z2(1)]);