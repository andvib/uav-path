close all;
clear all;

%% Initialize
param = 'aerosonde';
init;
bus_defintions;

if strcmp('aerosonde', param)
    X_trim = [0; 0; -100.0; 0.9993; -0.0011; 0.0332; 0.0186; 35;...
              0; 0; 0; 0; 0; -0.0300; 0; -0.0294; 0; 0.0978];
    U_trim = [-0.0594; 0.0005; -0.0010; 0.0978];
    Vad = 35;
else
    X_trim = [0.0000    0.0000 -100.0000    0.9997    0.0000    0.0232   -0.0000 18.0168    0.0001    0.8366    0.0000    0.0000    0.0000 0.0039 -0.0000    0.0039   -0.0000    0.1240]';
    U_trim = [0.0079    0.0000         0    0.1240]';
    Vad = 18;
end

[T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r]= compute_tf_model(X_trim,U_trim,P);
P=computeGainsAerosonde(T_phi_delta_a,T_v_delta_r,T_theta_delta_e,T_Va_theta,T_Va_delta_t,P);

%% RUN SIMULATORE

set_param('autopilot_fly', 'StopTime', int2str(300));
sim autopilot_fly

%% ALTER PATH
d = fdesign.lowpass('Fp,Fst,Ap,Ast',0.01,0.2,0.5,40,100);
Hd = design(d,'equiripple');
phi_filtered = filter(Hd, states.Attitude.phi.Data);

c_yb = states.Position.p_D.Data.*tan(phi_filtered);
p_bd = [c_yb.*sin(states.Attitude.psi.Data) ...
        -c_yb.*cos(states.Attitude.psi.Data) ...
        zeros(states.Attitude.psi.Length, 1)];
position = [states.Position.p_N.Data states.Position.p_E.Data states.Position.p_D.Data];

for i = (1:states.Position.p_N.Length)
    p_nd(i,:) = position(i,:) + p_bd(i,:);
end


%% MAKE PLOTS
figure(1);
hold on;
grid on;
title('Position');
plot(states.Position.p_N);
plot(states.Position.p_E);
plot(states.Position.p_D);
legend('N', 'E', 'D');

figure(2);
hold on;
grid on;
title('Attitude');
plot(states.Attitude.phi);
plot(states.Attitude.theta);
plot(states.Attitude.psi);
legend('\phi','\theta','\psi');

figure(3);
hold on;
grid on;
plot(states.Velocity.u);
plot(states.Velocity.v);
plot(states.Velocity.w);
plot(airdata.Va);
plot(va_desired);
title('Velocity');
legend('u','v','w','Va','Va_d');

figure(4);
hold on;
grid on;
plot(heading_desired);
plot(states.Attitude.psi);
legend('Desired', 'Actual');
title('Heading - actual vs. desired');

figure(5);
hold on;
grid on;
plot(states.Position.p_E.Data, states.Position.p_N.Data);

waypoints = [[0,0,0,0]; [1500,10,0,0];[1800,2500,0,pi/2];[4000,5000,0,pi]];
R = 500;

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
    v3 = [0 -1;1 0]*v1;
    a = linspace(0,mod(atan2(det([v1,v2]),dot(v1,v2)),2*pi));
    % Note the absence of the 'abs' function in 'atan2'
    v = v1*cos(a)+v3*sin(a);
    plot(v(1,:)+P0(1),v(2,:)+P0(2),'k')
end
plot(waypoints(1,2),waypoints(1,1),'*k');
plot(waypoints(i,2),waypoints(i,1),'*k');

for i = (1:states.Position.p_N.Length)
    [x_temp, y_temp] = camera_pos([states.Attitude.phi.Data(i), states.Attitude.theta.Data(i), states.Attitude.psi.Data(i)],...
                                  [states.Position.p_N.Data(i), states.Position.p_E.Data(i), states.Position.p_D.Data(i)], 0.331612558);
    c_n_1(:,i) = x_temp;
    c_n_2(:,i) = y_temp;
end

plot(c_n_1(2,:),c_n_1(1,:));
plot(c_n_2(2,:),c_n_2(1,:));

%plot(p_nd(:,2),p_nd(:,1));

%p_nd_sub = p_nd(1:500:end,:);
%plot(p_nd_sub(:,2),p_nd_sub(:,1));

title('Position - NE');

figure(6);
plot(control);
title('CONTROL INPUTS');
legend('e','a','r','t');
grid on;

%save('test.mat', 'p_nd_sub');

%% SECOND RUN

% set_param('autopilot_straight_lines', 'StopTime', int2str(300));
% sim autopilot_straight_lines;
% 
% figure(7);
% plot(states.Position.p_E.Data, states.Position.p_N.Data);
% hold on;
% grid on;
% 
% %waypoints = [[0,0,0,0]; [1200,10,0,0];[1400,2500,0,pi/2];[4000,5000,0,pi]];
% %R = 400;
% 
% %[N,~] = size(waypoints);
% 
% for i = (2: N)
%     [L, c_s, lambda_s, c_e, lambda_e, z1, q1, z2, z3, q3] = ...
%            dubinsPath(waypoints(i-1,1:3)',waypoints(i-1,4),waypoints(i,1:3)',waypoints(i,4),R);
%     
%     [x1, y1] = createCircle(c_s(2),c_s(1),R);
%     [x2, y2] = createCircle(c_e(2),c_e(1),R);
% 
%     %plot(x1,y1,'--k');
%     %plot(x2,y2,'--k');
%     %plot(waypoints(i,2),waypoints(i,1),'*b');
%     %plot(z1(2),z1(1),'ob');
%     %plot(z2(2),z2(1),'or');
%     %plot(z3(2),z3(1),'*b');
%     plot([z1(2),z2(2)],[z1(1),z2(1)],'k');
%     
%     if lambda_s == 1
%         P0 = [c_s(2);c_s(1)];
%         P2 = [waypoints(i-1,2);waypoints(i-1,1)];
%         P1 = [z1(2);z1(1)];
%     else
%         P0 = [c_s(2);c_s(1)];
%         P1 = [waypoints(i-1,2);waypoints(i-1,1)];
%         P2 = [z1(2);z1(1)];
%     end
% 
%     v1 = P1-P0;
%     v2 = P2-P0;
%     v3 = [0 -1;1 0]*v1; % (as you suggested)
%     a = linspace(0,mod(atan2(det([v1,v2]),dot(v1,v2)),2*pi));
%     % Note the absence of the 'abs' function in 'atan2'
%     v = v1*cos(a)+v3*sin(a);
%     plot(v(1,:)+P0(1),v(2,:)+P0(2),'k')
%     
%     if lambda_e == -1
%         P0 = [c_e(2);c_e(1)];
%         P2 = [waypoints(i,2);waypoints(i,1)];
%         P1 = [z2(2);z2(1)];
%     else
%         P0 = [c_e(2);c_e(1)];
%         P1 = [waypoints(i,2);waypoints(i,1)];
%         P2 = [z2(2);z2(1)];
%     end
% 
%     v1 = P1-P0;
%     v2 = P2-P0;
%     v3 = [0 -1;1 0]*v1;
%     a = linspace(0,mod(atan2(det([v1,v2]),dot(v1,v2)),2*pi));
%     % Note the absence of the 'abs' function in 'atan2'
%     v = v1*cos(a)+v3*sin(a);
%     plot(v(1,:)+P0(1),v(2,:)+P0(2),'k')
% end
% plot(waypoints(1,2),waypoints(1,1),'*k');
% plot(waypoints(i,2),waypoints(i,1),'*k');
% 
% for i = (1:states.Position.p_N.Length)
%     [x_temp, y_temp] = camera_pos([states.Attitude.phi.Data(i), states.Attitude.theta.Data(i), states.Attitude.psi.Data(i)],...
%                                   [states.Position.p_N.Data(i), states.Position.p_E.Data(i), states.Position.p_D.Data(i)], 0.331612558);
%     c_n_1(:,i) = x_temp;
%     c_n_2(:,i) = y_temp;
% end
% 
% plot(c_n_1(2,:),c_n_1(1,:));
% plot(c_n_2(2,:),c_n_2(1,:));
% 
% plot(p_nd_sub(:,2),p_nd_sub(:,1));
% 
% figure(8);
% hold on;
% grid on;
% title('Attitude');
% plot(states.Attitude.phi);
% plot(states.Attitude.theta);
% plot(states.Attitude.psi);
% legend('\phi','\theta','\psi');
% 
% figure(9);
% hold on;
% grid on;
% plot(heading_d);
% plot(states.Attitude.psi);
% legend('Desired', 'Actual');
% title('Heading - actual vs. desired');