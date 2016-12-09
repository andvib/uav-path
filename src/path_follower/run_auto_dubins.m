close all;
clear all;

%% Initialize
param = 'aerosonde';
init;
bus_defintions;

if strcmp('aerosonde', param)
    X_trim = [0; 0; -200.0; 0.9993; -0.0011; 0.0332; 0.0186; 35;...
              0; 0; 0; 0; 0; -0.0300; 0; -0.0294; 0; 0.0978];
    U_trim = [-0.0594; 0.0005; -0.0010; 0.0978];
    Vad = 35;
else
    X_trim = [0.0000    0.0000 -100.0000    0.9997    0.0000    0.0232   -0.0000 18.0168    0.0001    0.8366    0.0000    0.0000    0.0000 0.0039 -0.0000    0.0039   -0.0000    0.1240]';
    U_trim = [0.0079    0.0000         0    0.1240]';
    Vad = 18;
end
%% RUN SIMULATORE
% Create track
heading_time = [0 100];
heading_d = [0 0];

input_source0 = 1;
input_step_time = 0;
input_source = 3;

end_time = 150;

% Set inital conditions
X_trim(4:7) = euler2q(0,0,0);
X_trim(8) = Vad;
set_param('autopilot_fly', 'StopTime', int2str(200));
x0 = X_trim;

[phi0,theta0,psi0] = q2euler(X_trim(4:7)/norm(X_trim(4:7)));

sim autopilot_fly

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

waypoints = [[0,0,0,0]; [1000,10,0,0];[1400,2000,0,pi/2];[1000,4000,0,pi]];
R = 200;

[N,~] = size(waypoints);

plot(waypoints(1,2),waypoints(1,1),'*b');
for i = (2: N)
    [L, c_s, lambda_s, c_e, lambda_e, z1, q1, z2, z3, q3] = ...
           dubinsPath(waypoints(i-1,1:3)',waypoints(i-1,4),waypoints(i,1:3)',waypoints(i,4),R);

    [x1, y1] = createCircle(c_s(2),c_s(1),R);
    [x2, y2] = createCircle(c_e(2),c_e(1),R);

    %plot(x1,y1,'--k');
    %plot(x2,y2,'--k');
    plot(waypoints(i,2),waypoints(i,1),'*b');
    plot(z1(2),z1(1),'ob');
    plot(z2(2),z2(1),'ob');
    plot(z3(2),z3(1),'*b');
    plot([z1(2),z2(2)],[z1(1),z2(1)]);
end

states.Position.p_N.Length
for i = (1:states.Position.p_N.Length)
    [x_temp, y_temp] = camera_pos([states.Attitude.phi.Data(i), states.Attitude.theta.Data(i), states.Attitude.psi.Data(i)],...
                                  [states.Position.p_N.Data(i), states.Position.p_E.Data(i), states.Position.p_D.Data(i)], 0.331612558);
    c_n_1(:,i) = x_temp;
    c_n_2(:,i) = y_temp;
end

plot(c_n_1(2,:),c_n_1(1,:))
plot(c_n_2(2,:),c_n_2(1,:))

title('Position - NE');

figure(6);
plot(control);
title('CONTROL INPUTS');
legend('e','a','r','t');
grid on;