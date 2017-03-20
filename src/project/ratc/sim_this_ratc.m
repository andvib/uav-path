close all;
clear all;

param = 'aerosonde';
init;
bus_defintions;

X_trim =  [-0.0000  -0.0000 -150.0000 0.9994 -0.0009 0.0332 0.0131 35.7334 -0.0122 2.3795...
            0.0000 0.0000 0.0000 -0.0300 -0.0000 -0.0295 0.0000 0.0978]';
U_trim = [-0.0594 0.0005 -0.0010 0.0978]';
Vad = 35;

[T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r]= compute_tf_model(X_trim,U_trim,P);
P=computeGainsAerosonde(T_phi_delta_a,T_v_delta_r,T_theta_delta_e,T_Va_theta,T_Va_delta_t,P);

sim_time = 150;
set_param('ratc_sim', 'StopTime', int2str(sim_time));
sim ratc_sim;

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
plot(control);
title('CONTROL INPUTS');
legend('e','a','r','t');
grid on;

figure(4);
hold on;
grid on;
plot(states.Position.p_E.Data, states.Position.p_N.Data);

for i = (1:states.Position.p_N.Length)
    [x_temp, y_temp] = camera_pos([states.Attitude.phi.Data(i), states.Attitude.theta.Data(i), states.Attitude.psi.Data(i)],...
                                  [states.Position.p_N.Data(i), states.Position.p_E.Data(i), states.Position.p_D.Data(i)], 0.331612558);
    c_n_1(:,i) = x_temp;
    c_n_2(:,i) = y_temp;
end

plot(c_n_1(2,:),c_n_1(1,:))
plot(c_n_2(2,:),c_n_2(1,:))

title('Position - NE');

figure(5);
hold on;
grid on;
plot(states.Attitude.psi);
plot(course_desired);
title('COURSE');