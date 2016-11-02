% select gains for roll loop
    % get transfer function data for delta_a to phi
    
function P=computeGains(T_phi_delta_a,T_v_delta_r,T_theta_delta_e,T_Va_theta,T_Va_delta_t,P)
    [num,den]=tfdata(T_phi_delta_a,'v');    
    a_phi2 = num(3);
    a_phi1 = den(2);
    % maximum possible aileron command
    delta_a_max = 35*pi/180;
    % Roll command when delta_a_max is achieved
    phi_max = 100*pi/180;
    % pick natural frequency to achieve delta_a_max for step of phi_max
    zeta_roll = 0.7;
    %wn_roll = sqrt(a_phi2*delta_a_max/phi_max);
    wn_roll = sqrt(a_phi2*delta_a_max*sqrt(1-zeta_roll^2)/phi_max);
    
    % set control gains based on zeta and wn
    P.roll_kp = wn_roll^2/a_phi2;
    P.roll_kd = 1.1*(2*zeta_roll*wn_roll - a_phi1)/a_phi2;
    P.roll_ki = 0.1;
    
    % add extra roll damping
    P.roll_kd = P.roll_kd+1;
    
    
% select gains for course loop
   zeta_course = 0.8;
   wn_course = wn_roll/30;
   P.course_kp = 2*zeta_course*wn_course*P.Va/P.gravity;
   P.course_ki = wn_course^2*P.Va/P.gravity;
   P.course_kd = 0;
   
% select gains for sideslip hold
    % get transfer function data for delta_r to vr
    [num,den]=tfdata(T_v_delta_r,'v');
    a_beta2 = num(2);
    a_beta1 = den(2);
    % maximum possible rudder command
    delta_r_max = 20*pi/180;
    % Roll command when delta_r_max is achieved
    vr_max = 3;
    % pick natural frequency to achieve delta_a_max for step of phi_max
    zeta_beta = 0.707;
    P.beta_kp = delta_r_max/vr_max;
    wn_beta = (a_beta2*P.beta_kp+a_beta1)/2/zeta_beta;
    P.beta_ki = 0;%wn_beta^2/a_beta2;
    P.beta_kd = 0;

   
% select gains for the pitch loop
   % get transfer function delta_e to theta
   [num,den]=tfdata(T_theta_delta_e,'v');
   a_theta1 = den(2);
   a_theta2 = den(3);
   a_theta3 = num(3);
   % maximum possible elevator command
   delta_e_max = 35*pi/180;
   % Pitch command when delta_e_max is achieved
   theta_max = 15*pi/180;
   % pick natural frequency to achieve delta_e_max for step of theta_max
   zeta_pitch = 0.9;
   wn_pitch = sqrt(abs(a_theta3)*delta_e_max*sqrt(1-zeta_pitch^2)/theta_max);
   % set control gains based on zeta and wn
   P.pitch_kp = (wn_pitch^2-a_theta2)/a_theta3
   P.pitch_kd = (2*zeta_pitch*wn_pitch - a_theta1)/a_theta3
   P.pitch_ki = 0.0
   P.K_theta_DC = P.pitch_kp*a_theta3/(a_theta2+P.pitch_kp*a_theta3);

% select gains for altitude loop
   zeta_altitude = 0.7;
   wn_altitude = wn_pitch/40;
   P.altitude_kp = 2*zeta_altitude*wn_altitude/P.K_theta_DC/P.Va;
   P.altitude_ki = wn_altitude^2/P.K_theta_DC/P.Va;
   P.altitude_kd = 0;
 
% airspeed hold using pitch
   [num,den]=tfdata(T_Va_theta,'v');
   a_V1 = den(2);
   zeta_airspeed_pitch = 0.707;
   wn_airspeed_pitch = wn_pitch/10;
   P.airspeed_pitch_kp = (a_V1-2*zeta_airspeed_pitch*wn_airspeed_pitch)/P.K_theta_DC/P.gravity;
   P.airspeed_pitch_ki = -wn_airspeed_pitch^2/P.K_theta_DC/P.gravity;
   P.airspeed_pitch_kd = 0;
 
% airspeed hold using throttle
   [num,den]=tfdata(T_Va_delta_t,'v');
   a_Vt1 = den(2);
   a_Vt2 = num(2);
   zeta_airspeed_throttle = 0.707;
%    wn_airspeed_throttle = 5;   % a value of 5 causes instability...
   wn_airspeed_throttle = 3;
   P.airspeed_throttle_kp = (2*zeta_airspeed_throttle*wn_airspeed_throttle-a_Vt1)/a_Vt1;
   P.airspeed_throttle_ki = wn_airspeed_throttle^2/a_Vt2;
   P.airspeed_throttle_kd = 0;
   P.airspeed_throttle_integrator_gain = a_Vt1/a_Vt2/P.airspeed_throttle_ki;
 
% gains for slideslip
   P.sideslip_kp = .1;
   P.sideslip_kd = -.5;
   P.sideslip_ki = 0;
end


