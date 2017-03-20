function P = computeGainsAerosonde(T_phi_delta_a, T_v_delta_r, ...
                              T_theta_delta_e, T_Va_theta, T_Va_delta_t, P)
   %% Roll attitude hold loop
   % Get transfer function values
   [num, den] = tfdata(T_phi_delta_a, 'v');
   a_phi2 = num(3);
   a_phi1 = den(2);
   
   % Max control surface deflection
   delta_a_max = 35*pi/180;
   phi_max = 100*pi/180;
   
   % Natural frequency and damping ratio
   zeta_roll = 0.7; % DESIGN PARAMETER
   wn_roll = sqrt(a_phi2*delta_a_max*sqrt(1-zeta_roll^2)/phi_max);
   
   % Calculate control gains
   P.roll_kp = wn_roll^2/a_phi2;
   P.roll_kd = 1.1*(2*zeta_roll*wn_roll - a_phi1)/a_phi2 + 1;
   P.roll_ki = 0.1; % DESIGN PARAMETER
   
   %% Course hold
   % Natural frequency and damping ratio
   zeta_course = 0.8;%0.7; % DESIGN PARAMETER
   wn_course = wn_roll/50; % DESIGN PARAMETER
   
   % Calculate control gains
   P.course_kp = 2*zeta_course * wn_course * P.Va / P.gravity;
   P.course_ki = wn_course^2 * P.Va / P.gravity;
   P.course_kd = 0;
   
   %% Sideslip hold
   % Get transfer function values
   [num, den] = tfdata(T_v_delta_r, 'v');
   a_beta2 = num(2);
   a_beta1 = den(2);
   
   % Max control surface deflection
   delta_r_max = 20*pi/180;
   vr_max = 3; % Roll command when delta_r_max is achieved
   
   % Natural frequency and damping ratio
   zeta_beta = 0.707; % DESIGN PARAMETER
   
   % Calculate control gains
   P.beta_kp = delta_r_max / vr_max;
   wn_beta = (a_beta2 * P.beta_kp + a_beta1) / 2 / zeta_beta;
   P.beta_ki = 0; % wn_beta^2 / a_beta2;
   P.beta_kd = 0;
   
   
   %% Pitch loop
   % Get transfer function values
   [num, den] = tfdata(T_theta_delta_e, 'v');
   a_theta1 = den(2);
   a_theta2 = den(3);
   a_theta3 = num(3);
   
   % Max control surface deflection
   delta_e_max = 35*pi/180;
   theta_max = 15*pi/180;
   
   % Natural frequency and damping ratio
   zeta_pitch = 0.4; % DESIGN PARAMETER
   wn_pitch = sqrt(abs(a_theta3)*delta_e_max*sqrt(1-zeta_pitch^2)/theta_max);
   
   % Calculate control gains
   P.pitch_kp = (wn_pitch^2 - a_theta2) / a_theta3;
   P.pitch_kd = (2 * zeta_pitch * wn_pitch - a_theta1) / a_theta3;
   P.pitch_ki = 0;
   P.K_theta_DC = P.pitch_kp * a_theta3 / (a_theta2 + P.pitch_kp * a_theta3);
   
   %% Altitude hold using pitch 
   % Natural frequency and damping ratio
   zeta_altitude = 0.8; % DESIGN PARAMETER
   wn_altitude = wn_pitch / 40; % DESIGN PARAMETER
   
   % Calculate control gains
   P.altitude_kp = 2 * zeta_altitude * wn_altitude / (P.K_theta_DC * P.Va);
   P.altitude_ki = wn_altitude^2 / (P.K_theta_DC * P.Va);
   P.altitude_kd = 0;
   
   %% Airspeed hold using pitch
   % Get transfer function values
   [num, den] = tfdata(T_Va_theta, 'v');
   a_V1 = den(2);
   
   % Natural frequency and damping ratio
   zeta_airspeed_pitch = 0.8; % DESIGN PARAMETER
   wn_airspeed_pitch = wn_pitch / 30; % DESIGN PARAMETER
   
   % Calculate control gains
   P.airspeed_pitch_kp = (a_V1-2*zeta_airspeed_pitch*wn_airspeed_pitch)/(P.K_theta_DC*P.gravity);
   P.airspeed_pitch_ki = -wn_airspeed_pitch^2 / (P.K_theta_DC * P.gravity);
   P.airspeed_pitch_kd = 0;
   
   %% Airspeed hold using throttle
   % Get transfer function values
   [num, den] = tfdata(T_Va_delta_t, 'v');
   a_Vt1 = den(2);
   a_Vt2 = num(2);
   
   % Natural frequency and damping ratio
   zeta_airspeed_throttle = 0.9; % DESIGN PARAMETER
   wn_airspeed_throttle = 1; % DESIGN PARAMETER
   
   % Calculate control gains
   P.airspeed_throttle_kp = (2 * zeta_airspeed_throttle * wn_airspeed_throttle - a_Vt1) / a_Vt1; % Feil i Gryte-kode?
   P.airspeed_throttle_ki = wn_airspeed_throttle^2 / a_Vt2;
   P.airspeed_throttle_kd = 0;
   P.airspeed_throttle_integrator_gain = a_Vt1 / (a_Vt2 * P.airspeed_throttle_ki);
   
   
   %% RATC controller
   % Transfer function values
   Gamma = P.Jx*P.Jz-P.Jxz^2;
   Gamma4 = P.Jxz/Gamma;
   Gamma8 = P.Jx/Gamma;
   C_r_r = Gamma4*P.C_ell_r + Gamma8*P.C_n_r;
   C_r_delta_r = Gamma4*P.C_ell_delta_r + Gamma8*P.C_n_delta_r;
   a_phi1 = -(1/4) * P.rho * P.Va * P.S_wing * P.b^2 * C_r_r;
   a_phi2 = (1/2) * P.rho * P.Va^2 * P.S_wing * P.b^2 * C_r_delta_r;
   
   % Natural frequency and damping ratio
   zeta_heading_rudder = 0.7;
   wn_heading_rudder = 3;
   
   % Calculate control gains
   P.heading_rudder_kp = wn_heading_rudder^2 / a_phi2;
   P.heading_rudder_kd = (2*zeta_heading_rudder^2*wn_heading_rudder - a_phi1) / a_phi2 + 0.5;
   
   %% Gains for sideslip
   %P.sideslip_kp = 0.1; % DESIGN PARAMETER
   %P.sideslip_kd = -0.5; % DESIGN PARAMETER
   %P.sideslip_ki = 0; % DESIGN PARAMETER
end