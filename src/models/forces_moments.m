function output = forces_moments(x, delta, wind, P)
    % Ground speed
    u = x(4);
    v = x(5);
    w = x(6);
    
    % Attitude
    phi = x(7);
    theta = x(8);
    psi = x(9);
    p = x(10);
    q = x(11);
    r = x(12);

    % Wind
    W_Ns = wind(1);
    W_Es = wind(2);
    W_Ds = wind(3);
    u_Wg = wind(4);
    v_Wg = wind(5);
    w_Wg = wind(6);
    
    % Actuators
    d_elevator = delta(1);
    d_throttle = delta(2);
    d_aileron = delta(3);
    d_rudder = delta(4);
    
    %% Calculate wind
    % Rotation matrix vehicle to body frame
    Rb_v = [cos(theta)*cos(psi), cos(theta)*sin(psi), -sin(theta);
            sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), ...
            sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), sin(phi)*cos(theta);...
            cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi), ...
            cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi), cos(phi)*cos(theta)];
    
    % Wind in bodyframe
    steady_wind = [W_Ns ; W_Es ; W_Ds];
    gust_wind = [u_Wg ; v_Wg ; w_Wg];
    Vb_w = Rb_v*steady_wind + gust_wind;
    
    % Airspeed components
    u_r = u - Vb_w(1);
    v_r = v - Vb_w(2);
    w_r = w - Vb_w(3);
    
    % Calculate wind in NED
    W_NED = Rb_v' * Vb_w;
    
    %% Calculate airspeed, wind speed and ground speed
    V_a = sqrt(u_r^2 + v_r^2 + w_r^2);
    alpha = atan(w_r/u_r);
    beta = asin(v_r/V_a);

    %% Gravitational forces
    g = 9.81;
    f_g = [-P.m * g * sind(theta); ...
            P.m * g * cosd(theta) * sind(phi); ...
            P.m * g * cosd(theta) * cosd(phi)];

    %% Aerodynamic forces - longitudinal
    % Lift
    sigma = (1 + exp(-P.M * (alpha - P.alpha_0))  ...
               + exp(P.M * (alpha + P.alpha_0))) ...
            /((1 + exp(-P.M*(alpha-P.alpha_0)))...
            * (1 + exp(P.M*(alpha-P.alpha_0))));
           
    C_L = (1 - sigma) * (P.C_L0 + P.C_L_alpha*alpha) ...
          + sigma * (2* sign(alpha) * (sin(alpha))^2 * cos(alpha));
      
    % Drag
    C_D = P.C_Dp + ((P.C_L0 + P.C_L_alpha*alpha)^2 / ...
                    (pi * 1.0 * (P.b^2 / P.S)));
                
    % Lift and drag in body frame
    f_x = 0.5*P.rho*V_a^2*P.S * ((-C_D * cos(alpha) + C_L * sin(alpha)) ...
         + (-P.C_Dq * cos(alpha) + P.C_Lq * sin(alpha))*(P.c/(2*V_a))*q ...
         + (-P.C_D_delta_e * cos(alpha) + P.C_L_delta_e * sin(alpha))*d_elevator);
     
    f_z = 0.5*P.rho*V_a^2*P.S * ((-C_D * sin(alpha) - C_L * cos(alpha)) ...
         + (-P.C_Dq * sin(alpha) - P.C_Lq * cos(alpha))*(P.c/(2*V_a))*q ...
         + (-P.C_D_delta_e * sin(alpha) - P.C_L_delta_e * cos(alpha))*d_elevator);

    % Pitching moment
    m = 0.5*P.rho*V_a^2*P.S*P.c * (P.C_m0 + P.C_m_alpha *alpha ...
            + P.C_mq * (P.c/(2*V_a)) * q + P.C_m_delta_e * d_elevator);
    
    %% Aerodynamic forces and moments - lateral
    % Lateral force
    f_y = 0.5 * P.rho * V_a^2 * P.S * (P.C_Y0 + P.C_Y_beta * beta ...
            + P.C_Yp * (P.b/(2*V_a)) * p + P.C_Yr * (P.b/(2*V_a)) * r ...
            + P.C_Y_delta_a * d_aileron + P.C_Y_delta_r * d_rudder);
        
    % Roll moment
    l = 0.5 * P.rho * V_a^2 * P.S * (P.C_l0 + P.C_l_beta * beta ...
            + P.C_lp * (P.b/(2*V_a)) * p + P.C_lr * (P.b/(2*V_a)) * r ...
            + P.C_l_delta_a * d_aileron + P.C_l_delta_r * d_rudder);
        
    % Yaw moment
    n = 0.5 * P.rho * V_a^2 * P.S * (P.C_n0 + P.C_n_beta * beta ...
            + P.C_np * (P.b/(2*V_a)) * p + P.C_nr * (P.b/(2*V_a)) * r ...
            + P.C_n_delta_a * d_aileron + P.C_n_delta_r * d_rudder);
     
    %% Combine Aerodynamic forces and moments
    % Forces
    f_a = [f_x ; f_y; f_z];
    
    % Moments
    m_a = [l; m; n];
    %% Propulsion Forces and Moments
    % Propeller Thrust
    f_p = [0.5*P.rho*P.S_prop*P.C_prop * ((P.k_motor*d_throttle)^2 - V_a^2);...
           0; 0];

    % Propeller Torque
    m_p = [-P.k_tp * (P.k_omega * d_throttle)^2 ; 0 ; 0];

    %% Sum forces and moments
    F = f_g + f_a + f_p;
    M = m_a + m_p;

    %% Return values
    output = [F ; M ; V_a ; alpha ; beta ; W_NED];
end