function output = forces_moments(x, delta, wind, P)

    u = x(4);
    v = x(5);
    w = x(6);
    phi = x(7);
    theta = x(8);
    psi = x(9);
    p = x(10);
    q = x(11);
    r = x(12);

    d_elevator = delta(1);
    d_throttle = delta(2);
    d_aileron = delta(3);
    d_rudder = delta(4);
    
    %% Gravitational forces
    f_g = [-P.m * g * sind(theta); ...
            P.m * g * cosd(tehta) * sind(phi); ...
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
    C_D = P.C_Dp + ((C_L0 + C_L_alpha*alpha)^2 / ...
                    (pi * 1.0 * (P.b^2 / P.S)));
                
    % Lift and drag in body frame
    f_x = 0.5*P.rho*V_a^2*P.S * ((-C_D * cos(alpha) + C_L * sin(alpha)) ...
         + (-P.C_Dq * cos(alpha) + P.C_Lq * sin(alpha))*(P.c/(2*V_a))*q ...
         + (-P.C_D_delta_e * cos(alpha) + C_L_delta_e * sin(alpha))*d_elevator);
     
    f_y = 0.5*P.rho*V_a^2*P.S * ((-C_D * sin(alpha) - C_L * cos(alpha)) ...
         + (-P.C_Dq * sin(alpha) - P.C_Lq * cos(alpha))*(P.c/(2*V_A))*q ...
         + (-P.C_D_delta_e * sin(alpha) - P.C_L_delta_e * cos(alpha))*d_elevator);











    % Sum forces and moments
    F = f_g + f_a + f_p;
    M = m_a + m_p;
    
    output = [F M];

end