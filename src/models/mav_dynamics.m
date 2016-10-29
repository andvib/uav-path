function output = mav_dynamics(forces_moments, x, P)

    % Forces
    f_x = forces_moments(1);
    f_y = forces_moments(2);
    f_z = forces_moments(3);
    forces = [f_x ; f_y ; f_z];
    
    % Moments
    l = forces_moments(4);
    m = forces_moments(5);
    n = forces_moments(6);
    [l ; m ; n];
    % States
    p_n = x(1);
    p_e = x(2);
    p_d = x(3);
    pos = [p_n ; p_e ; p_d];
    
    u = x(4);
    v = x(5);
    w = x(6);
    vel = [u ; v ; w];
    
    phi = x(7);
    theta = x(8);
    psi = x(9);
    attitude = [phi ; theta ; psi];
    
    p = x(10);
    q = x(11);
    r = x(12);
    att_rates = [p ; q ; r];
    
    %% Calculate gammas
    gamma = P.J_x * P.J_z - P.J_xz^2;
    gamma_1 = (P.J_xz * (P.J_x - P.J_y + P.J_z) / gamma);
    gamma_2 = (P.J_z * (P.J_z - P.J_y) +P.J_xz^2) / gamma;
    gamma_3 = P.J_z / gamma;
    gamma_4 = P.J_xz / gamma;
    gamma_5 = (P.J_z - P.J_x) / P.J_y;
    gamma_6 = P.J_xz / P.J_y;
    gamma_7 = ((P.J_x - P.J_y) * P.J_x + P.J_xz^2) / gamma;
    gamma_8 = P.J_x / gamma;

    %% Calculate new position
    % Rotation matrix body to vehicle frame
    Rv_b = [cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), ...
            cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
            cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), ...
            cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
            -sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta)];
        
    pos_dot = Rv_b * vel;
    
    %% Calculate new velocity
    vel_dot = [r*v - q*w ; p*w - r*u ; q*u - p*v] + (1/P.m) * forces;
    
    %% Calculate new attitude
    att_eq = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
              0, cos(phi), -sin(phi);
              0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
          
     att_dot = att_eq * att_rates;

    %% Calculate new attitude rates
    rates_eq1 = [gamma_1*p*q - gamma_2*q*r;
                 gamma_5*p*r - gamma_6*(p^2 - r^2);
                 gamma_7*p*q - gamma_1*q*r];
             
    rates_eq2 = [gamma_3*l + gamma_4*n;
                 (1/P.J_y)*m;
                 gamma_4*l + gamma_8*n];
             
    att_rates_dot = rates_eq1 + rates_eq2;

    %% Return values
    output = [pos_dot ; vel_dot ; att_dot ; att_rates_dot];

end