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
    
    % Gravitational forces
    f_g = [-P.m * g * sind(theta); ...
           P.m * g * cosd(tehta) * sind(phi); ...
           P.m * g * cosd(theta) * cosd(phi)];

















    % Sum forces and moments
    F = f_g + f_a + f_p;
    M = m_a + m_p;
    
    output = [F M];

end