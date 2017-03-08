function [ dx ] = uavODE(t, x, control, p, w)
    wind = [0 0 0 0 0 0]';
    
    % Assign states
    p_N     = x(1);
    p_E     = x(2);
    p_D     = x(3);
    q1      = x(4);
    q2      = x(5);
    q3      = x(6);
    q4      = x(7);
    u       = x(8);
    v       = x(9);
    w       = x(10);
    p       = x(11);
    q       = x(12);
    r       = x(13);
    
    % Assign control
    elevator    = control(1);
    aileron     = control(2);
    rudder      = control(3);
    throttle    = control(4);
    
    % Normalize quaternion and calculate angles
    quat_norm = quatnormalize([q1, q2, q3, q4]);
    [phi, theta, psi] = q2euler(quat_norm);

    tau = forces([p_N, p_E, p_D, phi, theta, psi]', ...
                 [u, v, w, p, q, r]', ...
                 quat_norm, ...
                 [elevator, aileron, rudder, throttle]', wind);
             
    dx(1:13) = dynamics([p_N, p_E, p_D]', ...
                 quat_norm', ...
                 [u, v, w]', ...
                 [p, q, r]', tau(1:6));
    dx(14) = elevator*elevator + aileron*aileron + rudder*rudder + throttle*throttle;
    %dx(14) = v*v + w*w;
    %dx(14) = throttle*throttle;
end