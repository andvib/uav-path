function chi_c = pathFollower(pos, chi)
    persistent chi_temp flag first_run c lambda;
    
    if isempty(first_run)
        first_run = 1;
        chi_temp = 0;
        flag = 0;
        c = [0,0];
        lambda = 0;
    end
    
    % Generate waypoints
    waypoints = [[0,0,0,0]; [1500,10,0,0];[1800,2500,0,pi/2];[4000,5000,0,pi]];
    R = 500;
    
    [flag_ret, r, q, c_ret, ~, lambda_ret] = followWppDubins(waypoints, pos, R);
    
    if flag_ret > 0
        flag = flag_ret;
        c = c_ret;
    end
    if lambda_ret ~= -2
        lambda = lambda_ret;
    end
    
    %flag
    % Follow straight path
    if flag == 1
        chi_c = followStraightLine(r, q, pos, chi);
        %follow_this_line = 1
      
    % Follow orbit
    elseif flag == 2
        chi_c = followOrbit(c, R, lambda, pos, chi);
        %follow_this_orbit = 1
    else
        chi_c = pi;
    end
end


function chi_c = followStraightLine(wpp_s, q, pos, chi)
    chi_inf = pi/4;
    k_path = 0.005;
    
    chi_q = atan2(q(2), q(1));
    
    while (chi_q - chi) < -pi
        chi_q = chi_q + 2*pi;
    end
    
    while (chi_q - chi) > pi
        chi_q = chi_q -2*pi;
    end
    
    e_py = -sin(chi_q)*(pos(1) - wpp_s(1)) + cos(chi_q)*(pos(2) - wpp_s(2));
    
    chi_c = chi_q - chi_inf*(2/pi)*atan(k_path*e_py);
end


function chi_c = followOrbit(c, rho, lambda, pos, chi)
    k_orbit = 15;

    d = sqrt((pos(1)-c(1))^2 + (pos(2)-c(2))^2);
    phi = atan2(pos(2)-c(2),pos(1)-c(1));
    
    while (phi - chi) < -pi
        phi = phi + 2*pi;
    end
    
    while (phi - chi) > pi
        phi = phi - 2*pi;
    end
      
    chi_c = (phi + lambda*((pi/2) + atan(k_orbit*((d-rho)/rho))));
end