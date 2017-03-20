function chi_c = pathStraight(pos, chi)
    % Generate waypoints
    %persistent first_run hei;
    %if isempty(first_run)
    %    first_run=1;
    %end
    %hei = [[0,0,0];[0,0,0,0];[0,0,0,0]];
    
    waypoints = [[0,0,0,0];[1000,0,0,0]; [4000,200,1,1];...
                 [6500,1000,1,1];[10000,1300,1,1];[7000,5000,1,1]];
    
    [r, q] = followWpp(waypoints, pos);
    chi_c = followStraightLine(r, q, pos, chi);
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