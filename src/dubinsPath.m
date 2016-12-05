function [L, c_s, lambda_s, c_e, lambda_e, z1, q1, z2, z3, q3] = dubinsPath(p_s, chi_s, p_e, chi_e, R)
    % Calculate start position
    c_rs = p_s + R * R_z(pi/2) * [cos(chi_s) ; sin(chi_s) ; 0];
    c_ls = p_s + R * R_z(-pi/2) * [cos(chi_s) ; sin(chi_s) ; 0];
    c_re = p_e + R * R_z(pi/2) * [cos(chi_e) ; sin(chi_e) ; 0];
    c_le = p_e + R * R_z(-pi/2) * [cos(chi_e) ; sin(chi_e) ; 0];
    
    %% Calculate total path length for all cases
    % L1 - RSR
    theta = atan2(c_re(2)-c_rs(2), c_re(1)-c_rs(2));
    dist1 = sqrt((c_rs(1)-c_re(1))^2 + (c_rs(2)-c_re(2))^2); 
    L1 = dist1 + R*(2*pi + (theta-(pi/2)) - (chi_s-(pi/2))) ...
          + R*(2*pi + (chi_e-(pi/2) - (theta-(pi/2))))
      
    % L2 - RSL
    dist = sqrt((c_le(1)-c_rs(1))^2 + (c_le(2)-c_rs(2))^2);
    theta = atan2(c_rs(2)-c_le(2), c_rs(1)-c_le(2));
    theta2 = theta - (pi/2) + asin(2*R / dist);
    L2 = sqrt(dist^2-4*R^2) + R*(2*pi + theta2 - (chi_s-(pi/2))) ...
         + R*(2*pi + (theta2+pi) - (chi_e+(pi/2)))
      
    % L3 - LSR
    dist = sqrt((c_re(1)-c_ls(1))^2 + (c_re(2)-c_ls(2))^2);
    theta = atan2(c_ls(2)-c_re(2), c_ls(1)-c_re(2));
    theta2 = acos(2*R/dist);
    L3 = sqrt(dist^2 - 4*R^2) + R*(2*pi + (chi_s+(pi/2)) - (theta+theta2))...
         + R*(2*pi + (chi_e-(pi/2)) - (theta+theta2-pi))
     
    % L4 - LSL
    dist = sqrt((c_ls(1)-c_le(1))^2 + (c_ls(2)-c_le(2))^2);
    L4 = dist + R*(2*pi + (chi_s+(pi/2)) - (theta+(pi/2))) ...
         + R*(2*pi + (theta+(pi/2)) - (chi_e+(pi/2)))
    
    L = min([L1,L2,L3,L4]);
    
    %% Calulate case
    % Case RSR
    if (L == L1)
        L1
        c_s = c_rs;
        lambda_s = 1;
        c_e = c_re;
        lambda_e = 1;
        q1 = (c_e-c_s) / sqrt((c_e(1)-c_s(1))^2 + (c_e(2)-c_s(2))^2);
        z1 = c_s + R*R_z(-pi/2)*q1;
        z2 = c_e + R*R_z(-pi/2)*q1;
    
    % Case RSL
    elseif (L == L2)
        L2
        c_s = c_rs;
        lambda_s = 1;
        c_e = c_le;
        lambda_e = -1;
        l = sqrt((c_e(1)-c_s(1))^2 + (c_e(2)-c_s(2))^2);
        theta = atan2(c_e(2)-c_s(2), c_e(1)-c_s(2));
        theta2 = theta - (pi/2) - asin(2*R/l);
        q1 = R_z(theta2+(pi/2))*[1;0;0];
        z1 = c_s + R*R_z(theta2)*[1;0;0];
        z2 = c_e + R*R_z(theta2+pi)*[1;0;0];
        
    end
    
    
    z3 = p_e;
    q3 = R_z(chi_e)*[1;0;0];