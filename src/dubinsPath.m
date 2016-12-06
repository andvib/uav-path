function [L, c_s, lambda_s, c_e, lambda_e, z1, q1, z2, z3, q3] = dubinsPath(p_s, chi_s, p_e, chi_e, R)

    if(sqrt((p_s(1)-p_e(1))^2 + (p_s(2)-p_e(2))^2) <= 3*R)
        too_small = 1
    end

    % Calculate start position
    c_rs = p_s + R * R_z(pi/2) * [cos(chi_s) ; sin(chi_s) ; 0];
    c_ls = p_s + R * R_z(-pi/2) * [cos(chi_s) ; sin(chi_s) ; 0];
    c_re = p_e + R * R_z(pi/2) * [cos(chi_e) ; sin(chi_e) ; 0];
    c_le = p_e + R * R_z(-pi/2) * [cos(chi_e) ; sin(chi_e) ; 0];
    
    %% Calculate total path length for all cases
    % L1 - RSR
    theta = atan2(c_re(2)-c_rs(2), c_re(1)-c_rs(1));
    %dist = sqrt((c_rs(1)-c_re(1))^2 + (c_rs(2)-c_re(2))^2); 
    dist = norm(c_rs-c_re);
    L1 = dist + R*mod(2*pi + mod(theta-(pi/2),2*pi) - mod(chi_s-(pi/2),2*pi),2*pi) ...
          + R*mod(2*pi + mod(chi_e-(pi/2),2*pi) - mod(theta-(pi/2),2*pi),2*pi)
    %L1 = 40;
      
    % L2 - RSL
    %dist = sqrt((c_le(1)-c_rs(1))^2 + (c_le(2)-c_rs(2))^2);
    dist = norm(c_le-c_rs);
    theta = atan2(c_le(2)-c_rs(2), c_le(1)-c_rs(1));
    theta2 = theta - (pi/2) + asin(2*R / dist);
    L2 = sqrt(dist^2-4*R^2) + R*mod(2*pi + mod(theta2,2*pi) - mod(chi_s-(pi/2),2*pi),2*pi) ...
         + R*mod(2*pi + mod(theta2+pi,2*pi) - mod(chi_e+(pi/2),2*pi),2*pi)
    if isreal(theta2) == 0
        L2 = 99999
    end
    %L2 = 40;
      
    % L3 - LSR
    %dist = sqrt((c_re(1)-c_ls(1))^2 + (c_re(2)-c_ls(2))^2);
    dist = norm(c_re-c_ls);
    theta = atan2(c_re(2)-c_ls(2), c_re(1)-c_ls(1));
    theta2 = acos(2*R/dist);
    L3 = sqrt(dist^2 - 4*R^2) + R*mod(2*pi + mod(chi_s+(pi/2),2*pi) - mod(theta+theta2,2*pi),2*pi)...
         + R*mod(2*pi + mod(chi_e-(pi/2),2*pi) - mod(theta+theta2-pi,2*pi),2*pi)
    %L3 = 20;
     
    % L4 - LSL
    %dist = sqrt((c_ls(1)-c_le(1))^2 + (c_ls(2)-c_le(2))^2);
    dist = norm(c_ls-c_le);
    theta = atan2(c_le(2)-c_ls(2), c_le(1)-c_ls(1));
    L4 = dist + R*mod(2*pi + mod(chi_s+(pi/2),2*pi) - mod(theta+(pi/2),2*pi),2*pi) ...
         + R*mod(2*pi + mod(theta+(pi/2),2*pi) - mod(chi_e+(pi/2),2*pi),2*pi)
    %L4 = 40;

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
        l = norm(c_e-c_s);
        theta = atan2(c_e(2)-c_s(2), c_e(1)-c_s(1));
        theta2 = theta - (pi/2) + asin(2*R/l);
        q1 = R_z(theta2+(pi/2))*[1;0;0];
        z1 = c_s + R*R_z(theta2)*[1;0;0];
        z2 = c_e + R*R_z(theta2+pi)*[1;0;0];
        
        
    % Case LSR
    elseif (L == L3)
        L3
        c_s = c_ls;
        lambda_s = -1;
        c_e = c_re;
        lambda_e = 1;
        l = sqrt((c_e(1)-c_s(1))^2 + (c_e(2)-c_s(2))^2);
        theta = atan2(c_e(2)-c_s(2), c_e(1)-c_s(1));
        theta2 = acos(2*R/l);
        q1 = R_z(theta + theta2 - (pi/2))*[1;0;0];
        z1 = c_s + R*R_z(theta+theta2)*[1;0;0];
        z2 = c_e + R*R_z(theta+theta2-pi)*[1;0;0];
        
    % Case LSL
    elseif (L == L4)
        L4
        c_s = c_ls;
        lambda_s = -1;
        c_e = c_le;
        lambda_e = -1;
        l = sqrt((c_e(1)-c_s(1))^2 + (c_e(2)-c_s(2))^2);
        q1 = (c_e - c_s) ./ l;
        z1 = c_s + R*R_z(pi/2)*q1;
        z2 = c_e + R*R_z(pi/2)*q1;
        
    end
    
    
    z3 = p_e;
    q3 = R_z(chi_e)*[1;0;0];