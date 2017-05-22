function path = generateDubinsPath(waypoints, step, R)
    %R = 100;
    [N, ~] = size(waypoints);
    path = [];

    for i = (2: N)
        [~, c_s, lambda_s, c_e, lambda_e, z1, ~, z2, ~, ~] = ...
                    dubinsPath(waypoints(i-1,1:3)',waypoints(i-1,4),...
                               waypoints(i,1:3)',waypoints(i,4),R);
                             
        if lambda_s == 1
            P0 = [c_s(2);c_s(1)];
            P2 = [waypoints(i-1,2);waypoints(i-1,1)];
            P1 = [z1(2);z1(1)];
        else
            P0 = [c_s(2);c_s(1)];
            P1 = [waypoints(i-1,2);waypoints(i-1,1)];
            P2 = [z1(2);z1(1)];
        end

        v1 = P1-P0;
        v2 = P2-P0;
        v3 = [0 -1;1 0]*v1;
        a = linspace(0,mod(atan2(det([v1,v2]),dot(v1,v2)),2*pi),1000);
        v = v1*cos(a)+v3*sin(a);

        
        if lambda_s == 1
            path = [path; (fliplr(v(1,:))+P0(1))' (fliplr(v(2,:))+P0(2))'];
        else
            path = [path; (v(1,:)+P0(1))' (v(2,:)+P0(2))'];
        end
        
        path = [path; line_param([z1(2) z1(1)], [z2(2) z2(1)], 0.1)];
        
        if lambda_e == -1
            P0 = [c_e(2);c_e(1)];
            P2 = [waypoints(i,2);waypoints(i,1)];
            P1 = [z2(2);z2(1)];
        else
            P0 = [c_e(2);c_e(1)];
            P1 = [waypoints(i,2);waypoints(i,1)];
            P2 = [z2(2);z2(1)];
        end
    
        v1 = P1-P0;
        v2 = P2-P0;
        v3 = [0 -1;1 0]*v1;
        a = linspace(0,mod(atan2(det([v1,v2]),dot(v1,v2)),2*pi),1000);
        v = v1*cos(a)+v3*sin(a);

        if lambda_e == 1
            path = [path; (fliplr(v(1,:))+P0(1))' (fliplr(v(2,:))+P0(2))'];
        else
            path = [path; (v(1,:)+P0(1))' (v(2,:)+P0(2))'];
        end
    end
    
    path = [path(:,2) path(:,1)]; 

    x = path(1,1);
    y = path(1,2);
    path_temp = [x y];
    cont = 1;
    prev_i = 1;
    while(cont)
        for i = prev_i:length(path(:,1))
            x_dist = x - path(i,1);
            y_dist = y - path(i,2);
            dist = sqrt(x_dist^2 + y_dist^2);
            if (dist > step-0.05) && (dist < step+0.05)
                x = path(i,1);
                y = path(i,2);
                path_temp = [path_temp ; [x y]];
                prev_i = i;
                break;
            end
            if i == length(path(:,1))
                cont = 0;
                break;
            end
        end
          
    end
 
    path = path_temp;
    
end