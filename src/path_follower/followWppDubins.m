function [flag_ret, r_ret, q_ret, c_ret, rho_ret, lambda_ret] = followWppDubins(path, pos, R)
    % Persistant variables to hold states and wpp pointer
    persistent state wpp first_run;
    %persistent r q c rho lambda flag;
    % If this is the first time the function is run, initialize state
    % and wpp pointer
    if isempty(first_run)
        first_run = 5;
        state = 1;
        wpp = 2;
    end
    
    r = -1;
    q = -1;
    c = -1;
    rho = -1;
    lambda = -2;
    flag = -1;
    
    [N,~] = size(path);
    
    % Find dubins path for current wpp
    [~, c_s, lambda_s, c_e, lambda_e, z1, q1, z2, z3, q3] = ...
               dubinsPath(path(wpp-1,1:3)', path(wpp-1,4), path(wpp,1:3)', path(wpp,4), R);
    %state
    
    
    %% STATE MACHINE
    if state == 1
        flag = 2;
        c = c_s;
        rho = R;
        lambda = lambda_s;

        if ((pos(1:2)'-z1(1:2))'*(-q1(1:2))) > 0
            state = 2
        end
        
    elseif state == 2
        %pos(1:2)
        %flag = 2;
        if ((pos(1:2)'-z1(1:2))'*(q1(1:2))) > 0
            state = 3
        end
        
    elseif state == 3
        flag = 1;
        r = z1;
        q = q1;
        pos(1:2);
        if ((pos(1:2)'-z2(1:2))'*(q1(1:2))) > 0
            state = 4
        end
        
    elseif state == 4
        flag = 2;
        c = c_e;
        rho = R;
        lambda = lambda_e;
        if ((pos(1:2)'-z3(1:2))'*(-q3(1:2))) > 0
            state = 5
        end
        
    elseif state == 5
        if ((pos(1:2)'-z3(1:2))'*(q3(1:2))) > 0
            if (wpp < N)
                wpp = wpp + 1
                state = 1
                %[L, c_s, lambda_s, c_e, lambda_e, z1, q1, z2, z3, q3] = ...
                %    dubinsPath(path(wpp-1,1:3)', path(wpp-1,4), path(wpp,1:3)', path(wpp,4), R);
            end
         end
    end
    
    flag_ret = flag;
    r_ret = r;
    q_ret = q;
    c_ret = c;
    rho_ret = rho;
    lambda_ret = lambda;