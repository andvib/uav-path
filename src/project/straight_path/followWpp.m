function [r, q] = followWpp(waypoints, pos)
    persistent wpp first_run;
    
    if isempty(first_run)
        first_run = 1;
        wpp = 2;
    end
    
    [N, ~] = size(waypoints);
    
    w_curr = waypoints(wpp-1,:);
    w_next = waypoints(wpp,:);
    w_next_next = waypoints(wpp+1,:);
    
    r = w_curr;
    q_curr = (w_next-w_curr)/norm(w_next-w_curr);
    q_next = (w_next_next - w_next)/norm(w_next_next - w_next);
    n_next = (q_curr - q_next)/norm(q_curr+q_next);
    
    if ((pos(1:2)'-w_next(1:2)')'*(n_next(1:2)')) > 0
        if (wpp<N-1)
            wpp = wpp + 1
        end
    end
    q = q_curr;