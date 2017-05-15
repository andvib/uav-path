function line_data = line_param(p1, p2, step)
    
    rate = p2 - p1;
    
    dist = norm(rate);
    stepsize = step/dist;
    t = 0:stepsize:1;
    
    x = p1(1) + rate(1)*t;
    y = p1(2) + rate(2)*t;
    
    line_data = [x' y'];