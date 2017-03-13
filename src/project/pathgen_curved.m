function path = pathgen_curved(waypoints)
    x = waypoints(1,:);
    y = waypoints(2,:);
    time = waypoints(3,:);
    t = 0:.001:max(time);
    
    x_chip = pchip(time, x, t);
    y_chip = pchip(time, y, t);
    %plot(x_chip, y_chip);
    
    path = [x_chip ; y_chip];