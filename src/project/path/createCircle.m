function [x, y] = createCircle(x, y, r, step)
    th = 0:step:2*pi;
    
    x = r*cos(th) + x;
    y = r*sin(th) + y;