function createArc(radius, rad_ang)

    circr = @(radius,rad_ang)  [radius*cos(rad_ang);  radius*sin(rad_ang)];         % Circle Function For Angles In Radians
    circd = @(radius,deg_ang)  [radius*cosd(deg_ang);  radius*sind(deg_ang)];       % Circle Function For Angles In Degrees
    
    N = 25;                              % Number Of Points In Complete Circle
    r_angl = linspace(pi/4, 3*pi/4, N);  % Angle Defining Arc Segment (radians)
    radius = 1.5;                        % Arc Radius
    xy_r = circr(radius,r_angl);         % Matrix (2xN) Of (x,y) Coordinates
    
    plot(xy_r(1,:), xy_r(2,:), 'b-')                    % Draw An Arc Of Blue Stars
    axis([-1.25*radius  1.25*radius    0  1.25*radius]) % Set Axis Limits
    axis equal                                          % No Distortion With ‘axis equal’