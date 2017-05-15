function path = generateLinearPath(waypoints, step)
    path = [];
    for i = 2:length(waypoints(:,1))
       path = [path; line_param([waypoints(i-1,1) waypoints(i-1,2)], [waypoints(i,1) waypoints(i,2)], step)]; 
    end