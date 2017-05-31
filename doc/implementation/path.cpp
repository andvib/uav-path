VariablesGrid generateHorizon(double** path_data,
                              double timestep,
                              int horizon_length,
                              int path_length,
                              double x_start,
                              double y_start){

  /* Initalize storage */
  int no_timesteps = horizon_length/timestep;
  VariablesGrid path(8, 0, horizon_length,
                                 no_timesteps+1);
    
  DVector points(8);

  /* Initialize variables */
  double  x =  x_start;
  double  y =  y_start;

  double speed = 25.0;
  double distance = speed*timestep; // [m]

  points(0) = x;   // p_N
  points(1) = y;   // p_E
  points(2) = 0.0; // Speed (u)
  points(3) = 0.0; // h
  points(4) = 0.0; // d_elevator
  points(5) = 0.0; // d_aileron
  points(6) = 0.0; // d_rudder
  points(7) = 0.0; // d_throttle
  path.setVector(0, points);
  
  /* Generate path */
  int point_found = 0;
  for( int step = 0 ; step < no_timesteps ; step++ ){
      for( int i = path_length-1 ; i >= 0 ; --i ){            
          double x_dist = path_data[i][0] - x;
          double y_dist = path_data[i][1] - y;
          double radius = sqrt(x_dist*x_dist
                                    + y_dist*y_dist);
            
          if( (radius > distance-0.08)
                        && (radius < distance+0.08)){

            x = path_data[i][0];
            y = path_data[i][1];
                
            points(0) = path_data[i][0];
            points(1) = path_data[i][1];
                
            path.setVector(step+1, points);
            point_found = 1;
            break;
          }
      }
      if(!point_found){
        cout << "Could not find a point!\n";
        throw 1;
      }
  }
  return path;
}