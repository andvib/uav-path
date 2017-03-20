function rotation_matrix = R_z(angle)
    rotation_matrix = [[cos(angle) -sin(angle) 0];
                       [sin(angle) cos(angle) 0];
                       [0 0 1]];