function [c_n_1, c_n_2] = camera_pos(attitude, pos, aov)
    % Attitude is [phi, theta, psi], position is [N,E,D], aov in deg

    r_z = [[cosd(attitude(3)), -sind(attitude(3)), 0],
           [sind(attitude(3)), cosd(attitude(3)), 0],
           [0, 0, 1]];
    
    c_b_1 = [pos(3)*tan(attitude(2)), pos(3)*tan(attitude(1)+aov), 0]';
    c_b_2 = [pos(3)*tan(attitude(2)), pos(3)*tan(attitude(1)-aov), 0]';
    c_b_n_1 = r_z*c_b_1;
    c_b_n_2 = r_z*c_b_2;
    c_n_1 = pos' + c_b_n_1;
    c_n_2 = pos' + c_b_n_2;