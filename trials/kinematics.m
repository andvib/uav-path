phi = -30;
theta = -30;
psi = 45;
h = -10;

r_z = [[cosd(psi), -sind(psi), 0],
       [sind(psi), cosd(psi), 0],
       [0, 0, 1]];
   
p = [5, 5, h]';

c_b = [p(3)*sind(theta), p(3)*sind(phi), 0]';
c_b_n = r_z*c_b;
c_n = p + c_b_n