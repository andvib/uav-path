close all;
clear all;

aerosonde

Va = 18;
gamma = 10*pi / 180;
R = 150;

DX0 = [0; 0; -Va*sin(gamma); 0; 0; 0; 0; 0; Va/R; 0; 0; 0];
IDX = [3; 4; 5; 6; 7; 8; 9; 10; 11; 12];

X0 = [0; 0; 0; Va; 0; 0; 0; gamma; 0; 0; 0; 0];
IX0 = [];
U0 = [0; 0; 0; 1];
IU0 = [];
Y0 = [Va; gamma; 0];
IY0 = [1,3];

options(14) = 10000;
[X, U, Y, DX] = trim('trim_sys', X0, U0, Y0, IX0, IU0, IY0, DX0, IDX, options)

norm(DX(3:end)-DX0(3:end)) % Should be very low



%delta = [0;0;0;0];

%set_param('trim_sys', 'StopTime', int2str(10));
%sim('trim_sys')