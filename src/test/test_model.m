close all;
clear all;

aerosonde

Va = 17;
gamma = 5*pi / 180;
R = 150;

DX = [0; 0; -Va*sin(gamma); 0; 0; 0; 0; 0; Va/R; 0; 0; 0];
IDX = [3; 4; 5; 6; 7; 8; 9; 10; 11; 12];

X0 = [0; 0; 0; Va; 0; 0; 0; gamma; 0; 0; 0; 0];
IX0 = [];
U0 = [0; 0; 0; 1];
IU0 = [];
Y0 = [Va; gamma; 0];
IY0 = [1,3];

options(14) = 1000000;
[X, U, Y, DX] = trim('trim_sys', X0, U0, Y0, IX0, IU0, IY0, DX, IDX, options);

delta = [0;0;0;0];

%set_param('trim_sys', 'StopTime', int2str(10));
%sim('trim_sys')