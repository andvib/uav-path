close all;
clear all;

wpp_lin = [0 0     ; 200 0   ; 500 -200 ; 850 -200 ; 1000 -100 ; 1100 100;...
           1100 300 ; 950 550 ; 950 650 ; 1000 800 ; 1000 1500];
path_lin = generateLinearPath(wpp_lin,0.1);

wpp_cur = [[0,0,0,pi/2,150]; [0,150,0,pi/2-0.00001,150];
           [200,400,0,deg2rad(45),100];[0,800,0,pi-0.00001,232];
           [-600,400,0,pi,200];[-800,400,0,pi,10];[-1000,800,0,pi/2,200];
           [-600,1000,0,0,50];[-200,1000,0,0,10];[0, 1400, 0, pi/2, 200];
           [0, 2000,0,pi/2,10]];

path_cur = generateDubinsPath(wpp_cur, 0.1);

%path_lin = csvread('path_lin_90deg_2turns.txt');
%path_cur = csvread('path_cur_90deg_150m_2turns.txt');

figure(1);
grid on;
hold on;
%plot(path_lin(:,2), path_lin(:,1));
plot(path_cur(:,2), path_cur(:,1));

xlim([0 1700]);
ylim([-1500 300]);


csvwrite('path_cur_hard.txt',path_cur);
%csvwrite('path_lin_easy.txt',path_lin);