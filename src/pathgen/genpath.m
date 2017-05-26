close all;
clear all;

wpp_lin = [0 0; 400 0 ; 400 300 ; 2000 300];
path_lin = generateLinearPath(wpp_lin,0.1);

wpp_cur = [[0,0,0,0]; [250,0,0,0]; [550,300,0,0];[2000, 300, 0,0]];
path_cur = generateDubinsPath(wpp_cur, 0.1, 150);

%path_lin = csvread('path_lin_90deg_2turns.txt');
%path_cur = csvread('path_cur_90deg_150m_2turns.txt');

figure(1);
grid on;
hold on;
plot(path_lin(:,2), path_lin(:,1));
plot(path_cur(:,2), path_cur(:,1));
xlim([-200 800]);
ylim([0 1000]);

%csvwrite('path_cur_90deg_150m_2turns.txt',path_cur);
%csvwrite('path_lin_90deg_2turns.txt',path_lin);