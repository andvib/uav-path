close all;
clear all;

waypoints = [[0 2000 3500 6000];
             [0 1800 1800 5000];
             [0 30 60 90]];
        
path = pathgen_curved(waypoints);

figure(1);
hold on;
grid on;
plot(waypoints(1,:), waypoints(2,:),'o');
plot(path(1,:), path(2,:));
legend('Waypoints', 'Path','Location','Best');