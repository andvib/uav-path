close all;
clear all;

attitude = [-10, -5, 20];
pos = [0, 0, -10];
aov = 19;

[a, b] = camera_pos(attitude, pos, aov);

plot(pos(1),pos(2),'*');
hold on;
plot([a(2), b(2)],[a(1), b(1)]);
grid on;
ylabel('N');
xlabel('E');

saveas(gcf, 'kinematic_output', 'epsc');