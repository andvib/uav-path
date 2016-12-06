close all;
clear all;

c_start = [0;30;0];
c_end = [2;1;0];

[L, c_s, lambda_s, c_e, lambda_e, z1, q1, z2, z3, q3] = dubinsPath(c_start,0,c_end,0,1);

[x1, y1] = createCircle(c_s(2),c_s(1),1);
[x2, y2] = createCircle(c_e(2),c_e(1),1);

hold on;
grid on;
plot(x1,y1,'--k');
plot(x2,y2,'--k');
plot(c_start(2),c_start(1),'*b');
plot(z1(2),z1(1),'ob');
plot(z2(2),z2(1),'ob');
plot(z3(2),z3(1),'*b');
plot([z1(2),z2(2)],[z1(1),z2(1)]);
