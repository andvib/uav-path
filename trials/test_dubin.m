close all;
clear all;

[L, c_s, lambda_s, c_e, lambda_e, z1, q1, z2, z3, q3] = dubinsPath([1;1;0],0,[10;10;0],pi,1);

[x1, y1] = createCircle(c_s(2),c_s(1),1);
[x2, y2] = createCircle(c_e(2),c_e(1),1);

hold on;
grid on;
plot(x1,y1,'--k');
plot(x2,y2,'--k');
plot(1,1,'*b');
plot(z1(2),z1(1),'ob');
plot(z2(2),z2(1),'ob');
plot(z3(2),z3(1),'*b');
plot([z1(2),z2(2)],[z1(1),z2(1)]);
