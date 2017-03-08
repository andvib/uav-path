figure(1)
plot(out.STATES(:,3), out.STATES(:,2));
title('POSITION');

figure(2)
plot(out.STATES(:,3));
title('ALTITUDE');

figure(3)
plot(out.CONTROLS(:,2:5));
title('CONTROL');
legend('e','a','r','t');

figure(4)
plot(out.STATES(:,9:11));
title('Speed');
legend('u','v','w');