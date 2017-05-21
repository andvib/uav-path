close all;
clear all;


%% PLOT POSITION %%

figure(1);
grid on;
hold on;
title('POSITION');
for i = 90:10:120
    a = sprintf('results_horizon/horizon_lin_%d.m', i);
    run (a)
    
    plot(STATES(:,3), STATES(:,2));
end
xlim([-300 500]);
ylim([0 800]);


%% PLOT CAMERA CENTRE POINT%%

figure(2);
grid on;
hold on;
title('CAMERA CENTRE POINT');
for i = 90:10:120
    a = sprintf('results_horizon/horizon_lin_%d.m', i);
    run (a)
    
    for i = (1:length(STATES(:,1)))
        [x_temp, y_temp] = camera_pos([STATES(i,8), STATES(i,9), STATES(i,10)],...
                                  [STATES(i,2), STATES(i,3), STATES(i,4)], 0.0);
        c_n_1(:,i) = x_temp;
        c_n_2(:,i) = y_temp;
    end

    plot(c_n_1(2,:),c_n_1(1,:));
    plot(c_n_2(2,:),c_n_2(1,:));
end

plot(PATH(:,2), PATH(:,1),'k');
xlim([-300 500]);
ylim([0 800]);


%% PLOT HEIGHT %%

figure(3);
grid on;
hold on;
title('HEIGHT');
for i = 90:10:120
    a = sprintf('results_horizon/horizon_lin_%d.m', i);
    run (a)
    
    plot(STATES(:,1), STATES(:,4));
end
xlim([0 40]);
ylim([100 160]);


%% PLOT ATTITUDE %%

figure(4);
grid on;
hold on;
title('ATTITUDE');
k = 1;
for i = 90:10:120
    a = sprintf('results_horizon/horizon_lin_%d.m', i);
    run (a)
    
    phi(k,:) = STATES(:,8)';
    theta(k,:) = STATES(:,9)';
    psi(k,:) = STATES(:,10)';
    k = k+1;
end
subplot(3,1,1);
plot(STATES(:,1), phi);
subplot(3,1,2);
plot(STATES(:,1), theta);
subplot(3,1,3);
plot(STATES(:,1), psi);


%% PLOT CONTROLS %%

figure(5);
grid on;
hold on;
title('CONTROl');
k = 1;
for i = 90:10:120
    a = sprintf('results_horizon/horizon_lin_%d.m', i);
    run (a)
    
    elevator(k,:) = STATES(:,14)';
    aileron(k,:) = STATES(:,15)';
    rudder(k,:) = STATES(:,16)';
    throttle(k,:) = STATES(:,17)';
    k = k+1;
end
subplot(4,1,1);
plot(STATES(:,1), elevator);
subplot(4,1,2);
plot(STATES(:,1), aileron);
subplot(4,1,3);
plot(STATES(:,1), rudder);
subplot(4,1,4);
plot(STATES(:,1), throttle);

%% PLOT DURATION %%

figure(6);
grid on;
hold on;
title('DURATION');
k = 1;
for i = 90:10:120
    a = sprintf('results_horizon/horizon_lin_%d.m', i);
    run (a)
    
    dur(k,1) = i;
    dur(k,2) = DURATION;
    k = k+1;
end
plot(dur(:,1), dur(:,2),'*')