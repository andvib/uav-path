close all;
clear all;


%% PLOT POSITION %%

figure(1);
grid on;
hold on;

for i = 10:10:140
    a = sprintf('results_horizon/horizon_%d.m', i);
    run (a)
    
    plot(STATES(:,3), STATES(:,2));
end

xlim([-400 400]);
ylim([0 800]);


%% PLOT CAMERA CENTRE POINT%%

figure(2);
grid on;
hold on;

for i = 20:10:130
    a = sprintf('results_horizon/horizon_%d.m', i);
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
xlim([-400 400]);
ylim([0 800]);


%% PLOT CAMERA CENTRE POINT%%

% figure(3);
% grid on;
% hold on;
% 
% for i = 90:10:130
%     a = sprintf('results_horizon/horizon_%d.m', i);
%     run (a)
%     
%     for i = (1:length(STATES(:,1)))
%         [x_temp, y_temp] = camera_pos([STATES(i,8), STATES(i,9), STATES(i,10)],...
%                                   [STATES(i,2), STATES(i,3), STATES(i,4)], 0.0);
%         c_n_1(:,i) = x_temp;
%         c_n_2(:,i) = y_temp;
%     end
% 
%     plot(c_n_1(2,:),c_n_1(1,:));
%     plot(c_n_2(2,:),c_n_2(1,:));
% end
% 
% plot(PATH(:,2), PATH(:,1),'k');
% xlim([-400 400]);
% ylim([0 800]);
% legend('90','100','110','120','130');