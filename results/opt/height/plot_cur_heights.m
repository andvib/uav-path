close all;
clear all;

heights = [100, 200, 300];

%% PLOT POSITION %%
figure(1);
grid on;
hold on;
for i = heights
    a = sprintf('results/cur_45deg_200m_h%dm.m',i);
    run (a);
   
    plot(STATES(:,3), STATES(:,2));
end
plot(PATH(:,2),PATH(:,1),'k');
xlabel('East [m]');
ylabel('North [m]');

xlim([-100 200]);
ylim([300 600]);
legend('100m', '200m', '300m', 'Location','NW');
set(gca, 'fontsize', 14);
saveas(gcf, 'fig_cur/uav_pos', 'epsc');


%% PLOT CAMERA %%
figure(2);
grid on;
hold on;
for i = heights
    a = sprintf('results/cur_45deg_200m_h%dm.m',i);
    run (a);
   
    for j = (1:length(STATES(:,1)))
        [x_temp, y_temp] = camera_pos([STATES(j,8), STATES(j,9), STATES(j,10)],...
                                  [STATES(j,2), STATES(j,3), STATES(j,4)], 0.0);
        c_n_1(:,j) = x_temp;
        c_n_2(:,j) = y_temp;
    end
    
    plot(c_n_1(2,:), c_n_1(1,:));
end
plot(PATH(:,2),PATH(:,1),'k');
xlabel('East [m]');
ylabel('North [m]');

xlim([-100 200]);
ylim([300 600]);

legend('100m', '200m', '300m', 'Location','NW');
set(gca, 'fontsize', 14);
saveas(gcf, 'fig_cur/camera_pos', 'epsc');


%% PLOT POSITION %%
% figure(3);
% grid on;
% hold on;
% for i = heights
%     a = sprintf('results/lin_45deg_h%dm.m',i);
%     run (a);
%    
%     plot(STATES(:,3), STATES(:,2));
% end
% plot(PATH(:,2),PATH(:,1),'k');
% xlabel('East [m]');
% ylabel('North [m]');
% 
% xlim([-100 200]);
% ylim([300 600]);
% legend('100m', '200m', '300m', 'Location','NW');
% %saveas(gcf, 'fig_cur/uav_pos', 'epsc');
% 
% 
% %% PLOT CAMERA %%
% figure(4);
% grid on;
% hold on;
% for i = heights
%     a = sprintf('results/lin_45deg_h%dm.m',i);
%     run (a);
%    
%     for j = (1:length(STATES(:,1)))
%         [x_temp, y_temp] = camera_pos([STATES(j,8), STATES(j,9), STATES(j,10)],...
%                                   [STATES(j,2), STATES(j,3), STATES(j,4)], 0.0);
%         c_n_1(:,j) = x_temp;
%         c_n_2(:,j) = y_temp;
%     end
%     
%     plot(c_n_1(2,:), c_n_1(1,:));
% end
% plot(PATH(:,2),PATH(:,1),'k');
% xlabel('East [m]');
% ylabel('North [m]');
% 
% xlim([-100 200]);
% ylim([300 600]);
% 
% legend('100m', '200m', '300m', 'Location','NW');
% %saveas(gcf, 'fig_cur/camera_pos', 'epsc');