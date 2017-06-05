close all;
clear all;

iterations = (10:10:140);

%% PLOT DURATION %%

figure(1);
grid on;
hold on;
k = 1;
for i = iterations
    % LINEAR %
    
    path_lin = sprintf('lin_45deg/results_horizon/horizon_lin_%d.m',i);
    run (path_lin);
    
    dur_lin(k,1) = i;
    dur_lin(k,2) = DURATION/40;
    
    % CURVED %
    path_cur = sprintf('cur_45deg_200m/results_horizon/horizon_%d.m',i);
    run (path_cur);
    
    dur_cur(k,1) = i;
    dur_cur(k,2) = DURATION/40;
    
    k = k + 1;
end

plot(dur_lin(:,1), dur_lin(:,2),'LineWidth', 2.0);
plot(dur_cur(:,1), dur_cur(:,2),'LineWidth', 2.0);
ylabel('Duration [s]');
xlabel('Length horizon');
legend('Linear', 'Curved', 'Location', 'NW');
set(gca, 'fontsize', 14);
saveas(gcf, 'duration_both', 'epsc');


%% ERROR CALCULATION %%
k = 1;
for i = iterations
    a = sprintf('cur_45deg_200m/results_horizon/horizon_%d.m', i);
    run (a)
    
    for n = (1:length(STATES(:,1)))
        [x_temp, y_temp] = camera_pos([STATES(n,8), STATES(n,9), STATES(n,10)],...
                              [STATES(n,2), STATES(n,3), STATES(n,4)], 0.0);
    
        min_error = 100000;
    
        for j = 1:length(PATH(:,1))
            dist = sqrt((PATH(j,1) - x_temp(1))^2 + (PATH(j,2) - x_temp(2))^2);
            if dist < min_error
                min_error = dist;
            end
        end
    
        cur_path_error(k,n) = min_error;
    end
    k=k+1;
end

k = 1;
for i = iterations
    a = sprintf('lin_45deg/results_horizon/horizon_lin_%d.m', i);
    run (a)
    
    for n = (1:length(STATES(:,1)))
        [x_temp, y_temp] = camera_pos([STATES(n,8), STATES(n,9), STATES(n,10)],...
                              [STATES(n,2), STATES(n,3), STATES(n,4)], 0.0);
        
        min_error = 100000;
    
        for j = 1:length(PATH(:,1))
            dist = sqrt((PATH(j,1) - x_temp(1))^2 + (PATH(j,2) - x_temp(2))^2);
            if dist < min_error
                min_error = dist;
            end
        end
    
        lin_path_error(k,n) = min_error;
    end
    k=k+1;
end

for i = 1:14
    cur_average(i) = mean(cur_path_error(i,:));
    lin_average(i) = mean(lin_path_error(i,:));
end

figure(7);
hold on;
grid on;
plot(iterations, lin_average,'LineWidth', 2.0);
plot(iterations, cur_average,'LineWidth', 2.0);
ylim([0 200]);
xlabel('Horizon Length');
ylabel('Mean Error [m]');
legend('Linear', 'Curved');
set(gca, 'fontsize', 14);
saveas(gcf, 'error_both', 'epsc');