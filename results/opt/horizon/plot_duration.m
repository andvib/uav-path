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
    dur_lin(k,2) = DURATION;
    
    % CURVED %
    path_cur = sprintf('cur_45deg_200m/results_horizon/horizon_%d.m',i);
    run (path_cur);
    
    dur_cur(k,1) = i;
    dur_cur(k,2) = DURATION;
    
    k = k + 1;
end

plot(dur_lin(:,1), dur_lin(:,2));
plot(dur_cur(:,1), dur_cur(:,2));
ylabel('Duration [s]');
xlabel('Length horizon');
legend('Linear', 'Curved', 'Location', 'NW');
saveas(gcf, 'duration_both', 'epsc');