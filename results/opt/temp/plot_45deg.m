close all;
clear all;

run 'lin_70deg.m'

%% PLOT POSITION %%

figure(1);
grid on;
hold on;
plot(STATES(:,3), STATES(:,2));
xlim([-300 500]);
ylim([0 800]);
ylabel('North [m]');
xlabel('East [m]');

%% PLOT CAMERA CENTRE POINT%%
for i = (1:length(STATES(:,1)))
    [x_temp, y_temp] = camera_pos([STATES(i,8), STATES(i,9), STATES(i,10)],...
                            [STATES(i,2), STATES(i,3), STATES(i,4)], 0.0);%0.331612);
    c_n_1(:,i) = x_temp;
    c_n_2(:,i) = y_temp;
end

plot(c_n_1(2,:),c_n_1(1,:));
%plot(c_n_2(2,:),c_n_2(1,:));
%plot(STATES(:,3), STATES(:,2));

plot(PATH(:,2), PATH(:,1),'k');
xlim([-300 500]);
ylim([0 800]);
ylabel('North [m]');
xlabel('East [m]');
%matlab2tikz('fig/camera_position.tex');
saveas(gcf, 'lin_70deg_01', 'png');