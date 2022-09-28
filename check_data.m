labels = {'fx','fy','fz','mx','my','mz'};

%% Plot ft measured VS ft expected

offset = dataset.r_arm.ft_measured(:,1) - dataset.r_arm.ft_expected(:,1);

figure
for i = 1 : 6
    subplot(2,3,i)
    plot(dataset.r_arm.ft_measured(i,:)' - offset(i))
    hold on
    plot(dataset.r_arm.ft_expected(i,:)')
    ylabel(labels{i})
    legend('measured','expected')
end


%% Plot temperature

figure,plot(dataset.r_arm.ft_temperature)


%% Plot norm of diff of forces and moments
f = dataset.r_arm.ft_measured(1:3,:);
m = dataset.r_arm.ft_measured(4:6,:);

diff_f = diff(f');
diff_m = diff(m');

figure,
subplot(2,1,1)
plot(vecnorm(diff_f(10:end-10,:)'))
title('vecnorm(diff(f))')
subplot(2,1,2)
plot(vecnorm(diff_m(10:end-10,:)'))
title('vecnorm(diff(m))')



%% Check moments old method ext contact vs new method modified urdfs


load('/home/isorrentino/dev/ft-calib-net/datasets/urdf_robot_logger_device_2022_09_22_17_10_20.mat')
ft1 = dataset.r_arm.ft_expected';
load('/home/isorrentino/dev/ft-calib-net/datasets/robot_logger_device_2022_09_22_17_10_20.mat')
ft2 = dataset.r_arm.ft_expected';

figure
for i = 1 : 6
    subplot(2,3,i)
    plot(ft1(:,i),'linewidth',1.5)
    hold on
    plot(ft2(:,i),'linewidth',1.5)
    xlabel('samples')
    ylabel(labels{i})
    legend('new with 2.5 kg','old with 2.5 kg')
end

