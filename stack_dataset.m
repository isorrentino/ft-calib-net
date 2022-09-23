function [dataset_output] = stack_dataset(filename, percentage)
    load(filename);
    
    threshold_f = 0.1; % forces
    threshold_m = 0.005; % moments
    
    wbar = waitbar(0, 'Starting');

    prefices = {'l_arm', 'r_arm'};
    
    samples = size(dataset.(prefices{1}).ft_measured,2);
    
    f = dataset.r_arm.ft_measured(1:3,:);
    m = dataset.r_arm.ft_measured(4:6,:);

    distance_f = vecnorm(diff(f')');
    distance_m = vecnorm(diff(m')');
    
    idx = find((distance_f > threshold_f) | (distance_m > threshold_m));
    
    
%     indeces = 1:samples;

    for k = 1:floor(length(idx)*percentage)
        waitbar(double(k)/double(floor(length(idx)*percentage)), wbar, sprintf('Progress: %d %%', floor(double(k)/double(floor(length(idx)*percentage))*100.0)));
        fn = prefices;
        for j=1:numel(fn)
            dataset_output.(fn{j}).ft_expected(:,k) = dataset.(fn{j}).ft_expected(:,idx(k));
            dataset_output.(fn{j}).orientation_quat(:,k) = dataset.(fn{j}).orientation_quat(:,idx(k));
            dataset_output.(fn{j}).ft_measured(:,k) = dataset.(fn{j}).ft_measured(:,idx(k));
            dataset_output.(fn{j}).ang_vel(:,k) = dataset.(fn{j}).ang_vel(:,idx(k));
            dataset_output.(fn{j}).lin_acc(:,k) = dataset.(fn{j}).lin_acc(:,idx(k));
            dataset_output.(fn{j}).ft_temperature(:,k) = dataset.(fn{j}).ft_temperature(idx(k));
        end
        dataset_output.joints(:,k) = dataset.joints(:,idx(k));
    end

    close(wbar)
end