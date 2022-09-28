function [dataset_output] = stack_dataset(filename, percentage, calibration)
    load(filename);
    
    threshold_f = 0.1; % forces
    threshold_m = 0.005; % moments
    
    wbar = waitbar(0, 'Starting');

    prefices = {'r_arm'};
    
    samples = size(dataset.(prefices{1}).ft_measured,2);
    
    f = dataset.r_arm.ft_measured(1:3,:);
    m = dataset.r_arm.ft_measured(4:6,:);

    distance_f = vecnorm(diff(f')');
    distance_m = vecnorm(diff(m')');
    
    idx = find((distance_f > threshold_f) | (distance_m > threshold_m));
    
    if calibration
        start_index = 1;
        end_index = floor(length(idx)*percentage);
    else
        start_index = floor(length(idx) * (1-percentage)) + 1;
        end_index = idx(end);
    end
    
    indeces = start_index : end_index;

    for k = 1 : (end_index - start_index)
        waitbar(double(k)/double(length(indeces)), wbar, sprintf('Progress: %d %%', floor(double(k)/double(length(indeces))*100.0)));
        fn = prefices;
        for j=1:numel(fn)
            dataset_output.(fn{j}).ft_expected(:,k) = dataset.(fn{j}).ft_expected(:,indeces(k));
            dataset_output.(fn{j}).orientation_quat(:,k) = dataset.(fn{j}).orientation_quat(:,indeces(k));
            dataset_output.(fn{j}).ft_measured(:,k) = dataset.(fn{j}).ft_measured(:,indeces(k));
            dataset_output.(fn{j}).ang_vel(:,k) = dataset.(fn{j}).ang_vel(:,indeces(k));
            dataset_output.(fn{j}).lin_acc(:,k) = dataset.(fn{j}).lin_acc(:,indeces(k));
            dataset_output.(fn{j}).ft_temperature(:,k) = dataset.(fn{j}).ft_temperature(indeces(k));
        end
        dataset_output.joints(:,k) = dataset.joints(:,indeces(k));
    end

    close(wbar)
end