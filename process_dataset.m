clear all
clc
close all

calibration = true;
process_data = false;
stack_datasets = true;
dataset_percentage = 9/10;

% ### Calibration

datasets = {{"robot_logger_device_2022_09_22_15_32_00.mat", 73.2e-3}, ...
            {"robot_logger_device_2022_09_22_15_32_54.mat", 73.2e-3}, ...
            {"robot_logger_device_2022_09_22_16_11_10.mat", 705e-3}, ...
            {"robot_logger_device_2022_09_22_16_12_09.mat", 705e-3}, ...
            {"robot_logger_device_2022_09_22_16_28_53.mat", 1077.1e-3}, ...
            {"robot_logger_device_2022_09_22_16_29_50.mat", 1077.1e-3}, ...
            {"robot_logger_device_2022_09_22_16_42_52.mat", 1665e-3}, ...
            {"robot_logger_device_2022_09_22_16_43_41.mat", 1665e-3}, ...
            {"robot_logger_device_2022_09_22_16_56_10.mat", 2091.6e-3}, ...
            {"robot_logger_device_2022_09_22_16_57_04.mat", 2091.6e-3}, ...
            {"robot_logger_device_2022_09_22_17_10_20.mat", 2679.5e-3}, ...
            {"robot_logger_device_2022_09_22_17_11_18.mat", 2679.5e-3}, ...
            {"robot_logger_device_2022_09_22_17_22_58.mat", 2679.5e-3}, ...
            {"robot_logger_device_2022_09_22_17_23_52.mat", 2679.5e-3}, ...
            {"robot_logger_device_2022_09_22_17_37_12.mat", 2091.6e-3}, ...
            {"robot_logger_device_2022_09_22_17_38_08.mat", 2091.6e-3}, ...
            {"robot_logger_device_2022_09_22_17_53_23.mat", 1665e-3}, ...
            {"robot_logger_device_2022_09_22_17_54_17.mat", 1665e-3}, ...
            {"robot_logger_device_2022_09_22_18_09_53.mat", 1077.1e-3}, ...
            {"robot_logger_device_2022_09_22_18_10_40.mat", 1077.1e-3}, ...
            {"robot_logger_device_2022_09_22_18_23_20.mat", 705e-3}, ...
            {"robot_logger_device_2022_09_22_18_24_15.mat", 705e-3}, ...
            {"robot_logger_device_2022_09_22_18_37_14.mat", 73.2e-3}, ...
            {"robot_logger_device_2022_09_22_18_38_04.mat", 73.2e-3}, ...
            };


% ### Validation

% datasets = ["robot_logger_device_2022_09_09_14_29_47.mat", ...
%             "robot_logger_device_2022_09_09_14_59_33.mat", ...
%             "robot_logger_device_2022_09_09_15_15_56.mat", ...
%             "robot_logger_device_2022_09_09_15_59_28.mat"];


prefix = "./datasets/";
% prefix = "./dataset_complete/dataset_is/traj_ident_cut/";

dataset = struct([]);

for idx = 1 : size(datasets,2)
    d = datasets{idx}{1};
    weight = datasets{idx}{2};
    
    disp(d);
    
    if isempty(dataset)
        if process_data
            dataset = create_dataset(strcat(prefix, d), weight);
        else
            dataset = stack_dataset(strcat(prefix, d), dataset_percentage);
        end
    else
        if process_data
            temp_d  = create_dataset(strcat(prefix, d), weight);
        else
            temp_d = stack_dataset(strcat(prefix, d), dataset_percentage);
        end
        fn = fieldnames(dataset);
        for k=1:numel(fn)
            if strcmp(fn{k},"joints") == 0
                dataset.(fn{k}).orientation_quat = [dataset.(fn{k}).orientation_quat, temp_d.(fn{k}).orientation_quat];
                dataset.(fn{k}).ft_expected = [dataset.(fn{k}).ft_expected, temp_d.(fn{k}).ft_expected];
                dataset.(fn{k}).ft_measured = [dataset.(fn{k}).ft_measured, temp_d.(fn{k}).ft_measured];
                dataset.(fn{k}).ang_vel = [dataset.(fn{k}).ang_vel, temp_d.(fn{k}).ang_vel];
                dataset.(fn{k}).lin_acc = [dataset.(fn{k}).lin_acc, temp_d.(fn{k}).lin_acc];
                dataset.(fn{k}).ft_temperature = [dataset.(fn{k}).ft_temperature, temp_d.(fn{k}).ft_temperature];
            end
        end
        dataset.joints = [dataset.joints, temp_d.joints];
    end
    
    if stack_datasets == 0
        if calibration
            save(strcat("datasets/",d), "dataset", "-v7.3");
        else
            save("datasets/test_dataset.mat", "dataset", "-v7.3");
        end
    end
end

if stack_datasets
    if calibration
        save(strcat("datasets/calib_dataset.mat"), "dataset", "-v7.3");
    else
        save("datasets/test_dataset.mat", "dataset", "-v7.3");
    end
end


