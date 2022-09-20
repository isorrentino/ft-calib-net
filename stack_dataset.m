clear all
clc
close all

calibration = true;

% ### Calibration

datasets = {{"robot_logger_device_2022_09_09_15_15_56.mat", 2.0916}, ...
            {"robot_logger_device_2022_09_09_15_20_10.mat", 2.0916}, ...
            {"robot_logger_device_2022_09_09_15_23_26.mat", 2.0916}, ...
            {"robot_logger_device_2022_09_09_15_26_31.mat", 2.0916}, ...
            {"robot_logger_device_2022_09_09_15_36_36.mat", 2.6795}, ...
            {"robot_logger_device_2022_09_09_15_48_31.mat", 2.6795}, ...
            {"robot_logger_device_2022_09_09_15_53_01.mat", 2.6795}, ...
            {"robot_logger_device_2022_09_09_15_56_22.mat", 2.6795}, ...
            {"robot_logger_device_2022_09_09_15_59_28.mat", 2.6795}, ...
            };


% ### Validation

% datasets = ["robot_logger_device_2022_09_09_14_29_47.mat", ...
%             "robot_logger_device_2022_09_09_14_59_33.mat", ...
%             "robot_logger_device_2022_09_09_15_15_56.mat", ...
%             "robot_logger_device_2022_09_09_15_59_28.mat"];


prefix = "./dataset_complete/different_weights/";
% prefix = "./dataset_complete/dataset_is/traj_ident_cut/";

dataset = struct([]);

for idx = 1 : size(datasets,2)
    d = datasets{idx}{1}
    weight = datasets{idx}{2}
    
%     if isempty(dataset)    

    dataset = create_dataset(strcat(prefix, d), weight, calibration);
    
%     else
%         temp_d  = create_dataset(strcat(prefix, d), weight, calibration);
%         fn = fieldnames(dataset);
%         for k=1:numel(fn)      
%             dataset.(fn{k}).orientation_quat = [dataset.(fn{k}).orientation_quat, temp_d.(fn{k}).orientation_quat];
%             dataset.(fn{k}).ft_expected = [dataset.(fn{k}).ft_expected, temp_d.(fn{k}).ft_expected];
%             dataset.(fn{k}).ft_measured = [dataset.(fn{k}).ft_measured, temp_d.(fn{k}).ft_measured];
%             dataset.(fn{k}).ang_vel = [dataset.(fn{k}).ang_vel, temp_d.(fn{k}).ang_vel];
%             dataset.(fn{k}).lin_acc = [dataset.(fn{k}).lin_acc, temp_d.(fn{k}).lin_acc];
%         end
%     end
    
    if calibration
        save(strcat("datasets/",d), "dataset", "-v7.3")
    else
        save("datasets/test_dataset.mat", "dataset", "-v7.3")
    end
end
