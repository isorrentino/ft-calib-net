% datasets = ["robot_logger_device_2022_07_22_18_52_23.mat",  ...
%     "robot_logger_device_2022_07_22_18_56_33.mat", ...
%     "robot_logger_device_2022_07_22_19_06_14.mat", ...
%     "robot_logger_device_2022_07_22_19_10_37.mat", ...
%     "robot_logger_device_2022_07_22_19_16_28.mat", ...
%     "robot_logger_device_2022_07_22_19_19_45.mat"];

clear all
clc
close all

datasets = ["robot_logger_device_2022_07_25_15_42_57.mat",  ...
    "robot_logger_device_2022_07_25_15_49_36.mat", ...
    "robot_logger_device_2022_07_25_15_53_28.mat", ...
    "robot_logger_device_2022_07_25_15_56_07.mat"];


% datasets = ["robot_logger_device_2022_07_25_18_32_17.mat"];

prefix = "./dataset_complete/dataset_is/calib_iner/";
% prefix = "./dataset_complete/dataset_is/traj_ident_cut/";

dataset = struct([]);
for d = datasets
    d
    if isempty(dataset)
        dataset = create_dataset(strcat(prefix, d));
    else
        temp_d  = create_dataset(strcat(prefix, d));
        fn = fieldnames(dataset);
        for k=1:numel(fn)      
            dataset.(fn{k}).orientation_quat = [dataset.(fn{k}).orientation_quat, temp_d.(fn{k}).orientation_quat];
            dataset.(fn{k}).ft_expected = [dataset.(fn{k}).ft_expected, temp_d.(fn{k}).ft_expected];
            dataset.(fn{k}).ft_measured = [dataset.(fn{k}).ft_measured, temp_d.(fn{k}).ft_measured];
            dataset.(fn{k}).ang_vel = [dataset.(fn{k}).ang_vel, temp_d.(fn{k}).ang_vel];
            dataset.(fn{k}).lin_acc = [dataset.(fn{k}).lin_acc, temp_d.(fn{k}).lin_acc];
        end
    end
end

save("datasets/calib_dataset.mat", "dataset", "-v7.3")