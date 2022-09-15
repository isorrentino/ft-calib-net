% datasets = ["robot_logger_device_2022_07_22_18_52_23.mat",  ...
%     "robot_logger_device_2022_07_22_18_56_33.mat", ...
%     "robot_logger_device_2022_07_22_19_06_14.mat", ...
%     "robot_logger_device_2022_07_22_19_10_37.mat", ...
%     "robot_logger_device_2022_07_22_19_16_28.mat", ...
%     "robot_logger_device_2022_07_22_19_19_45.mat"];

clear all
clc
close all

calibration = true;

datasets = ["robot_logger_device_2022_09_09_12_12_59.mat",  ...
    "robot_logger_device_2022_09_09_13_57_49.mat", ...
    "robot_logger_device_2022_09_09_14_32_34.mat", ...
    "robot_logger_device_2022_09_09_14_48_55.mat", ...
    "robot_logger_device_2022_09_09_15_26_31.mat", ...
    "robot_logger_device_2022_09_09_15_53_01.mat"];

% datasets = ["robot_logger_device_2022_09_09_14_21_28.mat", ...
%             "robot_logger_device_2022_09_09_15_26_31.mat", ...
%             "robot_logger_device_2022_09_09_15_56_22.mat"];

weights = [0, 0.705, 1.0771, 1.665, 2.0916, 2.6795];
% weights = [1.0771, 2.0916, 2.6795];

% datasets = ["robot_logger_device_2022_09_09_12_30_29.mat", ...
%             "robot_logger_device_2022_09_09_14_21_28.mat", ...
%             "robot_logger_device_2022_09_09_15_23_26.mat", ...
%             "robot_logger_device_2022_09_09_15_56_22.mat"];


% datasets = ["robot_logger_device_2022_07_25_18_32_17.mat"];

prefix = "./dataset_complete/different_weights/";
% prefix = "./dataset_complete/dataset_is/traj_ident_cut/";

idx = 1;
dataset = struct([]);

for d = datasets
    d
    if isempty(dataset)
        dataset = create_dataset(strcat(prefix, d), weights(idx), calibration);
    else
        temp_d  = create_dataset(strcat(prefix, d), weights(idx), calibration);
        fn = fieldnames(dataset);
        for k=1:numel(fn)      
            dataset.(fn{k}).orientation_quat = [dataset.(fn{k}).orientation_quat, temp_d.(fn{k}).orientation_quat];
            dataset.(fn{k}).ft_expected = [dataset.(fn{k}).ft_expected, temp_d.(fn{k}).ft_expected];
            dataset.(fn{k}).ft_measured = [dataset.(fn{k}).ft_measured, temp_d.(fn{k}).ft_measured];
            dataset.(fn{k}).ang_vel = [dataset.(fn{k}).ang_vel, temp_d.(fn{k}).ang_vel];
            dataset.(fn{k}).lin_acc = [dataset.(fn{k}).lin_acc, temp_d.(fn{k}).lin_acc];
        end
    end
    idx = idx + 1;
end

if calibration
    save("datasets/calib_dataset.mat", "dataset", "-v7.3")
else
    save("datasets/test_dataset.mat", "dataset", "-v7.3")
end
