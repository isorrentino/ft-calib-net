clear all
clc
close all

calibration = false;
process_data = false;
stack_datasets = true;
dataset_percentage = 10/10; % if calibration is true it takes the first part otherwise it takes the last part

output_dataset_prefix = "2022_09_16";

% ### Calibration

% datasets = {{'robot_logger_device_2022_09_09_12_26_03.mat', 'iCubGenova09_r_arm_support.urdf'}, ...
%     {'robot_logger_device_2022_09_09_12_44_25.mat', 'iCubGenova09_r_arm_support.urdf'}, ...
%     {'robot_logger_device_2022_09_09_13_57_49.mat', 'iCubGenova09_r_arm_support_0_5kg.urdf'}, ...
%     {'robot_logger_device_2022_09_09_14_03_50.mat', 'iCubGenova09_r_arm_support_0_5kg.urdf'}, ...
%     {'robot_logger_device_2022_09_09_14_26_12.mat', 'iCubGenova09_r_arm_support_1kg.urdf'}, ...
%     {'robot_logger_device_2022_09_09_14_32_34.mat', 'iCubGenova09_r_arm_support_1kg.urdf'}, ...
%     {'robot_logger_device_2022_09_09_14_53_01.mat', 'iCubGenova09_r_arm_support_1_5kg.urdf'}, ...
%     {'robot_logger_device_2022_09_09_14_59_33.mat', 'iCubGenova09_r_arm_support_1_5kg.urdf'}, ...
%     {'robot_logger_device_2022_09_09_15_20_10.mat', 'iCubGenova09_r_arm_support_2kg.urdf'}, ...
%     {'robot_logger_device_2022_09_09_15_26_31.mat', 'iCubGenova09_r_arm_support_2kg.urdf'}, ...
%     {'robot_logger_device_2022_09_09_15_53_01.mat', 'iCubGenova09_r_arm_support_2_5kg.urdf'}, ...
%     {'robot_logger_device_2022_09_09_15_59_28.mat', 'iCubGenova09_r_arm_support_2_5kg.urdf'}, ...
%     };

% datasets = {{'urdf_robot_logger_device_2022_09_09_12_26_03.mat', 'iCubGenova09_r_arm_support.urdf'}, ...
%     {'urdf_robot_logger_device_2022_09_09_13_57_49.mat', 'iCubGenova09_r_arm_support_0_5kg.urdf'}, ...
%     {'urdf_robot_logger_device_2022_09_09_14_26_12.mat', 'iCubGenova09_r_arm_support_1kg.urdf'}, ...
%     {'urdf_robot_logger_device_2022_09_09_14_53_01.mat', 'iCubGenova09_r_arm_support_1_5kg.urdf'}, ...
%     {'urdf_robot_logger_device_2022_09_09_15_20_10.mat', 'iCubGenova09_r_arm_support_2kg.urdf'}, ...
%     {'urdf_robot_logger_device_2022_09_09_15_53_01.mat', 'iCubGenova09_r_arm_support_2_5kg.urdf'}, ...
%     };

% datasets = {{'urdf_robot_logger_device_2022_09_09_12_44_25.mat', 'iCubGenova09_r_arm_support.urdf'}, ...
%     {'urdf_robot_logger_device_2022_09_09_13_57_49.mat', 'iCubGenova09_r_arm_support_0_5kg.urdf'}, ...
%     {'urdf_robot_logger_device_2022_09_09_14_32_34.mat', 'iCubGenova09_r_arm_support_1kg.urdf'}, ...
%     {'urdf_robot_logger_device_2022_09_09_14_59_33.mat', 'iCubGenova09_r_arm_support_1_5kg.urdf'}, ...
%     {'urdf_robot_logger_device_2022_09_09_15_26_31.mat', 'iCubGenova09_r_arm_support_2kg.urdf'}, ...
%     {'urdf_robot_logger_device_2022_09_09_15_59_28.mat', 'iCubGenova09_r_arm_support_2_5kg.urdf'}, ...
%     };

% datasets = {
%     {'robot_logger_device_2022_09_16_14_33_36.mat', 'iCubGenova09_r_arm_support.urdf'}, ...
%     {'robot_logger_device_2022_09_16_14_41_45.mat', 'iCubGenova09_r_arm_support_0_5kg.urdf'}, ...
%     {'robot_logger_device_2022_09_16_14_48_14.mat', 'iCubGenova09_r_arm_support_1_5kg.urdf'}, ...
%     {'robot_logger_device_2022_09_16_14_54_32.mat', 'iCubGenova09_r_arm_support_1kg.urdf'}, ...
%     {'robot_logger_device_2022_09_16_14_59_47.mat', 'iCubGenova09_r_arm_support_2kg.urdf'}, ...
%     {'robot_logger_device_2022_09_16_15_06_31.mat', 'iCubGenova09_r_arm_support_2_5kg.urdf'}, ...
%     };

datasets = {{'urdf_robot_logger_device_2022_09_16_14_33_36.mat', 'iCubGenova09_r_arm_support.urdf'}, ...
    {'urdf_robot_logger_device_2022_09_16_14_41_45.mat', 'iCubGenova09_r_arm_support_0_5kg.urdf'}, ...
    {'urdf_robot_logger_device_2022_09_16_14_48_14.mat', 'iCubGenova09_r_arm_support_1_5kg.urdf'}, ...
    {'urdf_robot_logger_device_2022_09_16_14_54_32.mat', 'iCubGenova09_r_arm_support_1kg.urdf'}, ...
    {'urdf_robot_logger_device_2022_09_16_14_59_47.mat', 'iCubGenova09_r_arm_support_2kg.urdf'}, ...
    {'urdf_robot_logger_device_2022_09_16_15_06_31.mat', 'iCubGenova09_r_arm_support_2_5kg.urdf'}, ...
    };

% datasets = {{'robot_logger_device_2022_09_22_15_32_00.mat', 'iCubGenova09_r_arm_support.urdf'}, ...
%     {'robot_logger_device_2022_09_22_15_32_54.mat', 'iCubGenova09_r_arm_support.urdf'}, ...
%     {'robot_logger_device_2022_09_22_16_11_10.mat', 'iCubGenova09_r_arm_support_0_5kg.urdf'}, ...
%     {'robot_logger_device_2022_09_22_16_12_09.mat', 'iCubGenova09_r_arm_support_0_5kg.urdf'}, ...
%     {'robot_logger_device_2022_09_22_16_28_53.mat', 'iCubGenova09_r_arm_support_1kg.urdf'}, ...
%     {'robot_logger_device_2022_09_22_16_29_50.mat', 'iCubGenova09_r_arm_support_1kg.urdf'}, ...
%     {'robot_logger_device_2022_09_22_16_42_52.mat', 'iCubGenova09_r_arm_support_1_5kg.urdf'}, ...
%     {'robot_logger_device_2022_09_22_16_43_41.mat', 'iCubGenova09_r_arm_support_1_5kg.urdf'}, ...
%     {'robot_logger_device_2022_09_22_16_56_10.mat', 'iCubGenova09_r_arm_support_2kg.urdf'}, ...
%     {'robot_logger_device_2022_09_22_16_57_04.mat', 'iCubGenova09_r_arm_support_2kg.urdf'}, ...
%     {'robot_logger_device_2022_09_22_17_10_20.mat', 'iCubGenova09_r_arm_support_2_5kg.urdf'}, ...
%     {'robot_logger_device_2022_09_22_17_11_18.mat', 'iCubGenova09_r_arm_support_2_5kg.urdf'}, ...
%     {'robot_logger_device_2022_09_22_17_22_58.mat', 'iCubGenova09_r_arm_support_2_5kg.urdf'}, ...
%     {'robot_logger_device_2022_09_22_17_23_52.mat', 'iCubGenova09_r_arm_support_2_5kg.urdf'}, ...
%     {'robot_logger_device_2022_09_22_17_37_12.mat', 'iCubGenova09_r_arm_support_2kg.urdf'}, ...
%     {'robot_logger_device_2022_09_22_17_38_08.mat', 'iCubGenova09_r_arm_support_2kg.urdf'}, ...
%     {'robot_logger_device_2022_09_22_17_53_23.mat', 'iCubGenova09_r_arm_support_1_5kg.urdf'}, ...
%     {'robot_logger_device_2022_09_22_17_54_17.mat', 'iCubGenova09_r_arm_support_1_5kg.urdf'}, ...
%     {'robot_logger_device_2022_09_22_18_09_53.mat', 'iCubGenova09_r_arm_support_1kg.urdf'}, ...
%     {'robot_logger_device_2022_09_22_18_10_40.mat', 'iCubGenova09_r_arm_support_1kg.urdf'}, ...
%     {'robot_logger_device_2022_09_22_18_23_20.mat', 'iCubGenova09_r_arm_support_0_5kg.urdf'}, ...
%     {'robot_logger_device_2022_09_22_18_24_15.mat', 'iCubGenova09_r_arm_support_0_5kg.urdf'}, ...
%     {'robot_logger_device_2022_09_22_18_37_14.mat', 'iCubGenova09_r_arm_support.urdf'}, ...
%     {'robot_logger_device_2022_09_22_18_38_04.mat', 'iCubGenova09_r_arm_support.urdf'}, ...
%     };

% datasets = {{'urdf_robot_logger_device_2022_09_22_15_32_00.mat', 'iCubGenova09_r_arm_support.urdf'}, ...
%     {'urdf_robot_logger_device_2022_09_22_15_32_54.mat', 'iCubGenova09_r_arm_support.urdf'}, ...
%     {'urdf_robot_logger_device_2022_09_22_16_11_10.mat', 'iCubGenova09_r_arm_support_0_5kg.urdf'}, ...
%     {'urdf_robot_logger_device_2022_09_22_16_12_09.mat', 'iCubGenova09_r_arm_support_0_5kg.urdf'}, ...
%     {'urdf_robot_logger_device_2022_09_22_16_28_53.mat', 'iCubGenova09_r_arm_support_1kg.urdf'}, ...
%     {'urdf_robot_logger_device_2022_09_22_16_29_50.mat', 'iCubGenova09_r_arm_support_1kg.urdf'}, ...
%     {'urdf_robot_logger_device_2022_09_22_16_42_52.mat', 'iCubGenova09_r_arm_support_1_5kg.urdf'}, ...
%     {'urdf_robot_logger_device_2022_09_22_16_43_41.mat', 'iCubGenova09_r_arm_support_1_5kg.urdf'}, ...
%     {'urdf_robot_logger_device_2022_09_22_16_56_10.mat', 'iCubGenova09_r_arm_support_2kg.urdf'}, ...
%     {'urdf_robot_logger_device_2022_09_22_16_57_04.mat', 'iCubGenova09_r_arm_support_2kg.urdf'}, ...
%     {'urdf_robot_logger_device_2022_09_22_17_10_20.mat', 'iCubGenova09_r_arm_support_2_5kg.urdf'}, ...
%     {'urdf_robot_logger_device_2022_09_22_17_11_18.mat', 'iCubGenova09_r_arm_support_2_5kg.urdf'}, ...
%     {'urdf_robot_logger_device_2022_09_22_17_22_58.mat', 'iCubGenova09_r_arm_support_2_5kg.urdf'}, ...
%     {'urdf_robot_logger_device_2022_09_22_17_23_52.mat', 'iCubGenova09_r_arm_support_2_5kg.urdf'}, ...
%     {'urdf_robot_logger_device_2022_09_22_17_37_12.mat', 'iCubGenova09_r_arm_support_2kg.urdf'}, ...
%     {'urdf_robot_logger_device_2022_09_22_17_38_08.mat', 'iCubGenova09_r_arm_support_2kg.urdf'}, ...
%     {'urdf_robot_logger_device_2022_09_22_17_53_23.mat', 'iCubGenova09_r_arm_support_1_5kg.urdf'}, ...
%     {'urdf_robot_logger_device_2022_09_22_17_54_17.mat', 'iCubGenova09_r_arm_support_1_5kg.urdf'}, ...
%     {'urdf_robot_logger_device_2022_09_22_18_09_53.mat', 'iCubGenova09_r_arm_support_1kg.urdf'}, ...
%     {'urdf_robot_logger_device_2022_09_22_18_10_40.mat', 'iCubGenova09_r_arm_support_1kg.urdf'}, ...
%     {'urdf_robot_logger_device_2022_09_22_18_23_20.mat', 'iCubGenova09_r_arm_support_0_5kg.urdf'}, ...
%     {'urdf_robot_logger_device_2022_09_22_18_24_15.mat', 'iCubGenova09_r_arm_support_0_5kg.urdf'}, ...
%     {'urdf_robot_logger_device_2022_09_22_18_37_14.mat', 'iCubGenova09_r_arm_support.urdf'}, ...
%     {'urdf_robot_logger_device_2022_09_22_18_38_04.mat', 'iCubGenova09_r_arm_support.urdf'}, ...
%     };


% ### Validation

% datasets = ['robot_logger_device_2022_09_09_14_29_47.mat', ...
%             'robot_logger_device_2022_09_09_14_59_33.mat', ...
%             'robot_logger_device_2022_09_09_15_15_56.mat', ...
%             'robot_logger_device_2022_09_09_15_59_28.mat'];


% prefix = "./dataset_complete/con_pesi_per_right_ft_calib_16_09/";
prefix = "./datasets/";

dataset = struct([]);

for idx = 1 : size(datasets,2)
    d = datasets{idx}{1};
    urdf = datasets{idx}{2};
    
    disp(d);
    
    if isempty(dataset)
        if process_data
            dataset = create_dataset(strcat(prefix, d), urdf);
        else
            dataset = stack_dataset(strcat(prefix, d), dataset_percentage, calibration);
        end
    else
        if process_data
            temp_d  = create_dataset(strcat(prefix, d), urdf);
        else
            temp_d = stack_dataset(strcat(prefix, d), dataset_percentage, calibration);
        end
        if stack_datasets
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
        else
            dataset = temp_d;
        end
    end
    
    if stack_datasets == 0
        if calibration
            save(strcat('datasets/urdf_',d), "dataset", "-v7.3");
        else
            save(strcat("datasets/",output_dataset_prefix,"_test_dataset.mat"), "dataset", "-v7.3");
        end
    end
end

if stack_datasets
    if calibration
        save(strcat("datasets/",output_dataset_prefix,"_calib_dataset.mat"), "dataset", "-v7.3");
    else
        save(strcat("datasets/",output_dataset_prefix,"_test_dataset.mat"), "dataset", "-v7.3");
    end
end


