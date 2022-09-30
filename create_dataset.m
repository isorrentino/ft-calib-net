function [dataset] = create_dataset(filename, urdf)
    f = waitbar(0, 'Starting');
    robotName='iCubGenova09'; %% Name of the robot
    load(filename);

    %meshFilePrefix = [getenv('ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX') '/share']; %% Path to the model meshes
    %modelPath = [getenv('ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX') '/share/iCub/robots/' robotName '/'];  %% Path to the robot model
    modelPath = './urdf/';
    fileName = urdf; %% Name of the urdf file
    prefices = {'l_arm', 'r_arm'};
    joint_names = {'r_shoulder_pitch','r_shoulder_roll','r_shoulder_yaw','r_elbow','r_wrist_prosup'};
    %   ft_names_urdf = {'l_arm_ft_sensor';'r_arm_ft_sensor';'l_foot_front_ft_sensor'; ...
    %                    'l_foot_rear_ft_sensor';'r_foot_front_ft_sensor';'r_foot_rear_ft_sensor';'r_leg_ft_sensor'};
    contact_link = {'root_link'};
    gravityAcceleration = 9.80665;

    world_H_base = eye(4);
    KinDynModel = iDynTreeWrappers.loadReducedModel(robot_logger_device.description_list, 'root_link', modelPath, fileName, false);
    samples_temp(1) = robot_logger_device.joints_state.positions.dimensions(3);
    samples_temp(2) = robot_logger_device.accelerometers.r_arm_ft_acc.dimensions(3);
    samples_temp(3) = robot_logger_device.gyros.l_arm_ft_gyro.dimensions(3);
    samples_temp(4) = robot_logger_device.FTs.r_arm_ft_sensor.dimensions(3);
    samples_temp(5) = robot_logger_device.temperatures.r_arm_ft_sensor.dimensions(3);
    samples = min(samples_temp);

    dataset = struct();
    for p = prefices
        prefix = p{:};
        frame_name = [prefix, '_ft_sensor'];
        acc_name = [prefix, '_ft_acc'];
        gyro_name = [prefix, '_ft_gyro'];
        dataset.(prefix) = struct();
        dataset.(prefix).orientation_quat = [];
        dataset.(prefix).ang_vel = squeeze(robot_logger_device.gyros.(gyro_name).data(:,:,1:samples));
        dataset.(prefix).lin_acc = squeeze(robot_logger_device.accelerometers.(acc_name).data(:,:,1:samples));
        dataset.(prefix).ft_temperature = squeeze(robot_logger_device.temperatures.(frame_name).data(:,:,1:samples));
        dataset.(prefix).ft_measured = lowpass(squeeze(robot_logger_device.FTs.(frame_name).data(:,:,1:samples))', 0.5, 100)';
    end
    
    joints = squeeze(robot_logger_device.joints_state.positions.data(:,:,1:samples));
    dataset.joints = zeros(length(joint_names),size(joints,2));
    for i = 1 : length(joint_names)
        idx_in_description_list = find(contains(robot_logger_device.description_list,joint_names{i}));
        dataset.joints(i,:) = joints(idx_in_description_list,:);
    end

    consideredJoints = iDynTree.StringVector();
    for i = 1 : size(robot_logger_device.description_list,1)
        consideredJoints.push_back(robot_logger_device.description_list{i});
    end

    for p = prefices
        prefix = p{:};
        frame_name = [prefix, '_ft_sensor'];
        consideredJoints.push_back(frame_name);
    end

    estimator = iDynTree.ExtWrenchesAndJointTorquesEstimator();
    estimatorLoader = iDynTree.ModelLoader();
    estimatorLoader.loadReducedModelFromFile(strcat(modelPath, fileName), consideredJoints);
    estimator.setModelAndSensors(estimatorLoader.model(),estimatorLoader.sensors);

    dofs = length(robot_logger_device.description_list);
    q_idyn   = iDynTree.JointPosDoubleArray(dofs);
    dq_idyn  = iDynTree.JointDOFsDoubleArray(dofs);
    ddq_idyn = iDynTree.JointDOFsDoubleArray(dofs);
    % Prepare estimator variables
    % Store number of sensors
    n_fts = estimator.sensors().getNrOfSensors(iDynTree.SIX_AXIS_FORCE_TORQUE);

    % The estimated FT sensor measurements
    expFTmeasurements = iDynTree.SensorsMeasurements(estimator.sensors());

    % The estimated external wrenches
    estContactForces = iDynTree.LinkContactWrenches(estimator.model());

    % The estimated joint torques
    estJointTorques = iDynTree.JointDOFsDoubleArray(dofs);

    % Set the contact information in the estimator
    contact_index = estimator.model().getFrameIndex(char(contact_link));

    % Get trasform between inertial and wrist
%     rwrist_H_support = [0 0 1 -16.3e-3; -1 0 0 -41.9225e-3; 0 -1 0 9e-3; 0 0 0 1];
%     rw_H_support_idyn = iDynTree.Transform();
%     pos = iDynTree.Position();
%     pos.fromMatlab(rwrist_H_support(1:3, 4));
%     R = iDynTree.Rotation();
%     R.fromMatlab(rwrist_H_support(1:3, 1:3));
% 
%     rw_H_support_idyn.setPosition(pos);
%     rw_H_support_idyn.setRotation(R);

%     estimator.model().addAdditionalFrameToLink('r_wrist_1', 'r_external_support', rw_H_support_idyn);

%     ext_support_frame_idx = estimator.model().getFrameIndex('r_external_support');

    % Get the rotation between the inertial and the additional frame to
    % rotate the external wrench
%     ext_force_inertial = [0; 0; -additional_weight * gravityAcceleration];

    kinDyn = iDynTree.KinDynComputations();
    kinDyn.loadRobotModel(estimator.model());

    % Get ft names from urdf to know the corresponding order
    ft_names_from_urdf= cell(n_fts,1);
    for i = 1 : n_fts
        ft_names_from_urdf{i} = estimator.sensors().getSensor(iDynTree.SIX_AXIS_FORCE_TORQUE,i-1).getName();
    end

    grav_idyn = iDynTree.Vector3();
    grav_idyn.fromMatlab([0;0;-gravityAcceleration]);

    % Senso
    fullBodyUnknowns = iDynTree.LinkUnknownWrenchContacts(estimator.model());

    indeces = 1:samples; % I'm taking the dataset as it is

    for k = 1:length(indeces)
        i = indeces(k);
        waitbar(double(k)/double(length(indeces)), f, sprintf('Progress: %d %%', floor(double(k)/double(length(indeces))*100.0)));
        s = robot_logger_device.joints_state.positions.data(:,1,i);
        ds = robot_logger_device.joints_state.velocities.data(:,1,i);
        dds = 0 * ds;
        q_idyn.fromMatlab(s);
        dq_idyn.fromMatlab(ds);
        ddq_idyn.fromMatlab(dds);

        fullBodyUnknowns.clear();

        % Update kindyn
        kinDyn.setJointPos(q_idyn);

        % Get rotation matrix between additional frame and inertial frame
%         A_R_r_external_support = kinDyn.getWorldTransform('r_external_support').getRotation().toMatlab();
%         position_external_support = iDynTree.Position();
%         position_external_support.fromMatlab(kinDyn.getWorldTransform('r_external_support').getPosition().toMatlab());

        % Rotate external wrench from inertial (-mg) to the frame of the
        % support
%         ext_force_support = A_R_r_external_support' * ext_force_inertial;

%         rotated_ext_wrench_idyn = iDynTree.Wrench();
%         rotated_ext_wrench_idyn.fromMatlab([ext_force_support; 0; 0; 0]);

%         knownWrenchContact = iDynTree.UnknownWrenchContact();
%         knownWrenchContact.unknownType = iDynTree.NO_UNKNOWNS;
%         knownWrenchContact.contactPoint = position_external_support;
%         knownWrenchContact.knownWrench = rotated_ext_wrench_idyn;
% 
%         fullBodyUnknowns.addNewContactInFrame(estimator.model(), ext_support_frame_idx, knownWrenchContact);

        % Update robot kinematics
        estimator.updateKinematicsFromFixedBase(q_idyn,dq_idyn,ddq_idyn,contact_index,grav_idyn);

        % Specify unknown wrenches
        unknownWrench = iDynTree.UnknownWrenchContact();

        % Run the prediction of FT measurements
        % There are three output of the estimation, FT measurements, contact
        % forces and joint torques
        unknownWrench.unknownType = iDynTree.FULL_WRENCH;
        unknownWrench.contactPoint = iDynTree.Position.Zero();
        fullBodyUnknowns.addNewContactInFrame(estimator.model(),contact_index,unknownWrench);

        % wrench buffer
        estimatedSensorWrench = iDynTree.Wrench();
        estimatedSensorWrench.fromMatlab(zeros(1,6));

        % run the estimation
        estimator.computeExpectedFTSensorsMeasurements(fullBodyUnknowns,expFTmeasurements,estContactForces,estJointTorques);

        % store the estimated measurements
        iDynTreeWrappers.setRobotState(KinDynModel,s,ds,[0,0,-gravityAcceleration]);
        for j = 0:(n_fts-1)
            new_string=erase(ft_names_from_urdf{j+1}, '_ft_sensor');
            frame_name = ft_names_from_urdf{j+1};
            expFTmeasurements.getMeasurement(iDynTree.SIX_AXIS_FORCE_TORQUE, j, estimatedSensorWrench);
            dataset.(new_string).ft_expected(:,k) = estimatedSensorWrench.toMatlab();
            frameTransform = iDynTreeWrappers.getWorldTransform(KinDynModel, frame_name);
            dataset.(new_string).orientation_quat(:,k) = rotm2quat(frameTransform(1:3,1:3))';
        end
    end
    close(f)
end