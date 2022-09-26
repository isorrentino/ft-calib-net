viz.close();

clear;
close all;
clc;

consideredJoints={
    'r_hip_pitch';
    'r_hip_roll';
    'r_hip_yaw';
    'r_knee';
    'r_ankle_pitch';
    'r_ankle_roll';
    'l_hip_pitch';
    'l_hip_roll';
    'l_hip_yaw';
    'l_knee';
    'l_ankle_pitch';
    'l_ankle_roll';
    'torso_pitch';
    'torso_roll';
    'torso_yaw';
    'r_shoulder_pitch';
    'r_shoulder_roll';
    'r_shoulder_yaw';
    'r_elbow';
    'r_wrist_prosup';
    'r_wrist_pitch';
    'r_wrist_yaw';
    'l_shoulder_pitch';
    'l_shoulder_roll';
    'l_shoulder_yaw';
    'l_elbow';
    'l_wrist_prosup';
    'l_wrist_pitch';
    'l_wrist_yaw';
    'neck_pitch';
    'neck_roll';
    'neck_yaw'
    };


% Example
modelPath = './';
fileName='iCubGenova09_r_arm_support_2_5kg.urdf';

KinDynModel = iDynTreeWrappers.loadReducedModel(consideredJoints,'root_link',modelPath,fileName,false);


% KinDynModel = iDynTreeWrappers.loadReducedModel(consideredJoints, 'root_link', '.', 'iCubGenova09_original.urdf', false);

viz = iDynTree.Visualizer();

viz.init();

viz.addModel(KinDynModel.kinDynComp.model(), 'iCub');

env = viz.enviroment();

env.setElementVisibility('floor_grid', true);

env.setElementVisibility('world_frame', true);

viz.camera().animator().enableMouseControl(true);

viz.enviroment().lightViz('sun1').setType(iDynTree.DIRECTIONAL_LIGHT);
viz.enviroment().lightViz('sun1').setDirection(iDynTree.Direction(-1, 0, 0));
viz.enviroment().addLight('sun2');
viz.enviroment().lightViz('sun2').setType(iDynTree.DIRECTIONAL_LIGHT);
viz.enviroment().lightViz('sun2').setDirection(iDynTree.Direction(1, 0, 0));

viz.frames().addFrame(iDynTree.Transform.Identity(), 0.2);

model = KinDynModel.kinDynComp.model();

support_frame_idx = KinDynModel.kinDynComp.getFrameIndex('r_support_weight_origin');
H = KinDynModel.kinDynComp.getWorldTransform(support_frame_idx);

disp(H.getPosition().toMatlab());
disp(H.getRotation().toMatlab());

viz.frames().addFrame(H, 0.2);


%viz.frames().addFrame(0);
running = true;
while running

    iDynTreeWrappers.setRobotState(KinDynModel, [0 -1 0 0; 1 0 0 0; 0 0 1 0; 0 0 0 1], zeros(length(consideredJoints),1), zeros(6,1), zeros(length(consideredJoints),1), [0 0 -9.81']);

    viz.run();

    viz.draw();
    
    pause(0.01);

end



