%% Example to use this scripts
% This file is usefull to understand the functionality about all object
% included on this folder

%% Add FrictionJoint to Path
path_friction_joint = '../FrictionJoint';
if exist(path_friction_joint, 'dir')
    addpath(path_friction_joint);
end
clear path_friction_joint;
%% Load object Motor
% Set all information about your robot.
% - Name of robot
% - folder where you want save the data
% - codyco-superbuild folder (try to put "getenv('CODYCO_SUPERBUILD_ROOT')" )
robot = Robot('iCubGenova01', '/Users/Raffaello/Documents/MATLAB/FrictionJoint/experiments', '/Users/Raffaello/iit/codyco-superbuild');
% Setup robot configuration:
% Variables:
% - worldRefFrame
% - robot_fixed
robot = robot.setReferenceFrame('root_link','true');

%% Add motors to test and load friction estimation
robot.joints = robot.getJoint('r_hip_pitch');
%robot.joints = [robot.joints robot.getJoint('l_hip_yaw')];
%robot.joints = [robot.joints robot.getJoint('l_hip_roll')];
%robot.joints = [robot.joints robot.getCoupledJoints('torso')];
robot = robot.loadData('idle');
robot = robot.loadData('ref');
robot.plotAndPrintAllData();

%% Configure your computer
% Set all variables:
% - Name of joint list with your list of joints
% - If true, automatic set yarp namespace
robot.configure('JOINT_FRICTION','false');
%robot.buildFolders();

%% Load information motor to control
motors = robot.getListMotor(robot.joints{1});
for i=1:length(motors)
    motors{i} = motors{i}.controlValue('Firmware');
end

frictionS = struct;
frictionS.length = length(motors);
frictionS.frictionComp = 0;
frictionS.ktau = zeros(frictionS.length);
frictionS.bemf = zeros(frictionS.length);
frictionS.stictionUp = zeros(frictionS.length);
frictionS.stictionDown = zeros(frictionS.length);

for i=1:length(motors)
    frictionS.ktau(i,i) = motors{i}.ktau*motors{i}.ratioTorque;
    frictionS.bemf(i,i) = motors{i}.bemf*motors{i}.ratioTorque;
    frictionS.stictionUp(i,i) = motors{i}.stictionUp*motors{i}.ratioTorque;
    frictionS.stictionDown(i,i) = motors{i}.stictionDown*motors{i}.ratioTorque;
end

% control.ktau
%% Control 
control = struct;
control.kp = 2;
control.ki = 0;

clear i;
%clear motors;
%% Configure your computer
% Set all variables:
% - Name of joint list with your list of joints
% - If true, automatic set yarp namespace
robot.configure('TEST_CONTROL','false');
%robot.buildFolders();

%% Reference
Reference = struct;
Reference.sinAmp = 10;
Reference.sinFreq = 0.15;
Reference.sinBias = 0;

Reference.sinAmp = Reference.sinAmp/(180/pi);
qDth = 0.1;
%% Open Simulink
open('simpleControl.slx');