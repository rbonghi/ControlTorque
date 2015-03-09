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
robot = Robot('iCubGenova04', '/Users/Raffaello/Documents/MATLAB/FrictionJoint/experiments', '/Users/Raffaello/iit/codyco-superbuild');
% Setup robot configuration:
% Variables:
% - worldRefFrame
% - robot_fixed
robot = robot.setReferenceFrame('root_link','true');

%% Add motors to test and load friction estimation
robot.joints = robot.getJoint('l_knee');
%robot.joints = robot.getCoupledJoints('torso');
robot = robot.loadData('idle');
robot = robot.loadData('ref');
robot.plotAndPrintAllData();

%% Load information motor to control
motors = robot.getListMotor(robot.joints{1});
for i=1:length(motors)
    motors{i} = motors{i}.controlValue();
end

control = struct;
control.length = length(motors);
control.frictionComp = 0;
control.kff = zeros(control.length);
control.bemf = zeros(control.length);

for i=1:length(motors)
    control.kff(i,i) = motors{i}.kff;
    control.bemf(i,i) = motors{i}.bemf;
end
clear i;
clear motors;
%% Configure your computer
% Set all variables:
% - Name of joint list with your list of joints
% - If true, automatic set yarp namespace
robot.configure('TEST_CONTROL','false');
%robot.buildFolders();

%% Reference
name = 'ref';
Reference = struct;

if strcmp(name,'idle')
    Reference.sinAmp = 0;
    Reference.sinFreq = 0;
    Reference.sinBias = 0;
elseif strcmp(name,'ref')
    Reference.sinAmp = 2;
    Reference.sinFreq = 0.05;
    Reference.sinBias = 1;
end

qDth = 0.1;
clear name;
%% Open Simulink
open('ControlTorque.slx');