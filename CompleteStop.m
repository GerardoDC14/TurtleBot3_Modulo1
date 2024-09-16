% ====================================================================
% MATLAB Script to Send Both Robots to a Complete Stop
% ====================================================================
% This script publishes zero velocities to both robots' /cmd_vel
% topics to ensure they come to a complete halt.
%
% Robots:
%   - Robot 1: Publishes to '/cmd_vel'
%   - Robot 2: Publishes to '/tb3_0/cmd_vel'
%
% Usage:
%   Run this script whenever you need to stop both robots.
% ====================================================================

% Clear workspace and figures
clear;
clc;
close all;

%% ROS 2 Initialization

% Initialize ROS 2 environment and domain ID
% Ensure that ROS 2 is properly installed and sourced on your system
setenv('ROS_DOMAIN_ID', '0');  % Adjust domain ID if necessary

% Create a ROS 2 node
try
    node = ros2node('/matlab_stop_node');
    disp('ROS 2 node created successfully.');
catch ME
    error('Failed to create ROS 2 node: %s', ME.message);
end

%% Create Publishers for Both Robots

% Publisher for Robot 1
try
    cmdVelPub1 = ros2publisher(node, '/cmd_vel', 'geometry_msgs/Twist');
    disp('Publisher for Robot 1 (/cmd_vel) created.');
catch ME
    error('Failed to create publisher for Robot 1: %s', ME.message);
end

% Publisher for Robot 2
try
    cmdVelPub2 = ros2publisher(node, '/tb3_0/cmd_vel', 'geometry_msgs/Twist');
    disp('Publisher for Robot 2 (/tb3_0/cmd_vel) created.');
catch ME
    error('Failed to create publisher for Robot 2: %s', ME.message);
end

%% Prepare Zero Velocity Messages

% Create Twist message for Robot 1
cmdVelMsg1 = ros2message(cmdVelPub1.MessageType);
cmdVelMsg1.Linear.X = 0.0;    % No linear movement
cmdVelMsg1.Linear.Y = 0.0;    % No lateral movement
cmdVelMsg1.Linear.Z = 0.0;    % No vertical movement
cmdVelMsg1.Angular.X = 0.0;   % No roll
cmdVelMsg1.Angular.Y = 0.0;   % No pitch
cmdVelMsg1.Angular.Z = 0.0;   % No yaw

% Create Twist message for Robot 2
cmdVelMsg2 = ros2message(cmdVelPub2.MessageType);
cmdVelMsg2.Linear.X = 0.0;    % No linear movement
cmdVelMsg2.Linear.Y = 0.0;    % No lateral movement
cmdVelMsg2.Linear.Z = 0.0;    % No vertical movement
cmdVelMsg2.Angular.X = 0.0;   % No roll
cmdVelMsg2.Angular.Y = 0.0;   % No pitch
cmdVelMsg2.Angular.Z = 0.0;   % No yaw

%% Publish Zero Velocity Commands

% Send zero velocity to Robot 1
try
    send(cmdVelPub1, cmdVelMsg1);
    disp('Zero velocity command sent to Robot 1 (/cmd_vel).');
catch ME
    warning('Failed to send command to Robot 1: %s', ME.message);
end

% Send zero velocity to Robot 2
try
    send(cmdVelPub2, cmdVelMsg2);
    disp('Zero velocity command sent to Robot 2 (/tb3_0/cmd_vel).');
catch ME
    warning('Failed to send command to Robot 2: %s', ME.message);
end

%% Optional: Verify Command Sending

% Pause briefly to ensure commands are sent
pause(1);

% Optionally, you can subscribe to /cmd_vel and /tb3_0/cmd_vel to verify
% that the commands have been received. This requires additional subscribers
% and is not included in this basic script.

%% Optional: Gracefully Shutdown the ROS 2 Node

% Uncomment the following lines if you want to shutdown the node after sending commands
% rosshutdown;
% disp('ROS 2 node shut down.');

% ====================================================================
% End of Script
% ====================================================================
