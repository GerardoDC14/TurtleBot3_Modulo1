% Clear workspace and figures
clear all;
clc;
close all;

%% ROS 2 Initialization

% Initialize ROS 2 environment and domain ID
setenv('ROS_DOMAIN_ID', '0');  % Adjust domain ID if necessary

% Create a ROS 2 node
node = ros2node('/matlab_control_node');

% Create subscribers for the Robot
xSub = ros2subscriber(node, '/aruco_x', 'std_msgs/Float32');
ySub = ros2subscriber(node, '/aruco_y', 'std_msgs/Float32');
yawSub = ros2subscriber(node, '/aruco_yaw', 'std_msgs/Float32');

% Create publisher for velocity command
cmdVelPub = ros2publisher(node, '/cmd_vel', 'geometry_msgs/Twist');

% Create Twist message for the robot
cmdVelMsg = ros2message('geometry_msgs/Twist');

%% Simulation Parameters
dt = 0.1;  % Time step [s] adjusted to match control frequency
leader_speed = 0.01; %Adjust leader speed as necessary

%% Initialize Robot's State (positions will be updated from ROS topics)
robot = struct('x', 0, 'y', 0, 'theta', 0);

%% Wait for Robot Data Before Trajectory Drawing

% Wait until we receive valid robot data
disp('Waiting for robot data to initialize...');
while true
    xMsg = xSub.LatestMessage;
    yMsg = ySub.LatestMessage;
    yawMsg = yawSub.LatestMessage;

    if ~isempty(xMsg) && ~isempty(yMsg) && ~isempty(yawMsg)
        robot.x = double(xMsg.data);
        robot.y = double(yMsg.data);
        robot.theta = double(yawMsg.data);
        break;
    end
    pause(0.1); % Wait briefly before checking again
end
disp('Robot data received.');

%% Define Boundary Based on Given Corners

% Given corner points
boundary_x = [-0.2, 0.1, 1.30, 0.81];
boundary_y = [-0.4, -0.4, -0.72, -0.76];

% Close the polygon by adding the first point at the end
boundary_x = [boundary_x, boundary_x(1)];
boundary_y = [boundary_y, boundary_y(1)];

% Create a polyshape object for the boundary
boundary_poly = polyshape(boundary_x, boundary_y);

% Compute axis limits based on the boundary with some margin
x_margin = 0.1 * (max(boundary_x) - min(boundary_x));
y_margin = 0.1 * (max(boundary_y) - min(boundary_y));
xmin_plot = min(boundary_x) - x_margin;
xmax_plot = max(boundary_x) + x_margin;
ymin_plot = min(boundary_y) - y_margin;
ymax_plot = max(boundary_y) + y_margin;

%% Trajectory Drawing

disp('Draw the trajectory you want the robot to follow within the boundary.');
disp('Left-click and hold to begin drawing a freehand path.');
disp('Simply release the mouse button to finish.');

figure_handle = figure;
axis([xmin_plot xmax_plot ymin_plot ymax_plot]);  % Set axis limits based on boundary
xlabel('X [m]');
ylabel('Y [m]');
title('Draw the Desired Trajectory');
hold on;
grid on;
axis equal;

% Plot the boundary
plot(boundary_poly, 'FaceColor', 'none', 'EdgeColor', 'k', 'LineStyle', '--');

% Plot the robot's current position
plot(robot.x, robot.y, 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
text(robot.x + 0.05, robot.y, 'Robot Start', 'Color', 'b');

% Use drawfreehand to draw the trajectory
hFH = drawfreehand('Color','k','LineWidth',2,'Closed',false);

% Wait until the drawing is complete
wait(hFH);

% Get the xy coordinates of the trajectory
xy = hFH.Position;
x_traj = xy(:,1);
y_traj = xy(:,2);

% Close the figure
close(figure_handle);

% Check if the trajectory has enough points
if length(x_traj) < 2
    error('The drawn trajectory must contain at least two points. Please redraw the trajectory.');
end

% Ensure x_traj and y_traj are column vectors
x_traj = x_traj(:);
y_traj = y_traj(:);

% Check if the trajectory is within the boundary
[in_boundary, ~] = isinterior(boundary_poly, x_traj, y_traj);
if ~all(in_boundary)
    error('The drawn trajectory extends outside the boundary. Please redraw the trajectory within the boundary.');
end

% Smooth the trajectory to remove sharp turns
window_size = 5;
x_traj_smooth = smoothdata(x_traj, 'movmean', window_size);
y_traj_smooth = smoothdata(y_traj, 'movmean', window_size);

% Interpolate the trajectory to get evenly spaced points
distance = sqrt(diff(x_traj_smooth).^2 + diff(y_traj_smooth).^2);
cum_distance = [0; cumsum(distance)];
total_length = cum_distance(end);

% Create an evenly spaced distance vector
desired_spacing = 0.01; % Spacing between points in meters
num_points = max(round(total_length / desired_spacing), 2); % Ensure at least 2 points
even_distance = linspace(0, total_length, num_points)';

% Interpolate x and y over the even_distance vector
x_traj_interp = interp1(cum_distance, x_traj_smooth, even_distance);
y_traj_interp = interp1(cum_distance, y_traj_smooth, even_distance);

% Recompute the cumulative distance for interpolated trajectory
distance_interp = sqrt(diff(x_traj_interp).^2 + diff(y_traj_interp).^2);
cum_distance_interp = [0; cumsum(distance_interp)];
total_length_interp = cum_distance_interp(end);

%% Control Parameters

% Control gains (adjust as necessary)
Kp_linear = 0.8; Ki_linear = 0.0; Kd_linear = 0.1;
Kp_angular = 1.0; Ki_angular = 0.0; Kd_angular = 0.1;

% Velocity limits (adjust according to your robot)
max_linear_velocity = 0.07;   % [m/s] TurtleBot3 max linear velocity
max_angular_velocity = 0.8;  % [rad/s] TurtleBot3 max angular velocity

%% Import Robot Model

robot_model = importrobot('robotisTurtleBot3Burger.urdf'); % Use the correct URDF file name
robot_model.DataFormat = 'row';

%% Initialize Leader State

leader = struct('x', x_traj_interp(1), 'y', y_traj_interp(1), 'theta', 0);

% Initialize PID errors
[error_int, error_prev] = init_pid_errors();

% Leader start control
leader_started = false; % Flag to indicate if the leader has started moving
start_threshold_distance = 0.05; % Threshold distance for the robot to reach leader

%% Control Loop Setup

% Control loop frequency
control_frequency = 10; % [Hz]
dt = 1 / control_frequency; % Time step [s] adjusted to match control frequency
rate = rateControl(control_frequency);

% Initialize variables for plotting
robot_history = [];
leader_history = [];
leader_theta_history = [];

%% Set up figure for real-time plotting

figure_handle = figure;
set(figure_handle, 'Position', [100, 100, 800, 600]);
ax = axes('Parent', figure_handle);
hold(ax, 'on');
grid(ax, 'on');
axis(ax, 'equal');
xlim(ax, [xmin_plot, xmax_plot]);
ylim(ax, [ymin_plot, ymax_plot]);
xlabel(ax, 'X [m]');
ylabel(ax, 'Y [m]');
title(ax, 'Robot Following Drawn Trajectory');

% Plot boundary
plot(ax, boundary_poly, 'FaceColor', 'none', 'EdgeColor', 'k', 'LineStyle', '--');

% Plot the drawn trajectory
plot_traj = plot(ax, x_traj_smooth, y_traj_smooth, 'k--', 'LineWidth', 1);

% Initialize plots for leader and robot's trajectories
plot_leader_traj = plot(ax, nan, nan, 'k-', 'LineWidth', 2);
plot_robot_traj = plot(ax, nan, nan, 'b-', 'LineWidth', 2);

% Create hgtransform object for the robot
hg_robot = hgtransform('Parent', ax);

% Plot robot using robot model under the axes
show(robot_model, 'Parent', ax, 'Frames', 'off', 'PreservePlot', true, 'Visuals', 'on');
% Find the robot graphics and set their parent to hg_robot
robot_patches_all = findobj(ax, 'Type', 'Patch');
set(robot_patches_all, 'Parent', hg_robot);

legend(ax, [plot_traj, plot_leader_traj, plot_robot_traj], ...
    {'Drawn Trajectory', 'Leader Trajectory', 'Robot Trajectory'});

drawnow;

%% Main Control Loop

leader_distance = 0;
max_time = total_length_interp / leader_speed + 60; % Maximum time for simulation [s]
elapsed_time = 0;

while elapsed_time < max_time
    tic; % Start timer for this iteration

    %% Receive Position from ROS 2 Topics
    % Robot
    xMsg = xSub.LatestMessage;
    yMsg = ySub.LatestMessage;
    yawMsg = yawSub.LatestMessage;

    if isempty(xMsg) || isempty(yMsg) || isempty(yawMsg)
        disp('Waiting for Robot data...');
        pause(0.1);
        continue;
    end

    robot.x = double(xMsg.data);
    robot.y = double(yMsg.data);
    robot.theta = double(yawMsg.data);

    %% Leader Movement Control

    if ~leader_started
        % Compute distance between robot and leader
        distance_to_leader = sqrt((robot.x - leader.x)^2 + (robot.y - leader.y)^2);
        if distance_to_leader < start_threshold_distance
            leader_started = true;
            disp('Leader is starting to move.');
        else
            % Leader stays at initial position
            disp('Waiting for robot to reach leader.');
        end
    end

    if leader_started && leader_distance < total_length_interp
        leader_distance = leader_distance + leader_speed * dt;
        if leader_distance > total_length_interp
            leader_distance = total_length_interp;
        end

        % Get leader's position by interpolating x and y at leader_distance
        leader.x = interp1(cum_distance_interp, x_traj_interp, leader_distance);
        leader.y = interp1(cum_distance_interp, y_traj_interp, leader_distance);
    else
        % Leader remains at last position
        % leader.x and leader.y are already set
    end

    % Compute leader's orientation based on the trajectory direction
    if leader_distance < total_length_interp
        leader_dx = interp1(cum_distance_interp(1:end-1), diff(x_traj_interp)./diff(cum_distance_interp), leader_distance, 'linear', 'extrap');
        leader_dy = interp1(cum_distance_interp(1:end-1), diff(y_traj_interp)./diff(cum_distance_interp), leader_distance, 'linear', 'extrap');
        leader.theta = atan2(leader_dy, leader_dx);
    end

    % Record leader's position and orientation
    leader_history = [leader_history; leader.x, leader.y];
    leader_theta_history = [leader_theta_history; leader.theta];

    %% Desired Position for Robot
    % The robot should follow the leader directly
    desired_x = leader.x;
    desired_y = leader.y;
    desired_theta = leader.theta;

    %% Apply Control Algorithm
    % Control for Robot
    [cmd_vel, error_int, error_prev] = compute_control(robot, desired_x, desired_y, desired_theta, ...
        Kp_linear, Ki_linear, Kd_linear, Kp_angular, Ki_angular, Kd_angular, ...
        max_linear_velocity, max_angular_velocity, dt, ...
        error_int, error_prev);

    %% Boundary Checks
    % Check if robot is within the boundary polygon
    if ~isinterior(boundary_poly, robot.x, robot.y)
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
        disp('Robot is out of bounds!');
    end

    %% Publish Velocity Command
    % Robot
    cmdVelMsg.linear.x = cmd_vel.linear.x;
    cmdVelMsg.angular.z = cmd_vel.angular.z;
    send(cmdVelPub, cmdVelMsg);

    %% Record Positions for Plotting
    robot_history = [robot_history; robot.x, robot.y];

    %% Update Plots
    % Update leader trajectory plot
    set(plot_leader_traj, 'XData', leader_history(:,1), 'YData', leader_history(:,2));

    % Update robot's trajectory
    set(plot_robot_traj, 'XData', robot_history(:,1), 'YData', robot_history(:,2));

    % Update robot's transform
    robot_tf = makehgtform('translate', [robot.x, robot.y, 0], 'zrotate', robot.theta);
    set(hg_robot, 'Matrix', robot_tf);

    drawnow;

    %% Check for Completion
    % If the robot is close to the end of the trajectory, stop the loop
    distance_to_goal = sqrt((robot.x - x_traj_interp(end))^2 + (robot.y - y_traj_interp(end))^2);
    if distance_to_goal < 0.05 && leader_distance >= total_length_interp
        disp('Robot has reached the end of the trajectory.');
        break;
    end

    %% Wait for Next Control Loop
    elapsed_time = elapsed_time + toc;
    waitfor(rate);
end

%% Stop the Robot
% Robot
cmdVelMsg.linear.x = 0;
cmdVelMsg.angular.z = 0;
send(cmdVelPub, cmdVelMsg);

disp('Control loop finished. Robot stopped.');

%% Supporting Functions

% Initialize PID errors
function [error_int, error_prev] = init_pid_errors()
    error_int = struct('linear', 0, 'angular', 0);
    error_prev = struct('linear', 0, 'angular', 0);
end

% Compute control command
function [cmd_vel, error_int, error_prev] = compute_control(robot, desired_x, desired_y, desired_theta, ...
    Kp_linear, Ki_linear, Kd_linear, Kp_angular, Ki_angular, Kd_angular, ...
    max_linear_velocity, max_angular_velocity, dt, ...
    error_int, error_prev)

    % Calculate position errors
    error_x = desired_x - robot.x;
    error_y = desired_y - robot.y;
    distance_error = sqrt(error_x^2 + error_y^2);

    angle_to_goal = atan2(error_y, error_x);
    angle_error = angle_to_goal - robot.theta;
    angle_error = atan2(sin(angle_error), cos(angle_error));  % Normalize

    % PID control for linear velocity
    [v, error_int.linear, error_prev.linear] = pid_control(distance_error, error_int.linear, error_prev.linear, ...
        Kp_linear, Ki_linear, Kd_linear, dt, max_linear_velocity);

    % PID control for angular velocity
    [omega, error_int.angular, error_prev.angular] = pid_control(angle_error, error_int.angular, error_prev.angular, ...
        Kp_angular, Ki_angular, Kd_angular, dt, max_angular_velocity);

    % Command velocities
    cmd_vel.linear.x = v;
    cmd_vel.angular.z = omega;
end

% PID control function
function [output, error_int, error_prev] = pid_control(error, error_int, error_prev, Kp, Ki, Kd, dt, max_output)
    derivative = (error - error_prev) / dt;
    output_unsat = Kp * error + Ki * error_int + Kd * derivative;

    % Saturate output
    output = max(min(output_unsat, max_output), -max_output);

    % Anti-windup
    if abs(output_unsat) < max_output
        error_int = error_int + error * dt;
    end

    error_prev = error;
end
