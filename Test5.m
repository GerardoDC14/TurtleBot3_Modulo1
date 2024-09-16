% Clear workspace and figures
clear all;
clc;
close all;

%% ROS 2 Initialization

% Initialize ROS 2 environment and domain ID
setenv('ROS_DOMAIN_ID', '0');  % Adjust domain ID if necessary

% Create a ROS 2 node
node = ros2node('/matlab_control_node');

% Create subscribers for Robot 1 camera data
xSub1 = ros2subscriber(node, '/aruco_x', 'std_msgs/Float32');
ySub1 = ros2subscriber(node, '/aruco_y', 'std_msgs/Float32');
yawSub1 = ros2subscriber(node, '/aruco_yaw', 'std_msgs/Float32');

% Create subscribers for Robot 2 camera data
xSub2 = ros2subscriber(node, '/aruco1_x', 'std_msgs/Float32');
ySub2 = ros2subscriber(node, '/aruco1_y', 'std_msgs/Float32');
yawSub2 = ros2subscriber(node, '/aruco1_yaw', 'std_msgs/Float32');

% Create subscribers for Robot 1 odometry data
odomSub1 = ros2subscriber(node, '/odom', 'nav_msgs/Odometry');

% Create subscribers for Robot 2 odometry data
odomSub2 = ros2subscriber(node, '/tb3_0/odom', 'nav_msgs/Odometry');

% Create publishers for velocity commands
cmdVelPub1 = ros2publisher(node, '/cmd_vel', 'geometry_msgs/Twist');  % Robot 1
cmdVelPub2 = ros2publisher(node, '/tb3_0/cmd_vel', 'geometry_msgs/Twist');  % Robot 2

% Create Twist messages for the robots
cmdVelMsg1 = ros2message('geometry_msgs/Twist');  % Robot 1
cmdVelMsg2 = ros2message('geometry_msgs/Twist');  % Robot 2

%% Simulation Parameters
dt = 0.1;  % Time step [s] adjusted to match control frequency
leader_speed = 0.01; % Adjust leader speed as necessary

%% Initialize Robots' States (positions will be updated from ROS topics)
robot1 = struct('x', 0, 'y', 0, 'theta', 0);
robot2 = struct('x', 0, 'y', 0, 'theta', 0);

%% Wait for Robot Data Before Trajectory Drawing

% Wait until we receive valid robot data
disp('Waiting for robot data to initialize...');
while true
    % Robot 1 camera data
    xMsg1 = xSub1.LatestMessage;
    yMsg1 = ySub1.LatestMessage;
    yawMsg1 = yawSub1.LatestMessage;
    
    % Robot 2 camera data
    xMsg2 = xSub2.LatestMessage;
    yMsg2 = ySub2.LatestMessage;
    yawMsg2 = yawSub2.LatestMessage;
    
    % Robot 1 odometry data
    odomMsg1 = odomSub1.LatestMessage;
    
    % Robot 2 odometry data
    odomMsg2 = odomSub2.LatestMessage;
    
    if (~isempty(xMsg1) && ~isempty(yMsg1) && ~isempty(yawMsg1) && ...
        ~isempty(xMsg2) && ~isempty(yMsg2) && ~isempty(yawMsg2)) || ...
       (~isempty(odomMsg1) && ~isempty(odomMsg2))
       
        % Use camera data if available
        if ~isempty(xMsg1) && ~isempty(yMsg1) && ~isempty(yawMsg1)
            robot1.x = double(xMsg1.data);
            robot1.y = double(yMsg1.data);
            robot1.theta = double(yawMsg1.data);
        elseif ~isempty(odomMsg1)
            % Extract odometry data for Robot 1
            robot1 = update_robot_state_from_odom(odomMsg1);
        end
        
        if ~isempty(xMsg2) && ~isempty(yMsg2) && ~isempty(yawMsg2)
            robot2.x = double(xMsg2.data);
            robot2.y = double(yMsg2.data);
            robot2.theta = double(yawMsg2.data);
        elseif ~isempty(odomMsg2)
            % Extract odometry data for Robot 2
            robot2 = update_robot_state_from_odom(odomMsg2);
        end
        break;
    end
    pause(0.1); % Wait briefly before checking again
end
disp('Robot data received.');

%% Define Boundary Based on Camera's Field of View (Optional)

% If you still want to visualize the camera's playground, you can keep the boundary plotting.
% However, we will not stop the robots when they go out of bounds.

% Given corner points (Camera's field of view)
boundary_x = [-0.2, 0.1, 1.30, 0.81];
boundary_y = [-0.4, -0.4, -0.72, -0.76];

% Close the polygon by adding the first point at the end
boundary_x = [boundary_x, boundary_x(1)];
boundary_y = [boundary_y, boundary_y(1)];

% Create a polyshape object for the boundary
boundary_poly = polyshape(boundary_x, boundary_y);

% Compute axis limits based on the boundary with some margin
x_margin = 0.5 * (max(boundary_x) - min(boundary_x));
y_margin = 0.5 * (max(boundary_y) - min(boundary_y));
xmin_plot = min(boundary_x) - x_margin;
xmax_plot = max(boundary_x) + x_margin;
ymin_plot = min(boundary_y) - y_margin;
ymax_plot = max(boundary_y) + y_margin;

%% Trajectory Drawing

disp('Draw the trajectory you want the robots to follow.');
disp('Left-click and hold to begin drawing a freehand path.');
disp('Simply release the mouse button to finish.');

figure_handle = figure;
axis([xmin_plot xmax_plot ymin_plot ymax_plot]);  % Set axis limits
xlabel('X [m]');
ylabel('Y [m]');
title('Dibujar trayectoria');
hold on;
grid on;
axis equal;

% Plot the boundary (optional)
plot(boundary_poly, 'FaceColor', 'none', 'EdgeColor', 'k', 'LineStyle', '--');

% Plot the robots' current positions
plot(robot1.x, robot1.y, 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
text(robot1.x + 0.05, robot1.y, 'Inicio Robot ID 0', 'Color', 'b');

plot(robot2.x, robot2.y, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
text(robot2.x + 0.05, robot2.y, 'Inicio Robot ID 1', 'Color', 'r');

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

% Velocity limits (adjust according to your robots)
max_linear_velocity = 0.02;   % [m/s] TurtleBot3 max linear velocity
max_angular_velocity = 0.6;  % [rad/s] TurtleBot3 max angular velocity

%% Initialize Leader State

leader = struct('x', x_traj_interp(1), 'y', y_traj_interp(1), 'theta', 0);

% Initialize PID errors for both robots
[error_int1, error_prev1] = init_pid_errors();
[error_int2, error_prev2] = init_pid_errors();

% Leader start control
leader_started = false; % Flag to indicate if the leader has started moving
start_threshold_distance = 0.05; % Threshold distance for the robots to reach leader

%% Control Loop Setup

% Control loop frequency
control_frequency = 10; % [Hz]
dt = 1 / control_frequency; % Time step [s] adjusted to match control frequency
rate = rateControl(control_frequency);

% Initialize variables for plotting
robot_history1 = [];
robot_history2 = [];
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
title(ax, 'Consenso');

% Plot boundary (optional)
plot(ax, boundary_poly, 'FaceColor', 'none', 'EdgeColor', 'k', 'LineStyle', '--');

% Plot the drawn trajectory
plot_traj = plot(ax, x_traj_smooth, y_traj_smooth, 'k--', 'LineWidth', 1);

% Initialize plots for leader and robots' trajectories
plot_leader_traj = plot(ax, nan, nan, 'k-', 'LineWidth', 2);
plot_robot1_traj = plot(ax, nan, nan, 'b-', 'LineWidth', 2);
plot_robot2_traj = plot(ax, nan, nan, 'r-', 'LineWidth', 2);

% Create markers for robots
robot1_marker = plot(ax, robot1.x, robot1.y, 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
robot2_marker = plot(ax, robot2.x, robot2.y, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

legend(ax, [plot_traj, plot_leader_traj, plot_robot1_traj, plot_robot2_traj], ...
    {'Trayectoria dibujada', 'Trayectoria Lider', 'Trayectoria Robot ID 0', 'Trayectoria Robot ID 1 '});

drawnow;

%% Main Control Loop

leader_distance = 0;
max_time = total_length_interp / leader_speed + 60; % Maximum time for simulation [s]
elapsed_time = 0;

while elapsed_time < max_time
    tic; % Start timer for this iteration

    %% Receive Position from ROS 2 Topics
    % Robot 1 camera data
    xMsg1 = xSub1.LatestMessage;
    yMsg1 = ySub1.LatestMessage;
    yawMsg1 = yawSub1.LatestMessage;
    
    % Robot 2 camera data
    xMsg2 = xSub2.LatestMessage;
    yMsg2 = ySub2.LatestMessage;
    yawMsg2 = yawSub2.LatestMessage;
    
    % Robot 1 odometry data
    odomMsg1 = odomSub1.LatestMessage;
    
    % Robot 2 odometry data
    odomMsg2 = odomSub2.LatestMessage;
    
    % Update Robot 1's state
    if ~isempty(xMsg1) && ~isempty(yMsg1) && ~isempty(yawMsg1)
        % Use camera data
        robot1.x = double(xMsg1.data);
        robot1.y = double(yMsg1.data);
        robot1.theta = double(yawMsg1.data);
    elseif ~isempty(odomMsg1)
        % Use odometry data
        robot1 = update_robot_state_from_odom(odomMsg1);
    else
        disp('Waiting for Robot 1 data...');
    end
    
    % Update Robot 2's state
    if ~isempty(xMsg2) && ~isempty(yMsg2) && ~isempty(yawMsg2)
        % Use camera data
        robot2.x = double(xMsg2.data);
        robot2.y = double(yMsg2.data);
        robot2.theta = double(yawMsg2.data);
    elseif ~isempty(odomMsg2)
        % Use odometry data
        robot2 = update_robot_state_from_odom(odomMsg2);
    else
        disp('Waiting for Robot 2 data...');
    end
    
    %% Leader Movement Control

    if ~leader_started
        % Compute distance between robots and leader
        distance_to_leader1 = sqrt((robot1.x - leader.x)^2 + (robot1.y - leader.y)^2);
        distance_to_leader2 = sqrt((robot2.x - leader.x)^2 + (robot2.y - leader.y)^2);

        if distance_to_leader1 < start_threshold_distance && distance_to_leader2 < start_threshold_distance
            leader_started = true;
            disp('Leader is starting to move.');
        else
            % Leader stays at initial position
            disp('Waiting for robots to reach leader.');
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

    %% Desired Position for Robots
    % Both robots should follow the leader directly
    desired_x = leader.x;
    desired_y = leader.y;
    desired_theta = leader.theta;

    %% Apply Control Algorithm
    % Control for Robot 1
    [cmd_vel1, error_int1, error_prev1] = compute_control(robot1, desired_x, desired_y, desired_theta, ...
        Kp_linear, Ki_linear, Kd_linear, Kp_angular, Ki_angular, Kd_angular, ...
        max_linear_velocity, max_angular_velocity, dt, ...
        error_int1, error_prev1);

    % Control for Robot 2
    [cmd_vel2, error_int2, error_prev2] = compute_control(robot2, desired_x, desired_y, desired_theta, ...
        Kp_linear, Ki_linear, Kd_linear, Kp_angular, Ki_angular, Kd_angular, ...
        max_linear_velocity, max_angular_velocity, dt, ...
        error_int2, error_prev2);

    %% Publish Velocity Commands
    % Robot 1
    cmdVelMsg1.linear.x = cmd_vel1.linear.x;
    cmdVelMsg1.angular.z = cmd_vel1.angular.z;
    send(cmdVelPub1, cmdVelMsg1);

    % Robot 2
    cmdVelMsg2.linear.x = cmd_vel2.linear.x;
    cmdVelMsg2.angular.z = cmd_vel2.angular.z;
    send(cmdVelPub2, cmdVelMsg2);

    %% Record Positions for Plotting
    robot_history1 = [robot_history1; robot1.x, robot1.y];
    robot_history2 = [robot_history2; robot2.x, robot2.y];

    %% Update Plots
    % Update leader trajectory plot
    set(plot_leader_traj, 'XData', leader_history(:,1), 'YData', leader_history(:,2));

    % Update robot trajectories
    set(plot_robot1_traj, 'XData', robot_history1(:,1), 'YData', robot_history1(:,2));
    set(plot_robot2_traj, 'XData', robot_history2(:,1), 'YData', robot_history2(:,2));

    % Update robot markers
    set(robot1_marker, 'XData', robot1.x, 'YData', robot1.y);
    set(robot2_marker, 'XData', robot2.x, 'YData', robot2.y);

    drawnow;

    %% Check for Completion
    % If both robots are close to the end of the trajectory, stop the loop
    distance_to_goal1 = sqrt((robot1.x - x_traj_interp(end))^2 + (robot1.y - y_traj_interp(end))^2);
    distance_to_goal2 = sqrt((robot2.x - x_traj_interp(end))^2 + (robot2.y - y_traj_interp(end))^2);
    if distance_to_goal1 < 0.05 && distance_to_goal2 < 0.05 && leader_distance >= total_length_interp
        disp('Both robots have reached the end of the trajectory.');
        break;
    end

    %% Wait for Next Control Loop
    elapsed_time = elapsed_time + toc;
    waitfor(rate);
end

%% Stop the Robots
% Robot 1
cmdVelMsg1.linear.x = 0;
cmdVelMsg1.angular.z = 0;
send(cmdVelPub1, cmdVelMsg1);

% Robot 2
cmdVelMsg2.linear.x = 0;
cmdVelMsg2.angular.z = 0;
send(cmdVelPub2, cmdVelMsg2);

disp('Control loop finished. Robots stopped.');

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

% Update robot state from odometry message
function robot = update_robot_state_from_odom(odomMsg)
    % Extract position
    robot.x = odomMsg.pose.pose.position.x;
    robot.y = odomMsg.pose.pose.position.y;

    % Extract orientation (quaternion to yaw)
    q = odomMsg.pose.pose.orientation;
    siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    cosy_cosp = 1 - 2 * (q.y^2 + q.z^2);
    robot.theta = atan2(siny_cosp, cosy_cosp);
end
