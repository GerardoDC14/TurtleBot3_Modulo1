% Clear workspace and figures
clear all;
clc;
close all;

%% Simulation Parameters

dt = 0.02;  % Time step [s]
leader_speed = 0.08; % Adjust leader speed as necessary

%% Trajectory Drawing

disp('Draw the trajectory you want the robots to follow.');
disp('Left-click and hold to begin drawing a freehand path.');
disp('Simply release the mouse button to finish.');

figure_handle = figure;
axis([-1.5 1.5 -1.5 1.5]);  % Adjust axis limits according to your boundary box
xlabel('X [m]');
ylabel('Y [m]');
title('Dibujar trayectoria deseada');
hold on;

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

% Desired distances (lateral and longitudinal)
desired_distance_lateral1 = 0.2;        % Robot 1 lateral distance from leader [m] (left)
desired_distance_lateral2 = -0.2;       % Robot 2 lateral distance from leader [m] (right)

% Control gains (adjust as necessary)
Kp_linear = 0.8; Ki_linear = 0.0; Kd_linear = 0.1;
Kp_angular = 1.0; Ki_angular = 0.0; Kd_angular = 0.1;

% Velocity limits
max_linear_velocity = 0.22;   % [m/s] TurtleBot3 max linear velocity
max_angular_velocity = 2.84;  % [rad/s] TurtleBot3 max angular velocity

% Boundary box limits
xmin = -1.5; xmax = 1.5;
ymin = -1.5; ymax = 1.5;

%% Import Robot Model

robot_model = importrobot('robotisTurtleBot3Burger.urdf'); % Use the correct URDF file name
robot_model.DataFormat = 'row';

%% Initialize Leader State

leader = struct('x', x_traj_interp(1), 'y', y_traj_interp(1), 'theta', 0);

% Initialize Robots' States
robot1 = struct('x', leader.x + desired_distance_lateral1, 'y', leader.y, 'theta', 0);
robot2 = struct('x', leader.x + desired_distance_lateral2, 'y', leader.y, 'theta', 0);

% Initialize PID errors
[error_int1, error_prev1] = init_pid_errors();
[error_int2, error_prev2] = init_pid_errors();

%% Control Loop Setup

num_steps = round((total_length_interp / leader_speed) / dt);

% Initialize variables for plotting
robot1_history = [];
robot2_history = [];
leader_history = [];
leader_theta_history = [];

%% Set up figure for real-time plotting

figure_handle = figure;
set(figure_handle, 'Position', [100, 100, 800, 600]);
ax = axes('Parent', figure_handle);
hold(ax, 'on');
grid(ax, 'on');
axis(ax, 'equal');
xlim(ax, [xmin-0.5, xmax+0.5]);
ylim(ax, [ymin-0.5, ymax+0.5]);
xlabel(ax, 'X [m]');
ylabel(ax, 'Y [m]');
title(ax, 'TurtleBot3 - Consenso');

% Plot boundary box
rectangle('Parent', ax, 'Position', [xmin, ymin, xmax - xmin, ymax - ymin], 'EdgeColor', 'k', 'LineStyle', '--');

% Plot the drawn trajectory
plot_traj = plot(ax, x_traj_smooth, y_traj_smooth, 'k--', 'LineWidth', 1);

% Initialize plots for leader and robots' trajectories
plot_leader_traj = plot(ax, nan, nan, 'k-', 'LineWidth', 2);
plot_robot1_traj = plot(ax, nan, nan, 'b-', 'LineWidth', 2);
plot_robot2_traj = plot(ax, nan, nan, 'r-', 'LineWidth', 2);

% Create hgtransform objects for the robots
hg_robot1 = hgtransform('Parent', ax);
hg_robot2 = hgtransform('Parent', ax);

% Plot robots using robot models under the axes
% Robot 1
show(robot_model, 'Parent', ax, 'Frames', 'off', 'PreservePlot', true, 'Visuals', 'on');
% Find the robot graphics and set their parent to hg_robot1
robot1_patches = findobj(ax, 'Type', 'Patch');
set(robot1_patches, 'Parent', hg_robot1);

% Robot 2
show(robot_model, 'Parent', ax, 'Frames', 'off', 'PreservePlot', true, 'Visuals', 'on');
% Find the new robot graphics and set their parent to hg_robot2
robot2_patches = findobj(ax, 'Type', 'Patch', '-not', 'Parent', hg_robot1);
set(robot2_patches, 'Parent', hg_robot2);

legend(ax, [plot_traj, plot_leader_traj, plot_robot1_traj, plot_robot2_traj], ...
    {'Trayectoria Dibujada', 'Trayectoria del lider', 'Robot 1', 'Robot 2'});

drawnow;

%% Main Control Loop

leader_distance = 0;

for i = 1:num_steps
    %% Update Leader Position
    leader_distance = leader_distance + leader_speed * dt;
    if leader_distance > total_length_interp
        leader_distance = total_length_interp;
    end

    % Get leader's position by interpolating x and y at leader_distance
    leader.x = interp1(cum_distance_interp, x_traj_interp, leader_distance);
    leader.y = interp1(cum_distance_interp, y_traj_interp, leader_distance);

    % Compute leader's orientation based on the trajectory direction
    leader_dx = interp1(cum_distance_interp(1:end-1), diff(x_traj_interp)./diff(cum_distance_interp), leader_distance, 'linear', 'extrap');
    leader_dy = interp1(cum_distance_interp(1:end-1), diff(y_traj_interp)./diff(cum_distance_interp), leader_distance, 'linear', 'extrap');
    leader.theta = atan2(leader_dy, leader_dx);

    % Record leader's position and orientation
    leader_history = [leader_history; leader.x, leader.y];
    leader_theta_history = [leader_theta_history; leader.theta];

    %% Desired Positions for Robots
    % Robot 1 (left of leader)
    desired_x1 = leader.x + desired_distance_lateral1 * cos(leader.theta + pi/2);
    desired_y1 = leader.y + desired_distance_lateral1 * sin(leader.theta + pi/2);
    desired_theta1 = leader.theta;

    % Robot 2 (right of leader)
    desired_x2 = leader.x + desired_distance_lateral2 * cos(leader.theta + pi/2);
    desired_y2 = leader.y + desired_distance_lateral2 * sin(leader.theta + pi/2);
    desired_theta2 = leader.theta;

    %% Apply Control Algorithm
    % Control for Robot 1
    [cmd_vel1, error_int1, error_prev1] = compute_control(robot1, desired_x1, desired_y1, desired_theta1, ...
        Kp_linear, Ki_linear, Kd_linear, Kp_angular, Ki_angular, Kd_angular, ...
        max_linear_velocity, max_angular_velocity, dt, ...
        error_int1, error_prev1);

    % Control for Robot 2
    [cmd_vel2, error_int2, error_prev2] = compute_control(robot2, desired_x2, desired_y2, desired_theta2, ...
        Kp_linear, Ki_linear, Kd_linear, Kp_angular, Ki_angular, Kd_angular, ...
        max_linear_velocity, max_angular_velocity, dt, ...
        error_int2, error_prev2);

    %% Boundary Checks
    % Robot 1
    if robot1.x < xmin || robot1.x > xmax || robot1.y < ymin || robot1.y > ymax
        cmd_vel1.linear.x = 0;
        cmd_vel1.angular.z = 0;
        disp('Robot 1 is out of bounds!');
    end

    % Robot 2
    if robot2.x < xmin || robot2.x > xmax || robot2.y < ymin || robot2.y > ymax
        cmd_vel2.linear.x = 0;
        cmd_vel2.angular.z = 0;
        disp('Robot 2 is out of bounds!');
    end

    %% Update Robots' Positions and Orientations Based on Commands
    % Robot 1
    robot1.x = robot1.x + cmd_vel1.linear.x * cos(robot1.theta) * dt;
    robot1.y = robot1.y + cmd_vel1.linear.x * sin(robot1.theta) * dt;
    robot1.theta = robot1.theta + cmd_vel1.angular.z * dt;
    robot1.theta = atan2(sin(robot1.theta), cos(robot1.theta));  % Normalize angle

    % Robot 2
    robot2.x = robot2.x + cmd_vel2.linear.x * cos(robot2.theta) * dt;
    robot2.y = robot2.y + cmd_vel2.linear.x * sin(robot2.theta) * dt;
    robot2.theta = robot2.theta + cmd_vel2.angular.z * dt;
    robot2.theta = atan2(sin(robot2.theta), cos(robot2.theta));  % Normalize angle

    %% Record Positions for Plotting
    robot1_history = [robot1_history; robot1.x, robot1.y];
    robot2_history = [robot2_history; robot2.x, robot2.y];

    %% Update Plots
    % Update leader trajectory plot
    set(plot_leader_traj, 'XData', leader_history(:,1), 'YData', leader_history(:,2));

    % Update robots' trajectories
    set(plot_robot1_traj, 'XData', robot1_history(:,1), 'YData', robot1_history(:,2));
    set(plot_robot2_traj, 'XData', robot2_history(:,1), 'YData', robot2_history(:,2));

    % Update robots' transforms
    % Robot 1
    robot1_tf = makehgtform('translate', [robot1.x, robot1.y, 0], 'zrotate', robot1.theta);
    set(hg_robot1, 'Matrix', robot1_tf);

    % Robot 2
    robot2_tf = makehgtform('translate', [robot2.x, robot2.y, 0], 'zrotate', robot2.theta);
    set(hg_robot2, 'Matrix', robot2_tf);

    drawnow;

    %% Pause for Next Iteration
    pause(dt);
end

%% Stop the Simulation
disp('Simulation finished.');

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

    [omega, error_int.angular, error_prev.angular] = pid_control(angle_error, error_int.angular, error_prev.angular, ...
        Kp_angular, Ki_angular, Kd_angular, dt, max_angular_velocity);

    cmd_vel.linear.x = v;
    cmd_vel.angular.z = omega;
end

function [output, error_int, error_prev] = pid_control(error, error_int, error_prev, Kp, Ki, Kd, dt, max_output)
    derivative = (error - error_prev) / dt;
    output_unsat = Kp * error + Ki * error_int + Kd * derivative;

    output = max(min(output_unsat, max_output), -max_output);

    if abs(output_unsat) < max_output
        error_int = error_int + error * dt;
    end

    error_prev = error;
end
