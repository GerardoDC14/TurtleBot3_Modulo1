% Clear workspace and figures
clear all;
clc;
close all;

% Simulation parameters
dt = 0.02;  % Time step [s]
leader_speed = 0.08; % Reduced leader speed [m/s]

% Ask the user to draw a trajectory using drawfreehand
disp('Draw the trajectory you want the robots to follow.');
disp('Left-click and hold to begin drawing a freehand path.');
disp('Simply release the mouse button to finish.');

figure_handle = figure;
axis([0 10 0 10]);
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
desired_spacing = 0.1; % Spacing between points in meters
num_points = max(round(total_length / desired_spacing), 2); % Ensure at least 2 points
even_distance = linspace(0, total_length, num_points)';

% Interpolate x and y over the even_distance vector
x_traj_interp = interp1(cum_distance, x_traj_smooth, even_distance);
y_traj_interp = interp1(cum_distance, y_traj_smooth, even_distance);

% Recompute the cumulative distance for interpolated trajectory
distance_interp = sqrt(diff(x_traj_interp).^2 + diff(y_traj_interp).^2);
cum_distance_interp = [0; cumsum(distance_interp)];
total_length_interp = cum_distance_interp(end);

% Desired distances (lateral and longitudinal)
desired_distance_longitudinal1 = 0.0;   % Robot 1 longitudinal distance from leader [m]
desired_distance_lateral1 = 0.5;        % Robot 1 lateral distance from leader [m] (left)

desired_distance_longitudinal2 = 0.0;   % Robot 2 longitudinal distance from leader [m]
desired_distance_lateral2 = -0.5;       % Robot 2 lateral distance from leader [m] (right)

% Control gains (similar to the working code)
Kp_linear = 0.8; Ki_linear = 0.01; Kd_linear = 0.1;
Kp_angular = 1.0; Ki_angular = 0.05; Kd_angular = 0.1;

% Velocity limits
max_linear_velocity = 0.5;   % [m/s]
max_angular_velocity = 2.0;  % [rad/s]

% Initialize Leader State
leader = struct('x', x_traj_interp(1), 'y', y_traj_interp(1), 'theta', 0);

% Initialize Robot States
robot1 = init_robot(leader.x + desired_distance_lateral1, leader.y);
robot2 = init_robot(leader.x + desired_distance_lateral2, leader.y);

% Initialize leader's distance along the trajectory
leader_distance = 0;

% Compute total simulation steps based on total trajectory length and leader speed
total_time = total_length_interp / leader_speed;
time = 0:dt:total_time;
num_steps = length(time);

% Preallocate histories for leader and followers
leader_history = zeros(num_steps, 2);
robot1_history = zeros(num_steps, 2);
robot2_history = zeros(num_steps, 2);
leader_theta_history = zeros(num_steps, 1);
robot1_theta_history = zeros(num_steps, 1);
robot2_theta_history = zeros(num_steps, 1);

% Initialize errors for PID controllers
[error_int1, error_prev1] = init_pid_errors();
[error_int2, error_prev2] = init_pid_errors();

% Add noise and disturbance parameters
sensor_noise_std = 0.005;
disturbance_std = 0.01;

%% Main Simulation Loop
for i = 1:num_steps
    % Update leader's distance along the trajectory
    leader_distance = leader_distance + leader_speed * dt;
    if leader_distance > total_length_interp
        leader_distance = total_length_interp;
    end

    % Get leader's position by interpolating x and y at leader_distance
    leader.x = interp1(cum_distance_interp, x_traj_interp, leader_distance);
    leader.y = interp1(cum_distance_interp, y_traj_interp, leader_distance);

    % Compute leader's orientation based on the trajectory direction
    % Compute the derivative of x and y at leader_distance
    leader_dx = interp1(cum_distance_interp(1:end-1), diff(x_traj_interp)./diff(cum_distance_interp), leader_distance, 'linear', 'extrap');
    leader_dy = interp1(cum_distance_interp(1:end-1), diff(y_traj_interp)./diff(cum_distance_interp), leader_distance, 'linear', 'extrap');
    leader.theta = atan2(leader_dy, leader_dx);

    % Record leader's position and orientation
    leader_history(i,:) = [leader.x, leader.y];
    leader_theta_history(i) = leader.theta;

    % Desired positions for Robot 1 (left of leader)
    desired_x1 = leader.x - desired_distance_longitudinal1 * cos(leader.theta) + desired_distance_lateral1 * cos(leader.theta + pi/2);
    desired_y1 = leader.y - desired_distance_longitudinal1 * sin(leader.theta) + desired_distance_lateral1 * sin(leader.theta + pi/2);

    % Desired positions for Robot 2 (right of leader)
    desired_x2 = leader.x - desired_distance_longitudinal2 * cos(leader.theta) + desired_distance_lateral2 * cos(leader.theta + pi/2);
    desired_y2 = leader.y - desired_distance_longitudinal2 * sin(leader.theta) + desired_distance_lateral2 * sin(leader.theta + pi/2);

    % Control for Robot 1
    [robot1, error_int1, error_prev1] = update_robot_to_position(robot1, desired_x1, desired_y1, ...
        Kp_linear, Ki_linear, Kd_linear, Kp_angular, Ki_angular, Kd_angular, ...
        max_linear_velocity, max_angular_velocity, dt, sensor_noise_std, disturbance_std, ...
        error_int1, error_prev1);

    % Control for Robot 2
    [robot2, error_int2, error_prev2] = update_robot_to_position(robot2, desired_x2, desired_y2, ...
        Kp_linear, Ki_linear, Kd_linear, Kp_angular, Ki_angular, Kd_angular, ...
        max_linear_velocity, max_angular_velocity, dt, sensor_noise_std, disturbance_std, ...
        error_int2, error_prev2);

    % Record positions and orientations for plotting
    robot1_history(i, :) = [robot1.x, robot1.y];
    robot2_history(i, :) = [robot2.x, robot2.y];
    robot1_theta_history(i) = robot1.theta;
    robot2_theta_history(i) = robot2.theta;

    % Plot at reduced frequency for performance optimization
    if mod(i, round(1/dt/5)) == 0 || i == num_steps
        % Use the same figure window for plotting
        clf; % Clear current figure
        plot_robots(leader_history, robot1_history, robot2_history, leader_theta_history, robot1_theta_history, robot2_theta_history, i, x_traj_smooth, y_traj_smooth);
    end
end

% Final plot of the trajectory
plot_final_trajectory(leader_history, robot1_history, robot2_history, x_traj_smooth, y_traj_smooth);

%% Supporting Functions

% Initialize robot
function robot = init_robot(x, y)
    robot = struct('x', x, 'y', y, 'theta', 0);
end

% Initialize PID errors
function [error_int, error_prev] = init_pid_errors()
    error_int = struct('linear', 0, 'angular', 0);
    error_prev = struct('linear', 0, 'angular', 0);
end

% Robot update function to move towards desired position
function [robot, error_int, error_prev] = update_robot_to_position(robot, desired_x, desired_y, ...
    Kp_linear, Ki_linear, Kd_linear, Kp_angular, Ki_angular, Kd_angular, ...
    max_linear_velocity, max_angular_velocity, dt, sensor_noise_std, disturbance_std, ...
    error_int, error_prev)

    % Calculate errors
    error_x = desired_x - robot.x + sensor_noise_std * randn;
    error_y = desired_y - robot.y + sensor_noise_std * randn;
    distance_error = sqrt(error_x^2 + error_y^2);
    angle_to_goal = atan2(error_y, error_x);
    angle_error = angle_to_goal - robot.theta;
    angle_error = atan2(sin(angle_error), cos(angle_error));  % Normalize

    % Apply PID control for linear and angular velocities
    [v, error_int.linear, error_prev.linear] = pid_control(distance_error, error_int.linear, error_prev.linear, ...
        Kp_linear, Ki_linear, Kd_linear, dt, max_linear_velocity);
    [omega, error_int.angular, error_prev.angular] = pid_control(angle_error, error_int.angular, error_prev.angular, ...
        Kp_angular, Ki_angular, Kd_angular, dt, max_angular_velocity);

    % Add disturbances
    v = v + disturbance_std * randn;
    omega = omega + disturbance_std * randn;

    % Update robot state
    robot.x = robot.x + v * cos(robot.theta) * dt;
    robot.y = robot.y + v * sin(robot.theta) * dt;
    robot.theta = robot.theta + omega * dt;
    robot.theta = atan2(sin(robot.theta), cos(robot.theta));  % Normalize
end

% PID control with anti-windup
function [output, error_int, error_prev] = pid_control(error, error_int, error_prev, Kp, Ki, Kd, dt, max_output)
    % PID control with anti-windup
    derivative = (error - error_prev) / dt;
    output_unsat = Kp * error + Ki * error_int + Kd * derivative;

    % Saturate output
    output = max(min(output_unsat, max_output), -max_output);

    % Anti-windup: Only integrate if not saturated
    if abs(output_unsat) < max_output
        error_int = error_int + error * dt;
    end

    % Update previous error
    error_prev = error;
end

% Function to plot the leader and robots' trajectories with orientations
function plot_robots(leader_history, robot1_history, robot2_history, leader_theta_history, robot1_theta_history, robot2_theta_history, i, x_traj, y_traj)
    % Plot settings
    hold on;
    grid on;
    axis equal;
    xlabel('X [m]');
    ylabel('Y [m]');
    title('Leader and Followers Trajectories');

    % Plot the drawn trajectory
    plot(x_traj, y_traj, 'k--', 'LineWidth', 1); % Dashed black line for the trajectory

    % Plot leader's trajectory
    plot(leader_history(1:i,1), leader_history(1:i,2), 'k-', 'LineWidth', 2); % Black line for leader
    % Plot leader's current position and orientation
    quiver(leader_history(i,1), leader_history(i,2), cos(leader_theta_history(i)), sin(leader_theta_history(i)), 0.3, 'k', 'LineWidth', 2, 'MaxHeadSize', 2);

    % Plot Robot 1's trajectory
    plot(robot1_history(1:i,1), robot1_history(1:i,2), 'b-', 'LineWidth', 2); % Blue line for Robot 1
    % Plot Robot 1's current position and orientation
    quiver(robot1_history(i,1), robot1_history(i,2), cos(robot1_theta_history(i)), sin(robot1_theta_history(i)), 0.3, 'b', 'LineWidth', 2, 'MaxHeadSize', 2);

    % Plot Robot 2's trajectory
    plot(robot2_history(1:i,1), robot2_history(1:i,2), 'r-', 'LineWidth', 2); % Red line for Robot 2
    % Plot Robot 2's current position and orientation
    quiver(robot2_history(i,1), robot2_history(i,2), cos(robot2_theta_history(i)), sin(robot2_theta_history(i)), 0.3, 'r', 'LineWidth', 2, 'MaxHeadSize', 2);

    % Add a legend
    legend('Drawn Trajectory', 'Leader Trajectory', 'Leader', 'Robot 1 Trajectory', 'Robot 1', 'Robot 2 Trajectory', 'Robot 2');

    drawnow; % Update the figure
end

% Function to plot the final trajectory
function plot_final_trajectory(leader_history, robot1_history, robot2_history, x_traj, y_traj)
    figure;
    hold on;
    grid on;
    axis equal;
    xlabel('X [m]');
    ylabel('Y [m]');
    title('Final Trajectories of Leader and Followers');

    % Plot the drawn trajectory
    plot(x_traj, y_traj, 'k--', 'LineWidth', 1); % Dashed black line for the trajectory

    % Plot leader's trajectory
    plot(leader_history(:,1), leader_history(:,2), 'k-', 'LineWidth', 2); % Black line for leader

    % Plot Robot 1's trajectory
    plot(robot1_history(:,1), robot1_history(:,2), 'b-', 'LineWidth', 2); % Blue line for Robot 1

    % Plot Robot 2's trajectory
    plot(robot2_history(:,1), robot2_history(:,2), 'r-', 'LineWidth', 2); % Red line for Robot 2

    % Add a legend
    legend('Drawn Trajectory', 'Leader Trajectory', 'Robot 1 Trajectory', 'Robot 2 Trajectory');

    hold off;
end
