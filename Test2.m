% Clear workspace and figures
clear all;
clc;
close all;

% Global variables for keyboard control
global leader_speed_input leader_turn_rate_input;

leader_speed_input = 0;      % Initial leader speed
leader_turn_rate_input = 0;  % Initial leader turn rate

% Simulation parameters
dt = 0.02;  % Time step [s]
T = 50;     % Total simulation time [s]
time = 0:dt:T;  % Time vector

% Leader parameters
leader_max_speed = 0.2;      % Max linear speed [m/s]
leader_max_turn_rate = 0.5;  % Max angular speed [rad/s]

% Desired distances (lateral and longitudinal)
desired_distance_longitudinal1 = 0.0;   % Robot 1 longitudinal distance from leader [m]
desired_distance_lateral1 = 0.5;        % Robot 1 lateral distance from leader [m] (left)

desired_distance_longitudinal2 = 0.0;   % Robot 2 longitudinal distance from leader [m]
desired_distance_lateral2 = -0.5;       % Robot 2 lateral distance from leader [m] (right)

% Control gains (optimized for TurtleBot3)
Kp_linear = 0.5; Ki_linear = 0.005; Kd_linear = 0.05;  % Reduced gains
Kp_angular = 0.7; Ki_angular = 0.02; Kd_angular = 0.05; % Reduced gains

% Velocity limits
max_linear_velocity = 0.5;   % [m/s]
max_angular_velocity = 2.0;  % [rad/s]

% Error thresholds (deadband)
linear_error_threshold = 0.01;   % [m]
angular_error_threshold = 0.01;  % [rad]

% Boundary box limits
xmin = -5; xmax = 5;
ymin = -5; ymax = 5;

% Initialize Leader State
leader = struct('x', 0, 'y', 0, 'theta', 0);

% Initialize Robot States
robot1 = init_robot(leader.x + desired_distance_lateral1, leader.y);
robot2 = init_robot(leader.x + desired_distance_lateral2, leader.y);

% Preallocate histories for leader and followers
leader_history = zeros(length(time), 2);
robot1_history = zeros(length(time), 2);
robot2_history = zeros(length(time), 2);
leader_theta_history = zeros(length(time), 1);
robot1_theta_history = zeros(length(time), 1);
robot2_theta_history = zeros(length(time), 1);

% Initialize errors for PID controllers
[error_int1, error_prev1] = init_pid_errors();
[error_int2, error_prev2] = init_pid_errors();

% Add noise and disturbance parameters
sensor_noise_std = 0.005;
disturbance_std = 0.01;

% Create figure and set KeyPressFcn
figure_handle = figure('KeyPressFcn', @keyPress);

%% Main Simulation Loop
for i = 1:length(time)
    % Use the global variables for leader speed and turn rate
    global leader_speed_input leader_turn_rate_input;
    
    % Limit the leader's speed and turn rate
    leader_speed = max(min(leader_speed_input, leader_max_speed), -leader_max_speed);
    leader_turn_rate = max(min(leader_turn_rate_input, leader_max_turn_rate), -leader_max_turn_rate);
    
    % Update Leader's Position
    [leader.x, leader.y, leader.theta] = update_leader(leader, leader_speed, leader_turn_rate, dt);
    
    % Ensure the leader stays within the boundaries
    leader.x = max(min(leader.x, xmax), xmin);
    leader.y = max(min(leader.y, ymax), ymin);
    
    leader_history(i, :) = [leader.x, leader.y];
    leader_theta_history(i) = leader.theta;
    
    % Desired positions for Robot 1 (left of leader)
    desired_x1 = leader.x - desired_distance_longitudinal1 * cos(leader.theta) + desired_distance_lateral1 * cos(leader.theta + pi/2);
    desired_y1 = leader.y - desired_distance_longitudinal1 * sin(leader.theta) + desired_distance_lateral1 * sin(leader.theta + pi/2);
    
    % Desired positions for Robot 2 (right of leader)
    desired_x2 = leader.x - desired_distance_longitudinal2 * cos(leader.theta) + desired_distance_lateral2 * cos(leader.theta + pi/2);
    desired_y2 = leader.y - desired_distance_longitudinal2 * sin(leader.theta) + desired_distance_lateral2 * sin(leader.theta + pi/2);
    
    % Control for Robot 1
    [robot1, error_int1, error_prev1] = update_robot_to_position(robot1, desired_x1, desired_y1, leader.theta, ...
        Kp_linear, Ki_linear, Kd_linear, Kp_angular, Ki_angular, Kd_angular, ...
        max_linear_velocity, max_angular_velocity, dt, disturbance_std, ...
        error_int1, error_prev1, linear_error_threshold, angular_error_threshold);
    
    % Control for Robot 2
    [robot2, error_int2, error_prev2] = update_robot_to_position(robot2, desired_x2, desired_y2, leader.theta, ...
        Kp_linear, Ki_linear, Kd_linear, Kp_angular, Ki_angular, Kd_angular, ...
        max_linear_velocity, max_angular_velocity, dt, disturbance_std, ...
        error_int2, error_prev2, linear_error_threshold, angular_error_threshold);
    
    % Ensure robots stay within boundaries
    robot1.x = max(min(robot1.x, xmax), xmin);
    robot1.y = max(min(robot1.y, ymax), ymin);
    robot2.x = max(min(robot2.x, xmax), xmin);
    robot2.y = max(min(robot2.y, ymax), ymin);
    
    % Record positions and orientations for plotting
    robot1_history(i, :) = [robot1.x, robot1.y];
    robot2_history(i, :) = [robot2.x, robot2.y];
    robot1_theta_history(i) = robot1.theta;
    robot2_theta_history(i) = robot2.theta;
    
    % Plot at reduced frequency for performance optimization
    if mod(i, round(1/dt/5)) == 0
        plot_robots(leader_history, robot1_history, robot2_history, leader_theta_history, robot1_theta_history, robot2_theta_history, i, xmin, xmax, ymin, ymax);
    end
    
    % Process events and update the figure
    drawnow limitrate;
end

% Final plot of the trajectory
plot_final_trajectory(leader_history, robot1_history, robot2_history);

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

% Leader update function
function [x, y, theta] = update_leader(leader, speed, turn_rate, dt)
    x = leader.x + speed * cos(leader.theta) * dt;
    y = leader.y + speed * sin(leader.theta) * dt;
    theta = leader.theta + turn_rate * dt;
    theta = atan2(sin(theta), cos(theta));  % Normalize
end

% Robot update function to move towards desired position and match orientation
function [robot, error_int, error_prev] = update_robot_to_position(robot, desired_x, desired_y, desired_theta, ...
    Kp_linear, Ki_linear, Kd_linear, Kp_angular, Ki_angular, Kd_angular, ...
    max_linear_velocity, max_angular_velocity, dt, disturbance_std, ...
    error_int, error_prev, linear_error_threshold, angular_error_threshold)

    % Calculate position errors
    error_x = desired_x - robot.x;
    error_y = desired_y - robot.y;
    distance_error = sqrt(error_x^2 + error_y^2);
    
    % Apply deadband for distance error
    if distance_error < linear_error_threshold
        distance_error = 0;
    end
    
    angle_to_goal = atan2(error_y, error_x);
    angle_error_position = angle_to_goal - robot.theta;
    angle_error_position = atan2(sin(angle_error_position), cos(angle_error_position));  % Normalize

    % Orientation error
    angle_error_orientation = desired_theta - robot.theta;
    angle_error_orientation = atan2(sin(angle_error_orientation), cos(angle_error_orientation));  % Normalize

    % Combine angle errors with weighting
    angle_error = 0.5 * angle_error_position + 0.5 * angle_error_orientation;
    
    % Apply deadband for angle error
    if abs(angle_error) < angular_error_threshold
        angle_error = 0;
    end

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
function plot_robots(leader_history, robot1_history, robot2_history, leader_theta_history, robot1_theta_history, robot2_theta_history, i, xmin, xmax, ymin, ymax)
    % Clear the current figure
    clf;

    % Plot settings
    hold on;
    grid on;
    axis equal;
    xlim([xmin-1, xmax+1]);
    ylim([ymin-1, ymax+1]);
    xlabel('X [m]');
    ylabel('Y [m]');
    title('Leader and Followers Trajectories');

    % Plot boundary box
    rectangle('Position', [xmin, ymin, xmax - xmin, ymax - ymin], 'EdgeColor', 'k', 'LineStyle', '--');

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
    legend('Leader Trajectory', 'Leader', 'Robot 1 Trajectory', 'Robot 1', 'Robot 2 Trajectory', 'Robot 2');

    drawnow; % Update the figure
end

% Function to plot the final trajectory
function plot_final_trajectory(leader_history, robot1_history, robot2_history)
    figure;
    hold on;
    grid on;
    axis equal;
    xlabel('X [m]');
    ylabel('Y [m]');
    title('Final Trajectories of Leader and Followers');

    % Plot leader's trajectory
    plot(leader_history(:,1), leader_history(:,2), 'k-', 'LineWidth', 2); % Black line for leader

    % Plot Robot 1's trajectory
    plot(robot1_history(:,1), robot1_history(:,2), 'b-', 'LineWidth', 2); % Blue line for Robot 1

    % Plot Robot 2's trajectory
    plot(robot2_history(:,1), robot2_history(:,2), 'r-', 'LineWidth', 2); % Red line for Robot 2

    % Add a legend
    legend('Leader Trajectory', 'Robot 1 Trajectory', 'Robot 2 Trajectory');

    hold off;
end

% Key press callback function
function keyPress(~, event)
    global leader_speed_input leader_turn_rate_input;

    switch event.Key
        case 'w'  % Increase forward speed
            leader_speed_input = leader_speed_input + 0.02;
        case 's'  % Decrease forward speed
            leader_speed_input = leader_speed_input - 0.02;
        case 'a'  % Turn left
            leader_turn_rate_input = leader_turn_rate_input + 0.02;
        case 'd'  % Turn right
            leader_turn_rate_input = leader_turn_rate_input - 0.02;
        case 'x'  % Stop movement
            leader_speed_input = 0;
            leader_turn_rate_input = 0;
    end
end

