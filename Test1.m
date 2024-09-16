% Clear workspace and figures
clear all;
clc;
close all;

% Simulation parameters
dt = 0.02;  % Time step [s]
T = 50;     % Total simulation time [s]
time = 0:dt:T;  % Time vector

% Leader parameters
leader_speed = 0.1;      % Linear speed [m/s]
leader_turn_rate = 0.1;  % Angular speed [rad/s]

% Desired distances (lateral and longitudinal)
desired_distance_longitudinal1 = 0.0;   % Robot 1 longitudinal distance from leader [m]
desired_distance_lateral1 = 0.5;        % Robot 1 lateral distance from leader [m] (left)

desired_distance_longitudinal2 = 0.0;   % Robot 2 longitudinal distance from leader [m]
desired_distance_lateral2 = -0.5;       % Robot 2 lateral distance from leader [m] (right)

% Control gains (optimized for TurtleBot3)
Kp_linear = 0.8; Ki_linear = 0.01; Kd_linear = 0.1;
Kp_angular = 1.0; Ki_angular = 0.05; Kd_angular = 0.1;

% Velocity limits
max_linear_velocity = 0.5;   % [m/s]
max_angular_velocity = 2.0;  % [rad/s]

% Initialize Leader State
leader = struct('x', 0, 'y', 0, 'theta', 0);

% Initialize Robot States
robot1 = init_robot(leader.x + desired_distance_lateral1, leader.y);
robot2 = init_robot(leader.x + desired_distance_lateral2, leader.y);

% Preallocate histories for leader and followers
leader_history = zeros(length(time), 2);
robot1_history = zeros(length(time), 2);
robot2_history = zeros(length(time), 2);

% Initialize errors for PID controllers
[error_int1, error_prev1] = init_pid_errors();
[error_int2, error_prev2] = init_pid_errors();

% Add noise and disturbance parameters
sensor_noise_std = 0.005;
disturbance_std = 0.01;

%% Main Simulation Loop
for i = 1:length(time)
    % Update Leader's Position
    [leader.x, leader.y, leader.theta] = update_leader(leader, leader_speed, leader_turn_rate, dt);
    leader_history(i, :) = [leader.x, leader.y];
    
    % Desired positions for Robot 1 (left of leader)
    desired_x1 = leader.x - desired_distance_longitudinal1 * cos(leader.theta) + desired_distance_lateral1 * cos(leader.theta + pi/2) + sensor_noise_std * randn;
    desired_y1 = leader.y - desired_distance_longitudinal1 * sin(leader.theta) + desired_distance_lateral1 * sin(leader.theta + pi/2) + sensor_noise_std * randn;
    
    % Desired positions for Robot 2 (right of leader)
    desired_x2 = leader.x - desired_distance_longitudinal2 * cos(leader.theta) + desired_distance_lateral2 * cos(leader.theta + pi/2) + sensor_noise_std * randn;
    desired_y2 = leader.y - desired_distance_longitudinal2 * sin(leader.theta) + desired_distance_lateral2 * sin(leader.theta + pi/2) + sensor_noise_std * randn;
    
    % Control for Robot 1
    [robot1, error_int1, error_prev1] = update_robot_to_position(robot1, desired_x1, desired_y1, ...
        Kp_linear, Ki_linear, Kd_linear, Kp_angular, Ki_angular, Kd_angular, ...
        max_linear_velocity, max_angular_velocity, dt, disturbance_std, ...
        error_int1, error_prev1);
    
    % Control for Robot 2
    [robot2, error_int2, error_prev2] = update_robot_to_position(robot2, desired_x2, desired_y2, ...
        Kp_linear, Ki_linear, Kd_linear, Kp_angular, Ki_angular, Kd_angular, ...
        max_linear_velocity, max_angular_velocity, dt, disturbance_std, ...
        error_int2, error_prev2);
    
    % Record positions for plotting
    robot1_history(i, :) = [robot1.x, robot1.y];
    robot2_history(i, :) = [robot2.x, robot2.y];
    
    % Plot at reduced frequency for performance optimization
    if mod(i, round(1/dt/5)) == 0
        plot_robots(leader_history, robot1_history, robot2_history, i);
    end
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
end

% Robot update function to move towards desired position
function [robot, error_int, error_prev] = update_robot_to_position(robot, desired_x, desired_y, ...
    Kp_linear, Ki_linear, Kd_linear, Kp_angular, Ki_angular, Kd_angular, ...
    max_linear_velocity, max_angular_velocity, dt, disturbance_std, ...
    error_int, error_prev)

    % Calculate errors
    error_x = desired_x - robot.x;
    error_y = desired_y - robot.y;
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

% Function to plot the leader and robots' trajectories
function plot_robots(leader_history, robot1_history, robot2_history, i)
    % Clear the current figure
    clf;

    % Plot settings
    hold on;
    grid on;
    axis equal;
    xlabel('X [m]');
    ylabel('Y [m]');
    title('Leader and Followers Trajectories');

    % Plot leader's trajectory
    plot(leader_history(1:i,1), leader_history(1:i,2), 'k-', 'LineWidth', 2); % Black line for leader
    plot(leader_history(i,1), leader_history(i,2), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k'); % Black dot for leader's current position

    % Plot Robot 1's trajectory
    plot(robot1_history(1:i,1), robot1_history(1:i,2), 'b-', 'LineWidth', 2); % Blue line for Robot 1
    plot(robot1_history(i,1), robot1_history(i,2), 'bo', 'MarkerSize', 6, 'MarkerFaceColor', 'b'); % Blue dot for Robot 1's current position

    % Plot Robot 2's trajectory
    plot(robot2_history(1:i,1), robot2_history(1:i,2), 'r-', 'LineWidth', 2); % Red line for Robot 2
    plot(robot2_history(i,1), robot2_history(i,2), 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r'); % Red dot for Robot 2's current position

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
