clear all;
clc;

% Initialize ROS 2 environment and domain ID
setenv('ROS_DOMAIN_ID', '0');

% List ROS 2 topics to confirm connection
topics = ros2('topic', 'list');
disp('Available topics:');
disp(topics);

% Create a ROS 2 node
node = ros2node('/matlab_node');

% Create a publisher for the /cmd_vel topic
cmdVelPub = ros2publisher(node, '/cmd_vel', 'geometry_msgs/Twist');

% Create a Twist message
msg = ros2message(cmdVelPub);

% Create subscribers for the /aruco_x, /aruco_y, and /aruco_yaw topics
xSub = ros2subscriber(node, '/aruco_x', 'std_msgs/Float32');
ySub = ros2subscriber(node, '/aruco_y', 'std_msgs/Float32');
yawSub = ros2subscriber(node, '/aruco_yaw', 'std_msgs/Float32');

% Desired target position (goal)
x_target = 0.2;  % Target x position [meters]
y_target = -0.15; % Target y position [meters]

% Control gains
Kp_base = 1.5;  % Higher for faster response
Kp_nonlinear = 2.0;  % Nonlinear gain for larger distance errors
Ki = 0.05;  % Integral gain for reducing steady-state error
Kd = 0.2;  % Lowered for less damping
kpd = 5.5; % Gain for tanh control

angular_Kp_base = 1.8;  % Increase angular proportional gain

% Maximum velocity limits (saturation limits)
max_linear_velocity = 0.07;  % Limit max linear velocity
max_angular_velocity = 0.4;  % Limit max angular velocity

% Initialize arrays for position storage (optional for plotting)
x_positions = [];
y_positions = [];

% Initialize moving average filter variables for x, y, and yaw
window_size = 10;  % Size of the moving average window
x_buffer = zeros(1, window_size);  % Buffer to store recent x positions
y_buffer = zeros(1, window_size);  % Buffer to store recent y positions
yaw_buffer = zeros(1, window_size); % Buffer to store recent yaw values
buffer_index = 1;  % Index for circular buffer

% Integral error initialization
integral_error_x = 0;
integral_error_y = 0;
integral_windup_limit = 0.1;  % Limit to avoid integral windup

% Create figure for real-time plotting (optional)
figure;
hold on;
xlabel('X [m]');
ylabel('Y [m]');
title('Real-time Robot Position with Nonlinear PID Control');
grid on;
axis equal;

% Control loop frequency
rate = ros2rate(node, 50);  % Control loop frequency at 50 Hz

% Initialize previous error for derivative calculation
prev_error_distance = 0;

% Run loop to constantly receive and control the robot
for i = 1:1000
    % Receive messages from the /aruco_x, /aruco_y, and /aruco_yaw topics
    xMsg = receive(xSub, 0.6);
    yMsg = receive(ySub, 0.6);
    yawMsg = receive(yawSub, 0.6);

    % Extract data from the received messages
    x = double(xMsg.data);
    y = double(yMsg.data);
    yaw = double(yawMsg.data);

    % Update the moving average buffers for x, y, and yaw
    x_buffer(buffer_index) = x;
    y_buffer(buffer_index) = y;
    yaw_buffer(buffer_index) = yaw;
    buffer_index = mod(buffer_index, window_size) + 1;

    % Apply moving average filter
    x_filtered = mean(x_buffer);
    y_filtered = mean(y_buffer);
    yaw_filtered = mean(yaw_buffer);

    % Append new filtered positions to arrays for plotting
    x_positions = [x_positions, x_filtered];
    y_positions = [y_positions, y_filtered];

    % Plot the robot's position in real-time
    plot(x_positions, y_positions, 'b-', 'LineWidth', 2);
    drawnow;

    %% Calculate Errors
    error_x = x_target - x_filtered;
    error_y = y_target - y_filtered;

    % Calculate the distance to the target
    distance_to_target = sqrt(error_x^2 + error_y^2);

    % Calculate the angle to the target
    angle_to_target = atan2(error_y, error_x);

    % Calculate the yaw error using filtered yaw
    error_yaw = angle_to_target - yaw_filtered;

    % Normalize the angular error to be within [-pi, pi]
    error_yaw = atan2(sin(error_yaw), cos(error_yaw));

    %% Apply tanh control laws for velocities
    % Integral error accumulation with windup prevention
    integral_error_x = integral_error_x + error_x;
    integral_error_y = integral_error_y + error_y;

    % Apply windup limit to prevent integral from growing too large
    integral_error_x = max(min(integral_error_x, integral_windup_limit), -integral_windup_limit);
    integral_error_y = max(min(integral_error_y, integral_windup_limit), -integral_windup_limit);

    % Control laws for linear and angular velocities using tanh
    v_linear = -max_linear_velocity * tanh(((kpd * distance_to_target)^3) / max_linear_velocity);
    v_angular = -max_angular_velocity * tanh((angular_Kp_base * error_yaw)^3 / max_angular_velocity);

    % Saturate velocities
    linear_velocity = max(min(v_linear, max_linear_velocity), -max_linear_velocity);
    angular_velocity = max(min(v_angular, max_angular_velocity), -max_angular_velocity);

    %% Send velocity commands
    msg.linear.x = double(linear_velocity);
    msg.angular.z = double(angular_velocity);

    % Send the velocity command to the robot
    send(cmdVelPub, msg);

    % Wait for the next cycle
    waitfor(rate);

    % Stop if the robot is close enough to the target
    if distance_to_target < 0.01
        disp('Target reached. Stopping the robot.');
        break;
    end
end

% Stop the robot after reaching the target
msg.linear.x = 0.0;
msg.angular.z = 0.0;
send(cmdVelPub, msg);
disp('Sent stop command');

% Clean up: Clear the node and publisher
clear cmdVelPub node;

% Display a message indicating that the script has finished
disp('Node and publisher have been cleared. Exiting.');
