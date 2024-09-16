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

% Create a publisher for the first robot (/cmd_vel topic)
cmdVelPub = ros2publisher(node, '/cmd_vel', 'geometry_msgs/Twist');
msg = ros2message(cmdVelPub); % Twist message for the first robot

% Create subscribers for the first robot (/aruco_x, /aruco_y, and /aruco_yaw topics)
xSub = ros2subscriber(node, '/aruco_x', 'std_msgs/Float32');
ySub = ros2subscriber(node, '/aruco_y', 'std_msgs/Float32');
yawSub = ros2subscriber(node, '/aruco_yaw', 'std_msgs/Float32');

% Desired target position (goal) for the first robot
x_target = 0.2;  % Target x position [meters]
y_target = 0.0; % Target y position [meters]

% Create a publisher for the second robot (/tb3_1/cmd_vel topic)
cmdVelPub2 = ros2publisher(node, '/tb3_1/cmd_vel', 'geometry_msgs/Twist');
msg2 = ros2message(cmdVelPub2); % Twist message for the second robot

% Create subscribers for the second robot (/aruco1_x, /aruco1_y, and /aruco1_yaw topics)
xSub2 = ros2subscriber(node, '/aruco1_x', 'std_msgs/Float32');
ySub2 = ros2subscriber(node, '/aruco1_y', 'std_msgs/Float32');
yawSub2 = ros2subscriber(node, '/aruco1_yaw', 'std_msgs/Float32');

% Desired target position (goal) for the second robot
x_target2 = 0.0;  % Target x position [meters]
y_target2 = 0.0;   % Target y position [meters]

% Control gains (shared between both robots)
Kp_base = 1.5;
Kp_nonlinear = 2.0;
Ki = 0.05;
Kd = 0.2;
kpd = 5.5;
angular_Kp_base = 1.8;

% Maximum velocity limits
max_linear_velocity = 0.07;
max_angular_velocity = 0.4;

% Initialize buffers and positions for both robots
window_size = 10;

% For robot 1
x_buffer = zeros(1, window_size);
y_buffer = zeros(1, window_size);
yaw_buffer = zeros(1, window_size);
buffer_index = 1;

x_positions = [];
y_positions = [];

integral_error_x = 0;
integral_error_y = 0;
integral_windup_limit = 0.1;

% For robot 2
x_buffer2 = zeros(1, window_size);
y_buffer2 = zeros(1, window_size);
yaw_buffer2 = zeros(1, window_size);
buffer_index2 = 1;

x_positions2 = [];
y_positions2 = [];

integral_error_x2 = 0;
integral_error_y2 = 0;

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

% Main control loop for both robots
for i = 1:1000
    %% Control for the first robot
    % Receive messages from the /aruco_x, /aruco_y, and /aruco_yaw topics
    xMsg = receive(xSub, 1.2);
    yMsg = receive(ySub, 1.2);
    yawMsg = receive(yawSub, 1.2);

    % Extract data from the received messages
    x = double(xMsg.data);
    y = double(yMsg.data);
    yaw = double(yawMsg.data);

    % Update moving average buffers
    x_buffer(buffer_index) = x;
    y_buffer(buffer_index) = y;
    yaw_buffer(buffer_index) = yaw;
    buffer_index = mod(buffer_index, window_size) + 1;

    % Apply moving average filter
    x_filtered = mean(x_buffer);
    y_filtered = mean(y_buffer);
    yaw_filtered = mean(yaw_buffer);

    % Append filtered positions for plotting
    x_positions = [x_positions, x_filtered];
    y_positions = [y_positions, y_filtered];

    % Plot real-time position
    plot(x_positions, y_positions, 'b-', 'LineWidth', 2);

    % Calculate Errors
    error_x = x_target - x_filtered;
    error_y = y_target - y_filtered;

    distance_to_target = sqrt(error_x^2 + error_y^2);
    angle_to_target = atan2(error_y, error_x);

    % Calculate yaw error and normalize within [-pi, pi]
    error_yaw = angle_to_target - yaw_filtered;
    error_yaw = atan2(sin(error_yaw), cos(error_yaw));

    % Control law for linear and angular velocities
    v_linear = -max_linear_velocity * tanh(((kpd * distance_to_target)^3) / max_linear_velocity);
    v_angular = -max_angular_velocity * tanh((angular_Kp_base * error_yaw)^3 / max_angular_velocity);

    % Saturate velocities
    linear_velocity = max(min(v_linear, max_linear_velocity), -max_linear_velocity);
    angular_velocity = max(min(v_angular, max_angular_velocity), -max_angular_velocity);

    % Send velocity commands
    msg.linear.x = double(linear_velocity);
    msg.angular.z = double(angular_velocity);
    send(cmdVelPub, msg);

    %% Control for the second robot
    % Receive messages from the /aruco1_x, /aruco1_y, and /aruco1_yaw topics
    xMsg2 = receive(xSub2, 1.2);
    yMsg2 = receive(ySub2, 1.2);
    yawMsg2 = receive(yawSub2, 1.2);

    % Extract data from the received messages
    x2 = double(xMsg2.data);
    y2 = double(yMsg2.data);
    yaw2 = double(yawMsg2.data);

    % Update moving average buffers
    x_buffer2(buffer_index2) = x2;
    y_buffer2(buffer_index2) = y2;
    yaw_buffer2(buffer_index2) = yaw2;
    buffer_index2 = mod(buffer_index2, window_size) + 1;

    % Apply moving average filter
    x_filtered2 = mean(x_buffer2);
    y_filtered2 = mean(y_buffer2);
    yaw_filtered2 = mean(yaw_buffer2);

    % Append filtered positions for plotting
    x_positions2 = [x_positions2, x_filtered2];
    y_positions2 = [y_positions2, y_filtered2];

    % Plot real-time position
    plot(x_positions2, y_positions2, 'r-', 'LineWidth', 2);

    % Calculate Errors
    error_x2 = x_target2 - x_filtered2;
    error_y2 = y_target2 - y_filtered2;

    distance_to_target2 = sqrt(error_x2^2 + error_y2^2);
    angle_to_target2 = atan2(error_y2, error_x2);

    % Calculate yaw error and normalize within [-pi, pi]
    error_yaw2 = angle_to_target2 - yaw_filtered2;
    error_yaw2 = atan2(sin(error_yaw2), cos(error_yaw2));

    % Control law for linear and angular velocities
    v_linear2 = -max_linear_velocity * tanh(((kpd * distance_to_target2)^3) / max_linear_velocity);
    v_angular2 = -max_angular_velocity * tanh((angular_Kp_base * error_yaw2)^3 / max_angular_velocity);

    % Saturate velocities
    linear_velocity2 = max(min(v_linear2, max_linear_velocity), -max_linear_velocity);
    angular_velocity2 = max(min(v_angular2, max_angular_velocity), -max_angular_velocity);

    % Send velocity commands
    msg2.linear.x = double(linear_velocity2);
    msg2.angular.z = double(angular_velocity2);
    send(cmdVelPub2, msg2);

    % Stop conditions
    if distance_to_target < 0.01
        disp('First robot reached the target. Stopping...');
        break;
    end

    if distance_to_target2 < 0.01
        disp('Second robot reached the target. Stopping...');
        break;
    end

    % Pause for the next loop iteration
    drawnow;
    waitfor(rate);
end

% Stop both robots after reaching the targets
msg.linear.x = 0.0;
msg.angular.z = 0.0;
send(cmdVelPub, msg);
disp('First robot stopped.');

msg2.linear.x = 0.0;
msg2.angular.z = 0.0;
send(cmdVelPub2, msg2);
disp('Second robot stopped.');

% Clean up: Clear nodes and publishers
clear cmdVelPub node cmdVelPub2;

disp('Nodes and publishers cleared. Exiting.');
