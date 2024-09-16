clear all
clear
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
x_target = 0.02; % Target x position [meters]
y_target = 0.0; % Target y position [meters]

% Proportional control gains
Kp_linear = 0.5;  % Gain for linear velocity
Kp_angular = 0.8;  % Gain for angular velocity

% Initialize arrays for position storage (optional for plotting)
x_positions = [];
y_positions = [];

% Initialize moving average filter variables
window_size = 10;  % Size of the moving average window
x_buffer = zeros(1, window_size);  % Buffer to store the recent x positions
y_buffer = zeros(1, window_size);  % Buffer to store the recent y positions
buffer_index = 1;  % Index for circular buffer

% Create figure for real-time plotting (optional)
figure;
hold on;
xlabel('X [m]');
ylabel('Y [m]');
title('Real-time Robot Position');
grid on;
axis equal;

% Increase the frequency of the control loop
rate = ros2rate(node, 20); % Process at 20 Hz

% Run loop to constantly receive and control the robot
for i = 1:1000
    % Receive messages from the /aruco_x, /aruco_y, and /aruco_yaw topics
    xMsg = receive(xSub, 3);
    yMsg = receive(ySub, 3);
    yawMsg = receive(yawSub, 3);

    % Extract data from the received messages and ensure they're double
    x = double(xMsg.data);
    y = double(yMsg.data);
    yaw = double(yawMsg.data);

    % Update the moving average buffers
    x_buffer(buffer_index) = x;
    y_buffer(buffer_index) = y;
    buffer_index = mod(buffer_index, window_size) + 1;  % Update the circular buffer index

    % Apply moving average filter
    x_filtered = mean(x_buffer);
    y_filtered = mean(y_buffer);

    % Append the new filtered positions to the arrays (for plotting)
    x_positions = [x_positions, x_filtered];
    y_positions = [y_positions, y_filtered];

    % Plot the robot's position in real-time
    plot(x_positions, y_positions, 'b-', 'LineWidth', 2);
    drawnow;

    % Proportional control logic

    % Calculate the error in position
    error_x = x_target - x_filtered;
    error_y = y_target - y_filtered;

    % Calculate the distance to the target
    distance_to_target = sqrt(error_x^2 + error_y^2);

    % Calculate the angle to the target
    angle_to_target = atan2(error_y, error_x);

    % Calculate the angular error (difference between current orientation and target direction)
    error_yaw = angle_to_target - yaw;

    % Normalize the angular error to be within [-pi, pi]
    error_yaw = atan2(sin(error_yaw), cos(error_yaw));

    % Calculate linear and angular velocities based on proportional control
    linear_velocity = Kp_linear * distance_to_target;
    angular_velocity = Kp_angular * error_yaw;

    % Limit the maximum velocities to avoid instability (optional)
    linear_velocity = min(linear_velocity, 0.2);  % Limit max linear velocity
    angular_velocity = min(max(angular_velocity, -1.0), 1.0);  % Limit angular velocity between -1 and 1 rad/s

    % Set the velocities in the Twist message, ensuring values are doubles
    msg.linear.x = double(linear_velocity);
    msg.angular.z = double(angular_velocity);

    % Send the velocity command to the robot
    send(cmdVelPub, msg);

    % Wait for the next cycle (faster processing due to higher rate)
    waitfor(rate);

    % Stop if the robot is close enough to the target
    if distance_to_target < 0.005 % Threshold for reaching the target
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

