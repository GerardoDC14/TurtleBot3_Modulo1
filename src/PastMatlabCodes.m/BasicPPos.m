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
% cmdVelPub = ros2publisher(node, '/cmd_vel', 'geometry_msgs/Twist');


% Create a Twist message
msg = ros2message(cmdVelPub);

% Create subscribers for the /aruco_x, /aruco_y, and /aruco_yaw topics
xSub = ros2subscriber(node, '/aruco_x', 'std_msgs/Float32');
ySub = ros2subscriber(node, '/aruco_y', 'std_msgs/Float32');
yawSub = ros2subscriber(node, '/aruco_yaw', 'std_msgs/Float32');

% Initialize arrays for position storage
x_positions = [];
y_positions = [];

% Create figure for real-time plotting
figure;
hold on;
xlabel('X [m]');
ylabel('Y [m]');
title('Real-time Robot Position');
grid on;
axis equal;

% Set velocities to move the robot forward
msg.linear.x =  0.0;  % Forward velocity (m/s)
msg.angular.z = 0.0; % No rotation
send(cmdVelPub, msg);
disp('Sent forward command');

% Run loop to constantly receive and plot data from the topics
for i = 1:100
    % Receive messages from the /aruco_x, /aruco_y, and /aruco_yaw topics
    xMsg = receive(xSub, 3);
    yMsg = receive(ySub, 3);
    yawMsg = receive(yawSub, 3);

    % Extract data from the received messages
    x = xMsg.data;
    y = yMsg.data;
    yaw = yawMsg.data;

    % Append the new positions to the arrays
    x_positions = [x_positions, x];
    y_positions = [y_positions, y];

    % Plot the robot's position in real-time
    plot(x_positions, y_positions, 'b-', 'LineWidth', 2);
    drawnow;

    % Pause for a short time before the next update 
    pause(0.1);
end

% Stop the robot after the loop
msg.linear.x = 0.0;
msg.angular.z = 0.0;
send(cmdVelPub, msg);
disp('Sent stop command');

% Clean up: Clear the node and publisher
clear cmdVelPub node;

% Display a message indicating that the script has finished
disp('Node and publisher have been cleared. Exiting.');
