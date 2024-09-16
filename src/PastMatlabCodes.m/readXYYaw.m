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

% Create subscribers for the x, y, and yaw topics
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
title('Real-time Robot Position from ArUco Marker');
grid on;
axis equal;

% Run loop to constantly receive and plot data from the topics
for i = 1:100
    % Receive messages from the /aruco_x, /aruco_y, and /aruco_yaw topics
    xMsg = receive(xSub, 3);
    yMsg = receive(ySub, 3);
    yawMsg = receive(yawSub, 3);

    % Extract position data from the received messages
    x = (xMsg.data);
    y = (yMsg.data);
    yaw = yawMsg.data;
    
    % Append the new positions to the arrays
    x_positions = [x_positions, x];
    y_positions = [y_positions, y];

    % Plot the robot's position in real-time
    plot(x_positions, y_positions, 'b-', 'LineWidth', 20);
    axis([-0.5 0.5 -0.5 0.5])
    drawnow;
    
    % Pause for a short time before the next update
    pause(0.1);
end

% Clean up: Clear the node and subscribers
clear xSub ySub yawSub node;

% Display a message indicating that the script has finished
disp('Node and subscribers have been cleared. Exiting.');
