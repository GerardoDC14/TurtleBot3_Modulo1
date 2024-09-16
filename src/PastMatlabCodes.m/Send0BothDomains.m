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

% Create a publisher for the robot (/cmd_vel topic)
cmdVelPub = ros2publisher(node, '/tb3_6/cmd_vel', 'geometry_msgs/Twist');
msg = ros2message(cmdVelPub); % Twist message for the robot

% Subscribe to aruco_x and aruco_y topics for initial position
xArucoSub = ros2subscriber(node, '/aruco_x', 'std_msgs/Float32');
yArucoSub = ros2subscriber(node, '/aruco_y', 'std_msgs/Float32');

% Subscribe to odometry topic (/odom)
odomSub = ros2subscriber(node, '/tb3_6/odom', 'nav_msgs/Odometry');

% Wait to receive the initial position from the aruco topics
disp('Waiting for initial position from /aruco_x and /aruco_y...');
xArucoMsg = receive(xArucoSub, 10);
yArucoMsg = receive(yArucoSub, 10);
initial_x = double(xArucoMsg.data);
initial_y = double(yArucoMsg.data);
disp(['Initial Position: X = ', num2str(initial_x), ', Y = ', num2str(initial_y)]);

% Define the image center coordinates
image_center_x = 0.27*4;
image_center_y = 0.17;

% Calculate the offset of the robot's initial position from the image center
offset_x = initial_x - image_center_x;
offset_y = initial_y - image_center_y;

% Define the radius of the circular trajectory
circle_radius = 0.2;

% Generate Circle Trajectory around the image center
num_samples = 800;     % Number of points to sample on the circle
theta = linspace(0, 2*pi, num_samples);  % Angle range for the circle
x_circle = image_center_x + circle_radius * cos(theta);  % X coordinates
y_circle = image_center_y + circle_radius * sin(theta);  % Y coordinates

% PID control parameters (adjusted for faster response)
Kp_angle = 1.0;  % Increased Proportional gain for angle
Ki_angle = 0.02; % Integral gain for angle (decreased to avoid windup)
Kd_angle = 0;    % Derivative gain for angle

Kp_distance = 0.8;  % Increased Proportional gain for distance
Ki_distance = 0.01; % Integral gain for distance
Kd_distance = 0;    % Derivative gain for distance

% Initialize PID control terms
prev_angle_error = 0;
integral_angle = 0;

prev_distance_error = 0;
integral_distance = 0;

% Set manual limits for output (saturating values)
max_angular_velocity = 0.6;  % Increased maximum angular velocity
max_linear_velocity = 0.4;   % Increased maximum linear velocity

% Tolerance for reaching a point
tolerance = 0.05;  % Distance tolerance to stop the robot

% Initialize the figure for plotting the circle and robot movement
figure;
hold on;
plot(x_circle, y_circle, 'g--'); % Plot circle path in green dashed line
robot_plot = plot(0, 0, 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b'); % Plot robot position in blue
goal_plot = plot(0, 0, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');  % Plot goal point in red
xlabel('X [m]');
ylabel('Y [m]');
title('Robot Following Circular Path');
grid on;
axis equal;

% Main control loop
goal_index = 1;  % Start at the first point on the circle
rate = ros2rate(node, 20);  % 20 Hz control loop frequency for faster updates

while goal_index <= num_samples
    % Receive the current position and orientation from the odometry data
    odomMsg = receive(odomSub, 10);
    pose = odomMsg.pose.pose;
    
    % Extract position and orientation
    x = double(pose.position.x);
    y = double(pose.position.y);
    
    % Extract yaw (orientation in radians)
    quat = pose.orientation;
    angles = quat2eul([quat.w, quat.x, quat.y, quat.z]);
    yaw = angles(1);  % Extract the yaw (rotation around Z-axis)
    
    % Calculate the goal point (next point on the circle)
    goal_x = x_circle(goal_index);
    goal_y = y_circle(goal_index);
    
    % Compute the error in position
    inc_x = goal_x - x;
    inc_y = goal_y - y;
    
    % Calculate the distance to the goal and the angle to the goal
    distance_to_goal = sqrt(inc_x^2 + inc_y^2);
    angle_to_goal = atan2(inc_y, inc_x);
    
    % PID control for angle (turning)
    angle_error = angle_to_goal - yaw;
    angle_error = atan2(sin(angle_error), cos(angle_error));  % Normalize to [-pi, pi]
    
    % PID for angular velocity
    integral_angle = integral_angle + angle_error;
    derivative_angle = angle_error - prev_angle_error;
    angular_velocity = Kp_angle * angle_error + Ki_angle * integral_angle + Kd_angle * derivative_angle;
    prev_angle_error = angle_error;
    
    % Saturate the angular velocity
    angular_velocity = max(min(angular_velocity, max_angular_velocity), -max_angular_velocity);
    
    % PID control for distance (moving forward)
    if abs(angle_error) < 0.2  % Only move forward if heading is fairly correct
        distance_error = distance_to_goal;
        integral_distance = integral_distance + distance_error;
        derivative_distance = distance_error - prev_distance_error;
        linear_velocity = Kp_distance * distance_error + Ki_distance * integral_distance + Kd_distance * derivative_distance;
        prev_distance_error = distance_error;
    else
        linear_velocity = 0;  % Stop forward movement while adjusting angle
    end
    
    % Saturate the linear velocity
    linear_velocity = max(min(linear_velocity, max_linear_velocity), -max_linear_velocity);
    
    % Set the velocity message
    msg.linear.x = linear_velocity;
    msg.angular.z = angular_velocity;
    
    % Send the velocity commands to the robot
    send(cmdVelPub, msg);
    
    % Update the plot with the current robot position and goal
    set(robot_plot, 'XData', x, 'YData', y);  % Update robot position
    set(goal_plot, 'XData', goal_x, 'YData', goal_y);  % Update goal point
    drawnow;  % Update the plot in real-time
    
    % Check if the robot is close enough to the goal to move to the next point
    if distance_to_goal < tolerance
        goal_index = goal_index + 1;
        disp(['Reached point ', num2str(goal_index)]);
    end
    
    % Pause for the next loop iteration
    waitfor(rate);
end

% Stop the robot after completing the path
msg.linear.x = 0;
msg.angular.z = 0;
send(cmdVelPub, msg);
disp('Robot stopped. Path completed.');

% Clean up: Clear nodes and publishers
clear cmdVelPub node;

disp('Nodes and publishers cleared. Exiting.');
