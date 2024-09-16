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

% ------------------------ LEADER SETUP ------------------------
% Define the radius and center of the virtual leader's circular trajectory
leader_radius = 0.3;  % Radius for the leader's trajectory
leader_center_x = 0.1;  % Center position of the leader's circular path
leader_center_y = 0.1;

% Generate Circle Trajectory for the virtual leader
num_samples = 800;  % Number of points to sample on the circle
theta = linspace(0, 2*pi, num_samples);  % Angle range for the circle
x_leader = leader_center_x + leader_radius * cos(theta);  % X coordinates of leader path
y_leader = leader_center_y + leader_radius * sin(theta);  % Y coordinates of leader path

% ------------------------ ROBOT 1 SETUP (tb3_6) ------------------------
% Create a publisher for the first robot (/tb3_6/cmd_vel topic)
cmdVelPub1 = ros2publisher(node, '/tb3_6/cmd_vel', 'geometry_msgs/Twist');
msg1 = ros2message(cmdVelPub1); % Twist message for the first robot

% Subscribe to odometry topic (/tb3_6/odom) for robot 1
odomSub1 = ros2subscriber(node, '/tb3_6/odom', 'nav_msgs/Odometry');

% Subscribe to Aruco global positioning topics for robot 1
xArucoSub1 = ros2subscriber(node, '/aruco_x', 'std_msgs/Float32');
yArucoSub1 = ros2subscriber(node, '/aruco_y', 'std_msgs/Float32');
yawArucoSub1 = ros2subscriber(node, '/aruco_yaw', 'std_msgs/Float32');

% ------------------------ ROBOT 2 SETUP (cmd_vel) ------------------------
% Create a publisher for the second robot (/cmd_vel topic)
cmdVelPub2 = ros2publisher(node, '/cmd_vel', 'geometry_msgs/Twist');
msg2 = ros2message(cmdVelPub2); % Twist message for the second robot

% Subscribe to odometry topic (/odom) for robot 2
odomSub2 = ros2subscriber(node, '/odom', 'nav_msgs/Odometry');

% Subscribe to Aruco global positioning topics for robot 2
xArucoSub2 = ros2subscriber(node, '/aruco1_x', 'std_msgs/Float32');
yArucoSub2 = ros2subscriber(node, '/aruco1_y', 'std_msgs/Float32');
yawArucoSub2 = ros2subscriber(node, '/aruco1_yaw', 'std_msgs/Float32');

% ------------------------ PID CONTROL PARAMETERS ------------------------
% PID control parameters for both robots
Kp_angle = 1.0;  % Proportional gain for angle
Ki_angle = 0.02; % Integral gain for angle
Kd_angle = 0;    % Derivative gain for angle

Kp_distance = 0.8;  % Proportional gain for distance
Ki_distance = 0.01; % Integral gain for distance
Kd_distance = 0;    % Derivative gain for distance

% Set manual limits for velocity output
max_angular_velocity = 0.2;  % Maximum angular velocity (rad/s)
max_linear_velocity = 0.4;   % Maximum linear velocity (m/s)

% Tolerance for reaching the desired point
tolerance = 0.05;  % Distance tolerance to stop the robot

% Distance offset for the follower robots (distance from the leader)
robot1_offset_distance = 0.3;  % Distance to maintain from the leader for Robot 1
robot2_offset_distance = 0.3;  % Distance to maintain from the leader for Robot 2

% Initialize PID control terms
prev_angle_error1 = 0; integral_angle1 = 0;
prev_distance_error1 = 0; integral_distance1 = 0;

prev_angle_error2 = 0; integral_angle2 = 0;
prev_distance_error2 = 0; integral_distance2 = 0;

% Initialize the figure for plotting the leader and robots' movements
figure;
hold on;
plot(x_leader, y_leader, 'g--'); % Plot leader's circular path in green dashed line
robot1_plot = plot(0, 0, 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b'); % Plot robot 1 position in blue
robot2_plot = plot(0, 0, 'co', 'MarkerSize', 8, 'MarkerFaceColor', 'c'); % Plot robot 2 position in cyan
leader_plot = plot(0, 0, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');  % Plot virtual leader position in red
xlabel('X [m]');
ylabel('Y [m]');
title('Robots Following Virtual Leader with Sensor Fusion');
grid on;
axis equal;

% Main control loop
goal_index = 1;  % Start at the first point on the leader's circular path
rate = ros2rate(node, 20);  % Control loop frequency

while goal_index <= num_samples
    % ------------------------ VIRTUAL LEADER MOVEMENT ------------------------
    % Update the virtual leader's position
    leader_x = x_leader(goal_index);
    leader_y = y_leader(goal_index);
    set(leader_plot, 'XData', leader_x, 'YData', leader_y);  % Update leader position on plot
    
    % ------------------------ ROBOT 1 CONTROL USING SENSOR FUSION ------------------------
    % Receive global Aruco position and yaw data for robot 1
    xArucoMsg1 = receive(xArucoSub1, 1);  % Aruco x position for robot 1
    yArucoMsg1 = receive(yArucoSub1, 1);  % Aruco y position for robot 1
    yawArucoMsg1 = receive(yawArucoSub1, 1);  % Aruco yaw for robot 1

    x1 = double(xArucoMsg1.data);  % Use Aruco position
    y1 = double(yArucoMsg1.data);
    yaw1 = double(yawArucoMsg1.data);  % Use Aruco yaw
    
    % Calculate desired position offset for robot 1 (relative to the leader)
    goal_x1 = leader_x - robot1_offset_distance * cos(yaw1);  % Keep a distance from the leader
    goal_y1 = leader_y - robot1_offset_distance * sin(yaw1);
    
    % Compute the error in position for robot 1
    inc_x1 = goal_x1 - x1;
    inc_y1 = goal_y1 - y1;
    distance_to_goal1 = sqrt(inc_x1^2 + inc_y1^2);
    angle_to_goal1 = atan2(inc_y1, inc_x1);
    
    % PID control for angle (robot 1)
    angle_error1 = angle_to_goal1 - yaw1;
    angle_error1 = atan2(sin(angle_error1), cos(angle_error1));  % Normalize angle error to [-pi, pi]
    integral_angle1 = integral_angle1 + angle_error1;
    derivative_angle1 = angle_error1 - prev_angle_error1;
    angular_velocity1 = Kp_angle * angle_error1 + Ki_angle * integral_angle1 + Kd_angle * derivative_angle1;
    prev_angle_error1 = angle_error1;
    angular_velocity1 = max(min(angular_velocity1, max_angular_velocity), -max_angular_velocity);
    
    % PID control for distance (robot 1)
    if abs(angle_error1) < 0.2  % Only move forward if the heading is correct
        distance_error1 = distance_to_goal1;
        integral_distance1 = integral_distance1 + distance_error1;
        derivative_distance1 = distance_error1 - prev_distance_error1;
        linear_velocity1 = Kp_distance * distance_error1 + Ki_distance * integral_distance1 + Kd_distance * derivative_distance1;
        prev_distance_error1 = distance_error1;
    else
        linear_velocity1 = 0;  % Stop forward movement while adjusting angle
    end
    linear_velocity1 = max(min(linear_velocity1, max_linear_velocity), -max_linear_velocity);
    
    % Set velocity message for robot 1
    msg1.linear.x = linear_velocity1;
    msg1.angular.z = angular_velocity1;
    send(cmdVelPub1, msg1);  % Send velocity commands to robot 1
    
    % Update the plot for robot 1
    set(robot1_plot, 'XData', x1, 'YData', y1);
    
    % ------------------------ ROBOT 2 CONTROL USING SENSOR FUSION ------------------------
    % Receive global Aruco position and yaw data for robot 2
    xArucoMsg2 = receive(xArucoSub2, 1);  % Aruco x position for robot 2
    yArucoMsg2 = receive(yArucoSub2, 1);  % Aruco y position for robot 2
    yawArucoMsg2 = receive(yawArucoSub2, 1);  % Aruco yaw for robot 2

    x2 = double(xArucoMsg2.data);  % Use Aruco position
    y2 = double(yArucoMsg2.data);
    yaw2 = double(yawArucoMsg2.data);  % Use Aruco yaw
    
    % Calculate desired position offset for robot 2 (relative to the leader)
    goal_x2 = leader_x - robot2_offset_distance * cos(yaw2);  % Keep a distance from the leader
    goal_y2 = leader_y - robot2_offset_distance * sin(yaw2);
    
    % Compute the error in position for robot 2
    inc_x2 = goal_x2 - x2;
    inc_y2 = goal_y2 - y2;
    distance_to_goal2 = sqrt(inc_x2^2 + inc_y2^2);
    angle_to_goal2 = atan2(inc_y2, inc_x2);
    
    % PID control for angle (robot 2)
    angle_error2 = angle_to_goal2 - yaw2;
    angle_error2 = atan2(sin(angle_error2), cos(angle_error2));  % Normalize angle error to [-pi, pi]
    integral_angle2 = integral_angle2 + angle_error2;
    derivative_angle2 = angle_error2 - prev_angle_error2;
    angular_velocity2 = Kp_angle * angle_error2 + Ki_angle * integral_angle2 + Kd_angle * derivative_angle2;
    prev_angle_error2 = angle_error2;
    angular_velocity2 = max(min(angular_velocity2, max_angular_velocity), -max_angular_velocity);
    
    % PID control for distance (robot 2)
    if abs(angle_error2) < 0.2  % Only move forward if the heading is correct
        distance_error2 = distance_to_goal2;
        integral_distance2 = integral_distance2 + distance_error2;
        derivative_distance2 = distance_error2 - prev_distance_error2;
        linear_velocity2 = Kp_distance * distance_error2 + Ki_distance * integral_distance2 + Kd_distance * derivative_distance2;
        prev_distance_error2 = distance_error2;
    else
        linear_velocity2 = 0;  % Stop forward movement while adjusting angle
    end
    linear_velocity2 = max(min(linear_velocity2, max_linear_velocity), -max_linear_velocity);
    
    % Set velocity message for robot 2
    msg2.linear.x = linear_velocity2;
    msg2.angular.z = angular_velocity2;
    send(cmdVelPub2, msg2);  % Send velocity commands to robot 2
    
    % Update the plot for robot 2
    set(robot2_plot, 'XData', x2, 'YData', y2);
    
    % ------------------------ UPDATE AND MOVE TO NEXT LEADER POINT ------------------------
    % Check if the robots need to update their goal points
    if distance_to_goal1 < tolerance && distance_to_goal2 < tolerance
        goal_index = goal_index + 1;  % Move to the next point on the leader's path
    end
    
    % Pause for the next loop iteration
    drawnow;
    waitfor(rate);
end

% Stop both robots after completing their paths
msg1.linear.x = 0; msg1.angular.z = 0;
send(cmdVelPub1, msg1);
msg2.linear.x = 0; msg2.angular.z = 0;
send(cmdVelPub2, msg2);
disp('Both robots stopped. Path completed.');

% Clean up: Clear nodes and publishers
clear cmdVelPub1 cmdVelPub2 node;

disp('Nodes and publishers cleared. Exiting.');
