clear all;
clc;

setenv('ROS_DOMAIN_ID', '0');

topics = ros2('topic', 'list');
disp('Available topics:');
disp(topics);

node = ros2node('/matlab_node');

cmdVelPub = ros2publisher(node, '/tb3_0/cmd_vel', 'geometry_msgs/Twist');
msg = ros2message(cmdVelPub); 

xSub = ros2subscriber(node, '/aruco1_x', 'std_msgs/Float32');
ySub = ros2subscriber(node, '/aruco1_y', 'std_msgs/Float32');
yawSub = ros2subscriber(node, '/aruco1_yaw', 'std_msgs/Float32');

x_target = 0.2;  
y_target = 0.0;  

cmdVelPub2 = ros2publisher(node, '/cmd_vel', 'geometry_msgs/Twist');
msg2 = ros2message(cmdVelPub2);

xSub2 = ros2subscriber(node, '/aruco1_x', 'std_msgs/Float32');
ySub2 = ros2subscriber(node, '/aruco1_y', 'std_msgs/Float32');
yawSub2 = ros2subscriber(node, '/aruco1_yaw', 'std_msgs/Float32');

% Desired target position (goal) for the second robot
x_target2 = -0.2;  % Target x position [meters]
y_target2 = 0.0;   % Target y position [meters]

% Control gains (shared between both robots)
Kp_base = 0.8;  % Increased proportional gain to respond faster
Kp_nonlinear = 1.0;
Ki = 0.02;  % Integral gain to reduce windup
Kd = 0.3;  % Slightly reduced derivative gain for more aggressive corrections
kpd = 3.5;  % Nonlinear gain for the distance
angular_Kp_base = 0.6;  % Increased angular proportional gain for faster corrections

% Threshold for stopping when close to the target
distance_threshold = 0.05;  % Stop control when within 5 cm of the target
angle_threshold = 0.1;      % Stop turning when aligned within a small angle

% Maximum velocity limits
max_linear_velocity = 0.08;  % Increased linear velocity limit for faster motion
max_angular_velocity = 0.5;  % Increased angular velocity for faster turning

% Initialize buffers and positions for both robots
window_size = 5;  % Reduced window size for quicker response

% For robot 1
x_buffer = zeros(1, window_size);
y_buffer = zeros(1, window_size);
yaw_buffer = zeros(1, window_size);
buffer_index = 1;

x_positions = [];
y_positions = [];

integral_error_x = 0;
integral_error_y = 0;
integral_windup_limit = 0.02;  % Reducing integral windup limit

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
rate = ros2rate(node, 100);  % Control loop frequency at 100 Hz

% Initialize flags to track if both robots have reached their target
robot1_reached = false;
robot2_reached = false;

% Main control loop for both robots
for i = 1:1000
    %% Control for the first robot
    if ~robot1_reached
        % Receive messages from the /aruco_x, /aruco_y, and /aruco_yaw topics
        xMsg = receive(xSub, 10);
        yMsg = receive(ySub, 10);
        yawMsg = receive(yawSub, 10);

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

        % Slow down as we get closer to the target
        if distance_to_target < 0.2
            max_linear_velocity = 0.04;
            max_angular_velocity = 0.2;
        else
            max_linear_velocity = 0.08;
            max_angular_velocity = 0.5;
        end

        % Control law for linear and angular velocities
        v_linear = max_linear_velocity * tanh((kpd * distance_to_target) / (2 * max_linear_velocity));
        v_angular = max_angular_velocity * tanh((angular_Kp_base * error_yaw) / max_angular_velocity);

        % Saturate velocities
        linear_velocity = max(min(v_linear, max_linear_velocity), -max_linear_velocity);
        angular_velocity = max(min(v_angular, max_angular_velocity), -max_angular_velocity);

        % Send velocity commands
        msg.linear.x = double(linear_velocity);
        msg.angular.z = double(angular_velocity);
        send(cmdVelPub, msg);

        % Check if the first robot reached its target within the threshold
        if distance_to_target < distance_threshold && abs(error_yaw) < angle_threshold
            disp('First robot reached the target.');
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;
            send(cmdVelPub, msg);
            robot1_reached = true;
        end
    end

    %% Control for the second robot
    if ~robot2_reached
        % Receive messages from the /aruco1_x, /aruco1_y, and /aruco1_yaw topics
        xMsg2 = receive(xSub2, 10);
        yMsg2 = receive(ySub2, 10);
        yawMsg2 = receive(yawSub2, 10);

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

        % Slow down as we get closer to the target
        if distance_to_target2 < 0.2
            max_linear_velocity = 0.04;
            max_angular_velocity = 0.2;
        else
            max_linear_velocity = 0.08;
            max_angular_velocity = 0.5;
        end

        % Control law for linear and angular velocities
        v_linear2 = max_linear_velocity * tanh((kpd * distance_to_target2) / (2 * max_linear_velocity));
        v_angular2 = max_angular_velocity * tanh((angular_Kp_base * error_yaw2) / max_angular_velocity);

        % Saturate velocities
        linear_velocity2 = max(min(v_linear2, max_linear_velocity), -max_linear_velocity);
        angular_velocity2 = max(min(v_angular2, max_angular_velocity), -max_angular_velocity);

        % Send velocity commands
        msg2.linear.x = double(linear_velocity2);
        msg2.angular.z = double(angular_velocity2);
        send(cmdVelPub2, msg2);

        % Check if the second robot reached its target
        if distance_to_target2 < distance_threshold && abs(error_yaw2) < angle_threshold
            disp('Second robot reached the target.');
            msg2.linear.x = 0.0;
            msg2.angular.z = 0.0;
            send(cmdVelPub2, msg2);
            robot2_reached = true;
        end
    end

    % Stop if both robots have reached their targets
    if robot1_reached && robot2_reached
        disp('Both robots reached their targets. Stopping...');
        break;
    end

    % Pause for the next loop iteration
    drawnow;
    waitfor(rate);
end

% Stop both robots after reaching the targets
disp('Robots stopped.');

% Clean up: Clear nodes and publishers
clear cmdVelPub node cmdVelPub2;

disp('Nodes and publishers cleared. Exiting.');