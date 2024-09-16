clear all;
clc;

% Initialize ROS 2 environment and domain ID
setenv('ROS_DOMAIN_ID', '0');

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

% Control gains
kpr = 5;  % Proportional gain for angular velocity
kpt = 10; % Proportional gain for linear velocity
kgamma = 10;  % Nonlinear adjustment for velocity
v_max = 0.05;  % Maximum linear velocity
w_max = 0.3;   % Maximum angular velocity

% Time parameters
t = 0;   % Start time
dt = 0.05;  % Time step (50 ms)
total_tf = 10;  % Total time for the trajectory

% Polynomial trajectory coefficients for a single target point
x_target = 0.45;  % Desired target x position
y_target = -0.158;  % Desired target y position
[A, B, C, D, E, F] = poldef(0, x_target, 0, 0, 0, 0, total_tf);  % Polynomial for x
[Ay, By, Cy, Dy, Ey, Fy] = poldef(0, y_target, 0, 0, 0, 0, total_tf);  % Polynomial for y

% Control loop frequency (rate control)
rate = ros2rate(node, 20);  % 20 Hz control loop

while t < total_tf
    % Generate the desired trajectory positions (xd, yd) using polynomial interpolation
    xd = A*t.^5 + B*t.^4 + C*t.^3 + D*t.^2 + E*t + F;  % Desired x position
    yd = Ay*t.^5 + By*t.^4 + Cy*t.^3 + Dy*t.^2 + Ey*t + Fy;  % Desired y position
    
    % Fetch current robot position and orientation from the ROS topics
    xMsg = receive(xSub, 3);
    yMsg = receive(ySub, 3);
    yawMsg = receive(yawSub, 3);

    % Extract position and orientation data from the received messages
    x = double(xMsg.data);
    y = double(yMsg.data);
    theta = double(yawMsg.data);  % Current robot orientation (yaw)

    % Compute the angle to the desired point
    thetad = atan2((yd - y), (xd - x));  % Desired heading to the target point
    thetae = thetad - theta;  % Orientation error

    % Normalize the angular error to the range [-pi, pi]
    if thetae > pi
        thetae = thetae - 2*pi;
    elseif thetae < -pi
        thetae = thetae + 2*pi;
    end

    % Compute the distance to the target point
    d = sqrt((xd - x)^2 + (yd - y)^2);

    % Nonlinear velocity scaling based on orientation error
    gamma = -kgamma * abs(thetae) + 1;  % Gamma scaling factor

    % Compute linear and angular velocities
    V = min(v_max, gamma * kpt * d);  % Linear velocity scaled by the distance
    w = min(w_max, kpr * thetae);  % Angular velocity based on orientation error

    % Update ROS 2 Twist message with computed velocities
    msg.linear.x = V;
    msg.angular.z = w;

    % Publish the velocity command to the /cmd_vel topic
    send(cmdVelPub, msg);

    % Increment time step
    t = t + dt;
    
    % Wait for the next control loop iteration
    waitfor(rate);
end

% Stop the robot at the end of the trajectory
msg.linear.x = 0.0;
msg.angular.z = 0.0;
send(cmdVelPub, msg);

% Clean up: Clear the node and publisher
clear cmdVelPub node;

disp('Trajectory complete, robot stopped.');

% Polynomial definition function
function [A,B,C,D,E,F] = poldef(p0, pf, v0, vf, a0, af, tf)
    M = [1, 0, 0, 0, 0, 0;
         1, tf, tf^2, tf^3, tf^4, tf^5;
         0, 1, 0, 0, 0, 0;
         0, 1, 2*tf, 3*tf^2, 4*tf^3, 5*tf^4;
         0, 0, 2, 0, 0, 0;
         0, 0, 2, 6*tf, 12*tf^2, 20*tf^3];
    R = [p0; pf; v0; vf; a0; af];
    Coeffs = M\R;  % Solve the system of equations to get the coefficients
    
    A = Coeffs(6);
    B = Coeffs(5);
    C = Coeffs(4);
    D = Coeffs(3);
    E = Coeffs(2);
    F = Coeffs(1);
end
