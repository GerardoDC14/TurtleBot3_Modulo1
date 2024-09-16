clear all;
clc;

% Initialize ROS environment and domain ID
setenv('ROS_DOMAIN_ID', '0');

% List ROS topics to confirm connection
topics = ros2('topic', 'list');
disp('Available topics:');
disp(topics);

% Create a ROS node
node = ros2node('/matlab_node');

% Subscribe to the IMU topic
imuSub = ros2subscriber(node, '/imu', 'sensor_msgs/Imu');

% Crear un cliente TCP/IP
t = tcpclient('192.168.0.154', 80);

% Inicializar la figura
figure;
h = animatedline('MaximumNumPoints', 100);
ax = gca;
ax.YGrid = 'on';
ax.YLim = [-180 180];

% Leer y graficar los datos y enviarlos a la ESP32
while true
    imuMsg = receive(imuSub, 10); % Esperar hasta 10 segundos para recibir un mensaje
    if ~isempty(imuMsg)
        orientation = imuMsg.Orientation;
        addpoints(h, now, rad2deg(orientation.X)); % Graficar solo el eje X por simplicidad
        datetick('x', 'keeplimits');
        drawnow;
        
        % Enviar datos a la ESP32
        data = sprintf('Orientation: X: %.2f Y: %.2f Z: %.2f\n', rad2deg(orientation.X), rad2deg(orientation.Y), rad2deg(orientation.Z));
        write(t, data);
    end
    pause(1);
end

% Cerrar la conexión
clear t;
