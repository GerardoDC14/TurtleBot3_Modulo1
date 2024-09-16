clear all;
clc;

% Inicializa el entorno ROS 2 y establece el ID de dominio
setenv('ROS_DOMAIN_ID', '0');

% Lista los tópicos de ROS 2 para confirmar la conexión
topics = ros2('topic', 'list');
disp('Available topics:');
disp(topics);

% Crea un nodo ROS 2
node = ros2node('/matlab_node');

% Suscribirse al tópico de IMU '/tb3_0/imu'
imuSub = ros2subscriber(node, '/tb3_0/imu', 'sensor_msgs/Imu');

% Dirección IP y puerto del servidor ESP32
serverIP = '192.168.0.214'; % Reemplaza con la IP de tu ESP32
serverPort = 80;

% Crear un cliente TCP y mantener la conexión abierta
t = tcpclient(serverIP, serverPort);

% Inicializar las figuras para graficar
figure('Name', 'Datos IMU en MATLAB');

% Graficar Orientación (Theta)
subplot(3,1,1);
hTheta = animatedline('Color', 'r');
title('Orientación Theta (grados)');
xlabel('Tiempo');
ylabel('Theta');
grid on;
ylim([-180 180]);

% Graficar Velocidad Angular (eje X)
subplot(3,1,2);
hAngVel = animatedline('Color', 'g');
title('Velocidad Angular X (rad/s)');
xlabel('Tiempo');
ylabel('Angular Velocity X');
grid on;
ylim([-10 10]); % Ajusta según el rango esperado

% Graficar Aceleración Lineal (eje X)
subplot(3,1,3);
hLinAcc = animatedline('Color', 'b');
title('Aceleración Lineal X (m/s²)');
xlabel('Tiempo');
ylabel('Linear Acceleration X');
grid on;
ylim([-20 20]); % Ajusta según el rango esperado

% Variables para controlar el tiempo en el eje X
startTime = datetime('now');

% Bucle principal para leer, enviar y graficar datos
while ishandle(gcf) % Mantener el bucle mientras la figura esté abierta
    try
        imuMsg = receive(imuSub, 1); % Esperar hasta 1 segundo para recibir un mensaje
        if ~isempty(imuMsg)
            % Obtener datos de orientación (cuaternión)
            quat = imuMsg.orientation;
            angles = quat2eul([quat.w quat.x quat.y quat.z]);
            theta = rad2deg(angles(1)); % Extraer Theta
            
            % Obtener velocidad angular
            angVelX = imuMsg.angular_velocity.x;
            
            % Obtener aceleración lineal
            linAccX = imuMsg.linear_acceleration.x;
            
            % Mostrar datos en la consola
            disp(['Theta: ', num2str(theta), ' grados']);
            disp(['Angular Velocity X: ', num2str(angVelX), ' rad/s']);
            disp(['Linear Acceleration X: ', num2str(linAccX), ' m/s²']);
            
            % Preparar datos para enviar
            % Formato CSV: theta,angVelX,linAccX
            dataToSend = sprintf('%.2f,%.2f,%.2f\n', theta, angVelX, linAccX);
            write(t, dataToSend); % Enviar los datos al ESP32
            
            % Obtener el tiempo actual para el gráfico
            currentTime = datetime('now') - startTime;
            timeInSeconds = seconds(currentTime);
            
            % Graficar Theta
            subplot(3,1,1);
            addpoints(hTheta, timeInSeconds, theta);
            xlim([max(timeInSeconds - 60, 0) max(timeInSeconds, 60)]); % Ventana de 60 segundos
            drawnow;
            
            % Graficar Velocidad Angular X
            subplot(3,1,2);
            addpoints(hAngVel, timeInSeconds, angVelX);
            xlim([max(timeInSeconds - 60, 0) max(timeInSeconds, 60)]);
            drawnow;
            
            % Graficar Aceleración Lineal X
            subplot(3,1,3);
            addpoints(hLinAcc, timeInSeconds, linAccX);
            xlim([max(timeInSeconds - 60, 0) max(timeInSeconds, 60)]);
            drawnow;
        end
    catch ME
        disp('Error occurred:');
        disp(ME.message);
    end
    pause(0.1); % Controla la frecuencia de actualización
end

% Cerrar la conexión TCP al cerrar la figura
if exist('t', 'var')
    clear t;
end

% Limpiar el nodo ROS 2 al finalizar
clear imuSub node;
