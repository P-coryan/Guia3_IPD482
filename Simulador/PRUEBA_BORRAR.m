clc
clear
close all

load('Datos')
Laser = Datos.Laser;
robot = Datos.robot;
M = Datos.postes;
nubePtos = Datos.nubePtos;


% Laser a coordenadas globales
% Matrices de traslacion y rotacion global
T = @(tx,ty) [1 0 tx; 0 1 ty; 0 0 1];
R = @(tita) [cos(tita) -sin(tita); sin(tita) cos(tita)];

Laser(2,:) = deg2rad(Laser(2,:)) - pi/2; %angulo grados a radianes
[ptoX, ptoY] = pol2cart(Laser(2,:), Laser(1,:));    % ptos cartesianos
Laser = [ptoX ; ptoY];
newCord = R(robot.tita) * [Laser(1,:);Laser(2,:)];
newCord = T(robot.x, robot.y) * [newCord; ones(1,181)]; 
Laser = newCord(1:2,:);


% Parámetros del filtro de Kalman
dt = Datos.pasoTiempo;               % Intervalo de tiempo
A = [1 dt 0 0 0; 0 1 0 0 0; 0 0 1 dt 0; 0 0 0 1 0; 0 0 0 0 1];   % Matriz de transición de estado
B = eye(5);             % Matriz de control de entrada
C = [1 0 0 0 0; 0 0 1 0 0]; % Matriz de observación

% Ruido del proceso y de la medición
Q = 0.1*eye(5);         % Covarianza del ruido del proceso
R = 1*eye(2);           % Covarianza del ruido de la medición

% Inicialización de las variables del filtro
x = [0; 0; 0; 0; 0];    % Estado inicial [posición_x; velocidad_x; posición_y; velocidad_y; ángulo_theta]
P = eye(5);             % Covarianza inicial del estado

% Datos del LiDAR
lidar_data = nubePtos;
% lidar_data = Laser';     % Datos del LiDAR en formato [distancia_x, distancia_y]

plot(Laser(1,:),Laser(2,:),'.r')


% Estimación del estado con el filtro de Kalman
estimated_states = [];
for i = 1:size(lidar_data, 1)
    % Predicción del estado
    x = A*x;
    P = A*P*A' + Q;
    
    % Actualización del estado basado en las mediciones del LiDAR
    z = lidar_data(i, :)';
    y = z - C*x;
    S = C*P*C' + R;
    K = P*C'*inv(S);
    x = x + K*y;
    P = (eye(5) - K*C)*P;
    
    % Almacenar el estado estimado
    estimated_states = [estimated_states x];
end

% Graficar los resultados
t = dt*(0:size(lidar_data, 1)-1);
figure;
subplot(3, 1, 1);
plot(t, estimated_states(1, :), 'b-', 'LineWidth', 2);
hold on;
% plot(t, lidar_data(:, 1), 'ro', 'MarkerSize', 5);
xlabel('Tiempo');
ylabel('Posición X');
legend('Estimada', 'Medición');
title('Estimación de posición del robot en X');
grid on;

subplot(3, 1, 2);
plot(t, estimated_states(3, :), 'b-', 'LineWidth', 2);
hold on;
% plot(t, lidar_data(:, 2), 'ro', 'MarkerSize', 5);
xlabel('Tiempo');
ylabel('Posición Y');
legend('Estimada', 'Medición');
title('Estimación de posición del robot en Y');
grid on;

subplot(3, 1, 3);
plot(t, estimated_states(5, :), 'b-', 'LineWidth', 2);
xlabel('Tiempo');
ylabel('Ángulo \theta');
title('Estimación del ángulo de orientación del robot');
grid on;








%%
clc
clear
close all


% Parámetros del modelo del robot
dt = 0.1; % Intervalo de tiempo entre mediciones (segundos)
A = [1 dt 0 0 0; 0 1 0 0 0; 0 0 1 dt 0; 0 0 0 1 0; 0 0 0 0 1]; % Matriz de transición de estado
C = [1 0 0 0 0; 0 0 1 0 0]; % Matriz de observación (medición)
Q = diag([0.1 0.1 0.1 0.1 0.1]); % Covarianza del proceso
R = diag([0.5 0.5]); % Covarianza de la medición

% Estado inicial del robot
x = [0; 0; 0; 0; 0]; % [posición X; velocidad X; posición Y; velocidad Y; ángulo de orientación]

% Covarianza inicial del estado
P = eye(5);

% Generación de datos del LiDAR (ejemplo)
lidar_data = cell(1, 3); % Número de instantes de tiempo considerados (3 en este ejemplo)
% Instante de tiempo 1
lidar_data{1} = {[1 2 3; 4 5 6]}; % Ejemplo de datos del LiDAR en coordenadas cartesianas
% Instante de tiempo 2
lidar_data{2} = {[2 3 4; 7 8 9]}; % Ejemplo de datos del LiDAR en coordenadas cartesianas
% Instante de tiempo 3
lidar_data{3} = {[3 4 5; 10 11 12]}; % Ejemplo de datos del LiDAR en coordenadas cartesianas

% Estimación del estado con el filtro de Kalman
estimated_states = zeros(5, numel(lidar_data));
for k = 1:numel(lidar_data) % Iterar a través de los instantes de tiempo
    % Extraer los datos del LiDAR para el instante de tiempo k
    lidar_data_k = lidar_data{k}{1};
    
    % Predicción del estado
    x = A*x;
    P = A*P*A' + Q;
    
    % Actualización del estado basado en las mediciones del LiDAR
    z = lidar_data_k;
    y = z - C*x;
    S = C*P*C' + R;
    
    K = P*C'*inv(S);
    x = x + K*y;
    P = (eye(5) - K*C)*P;
    
    % Almacenar el estado estimado
    estimated_states(:, k) = x;
end

% Graficar los resultados
t = dt*(0:numel(lidar_data)-1);

% Estimación de posición en X
figure;
subplot(3, 1, 1);
plot(t, estimated_states(1, :), 'b-', 'LineWidth', 2);
xlabel('Tiempo');
ylabel('Posición X');
title('Estimación de posición del robot en X');
grid on;

% Estimación de posición en Y
subplot(3, 1, 2);
plot(t, estimated_states(3, :), 'b-', 'LineWidth', 2);
xlabel('Tiempo');
ylabel('Posición Y');
title('Estimación de posición del robot en Y');
grid on;

% Estimación del ángulo de orientación
subplot(3, 1, 3);
plot(t, estimated_states(5, :), 'b-', 'LineWidth', 2);
xlabel('Tiempo');
ylabel('Ángulo \theta');
title('Estimación del ángulo de orientación del robot');
grid on;




