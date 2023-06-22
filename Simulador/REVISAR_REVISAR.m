%%
clc
clear
close all
% Parámetros del filtro de Kalman
dt = 0.1;  % Intervalo de tiempo entre las mediciones

% Matrices de covarianza de ruido
Q = diag([0.1, 0.1, 0.1, 0.1, 0.1]);  % Covarianza del ruido de proceso
R = 0.2*eye(6);  % Covarianza del ruido de medición

% Estado inicial estimado
x = [0; 0; 0; 0; 0];  % [x; y; vx; vy; theta]
P = diag([1, 1, 1, 1, 1]);  % Matriz de covarianza inicial

% Mediciones de los obstáculos (nube de puntos en coordenadas relativas al robot)
obstacles = [1, 1; -1, 2; 2, -1];

% Función de mapeo: proyectar las coordenadas del robot al sistema de coordenadas globales
map_state_to_observation = @(robot_state) [robot_state(1) + obstacles(:, 1), robot_state(2) + obstacles(:, 2)];

Lidar = [1.1, 1.1; -1.1, 2.1; 2.1, -1.1];

% Datos de velocidad lineal y velocidad angular del robot (puedes modificarlos según tus necesidades)
linear_velocity = 0.5;
angular_velocity = 0.1;

% Iteraciones del filtro de Kalman
num_iterations = 100;

% Iterar en cada paso de tiempo
for k = 1:num_iterations
    % Predicción del estado y de la matriz de covarianza
    x(1) = x(1) + (x(3) * cos(x(5)) - x(4) * sin(x(5))) * dt;
    x(2) = x(2) + (x(3) * sin(x(5)) + x(4) * cos(x(5))) * dt;
    x(3) = x(3);
    x(4) = x(4);
    x(5) = x(5) + angular_velocity * dt;
    
    F = eye(5);  % Matriz de transición de estado linealizada (A)
    
    P = F * P * F' + Q;  % Predicción de la matriz de covarianza del estado
    
    % Actualización del estado y de la matriz de covarianza utilizando las mediciones
    observed_obstacles = map_state_to_observation(x);
    
    z = Lidar;  % Vector de mediciones (Lidar 2xN )
    y = z - map_state_to_observation(x);  % Innovación (diferencia entre las mediciones reales y las estimadas)
    y = reshape(y', [], 1);
    
    H = [eye(2), zeros(2, 3);
        eye(2), zeros(2, 3);
        eye(2), zeros(2, 3)];  % Matriz de observación linealizada

    S = H * P * H.' + R;  % Covarianza de innovación
    K = P * H' / S;  % Ganancia de Kalman
    
    x = x + K * y;  % Actualización del estado estimado
    P = (eye(5) - K * H) * P;  % Actualización de la matriz de covarianza
    
    % Visualización de los resultados
    scatter(obstacles(:, 1), obstacles(:, 2), 'r', 'filled');  % Obstáculos reales (en rojo)
    hold on;
    scatter(observed_obstacles(:, 1), observed_obstacles(:, 2), 'b', 'filled');  % Obstáculos observados (en azul)
    scatter(x(1), x(2), 'g', 'filled');  % Posición estimada del robot (en verde)
    hold off;
    xlim([-5, 5]);
    ylim([-5, 5]);
    title('Filtro de Kalman - Estimación de posición del robot');
    legend('Obstáculos reales', 'Obstáculos observados', 'Posición estimada del robot');
    drawnow;
end


%### REVISAR ESTOS COMENTARIOS ###%

% "R" cambia con la cantidad de postes que vea en cada iteracion, si ve 3
% postes (xi,yi) entonces R=6x6 (dos coordenadas por poste).

% "Lidar" se deben agregar mas puntos captados para las iteraciones
% siguientes

% "H" cambia su tamaño en funcion de los postes que se esten viendo, cada
% que vea(o no vea) un poste se agrega(o elimina) un "eye(2), zeros(2, 3)"

% "x" para que las dimensiones calzen en la actualizacion del kalman, se
% puso un reshape en la innovacion "y"









% function observed_obstacles = map_state_to_observation(robot_state, obstacles)
%     robot_x = robot_state(1);
%     robot_y = robot_state(2);
%     robot_vx = robot_state(3);
%     robot_vy = robot_state(4);
%     robot_theta = robot_state(5);
% 
%     % Aplicar transformaciones al estado del robot para obtener las coordenadas globales
%     observed_obstacles = zeros(size(obstacles));
%     for i = 1:size(obstacles, 1)
%         obstacle_x = obstacles(i, 1);
%         obstacle_y = obstacles(i, 2);
% 
%         % Realizar transformaciones según el estado del robot
%         transformed_x = obstacle_x + robot_x + (robot_vx * cos(robot_theta) - robot_vy * sin(robot_theta));
%         transformed_y = obstacle_y + robot_y + (robot_vx * sin(robot_theta) + robot_vy * cos(robot_theta));
% 
%         observed_obstacles(i, :) = [transformed_x, transformed_y];
%     end
% end

