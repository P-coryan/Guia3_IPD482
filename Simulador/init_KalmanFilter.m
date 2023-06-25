
% Caso 1: Conocemos todas las caracteristicas apriori

function [xhat, P] = init_KalmanFilter(Laser, robot, M) %robot_hat

    % #### EVALUAR SACAR ESTO DE LA FUNCION ### %
    % Procesamiento de los datos (get_measurements)
    nubePtos = DeteccionPostes(Laser,robot);    % eje global
    [caract, cov_caract, ~, ~, ~] = ClusteringNube(nubePtos, M, 0.2, 3); % eje global
    % ########################################## %
        
    M = reshape(M', [], 1);
    
    % Condicion Inicial Kalman Filter (xhat(0/0)) Vector estado inicial 
    xhat = zeros( 3+length(M) ,2);
    xhat(:,2) = [robot.x ; robot.y ; robot.tita ; M];
    P = diag( [ ones(1,2), 0.1, 0.0001*ones(1,length(M))] );

end
