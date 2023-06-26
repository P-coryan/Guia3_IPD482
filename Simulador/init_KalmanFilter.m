
% Caso 1: Conocemos todas las caracteristicas apriori

function [xhat, P] = init_KalmanFilter(Laser, robot, M, flag) %robot_hat
    
    covCaract_init = 0;
    
    if flag == 1
        % Mapa conocido apriori
        M = reshape(M', [], 1);
        % Condicion Inicial Kalman Filter (xhat(0/0)) Vector estado inicial 
        xhat = zeros( 3+length(M) ,2);
        xhat(:,2) = [robot.x ; robot.y ; robot.tita ; M];
        P = diag( [ ones(1,2), 0.1, zeros(1,length(M))] );

    elseif flag == 0
        % Procesamiento de los datos (get_measurements)
        nubePtos = DeteccionPostes(Laser,robot);    % eje global
        [caract, cov_caract, ~, ~] = ClusteringKalman(nubePtos, 0.2, 2); % eje global
        caract = reshape(caract', [], 1);
        
        % Condicion Inicial Kalman Filter (xhat(0/0)) Vector estado inicial 
        xhat = zeros( 3+length(caract) ,2);
        xhat(:,2) = [robot.x ; robot.y ; robot.tita ; caract];
        
        R_init = zeros(2*length(cov_caract));
        for t = 1:length(cov_caract)
            j = 2*(t-1) + 1;
            R_init(  j:j+1 , j:j+1 ) = cov_caract{t};           
        end      
        P = [diag([ ones(1,2), 0.1] ) , zeros(3, length(xhat(4:end,2))) ;  zeros(length(xhat(4:end,2)),3)  , R_init];
    
    end
end
