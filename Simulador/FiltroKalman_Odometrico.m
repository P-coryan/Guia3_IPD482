                

% SOLO PARA QUE VEA 1 POSTE
function [xhat, P, threshold] = FiltroKalman_Odometrico(Laser ,xhat ,P , V, W, tau, k, M, threshold, robot) %robot_hat
    
    [dHx, h_k, g, J1, J2] = matrixData();
    %%% Prediccion
    xhat(1,k) = xhat(1,k-1) + tau*V*cos(xhat(3,k-1));
    xhat(2,k) = xhat(2,k-1) + tau*V*sin(xhat(3,k-1));
    xhat(3,k) = xhat(3,k-1) + tau*W;
    xhat(4,k) = xhat(4,k-1);        % poste estatico
    xhat(5,k) = xhat(5,k-1);
    
    robot_hat.x = xhat(1,k);
    robot_hat.y = xhat(2,k);
    robot_hat.tita = xhat(3,k);
    
    Qu = zeros(2);  % Covarianza de la entrada u_t (caso particular)
    Q = [Qu , zeros(2,2) ; zeros(2,2) , zeros(2)];
    F_k = [J1(V,xhat(3,k),tau) , zeros(3,2) ; zeros(2,3) , eye(2)];
    G_k = [J2(xhat(3,k),tau) , zeros(3,2) ; zeros(2,2) , zeros(2)]; 
    
    P = F_k*P*F_k' + G_k*Q*G_k';
    
    %%% Medicion del tiempo k
    nubePtos = DeteccionPostes(Laser,robot_hat);    %coord global
    [z_k, R_k, ~, ~] = ClusteringKalman(nubePtos, 0.2, 3); 
    posteMed_G = z_k;                               %coord global
    z_k = h_k(z_k,robot_hat);                       %coord local
    %%% Estimacion Medicion del tiempo k
    zhat_k = h_k([xhat(4,k), xhat(5,k)],robot_hat); %coord local
    postePredict_G = [xhat(4,k), xhat(5,k)];        %coord global
    
    threshold(k) = sqrt(   (posteMed_G(1,1)-postePredict_G(1,1))^2 +  (posteMed_G(1,2)-postePredict_G(1,2))^2  );
    % data_association
%     if threshold < 10         % si no entra, debe analizar otro poste
        H_k = [  zeros(2)   , dHx(xhat(4,k),xhat(5,k),zhat_k(1,1),robot_hat)]; 
%     else
%         H_k = [  zeros(2)   , zeros(2,3)]; 
%     end
    
    %%% Actualizacion
    S_k = H_k*P*H_k' + R_k{1};
    K_k = P*H_k'/S_k;
    P = (eye(length(P)) - K_k*H_k)*P;
    v_k = z_k - zhat_k;                 %innovacion (2xn)
    xhat(:,k) = xhat(:,k) + K_k*v_k;
    
    % add_feat (agregar las caracteristicas nuevas al vector de estados)
    % la etapa de actualizacion se realiza con las caracteristicas NO
    % nuevas
 

end

% % SOLO PARA QUE VEA 1 POSTE
% function [xhat, P] = init_KalmanFilter(Laser, robot, M, k) %robot_hat
% % Procesamiento de los datos (get_measurements)
% nubePtos = DeteccionPostes(Laser,robot);    % eje global
% [caract, cov_caract, ~, ~, ~] = ClusteringNube(nubePtos, M, 0.2, 3); % eje global
%     % considerar agregar un reshape a caract para mas analizar postes
% 
% % Condicion Inicial Kalman Filter (xhat(0/0))    
% % Vector estado inicial
% xhat = zeros( 3+length(caract) ,k);
% xhat(:,k) = [robot.x ; robot.y ; robot.tita ; caract(1) ; caract(2)];
% P = diag( [ ones(1,2), 0.1, ones(1,length(caract))] );
% 
% end





