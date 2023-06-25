                

% SOLO PARA QUE VEA 1 POSTE
function [xhat, P] = FiltroKalman_Odometrico(Laser ,xhat ,P , V, W, tau, k) %robot_hat
    
    [h_k, dH_xv, dH_xfn, g, dGz, dYxz, J1, J2] = matrixFunctions();
    
    n = length(xhat(4:end,k-1))/2;        % dimension de postes estimados (cantidad)    
    %%% Prediccion Kalman
    xhat(1,k) = xhat(1,k-1) + tau*V*cos(xhat(3,k-1));
    xhat(2,k) = xhat(2,k-1) + tau*V*sin(xhat(3,k-1));
    xhat(3,k) = xhat(3,k-1) + tau*W;
    xhat(4:end,k) = xhat(4:end,k-1);        % poste estatico
    
    robot_hat.x = xhat(1,k);
    robot_hat.y = xhat(2,k);
    robot_hat.tita = xhat(3,k);
    
    Qu = zeros(2);  % Covarianza de la entrada u_t (caso particular u_t gaussiano)
    Q = [Qu , zeros(2,2*n) ; zeros(2*n,2) , zeros(2*n)];
    
    F_k = [J1(V,xhat(3,k),tau) , zeros(3,2*n) ; zeros(2*n,3) , eye(2*n)];
    G_k = [J2(xhat(3,k),tau) , zeros(3,2*n) ; zeros(2*n,2) , zeros(2*n)]; 
    
    P = F_k*P*F_k' + G_k*Q*G_k';
    
    %%% Medicion del tiempo k
    nubePtos = DeteccionPostes(Laser,robot_hat);                        %coord global
    [posteMed_G, covMed, ~, ~] = ClusteringKalman(nubePtos, 0.2, 3);    %coord global                           
    z_k = zeros(2*length(posteMed_G(:,1)),1);       
    for t = 1:length(posteMed_G(:,1))
        z_k(((1:2)+2*(t-1)),1) = h_k([posteMed_G(t,1), posteMed_G(t,2)]  ,robot_hat); %coord local
    end
    
    %%% Estimacion salida del tiempo k
    zhat_k = zeros(length(z_k),1);
    postePredict_G = zeros(n,2);
    for t = 1:n
        postePredict_G(t,:) = [xhat((4+2*(t-1)),k), xhat((5+2*(t-1)),k)];                       %coord global
    end
    
    % data_association
    H_k = zeros( length(z_k) , length(xhat(:,k)) );
    R_k = zeros(length(z_k));
    zhatFinal_ordenado = zeros(length(zhat_k),1);
    for t = 1:n   % (estimados xhat == M)     n de 1 a 3
        for i = 1:length(posteMed_G(:,1))   % (medicion)    i de 1 a 3
        
            threshold = sqrt(   (posteMed_G(i,1)-postePredict_G(t,1))^2 +  (posteMed_G(i,2)-postePredict_G(t,2))^2  );
            if threshold < 0.6  % si no entra, debe analizar otro poste
                
                H_k((2*(i-1)+1):(2*(i-1)+2), 1:3) = dH_xv(posteMed_G(i,1) , posteMed_G(i,2), robot_hat);         % 2x3
                
                H_k((2*(i-1)+1):(2*(i-1)+2), (2*(t-1)+4):(2*(t-1)+5)) = dH_xfn(posteMed_G(i,1) , posteMed_G(i,2), robot_hat);    % 2x2

                R_k((2*(i-1)+1):(2*(i-1)+2), (2*(i-1)+1):(2*(i-1)+2))= covMed{i};
                
                zhatFinal_ordenado((2*(i-1)+1):(2*(i-1)+2)) = h_k([xhat((4+2*(t-1)),k), xhat((5+2*(t-1)),k)], robot_hat);
                
                break
            end
        end
    end
       
    %%% Actualizacion Kalman
    S_k = H_k*P*H_k' + R_k;
    K_k = P*H_k'/S_k;
    P = (eye(length(P)) - K_k*H_k)*P;
    v_k = z_k - zhatFinal_ordenado;                 %innovacion (2xn)  
    xhat(:,k) = xhat(:,k) + K_k*v_k;
    
    % add_feat (agregar las caracteristicas nuevas al vector de estados)
    % la etapa de actualizacion se realiza con las caracteristicas NO
    % nuevas
 

end

