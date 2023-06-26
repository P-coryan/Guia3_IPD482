                
% Filtro de Kalman robot odometrico (con o sin conocimiento del Mapa)
% (knowMap==1 SI conocemos las caract)(knowMap==0 NO conocemos las caract)
function [xhat, P] = FiltroKalman_Odometric(Laser ,xhat ,P , V, W, tau, k, knowMap)
    
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
    [posteMed_G, covMed, ~, ~] = ClusteringKalman(nubePtos, 0.3, 2);    %coord global
    z_k = zeros(2*length(posteMed_G(:,1)),1);       
    for t = 1:length(posteMed_G(:,1))
        z_k(((1:2)+2*(t-1)),1) = h_k([posteMed_G(t,1), posteMed_G(t,2)]  ,robot_hat); %coord local
    end
    
    %%% Estimacion salida del tiempo k
    zhat_k = zeros(2*n,1);
    postePredict_G = zeros(n,2);
    for t = 1:n
        postePredict_G(t,:) = [xhat((4+2*(t-1)),k), xhat((5+2*(t-1)),k)];  %coord global
    end
    
    % data_association
    H_k = zeros( length(z_k) , length(xhat(:,k)) );
    R_k = zeros(length(z_k));
    zhatFinal_ordenado = zeros(length(zhat_k),1);
    asociacionMed = [posteMed_G , zeros(length(posteMed_G(:,1)),1)];
    
    for t = 1:n   % (estimados xhat == M)     n de 1 a 3
        for i = 1:length(posteMed_G(:,1))   % (medicion)    i de 1 a 3
        
            threshold = sqrt(   (posteMed_G(i,1)-postePredict_G(t,1))^2 +  (posteMed_G(i,2)-postePredict_G(t,2))^2  );
            if threshold < 1  % si no entra, debe analizar otro poste
                
                H_k((2*(i-1)+1):(2*(i-1)+2), 1:3) = dH_xv(posteMed_G(i,1) , posteMed_G(i,2), robot_hat);         % 2x3
                
                H_k((2*(i-1)+1):(2*(i-1)+2), (2*(t-1)+4):(2*(t-1)+5)) = dH_xfn(posteMed_G(i,1) , posteMed_G(i,2), robot_hat);    % 2x2

                R_k((2*(i-1)+1):(2*(i-1)+2), (2*(i-1)+1):(2*(i-1)+2))= covMed{i};
                
                zhatFinal_ordenado((2*(i-1)+1):(2*(i-1)+2)) = h_k([xhat((4+2*(t-1)),k), xhat((5+2*(t-1)),k)], robot_hat);
                
                asociacionMed(i,3) = t;
                
                break
            end
        end
    end
    
    % separa el poste nuevo para agregarlo mas adelante (solo si hay)
    add_feature = [];
    add_CovFeature = [];
    cont_asociacion = 0;
    for i = 1:length(asociacionMed(:,3))
        % Solo postes NUEVOS
        if  asociacionMed(i,3) == 0
            add_feature = [add_feature ; z_k((2*(i-1)+1):(2*(i-1)+2),1)];
            cont_asociacion = cont_asociacion +1;
            add_CovFeature((1:2)+2*(cont_asociacion-1),  (1:2)+2*(cont_asociacion-1)  ) = covMed{i};
            
            z_k((2*(i-1)+1):(2*(i-1)+2),:) = [];
            zhatFinal_ordenado((2*(i-1)+1):(2*(i-1)+2),:) = [];
            H_k((2*(i-1)+1):(2*(i-1)+2),:) = [];
            R_k((2*(i-1)+1):(2*(i-1)+2),:) = [];
            R_k(:,(2*(i-1)+1):(2*(i-1)+2)) = [];
        end
    end
    
    % Elimina de z_hat los estados que no son medidos en la salida
    if length(z_k) ~= length(zhatFinal_ordenado)
        zhatFinal_ordenado = zhatFinal_ordenado(zhatFinal_ordenado~=0);
    end

    %%% Actualizacion Kalman
    S_k = H_k*P*H_k' + R_k;
    K_k = P*H_k'/S_k;
    P = (eye(length(P)) - K_k*H_k)*P;
    v_k = z_k - zhatFinal_ordenado;    %vector innovacion   
    xhat(:,k) = xhat(:,k) + K_k*v_k;
    
    robot_hat.x = xhat(1,k);
    robot_hat.y = xhat(2,k);
    robot_hat.tita = xhat(3,k);
    
    % add_feat (agregar las caracteristicas nuevas al vector de estados y varianza)
    if ~isempty(add_feature)
        add_featGLOBAL = zeros(length(add_feature(:,1)),1);
        for i = length(add_feature(:,1))/2
            add_featGLOBAL((2*(i-1)+1):(2*(i-1)+2),1) = g(add_feature(2*(i-1)+1,1), add_feature(2*(i-1)+2,1), robot_hat);
        end
        
        if knowMap == 0
            P = [P , zeros(length(xhat(:,k)),2*cont_asociacion) ; zeros(2*cont_asociacion, length(xhat(:,k)))   , add_CovFeature];
        elseif knowMap == 1
            P = [P , zeros(length(xhat(:,k)),2*cont_asociacion) ; zeros(2*cont_asociacion, length(xhat(:,k)))   , zeros(2*cont_asociacion)];            
        end

        xhat = [xhat ; zeros(length(add_featGLOBAL(:,1)), (k-1)) ,  add_featGLOBAL ];
        
    end

end

