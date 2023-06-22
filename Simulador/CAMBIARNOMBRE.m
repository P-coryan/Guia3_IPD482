
% INICIALIZAR robot_hat con la pose inicial conocida entes de moverse


function salida = CAMBIARNOMBRE(robot_hat, V, W, Laser, caractM, step, pasoTiempo)
    %%%%% es posible poner la funcion de deteccion de postes aca %%%%%%%%%
    
    % Medicion Real
    y = Laser;

    % Condicion Inicial Kalman Filter (xhat(0/0))
    xhat = robot_hat;
    
    
    for i = 1:1:step
    
        % Estimacion Postes de interes
        rangoRadio = 10;
        yhat = caractCart2LaserPolar(caractM, xhat);
        yhat(:, yhat(1,:) > rangoRadio ) =  [];     % filtro por radio
        yhat(:, yhat(2,:)<1   )   =  [];    % filtro po angulo
        yhat(:, yhat(2,:)>181 )   =  [];
    
        % Prediccion de Kalman
        xhat.x = xhat.x + pasoTiempo*V*cos(xhat.tita);
        xhat.y = xhat.y + pasoTiempo*V*sin(xhat.tita);
        xhat.tita = xhat.tita + pasoTiempo*W;
        
        % Actualizacion Filtro de Kalman
        K_kf(:,i) = Sigma(:,:,i)*C' / (R + C*Sigma(:,:,i)*C');
        xhat.x = xhat.x + K_kf*(y - yhat);

    end


end



% Filtro de Kalman General (ruidos correlacionados)
function [x_hat, Sigma] = KalmanFilterGeneral(steps, Laser)
    % valor inicial xhat(1/0) se actualiza con el primer dato xhat(1/1) y 
    % despues predice xhat(2/1)
    
    % Actualizacion
    K_kf(:,i) = Sigma(:,:,i)*C' / (R + C*Sigma(:,:,i)*C');
    x_hat(:,i) = x_hat(:,i) + K_kf(:,i)*(y(1,i) - C*x_hat(:,i) - D*u(1,i));
    Sigma(:,:,i) = Sigma(:,:,i) - K_kf(:,i)*C*Sigma(:,:,i);

    % Prediccion
    x_hat(:,i+1) = (A-S*R^(-1)*C) *x_hat(:,i) + (B-S*R^(-1)*D) *u(1,i) + S*R^(-1)*y(1,i);
    Sigma(:,:,i+1) = Q - S*R^(-1)*S' + (A-S*R^(-1)*C) *Sigma(:,:,i) *(A-S*R^(-1)*C)';

end


function z = caractCart2LaserPolar(caractM, robot_hat)
    
    z = zeros(2,length(caractM(:,1)));
    for i = 1:length(caractM(:,1))
        z(1,i) = sqrt( (caractM(i,1) - robot_hat.x).^2 + (caractM(i,2) - robot_hat.y).^2  ) ;
        z(2,i) = rad2deg(   atan2((caractM(i,2)-robot_hat.y),(caractM(i,1)-robot_hat.x))  -robot_hat.tita  +pi/2 );
    end

end









function p = FiltrajePostes(robot_hat, M)
    % Matrices de traslacion y rotacion global
    T = @(tx,ty) [1 0 tx; 0 1 ty; 0 0 1];
    R = @(tita) [cos(tita) -sin(tita); sin(tita) cos(tita)];
    
    CPg = zeros(length(M(:,1),2));
    for i=1:lenth(CPg(:,1))
        newCord = R(t0) * [M(i,1);M(i,2)];
        newCord = T(x0, y0) * [newCord; 1]; 
        CPg(i,:) = newCord(1:2);
    end
    

    
    M = zeros(2,181);
    M(1,:) = 10;
    M(2,:) = 0:1:180;

    
    
end
