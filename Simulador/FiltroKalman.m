% Condiciones iniciales Filtro Kalman
% x(:,1) =  [0 ; 0];          %x(1)
x_hat(:,1) =  mu_x1;        %xhat(1/0)
Sigma(:,:,1) = var_x1;      %Sigma(1/0)

for i =1:iteracionesN
    % Entrada
    u(1,i) = mu_u + sqrt(var_u)*randn;
    % Ruidos
    w(:,i) = mu_w + sqrt(var_w)*randn(2,1);
    v(1,i) = mu_v + sqrt(var_v)*randn;
    
    % Planta
    x(:,i+1) = A*x(:,i) + B*u(1,i) + w(:,i);
    y(1,i) = C*x(:,i) + D*u(1,i) + v(1,i);
    
    %##### Filtro de Kalman #####%
    [x_hat, Sigma] = KalmanFilterGeneral(i,A,B,C,D,Q,S,R,y,u,x_hat,Sigma);
end



%%


A = eye(3,3);
B = pasoTiempo*[cos(robot2.tita) 0 0 ; 0 sin(robot2.tita) 0 ; 0 0 1];


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


% Filtro de Kalman General (ruidos correlacionados)
function [x_hat, Sigma] = KalmanFilterGeneral(i,A,B,C,D,Q,S,R,y,u,x_hat,Sigma)
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