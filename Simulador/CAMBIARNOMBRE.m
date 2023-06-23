


dHx = @(xi,yi,r,robot) [  (xi-robot.x)/r , (yi-robot.y)/r , 0 ;
                 (yi-robot.y)/r^2 , -(xi-robot.x)/r^2 , -1  ];

zi = @(caract,robot) [ sqrt((caract(1,1)-robot.x)^2 + (caract(1,2)-robot.y)^2) ;
        rad2deg( atan2((caract(1,2)-robot.y),(caract(1,1)-robot.x)) -robot.tita +pi/2 )];
         
g = @(r,tita,robot) [robot.x+r*cos(tita+robot.tita) ; 
                    robot.y+r*sin(tita+robot.tita)];
             
dGz =  [cos(tita + robot.tita) , -r*sen(tita+robot.tita)  ; 
        sin(tita + robot.tita) ,  r*cos(tita+robot.tita) ];
                
dYxz = @(n,r,tita,robot) [eye(n), zeros(n,2) ; zeros(2,n), dGz];
              
J1 = @(v,theta,tau) [1 0 -v*tau*sin(theta);
                     0 1 v*tau*cos(theta);
                     0 0 1];
J2 = @(theta,tau) [tau*cos(theta) 0;
                     tau*sin(theta) 0 ;
                     0  1];
                 

% SOLO PARA QUE VEA 1 POSTE
function salida = FiltroKalman_Odometrico(Laser, robot ,xhat ,P , V, W, k) %robot_hat
    
    % Prediccion
    xhat(1,k+1) = xhat(1,k) + pasoTiempo*V*cos(xhat(3,k));
    xhat(2,k+1) = xhat(2,k) + pasoTiempo*V*sin(xhat(3,k));
    xhat(3,k+1) = xhat(3,k) + pasoTiempo*W;
    xhat(4,k+1) = xhat(4,k);        % poste estatico
    xhat(5,k+1) = xhat(5,k);
    
    F_k = [J1(V,xhat(3,k),tau) , zeros(3,2) ; zeros(2,3) , eye(2)];
    G_k = [J2(xhat(3,k),tau) , zeros(3,2) ; zeros(2,2) , zeros(2)]; 
    Q = 0;  % Covarianza de la entrada u_t (caso particular)
    P = F_k*P*F_k' + G_k*Q*G_k';
    
    
    
    % Definicion de mediciones y Covarianza de las mediciones
%     z = zi(caract, robot);  %r,theta
%     R = cov_caract;         %cov_r,cov_theta (OJO)

    
 

end

% SOLO PARA QUE VEA 1 POSTE
function [xhat, P] = init_KalmanFilter(Laser, robot, M, k) %robot_hat
% Procesamiento de los datos (get_measurements)
nubePtos = DeteccionPostes(Laser,robot);    % eje global
[caract, cov_caract, ~, ~, ~] = ClusteringNube(nubePtos, M, 0.2, 3); % eje global
    % considerar agregar un reshape a caract para mas analizar postes

% Condicion Inicial Kalman Filter (xhat(0/0))    
% Vector estado inicial
xhat = zeros( 3+length(caract) ,k);
xhat(:,k) = [robot.x ; robot.y ; robot.tita ; caract(1) ; caract(2)];
P = diag( [ ones(1,2), 0.1, ones(1,length(caract))] );

end




% function z = caractCart2LaserPolar(caractM, robot_hat)
%     
%     z = zeros(2,length(caractM(:,1)));
%     for i = 1:length(caractM(:,1))
%         z(1,i) = sqrt( (caractM(i,1) - robot_hat.x).^2 + (caractM(i,2) - robot_hat.y).^2  ) ;
%         z(2,i) = rad2deg(   atan2((caractM(i,2)-robot_hat.y),(caractM(i,1)-robot_hat.x))  -robot_hat.tita  +pi/2 );
%     end
% 
% end

% 
% function p = FiltrajePostes(robot_hat, M)
%     % Matrices de traslacion y rotacion global
%     T = @(tx,ty) [1 0 tx; 0 1 ty; 0 0 1];
%     R = @(tita) [cos(tita) -sin(tita); sin(tita) cos(tita)];
%     
%     CPg = zeros(length(M(:,1),2));
%     for i=1:lenth(CPg(:,1))
%         newCord = R(t0) * [M(i,1);M(i,2)];
%         newCord = T(x0, y0) * [newCord; 1]; 
%         CPg(i,:) = newCord(1:2);
%     end
%     
% 
%     
%     M = zeros(2,181);
%     M(1,:) = 10;
%     M(2,:) = 0:1:180;
% 
%     
%     
% end
